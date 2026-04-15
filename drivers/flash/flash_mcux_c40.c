/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NXP flash controller driver for C40 flash part
 */

#define DT_DRV_COMPAT nxp_c40_flash_controller

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/cache.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/spinlock.h>

LOG_MODULE_REGISTER(flash_mcux_c40, CONFIG_FLASH_LOG_LEVEL);

#include <fsl_c40_flash.h>

static inline int mcux_to_errno(status_t s)
{
	switch (s) {
	case kStatus_FLASH_Success:
		return 0;
	case kStatus_FLASH_InvalidArgument:
	case kStatus_FLASH_SizeError:
	case kStatus_FLASH_AlignmentError:
	case kStatus_FLASH_AddressError:
		return -EINVAL;
	case kStatus_FLASH_AccessError:
	case kStatus_FLASH_ProtectionViolation:
	case kStatus_FLASH_CommandFailure:
		return -EIO;
	case kStatus_FLASH_EraseKeyError:
		return -EPERM;
	default:
		return -EIO;
	}
}

struct prot_range {
	uint32_t off;
	uint32_t len;
	const char *name;
};

struct mcux_c40_cfg {
	uint32_t code_base;    /* flash memory-mapping address for code bank */
	uint32_t code_size;    /* total bytes in code bank */

	uint32_t erase_block; /* 8 KiB on C40 */
	uint32_t write_block; /* 8 bytes min. program unit */

	uint32_t data_base;   /* flash memory-mapping address for data bank */
	uint32_t data_size;   /* total bytes in code bank */
#if 0
	uint32_t utest_base;
	uint32_t utest_size;
#endif

	const struct flash_parameters *params;
#if defined(CONFIG_SOC_FLASH_MCUX_C40_APPLY_PROTECTION)
	const struct prot_range *prot_tbl;
	size_t prot_cnt;
#endif
};

static inline bool flash_mcux_c40_calculate_base_and_offset( const struct mcux_c40_cfg *cfg, uint32_t *base, uint32_t *io_offset, uint32_t len )
{
	uint32_t offset = (*io_offset);
	if ( ( offset >= 0 ) && ( offset < cfg->code_size ) )
	{
		if ( offset + len > cfg->code_size )
		{
			return false;
		}
		*base = cfg->code_base;
		// code bank is at 0, so no need to account for offset
		return true;
	}

	if ( ( offset >= cfg->code_size ) && ( ( offset - cfg->code_size ) < cfg->data_size ) )
	{
		*base = cfg->data_base;
		// must now adjust offset
		offset -= cfg->code_size;

		if ( offset + len > cfg->data_size )
		{
			return false;
		}

		*io_offset = offset;
		return true;
	}

	return false;
}

struct mcux_c40_data {
	struct k_spinlock lock;
	flash_config_t cfg; /* MCUX HAL context */
};

static inline bool intersects(uint32_t a_off, uint32_t a_len, uint32_t b_off, uint32_t b_len)
{
	const uint32_t a_end = a_off + a_len;
	const uint32_t b_end = b_off + b_len;

	return (a_off < b_end) && (b_off < a_end);
}

static int flash_mcux_c40_read(const struct device *dev, off_t offt, void *buf, size_t len)
{
	const struct mcux_c40_cfg *cfg = dev->config;

	uint32_t base;
	uint32_t off = offt;
	if (!flash_mcux_c40_calculate_base_and_offset( cfg, &base, &off, len ) ) {
		return -EINVAL;
	}

	memcpy(buf, (const void *)(base + (uintptr_t)off), len);

	return 0;
}

static int flash_mcux_c40_write(const struct device *dev, off_t offt, const void *buf, size_t len)
{
	const struct mcux_c40_cfg *cfg = dev->config;
	struct mcux_c40_data *data = dev->data;
	k_spinlock_key_t key;
	status_t st;

	/* Bounds check */
	uint32_t base;
	uint32_t off = offt;
	if (!flash_mcux_c40_calculate_base_and_offset( cfg, &base, &off, len ) ) {
		return -EINVAL;
	}

	/* The HAL enforces alignment to C40 constraints:
	 * Minimum write chunk is C40_WRITE_SIZE_MIN (8 bytes).
	 * Writes are most efficient in quad-page (128 bytes).
	 * We perform a light check: write must be multiple of write_block,
	 * and start aligned to write_block. For more flexibility, you could
	 * implement a read-modify-write cache; for now we map 1:1 to HAL.
	 */
	if (((uintptr_t)off % cfg->write_block) != 0 || (len % cfg->write_block) != 0) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	barrier_dsync_fence_full();

	z_barrier_isync_fence_full();

	st = FLASH_Program(&data->cfg, (uint32_t)(base + (uintptr_t)off), (uint32_t *)buf,
			   (uint32_t)len);

	barrier_dsync_fence_full();

	k_spin_unlock(&data->lock, key);

	/* The array changed behind the CPU; drop stale D-cache lines covering the range */
	sys_cache_data_invd_range((void *)(base + (uintptr_t)off), len);

	return mcux_to_errno(st);
}

static int flash_mcux_c40_erase(const struct device *dev, off_t offt, size_t len)
{
	const struct mcux_c40_cfg *cfg = dev->config;
	struct mcux_c40_data *data = dev->data;
	k_spinlock_key_t key;
	status_t st;

	uint32_t base;
	uint32_t off = offt;
	if (!flash_mcux_c40_calculate_base_and_offset( cfg, &base, &off, len ) ) {
		return -EINVAL;
	}

	if (((uintptr_t)off % cfg->erase_block) != 0 || (len % cfg->erase_block) != 0) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	barrier_dsync_fence_full();

	z_barrier_isync_fence_full();

	st = FLASH_Erase(&data->cfg, (uint32_t)(base + (uintptr_t)off), (uint32_t)len,
			 kFLASH_ApiEraseKey);

	barrier_dsync_fence_full();

	k_spin_unlock(&data->lock, key);

	sys_cache_data_invd_range((void *)(base + (uintptr_t)off), len);

	return mcux_to_errno(st);
}

static const struct flash_parameters *flash_mcux_c40_get_parameters(const struct device *dev)
{
	const struct mcux_c40_cfg *cfg = dev->config;

	return cfg->params;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT

struct flash_pages_layout c40_layout[] = {
	// pflash
	{ .pages_count = DT_REG_SIZE( DT_NODELABEL( pflash ) ) / DT_PROP( DT_NODELABEL( flash0 ), erase_block_size ), .pages_size = DT_PROP( DT_NODELABEL( flash0 ), erase_block_size )},
	// dflash
	{ .pages_count = DT_REG_SIZE( DT_NODELABEL( dflash ) ) / DT_PROP( DT_NODELABEL( flash0 ), erase_block_size ), .pages_size = DT_PROP( DT_NODELABEL( flash0 ), erase_block_size )},
};

static void flash_mcux_c40_pages_layout(const struct device *dev,
				  const struct flash_pages_layout **layout, size_t *layout_size)
{
	*layout = c40_layout;
	*layout_size = 2;
}
#endif

/* Optional “lock policy” executed at init (opt-in via Kconfig) */
#if defined(CONFIG_SOC_FLASH_MCUX_C40_APPLY_PROTECTION)
static __attribute__((noinline)) status_t flash_c40_apply_protection(struct mcux_c40_data *data,
							       uint32_t flash_base,
							       uint32_t total_sz, uint32_t erase_sz,
							       const struct prot_range *pr,
							       size_t npr)
{
	uint32_t off;
	uint32_t abs;
	k_spinlock_key_t key;
	size_t i;
	bool lock_it;
	flash_config_t	*fcfg = &data->cfg;

	for (off = 0; off < total_sz; off += erase_sz) {
		status_t ps;

		lock_it = false;

		for (i = 0; i < npr; i++) {
			if (intersects(off, erase_sz, pr[i].off, pr[i].len)) {
				lock_it = true;
				break;
			}
		}
		abs = flash_base + off;

		ps = FLASH_GetSectorProtection(fcfg, abs);
		if (lock_it) {
			status_t s;

			if (ps != kStatus_FLASH_SectorLocked) {
				key = k_spin_lock(&data->lock);

				/* Didn't use ISB here since there is no
				 * instruction side stream change
				 */
				barrier_dsync_fence_full();

				s = FLASH_SetSectorProtection(fcfg, abs, true);

				barrier_dsync_fence_full();

				k_spin_unlock(&data->lock, key);

				if (s != kStatus_FLASH_Success) {
					return s;
				}
			}
		} else {
			status_t s;

			if (ps != kStatus_FLASH_SectorUnlocked) {
				key = k_spin_lock(&data->lock);

				barrier_dsync_fence_full();

				s = FLASH_SetSectorProtection(fcfg, abs, false);

				barrier_dsync_fence_full();

				k_spin_unlock(&data->lock, key);
				if (s != kStatus_FLASH_Success) {
					return s;
				}
			}
		}
	}
	return kStatus_FLASH_Success;
}
#endif /* CONFIG_SOC_FLASH_MCUX_C40_APPLY_PROTECTION */

static int flash_mcux_c40_init(const struct device *dev)
{
	const struct mcux_c40_cfg *cfg = dev->config;
	struct mcux_c40_data *data = dev->data;
	status_t st;

	st = FLASH_Init(&data->cfg);
	if (st != kStatus_FLASH_Success) {
		LOG_ERR("FLASH_Init failed: %d", (int)st);
		return mcux_to_errno(st);
	}

	LOG_DBG("C40 flash: erase=0x%lx write=0x%lx",
		(unsigned long)cfg->erase_block,
		(unsigned long)cfg->write_block);

#if defined(CONFIG_SOC_FLASH_MCUX_C40_APPLY_PROTECTION)
	/* Align protected windows to sector boundaries */
	struct prot_range prot_aligned[8];
	size_t nprot_al = 0;
	size_t i;
	uint32_t s;
	uint32_t e;
	uint32_t sa;
	uint32_t ea;

	for (i = 0; i < cfg->prot_cnt && nprot_al < ARRAY_SIZE(prot_aligned); i++) {
		s = cfg->prot_tbl[i].off;
		e = s + cfg->prot_tbl[i].len;
		sa = ROUND_DOWN(s, cfg->erase_block);
		ea = ROUND_UP(e, cfg->erase_block);
		if (sa >= cfg->size) {
			continue;
		}
		if (ea > cfg->size) {
			ea = cfg->size;
		}

		prot_aligned[nprot_al] = (struct prot_range){
			.off = sa,
			.len = ea - sa,
			.name = cfg->prot_tbl[i].name,
		};
		nprot_al++;
	}

#if 0
	st = flash_c40_apply_protection(data, cfg->base, cfg->size, cfg->erase_block, prot_aligned,
				  nprot_al);
#else
	// get from DTS property!
	st = flash_c40_apply_protection(data, 0x400000, cfg->size, cfg->erase_block, prot_aligned,
				  nprot_al);
#endif
	if (st != kStatus_FLASH_Success) {
		LOG_ERR("Protection apply failed: %d", (int)st);
		return mcux_to_errno(st);
	}
	LOG_DBG("Protection policy applied (%u window%s)", (unsigned int)nprot_al,
		(nprot_al == 1 ? "" : "s"));
#endif
	return 0;
}

static DEVICE_API(flash, mcux_c40_api) = {
	.read = flash_mcux_c40_read,
	.write = flash_mcux_c40_write,
	.erase = flash_mcux_c40_erase,
	.get_parameters = flash_mcux_c40_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_mcux_c40_pages_layout,
#endif
};

/* Controller node for this instance */
#define C40_CTRL_NODE(inst)   DT_DRV_INST(inst)

#define C40_FLASH_NODE(inst)  DT_INST_CHILD(inst,flash_0)

#define C40_PFLASH_NODE(inst) DT_NODELABEL( pflash )

#define C40_DFLASH_NODE(inst) DT_NODELABEL( dflash )


// NOTE: all this partition table protection is too knowledgeable about the rest
//	of the system, including partition names, etc -- should be done
//	OUTSIDE driver (though driver might need to expose API)

/* Get the partition table properties of this instance's flash */
#define C40_PROT_ENTRY(lbl, inst)								\
	COND_CODE_1(										\
		DT_NODE_HAS_STATUS(DT_NODELABEL(lbl), okay),					\
		(COND_CODE_1(									\
			DT_SAME_NODE(								\
				DT_GPARENT(DT_NODELABEL(lbl)),					\
				C40_FLASH_NODE(inst)),						\
			({ .off  = (uint32_t)PARTITION_OFFSET(lbl),			\
			    .len  = (uint32_t)PARTITION_SIZE(lbl),			\
			    .name = #lbl },),							\
			())									\
		),										\
		())

#if defined(CONFIG_SOC_FLASH_MCUX_C40_APPLY_PROTECTION)
#define C40_MAKE_PROT_TBL(inst)									\
	static const struct prot_range mcux_c40_prot_##inst[] = {				\
		/* keep IVT and bootloader areas read-only on XIP systems */			\
		C40_PROT_ENTRY(ivt_header,   inst)						\
		C40_PROT_ENTRY(ivt_pad,      inst) /* if present */				\
		C40_PROT_ENTRY(mcuboot,      inst) /* or boot_partition */			\
		C40_PROT_ENTRY(boot_partition, inst)						\
	};											\
	enum { mcux_c40_prot_cnt_##inst = ARRAY_SIZE(mcux_c40_prot_##inst) }
#else
#define C40_MAKE_PROT_TBL(inst)
#endif

#define C40_PARAMS(inst)									\
	static const struct flash_parameters mcux_c40_params_##inst = {				\
		.write_block_size = DT_PROP(C40_FLASH_NODE(inst), write_block_size),		\
		.erase_value = 0xFF,								\
	}

#define C40_INIT(inst)										\
	C40_MAKE_PROT_TBL(inst);								\
	C40_PARAMS(inst);									\
	static const struct mcux_c40_cfg mcux_c40_cfg_##inst = {				\
		.code_base     = DT_REG_ADDR(C40_PFLASH_NODE(inst)),				\
		.code_size     = DT_REG_SIZE(C40_PFLASH_NODE(inst)),				\
		.data_base     = DT_REG_ADDR(C40_DFLASH_NODE(inst)),				\
		.data_size     = DT_REG_SIZE(C40_DFLASH_NODE(inst)),				\
		.erase_block = DT_PROP(C40_FLASH_NODE(inst), erase_block_size),			\
		.write_block = DT_PROP(C40_FLASH_NODE(inst), write_block_size),			\
		.params      = &mcux_c40_params_##inst,						\
		IF_ENABLED(CONFIG_SOC_FLASH_MCUX_C40_APPLY_PROTECTION, (			\
			.prot_tbl = mcux_c40_prot_##inst,					\
			.prot_cnt = mcux_c40_prot_cnt_##inst,					\
		))										\
	};											\
	static struct mcux_c40_data mcux_c40_data_##inst;					\
	DEVICE_DT_INST_DEFINE(inst, flash_mcux_c40_init, NULL, &mcux_c40_data_##inst,		\
		&mcux_c40_cfg_##inst, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
		&mcux_c40_api);

DT_INST_FOREACH_STATUS_OKAY(C40_INIT)
