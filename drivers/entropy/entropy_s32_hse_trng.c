#include <zephyr/drivers/entropy.h>
#include <zephyr/device.h>

#if NOT_YET
	#include "Hse_Ip.h"   /* RTD */
#endif

#define DT_DRV_COMPAT nxp_s32_hse_trng

#warning "FIX ME: implement HSE TRNG driver"


struct s32_hse_trng_dev_data
{
	/* nothing needed yet */
};

struct s32_hse_trng_dev_cfg
{
	/* nothing needed yet */
};

static struct s32_hse_trng_dev_data s32_hse_trng_data = {};
static struct s32_hse_trng_dev_cfg s32_hse_trng_config = {};

static int s32_hse_trng_get_entropy( const struct device *dev, uint8_t *buffer, uint16_t length )
{
#if 0
	hseSrvResponse_t response;
	hseTrngSrv_t trng_srv;

	trng_srv.pRandomNum = buffer;
	trng_srv.randomNumLength = length;

	response = Hse_Ip_TrngRequest(&trng_srv);

	if (response != HSE_SRV_RSP_OK)
	{
	return -EIO;
	}

	return 0;
#else
	memset( buffer, 0x01, length );
	return 0;
#endif
}

#if 0
static const struct entropy_driver_api s32_hse_trng_api =
{
	.get_entropy = s32_hse_trng_get_entropy,
};
#endif

static int s32_hse_trng_init(const struct device *dev)
{
	/* If HSE already initialized elsewhere → nothing */

	return 0;
}

static DEVICE_API(entropy, s32_hse_trng_api) = {
	.get_entropy = s32_hse_trng_get_entropy,
	//.get_entropy_isr = entropy_stm32_rng_get_entropy_isr
};

DEVICE_DT_INST_DEFINE(0,
			s32_hse_trng_init,
			NULL,
			&s32_hse_trng_data,
			&s32_hse_trng_config,
			POST_KERNEL,
			CONFIG_ENTROPY_INIT_PRIORITY,
			&s32_hse_trng_api);
