zephyr_library()

zephyr_library_sources(bq34z100.c)

zephyr_include_directories_ifdef(CONFIG_EMUL_BQ34Z100 .)
zephyr_library_sources_ifdef(CONFIG_EMUL_BQ34Z100 ./emul_bq34z100.c)
