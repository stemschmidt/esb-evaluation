# sysbuild.cmake

# cpunet: Network Core (ESB)
ExternalZephyrProject_Add(
    APPLICATION cpunet
    SOURCE_DIR  ${APP_DIR}/cpunet
    BOARD       nrf5340dk/nrf5340/cpunet
)

# cpunet wird von cpuapp gestartet (Standard beim nRF5340)
set_property(GLOBAL APPEND PROPERTY PM_DOMAINS CPUNET)
set_property(GLOBAL APPEND PROPERTY PM_CPUNET_IMAGES cpunet)
set_property(GLOBAL PROPERTY DOMAIN_APP_CPUNET cpunet)
set(CPUNET_PM_DOMAIN_DYNAMIC_PARTITION cpunet CACHE INTERNAL "")
