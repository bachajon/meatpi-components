# Host test of the vendored isotp-c link (isotp-c/isotp.c) with the local
# changes PROVENANCE.md lists: no TWAI, no RTOS. esp_isotp.c itself REQUIRES
# esp_driver_twai and cannot build for linux, so the library is compiled
# here the way the component compiles it: src/isotp_config.h forced in
# front of every file, with the Kconfig values it reads supplied by
# isotp_test_kconfig.h (the firmware's own, see that file).
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "${comp_dir}/isotp-c/isotp.c"
    INCLUDE_DIRS "${comp_dir}/isotp-c" "."
    REQUIRES unity
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_LIST_DIR}/isotp_test_kconfig.h"
    "SHELL:-include ${comp_dir}/src/isotp_config.h"
)
