# Host test: the pure J2534 wire codec + filter match (no I/O), and since
# 2026-10-27 the settings migration (j2534_server_migrate.c, cJSON only:
# v1 -> v2 drops allow_reflash; cJSON via idf_component.yml).
# 2026-11-17: the ISO15765 channel's SET_CONFIG / GET_CONFIG lists
# (j2534_iso_cfg.c, pure).
# 2026-11-18: what a session's wire version decides (j2534_proto_session.c,
# pure): the status numbers, the EVENT frame, the loss count.
# 2026-11-20: ISO15765 extended addressing, the layout (j2534_iso_ae.c, pure).
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c" "test_voltage.c" "test_iso_cfg.c" "test_session.c"
         "test_iso_ae.c"
         "${comp_dir}/j2534_proto.c"
         "${comp_dir}/j2534_proto_session.c"
         "${comp_dir}/j2534_iso_cfg.c"
         "${comp_dir}/j2534_iso_ae.c"
         "${comp_dir}/j2534_server_migrate.c"
         "${comp_dir}/j2534_server_voltage.c"
    INCLUDE_DIRS "${comp_dir}/include" "${comp_dir}"
                 "${comp_dir}/../battery_monitor/include"
    REQUIRES unity
)
