set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "test_backlog.c"
         "${comp_dir}/tx_backlog.c"
    INCLUDE_DIRS "${comp_dir}/include"
    REQUIRES unity
)
