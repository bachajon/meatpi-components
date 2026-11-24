# Host test compiles ONLY the pure helpers (auth/url, and why a request
# failed): no esp_http_client, no esp-tls.
set(comp_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS "test_main.c"
         "${comp_dir}/http_client_manager_auth.c"
         "${comp_dir}/http_client_manager_fail.c"
    INCLUDE_DIRS "${comp_dir}/include" "${comp_dir}"
    REQUIRES unity
)
