idf_component_register(
    SRCS "socket_manager.c"
         "socket_manager_settings.c"
         "socket_manager_net.c"
         "socket_manager_tx.c"
         "socket_manager_pace.c"
         "socket_manager_policy.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    # Public header exposes only esp_err/FreeRTOS queue types -> all private.
    # cJSON comes from the managed espressif/cjson dep (idf_component.yml).
    # tx_backlog: a client's send backlog (the ring, the pool, the push).
    PRIV_REQUIRES lwip settings_manager log_manager esp_timer tx_backlog
)
