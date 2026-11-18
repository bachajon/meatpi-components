idf_component_register(
    SRCS "translator_gvret.c"
         "translator_gvret_codec.c"
    INCLUDE_DIRS "include"
    # esp_timer: the clock a TIME_SYNC answer carries (translator_gvret.c).
    PRIV_REQUIRES bridge_manager can_manager esp_timer
)
