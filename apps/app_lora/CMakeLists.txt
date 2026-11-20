idf_component_register(
    SRCS "app_lora.c"
    INCLUDE_DIRS "."
    REQUIRES deck_core deck_ui deck_hal deck_lora
)
