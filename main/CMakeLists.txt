idf_component_register(
    REQUIRES esp_timer freertos nvs_flash
    SRCS "main.cpp" "IMU.cpp" "merge.cpp" # "lsm6dsv16x_reg.c" 
    PRIV_REQUIRES LSM6DSV16X
    INCLUDE_DIRS ""
)

# target_compile_options(${COMPONENT_TARGET} PUBLIC
#     -DARDUINO_BOARD="ESP32S2_DEV"                  #         <<<<<<=== Board Name (Any one, here is set as ESP32 S2 Dev Kit)
#     -DARDUINO_VARIANT="esp32s2"                    #         <<<<<<=== Variant "folder" must match "/variants/folder" name
#     -DBOARD_HAS_PSRAM                              #         <<<<<=== Does the ESP32-S2 have PSRAM? Remove it if not.
#     -DARDUINO_USB_MODE=0                           #         <<<<<=== Activate USB OTG Mode
#     -DARDUINO_USB_CDC_ON_BOOT=1                    #         <<<<<=== Enable Serial as USB CDC Interface
# )
