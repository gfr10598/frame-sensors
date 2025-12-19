#include "tft.h"

void init_SPI()
{
    printf("Initializing SD card...\n");
    SPI.begin(SCK, MISO, MOSI, TFT_CS);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(16 * 1000 * 1000);
}

void setup_tft()
{
    init_SPI();
    // Initialize TFT display
    Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, 40, 41); // CS, DC, RST
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH); // GPIO45 for S2/S3 backlight

    // turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(100);

    // Hello world on the TFT
    tft.init(135, 240); // Init ST7789 240x135
    tft.setRotation(3);
    auto start = esp_timer_get_time();
    tft.fillScreen(ST77XX_BLACK);
    printf("TFT black screen in %lld usec\n", esp_timer_get_time() - start);
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextWrap(true);
    tft.setTextSize(2);
    start = esp_timer_get_time();
    tft.print(
        "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur "
        "adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, "
        "fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor "
        "neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet "
        "ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a "
        "tortor imperdiet posuere. ");
    printf("TFT text in %lld usec\n", esp_timer_get_time() - start);
}