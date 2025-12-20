/*
Just collecting the data is likely to require about 30% of one CPU,
because the CPU is busy when reading the data over I2C.
This will likely mean power consumption of at least 20mA just to read
the data.  We can hopefully use the other processor to concurrently
write the data to the flash.

We can download 32 records at a time, which is 16 samples, which is
about 8 msec.  So, we might want to download on a timer, say about
every 5 msec, to keep the FIFO from overflowing.

This could allow us to wake up every 5 msec, read the data, post it
to the other processor through a queue, and go back
into light sleep.
Periodically, the other processor will write the data to flash.
Apparently the SD card write speed is about 200kB/sec, so a 4kB
write will take about 20 msec.  But the incoming data rate, before
expanding to base64, is about 2*2*7byte/ms = 28kB/sec.  So the
processor will only require about 20% duty cycle to keep up.

*/

#include "Arduino.h"
#include <stdio.h>
#include "LSM6DSV16XSensor.h"
#include <string>
#include "base64_encode.hpp"

#include "IMU.h"
#include "merge.h"

#include "tft.h"

/// @brief  Read many records from the FIFO and print them.
///  It appears that all records from a clock tick appear simultaneously.
/// @param LSM
/// @param avail
/// @return
int read_all(LSMExtension &imu, lsm6dsv16x_fifo_record_t *records, int max)
{
    uint16_t actual;
    // The read time is around 2.2 msec for 20 records.
    if (LSM6DSV16X_OK != imu.Read_FIFO_Data(max, records, &actual))
    {
        printf("LSM6DSV16X Sensor failed to read FIFO data\n");
        vTaskSuspend(NULL);
    }

    return actual;
}

extern "C" void app_main()
{
    initArduino();

    setup_tft();

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);

    printf("TFT should show text now\n");

    Wire.begin(3, 4, 1000000);
    auto imu1 = init_lsm(&Wire, LSM6DSV16X_I2C_ADD_L);
    auto imu2 = init_lsm(&Wire, LSM6DSV16X_I2C_ADD_H);
    imu1.Disable_G();
    imu2.Disable_G();
    printf("LSM initialized\n");

    // Start logger task
    QueueHandle_t q = xQueueCreate(40, sizeof(LoggerMsg));
    TaskHandle_t xHandle = NULL;
    xTaskCreate(
        logger_task, /* Function that implements the task. */
        "LoggerTask",
        4096,             /* Stack size in words, not bytes. */
        (void *)q,        /* Parameter passed into the task. */
        tskIDLE_PRIORITY, /* Priority at which the task is created. */
        &xHandle);

    int led = HIGH;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    LoggerMsg msg;
    while (read_all(imu1, msg.records, 32) > 4)
        ;
    while (read_all(imu2, msg.records, 32) > 4)
        ;

    xTaskDelayUntil(&xLastWakeTime, 2);
    bool toggle = false;
    while (1)
    {
        auto delayed = xTaskDelayUntil(&xLastWakeTime, 2);
        if (true)
        {
            LoggerMsg msg;
            int actual = 0;
            msg.imu = toggle;
            msg.delayed = delayed == pdTRUE ? true : false;
            if (toggle)
            {
                actual = read_all(imu1, msg.records, 32);
            }
            else
            {
                actual = read_all(imu2, msg.records, 32);
            }
            toggle = !toggle;
            msg.read_time = esp_timer_get_time();
            msg.sample_count = actual;
            xQueueSend(q, &msg, 0);

            if (10 < uxQueueMessagesWaiting(q))
            {
                printf("**********   Warning: logger queue has %d messages pending\n", uxQueueMessagesWaiting(q));
                vTaskSuspend(NULL);
            }
        }

        auto ticks = xTaskGetTickCount();
        if (ticks / 1000 % 2 != led)
        {
            led = led ^ 1;
            digitalWrite(13, led);
        }
    }
}
