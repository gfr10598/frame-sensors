#include "IMU.h"

/**
 * @brief  Get the LSM6DSV16X FIFO raw data

 * @param  Data FIFO raw data array [6]
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSMExtension::FIFO_Get_Data(uint8_t *Data)
{
    return (LSM6DSV16XStatusTypeDef)lsm6dsv16x_read_reg(&reg_ctx, LSM6DSV16X_FIFO_DATA_OUT_X_L, Data, 6);
}

LSM6DSV16XStatusTypeDef LSMExtension::FIFO_Get_Tag_And_Data(uint8_t *Data)
{
    return (LSM6DSV16XStatusTypeDef)lsm6dsv16x_read_reg(&reg_ctx, LSM6DSV16X_FIFO_DATA_OUT_TAG, Data, 7);
}

/**
 * @brief  Read multiple records from the LSM6DSV16X FIFO
 * @param  max Maximum number of records to read
 * @param  count Count of records read
 * @param  records Array of records
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSV16XStatusTypeDef LSMExtension::Read_FIFO_Data(uint16_t max, lsm6dsv16x_fifo_record_t *records, uint16_t *count)
{
    int status = FIFO_Get_Num_Samples(count);
    if (status != LSM6DSV16X_OK)
        return LSM6DSV16X_ERROR;
    if (*count == 0)
    {
        return LSM6DSV16X_OK;
    }
    if (*count > max)
        *count = max;
    // If we read more than this, i2c doesn't seem to actually read all the data.
    if (*count > 32)
        *count = 32;
    status = lsm6dsv16x_read_reg(&reg_ctx, LSM6DSV16X_FIFO_DATA_OUT_TAG, (uint8_t *)records, *count * 7);
    return (LSM6DSV16XStatusTypeDef)status;
}

static DMA_ATTR lsm6dsv16x_fifo_record_t records[32];

LSM6DSV16XStatusTypeDef LSMExtension::Slow()
{
    LSM6DSV16XStatusTypeDef status = Set_SFLP_ODR(LSM6DSV16X_ODR_AT_1Hz875);
    if (status != LSM6DSV16X_OK)
        return status;
    status = Set_X_ODR(15);
    if (status != LSM6DSV16X_OK)
        return status;
    status = Set_G_ODR(15);
    if (status != LSM6DSV16X_OK)
        return status;
    status = FIFO_Set_X_BDR(0); // disable sensor output to FIFO
    if (status != LSM6DSV16X_OK)
        return status;
    status = FIFO_Set_G_BDR(0);

    if (status != LSM6DSV16X_OK)
        return status;
    status = FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
    if (status != LSM6DSV16X_OK)
        return status;
    status = FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
    if (status != LSM6DSV16X_OK)
        return status;

    return LSM6DSV16X_OK;
}

// NOT thread safe!
void LSMExtension::HandleSlow()
{
    // The driver will only read 32 records at a time - 16 samples at 1.875 Hz is about 8 seconds.
    uint16_t samples_read = 0;
    LSM6DSV16XStatusTypeDef status = Read_FIFO_Data(32, &records[0], &samples_read);

    if (status != LSM6DSV16X_OK)
    {
        printf("Error reading slow FIFO data %d\n", status);
        return;
    }
    if (samples_read == 0)
    {
        // No data available.
        printf("No slow data available\n");
        return;
    }
    for (uint16_t i = 0; i < samples_read; i++)
    {
        lsm6dsv16x_fifo_record_t datum = records[i];
        printf("Slow Record %d: Cnt=0x%02X  Tag=0x%02X Data=%-6d %-6d %-6d\n", i,
               datum.tag.tag_cnt,
               datum.tag.tag_sensor,
               datum.data[0], datum.data[1], datum.data[2]);
    }
}

#define FIFO_SAMPLE_THRESHOLD 20
#define FLASH_BUFF_LEN 8192
// Writing to flash takes 4 msec for 4kB at 16MHz SPI clock.
// We only need to write about 7*2*2k = 28k bytes per second, so we only need to write
// about 7 blocks per second.  We need to read the sensor about every 5 msec though,
// to keep from overflowing the FIFO.
// So we might need to offload the flash writing to a task on the other processor.
#define SENSOR_ODR 1920

LSMExtension init_lsm(TwoWire *wire, uint8_t address)
{
    // Initialize i2c.
    // We need to read roughly 7*2*2khz = 28k bytes per second from the LSM6DSV16X.
    // Because we are reading many bytes at a time, 1MHz I2C can provide perhaps
    // 100k bytes/sec.  So we will be running around 30% duty cycle just reading the data.
    LSMExtension LSM(wire, address);
    printf("LSM (extension) created\n");
    if (LSM6DSV16X_OK != LSM.begin())
    {
        printf("LSM.begin() Error\n");
    }
    int status = 0;

    // The gyroscopes have 16 bit signed range, so +/- 32768 counts.
    // At 2000 dps, this give 16.384 LSB/dps, or about 61 dps per count.
    // The 2024 data is collected with FS = 1000 dps.
    // The potential error of 1/32 of a degree per second is not actually observed,
    // because noise in the signal means that quantization error is distributed randomly.
    // So, we expect jitter of about 1/32/sqrt(2000) per root hz.  So, at 1 hz, the jitter
    // is reduced to about 1/1500 degree, which is much smaller than the other sources of error.

    // We should probably be just fine using FS=2000, which gives more headroom for shocks.

    if (status == 0)
        status |= LSM.Set_G_FS(1000); // Need minimum of 600 dps.
    if (status == 0)
        status |= LSM.Set_X_FS(16); // To handle large impulses from clapper.
    if (status == 0)
        status |= LSM.Set_X_ODR(SENSOR_ODR);
    if (status == 0)
        status |= LSM.Set_G_ODR(SENSOR_ODR);
    if (status == 0)
        status |= LSM.Set_Temp_ODR(LSM6DSV16X_TEMP_BATCHED_AT_1Hz875);

    // Set FIFO to timestamp data at 20 Hz
    if (status == 0)
        status |= LSM.FIFO_Enable_Timestamp();
    if (status == 0)
        status |= LSM.FIFO_Set_Timestamp_Decimation(LSM6DSV16X_TMSTMP_DEC_32);
    if (status == 0)
        status |= LSM.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);

    // Configure FIFO BDR for acc and gyro
    if (status == 0)
        status |= LSM.FIFO_Set_X_BDR(SENSOR_ODR);
    if (status == 0)
        status |= LSM.FIFO_Set_G_BDR(SENSOR_ODR);

    // Set FIFO in Continuous mode
    if (status == 0)
        status |= LSM.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

    if (status == 0)
        status |= LSM.Enable_G();
    if (status == 0)
        status |= LSM.Enable_X();
    if (status == 0)
        status |= LSM.Enable_Gravity_Vector();
    if (status == 0)
        status |= LSM.Enable_Gyroscope_Bias();
    if (status == 0)
        status |= LSM.Set_SFLP_Batch(false, true, true);
    if (status == 0)
        status |= LSM.Set_SFLP_ODR(LSM6DSV16X_SFLP_15Hz);
    if (status != LSM6DSV16X_OK)
    {
        printf("LSM6DSV16X Sensor failed to configure %d\n", status);
        while (1)
            ;
    }
    else
    {
        printf("LSM enabled\n");
        delay(3); // Should allow about 12 samples.
        uint16_t samples = 0;
        LSM.FIFO_Get_Num_Samples(&samples);
        lsm6dsv16x_fifo_record_t data[32];
        uint16_t samples_read = 0;
        LSM.Read_FIFO_Data(32, &data[0], &samples_read);
        printf("%d Samples available  %d Samples read\n", samples, samples_read);
        for (uint16_t i = 0; i < samples_read; i++)
        {
            lsm6dsv16x_fifo_record_t datum = data[i];
            printf("Record %d: Cnt=0x%02X  Tag=0x%02X Data=%-6d %-6d %-6d\n", i,
                   datum.tag.tag_cnt,
                   datum.tag.tag_sensor,
                   datum.data[0], datum.data[1], datum.data[2]);
        }
    }

    // LSM.Slow();
    // for (int i = 0; i < 10; i++)
    // {
    //     delay(1000);
    //     LSM.HandleSlow();
    // }

    return LSM;
}
