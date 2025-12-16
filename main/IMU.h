
#ifndef IMU_H
#define IMU_H

#include "LSM6DSV16XSensor.h"

typedef struct __attribute__((packed)) lsm6dsv16x_fifo_record_t
{
    lsm6dsv16x_fifo_data_out_tag_t tag;
    int16_t data[3];
} lsm6dsv16x_fifo_record_t;

class LSMExtension : public LSM6DSV16XSensor
{
public:
    using LSM6DSV16XSensor::LSM6DSV16XSensor;

    LSM6DSV16XStatusTypeDef FIFO_Get_Data(uint8_t *Data);
    LSM6DSV16XStatusTypeDef FIFO_Get_Tag_And_Data(uint8_t *Data);
    LSM6DSV16XStatusTypeDef Read_FIFO_Data(uint16_t max, lsm6dsv16x_fifo_record_t *records, uint16_t *count);

    LSM6DSV16XStatusTypeDef Fast()
    {
        LSM6DSV16XStatusTypeDef status = Enable_G();
        if (status != LSM6DSV16X_OK)
            return status;
        return Enable_X();
    }

    LSM6DSV16XStatusTypeDef Medium()
    {
        LSM6DSV16XStatusTypeDef status = Set_X_ODR(LSM6DSV16X_ODR_AT_15Hz);
        if (status != LSM6DSV16X_OK)
            return status;
        return Fast();
    }

    // Slow just reads the gyro bias and gravity vector at 15 Hz.
    // The 1.5k FIFO will fill at 14 * 15 bytes/sec, so we have to read it about every 5 seconds,
    // unless we don't mind losing data.
    // For actual lowest power operation, we need to run in low power mode, accelerometer only.
    // We can still use SFLP at 15 Hz, with very low power consumption on the device.
    // But the gyro bias won't be available, which is unfortunate.
    // So - how about switching to gyro+accel mode about once a minute or so, to
    // get the gyro bias.  Then we will have a recent bias we can use when we have to
    // wake up to collect data.
    LSM6DSV16XStatusTypeDef Slow();

    // Query the IMU in slow mode.
    void HandleSlow();
};

LSMExtension init_lsm(TwoWire *wire, uint8_t address = LSM6DSV16X_I2C_ADD_H);

#endif // IMU_H
