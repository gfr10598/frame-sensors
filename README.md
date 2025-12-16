# Strikeometer based on frame mounted sensors

## Repo init
This code depends on a library submodule in components/LSM6DSV...
It is necessary to run git submodule init and git submodule update.
Also, the component requires a CMakeLists.txt containing

```CMake
cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS "src/LSM6DSV16XSensor.cpp"
         "src/lsm6dsv16x_reg.c"
    INCLUDE_DIRS "src"
    REQUIRES arduino-esp32
)
```

## When compiler can't find the .h file...
idf.py reconfigure


## Data merging
When we read a set of samples, we should also read the current time after the read
completes.  This should be shortly after the last sample was pushed into the FIFO.

If we also saved the timestamp on the previous collection, we know the interval 
covered by the sample set.  We can use a more precise estimate of the sample
interval to set the approximate times of the individual samples.

Generally speaking, the two sample sets will have a nearly 1:1 correspondence.
However, if there is, e.g. a 1% skew, then roughly every 4 blocks, we will need
to insert or drop a sample from one IMU or the other.

## Sync on Counts
Every record has a count field, modulo 4.  These will have a fairly stable
alignment, with slight drift resulting in an occasional adjustment.  The 
relationship will be a modulo 4 value as well.

This should be the primary method for merging the streams, and the secondary
system should be adjusting the skew between the counts.

## Tasks

### IMU reader
Runs at high priority, every 2 msec, and ping pongs between the two IMU devices.

Reading 4 records (all we need) usually takes 850 msecc, but occasionally takes
up to 1100 msec.  Since we need to do two collections on the same bus, we
probably should target a 4 msec collection interval, which will mean about 8
records from each device.  That will take up to 1300 msec per device, which is
comfortable.

### Matcher / Encoder / Sender
Merges the data, and sends combined data out to the serial port.
A single merged record will have 6 16 bit values.  This works out to 
16 bytes in base64.  
We expect to send 10 merged records roughly every 5 msec.  The sender will receive
batches of ~8 records from each imu.
Messages will be exactly ... (180) bytes, containing 10 records of 6 channels each. 


## How multiple read works:
an4987-lsm6dsm
