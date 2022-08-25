// #include <linux/i2c-dev.h>
// #include <i2c/smbus.h>

#include <sys/stat.h>
#include <fcntl.h>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
    #include <stdio.h> 
    #include <stdlib.h> 
    }
#include <iostream>
#include <sys/ioctl.h>
#include "i2c.h"
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <vector>
#include <fstream>

#define _POWER_REGISTER 0x3E
#define CONFIG_MODE 0x00
#define ACCONLY_MODE  0x01
#define MAGONLY_MODE  0x02
#define GYRONLY_MODE 0x03
#define ACCMAG_MODE 0x04
#define ACCGYRO_MODE 0x05
#define MAGGYRO_MODE 0x06
#define AMG_MODE 0x07
#define IMUPLUS_MODE 0x08
#define COMPASS_MODE 0x09
#define M4G_MODE 0x0A
#define NDOF_FMC_OFF_MODE 0x0B
#define NDOF_MODE 0x0C 

#define ACCEL_2G 0x00  // For accel_range property
#define ACCEL_4G 0x01  // Default
#define ACCEL_8G 0x02
#define ACCEL_16G 0x03
#define ACCEL_7_81HZ 0x00  // For accel_bandwidth property
#define ACCEL_15_63HZ 0x04
#define ACCEL_31_25HZ 0x08
#define ACCEL_62_5HZ 0x0C  // Default
#define ACCEL_125HZ 0x10
#define ACCEL_250HZ 0x14
#define ACCEL_500HZ 0x18
#define ACCEL_1000HZ 0x1C
#define ACCEL_NORMAL_MODE 0x00  // Default. For accel_mode property
#define ACCEL_SUSPEND_MODE 0x20
#define ACCEL_LOWPOWER1_MODE 0x40
#define ACCEL_STANDBY_MODE 0x60
#define ACCEL_LOWPOWER2_MODE 0x80
#define ACCEL_DEEPSUSPEND_MODE 0xA0 #define 

#define GYRO_2000_DPS 0x00  // Default. For gyro_range property
#define GYRO_1000_DPS 0x01
#define GYRO_500_DPS 0x02
#define GYRO_250_DPS 0x03
#define GYRO_125_DPS 0x04
#define GYRO_523HZ 0x00  // For gyro_bandwidth property
#define GYRO_230HZ 0x08
#define GYRO_116HZ 0x10
#define GYRO_47HZ 0x18
#define GYRO_23HZ 0x20
#define GYRO_12HZ 0x28
#define GYRO_64HZ 0x30
#define GYRO_32HZ 0x38  // Default
#define GYRO_NORMAL_MODE 0x00  // Default. For gyro_mode property
#define GYRO_FASTPOWERUP_MODE 0x01
#define GYRO_DEEPSUSPEND_MODE 0x02
#define GYRO_SUSPEND_MODE 0x03
#define GYRO_ADVANCEDPOWERSAVE_MODE 0x04 

#define MAGNET_2HZ 0x00  // For magnet_rate property
#define MAGNET_6HZ 0x01
#define MAGNET_8HZ 0x02
#define MAGNET_10HZ 0x03
#define MAGNET_15HZ 0x04
#define MAGNET_20HZ 0x05  // Default
#define MAGNET_25HZ 0x06
#define MAGNET_30HZ 0x07
#define MAGNET_LOWPOWER_MODE 0x00  // For magnet_operation_mode property
#define MAGNET_REGULAR_MODE 0x08  // Default
#define MAGNET_ENHANCEDREGULAR_MODE 0x10
#define MAGNET_ACCURACY_MODE 0x18
#define MAGNET_NORMAL_MODE 0x00  // for magnet_power_mode property
#define MAGNET_SLEEP_MODE 0x20
#define MAGNET_SUSPEND_MODE 0x40
#define MAGNET_FORCEMODE_MODE 0x60  // Default

#define _POWER_NORMAL 0x00
#define _POWER_LOW 0x01
#define _POWER_SUSPEND 0x02

#define _MODE_REGISTER 0x3D
#define _PAGE_REGISTER 0x07
#define _ACCEL_CONFIG_REGISTER 0x08
#define _MAGNET_CONFIG_REGISTER 0x09
#define _GYRO_CONFIG_0_REGISTER 0x0A
#define _GYRO_CONFIG_1_REGISTER 0x0B
#define _CALIBRATION_REGISTER 0x35
#define _OFFSET_ACCEL_REGISTER 0x55
#define _OFFSET_MAGNET_REGISTER 0x5B
#define _OFFSET_GYRO_REGISTER 0x61
#define _RADIUS_ACCEL_REGISTER 0x67
#define _RADIUS_MAGNET_REGISTER 0x69
#define _TRIGGER_REGISTER 0x3F
#define _POWER_REGISTER 0x3E
#define _ID_REGISTER 0x00
// Axis remap registers and values
#define _AXIS_MAP_CONFIG_REGISTER 0x41
#define _AXIS_MAP_SIGN_REGISTER 0x42
#define AXIS_REMAP_X 0x00
#define AXIS_REMAP_Y 0x01
#define AXIS_REMAP_Z 0x02
#define AXIS_REMAP_POSITIVE 0x00
#define AXIS_REMAP_NEGATIVE  0x01

void print_i2c_data(char *data, size_t len);
class dataxyz
{
    public:
        double x;
        double y;
        double z;


};

class IMU_DRIVER
{
    /*
    this class will be used for BNO055
    */
    public:
        int bus;
        // since these two used much
        const char page_num0 = 0x00;
        const char page_num1 = 0x01;
        I2C_WRITE_HANDLE i2c_write_handle = i2c_write;
        IMU_DRIVER();
        ~IMU_DRIVER();
        I2CDevice device_;
        char filename[20];
        void connect(int adapter_nr_);
        void write_register(unsigned int iaddr, const void *buf, size_t len);
        void read_register(unsigned int iaddr,  void *buf, size_t len);
        void _reset();
        void set_mode(const char new_mode=AMG_MODE);
        int init();
        void set_accel_range(const char acc_mode = ACCEL_4G);
        void set_gyro_range(const char gyro_mode = GYRO_2000_DPS); // max dps -> slow rotation will not be detected.
        void set_magnet_rate(const char rate = MAGNET_30HZ);
        void set_mag_op_mode(const char op_mode=MAGNET_ACCURACY_MODE);
        void set_accel_bandwith(const char acc_bandwidth = ACCEL_62_5HZ);
        void set_gyro_bandwidth(const char gyro_bandwidth=GYRO_32HZ);
        int check_status_bit();
        std::vector<int> calibration_status();
        int calibrated();
        std::vector<char> get_calibration_data();
        void update_calibration(std::vector<char> calib_data);

        //NOTE: for magnotometer FORCED mode is default so it kept that way
        dataxyz read_accel(int print_data = 0);
        dataxyz read_linear_accel(int print_data = 0);
        dataxyz read_gyro(int print_data = 0);
        dataxyz read_mag(int print_data = 0);



        
};
