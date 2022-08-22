#include <iostream>
#include "imu.h"

// void print_i2c_data(const unsigned char *data, size_t len)
// {
//     size_t i = 0;

//     for (i = 0; i < len; i++) {

//         if (i % 16 == 0) {

//             fprintf(stdout, "\n");
//         }

//         fprintf(stdout, "%02x ", data[i]);
//     }

//     fprintf(stdout, "\n");

// }


int main()
{
    IMU_DRIVER imu_reader;
    imu_reader.init();
    unsigned char buffer[1];
    imu_reader.connect(0x28);
    imu_reader.read_register(0x01, buffer, sizeof(buffer) );
    print_i2c_data(buffer, sizeof(buffer));
    unsigned char write_buf[] = "0x23";
    imu_reader.write_register( 0xf, write_buf, sizeof(write_buf));


    return 0;
}