#include <iostream>
#include "imu.h"
#include <chrono>
#include <ctime>  

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

using namespace std;
int main()
{
    IMU_DRIVER imu_reader;
    imu_reader.connect(0x28);
    imu_reader.init();
    char buffer[128];

    const char page_num1 = 0x00;
    imu_reader.write_register(_PAGE_REGISTER, &page_num1, 1);
    // Read register works as expected
    imu_reader.read_register(0x00, buffer, sizeof(buffer));
    cout<<"Page 0";
    print_i2c_data(buffer, sizeof(buffer));


    const char page_num2 = 0x01;
    imu_reader.write_register(_PAGE_REGISTER, &page_num2, 1);
    // Read register works as expected
    imu_reader.read_register(0x00, buffer, sizeof(buffer));
    cout<<"Page 1";
    print_i2c_data(buffer, sizeof(buffer));
    int status = 0;
    imu_reader.write_register(_PAGE_REGISTER, &page_num1, 1);
     
    // Some computation here
    auto end = std::chrono::system_clock::now();


    int just_once = 1;
    // imu_reader.get_calibration_data();

    while(true)
    {
            auto start = std::chrono::system_clock::now();
            cout<<"--------------"<<endl;
            imu_reader.read_accel();
            imu_reader.read_gyro();
            imu_reader.read_mag();
            cout<<"--------------"<<endl;
            if(status==5){

            imu_reader.read_linear_accel();
            }
            //// just shows elapsed timed
            // auto end = std::chrono::system_clock::now();

            // std::chrono::duration<double> elapsed_seconds = end-start;
            // std::time_t end_time = std::chrono::system_clock::to_time_t(end);
        
            // std::cout << "finished computation at " << std::ctime(&end_time)
            //         << "elapsed time: " << elapsed_seconds.count() << "s"
            //         << std::endl;
            status = imu_reader.check_status_bit();

            imu_reader.calibration_status();
            
            if(imu_reader.calibrated() && just_once)
            {
                vector<char> l = imu_reader.get_calibration_data();
                just_once = 0;
                
            }
            imu_reader.read_register(0x00, buffer, sizeof(buffer));
            cout<<"Page 1";
            print_i2c_data(buffer, sizeof(buffer));
            sleep(1);
    
    }

    return 0;
}