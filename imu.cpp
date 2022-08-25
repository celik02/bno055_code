#include "imu.h"

using namespace std;

void print_i2c_data(char *data, size_t len)
{
    size_t i = 0;

    for (i = 0; i < len; i++) {

        if (i % 16 == 0) {

            fprintf(stdout, "\n");
        }

        fprintf(stdout, "%02x ", data[i]);
    }

    fprintf(stdout, "\n");

}

IMU_DRIVER::IMU_DRIVER()
{
    //Open the bus file
        if ((bus = i2c_open("/dev/i2c-1")) == -1) {

        /* Error process */
        cout<<"[ERRO] Bus is not created"<<endl;
        }
}

IMU_DRIVER::~IMU_DRIVER() // destructor -> close the bus
{
    i2c_close(bus);
}

void IMU_DRIVER::connect(int device_addr = 0x28)
{
        char buf[10];
        /* Open i2c bus /dev/i2c-0 */

        memset(&device_, 0, sizeof(device_)); //set all vlues of device to 0

        /* 24C04 */
        device_.bus = bus;	/* Bus 0 */
        device_.addr = device_addr;	/* Slave address is 0x50, 7-bit */
        device_.iaddr_bytes = 1;	/* Device internal address is 1 byte */
        device_.page_bytes = 0x7f; /* Device are capable of 16 bytes per page */
        // device_.delay = 10;

        unsigned char buffer[2];
        ssize_t size = sizeof(buffer);
        memset(buffer, 0, sizeof(buffer));

        /* From i2c 0x0 address read 256 bytes data to buffer */
        // just to check if connection is established
        if ((i2c_read(&device_,  0x00, buffer, size)) != size) {

            /* Error process */
            cout<<"[ERROR] occured..."<<endl;
        }
    
}

int IMU_DRIVER::init()
{
    // Power the IMU
    // return 0 for success, -1 for error

    _reset();
    
    const char powmod = 0x00;
    this->write_register(_POWER_REGISTER, &powmod, 1);
    const char page_num0 = 0x00;
    this->write_register(_PAGE_REGISTER, &page_num0, 1);
    sleep(0.2);
    const char trigger = 0x00;
    this->write_register(_TRIGGER_REGISTER, &trigger, 1);
    sleep(3.2);
    
    //default values are used.
    set_accel_range(ACCEL_4G);
    set_gyro_range(GYRO_2000_DPS);
    set_magnet_rate(MAGNET_30HZ);

    // read calibration data and write to registers
    string line;    
    std::ifstream input( "calibration_data.bin", std::ios::binary);
    // copies all data into buffer
    vector<char> calib_data(std::istreambuf_iterator<char>(input), {});
    sleep(0.01);
    update_calibration(calib_data);

    sleep(0.01);
    set_mag_op_mode(); //set mode to HIGH ACCURACY
    sleep(0.1);

    set_mode(NDOF_MODE); //AMG_MODE
    sleep(0.2);
    if(check_status_bit() == 5)
        return 0;
    return -1; // There is an error. We want to be in fusion mode.

}

void IMU_DRIVER::read_register(unsigned int iaddr,  void *buf, size_t len)
{   //iaddr -> internal register address

    if ((i2c_read(&(this->device_),  iaddr , buf, len)) != len) {

            /* Error process */
            cout<<"[ERROR] occured..."<<endl;
        }

}

// should buf be unsigned char or void pointer? --> data format is important
void IMU_DRIVER::write_register(unsigned int iaddr, const void *buf, size_t len)
{

    ssize_t ret = i2c_write_handle(&(this->device_), iaddr, buf, len);
    if (ret == -1 || (size_t)ret != len)
    {
        cout<<"[ERROR] Register could not written..."<<endl;
        return;
    }

}

void IMU_DRIVER::_reset()
{
    cout<<"Resetting..."<<endl;
    const char  config_mode = 0x00;
    const char trigger = 0x20;
    this->write_register(_MODE_REGISTER, &config_mode, 1);
    sleep(0.2);
    this->write_register(_TRIGGER_REGISTER, &trigger, 1);
    sleep(1.7);

    cout<<"Reset finished..."<<endl;
}

void IMU_DRIVER::set_mode(const char new_mode)
{   cout<<"[INFO] Changing mode..."<<endl;

    const char config_mode = CONFIG_MODE;
    this->write_register(_MODE_REGISTER, &config_mode, 1); //Emprically this is needed
    sleep(1.2);
    if(config_mode != new_mode)
    {
        
        this->write_register(_MODE_REGISTER, &new_mode, 1);
        cout<<"[INFO] Changed the mode of operation..."<<endl;
    }

    cout<<"Mode of operation changed to:"<<static_cast<int>(new_mode)<<endl;
    sleep(1.7);
}

void IMU_DRIVER::set_accel_range(const char acc_mode)
{
    //change page number
    write_register(_PAGE_REGISTER, &page_num1, 1);
    //read current conf
    unsigned char buf;
    read_register(_ACCEL_CONFIG_REGISTER, &buf, 1); 
    //set Accel Range is 4g by default
    char masked_val = 0b11111100&buf;
    char val = (masked_val | acc_mode);
    write_register(_ACCEL_CONFIG_REGISTER, &val , 1 );
    //change page number back to 0
    write_register(_PAGE_REGISTER, &page_num0, 1);
}

void IMU_DRIVER::set_gyro_range(const char gyro_mode)
{
    // if in fusion mode -> gyro range cannot be set
    write_register(_PAGE_REGISTER,&page_num1,1);
    unsigned char buf;
    read_register(_GYRO_CONFIG_0_REGISTER, &buf, 1);
    char masked_val = (0b00111000&buf) | gyro_mode;
    write_register(_GYRO_CONFIG_0_REGISTER, &masked_val,1);
    write_register(_PAGE_REGISTER, &page_num0, 1);
}

void IMU_DRIVER::set_magnet_rate(const char rate)
{
    write_register(_PAGE_REGISTER, &page_num1, 1);
    char buf;
    read_register(_MAGNET_CONFIG_REGISTER, &buf, 1);
    char masked_val =  (0b01111000 & buf)|rate;
    write_register(_MAGNET_CONFIG_REGISTER, &masked_val, 1);
    write_register(_PAGE_REGISTER, &page_num0, 1);

}


void IMU_DRIVER::set_mag_op_mode(const char op_mode)
{
    write_register(_PAGE_REGISTER, &page_num1, 1);
    char buf;
    read_register(_MAGNET_CONFIG_REGISTER, &buf, 1);
    char masked_val = (0b01100111 & buf)| op_mode;
    write_register(_MAGNET_CONFIG_REGISTER, &masked_val, 1);
    write_register(_PAGE_REGISTER, &page_num0, 1);

}

void IMU_DRIVER::set_accel_bandwith(const char acc_bandwidth)
{
    write_register(_PAGE_REGISTER, &page_num1,1);
    char value;
    read_register(_ACCEL_CONFIG_REGISTER, &value, 1);
    char masked_val = (0b11100011 & value)| acc_bandwidth;
    write_register(_ACCEL_CONFIG_REGISTER, &masked_val, 1);
    write_register(_PAGE_REGISTER, &page_num0, 1);
}

void IMU_DRIVER::set_gyro_bandwidth(const char gyro_bandwidth)
{
    write_register(_PAGE_REGISTER, &page_num1, 1);
    char value;
    read_register(_GYRO_CONFIG_0_REGISTER, &value, 1);
    char masked_val = (0b00000111 & value)| gyro_bandwidth;
    write_register(_GYRO_CONFIG_0_REGISTER, &masked_val, 1);
    write_register(_PAGE_REGISTER, &page_num0, 1);
}


dataxyz IMU_DRIVER::read_accel(int print_data)
{
    char accel_data_raw[6];
    
    read_register(0x08, &accel_data_raw, 6);
    dataxyz accel_data;
    short int x = (static_cast<short int>(accel_data_raw[1])<<8 | static_cast<short int>(accel_data_raw[0]));
    short int y = (static_cast<short int>(accel_data_raw[3])<<8 | static_cast<short int>(accel_data_raw[2]));
    short int z = (static_cast<short int>(accel_data_raw[5])<<8 | static_cast<short int>(accel_data_raw[4]));
    accel_data.x = static_cast<double>(x)/100.0;
    accel_data.y = static_cast<double>(y)/100.0;
    accel_data.z = static_cast<double>(z)/100.0;

    if(print_data==1)
        cout<<"Accel X:"<<accel_data.x<<" Y:"<<accel_data.y<<" Z:"<<accel_data.z<<endl;
    // print_i2c_data(accel_data_raw, sizeof(accel_data_raw));
    return accel_data;
}

dataxyz IMU_DRIVER::read_gyro(int print_data)
{
    char gyro_data_raw[6];
    dataxyz gyro_data;
    read_register(0x14, gyro_data_raw, 6);
    short int x = (static_cast<short int>(gyro_data_raw[1])<<8 | static_cast<short int>(gyro_data_raw[0]));
    short int y = (static_cast<short int>(gyro_data_raw[3])<<8 | static_cast<short int>(gyro_data_raw[2]));
    short int z = (static_cast<short int>(gyro_data_raw[5])<<8 | static_cast<short int>(gyro_data_raw[4]));

    gyro_data.x = static_cast<double>(x)/16.0;
    gyro_data.y = static_cast<double>(y)/16.0;
    gyro_data.z = static_cast<double>(z)/16.0;

    if(print_data)
        cout<<"Gyro X:"<<gyro_data.x<<" Y:"<<gyro_data.y<<" Z:"<<gyro_data.z<<endl;
    return gyro_data;

}


dataxyz IMU_DRIVER::read_linear_accel(int print_data)
{
    //just avaliable in fusion modes.

    char lin_accel_raw[6];
    dataxyz lin_accel_data;
    read_register(0x28, lin_accel_raw, 6);
    short int x = (static_cast<short int>(lin_accel_raw[1])<<8 | static_cast<short int>(lin_accel_raw[0]));
    short int y = (static_cast<short int>(lin_accel_raw[3])<<8 | static_cast<short int>(lin_accel_raw[2]));
    short int z = (static_cast<short int>(lin_accel_raw[5])<<8 | static_cast<short int>(lin_accel_raw[4]));
    lin_accel_data.x = static_cast<double>(x)/100.0;
    lin_accel_data.y = static_cast<double>(y)/100.0;
    lin_accel_data.z = static_cast<double>(z)/100.0;

    if(print_data)
        cout<<"LinAccel X:"<<lin_accel_data.x<<" Y:"<<lin_accel_data.y<<" Z:"<<lin_accel_data.z<<endl;

    return lin_accel_data;


}


int IMU_DRIVER::check_status_bit()
{
    write_register(_PAGE_REGISTER, &page_num0, 1);
    char status;
    read_register(0x39, &status, 1);
    cout<<"System Status:"<< static_cast<int>(status)<<endl;
    switch (static_cast<int>(status))
    {
    case 0 : 
        cout<<"[INFO] System is idle"<<endl;
        break;
    case 1 :
        cout<<"[ERROR] System Error..."<<endl;
        break;
    case 2 :
        cout<<"[INFO] Initializing peripherals"<<endl;
        break;
    case 3 :
        cout<<"[INFO] System initialization"<<endl;
        break;
    case 4:
        cout<<"[INFO] Initializing peripherals"<<endl;
        break;
    case 5:
        cout<<"[INFO] Sensor fusion algorithm running"<<endl;
        break;
    case 6:
        cout<<"[INFO] System running without fusion algorithm"<<endl;
    default:
        break;
    }

    return static_cast<int>(status);

}

dataxyz IMU_DRIVER::read_mag(int print_data)
{
    char mag_data_raw[6];
    dataxyz mag_data;
    read_register(0x0E, mag_data_raw, 6 );

    short int x = (static_cast<short int>(mag_data_raw[1])<<8 | static_cast<short int>(mag_data_raw[0]));
    short int y = (static_cast<short int>(mag_data_raw[3])<<8 | static_cast<short int>(mag_data_raw[2]));
    short int z = (static_cast<short int>(mag_data_raw[5])<<8 | static_cast<short int>(mag_data_raw[4]));
    mag_data.x = static_cast<double>(x)/16.0;
    mag_data.y = static_cast<double>(y)/16.0;
    mag_data.z = static_cast<double>(z)/16.0;

    if(print_data)
        cout<<"Mag X:"<<mag_data.x<<" Y:"<<mag_data.y<<" Z:"<<mag_data.z<<endl;
    
    // print_i2c_data(accel_data_raw, sizeof(accel_data_raw));
    return mag_data;
}


vector<int> IMU_DRIVER::calibration_status()
{
    //sys, gyro, accel, mag in order 
    char calib_raw;
    read_register(_CALIBRATION_REGISTER, &calib_raw,1);

    int sys = static_cast<int>((calib_raw>>6)&0x03);
    int gyro = static_cast<int>((calib_raw >> 4) & 0x03);
    int accel = static_cast<int>((calib_raw >> 2) & 0x03);
    int mag = static_cast<int>(calib_raw & 0x03);
    cout<<"sys:"<<sys<<" gyro:"<<gyro<<" accel:"<<accel<<" mag:"<<mag<<endl;
    vector<int> calib = {sys, gyro, accel, mag};
    return calib;
}

int IMU_DRIVER::calibrated()
{
    vector<int> calib = calibration_status();

    return (calib.at(0)==3)&&(calib.at(1)==3)&&(calib.at(2)==3)&&(calib.at(3)==3);
}

std::vector<char> IMU_DRIVER::get_calibration_data()
{   
    set_mode(CONFIG_MODE);
    //run this function after calibrated returns true.
    char accel_offset[6];
    char gyro_offset[6];
    char mag_offset[6];
    char acc_radius[2];
    char mag_radius[2];
    ofstream calib_file;
    calib_file.open("calibration_data.txt", ios::binary);

    read_register(_OFFSET_ACCEL_REGISTER, accel_offset, 6);
    read_register(_OFFSET_MAGNET_REGISTER, mag_offset, 6);
    read_register(_OFFSET_GYRO_REGISTER, gyro_offset, 6);
    read_register(_RADIUS_ACCEL_REGISTER, acc_radius, 2);
    read_register(_RADIUS_MAGNET_REGISTER, mag_radius, 2);

    int acc_x = static_cast<short int>(accel_offset[1]<<8) | static_cast<short int>(accel_offset[0]);


    vector<char> data;
    
    for(auto x: accel_offset)
        data.push_back(x);
    for(auto x: mag_offset)
        data.push_back(x);
    for(auto x: gyro_offset)
        data.push_back(x);
    for(auto x: acc_radius)
        data.push_back(x);
    for(auto x: mag_radius)
        data.push_back(x);
    
    for(auto x:data)
        calib_file<<x;

    calib_file.close();
    set_mode(NDOF_MODE);
    return data;
}

void IMU_DRIVER::update_calibration(vector<char> calib_data)
{
    set_mode(CONFIG_MODE);
    cout<<"[INFO] Performing offline calibration"<<endl;
    char* accel_offset = calib_data.data();
    char* mag_offset = calib_data.data() + 6;
    char* gyro_offset = calib_data.data() + 12;
    char* acc_radius = calib_data.data() + 18;
    char* mag_radius = calib_data.data() + 20;
    write_register(_OFFSET_ACCEL_REGISTER,accel_offset, 6);
    write_register(_OFFSET_MAGNET_REGISTER, mag_offset, 6);
    write_register(_OFFSET_GYRO_REGISTER,gyro_offset, 6);
    write_register(_RADIUS_ACCEL_REGISTER, acc_radius, 2);
    write_register(_RADIUS_MAGNET_REGISTER, mag_radius, 2);
    cout<<"[INFO] Calibration done..."<<endl;
    set_mode(NDOF_MODE);
    return;

}