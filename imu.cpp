#include "imu.h"

using namespace std;

void print_i2c_data(const unsigned char *data, size_t len)
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

void IMU_DRIVER::connect(int device_addr = 0x28)
{
        char buf[10];
        /* Open i2c bus /dev/i2c-0 */

        memset(&device_, 0, sizeof(device_)); //set all vlues of device to 0

        /* 24C04 */
        device_.bus = bus;	/* Bus 0 */
        device_.addr = device_addr;	/* Slave address is 0x50, 7-bit */
        device_.iaddr_bytes = 1;	/* Device internal address is 1 byte */
        device_.page_bytes = 16; /* Device are capable of 16 bytes per page */

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

void IMU_DRIVER::read_register(unsigned int iaddr,  void *buf, size_t len)
{   //iaddr -> internal register address

    if ((i2c_read(&(this->device_),  iaddr , buf, len)) != len) {

            /* Error process */
            cout<<"[ERROR] occured..."<<endl;
        }

}

// should buf be unsigned char or void pointer? --> data format is important
void IMU_DRIVER::write_register(unsigned int iaddr, unsigned char *buf, size_t len)
{

    ssize_t ret = i2c_ioctl_write(&(this->device_), iaddr, buf, len);
    if (ret == -1 || (size_t)ret != len)
    {
        cout<<"[ERROR] Register could not written..."<<endl;
    }

    // print_i2c_data(buf, len);
    cout<< buf<<" is written to "<< iaddr<< endl;

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

void IMU_DRIVER::_reset()
{
    this->set_mode(CONFIG_MODE);
    cout<<"[INFO] IMU resetted..."<<endl;
}

void IMU_DRIVER::set_mode(int new_mode)
{   cout<<"Try to Changed mode..."<<endl;
    unsigned char buf_conf = static_cast<unsigned char>(CONFIG_MODE);
    unsigned char buf_new = static_cast<unsigned char>(new_mode);
    this->write_register(_MODE_REGISTER, &buf_conf, 1);
    sleep(0.02);
    cout<<"Changed mode..."<<endl;
    if(new_mode != CONFIG_MODE)
    {
        this->write_register(_MODE_REGISTER, &buf_new, 1);
    }
}

void IMU_DRIVER::init()
{
    _reset();
}
