#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#include "imu.h"

/***************************************************************
    ICM20600 I2C Register
 ***************************************************************/
#define ICM20600_XG_OFFS_TC_H           0x04
#define ICM20600_XG_OFFS_TC_L           0x05
#define ICM20600_YG_OFFS_TC_H           0x07
#define ICM20600_YG_OFFS_TC_L           0x08
#define ICM20600_ZG_OFFS_TC_H           0x0a
#define ICM20600_ZG_OFFS_TC_L           0x0b
#define ICM20600_SELF_TEST_X_ACCEL      0x0d
#define ICM20600_SELF_TEST_Y_ACCEL      0x0e
#define ICM20600_SELF_TEST_Z_ACCEL      0x0f
#define ICM20600_XG_OFFS_USRH           0x13
#define ICM20600_XG_OFFS_USRL           0x14
#define ICM20600_YG_OFFS_USRH           0x15
#define ICM20600_YG_OFFS_USRL           0x16
#define ICM20600_ZG_OFFS_USRH           0x17
#define ICM20600_ZG_OFFS_USRL           0x18
#define ICM20600_SMPLRT_DIV             0x19
#define ICM20600_CONFIG                 0x1a
#define ICM20600_GYRO_CONFIG            0x1b
#define ICM20600_ACCEL_CONFIG           0x1c
#define ICM20600_ACCEL_CONFIG2          0x1d
#define ICM20600_GYRO_LP_MODE_CFG       0x1e
#define ICM20600_ACCEL_WOM_X_THR        0x20
#define ICM20600_ACCEL_WOM_Y_THR        0x21
#define ICM20600_ACCEL_WOM_Z_THR        0x22
#define ICM20600_FIFO_EN                0x23
#define ICM20600_FSYNC_INT              0x36
#define ICM20600_INT_PIN_CFG            0x37
#define ICM20600_INT_ENABLE             0x38
#define ICM20600_FIFO_WM_INT_STATUS     0x39
#define ICM20600_INT_STATUS             0x3a
#define ICM20600_ACCEL_XOUT_H           0x3b
#define ICM20600_ACCEL_XOUT_L           0x3c
#define ICM20600_ACCEL_YOUT_H           0x3d
#define ICM20600_ACCEL_YOUT_L           0x3e
#define ICM20600_ACCEL_ZOUT_H           0x3f
#define ICM20600_ACCEL_ZOUT_L           0x40
#define ICM20600_TEMP_OUT_H             0x41
#define ICM20600_TEMP_OUT_L             0x42
#define ICM20600_GYRO_XOUT_H            0x43
#define ICM20600_GYRO_XOUT_L            0x44
#define ICM20600_GYRO_YOUT_H            0x45
#define ICM20600_GYRO_YOUT_L            0x46
#define ICM20600_GYRO_ZOUT_H            0x47
#define ICM20600_GYRO_ZOUT_L            0x48
#define ICM20600_SELF_TEST_X_GYRO       0x50
#define ICM20600_SELF_TEST_Y_GYRO       0x51
#define ICM20600_SELF_TEST_Z_GYRO       0x52
#define ICM20600_FIFO_WM_TH1            0x60
#define ICM20600_FIFO_WM_TH2            0x61
#define ICM20600_SIGNAL_PATH_RESET      0x68
#define ICM20600_ACCEL_INTEL_CTRL       0x69
#define ICM20600_USER_CTRL              0x6A
#define ICM20600_PWR_MGMT_1             0x6b
#define ICM20600_PWR_MGMT_2             0x6c
#define ICM20600_I2C_IF                 0x70
#define ICM20600_FIFO_COUNTH            0x72
#define ICM20600_FIFO_COUNTL            0x73
#define ICM20600_FIFO_R_W               0x74
#define ICM20600_WHO_AM_I               0x75
#define ICM20600_XA_OFFSET_H            0x77
#define ICM20600_XA_OFFSET_L            0x78
#define ICM20600_YA_OFFSET_H            0x7a
#define ICM20600_YA_OFFSET_L            0x7b
#define ICM20600_ZA_OFFSET_H            0x7d
#define ICM20600_ZA_OFFSET_L            0x7e

#define ICM20600_I2C_ADDR1              0x68
#define ICM20600_I2C_ADDR2              0x69

#define ICM20600_FIFO_EN_BIT            (1 << 6)
#define ICM20600_FIFO_RST_BIT           (1 << 2)
#define ICM20600_RESET_BIT              (1 << 0)
#define ICM20600_DEVICE_RESET_BIT       (1 << 7)

#define I2C0_NODE DT_NODELABEL(icm20600)

LOG_MODULE_REGISTER(imu, CONFIG_IMU_LOG_LEVEL);

// Gyroscope scale range
enum gyro_scale_type_t {
    RANGE_250_DPS = 0,
    RANGE_500_DPS,
    RANGE_1K_DPS,
    RANGE_2K_DPS,
};

// Accelerometer scale range
enum acc_scale_type_t {
    RANGE_2G = 0,
    RANGE_4G,
    RANGE_8G,
    RANGE_16G,
};

// Gyroscope output data rate
enum gyro_lownoise_odr_type_t {
    GYRO_RATE_8K_BW_3281 = 0,
    GYRO_RATE_8K_BW_250,
    GYRO_RATE_1K_BW_176,
    GYRO_RATE_1K_BW_92,
    GYRO_RATE_1K_BW_41,
    GYRO_RATE_1K_BW_20,
    GYRO_RATE_1K_BW_10,
    GYRO_RATE_1K_BW_5,
};

// Accelerometer output data rate
enum acc_lownoise_odr_type_t {
    ACC_RATE_4K_BW_1046 = 0,
    ACC_RATE_1K_BW_420,
    ACC_RATE_1K_BW_218,
    ACC_RATE_1K_BW_99,
    ACC_RATE_1K_BW_44,
    ACC_RATE_1K_BW_21,
    ACC_RATE_1K_BW_10,
    ACC_RATE_1K_BW_5,
};

// Averaging filter settings for Low Power Accelerometer mode
enum acc_averaging_sample_type_t {
    ACC_AVERAGE_4 = 0,
    ACC_AVERAGE_8,
    ACC_AVERAGE_16,
    ACC_AVERAGE_32,
};

// Averaging filter configuration for low-power gyroscope mode
enum gyro_averaging_sample_type_t {
    GYRO_AVERAGE_1 = 0,
    GYRO_AVERAGE_2,
    GYRO_AVERAGE_4,
    GYRO_AVERAGE_8,
    GYRO_AVERAGE_16,
    GYRO_AVERAGE_32,
    GYRO_AVERAGE_64,
    GYRO_AVERAGE_128,
};

// ICM20600 power mode
enum icm20600_power_type_t {
    ICM_SLEEP_MODE = 0,
    ICM_STANDYBY_MODE,
    ICM_ACC_LOW_POWER,
    ICM_ACC_LOW_NOISE,
    ICM_GYRO_LOW_POWER,
    ICM_GYRO_LOW_NOISE,
    ICM_6AXIS_LOW_POWER,
    ICM_6AXIS_LOW_NOISE,
};


//LOG_MODULE_DECLARE(imu, CONFIG_IMU_LOG_LEVEL);

int32_t Acc_x, Acc_y, Acc_z, Gyro_x, Gyro_y, Gyro_z;
int16_t Temp;
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);
int ret;
int8_t Buffer[16] = {0};
uint16_t _acc_scale, _gyro_scale;

void ICM20600_startup(void)
{
    LOG_DBG("Starting...\n");//I2C_SPEED_GET(cfg)
    if (device_is_ready(dev_i2c.bus)) {
	LOG_DBG("I2C bus %X is ready!\n\r",(uint32_t)(dev_i2c.bus->name));
    }
    

    // Get the Device ID
	ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_WHO_AM_I,Buffer);
    LOG_DBG("Stage: Config\n"); 
	ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_CONFIG,0x00);

	// disable fifo: 1K byte FIFO buffer enables the applications processor to read the data in bursts

    LOG_DBG("Stage: disabling fifo\n"); 
	ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_FIFO_EN,0x00);
	
    //Set Power 
    uint8_t data_pwr1;
    uint8_t data_pwr2 = 0x00;
    uint8_t data_gyro_lp;
    LOG_DBG("Stage: Setting Power\n"); 
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_PWR_MGMT_1,Buffer); // _addr, ICM20600_PWR_MGMT_1, Buffer
    LOG_DBG("Power Management 1: %X\n",Buffer[0]);
    data_pwr1 = Buffer[0];
    data_pwr1 &= 0x8f;                  // 0b10001111
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_LP_MODE_CFG,Buffer);
    LOG_DBG("Power Management 2: %X\n",Buffer[0]);
    data_gyro_lp = Buffer[0];
    // When set to ‘1’ low-power gyroscope mode is enabled. Default setting is 0
    data_gyro_lp &= 0x7f;               // 0b01111111
    data_pwr1 |= 0x00;          // dont set bit5 0b00100000
    data_gyro_lp |= 0x80;        // set 0b01000000


    LOG_DBG("Gyro Low Power: %X\n",data_gyro_lp);
    LOG_DBG("Stage: Setting Power mode 1\n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_PWR_MGMT_1,data_pwr1);
    LOG_DBG("Stage: Setting Power mode 2\n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_PWR_MGMT_2,data_pwr2);
    LOG_DBG("Stage: Setting low power mode for Gyro \n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_GYRO_LP_MODE_CFG,data_gyro_lp);
    
    // Gyro Config
    LOG_DBG("Stage: reading Gyro Config \n");
    uint8_t data = 0;
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_CONFIG,Buffer);
    LOG_DBG("Gyro Config: %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0xe7; // 0b11100111
    data |= 0x18;   // 0bxxx11xxx
    _gyro_scale = 4000;


    LOG_DBG("Stage: Setting Gyro Config \n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_GYRO_CONFIG,data);
    data = 0;
     ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_CONFIG,Buffer);
     LOG_DBG("Gyro Config: %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0xf8;
    data |= 0x01;
    LOG_DBG("Stage: Setting I2c device config \n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_CONFIG,data);
    data = 0;
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_LP_MODE_CFG,Buffer);
    LOG_DBG("Gyro Config lp mode cfg(maybe 0): %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0x8f;  
    data |= 0x00;         // 0b10001111
    LOG_DBG("Stage: Setting setting i2c gyro config \n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_GYRO_CONFIG,data);
    // accel config
    data = 0;
    LOG_DBG("Stage: Setting i2c accel config \n");
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG,Buffer);
    LOG_DBG("Gyro Accleration Config: %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0xe7;
    data |= 0x18;   // 0bxxx11xxx
    _acc_scale = 32000;
    LOG_DBG("Gyro Accleration Config set point: %X\n",data);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG,data);
    data = 0;
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG2,Buffer);
    LOG_DBG("Gyro Accleration Config: %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0xf0;  // 0b11110000
    data |= 0x07;
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG2, data);
    data = 0;
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG2,Buffer);
    LOG_DBG("Gyro Accleration Config 2: %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0xcf; // & 0b11001111
    data |= 0x00;
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG2,data);
}

int32_t getRawAccelerationX(void) {
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_XOUT_H,Buffer);
    int32_t raw_data = ((int16_t)Buffer[0] << 8) + Buffer[1];
    raw_data = (raw_data * _acc_scale) >> 16;
    return (int16_t)raw_data*9.81;
}

int32_t getRawAccelerationY(void) {
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_YOUT_H,Buffer);
    int32_t raw_data = ((int16_t)Buffer[0] << 8) + Buffer[1];
    raw_data = (raw_data * _acc_scale) >> 16;
    return (int16_t)raw_data*9.81;
}

int32_t getRawAccelerationZ(void) {
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_ZOUT_H,Buffer);
    int32_t raw_data = ((int16_t)Buffer[0] << 8) + Buffer[1];
    raw_data = (raw_data * _acc_scale) >> 16;
    return (int16_t)raw_data*9.81;
}


int16_t getRawGyroscopeX(void) {
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_XOUT_H,Buffer);
    int32_t raw_data = ((int32_t)Buffer[0] << 8) + Buffer[1];
    raw_data = (raw_data * _gyro_scale) >> 16;
    return (int16_t)raw_data;
}

int16_t getRawGyroscopeY(void) {
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_YOUT_H,Buffer);
    int32_t raw_data = ((int32_t)Buffer[0] << 8) + Buffer[1];
    raw_data = (raw_data * _gyro_scale) >> 16;
    return (int16_t)raw_data;
}

int16_t getRawGyroscopeZ(void) {
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_ZOUT_H,Buffer);
    int32_t raw_data = ((int32_t)Buffer[0] << 8) + Buffer[1];
    raw_data = (raw_data * _gyro_scale) >> 16;
    return (int16_t)raw_data;
}

int16_t getTemperature(void) {
    uint16_t rawdata;
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_TEMP_OUT_H,Buffer);
    rawdata = (((uint32_t)Buffer[0]) << 8) + Buffer[1];
    return (int16_t)(rawdata / 327 + 25);
}

void imuTestLoop(void) {
    while(true)
    {
        LOG_DBG("Accl_x: %d mm/s\n",getRawAccelerationX());
        LOG_DBG("Accl_y: %d mm/s\n",getRawAccelerationY());
        LOG_DBG("Accl_z: %d mm/s\n",getRawAccelerationZ());
        LOG_DBG("Gyro_x: %d dps\n",getRawGyroscopeX());
        LOG_DBG("Gyro_y: %d dps\n",getRawGyroscopeY());
        LOG_DBG("Gyro_z: %d dps\n",getRawGyroscopeZ());
        k_msleep(500);
    }
}
