#ifndef ICM20600_h
#define ICM20600_h
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/device.h>


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

// System constants
#define gyroMeasError 0.5/3.14159265358979f   // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define I2C0_NODE DT_NODELABEL(icm20600) 
#define g 9.80665
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

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions
double SE_q[4] = {1,0,0,0};
double theta = 0;
double phi = 0;
double psi = 0;

void ICM20600_startup()
{
    int ret;
    int8_t Buffer[16] = {0};
    printf("Starting...\n");//I2C_SPEED_GET(cfg)
    if (device_is_ready(dev_i2c.bus)) {
	printf("I2C bus %X is ready!\n\r",(uint32_t)(dev_i2c.bus->name));
    }
    

    // Get the Device ID
	ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_WHO_AM_I,Buffer);
    printf("Stage: Config\n"); 
	ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_CONFIG,0x00);

	// disable fifo: 1K byte FIFO buffer enables the applications processor to read the data in bursts

    printf("Stage: disabling fifo\n"); 
	ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_FIFO_EN,0x00);
	
    //Set Power 
    uint8_t data_pwr1;
    uint8_t data_pwr2 = 0x00;
    uint8_t data_gyro_lp;
    printf("Stage: Setting Power\n"); 
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_PWR_MGMT_1,Buffer); // _addr, ICM20600_PWR_MGMT_1, Buffer
    printf("Power Management 1: %X\n",Buffer[0]);
    data_pwr1 = Buffer[0];
    data_pwr1 &= 0x8f;                  // 0b10001111
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_LP_MODE_CFG,Buffer);
    printf("Power Management 2: %X\n",Buffer[0]);
    data_gyro_lp = Buffer[0];
    // When set to ‘1’ low-power gyroscope mode is enabled. Default setting is 0
    data_gyro_lp &= 0x7f;               // 0b01111111
    data_pwr1 |= 0x00;          // dont set bit5 0b00100000
    data_gyro_lp |= 0x80;        // set 0b01000000


    printf("Gyro Low Power: %X\n",data_gyro_lp);
    printf("Stage: Setting Power mode 1\n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_PWR_MGMT_1,data_pwr1);
    printf("Stage: Setting Power mode 2\n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_PWR_MGMT_2,data_pwr2);
    printf("Stage: Setting low power mode for Gyro \n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_GYRO_LP_MODE_CFG,data_gyro_lp);
    
    // Gyro Config
    printf("Stage: reading Gyro Config \n");
    uint8_t data = 0;
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_CONFIG,Buffer);
    printf("Gyro Config: %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0xe7; // 0b11100111
    data |= 0x18;   // 0bxxx11xxx
    


    printf("Stage: Setting Gyro Config \n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_GYRO_CONFIG,data);
    data = 0;
     ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_CONFIG,Buffer);
     printf("Gyro Config: %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0xf8;
    data |= 0x01;
    printf("Stage: Setting I2c device config \n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_CONFIG,data);
    data = 0;
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_LP_MODE_CFG,Buffer);
    printf("Gyro Config lp mode cfg(maybe 0): %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0x8f;  
    data |= 0x00;         // 0b10001111
    printf("Stage: Setting setting i2c gyro config \n");
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_GYRO_CONFIG,data);
    // accel config
    data = 0;
    printf("Stage: Setting i2c accel config \n");
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG,Buffer);
    printf("Gyro Accleration Config: %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0xe7;
    data |= 0x18;   // 0bxxx11xxx
    
    printf("Gyro Accleration Config set point: %X\n",data);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG,data);
    data = 0;
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG2,Buffer);
    printf("Gyro Accleration Config: %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0xf0;  // 0b11110000
    data |= 0x07;
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG2, data);
    data = 0;
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG2,Buffer);
    printf("Gyro Accleration Config 2: %X\n",Buffer[0]);
    data = Buffer[0];
    data &= 0xcf; // & 0b11001111
    data |= 0x00;
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_ACCEL_CONFIG2,data);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_XG_OFFS_USRH,0xEE);//EE5c
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_XG_OFFS_USRL,0x50);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_YG_OFFS_USRH,0xFF);//FFF5
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_YG_OFFS_USRL,0xF5);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_ZG_OFFS_USRH,0x00);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_ZG_OFFS_USRL,0x10);//0020
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_XA_OFFSET_H,0x03);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_XA_OFFSET_L,0x03);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_YA_OFFSET_H,0xFF);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_YA_OFFSET_L,0x00);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_ZA_OFFSET_H,0xFF);
    ret = i2c_reg_write_byte_dt(&dev_i2c,ICM20600_ZA_OFFSET_L,0x00);

    // To the offset, take the negative hexdecimal as a decimal: 0x4500 = 
}

int32_t getRawAccelerationX() {
    int8_t Buffer[16] = {0};
    int8_t buf[2] = {0};
    int ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_XOUT_H,Buffer);
    buf[0] = Buffer[0];
    //ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_XOUT_L,Buffer);
   // buf[1] = Buffer[0];
    int32_t raw_data = ((int16_t)buf[0] << 8) + buf[1];
    //printf("X: %x,",raw_data);
    raw_data = (raw_data * (32000)) >> 16;
    return (int16_t)raw_data;
}

int32_t getRawAccelerationY() {
    int8_t Buffer[16] = {0};
    int8_t buf[2] = {0};
    int ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_YOUT_H,Buffer);
    buf[0] = Buffer[0];
    //ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_YOUT_L,Buffer);
   //buf[1] = Buffer[0];
    int32_t raw_data = ((int16_t)buf[0] << 8) + buf[1];
    //printf("Y:  %x\n",raw_data);
    raw_data = (raw_data * (32000)) >> 16;
    return raw_data;
}

int32_t getRawAccelerationZ() {
    int8_t Buffer[16] = {0};
    int8_t buf[2] = {0};
    int ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_ZOUT_H,Buffer);
    buf[0] = Buffer[0];
    //ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_ACCEL_ZOUT_L,Buffer);
    //buf[1] = Buffer[0];
    int32_t raw_data = ((int16_t)buf[0] << 8) + buf[1];
    //printf(",Z:  %x\n",raw_data);
    raw_data = (raw_data * (32000)) >> 16;
    return raw_data;
}


int32_t getRawGyroscopeX() {
    int8_t Buffer[16] = {0};
    int8_t buf[2] = {0};
    int ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_XOUT_H,Buffer);
    buf[0] = Buffer[0];
    //ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_XOUT_L,Buffer);
   // buf[1] = Buffer[0];
    int32_t raw_data = ((int32_t)buf[0] << 8) + buf[1];
    //printf("Gyro_x: %x\n",raw_data);
    raw_data = (raw_data * (4000)) >> 16;
    return raw_data;
}

int32_t getRawGyroscopeY() {
    int8_t Buffer[16] = {0};
    int8_t buf[2] = {0};
    int ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_YOUT_H,Buffer);
    buf[0] = Buffer[0];
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_YOUT_L,Buffer);
    buf[1] = Buffer[0];
    int32_t raw_data = ((int32_t)buf[0] << 8) + buf[1];
    //printf("Gyro_y: %x\n",raw_data);
    raw_data = (raw_data *(4000)) >> 16;
    return raw_data;
}

int32_t getRawGyroscopeZ() {
    int8_t Buffer[16] = {0};
    int8_t buf[2] = {0};
    int ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_ZOUT_H,Buffer);
    buf[0] = Buffer[0];
    ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_GYRO_ZOUT_L,Buffer);
    buf[1] = Buffer[0];
    int32_t raw_data = ((int32_t)buf[0] << 8) + buf[1];
    //printf("Gyro_z: %x\n",raw_data);
    raw_data = (raw_data * (4000)) >> 16;
    return raw_data;
}

int32_t getTemperature(void) {
    uint16_t rawdata;
    uint8_t Buffer[16] = {0};
    int ret = i2c_reg_read_byte_dt(&dev_i2c,ICM20600_TEMP_OUT_H,Buffer);
    rawdata = (((uint16_t)Buffer[0]) << 8) + Buffer[1];
    return (int16_t)(rawdata / 327 + 25);
}





void SimpleComplementaryFilter(double a_x, double a_y, double a_z,double w_x,double w_y,double w_z,double dt)
{
    printf("a_x:%f,a_y:%f,a_z:%f,w_x:%f,w_y:%f,w_z:%f,",a_x,a_y,a_z,w_x,w_y,w_z);
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    float f_1, f_2, f_3; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;
    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    // Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
    // Normalise the gradient
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;
    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // Compute then integrate the estimated quaternion derrivative
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;
    // Normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
    theta = -asin(2*SEq_2*SEq_4+2*SEq_2*SEq_4)-0.3;
    if(isnan(theta))
    {
        theta = 0;
    }
    phi = atan2(2*SEq_3*SEq_4-2*SEq_1*SEq_2,2*SEq_1*SEq_1+2*SEq_4*SEq_4-1)+1.61;
    if(isnan(phi))
    {
        phi = 0;
    }
    psi = atan2(2*SEq_2*SEq_3-2*SEq_1*SEq_4,2*SEq_1*SEq_1+2*SEq_2*SEq_2-1)+0.07;
    if(isnan(psi))
    {
        psi = 0;
    }
    printf("%f,%f,%f\n",theta,phi,psi);
    return theta, phi,psi;
}
#endif

/*int32_t Compute_full_Sensor_Fusion(double a_x, double a_y, double a_z,double w_x,double w_y,double w_z,double dt)
{
    printf("a_x:%f,a_y:%f,a_z:%f,w_x:%f,w_y:%f,w_z:%f",a_x,a_y,a_z,w_x,w_y,w_z);
    // Normalise the accelerometer measurement
    double norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x = a_x/norm;
    a_y = a_y/norm;
    a_z = a_z/norm;
    //printf("a_x:%f,a_y:%f,a_z:%f,w_x:%f,w_y:%f,w_z:%f\n",a_x,a_y,a_z,w_x,w_y,w_z);
    double S_w_t[4] = {0,w_x,w_y,w_z}; // Set up the angular rates from gyro as 1D array 
    double s_a_t[4] = {0,a_x,a_y,a_z}; // Set up the accelerations as 1D array
    // Tensor Product between orientation quat and angular rate 
    //printf("S_w_t[0]: %f,S_w_t[1]: %f,S_w_t[2]: %f,S_w_t[3]: %f\n",S_w_t[0],S_w_t[1],S_w_t[2],S_w_t[3]);
    //printf("S_a_t[0]: %f,S_a_t[1]: %f,S_a_t[2]: %f,S_a_t[3]: %f\n",s_a_t[0],s_a_t[1],s_a_t[2],s_a_t[3]);
    double SE_qdot_1 = -SE_q[0]*S_w_t[0] - SE_q[1]*S_w_t[1] - SE_q[2]*S_w_t[2] - SE_q[3]*S_w_t[3]; 
    double SE_qdot_2 = SE_q[0]*S_w_t[1] + SE_q[1]*S_w_t[0] + SE_q[2]*S_w_t[3] - SE_q[3]*S_w_t[2]; 
    double SE_qdot_3 = SE_q[0]*S_w_t[2] - SE_q[1]*S_w_t[3] + SE_q[2]*S_w_t[0] + SE_q[3]*S_w_t[1]; 
    double SE_qdot_4 = SE_q[0]*S_w_t[3] + SE_q[1]*S_w_t[2] - SE_q[2]*S_w_t[1] + SE_q[3]*S_w_t[0]; 
    double SE_qdot[4] = {0.5*SE_qdot_1,0.5*SE_qdot_2,0.5*SE_qdot_3,0.5*SE_qdot_4};
    //printf("SE_qdot:1 - %f,2-%f,3-%f,4-%f\n",SE_qdot_1,SE_qdot_2,SE_qdot_3,SE_qdot_4);
    //printf("SE_q:1 - %f,2-%f,3-%f,4-%f\n",SE_q[0],SE_q[1],SE_q[2],SE_q[3]);
    double function_g_1 = 2*(SE_q[1]*SE_q[3]-SE_q[0]*SE_q[2]) - a_x;
    double function_g_2 = 2*(SE_q[0]*SE_q[1]+SE_q[2]*SE_q[3]) - a_y;
    double function_g_3 = 2*(0.5-SE_q[1]*SE_q[1]-SE_q[2]*SE_q[2]) - a_z;
    double function_g[3] = {function_g_1,function_g_2,function_g_3};
    double Jacobian_T[4][3] = {{-2*SE_q[2],2*SE_q[1],0},{2*SE_q[3],2*SE_q[0],-4*SE_q[1]},{-2*SE_q[0],2*SE_q[3],-4*SE_q[3]},{2*SE_q[1],2*SE_q[2],0}}; // [No of Rows][No of Columns]
    double J_fg[4] = {Jacobian_T[0][0]*function_g[0]+Jacobian_T[0][1]*function_g[1]+Jacobian_T[0][2]*function_g[2],Jacobian_T[1][0]*function_g[0]+Jacobian_T[1][1]*function_g[1]+Jacobian_T[1][2]*function_g[2],Jacobian_T[2][0]*function_g[0]+Jacobian_T[2][1]*function_g[1]+Jacobian_T[2][2]*function_g[2],Jacobian_T[3][0]*function_g[0]+Jacobian_T[3][1]*function_g[1]+Jacobian_T[3][2]*function_g[2]};
    //printf("function_g:1 - %f,2-%f,3-%f\n",function_g_1,function_g_2,function_g_3);
    //printf("Jacobian_T1:1 - %f,2-%f,3-%f\n",Jacobian_T[0][0],Jacobian_T[0][1],Jacobian_T[0][2]);
    //printf("Jacobian_T3:1 - %f,2-%f,3-%f\n",Jacobian_T[2][0],Jacobian_T[2][1],Jacobian_T[2][2]);
    //printf("Jacobian_T4:1 - %f,2-%f,3-%f\n",Jacobian_T[3][0],Jacobian_T[3][1],Jacobian_T[3][2]);
    //printf("J_fg:1 - %f,2-%f,3-%f,4-%f\n",J_fg[0],J_fg[1],J_fg[2],J_fg[3]);
    for(int i =0;i<4;i++)
    {
        J_fg[i] = J_fg[i]/sqrt(J_fg[0]*J_fg[0]+J_fg[1]*J_fg[1]+J_fg[2]*J_fg[2]+J_fg[3]*J_fg[3]);
        SE_q[i] = SE_q[i] + (SE_qdot[i] - (beta)*J_fg[i])*dt;
        SE_q[i] = SE_q[i]/sqrt(SE_q[0]*SE_q[0]+SE_q[1]*SE_q[1]+SE_q[2]*SE_q[2]+SE_q[3]*SE_q[3]);
    }
    //printf("SE_q:1 - %f,2-%f,3-%f,4-%f\n",SE_q[0],SE_q[1],SE_q[2],SE_q[3]);

    theta = -asin(2*SE_q[1]*SE_q[3]+2*SE_q[1]*SE_q[3])+0.38945;
    if(isnan(theta))
    {
        theta = 0;
    }
    phi = atan2(2*SE_q[2]*SE_q[3]-2*SE_q[0]*SE_q[1],2*SE_q[0]*SE_q[0]+2*SE_q[3]*SE_q[3]-1)-1.742986;
    if(isnan(phi))
    {
        phi = 0;
    }
    psi = atan2(2*SE_q[1]*SE_q[2]-2*SE_q[0]*SE_q[3],2*SE_q[0]*SE_q[0]+2*SE_q[1]*SE_q[1]-1)-1.575367;
    if(isnan(psi))
    {
        psi = 0;
    }
    //printf(",%f,%f,%f\n",theta,phi,psi);
    return theta, psi, phi;
}

int32_t VariableGravityFrame_Compute_full_Sensor_Fusion(double a_x, double a_y, double a_z,double w_x,double w_y,double w_z,double dt)
{
    printf("a_x:%f,a_y:%f,a_z:%f,w_x:%f,w_y:%f,w_z:%f\n",a_x,a_y,a_z,w_x,w_y,w_z);
    // Normalise the accelerometer measurement
    double norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x = a_x/norm;
    a_y = a_y/norm;
    a_z = a_z/norm;
    //printf("a_x:%f,a_y:%f,a_z:%f,w_x:%f,w_y:%f,w_z:%f\n",a_x,a_y,a_z,w_x,w_y,w_z);
    double S_w_t[4] = {0,w_x,w_y,w_z}; // Set up the angular rates from gyro as 1D array 
    double s_a_t[4] = {0,a_x,a_y,a_z}; // Set up the accelerations as 1D array
    // Tensor Product between orientation quat and angular rate 
    //printf("S_w_t[0]: %f,S_w_t[1]: %f,S_w_t[2]: %f,S_w_t[3]: %f\n",S_w_t[0],S_w_t[1],S_w_t[2],S_w_t[3]);
    //printf("S_a_t[0]: %f,S_a_t[1]: %f,S_a_t[2]: %f,S_a_t[3]: %f\n",s_a_t[0],s_a_t[1],s_a_t[2],s_a_t[3]);
    double SE_qdot_1 = SE_q[0]*S_w_t[0] - SE_q[1]*S_w_t[1] - SE_q[2]*S_w_t[2] - SE_q[3]*S_w_t[3]; 
    double SE_qdot_2 = SE_q[0]*S_w_t[1] + SE_q[1]*S_w_t[0] + SE_q[2]*S_w_t[3] - SE_q[3]*S_w_t[2]; 
    double SE_qdot_3 = SE_q[0]*S_w_t[2] - SE_q[1]*S_w_t[3] + SE_q[2]*S_w_t[0] + SE_q[3]*S_w_t[1]; 
    double SE_qdot_4 = SE_q[0]*S_w_t[3] + SE_q[1]*S_w_t[2] - SE_q[2]*S_w_t[1] + SE_q[3]*S_w_t[0]; 
    double SE_qdot[4] = {0.5*SE_qdot_1,0.5*SE_qdot_2,0.5*SE_qdot_3,0.5*SE_qdot_4};
    //printf("SE_qdot:1 - %f,2-%f,3-%f,4-%f\n",SE_qdot_1,SE_qdot_2,SE_qdot_3,SE_qdot_4);
    //printf("SE_q:1 - %f,2-%f,3-%f,4-%f\n",SE_q[0],SE_q[1],SE_q[2],SE_q[3]);
    double d[4] = {0,0,0,1};
    double function_g_1 = 2*d[1]*(0.5-SE_q[2]*SE_q[2]-SE_q[3]*SE_q[3])+2*d[2]*(SE_q[0]*SE_q[3]+SE_q[1]*SE_q[2])+2*d[3]*(SE_q[1]*SE_q[3]-SE_q[0]*SE_q[2]) - a_x;
    double function_g_2 = 2*d[1]*(SE_q[1]*SE_q[2]-SE_q[0]*SE_q[3])+2*d[2]*(0.5-SE_q[1]*SE_q[1]-SE_q[3]*SE_q[3])+2*d[3]*(SE_q[0]*SE_q[1]+SE_q[2]*SE_q[3]) - a_y;
    double function_g_3 = 2*d[1]*(SE_q[0]*SE_q[2]+SE_q[1]*SE_q[3])+2*d[2]*(SE_q[2]*SE_q[3]-SE_q[0]*SE_q[1])+2*d[3]*(0.5-SE_q[1]*SE_q[1]-SE_q[2]*SE_q[2]) - a_z;
    double function_g[3] = {function_g_1,function_g_2,function_g_3};
    double Jacobian_T_1[3] = {2*d[2]*SE_q[3]-2*d[3]*SE_q[2],2*d[1]*SE_q[3]+2*d[3]*SE_q[1],2*d[1]*SE_q[2]-2*d[2]*SE_q[1]};
    double Jacobian_T_2[3] = {2*d[2]*SE_q[2]+2*d[3]*SE_q[3],2*d[1]*SE_q[2]-4*d[2]*SE_q[1]+2*d[3]*SE_q[0],2*d[1]*SE_q[3]-2*d[2]*SE_q[0]-4*d[3]*SE_q[1]};
    double Jacobian_T_3[3] = {-4*d[1]*SE_q[2]+2*d[2]*SE_q[1]-2*d[3]*SE_q[0],2*d[1]*SE_q[1]+2*d[3]*SE_q[3],2*d[1]*SE_q[0]+2*d[2]*SE_q[3]-4*d[3]*SE_q[2]};
    double Jacobian_T_4[3] = {-4*d[1]*SE_q[3]+2*d[2]*SE_q[0]+2*d[3]*SE_q[1],-2*d[1]*SE_q[0]-4*d[2]*SE_q[3]+2*d[3]*SE_q[2],2*d[1]*SE_q[1]+2*d[2]*SE_q[2]};
    double J_fg[4] = {Jacobian_T_1[0]*function_g_1+Jacobian_T_1[1]*function_g_2+Jacobian_T_1[2]*function_g_3,Jacobian_T_2[0]*function_g_1+Jacobian_T_2[1]*function_g_2+Jacobian_T_2[2]*function_g_3,Jacobian_T_3[0]*function_g_1+Jacobian_T_3[1]*function_g_2+Jacobian_T_3[2]*function_g_3,Jacobian_T_4[0]*function_g_1+Jacobian_T_4[1]*function_g_2+Jacobian_T_4[2]*function_g_3};
    //printf("function_g:1 - %f,2-%f,3-%f\n",function_g_1,function_g_2,function_g_3);
    //printf("Jacobian_T1:1 - %f,2-%f,3-%f\n",Jacobian_T[0][0],Jacobian_T[0][1],Jacobian_T[0][2]);
    //printf("Jacobian_T3:1 - %f,2-%f,3-%f\n",Jacobian_T[2][0],Jacobian_T[2][1],Jacobian_T[2][2]);
    //printf("Jacobian_T4:1 - %f,2-%f,3-%f\n",Jacobian_T[3][0],Jacobian_T[3][1],Jacobian_T[3][2]);
    //printf("J_fg:1 - %f,2-%f,3-%f,4-%f\n",J_fg[0],J_fg[1],J_fg[2],J_fg[3]);
    for(int i =0;i<4;i++)
    {
        J_fg[i] = J_fg[i]/sqrt(J_fg[0]*J_fg[0]+J_fg[1]*J_fg[1]+J_fg[2]*J_fg[2]+J_fg[3]*J_fg[3]);
        SE_q[i] = SE_q[i] + (SE_qdot[i] - (beta)*J_fg[i])*dt;
        SE_q[i] = SE_q[i]/sqrt(SE_q[0]*SE_q[0]+SE_q[1]*SE_q[1]+SE_q[2]*SE_q[2]+SE_q[3]*SE_q[3]);
    }
    //printf("SE_q:1 - %f,2-%f,3-%f,4-%f\n",SE_q[0],SE_q[1],SE_q[2],SE_q[3]);
    //printf("%f,%f,%f,%f,",SE_q[0],SE_q[1],SE_q[2],SE_q[3]);
    theta = -asin(2*SE_q[1]*SE_q[3]+2*SE_q[1]*SE_q[3])-0.279253;
    if(isnan(theta))
    {
        theta = 0;
    }
    phi = atan2(2*SE_q[2]*SE_q[3]-2*SE_q[0]*SE_q[1],2*SE_q[0]*SE_q[0]+2*SE_q[3]*SE_q[3]-1);
    if(isnan(phi))
    {
        phi = 0;
    }
    psi = atan2(2*SE_q[1]*SE_q[2]-2*SE_q[0]*SE_q[3],2*SE_q[0]*SE_q[0]+2*SE_q[1]*SE_q[1]-1)+1.521821;
    if(isnan(psi))
    {
        psi = 0;
    }
    //printf("%f,%f,%f,",theta,phi,psi);
    return theta, psi, phi;
}
#endif

void SimpleAngleCal(double a_x, double a_y, double a_z,double w_x,double w_y,double w_z,double dt)
{
    //printf("a_x:%f,a_y:%f,a_z:%f,w_x:%f,w_y:%f,w_z:%f,",a_x,a_y,a_z,w_x,w_y,w_z);
    theta = asin(a_x/g)-1.066054;
    phi = atan(a_y/g);
    //printf("%f,%f,%f\n",theta,phi,psi);
}
*/