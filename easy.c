/*                                                                                                                                              *
 * The MPU6050 sensor is a three-axis gyroscope and a three-axis accelerometer. It has 16 bit data output. Communication is provided using I2C. *
 * The value of the axes is regulated by the Kalman filter.                                                                                     *
 * Datasheet for MPU6080: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf                                      *
 *                                                                                                                                              */

#include <math.h> //Library with math functions
#include "mpu6050.h" //Required library for mpu6050 sensor

#define RAD2DEG 57.295779513082320876798154814105 //Rad to degree conv.
#define WHOAMI 0x75 //Register address of the chip to be read
#define PWR_MGMT 0x6B //Power management register address
#define SMPLRT_DIV 0x19 //Sample rate divider register address 
#define ACCEL_CONFIG 0x1C // Accel config register address
#define ACCEL_XOUT_H 0x3B // Register address for x-axis accelerometer
#define TEMP_OUT_H 0x41 //High byte register temperature
#define GYRO_CONFIG 0x1B //Gyro config registeraddress
#define GYRO_XOUT_H 0x43 // Register address for x-axis gyroscope
#define MPU6050_ADDR 0xD0 //MPU6050 chip address
const uint16_t I2C_TIMEOUT = 100; //Value defined as const in data type uint16. I2C connection timeout (ms)
const double Acc_Z_corrector = 14418.0; // z-axis accelerometer correction value
uint32_t timer;

//Kalman filter structure for X
Filter_t FilterX = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f};

//Kalman filter structure for Y 
Filter_t FilterY = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f,
};

/// @brief MPU6050 starts
/// @param I2Cx Takes the I2C pointer as a parameter
/// @return Returns uint8 type value

uint8_t MPU_Init(I2C_HandleTypeDef *I2Cx) 
{
    uint8_t check; //The memory address that holds the data to be read. for WHOAMI
    uint8_t Data; //Memory address holding data to be written

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHOAMI, 1, &check, 1, I2C_TIMEOUT); //STM32 HAL I2C read function. Used to read WHOAMI register

    if (check == 104) //104 = 0x68 Value returned by the sensor
    {   
        //In this function, pwr_mgmt, smplrt_div, accel_config and gyro_config settings are made 
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT, 1, &Data, 1, I2C_TIMEOUT);

        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, I2C_TIMEOUT);

        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, I2C_TIMEOUT);

        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, I2C_TIMEOUT);
        return 0;
    }
    return 1;
} 

/// @brief Reading function for accelerometer.
/// @param I2Cx Takes the I2C pointer as a parameter.
/// @param DataStruct Retrieves the Datastruct pointer of the sensor.

void Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];  // Buffer size 6 bytes

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT); //STM32 HAL I2C read function. Takes the accelerometer reading from the sensor
    
    //Bit shift operation
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    
    //The x, y, z values taken from the accelerometer are arranged. Z value is made according to the defined correction value.
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector;
}

/// @brief Reading function for gyroscope.
/// @param I2Cx Takes the I2C pointer as a parameter
/// @param DataStruct Retrieves the Datastruct pointer of the sensor.

void Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6]; // Buffer size 6 bytes

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT); //STM32 HAL I2C read function. Takes the gyroscope reading from the sensor
    
    //Bit shift operation
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    
    //The x, y, z values taken from the gyroscope are arranged.
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

/// @brief Temperature reading function.
/// @param I2Cx Takes the I2C pointer as a parameter
/// @param DataStruct Retrieves the Datastruct pointer of the sensor.

void Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2]; // Buffer size 2 bytes
    int16_t temp;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H, 1, Rec_Data, 2, I2C_TIMEOUT); //STM32 HAL I2C read function. Takes the temperature reading from the sensor
    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53); //The temperature value is regulated
}

/// @brief All values received from the sensor are read
/// @param I2Cx Takes the I2C pointer as a parameter
/// @param DataStruct Retrieves the Datastruct pointer of the sensor.

void Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14]; // Buffer size 14 bytes
    int16_t temp;
    
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 14, I2C_TIMEOUT); //STM32 HAL I2C read function. 14 bytes of x-axis data are read from accelerometer
    
    //Bit shift operation for accelerometer x, y, z
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    //Bit shift operation for temperature
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);

    //Bit shift operation for gyroscope x, y, z
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    //Values are edited
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector;

    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
    
    //The timer is started
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    
    double roll; //A variable of type double is defined for the roll axis
    //The roll axis is calculated. Rad to deg conversion is done
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD2DEG;
    }
    else
    {
        roll = 0.0;
    }
    //Pitch axis is calculated
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD2DEG;
    if ((pitch < -90 && DataStruct->FilterAngleY > 90) || (pitch > 90 && DataStruct->FilterAngleY < -90))
    {
        FilterY.angle = pitch;
        DataStruct->FilterAngleY = pitch;
    }
    else
    {
        DataStruct->FilterAngleY = Filter_getAngle(&FilterY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->FilterAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->FilterAngleX = Filter_getAngle(&FilterX, roll, DataStruct->Gx, dt);
}

/// @brief The Kalman Filter function works
/// @param Filter Gets the filter pointer
/// @param newAngle Angle value in double
/// @param newRate Rate value in double
/// @param dt timer
/// @return Returns the angles in double type

double Filter_getAngle(Filter_t *Filter, double newAngle, double newRate, double dt)
{
    double rate = newRate - Filter->bias;
    Filter->angle += dt * rate;

    Filter->P[0][0] += dt * (dt * Filter->P[1][1] - Filter->P[0][1] - Filter->P[1][0] + Filter->Q_ANGLE);
    Filter->P[0][1] -= dt * Filter->P[1][1];
    Filter->P[1][0] -= dt * Filter->P[1][1];
    Filter->P[1][1] += Filter->Q_BIAS * dt;

    double S = Filter->P[0][0] + Filter->R_MEASURE;
    double K[2];
    K[0] = Filter->P[0][0] / S;
    K[1] = Filter->P[1][0] / S;

    double y = newAngle - Filter->angle;
    Filter->angle += K[0] * y;
    Filter->bias += K[1] * y;

    double P00_temp = Filter->P[0][0];
    double P01_temp = Filter->P[0][1];

    Filter->P[0][0] -= K[0] * P00_temp;
    Filter->P[0][1] -= K[0] * P01_temp;
    Filter->P[1][0] -= K[1] * P00_temp;
    Filter->P[1][1] -= K[1] * P01_temp;

    return Filter->angle;
};