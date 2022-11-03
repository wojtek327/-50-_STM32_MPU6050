#include "stm32_mpu6050.h"

extern I2C_HandleTypeDef hi2c1;

static HAL_StatusTypeDef MPU6050_WriteRegisterBits(uint8_t reg, uint8_t data, unsigned pos, unsigned len);
static uint8_t MPU6050_ReadRegBits(uint8_t reg, unsigned pos, unsigned len);
static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t address, uint8_t cmd);
static HAL_StatusTypeDef MPU6050_ReadRegister(uint8_t address, uint8_t dataSize, uint8_t *rec);

uint8_t MPU6050_Init_Default(void) {
	HAL_StatusTypeDef opStatSelectClock = MPU6050_SelectClockSource(MPU6050_CLOCK_PLL_XGYRO);
	if(opStatSelectClock != HAL_OK) { return 1; }

	HAL_StatusTypeDef opStatSetFullScaleGyro = MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
	if(opStatSetFullScaleGyro != HAL_OK) { return 2; }

	HAL_StatusTypeDef opStatSetFullScaleaccel = MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	if(opStatSetFullScaleaccel != HAL_OK) { return 3; }

	HAL_StatusTypeDef opStatSetSleepEnabled = MPU6050_SetSleepEnabled(0);
	if(opStatSetSleepEnabled != HAL_OK) { return 4; }

	return 0;
}

uint8_t MPU6050_InitCustomConf(const enum mpu6050_ClockSource_e clockSource, const enum gyro_FullScale_e gyroRange, const enum accel_FullScale_e accelRange) {
	HAL_StatusTypeDef opStatSelectClock = MPU6050_SelectClockSource((uint8_t)clockSource);
	if(opStatSelectClock != HAL_OK) { return 1; }

	HAL_StatusTypeDef opStatSetFullScaleGyro = MPU6050_SetFullScaleGyroRange((uint8_t)gyroRange);
	if(opStatSetFullScaleGyro != HAL_OK) { return 2; }

	HAL_StatusTypeDef opStatSetFullScaleaccel = MPU6050_SetFullScaleAccelRange((uint8_t)accelRange);
	if(opStatSetFullScaleaccel != HAL_OK) { return 3; }

	HAL_StatusTypeDef opStatSetSleepEnabled = MPU6050_SetSleepEnabled(0);
	if(opStatSetSleepEnabled != HAL_OK) { return 4; }

	return 0;
}

float MPU6050_RetAccelScalingFactor(uint8_t accelFullScale)
{
	if(accelFullScale == AFS_SEL_2g) 		{ return (2000.0f/32768.0f); }
	else if(accelFullScale == AFS_SEL_4g)	{ return (4000.0f/32768.0f); }
	else if(accelFullScale == AFS_SEL_8g) 	{ return (8000.0f/32768.0f); }
	else if(accelFullScale == AFS_SEL_16g)  { return (16000.0f/32768.0f); }
	return 0;
}

float MPU6050_RetGyroScalingFactor(uint8_t gyroFullScale)
{
	if(gyroFullScale == FS_SEL_250) 		{ return (250.0f/32768.0f); }
	else if(gyroFullScale == FS_SEL_500) 	{ return (500.0f/32768.0f); }
	else if(gyroFullScale == FS_SEL_1000) 	{ return (1000.0f/32768.0f); }
	else if(gyroFullScale == FS_SEL_2000) 	{ return (2000.0f/32768.0f); }
	return 0;
}


uint8_t MPU6050_ReadAndCheckDeviceId(void)
{
    uint8_t deviceId = MPU6050_ReadDeviceID();

    if((deviceId == 0x34) || (deviceId == 0xC)) {
    	return 1;
    }
    return 0;
}

uint8_t MPU6050_GetAuxVDDIOLevel(void) {
	return MPU6050_ReadRegBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, 1);
}

HAL_StatusTypeDef MPU6050_SetAuxVDDIOLevel(const uint8_t level) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_YG_OFFS_TC, level, MPU6050_TC_PWR_MODE_BIT, 1);
}

uint8_t MPU6050_GetRate(void) {
	uint8_t readData = 0;
	return MPU6050_ReadRegister(MPU6050_RA_SMPLRT_DIV, 1, &readData);
}

HAL_StatusTypeDef MPU6050_SetRate(const uint8_t rate) {
	return MPU6050_WriteRegister(MPU6050_RA_SMPLRT_DIV, rate);
}

HAL_StatusTypeDef MPU6050_SelectClockSource(const uint8_t source) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_PWR_MGMT_1, source, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH);
}

uint8_t MPU6050_GetFullScaleGyroRange(void) {
	return MPU6050_ReadRegBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH);
}

HAL_StatusTypeDef MPU6050_SetFullScaleGyroRange(const uint8_t range) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_GYRO_CONFIG, range, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH);
}

HAL_StatusTypeDef MPU6050_SetFullScaleAccelRange(uint8_t range) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_ACCEL_CONFIG, range, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH);
}

HAL_StatusTypeDef MPU6050_SetSleepEnabled(uint8_t enabled) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_PWR_MGMT_1, enabled, MPU6050_PWR1_SLEEP_BIT, 1);
}

/*
 * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
 * between sleep mode and waking up to take a single sample of data from active
 * sensors at a rate determined by LP_WAKE_CTRL (register 108).
 */
uint8_t MPU6050_GetWakeCycleEnabled(void) {
	return MPU6050_ReadRegBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, 1);
}

HAL_StatusTypeDef MPU6050_SetWakeCycleEnabled(const uint8_t enabled) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_PWR_MGMT_1, enabled, MPU6050_PWR1_CYCLE_BIT, 1);
}

/*
* Note: this register stores the *disabled* value, but for consistency with the
* rest of the code, the function is named and used with standard true/false
* values to indicate whether the sensor is enabled or disabled, respectively.
* 1 means that sensor is disable
*/
uint8_t MPU6050_GetTempSensorEnabled(void) {
	return MPU6050_ReadRegBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, 1);
}

/*
	Value 1	disable temperature sensor.
*/
HAL_StatusTypeDef MPU6050_SetTempSensorEnabled(const uint8_t enabled) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_PWR_MGMT_1, enabled, MPU6050_PWR1_TEMP_DIS_BIT, 1);
}

HAL_StatusTypeDef MPU6050_DeviceReset(void) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_PWR_MGMT_1, 1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

uint8_t MPU6050_ReadDeviceID(void) {
	return MPU6050_ReadRegBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH);
}

HAL_StatusTypeDef MPU6050_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	uint8_t buffer[14] = {0x00};

	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(MPU6050_RA_ACCEL_XOUT_H, 14, &buffer[0]);

	if(opStatus != HAL_OK) { return opStatus; }

    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];

    return opStatus;
}

HAL_StatusTypeDef MPU6050_GetAcceleration(int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t buffer[6] = {0x00};

	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(MPU6050_RA_ACCEL_XOUT_H, 6, &buffer[0]);

	if(opStatus != HAL_OK) { return opStatus; }

    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];

    return opStatus;
}

int16_t MPU6050_GetAccelerationX(void) {
	uint8_t buffer[2] = {0x00};

	MPU6050_ReadRegister(MPU6050_RA_ACCEL_XOUT_H, 2, &buffer[0]);

    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t MPU6050_GetAccelerationY(void) {
	uint8_t buffer[2] = {0x00};

	MPU6050_ReadRegister(MPU6050_RA_ACCEL_YOUT_H, 2, &buffer[0]);

    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t MPU6050_GetAccelerationZ(void) {
	uint8_t buffer[2] = {0x00};

	MPU6050_ReadRegister(MPU6050_RA_ACCEL_ZOUT_H, 2, &buffer[0]);

    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t MPU6050_GetRawTemperature(void) {
	uint8_t buffer[2] = {0x00};

	MPU6050_ReadRegister(MPU6050_RA_TEMP_OUT_H, 2, &buffer[0]);

    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

float MPU6050_ConvertTemperature(int16_t readedTempVal)
{
	return (float)(readedTempVal/340.0)+36.53;
}

HAL_StatusTypeDef MPU6050_GetRotation(int16_t* x, int16_t* y, int16_t* z) {
	uint8_t buffer[6] = {0x00};

	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(MPU6050_RA_GYRO_XOUT_H, 6, &buffer[0]);

	if(opStatus != HAL_OK) { return opStatus; }

    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];

    return opStatus;
}

int16_t MPU6050_GetRotationX(void) {
	uint8_t buffer[2] = {0x00};

	MPU6050_ReadRegister(MPU6050_RA_GYRO_XOUT_H, 2, &buffer[0]);

    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t MPU6050_GetRotationY(void) {
	uint8_t buffer[2] = {0x00};

	MPU6050_ReadRegister(MPU6050_RA_GYRO_YOUT_H, 2, &buffer[0]);

    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t MPU6050_GetRotationZ(void) {
	uint8_t buffer[2] = {0x00};

	MPU6050_ReadRegister(MPU6050_RA_GYRO_ZOUT_H, 2, &buffer[0]);

    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

uint8_t MPU6050_GetExternalSensorByte(int position) {
	uint8_t readByte = 0;

	MPU6050_ReadRegister(MPU6050_RA_EXT_SENS_DATA_00 + position, 1, &readByte);

    return readByte;
}

uint16_t MPU6050_GetExternalSensorWord(const int position) {
	uint8_t buffer[2] = {0x00};

	MPU6050_ReadRegister(MPU6050_RA_EXT_SENS_DATA_00 + position, 2, &buffer[0]);

    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

uint32_t MPU6050_GetExternalSensorDWord(const int position) {
	uint8_t buffer[2] = {0x00};

	MPU6050_ReadRegister(MPU6050_RA_EXT_SENS_DATA_00 + position, 4, &buffer[0]);

    return (((uint32_t)buffer[0]) << 24) | (((uint32_t)buffer[1]) << 16) | (((uint16_t)buffer[2]) << 8) | buffer[3];
}

uint8_t MPU6050_GetMotionStatus(void) {
	uint8_t readByte = 0;

	MPU6050_ReadRegister(MPU6050_RA_MOT_DETECT_STATUS, 1, &readByte);

    return readByte;
}

uint8_t MPU6050_GetAccelXSelfTestFactoryTrim(void) {
	uint8_t buffer[2] = {0x00};

	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(MPU6050_RA_SELF_TEST_X, 1, &buffer[0]);
	if(opStatus != HAL_OK) { return 0xFF; }
	opStatus = MPU6050_ReadRegister(MPU6050_RA_SELF_TEST_A, 1, &buffer[0]);
	if(opStatus != HAL_OK) { return 0xFF; }

	return (buffer[0]>>3) | ((buffer[1]>>4) & 0x03);
}

uint8_t MPU6050_GetAccelYSelfTestFactoryTrim(void) {
	uint8_t buffer[2] = {0x00};

	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(MPU6050_RA_SELF_TEST_Y, 1, &buffer[0]);
	if(opStatus != HAL_OK) { return 0xFF; }
	opStatus = MPU6050_ReadRegister(MPU6050_RA_SELF_TEST_A, 1, &buffer[0]);
	if(opStatus != HAL_OK) { return 0xFF; }

	return (buffer[0]>>3) | ((buffer[1]>>2) & 0x03);
}

uint8_t MPU6050_GetAccelZSelfTestFactoryTrim(void) {
	uint8_t buffer[2] = {0x00};

	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(MPU6050_RA_SELF_TEST_Z, 2, &buffer[0]);
	if(opStatus != HAL_OK) { return 0xFF; }

	return (buffer[0]>>3) | (buffer[1] & 0x03);
}

uint8_t MPU6050_GetGyroXSelfTestFactoryTrim(void) {
	uint8_t readByte = 0;

	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(MPU6050_RA_SELF_TEST_X, 1, &readByte);
	if(opStatus != HAL_OK) { return 0xFF; }

    return (readByte & 0x1F);
}

uint8_t MPU6050_GetGyroYSelfTestFactoryTrim(void) {
	uint8_t readByte = 0;

	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(MPU6050_RA_SELF_TEST_Y, 1, &readByte);
	if(opStatus != HAL_OK) { return 0xFF; }

    return (readByte & 0x1F);
}

uint8_t MPU6050_GetGyroZSelfTestFactoryTrim(void) {
	uint8_t readByte = 0;

	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(MPU6050_RA_SELF_TEST_Z, 1, &readByte);
	if(opStatus != HAL_OK) { return 0xFF; }

    return (readByte & 0x1F);
}

uint8_t MPU6050_GetAccelXSelfTest(void) {
	return MPU6050_ReadRegBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, 1);
}

HAL_StatusTypeDef MPU6050_SetAccelXSelfTest(const uint8_t enabled) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_ACCEL_CONFIG, enabled, MPU6050_ACONFIG_XA_ST_BIT, 1);
}

uint8_t MPU6050_GetAccelYSelfTest(void) {
	return MPU6050_ReadRegBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, 1);
}

HAL_StatusTypeDef MPU6050_SetAccelYSelfTest(const uint8_t enabled) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_ACCEL_CONFIG, enabled, MPU6050_ACONFIG_YA_ST_BIT, 1);
}

uint8_t MPU6050_GetAccelZSelfTest(void) {
	return MPU6050_ReadRegBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, 1);
}

HAL_StatusTypeDef MPU6050_SetAccelZSelfTest(const uint8_t enabled) {
	return MPU6050_WriteRegisterBits(MPU6050_RA_ACCEL_CONFIG, enabled, MPU6050_ACONFIG_ZA_ST_BIT, 1);
}

static HAL_StatusTypeDef MPU6050_WriteRegisterBits(uint8_t reg, uint8_t data, unsigned pos, unsigned len)
{
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(reg, 1, &readData);

	if(opStatus != HAL_OK)
	{
		return opStatus;
	}

    uint8_t mask = ((1 << len) - 1) << pos;

    data <<= pos;
    data &= mask;
    readData &= ~(mask);
    readData |= data;

    return MPU6050_WriteRegister(reg, readData);
}

static uint8_t MPU6050_ReadRegBits(uint8_t reg, unsigned pos, unsigned len)
{
	uint8_t readData = 0;
	HAL_StatusTypeDef opStatus = MPU6050_ReadRegister(reg, 1, &readData);

	if(opStatus != HAL_OK)
	{
		return 0x00;
	}

    uint8_t mask = (1 << len) - 1;
    readData >>= pos;
    readData &= mask;

    return readData;
}

static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t address, uint8_t cmd)
{
	uint8_t data[2] = {0x00};
	HAL_StatusTypeDef opResoult = HAL_ERROR;
	data[0] = address;
	data[1] = cmd;

	opResoult = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEFAULT_ADDRESS, data, 2, 10);

	return opResoult;
}

static HAL_StatusTypeDef MPU6050_ReadRegister(uint8_t address, uint8_t dataSize, uint8_t *rec)
{
	HAL_StatusTypeDef opResoult = HAL_ERROR;
	uint8_t data[1] = {0x00};
	data[0] = address;

	opResoult = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEFAULT_ADDRESS, data, 1, 10);
	opResoult = HAL_ERROR;
	opResoult = HAL_I2C_Master_Receive(&hi2c1, MPU6050_DEFAULT_ADDRESS + 1, &rec[0], dataSize, 10);

	return opResoult;
}
