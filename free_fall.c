/*
 * free_fall.c
 *
 *  Created on: Feb 7, 2018
 *      Author: ALEX
 */

#include "free_fall.h"
#include "clock_config.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "fsl_i2c.h"

#define BLUE_RED_LED_PORT PORTB
#define RED_LED_PIN 22

static bool g_MasterCompletionFlag = false;

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
        status_t status, void * userData)
{

	if (status == kStatus_Success)
	{
		g_MasterCompletionFlag = true;
	}
}

void I2C_common_init()
{
	CLOCK_EnableClock(kCLOCK_PortE); //for the I2C0 SCL and SDA enable
	CLOCK_EnableClock(kCLOCK_I2c0);	//for the I2C0 clock enable

	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt5,
	        kPORT_UnlockRegister, };
	PORT_SetPinConfig(PORTE, 24, &config_i2c); //I2C0 SCL pin config
	PORT_SetPinConfig(PORTE, 25, &config_i2c); //I2C0 SDA pin config

	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	i2c_master_handle_t g_m_handle; //handle created for the callback
	I2C_MasterTransferCreateHandle(I2C0, &g_m_handle,
	        i2c_master_callback, NULL);
}

void Red_Led_init()
{
	CLOCK_EnableClock(kCLOCK_PortB);

	port_pin_config_t config_led =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig(BLUE_RED_LED_PORT, RED_LED_PIN, &config_led);

	gpio_pin_config_t led_config_gpio =
	{ kGPIO_DigitalOutput, 1 };
	GPIO_PinInit(GPIOB, RED_LED_PIN, &led_config_gpio);
}


int main(void)
{

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	i2c_master_transfer_t masterXfer;


	uint8_t data_buffer = 0x01;

#if 0
	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = &data_buffer;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferNoStopFlag;

	I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle,
	        &masterXfer);
	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;

	uint8_t read_data;

	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = &read_data;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferRepeatedStartFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle,
	        &masterXfer);
	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;
#else

	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x2A;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &data_buffer;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle,
	        &masterXfer);
	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;

#endif
	/* Force the counter to be placed into memory. */
	volatile static int i = 0;
	uint8_t buffer[6];
	int16_t accelerometer[3];
	/* Enter an infinite loop, just incrementing a counter. */
	while (1)
	{
		masterXfer.slaveAddress = 0x1D;
		masterXfer.direction = kI2C_Read;
		masterXfer.subaddress = 0x01;
		masterXfer.subaddressSize = 1;
		masterXfer.data = buffer;
		masterXfer.dataSize = 6;
		masterXfer.flags = kI2C_TransferDefaultFlag;

		I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle,
				&masterXfer);
		while (!g_MasterCompletionFlag){}
		g_MasterCompletionFlag = false;

		accelerometer[0] = buffer[0]<<8 | buffer[1];
		accelerometer[1] = buffer[2]<<8 | buffer[3];
		accelerometer[2] = buffer[4]<<8 | buffer[5];
	}
	return 0;
}
