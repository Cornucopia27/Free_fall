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
static i2c_master_transfer_t masterXfer;
static i2c_master_handle_t g_m_handle; //handle created for the callback

void PIT0_IRQHandler()
{
    PIT_ClearStatusFlags( PIT, kPIT_Chnl_0, kPIT_TimerFlag );
}

static void i2c_master_callback( I2C_Type *base, i2c_master_handle_t *handle,
        status_t status, void * userData )
{

    if ( status == kStatus_Success )
    {
        g_MasterCompletionFlag = true;
    }
}

void I2C_common_init()
{
    CLOCK_EnableClock( kCLOCK_PortE ); //for the I2C0 SCL and SDA enable
    CLOCK_EnableClock( kCLOCK_I2c0 );	//for the I2C0 clock enable

    port_pin_config_t config_i2c = { kPORT_PullDisable, kPORT_SlowSlewRate,
            kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength, kPORT_MuxAlt5, kPORT_UnlockRegister, };
    PORT_SetPinConfig( PORTE, 24, &config_i2c ); //I2C0 SCL pin config
    PORT_SetPinConfig( PORTE, 25, &config_i2c ); //I2C0 SDA pin config

    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig( &masterConfig );
    masterConfig.baudRate_Bps = 100000;
    I2C_MasterInit( I2C0, &masterConfig, CLOCK_GetFreq( kCLOCK_BusClk ) );

    I2C_MasterTransferCreateHandle( I2C0, &g_m_handle, i2c_master_callback,
            NULL );
}

void Red_Led_init()
{
    CLOCK_EnableClock( kCLOCK_PortB );

    port_pin_config_t config_led = { kPORT_PullDisable, kPORT_SlowSlewRate,
            kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister, };

    PORT_SetPinConfig( BLUE_RED_LED_PORT, RED_LED_PIN, &config_led );

    gpio_pin_config_t led_config_gpio = { kGPIO_DigitalOutput, 1 };
    GPIO_PinInit( GPIOB, RED_LED_PIN, &led_config_gpio );
}

void Pit_common_init()
{
    pit_config_t Pit_config;
    PIT_GetDefaultConfig( &Pit_config );
    CLOCK_EnableClock( kCLOCK_Pit0 );
    PIT_Init( PIT, &Pit_config );
    uint32_t period = 1;
    PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, period * CLOCK_GetBusClkFreq() );
    PIT_EnableInterrupts( PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable );
    EnableIRQ( PIT0_IRQn );
    PIT_StartTimer( PIT, kPIT_Chnl_0 );
}

void i2c_release_bus_delay()
{
    uint32_t i = 0;
    for ( i = 0; i < 100; i++ )
    {
        __NOP();
    }
}

void i2c_ReleaseBus()
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = { 0 };

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock( kCLOCK_PortE );
    PORT_SetPinConfig( PORTE, 24, &i2c_pin_config );
    PORT_SetPinConfig( PORTE, 25, &i2c_pin_config );

    GPIO_PinInit( GPIOE, 24, &pin_config );
    GPIO_PinInit( GPIOE, 25, &pin_config );

    GPIO_PinWrite( GPIOE, 25, 0U );
    i2c_release_bus_delay();

    for ( i = 0; i < 9; i++ )
    {
        GPIO_PinWrite( GPIOE, 24, 0U );
        i2c_release_bus_delay();

        GPIO_PinWrite( GPIOE, 25, 1U );
        i2c_release_bus_delay();

        GPIO_PinWrite( GPIOE, 24, 1U );
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    GPIO_PinWrite( GPIOE, 24, 0U );
    i2c_release_bus_delay();

    GPIO_PinWrite( GPIOE, 25, 0U );
    i2c_release_bus_delay();

    GPIO_PinWrite( GPIOE, 24, 1U );
    i2c_release_bus_delay();

    GPIO_PinWrite( GPIOE, 25, 1U );
    i2c_release_bus_delay();
}

void IMU_accelerometer_init()
{
    uint8_t data_buffer = 0x01;
    masterXfer.slaveAddress = 0x1D;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0x2A;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &data_buffer;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking( I2C0, &g_m_handle, &masterXfer );
    while ( !g_MasterCompletionFlag )
    {
    }
    g_MasterCompletionFlag = false;
}

void Accelerometer_measures()
{
    uint8_t buffer[6];
    int16_t accelerometer[3];

    masterXfer.slaveAddress = 0x1D;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = 0x01;
    masterXfer.subaddressSize = 1;
    masterXfer.data = buffer;
    masterXfer.dataSize = 6;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking( I2C0, &g_m_handle, &masterXfer );
    while ( !g_MasterCompletionFlag )
    {
    }
    g_MasterCompletionFlag = false;

    accelerometer[0] = buffer[0] << 8 | buffer[1];
    accelerometer[1] = buffer[2] << 8 | buffer[3];
    accelerometer[2] = buffer[4] << 8 | buffer[5];
    //adjustment of accelerometer value from being in 2 g sensitivity mode
    float x_value = 0;
    float y_value = 0;
    float z_value = 0;
    x_value = accelerometer[0] * .000244;
    y_value = accelerometer[1] * .000244;
    z_value = accelerometer[2] * .000244;
}
