/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * The Problem:
 * Create a Task scheduler and tasks to read the button state, the temperature sensor and
 * out put the data to the UART:
 *
 * Modify the provided gpiointerrupt.c code. Your code needs to
 * check the buttons every 200ms,
 * check the temperature every 500ms, and
 * update the LED and report to the server every second (via the UART).
 * If you push a button, it increases or decreases the temperature set-point by 1 degree
 * every 200ms. If the temperature is greater than the set-point, the LED should turn off. If the
 * temperature is less than the set-point, the LED should turn on (the LED controls a heater). You can
 * simulate a heating or cooling room by putting your finger on the temperature sensor. The output to
 * the server (via UART) should be formatted as <AA,BB,S,CCCC>.

 * To accomplish this work, you will need to use a task scheduler, as described in your course
 * resources. Note that some additional content from your course will be useful in helping you
 * complete this project.
 *
 *Notes:
 * Milestone Two covered how to turn an LED on or off.
 * Milestone Three covered how to change the period for the timer interrupt.
 * Milestone Three also showed you how to use the button interrupt callbacks.
 * zyBooks activities covered how to create a task scheduler.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h> //[JK] Included the this header file to remove implicitly declared Warnings for snprintf


/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver for I2C */
#include <ti/drivers/I2C.h>

/* Driver for UART */
#include <ti/drivers/UART.h>
#define DISPLAY(x) UART_write(uart, &output, x);

/* Driver configuration */
#include "ti_drivers_config.h"

/* Timer Driver*/
#include <ti/drivers/Timer.h>

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
I2C_Handle i2c;
UART_Handle uart;
Timer_Handle timer0;

//From video
#define TRUE 1
#define FALSE 0
#define NULL 0
#define NUMBER_OF_TASKS 3
#define GLOBAL_PERIOD 100   // milliseconds
#define DEFAULT_SET_TEMP 25 // Default Celsius value to be close to Temperature sensor reading

char ready_tasks = FALSE;

// Global period to be used in Init Timer
int global_period = GLOBAL_PERIOD;

volatile unsigned char TimerFlag = FALSE;
int16_t setpoint = DEFAULT_SET_TEMP;
volatile unsigned char btn_0_Flag = FALSE;
volatile unsigned char btn_1_Flag = FALSE;

// Testing variables//
int16_t tempCapture = 0;
volatile unsigned char heat = 0; //either 0 or 1
unsigned int seconds = 0;
unsigned char timer = 0;

/* Adding structure for tasks*/
//A single task in the task list
typedef struct task_entry {
    void (*f)(); // Function to call to perform the task
    unsigned long elapsedTime; // Amount of time since last triggered
    unsigned long period; // Period of task in MS
    char triggered; //True or False whether or not the task was triggered
}task_entry;  //[JK] Added variable name to make structure name and removed Warning "declaration requires a typedef name"

// Forward declaration of functions/Tasks
void chk_btn();
void readTmp();
void updtLEDReport();

// The task List
struct task_entry tasks[NUMBER_OF_TASKS] =
{
 {&chk_btn, 200, 200, FALSE}, //Function, elapsed time, period, Triggered
 {&readTmp, 500, 500, FALSE},
 {&updtLEDReport, 1000, 1000, FALSE}
};

// Function to Turn Off the LED that represents the Heat coming Off
void TurnHeatOFF()
{
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    heat = FALSE;
}
//Function to Turn On the LED that represents the Heat coming On
void TurnHeatOn()
{
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    heat = TRUE;
}

void initUART(void)
{

    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL)
    {
        /* UART_open() failed */
        while (1);
    }
}

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY( snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL){
        DISPLAY( snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY( snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY( snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY( snprintf(output, 64, "No\n\r"))
    }

    if (found)
    {
        DISPLAY( snprintf(output, 64, "Detected TMP%s I2C address :%x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }

    else
    {
        DISPLAY( snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else {
        DISPLAY( snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY( snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
}

//Start Tasks Functions
// Check button every 200ms
void chk_btn()
{
    //processing for check button task place
    //printf("\nSeconds = %d\nChecking button: Elapsed = %d\n", seconds, tasks[0].elapsedTime);
    switch (btn_0_Flag) {
    case 0:
        break;
    case 1:
        setpoint++;
        btn_0_Flag = FALSE;
        break;
    }
    switch (btn_1_Flag) {
    case 0:
        break;
    case 1:
        setpoint--;
        btn_1_Flag = FALSE;
        break;
    }
}

//Read Temperature every 500ms
//This function has to be a void due to task structure
void readTmp()
{
    //processing for read temperature task place
    tempCapture = readTemp();
    //printf("Reading Temperature: Elapsed = %d\n", tasks[1].elapsedTime);

}

//Update the LED and report to the server (UART) every second
void updtLEDReport()
{
    //processing for task three task place
    if (tempCapture >= setpoint)
    {
        TurnHeatOFF();
    }

    else
    {
        TurnHeatOn();

    }
//    printf("Updating and Reporting: Elapsed = %d\n", tasks[2].period);

    DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", tempCapture, setpoint, heat, seconds))
    seconds++;
}
//End Tasks Functions

/* Timer Callback Function*/
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = TRUE;

    //printf("\nSeconds = %d\nCheck Button Elapsed Time = %d\n",seconds, tasks[0].elapsedTime);

    //from Lecture Video
    // set the ready-tasks to true, if any task interval has expired
    int x = 0;
    for (x=0; x < NUMBER_OF_TASKS; x++)
    {
        //Check if tasks interval has expired
        if (tasks[x].elapsedTime >= tasks[x].period)
        {
            //Bing! This tasks timer is up
            // Set its flag and the global flag
            tasks[x].triggered = TRUE;
            ready_tasks = TRUE;
            // Reset the elapsed time
            tasks[x].elapsedTime = 0;
        }
        else
        {
            tasks[x].elapsedTime += global_period;
        }
    }
}

void initTimer(void)
{
    Timer_Params params;

    // Initiate the driver
    Timer_init();

    //Configure the driver
    Timer_Params_init(&params);
    //[JK] changed to from 1,000,000 to 100,000
    params.period = 100000;//200, 500, 1000 millisecond can be divided by 100 milliseconds or 100,000 microseconds
    // TODO Mod operator to check intervals
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    //Open the Driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL)
    {
        /* Failed to initialized timer */
        while (1) {}
     }

     if (Timer_start(timer0) == Timer_STATUS_ERROR)
     {
         /* Failed to start timer */
         while (1) {}
     }
}
/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    btn_0_Flag = TRUE;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    btn_1_Flag = TRUE;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW); //Sets pins output to 0
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING); //Interrupt on falling edge

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    initUART(); //The UART must be initialized before calling initI2C()
    initI2C();
    initTimer();

    //Loop Forever
    // The Student should add flags (similar to the timer flag) to the button handlers
    while(1)
    {

        //[JK] commented out the below line
        //DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", readTemp(), setpoint, heat, seconds))

        //Wait for the task interval to elapse
        //Process the tasks for which the interval has expired
        //reset everything (eg flags) and go back to the beginning
        while (!ready_tasks) {}
        int x=0;

        // Loop through each of the tasks, check if they are triggered, call thier functions, reset trigger
        for (x=0; x < NUMBER_OF_TASKS; x++)
        {
            if (tasks[x].triggered)
            {
                tasks[x].f();
                //reset
                tasks[x].triggered = FALSE;
            }
        }
        ready_tasks = FALSE;
        TimerFlag = FALSE;

    }

}
