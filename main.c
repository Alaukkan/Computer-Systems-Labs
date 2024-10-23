/*
 * main.c
 *
 *  Created on: 20 Oct 2024
 *      Author: alexl
 */

#include <stdio.h>
#include <time.h>
#include <string.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "sensors/mpu9250.h"
#include "buzzer.h"

/* Task */
#define STACKSIZE 5096
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char buzzerTaskStack[1024];


enum state { WAITING=1, READY };
enum state programState = WAITING;
enum state dataState = WAITING;

float timestamp, ax, ay, az, gx, gy, gz;

Char message = ' ';

enum melody {OFF=1, MELODY_START_UP, MELODY_SHUT_DOWN};
enum melody playing_melody = OFF;

static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};

PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};

PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

Void buzzer(int m);

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    uint_t pinValue = PIN_getOutputValue(Board_LED0);
    pinValue = !pinValue;
    programState = !programState;
    PIN_setOutputValue(ledHandle, Board_LED0, pinValue);
    if (programState == WAITING) {
        //playing_melody = MELODY_SHUT_DOWN;
        buzzer(MELODY_SHUT_DOWN);
    } else {
        //playing_melody = MELODY_START_UP;
        buzzer(MELODY_START_UP);
    }
}

Void uartTaskFxn(UArg arg0, UArg arg1) {
    UART_Handle uart;
    UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.baudRate = 9600;
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        System_abort("Error opening the UART");
    }

    while (1) {
        if (programState == WAITING) {
            Task_sleep(50000/ Clock_tickPeriod);
            continue;
        }

        /*char str[6];
        if (programState == DATA_READY) {
            sprintf(str, "%c\n\r", message);
            System_printf(str);
            System_flush();
            UART_write(uart, str, strlen(str));
            programState = WAITING;
        }

        System_printf("uartTask\n");
        System_flush();*/

        char str2[64];
        if (dataState == READY) {
            sprintf(str2, "%f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n\r", timestamp, ax, ay, az, gx, gy, gz);
            UART_write(uart, str2, strlen(str2));
            dataState = WAITING;
        }

        //System_printf("uartTask\n");
        //System_flush();

        // Once per 50ms
        Task_sleep(50000/ Clock_tickPeriod);
    }
}



Void sensorTaskFxn(UArg arg0, UArg arg1) {
    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // MPU setup and calibration
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();

    while (1) {
        if (programState == WAITING) {
            Task_sleep(50000/ Clock_tickPeriod);
            continue;
        }
        // MPU ask data
        mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
        uint32_t ticks = Clock_getTicks();;
        timestamp = (float) ticks * (Clock_tickPeriod / 1000000.0);
        dataState = READY;

        Task_sleep(50000 / Clock_tickPeriod);
    }
}

Void buzzerTaskFxn(UArg arg0, UArg arg1) {
    while (1) {
        switch(playing_melody) {
            case(OFF):
                break;
            case(MELODY_START_UP):
                buzzerOpen(hBuzzer);
                buzzerSetFrequency(2000);
                Task_sleep(100000 / Clock_tickPeriod);
                buzzerClose();
                playing_melody = OFF;
                break;
            case(MELODY_SHUT_DOWN):

                break;
            default:
                break;
        }
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

Void buzzer(int m) {
    switch(m) {
        case(OFF):
            break;
        case(2):
            buzzerOpen(hBuzzer);
            buzzerSetFrequency(2000);
            Task_sleep(500000 / Clock_tickPeriod);
            buzzerClose();
            break;
        case(MELODY_SHUT_DOWN):

            break;
        default:
            break;
    }
}

Int main(void) {

    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    //Task_Handle buzzerTaskHandle;
    //Task_Params buzzerTaskParams;

    // Initialize board
    Board_initGeneral();

    Board_initUART();

    Board_initI2C();

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }

    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
       System_abort("Error initializing button pins\n");
    }
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
        System_abort("Error initializing LED pins\n");
    }

    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
        System_abort("Error registering button callback function");
    }

    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
        System_abort("Pin open failed!");
    }


    /* Tasks */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority = 2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority = 2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }
    /*
    Task_Params_init(&buzzerTaskParams);
    buzzerTaskParams.stackSize = STACKSIZE;
    buzzerTaskParams.stack = &buzzerTaskStack;
    buzzerTaskParams.priority = 2;
    buzzerTaskHandle = Task_create(buzzerTaskFxn, &buzzerTaskParams, NULL);
    if (buzzerTaskHandle == NULL) {
        System_abort("Task create failed!");
    }*/

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}











