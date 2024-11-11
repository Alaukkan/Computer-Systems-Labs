/*
 * main.c
 *
 *  Created on: 20 Oct 2024
 *      Author: alexl
 */

#include <library.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <math.h>

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
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE * 2];
Char buzzerTaskStack[STACKSIZE];

enum state { WAITING=0, READY };
enum state programState = WAITING;
enum state dataState = WAITING;

struct mpu_sample_t sensor_buffer[3]; // buffer saves last 3 recorded data samples

struct melody_t * playing_melody;

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

bool checkTilt(int * tilt, struct mpu_sample_t samples[]);
bool checkJolt(int * jolt, struct mpu_sample_t samples[]);
char checkMovement(int * movement, struct mpu_sample_t samples[]);
Void sensorInit(I2C_Handle *i2c, I2C_Params *i2cParams);


void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    uint_t pinValue = PIN_getOutputValue(Board_LED0);
    pinValue = !pinValue;
    PIN_setOutputValue(ledHandle, Board_LED0, pinValue);
    if (programState == WAITING) {
        playing_melody = startup;
        programState = READY;
    } else {
        playing_melody = shutdown;
        programState = WAITING;
    }
}

Void buzzerTaskFxn(UArg arg0, UArg arg1) {
    while (1) {
        if (playing_melody != NULL) {
            buzzerOpen(hBuzzer);
            int i = 0;

            while (playing_melody[i].duration > 0) {
                buzzerSetFrequency(playing_melody[i].frequency);
                Task_sleep(playing_melody[i].duration * 1000 / Clock_tickPeriod);
                i++;
            }

            buzzerClose();
            playing_melody = NULL;
        }
        Task_sleep(50000 / Clock_tickPeriod);
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

    int movement[2] = {-1, 0};
    char curr_message[6];
    int message_len = 0;

    while (1) {
        if (programState == WAITING) {
            Task_sleep(50000/ Clock_tickPeriod);
            continue;
        }

        /* FOR DETECTING MOVEMENT */
        if (dataState == READY) {

            /*Sanity check
            char str1[32];
            if (tilt) {
                sprintf(str1, "Tilt detected %d\n\r", tilt);
            } else {
                sprintf(str1, "Device is idle\n\r");
            }
            UART_write(uart, str1, strlen(str1) + 1);
            */

            char morse = checkMovement(movement, sensor_buffer);

            if (morse != '\0') {
                char str[4] = {morse, '\n', '\r', '\0'};
                UART_write(uart, str, 4);
                switch(morse){
                case('.'):
                    playing_melody = dot;
                    break;
                case('-'):
                    playing_melody = dash;
                    break;
                }

                if (morse == ' ') { // translating morse to latin
                    int i;
                    if (strlen(curr_message) == 5) {
                        for (i = 0; i < 10; i++) {
                            if (strcmp(num_translate[i].morse, curr_message) == 0) {
                                char str[4] = {num_translate[i].number, '\n', '\r', '\0'};
                                UART_write(uart, str, 4);
                                break;
                            }
                        }
                    } else {
                        for (i = 0; i < 26; i++) {
                            if (strcmp(translate[i].morse, curr_message) == 0) {
                                char str[4] = {translate[i].latin, '\n', '\r', '\0'};
                                UART_write(uart, str, 4);
                                break;
                            }
                        }
                    }
                    curr_message[0] = '\0';
                    message_len = 0;
                } else if (message_len < 5) {
                    curr_message[message_len + 1] = curr_message[message_len];
                    curr_message[message_len] = morse;
                    message_len++;
                } else {
                    curr_message[0] = '\0';
                    message_len = 0;
                }
            }


            dataState = WAITING;
        }
        /* FOR COLLECTING DATA
        char str1[64];
        if (dataState == READY) {
            sprintf(str1, "%f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n\r", sensor_buffer[2].timestamp, sensor_buffer[2].accel.ax, sensor_buffer[2].accel.ay, sensor_buffer[2].accel.az, sensor_buffer[2].gyro.gx, sensor_buffer[2].gyro.gy, sensor_buffer[2].gyro.gz);
            UART_write(uart, str1, strlen(str1) + 1);
            dataState = WAITING;
        }*/

        // Once per 50ms
        Task_sleep(50000/ Clock_tickPeriod);
    }
}

bool checkTilt(int * tilt, struct mpu_sample_t samples[]) {
    float diff_gy = tilt_data[0].gyro.gy - samples[2].gyro.gy;
    int threshold = 170;
    if (*tilt == 0) {
        if (diff_gy > threshold + 50) {
            (*tilt)++;
        } else {
            *tilt = 0;
        }
    } else if (*tilt < 4) {
        if (diff_gy < -threshold) {
            return true;
        } else {
            (*tilt)++;
        }
    }
    return false;
}

bool checkJolt(int * jolt, struct mpu_sample_t samples[]) {
    float diff_ay = tilt_data[0].accel.ay - samples[2].accel.ay;
    float threshold = 0.6;
    if (*jolt == 0) {
        if (diff_ay > threshold + 0.2) {
            (*jolt)++;
        } else {
            *jolt = 0;
        }
    } else if (*jolt < 4) {
        if (diff_ay < -threshold) {
            return true;
        } else {
            (*jolt)++;
        }
    }
    return false;
}

char checkMovement(int * movement, struct mpu_sample_t samples[]) {
    char result = '\0';
    if (movement[0] == -1 || movement[0] == 0) {
        // dot
        if (checkTilt(&movement[1], samples)) {
            result = '.';
            movement[0] = -1;
            movement[1] = 0;
        } else if (movement[1] > 0 && movement[1] < 4) {
            movement[0] = 0;
        } else {
            movement[0] = -1;
            movement[1] = 0;
        }
    }
    if (movement[0] == -1 || movement[0] == 1) {
        // dash
        if (checkJolt(&movement[1], samples)) {
            result = ' ';
            movement[0] = -1;
            movement[1] = 0;
        } else if (movement[1] > 0 && movement[1] < 4) {
            movement[0] = 1;
        } else {
            movement[0] = -1;
            movement[1] = 0;
        }
    }
    if (movement[0] == -1 || movement[0] == 2) {
        // space

    }
    return result;
}

Void sensorInit(I2C_Handle *i2c, I2C_Params *i2cParams) {

    I2C_Params_init(i2cParams);
    i2cParams->bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cParams->custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    *i2c = I2C_open(Board_I2C, i2cParams);
    if (*i2c == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // MPU setup and calibration
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(i2c);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();
}

Void sensorTaskFxn(UArg arg0, UArg arg1) {
    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;

    sensorInit(&i2cMPU, &i2cMPUParams);
    while (1) {
        if (programState == WAITING) {
            Task_sleep(50000/ Clock_tickPeriod);
            continue;
        }
        if (dataState == WAITING) {
            // update buffer
            sensor_buffer[0] = sensor_buffer[1];
            sensor_buffer[1] = sensor_buffer[2];
            // MPU ask data
            mpu9250_get_data(&i2cMPU, &(sensor_buffer[2].accel.ax), &(sensor_buffer[2].accel.ay), &(sensor_buffer[2].accel.az), &(sensor_buffer[2].gyro.gx), &(sensor_buffer[2].gyro.gy), &(sensor_buffer[2].gyro.gz));
            uint32_t ticks = Clock_getTicks();
            sensor_buffer[2].timestamp = (float) ticks * (Clock_tickPeriod / 1000000.0);
            dataState = READY;
        }

        Task_sleep(50000 / Clock_tickPeriod);
    }
}

Int main(void) {

    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    Task_Handle buzzerTaskHandle;
    Task_Params buzzerTaskParams;

    // Initialize board
    Board_initGeneral();

    Board_initUART();

    Board_initI2C();

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
    uartTaskParams.priority = 1;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&buzzerTaskParams);
    buzzerTaskParams.stackSize = STACKSIZE;
    buzzerTaskParams.stack = &buzzerTaskStack;
    buzzerTaskParams.priority = 3;
    buzzerTaskHandle = Task_create((Task_FuncPtr)buzzerTaskFxn, &buzzerTaskParams, NULL);
    if (buzzerTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
