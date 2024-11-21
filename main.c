/*
 * main.c
 *
 *  Created on: 20 Oct 2024
 *      Author: alexl
 */

#include "library.h"
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


#define STACKSIZE 2048

#define BUFFER_SAMPLE_LENGTH 3
#define REFERENCE_SAMPLE_LENGTH 7
#define NUMBER_OF_MOVEMENTS 3

#define BLOCK_SIZE 2

#define ACCEL_THRESHOLD 0.3f
#define GYRO_THRESHOLD 200

Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE * 2];
Char buzzerTaskStack[STACKSIZE];

enum state { WAITING=0, READY, RECIEVING_WAITING, RECIEVING };
enum state programState = WAITING;
enum state dataState = WAITING;

enum MovementType { L_R=0, U_D=1 , R_L=2};

struct mpu_sample_t sensor_buffer[BUFFER_SAMPLE_LENGTH]; // buffer saves last 3 recorded data samples
uint8_t uartBuffer[30]; // Receive buffer

struct melody_t * playing_melody;

char morse;
char recieved_string[33];
int recieved_string_len = 0;

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

char translate(char curr_message[]);
void calculateMean(struct mpu_sample_t sample[], struct mpu_sample_t *mean, int startIndex, int n, int sample_length);
bool sampleCompare(struct mpu_sample_t *a, struct mpu_sample_t *b);
bool updateMovement(int iteration[]);
Void sensorInit(I2C_Handle *i2c, I2C_Params *i2cParams);


void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    uint_t pinValue = PIN_getOutputValue(Board_LED0);
    pinValue = !pinValue;
    PIN_setOutputValue(ledHandle, Board_LED0, pinValue);
    if (programState == WAITING) {
        playing_melody = startup;
        programState = READY;
    } else if (programState == READY) {
        playing_melody = shutdown;
        programState = WAITING;
    } else if (programState == RECIEVING_WAITING) { // waits and rings until user presses button
        playing_melody = startup;
        programState = RECIEVING; // uartTask will start showing the morse code
    }
}

Void buzzerTaskFxn(UArg arg0, UArg arg1) {
    while (1) {
        if (playing_melody != NULL) {
            buzzerOpen(hBuzzer);
            int i = 0;

            while (playing_melody[i].duration > 0) {
                if (playing_melody[i].frequency > 0) { // if freq is 0 no note is played, can be used as pauses in the melodies
                    buzzerSetFrequency(playing_melody[i].frequency);
                    Task_sleep(playing_melody[i].duration * 1000 / Clock_tickPeriod);
                } else {
                    buzzerClose();
                    Task_sleep(playing_melody[i].duration * 1000 / Clock_tickPeriod);
                    buzzerOpen(hBuzzer);
                }
                i++;
            }

            buzzerClose();
            //if (programState != RECIEVING_WAITING) { // repeating "phone call"
            playing_melody = NULL;
            //}
        }
        Task_sleep(50000 / Clock_tickPeriod);
    }
}


void uartFxn(UART_Handle uart, void *rxBuf, size_t len) {
    uint8_t recieved_morse = *((uint8_t*)rxBuf);

    //if (recieved_morse == ' ' || recieved_morse == '.' || recieved_morse == '-') {
        recieved_string[recieved_string_len] = (char)recieved_morse;
        recieved_string_len++;
        programState = RECIEVING;

    //}
    //playing_melody = recieving;

    UART_read(uart, rxBuf, 3);
}

Void uartTaskFxn(UArg arg0, UArg arg1) {
    UART_Handle uart;
    UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_CALLBACK; // Interrupt-based reception
    uartParams.readCallback = &uartFxn; // Handler function
    uartParams.baudRate = 9600;
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        System_abort("Error opening the UART");
    }

    char morse_str[4];

    UART_read(uart, uartBuffer, 3);

    while (1) {

        /* FOR SENDING MORSE */
        if (dataState == READY) {

            if (morse != '\0') {

                sprintf(morse_str, "%c\r\n", morse);

                UART_write(uart, morse_str, 4);

            }
            morse = '\0';
            dataState = WAITING;

            /* FOR COLLECTING DATA
            char str1[64];
            sprintf(str1, "%f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", sensor_buffer[2].timestamp, sensor_buffer[2].accel.ax, sensor_buffer[2].accel.ay, sensor_buffer[2].accel.az, sensor_buffer[2].gyro.gx, sensor_buffer[2].gyro.gy, sensor_buffer[2].gyro.gz);
            UART_write(uart, str1, strlen(str1) + 1); */

        }

        if (programState == RECIEVING) {
            int i;
            uint_t pinValue = PIN_getOutputValue(Board_LED0);
            recieved_string[recieved_string_len] = '\0';
            UART_write(uart, recieved_string, strlen(recieved_string) + 1);
            for (i = 0; i < recieved_string_len; i++) {
                switch(recieved_string[i]) {
                    case(' '):
                        Task_sleep(1000000/ Clock_tickPeriod); // 1,5 sec silence between every letter
                        break;
                    case ('.'):
                        PIN_setOutputValue(ledHandle, Board_LED0, !pinValue);
                        Task_sleep(100000/ Clock_tickPeriod);
                        PIN_setOutputValue(ledHandle, Board_LED0, pinValue);
                        //playing_melody = dot;
                        break;
                    case('-'):
                        PIN_setOutputValue(ledHandle, Board_LED0, !pinValue);
                        Task_sleep(500000/ Clock_tickPeriod);
                        PIN_setOutputValue(ledHandle, Board_LED0, pinValue);
                        //playing_melody = dash;
                        break;
                    default:
                        PIN_setOutputValue(ledHandle, Board_LED0, !pinValue);
                        Task_sleep(1000000/ Clock_tickPeriod); // 1,5 sec silence between every letter
                        PIN_setOutputValue(ledHandle, Board_LED0, pinValue);
                        break;
                }
                Task_sleep(500000/ Clock_tickPeriod);
            }

            recieved_string_len = 0;
            programState = READY;
/*
            switch(recieved_string[0]) {
                case(' '):
                    Task_sleep(1000000/ Clock_tickPeriod); // 2 sec silence between every letter
                    break;
                case ('.'):
                    playing_melody = dot;
                    break;
                case('-'):
                    playing_melody = dash;
                    break;
                case('\0'):
                    programState = READY;
                    recieved_string[0] = '\0';
                    i = -1; // is incremented back to 0
                    break;
            }
            i++;
            Task_sleep(1000000/ Clock_tickPeriod); // 1 sec between every morse*/
        }

        // Once per 50ms
        Task_sleep(50000/ Clock_tickPeriod);
    }
}

char translate(char curr_message[]) {
    int i;
    for (i = 0; i < 36; i++) {
        if (strcmp(translation[i].morse, curr_message) == 0) {
            return translation[i].latin;
        }
    }
    return '\0'; // no translation found
}

bool sampleCompare(struct mpu_sample_t *a, struct mpu_sample_t *b) {
    return (fabsf(a->accel.ax - b->accel.ax) < ACCEL_THRESHOLD) &&
           (fabsf(a->accel.ay - b->accel.ay) < ACCEL_THRESHOLD) &&
           // (fabsf(a->accel.az - b->accel.az) < ACCEL_THRESHOLD) && // z-axis doesn't seem to work correctly
           (fabsf(a->gyro.gx - b->gyro.gx) < GYRO_THRESHOLD) &&
           (fabsf(a->gyro.gy - b->gyro.gy) < GYRO_THRESHOLD) &&
           (fabsf(a->gyro.gz - b->gyro.gz) < GYRO_THRESHOLD);
}

void calculateMean(struct mpu_sample_t sample[], struct mpu_sample_t *mean, int startIndex, int n, int sample_length) {
    /* Helper function to calculate mean (or moving average) of buffer/data */
    // check if startIndex and n are valid
    if (startIndex < 0 || n <= 0 || startIndex + n > sample_length) {
        return;
    }
    // ensure mean is empty
    mean->accel.ax = mean->accel.ay = mean->accel.az = 0;
    mean->gyro.gx = mean->gyro.gy = mean->gyro.gz = 0;

    int i;
    for (i = startIndex; i < startIndex + n; i++) {
        mean->accel.ax += sample[i].accel.ax / n;
        mean->accel.ay += sample[i].accel.ay / n;
        mean->accel.az += sample[i].accel.az / n;
        mean->gyro.gx += sample[i].gyro.gx / n;
        mean->gyro.gy += sample[i].gyro.gy / n;
        mean->gyro.gz += sample[i].gyro.gz / n;
    }
}

bool updateMovement(int iteration[]) {
    /* Always calculates mean of the 3 last samples in sensor_buffer
     * Calculates rolling mean of recorded data with iteration going from 0 to N-3
     * compares calculated means with defined thresholds
     * Iteration stays at 0 until match is found
     * When a match is found, iteration moves up
     * If matching streak breaks, iteration goes back to 0
     * */
    struct mpu_sample_t buffer_mean = {0, {0, 0, 0}, {0, 0, 0}};
    struct mpu_sample_t data_mean = {0, {0, 0, 0}, {0, 0, 0}};
    bool skip[NUMBER_OF_MOVEMENTS] = {false};
    bool movement_detected = false;

    // calculate mean of buffer
    calculateMean(sensor_buffer, &buffer_mean, 0, BLOCK_SIZE, BUFFER_SAMPLE_LENGTH);

    // check from which data to calculate the rolling mean
    if (iteration[L_R] == 0 && iteration[U_D] == 0 && iteration[R_L] == 0) { // no movements detected yet
        //calculate all, for loop doesn't skip anything
        skip[L_R] = skip[U_D] = skip[R_L] = false;

    } else { // at least one movement has been "detected"
        // figure out which movements need to be skipped and which ones need to be checked
        if (iteration[L_R] == 0) {
            // no tilt, for loop skips tilting check
            skip[L_R] = true;
        }
        if (iteration[U_D] == 0) {
            // no jolt, for loop skips jolting check
            skip[U_D] = true;
        }
        if (iteration[R_L] == 0) {

            skip[R_L] = true;
        }

    }
    int i;
    for (i = 0; i < NUMBER_OF_MOVEMENTS; i++) {
        if (skip[i]) { // check if movement check needs to be skipped
            continue;
        }
        // calculate an iteration of rolling mean of data
        calculateMean((*reference_data)[i], &data_mean, iteration[i], BLOCK_SIZE, REFERENCE_SAMPLE_LENGTH);

        //compare data and buffer
        if (sampleCompare(&buffer_mean, &data_mean)) {
            iteration[i]++;
            movement_detected = true;
        } else {
            iteration[i] = 0;
        }
    }
    //morse = (char)(iteration[L_R] + 48); //for debugging

    return movement_detected;
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

    int iteration[NUMBER_OF_MOVEMENTS] = {0}; // [tilt, jolt]

    while (1) {
        if (programState != READY) {
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

            //dataState = READY;      // IF COLLECTING DATA, UNCOMMENT THIS AND THE NEXT LINE
            //continue;               // FOR DATA COLLECTION

            // check for movement
            if (updateMovement(iteration)){
                int i;
                for (i = 0; i < NUMBER_OF_MOVEMENTS; i++) {
                    if (iteration[i] >= REFERENCE_SAMPLE_LENGTH - 3) {
                        switch(i) {
                            case(L_R):
                                morse = '.';
                                playing_melody = dot;
                                break;
                            case(U_D):
                                morse = '-';
                                playing_melody = dash;
                                break;
                            case(R_L):
                                morse = ' ';
                                playing_melody = space;
                                break;
                        }
                        // ensure all movements are back to 0
                        iteration[L_R] = iteration[U_D] = iteration[R_L] = 0;
                        dataState = READY;
                        break;
                    }
                }
            }
            //dataState = READY;
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
    buzzerTaskParams.priority = 2;
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
