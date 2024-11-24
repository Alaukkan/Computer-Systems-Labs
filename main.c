/*
 * main.c
 *
 *  Created on: 20 Oct 2024
 *      Authors: Alex Laukkanen & Anniina Viskari
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
#define TASK_SLEEP 50000 // microseconds

#define MAX_RECIEVED_MORSE_STRING_LEN 100
#define BUFFER_SAMPLE_LENGTH 3
#define REFERENCE_DATA_LENGTH 8
#define NUMBER_OF_MOVEMENTS 3
#define BLOCK_SIZE 3

#define ACCEL_THRESHOLD 0.45f
#define GYRO_THRESHOLD 150

Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE * 2];
Char buzzerTaskStack[STACKSIZE];

enum state { WAITING=0, READY, RECIEVING_WAITING, RECIEVING };
enum state programState = WAITING;
enum state dataState = WAITING;

enum MovementType { R_L=0, F_B, TWIST };

struct mpu_sample_t sensor_buffer[BUFFER_SAMPLE_LENGTH]; // buffer saves last 3 recorded data samples
uint8_t uartBuffer[30]; // Receive buffer

struct melody_t * playing_melody;

char morse;

char recieved_string[MAX_RECIEVED_MORSE_STRING_LEN];
int recieved_string_len = 0;
float last_morse_time = 0;

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
void calculateMean(const struct mpu_sample_t sample[], struct mpu_sample_t *mean, int startIndex, int n, int sample_length);
bool sampleCompare(struct mpu_sample_t *a, struct mpu_sample_t *b);
bool updateMovement(int iteration[]);
Void sensorInit(I2C_Handle *i2c, I2C_Params *i2cParams);


void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    if (programState == WAITING) {
        PIN_setOutputValue(ledHandle, Board_LED0, 1);
        playing_melody = startup;
        programState = READY;

    } else if (programState == READY) {
        PIN_setOutputValue(ledHandle, Board_LED0, 0);
        playing_melody = shutdown;
        programState = WAITING;

    } else if (programState == RECIEVING_WAITING) { // waits and rings until user presses button
        playing_melody = NULL;
        programState = RECIEVING; // uartTask will start showing the morse code
    }
}

Void buzzerTaskFxn(UArg arg0, UArg arg1) {
    while (1) {
        if (playing_melody != NULL) {
            buzzerOpen(hBuzzer);
            int i = 0;

            while (playing_melody[i].duration > 0 && playing_melody != NULL) {
                if (playing_melody[i].frequency == 0) { // if freq is 0 no note is played, can be used as pauses in the melodies
                    buzzerClose();
                    Task_sleep(playing_melody[i].duration * 1000 / Clock_tickPeriod);
                    buzzerOpen(hBuzzer);
                } else {
                    buzzerSetFrequency(playing_melody[i].frequency);
                    Task_sleep(playing_melody[i].duration * 1000 / Clock_tickPeriod);
                }
                i++;
            }

            buzzerClose();
            if (programState != RECIEVING_WAITING) { // repeating "phone call"
                playing_melody = NULL;
            }
        }
        Task_sleep(TASK_SLEEP / Clock_tickPeriod);
    }
}


void uartFxn(UART_Handle uart, void *rxBuf, size_t len) {
    uint8_t recieved_morse = *(uint8_t*)rxBuf; //cast and dereference
    //only valid morse characters and check is the array recieved_string has room
    if ((recieved_morse == '.' || recieved_morse == '-' || recieved_morse == ' ') && recieved_string_len < MAX_RECIEVED_MORSE_STRING_LEN) {
        //detecting spaces with timing of the morse code
        uint32_t ticks = Clock_getTicks();
        float this_morse_time = (float) ticks * (Clock_tickPeriod / 1000000.0);
        float time_diff = this_morse_time - last_morse_time;

        if (time_diff > 1 && time_diff < 2) { // pause in the morse code = space
            recieved_string[recieved_string_len] = ' ';
            recieved_string_len++;
        }

        recieved_string[recieved_string_len] = (char)recieved_morse;
        recieved_string_len++;
        last_morse_time = this_morse_time;

        if (programState != RECIEVING) { // in case user is in the midst of "reading" message at the same time it is still sending it
            programState = RECIEVING_WAITING;
            playing_melody = recieving;
        }
    }
    UART_read(uart, rxBuf, 1);
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
    int i = 0;

    /* IF NOT USING SERIAL.CLIENT
    char curr_message[7];
    char translated_message[64];
    // ensure proper initialization
    curr_message[0] = '\0';
    translated_message[0] = '\0';

    int curr_message_len = 0;
    int translated_message_len = 0;
    int whitespaces = 0;
    IF NOT USING THE SERIAL.CLIENT ENDS */

    UART_read(uart, uartBuffer, 1);

    while (1) {

        /* FOR SENDING MORSE */
        if (dataState == READY) {

            if (morse != '\0') {
                sprintf(morse_str, "%c\r\n", morse);
                UART_write(uart, morse_str, 4);


                /* IF NOT USING THE SERIAL.CLIENT
                if (morse == ' ') { // if a word is completed, translate morse to latin
                    // track number of whitespaces in a row
                    whitespaces++;

                    // check how many whitespaces have been sent in a row
                    if (whitespaces == 1) { // translate single character from curr_message into translated_message
                        char translation = translate(curr_message);
                        if (translation != '\0'){
                            translated_message[translated_message_len + 1] = '\0';
                            translated_message[translated_message_len] = translation;
                            translated_message_len++;
                        } else { // if there is no translation, reset and send info to user
                            char str[37];
                            sprintf(str, "RESET: UNIDENTIFIED MORSE (%s)\r\n", curr_message);
                            UART_write(uart, str, strlen(str) + 1);
                            Task_sleep(300000/ Clock_tickPeriod); // 300ms to avoid overlapping buzzer
                            playing_melody = error;
                        }
                        // reset curr_message
                        curr_message[0] = '\0';
                        curr_message_len = 0;

                    } else if (whitespaces == 2) { // add a whitespace to translated_message
                        translated_message[translated_message_len + 1] = '\0'; // move null terminator
                        translated_message[translated_message_len] = ' ';
                        translated_message_len++;

                    } else if (whitespaces == 3) { // show the translated message
                        translated_message[translated_message_len] = '\r'; // add newline and move null terminator
                        translated_message[translated_message_len + 1] = '\n';
                        translated_message[translated_message_len + 2] = '\0';
                        UART_write(uart, translated_message, strlen(translated_message) + 1);
                        // reset translated_message
                        translated_message[0] = '\0';
                        translated_message_len = 0;
                    }
                } else if (curr_message_len < 5) { // add morse char to current recorded line of morse code
                    curr_message[curr_message_len + 1] = '\0'; // move null terminator
                    curr_message[curr_message_len] = morse;
                    curr_message_len++;
                    whitespaces = 0;
                } else { // if morse code is too long to be any Latin character or number, reset current recorded line of morse code
                    char str[32];
                    sprintf(str, "RESET: MORSE TOO LONG (%s)\n\r", curr_message);
                    UART_write(uart, str, strlen(str) + 1);
                    Task_sleep(300000/ Clock_tickPeriod); // 300ms to avoid overlapping buzzer
                    playing_melody = error;
                    curr_message[0] = '\0';
                    curr_message_len = 0;
                    whitespaces = 0;
                }
                IF NOT USING THE SERIAL.CLIENT ENDS */
            }

            morse = '\0';
            dataState = WAITING;

            /* FOR COLLECTING DATA
            char str1[64];
            sprintf(str1, "%f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", sensor_buffer[2].timestamp, sensor_buffer[2].accel.ax, sensor_buffer[2].accel.ay, sensor_buffer[2].accel.az, sensor_buffer[2].gyro.gx, sensor_buffer[2].gyro.gy, sensor_buffer[2].gyro.gz);
            UART_write(uart, str1, strlen(str1) + 1); */

        }
        /* IF NOT USING THE SERIAL.CLIENT
        if (programState == WAITING) {
            //pressing the button resets the morse character being written
            if (curr_message_len > 0) {
                char str[26];
                sprintf(str, "MANUAL RESET (%s)\r\n", curr_message);
                UART_write(uart, str, strlen(str) + 1);
                curr_message[0] = '\0';
                curr_message_len = 0;
            }
            morse = '\0';
        }
        IF NOT USING THE SERIAL.CLIENT ENDS */

        // When recieving morse code
        if (programState == RECIEVING) {
            Task_sleep(1000000/ Clock_tickPeriod);

            // printing for debugging
            // sprintf(morse_str, "%c\r\n", recieved_string[i]);
            // UART_write(uart, morse_str, 4);

            switch(recieved_string[i]) {
                case(' '):
                    Task_sleep(1000000/ Clock_tickPeriod); // space is just an extra pause
                    break;
                case('.'):
                    playing_melody = dot;
                    break;
                case('-'):
                    playing_melody = dash;
                    break;
                default:  // in case there has slipped some other character, shouldn't be possible
                    playing_melody = error;
                    break;
            }

            i++;
            if (i == recieved_string_len) {
                Task_sleep(1000000/ Clock_tickPeriod); // wait for the last morse to be heard
                playing_melody = space; // space sound to indicate end of message
                // reset everything and set programState to WAITING
                recieved_string_len = 0;
                i = 0;
                PIN_setOutputValue(ledHandle, Board_LED0, 0);
                programState = WAITING;
            }
        }

        // Once per 75ms
        Task_sleep(TASK_SLEEP / Clock_tickPeriod);
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

void calculateMean(const struct mpu_sample_t sample[], struct mpu_sample_t *mean, int startIndex, int n, int sample_length) {
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
    /* Always calculates mean of the last 3 samples in sensor_buffer
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
    if (iteration[R_L] == 0 && iteration[F_B] == 0 && iteration[TWIST] == 0) { // no movements detected yet
        //calculate all, for loop doesn't skip anything
        skip[R_L] = skip[F_B] = skip[TWIST] = false;

    } else { // at least one movement has been "detected"
        // figure out which movements need to be skipped and which ones need to be checked
        if (iteration[R_L] == 0) {
            skip[R_L] = true;
        }
        if (iteration[F_B] == 0) {
            skip[F_B] = true;
        }
        if (iteration[TWIST] == 0) {
            skip[TWIST] = true;
        }

    }
    int i;
    for (i = 0; i < NUMBER_OF_MOVEMENTS; i++) {
        if (skip[i]) { // check if movement check needs to be skipped
            continue;
        }
        // calculate an iteration of rolling mean of data
        calculateMean((*reference_data)[i], &data_mean, iteration[i], BLOCK_SIZE, REFERENCE_DATA_LENGTH);

        //compare data and buffer
        if (sampleCompare(&buffer_mean, &data_mean)) {
            iteration[i]++;
            movement_detected = true;
        } else {
            iteration[i] = 0;
        }
    }

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

    int iteration[NUMBER_OF_MOVEMENTS] = {0}; // {R_L, F_B, TWIST}

    while (1) {
        if (programState != READY) {
            Task_sleep(TASK_SLEEP / Clock_tickPeriod);
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
                    if (iteration[i] > REFERENCE_DATA_LENGTH - BLOCK_SIZE) {
                        switch(i) {
                            case(R_L):
                                morse = '.';
                                playing_melody = dot;
                                break;
                            case(F_B):
                                morse = '-';
                                playing_melody = dash;
                                break;
                            case(TWIST):
                                morse = ' ';
                                playing_melody = space;
                                break;
                        }
                        // ensure all movements are back to 0
                        iteration[R_L] = iteration[F_B] = iteration[TWIST] = 0;
                        dataState = READY;
                        break;
                    }
                }
            }
        }

        // Once per 75ms
        Task_sleep(TASK_SLEEP / Clock_tickPeriod);
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
