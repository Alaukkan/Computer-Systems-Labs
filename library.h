/*
 * data.h
 *
 *  Created on: 24 Oct 2024
 *      Author: alexl
 */

#ifndef LIBRARY_H_
#define LIBRARY_H_

struct accelerometer_t {
    float ax, ay, az;
};

struct gyroscope_t {
    float gx, gy, gz;
};

struct mpu_sample_t {
    float timestamp; //timestamp mostly used for analyzing data
    struct accelerometer_t accel;
    struct gyroscope_t gyro;
};

struct melody_t {
    int frequency;
    int duration; // in milliseconds
};

struct dictionary_t {
    char latin;
    char morse[6];
};


// Reference data for movement detection for the movement "tilt"
struct mpu_sample_t stoop_data[11] = {
    {0.46279, {-1.83000, -0.05500, 0.61500}, {6.49000, -36.33000, -31.85500}},
    {0.56411, {-1.08000, -0.04500, 0.73000}, {21.63500, -69.24000, -24.79500}},
    {0.66545, {-0.05500, 0.10000, 0.49500}, {21.55500, -26.67500, -4.42000}},
    {0.76680, {-0.31500, 0.00000, 0.88500}, {-10.80500, 2.88000, -3.37000}},
    {0.86806, {-0.97000, -0.05000, 0.92500}, {-17.13500, 34.40000, 12.98000}},
    {0.96941, {-1.29000, -0.14000, 1.02500}, {-18.36500, 39.31500, 7.19000}},
    {1.07064, {-1.12000, -0.67000, 0.65500}, {15.12000, 28.72000, -8.07500}},
    {1.19695, {-0.96000, -0.46500, 0.79500}, {73.38000, 7.65500, 22.58500}},
    {1.32326, {-0.89000, 0.86000, 0.67500}, {19.33500, -11.84500, 19.64500}},
    {1.44960, {-1.02000, 0.33500, 0.62500}, {-21.18000, -5.70500, 7.71000}},
    {1.57595, {-0.95500, 0.24500, 0.66500}, {-26.97500, 0.16000, 10.33000}}
};

// Reference data for movement detection for the movement "circle"
struct mpu_sample_t circle_data[11] = {
    {0.74145, {-0.895, -0.335, -0.135}, {20.285, 2.885, 0.445}},
    {0.84275, {-1.145, -0.295, -0.230}, {65.735, 7.670, -0.205}},
    {0.94407, {-1.410, 0.045, -0.235}, {38.130, -12.690, 20.000}},
    {1.04538, {-1.340, 0.650, -0.115}, {43.440, -57.830, -3.735}},
    {1.14668, {-0.820, 0.490, 0.080}, {-7.710, -62.285, -2.330}},
    {1.24800, {-0.500, 0.265, 0.145}, {-30.115, -29.735, 7.210}},
    {1.34930, {-0.455, -0.010, -0.035}, {-39.805, 10.765, -38.065}},
    {1.45062, {-0.590, -0.320, -0.130}, {-49.680, 28.590, -26.290}},
    {1.55191, {-0.785, -0.510, -0.245}, {-29.325, 24.410, -32.395}},
    {1.65323, {-1.120, -0.600, -0.230}, {-22.965, 38.435, 0.360}},
    {1.75455, {-1.115, -0.115, -0.180}, {5.540, 14.175, 35.990}}
};

struct mpu_sample_t left_and_right[7] = {
    {44.13351, {-0.05000, -0.79667, -1.11667}, {-0.44333, -1.85667, -52.01333}},
    {44.23483, {-0.05333,  0.55333, -1.03667}, {-1.93333, -2.30000, -85.83000}},
    {44.33614, {-0.08333,  0.92000, -1.00333}, { 0.33333, -0.48667,  18.72000}},
    {44.43744, {-0.11000,  0.16667, -1.12667}, { 1.38000, -0.17333, 104.77333}},
    {44.53875, {-0.01000, -0.48333, -1.06667}, {-1.19333, -0.70333,  23.88667}},
    {44.64005, { 0.01000,  0.00667, -1.01333}, { 0.26000, -0.39000,  -0.53333}},
    {44.72455, { 0.01333,  0.00333, -1.03000}, { 0.53333, -0.51333,  -0.08333}}
};

struct mpu_sample_t up_and_down[7] = {
    {14.60594, { 0.38333,  0.01667, -0.74000}, {-0.65000, -2.02000,  2.24333}},
    {14.70727, { 0.26000,  0.04333, -0.78667}, {-0.99333, -4.65333,  6.49000}},
    {14.79188, {-0.30333,  0.01000, -0.74667}, {-1.84333, -2.00667,  5.27000}},
    {14.89321, {-0.51667, -0.02667, -0.77667}, {-0.50333, -1.50000,  2.04667}},
    {14.99451, {-0.11667,  0.02333, -0.78333}, { 0.89333, -1.26000, -7.66333}},
    {15.09581, { 0.34000,  0.03333, -0.79667}, { 0.39667, -0.22667, -3.09000}},
    {15.19713, { 0.11333,  0.01000, -0.72000}, { 0.65667, -1.75000, -1.41000}}
};

struct mpu_sample_t right_and_left[7] = {
    //{57.28867, {-0.02667, 1.26333, -0.44333}, {3.99333, -1.00000, 71.49333}},
    //{57.39000, {-0.28667, -1.08000, -0.47333}, {0.18333, 3.51000, 154.70333}},
    //{57.49131, {-0.08667, -1.04000, -0.44000}, {-3.61000, -2.60333, -115.96667}},
    //{57.59263, {-0.21000, 0.75333, -0.46000}, {2.74667, -1.70333, -151.64333}},
    //{57.69396, {-0.01000, 0.06667, -0.43667}, {2.46000, 0.09000, 7.24000}},
    //{57.79527, {-0.01667, 0.01667, -0.42000}, {-1.57667, -1.12667, 0.81333}},
    //{57.87988, {-0.01333, 0.01667, -0.41667}, {0.20667, -0.52000, 1.83667}}

    {14.60594, {-0.38333,  0.01667, -0.74000}, {-0.65000, -2.02000,  2.24333}},
    {14.70727, {-0.26000,  0.04333, -0.78667}, {-0.99333, -4.65333,  6.49000}},
    {14.79188, { 0.30333,  0.01000, -0.74667}, {-1.84333, -2.00667,  5.27000}},
    {14.89321, { 0.51667, -0.02667, -0.77667}, {-0.50333, -1.50000,  2.04667}},
    {14.99451, { 0.11667,  0.02333, -0.78333}, { 0.89333, -1.26000, -7.66333}},
    {15.09581, {-0.34000,  0.03333, -0.79667}, { 0.39667, -0.22667, -3.09000}},
    {15.19713, {-0.11333,  0.01000, -0.72000}, { 0.65667, -1.75000, -1.41000}}
};


struct mpu_sample_t* reference_data[3][7] = {left_and_right, up_and_down, right_and_left};



// different melodies
struct melody_t startup[3] = {
    {400, 100},
    {1000, 100},
    {0, 0}
};

struct melody_t shutdown[4] = {
    {1000, 100},
    {800, 100},
    {400, 200},
    {0, 0}
};

struct melody_t dot[2] = {
    {1000, 50},
    {0, 0}
};

struct melody_t dash[2] = {
    {1000, 200},
    {0, 0}
};

struct melody_t space[4] = {
    {1000, 50},
    {400, 50},
    {600, 100},
    {0, 0}
};

struct melody_t error[2] = {
    {100, 300},
    {0, 0}
};

struct melody_t recieving[15] = {
    {659, 100},
    {587, 100},
    {369, 200},
    {415, 200},
    {554, 100},
    {494, 100},
    {293, 200},
    {330, 200},
    {494, 100},
    {440, 100},
    {277, 200},
    {330, 200},
    {440, 400},
    {0, 400},
    {0, 0}
};


// dictionary for translating morse to Latin and eventually Latin to morse
struct dictionary_t translation[36] = {
    {'A', ".-"},
    {'B', "-..."},
    {'C', "-.-."},
    {'D', "-.."},
    {'E', "."},
    {'F', "..-."},
    {'G', "--."},
    {'H', "...."},
    {'I', ".."},
    {'J', ".---"},
    {'K', "-.-"},
    {'L', ".-.."},
    {'M', "--"},
    {'N', "-."},
    {'O', "---"},
    {'P', ".--."},
    {'Q', "--.-"},
    {'R', ".-."},
    {'S', "..."},
    {'T', "-"},
    {'U', "..-"},
    {'V', "...-"},
    {'W', ".--"},
    {'X', "-..-"},
    {'Y', "-.--"},
    {'Z', "--.."},
    {'0', "-----"},
    {'1', ".----"},
    {'2', "..---"},
    {'3', "...--"},
    {'4', "....-"},
    {'5', "....."},
    {'6', "-...."},
    {'7', "--..."},
    {'8', "---.."},
    {'9', "----."}
};

/* apparently all this logic is unnecessary
 *
 *pressing the button resets the morse character being written
            if (curr_message_len > 0) {
                //char str[26];
                //sprintf(str, "MANUAL RESET (%s)\r\n", curr_message);
                //UART_write(uart, str, strlen(str) + 1);
                curr_message[0] = '\0';
                curr_message_len = 0;
            }
            morse = '\0';
    char curr_message[7];
    char translated_message[64];
    // ensure proper initialization
    curr_message[0] = '\0';
    translated_message[0] = '\0';

    int curr_message_len = 0;
    int translated_message_len = 0;
    int whitespaces = 0;
                     *
                    // if a word is completed, translate morse to latin
                    if (morse_str[0] == ' ') {
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
                                char str[40];
                                sprintf(str, "RESET: UNIDENTIFIED MORSE (%s)\r\n", curr_message);
                                UART_write(uart, str, strlen(str) + 1);
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
                        playing_melody = error;
                        curr_message[0] = '\0';
                        curr_message_len = 0;
                        whitespaces = 0;
                    }*/



/* IGNORE THIS, OLD MOVEMENT DETECTION
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
}*/

#endif /* LIBRARY_H_ */
