#include <mbed.h>
#include <cmath>
#include "drivers/LCD_DISCO_F429ZI.h"

// --- Gyroscope Register and Configurations ---
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01101111
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b00100000
#define OUT_X_L 0x28
#define DEG_TO_RAD (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

#define BUFFER_SIZE 80  // Number of samples in the window (4 seconds at 20 Hz)
#define MA_WINDOW_SIZE 5  // Moving average window size
#define SPI_FLAG 1
#define LEARNING_GESTURES 6  // Number of gestures to learn

EventFlags flags;
void spi_cb(int event) { flags.set(SPI_FLAG); }

// Circular Buffers for data
float gx_buffer[BUFFER_SIZE], gy_buffer[BUFFER_SIZE], gz_buffer[BUFFER_SIZE];
float gx_ma_buffer[MA_WINDOW_SIZE], gy_ma_buffer[MA_WINDOW_SIZE], gz_ma_buffer[MA_WINDOW_SIZE];
int buffer_index = 0, ma_index = 0;

// Gesture Profiles
struct GestureProfile {
    float reference_gx[BUFFER_SIZE];  // Store reference signal for X-axis
    float reference_gy[BUFFER_SIZE];  // Store reference signal for Y-axis
    float reference_gz[BUFFER_SIZE];  // Store reference signal for Z-axis
    float mean_gx, mean_gy, mean_gz;
    float std_gx, std_gy, std_gz;
    float covariance[3];  // Covariance for X-Y, Y-Z, Z-X
    float svm_threshold;
};

GestureProfile gesture_db[LEARNING_GESTURES];
int learned_gestures = 0;

// Thresholds
const float RHO_THRESHOLD = 0.43;  // Normalized covariance threshold
const float SVM_THRESHOLD = 0.2; // SVM threshold for gesture detection

// Function to calculate mean
float compute_mean(float* buffer, int size) {
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

// Function to calculate variance and standard deviation
float compute_std(float* buffer, int size, float mean) {
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += pow(buffer[i] - mean, 2);
    }
    return sqrt(sum / size);
}

// Function to calculate covariance
float compute_covariance(float* buffer1, float* buffer2, int size, float mean1, float mean2) {
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += (buffer1[i] - mean1) * (buffer2[i] - mean2);
    }
    return sum / size;
}

// Moving Average Filter
float apply_moving_average(float* ma_buffer, int ma_size, float new_value) {
    ma_buffer[ma_index] = new_value;
    ma_index = (ma_index + 1) % ma_size;  // Update circular buffer index

    float sum = 0.0f;
    for (int i = 0; i < ma_size; i++) {
        sum += ma_buffer[i];
    }
    return sum / ma_size;
}

// Function to calculate normalized covariance
float compute_normalized_covariance(float* buffer1, float* buffer2, int size, float mean1, float mean2, float std1, float std2) {
    float covariance = compute_covariance(buffer1, buffer2, size, mean1, mean2);
    return covariance / (std1 * std2);
}

// Function to calculate SVM
float compute_svm(float* buffer_x, float* buffer_y, float* buffer_z, int size) {
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += sqrt(pow(buffer_x[i], 2) + pow(buffer_y[i], 2) + pow(buffer_z[i], 2));
    }
    return sum/size;
}

int main() {
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    int8_t write_buf[32], read_buf[32];
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Initialize gyroscope
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    //Initialise LCD 
    LCD_DISCO_F429ZI lcd;
    lcd.Clear(LCD_COLOR_WHITE); // Clear the screen to ensure it's ready for display

    while (1) {
         
        if (learned_gestures < LEARNING_GESTURES) {
            // Learning Mode
            // LEARNING MODE (CENTRE, NORMAL FONT)
            // Display text centered horizontally and vertically
            lcd.Clear(LCD_COLOR_WHITE); // Clear the screen to ensure it's ready for display
            lcd.SetBackColor(LCD_COLOR_WHITE); // Set background color
            lcd.SetTextColor(LCD_COLOR_BLACK); // Set text color

            // Display in the center of the screen
            lcd.SetFont(&Font16);
            lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"LEARNING MODE!", CENTER_MODE);
            printf("LEARNING MODE: Perform gesture %d of %d (5 seconds to prepare)...\n", learned_gestures + 1, LEARNING_GESTURES);
            
            // lcd: GET READY TO PERFORM KEY (1 LINE SPACE, MIDDLE, BIG FONT)
            lcd.SetBackColor(LCD_COLOR_WHITE); // Set background color
            lcd.SetTextColor(LCD_COLOR_BLACK); // Set text color

            // Display in the center of the screen
            lcd.SetFont(&Font24);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"GET READY", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"TO PERFORM", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"KEY", CENTER_MODE);

            thread_sleep_for(5000);
            printf("Perform the gesture now (4 seconds)...\n");
            // lcd: GET READY TO PERFORM KEY (1 LINE SPACE, MIDDLE, BIG FONT)
            lcd.Clear(LCD_COLOR_WHITE); // Clear the screen to ensure it's ready for display
            lcd.SetBackColor(LCD_COLOR_WHITE); // Set background color
            lcd.SetTextColor(LCD_COLOR_BLACK); // Set text color
            // Display in the center of the screen
            lcd.SetFont(&Font16);
            lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"LEARNING MODE!", CENTER_MODE);

            // Display in the center of the screen
            lcd.SetFont(&Font24);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"PERFORM THE", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"KEY", CENTER_MODE);
        } 
        else {
            // Recognition Mode
            printf("RECOGNITION MODE: Get ready to perform a gesture (5 seconds)...\n");
            // Display text centered horizontally and vertically
            lcd.Clear(LCD_COLOR_WHITE); // Clear the screen to ensure it's ready for display
            lcd.SetBackColor(LCD_COLOR_WHITE); // Set background color
            lcd.SetTextColor(LCD_COLOR_BLACK); // Set text color
            // Display in the center of the screen
            lcd.SetFont(&Font16);
            lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"UNLOCK MODE!", CENTER_MODE);
            // Display in the center of the screen
            lcd.SetFont(&Font24);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"GET READY", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"TO PERFORM", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"KEY", CENTER_MODE);


            // UNLOCK MODE (CENTRE, NORMAL FONT)
            // lcd: GET READY TO PERFORM KEY (1 LINE SPACE, MIDDLE, BIG FONT)
            thread_sleep_for(5000);
            printf("Perform the gesture now (4 seconds)...\n");
            lcd.Clear(LCD_COLOR_WHITE); // Clear the screen to ensure it's ready for display
            lcd.SetBackColor(LCD_COLOR_WHITE); // Set background color
            lcd.SetTextColor(LCD_COLOR_BLACK); // Set text color

            lcd.SetFont(&Font16);
            lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"UNLOCK MODE!", CENTER_MODE);
            // Display in the center of the screen
            lcd.SetFont(&Font24);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"PERFORM THE", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"KEY", CENTER_MODE);
        }

        buffer_index = 0;

        // Collect data for 4 seconds
        while (buffer_index < BUFFER_SIZE) {
            uint16_t raw_gx, raw_gy, raw_gz;
            float gx, gy, gz;

            // Read gyroscope data
            write_buf[0] = OUT_X_L | 0x80 | 0x40;
            spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
            flags.wait_all(SPI_FLAG);

            raw_gx = (((uint16_t)read_buf[2]) << 8) | read_buf[1];
            raw_gy = (((uint16_t)read_buf[4]) << 8) | read_buf[3];
            raw_gz = (((uint16_t)read_buf[6]) << 8) | read_buf[5];

            gx = raw_gx * DEG_TO_RAD;
            gy = raw_gy * DEG_TO_RAD;
            gz = raw_gz * DEG_TO_RAD;

            // Apply moving average
            float smoothed_gx = apply_moving_average(gx_ma_buffer, MA_WINDOW_SIZE, gx);
            float smoothed_gy = apply_moving_average(gy_ma_buffer, MA_WINDOW_SIZE, gy);
            float smoothed_gz = apply_moving_average(gz_ma_buffer, MA_WINDOW_SIZE, gz);

            // Store data in buffers
            gx_buffer[buffer_index] = smoothed_gx;
            gy_buffer[buffer_index] = smoothed_gy;
            gz_buffer[buffer_index] = smoothed_gz;

            buffer_index++;
            thread_sleep_for(50);  // 20 Hz sampling
        }

        // Calculate statistics
        float mean_gx = compute_mean(gx_buffer, BUFFER_SIZE);
        float mean_gy = compute_mean(gy_buffer, BUFFER_SIZE);
        float mean_gz = compute_mean(gz_buffer, BUFFER_SIZE);

        float std_gx = compute_std(gx_buffer, BUFFER_SIZE, mean_gx);
        float std_gy = compute_std(gy_buffer, BUFFER_SIZE, mean_gy);
        float std_gz = compute_std(gz_buffer, BUFFER_SIZE, mean_gz);

        if (learned_gestures < LEARNING_GESTURES) {
            // Save gesture profile
            for (int j = 0; j < BUFFER_SIZE; j++) {
                gesture_db[learned_gestures].reference_gx[j] = gx_buffer[j];
                gesture_db[learned_gestures].reference_gy[j] = gy_buffer[j];
                gesture_db[learned_gestures].reference_gz[j] = gz_buffer[j];
            }
            gesture_db[learned_gestures].mean_gx = mean_gx;
            gesture_db[learned_gestures].mean_gy = mean_gy;
            gesture_db[learned_gestures].mean_gz = mean_gz;
            gesture_db[learned_gestures].std_gx = std_gx;
            gesture_db[learned_gestures].std_gy = std_gy;
            gesture_db[learned_gestures].std_gz = std_gz;
            printf("Gesture %d learned successfully!\n", learned_gestures + 1);
            // LCD: LEARNED SUCCESSFULLY 
            learned_gestures++;
        } else {
            // Recognition mode
            float max_rho = 0.0f;
            int recognized_gesture = -1;

            for (int i = 0; i < LEARNING_GESTURES; i++) {
                // Calculate normalized covariance
                float rho_gx = compute_normalized_covariance(gx_buffer, gesture_db[i].reference_gx, BUFFER_SIZE, mean_gx, gesture_db[i].mean_gx, std_gx, gesture_db[i].std_gx);
                if (rho_gx > RHO_THRESHOLD) {
                    max_rho = rho_gx;
                    recognized_gesture = i;
                }
            }

            if (recognized_gesture >= 0) {
                printf("Gesture %d recognized with ρ = %.2f!\n", recognized_gesture + 1, max_rho);
                //LCD: DARK GREEN SCREEN 
                lcd.Clear(LCD_COLOR_DARKGREEN); // Clear the screen to GREEN
                //3 SEC DELAY
                thread_sleep_for(3000);
            } else {
                printf("No gesture recognized.\n");
                //LCD: RED SCREEN
                lcd.Clear(LCD_COLOR_DARKRED); // Clear the screen to RED
                //3 SEC DELAY
                thread_sleep_for(3000);
            }
        }
    }
}