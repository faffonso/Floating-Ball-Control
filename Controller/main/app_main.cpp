/* INCLUDES */

// Components
#include "VL53L0X.h"
#include "ultrasonic.h"
#include "pid_ctrl.h"

// Libs
#include "actuators.h"
#include "MovingAverageFilter.hpp"

// Esp-idf drivers
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

/* MACROS */

#define DEBUG 1        // Verbose print type

// VL53L0X (Laser Sensor)
#define OFFSET 420 // Measure offset in mm
#define I2C_PORT I2C_NUM_0
#define PIN_SDA GPIO_NUM_21
#define PIN_SCL GPIO_NUM_22

// Ultrassonic
#define MAX_DISTANCE_CM 35 
#define TRIGGER_GPIO GPIO_NUM_2
#define ECHO_GPIO GPIO_NUM_4

// PID
#define DELAY 10       

#define Kp 11.1
#define Ki 1.11
#define Kd 111

#define MIN_OUT         0
#define MAX_OUT         8192
#define MAX_INTEGRAL    8192/Ki

/* GLOBAL VARIABLES & FUNCTIONS */
void init();
const char* TAG_M = "main";

VL53L0X sensor(I2C_PORT);
ultrasonic_sensor_t ultrassonic;
pid_ctrl_block_handle_t pid;
MovingAverageFilter movingAvgFilter(22); // Window size


extern "C" void app_main() {
    // Init all modules
    init();

    // Local variables
    float   setpoint,           // Control setpoint
            ultra_result,       // Ultrassonic measure
            error,              // State error
            u;                  // Action control for PWM
    
    uint16_t laser_result = 0;  // Laser measure

    // Main loop
    while(1)
    {
        // Getting ultrassonig measure (setpoint)
        esp_err_t res =  ultrasonic_measure(&ultrassonic, MAX_DISTANCE_CM, &ultra_result);
        
        // Transform measure to mm and apply a moving avage filter
        ultra_result *= 1000;
        setpoint = (ultra_result < 335) ? movingAvgFilter.addData(ultra_result) : movingAvgFilter.getFilteredValue();
        
        // Getting laser measure (feedback value)
        bool read = sensor.read(&laser_result);

        // Calculate the error and apply the PID control
        error = setpoint - (OFFSET - laser_result);
        pid_compute(pid, error, &u);
        update_motor(u, DEBUG);
        
        // Print system informations
        if (DEBUG == 1)
        {
            ESP_LOGI(TAG_M, "Result: %f [mm]", ultra_result);
            ESP_LOGI(TAG_M, "Setpoint: %f [mm]", setpoint);
            ESP_LOGI(TAG_M, "Range: %d [mm]", OFFSET - (int)laser_result);
            ESP_LOGI(TAG_M, "Error: %f [mm]", error);
        }

        vTaskDelay(pdMS_TO_TICKS(DELAY));
    }
}

// Init all system modules (sensors, actuators and controllers)
void init()
{

    // Init H-Bridge
    init_gpio();
    init_pwm();

    // Init Laser Sensor
    sensor.i2cMasterInit(PIN_SDA, PIN_SCL);

    if (!sensor.init()) {
        ESP_LOGE(TAG_M, "Failed to initialize VL53L0X");
        vTaskDelay(portMAX_DELAY);
    }

    // Init Ultrassonic Sensor
    ultrassonic = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&ultrassonic);

    // Init PID
    pid_ctrl_parameter_t pid_param = {
        .kp = Kp,
        .ki = Ki,
        .kd = Kd,
        .max_output = MAX_OUT,
        .min_output = MIN_OUT,
        .max_integral = MAX_INTEGRAL,
        .min_integral = 0,
        .cal_type = PID_CAL_TYPE_POSITIONAL
    };
    pid_ctrl_config_t pid_config;
    pid_config.init_param = pid_param;

    pid_new_control_block(&pid_config, &pid);
}