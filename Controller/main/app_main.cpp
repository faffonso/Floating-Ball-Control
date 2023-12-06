#include "VL53L0X.h"
#include "ultrasonic.h"
#include "pid_ctrl.h"

#include "actuators.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#define DEBUG 1
#define DELAY 500

#define MAX_DISTANCE_CM 500 // 5m max

#define DEBUG_ACTUATOR      0
#define DEBUG_ULTRASSONIC   0
#define DEBUG_VL53L0X       0
#define DEBUG_CONTROL       0



extern "C" void app_main() {

    #if DEBUG_ACTUATOR
        
        init_gpio();
        init_pwm();
        uint16_t u = 800;

        //Inf Loop
        while(1)
        {
            // Ascendent motor speed
            for (int i=0; i<10; i++)
            {
                update_motor(u * i, DEBUG);
                vTaskDelay( DELAY / portTICK_PERIOD_MS);
            }
        }

    #endif

    #if DEBUG_ULTRASSONIC
        #define TRIGGER_GPIO GPIO_NUM_17
        #define ECHO_GPIO GPIO_NUM_16


        ultrasonic_sensor_t sensor = {
            .trigger_pin = TRIGGER_GPIO,
            .echo_pin = ECHO_GPIO
        };

        ultrasonic_init(&sensor);

        while (true)
        {
            float distance;
            esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
            if (res != ESP_OK)
            {
                printf("Error %d: ", res);
                switch (res)
                {
                    case ESP_ERR_ULTRASONIC_PING:
                        printf("Cannot ping (device is in invalid state)\n");
                        break;
                    case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                        printf("Ping timeout (no device found)\n");
                        break;
                    case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                        printf("Echo timeout (i.e. distance too big)\n");
                        break;
                    default:
                        printf("%s\n", esp_err_to_name(res));
                }
            }
            else
                printf("Distance: %0.04f m\n", distance);

            vTaskDelay(pdMS_TO_TICKS(500));
        }
    #endif

    #if DEBUG_VL53L0X
        #define I2C_PORT I2C_NUM_0
        #define PIN_SDA GPIO_NUM_21
        #define PIN_SCL GPIO_NUM_22

        VL53L0X sensor(I2C_PORT);
        sensor.i2cMasterInit(PIN_SDA, PIN_SCL);

        if (!sensor.init()) {
            ESP_LOGE(TAG, "Failed to initialize VL53L0X");
            vTaskDelay(portMAX_DELAY);
        }

        // Measurement
        while (1) {
            //Read value
            uint16_t result_mm = 0;
            
            //Start time
            TickType_t tick_start = xTaskGetTickCount();

            //Reading sensor
            bool read = sensor.read(&result_mm);
            
            //Calculate time spend 
            TickType_t tick_end = xTaskGetTickCount();
            int took_ms = ((int)tick_end - tick_start) / portTICK_PERIOD_MS;
            
            //Print measure
            if (read)
                ESP_LOGI(TAG, "Range: %d [mm] took %d [ms]", (int)result_mm, took_ms);
            else
                ESP_LOGE(TAG, "Failed to measure :(");
        }


    #endif

    #if DEBUG_CONTROL
        #define Kp 1
        #define Ki 1
        #define Kd 1

        #define MAX_OUT         100
        #define MAX_INTEGRAL    1000

        pid_ctrl_block_handle_t pid;
        pid_ctrl_parameter_t pid_param = {
            .kp = Kp,
            .ki = Ki,
            .kd = Kd,
            .max_output = MAX_OUT,
            .min_output = 0,
            .max_integral = MAX_INTEGRAL,
            .min_integral = 0,
            .cal_type = PID_CAL_TYPE_POSITIONAL
        };
        pid_ctrl_config_t pid_config;
        pid_config.init_param = pid_param;

        pid_new_control_block(&pid_config, &pid);

        float error, u;
        while(1)
        {
            error = 10.5;
            pid_compute(pid, error, &u);
            ESP_LOGI(TAG, "Action control: %f.2", u);

            error = 10.5 - u;
            vTaskDelay(pdMS_TO_TICKS(DELAY));
        }
    #endif
}