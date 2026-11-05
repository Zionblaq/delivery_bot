#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"


<<<<<<< HEAD
=======
#define LF_TRIG  GPIO_NUM_35
#define LF_ECHO  GPIO_NUM_36
#define MID_TRIG  GPIO_NUM_1
#define MID_ECHO  GPIO_NUM_2
#define RG_TRIG  GPIO_NUM_16
#define RG_ECHO  GPIO_NUM_15
#define BK_TRIG  GPIO_NUM_37
#define BK_ECHO  GPIO_NUM_38
>>>>>>> 99be0c6 (Hopefully Works)

#define DELAY_MS(ms)       vTaskDelay(pdMS_TO_TICKS(ms))

#define rightSensor     GPIO_NUM_6
#define leftSensor      GPIO_NUM_7
#define midSensor       GPIO_NUM_5

// #define MOTOR_SPEED      200
// #define slow_speed        150




#define ENABLE_RIGHT_MOTOR  GPIO_NUM_14
#define RIGHT_MOTOR_PIN1    GPIO_NUM_13
#define RIGHT_MOTOR_PIN2    GPIO_NUM_12
#define ENABLE_LEFT_MOTOR   GPIO_NUM_9
#define LEFT_MOTOR_PIN1     GPIO_NUM_11
#define LEFT_MOTOR_PIN2     GPIO_NUM_10


#define LF_TRIG  GPIO_NUM_35
#define LF_ECHO  GPIO_NUM_36
#define MID_TRIG  GPIO_NUM_1
#define MID_ECHO  GPIO_NUM_2
#define RG_TRIG  GPIO_NUM_16
#define RG_ECHO  GPIO_NUM_15
#define BK_TRIG  GPIO_NUM_37
#define BK_ECHO  GPIO_NUM_38



#define ULTRASONIC_TIMEOUT_US   25000   
#define ULTRASONIC_GAP_MS       35     
 

constexpr int MOTOR_SPEED = 180;
constexpr int TURN_SPEED  = 100;
constexpr int SLOW_SPEED  = 90;        


constexpr int OBSTACLE_DISTANCE = 25;
constexpr int CLEAR_DISTANCE    = 30;
constexpr int SIDE_CLEARANCE    = 25;
constexpr int BLOCKED_DISTANCE  = 10;
constexpr uint32_t TURN_TIME_MS            = 500;
constexpr uint32_t TURN_LEFT_MS            = TURN_TIME_MS;  
constexpr uint32_t TURN_RIGHT_MS           = TURN_TIME_MS;   
constexpr uint32_t CLEAR_MARGIN_MS         = 300; 
constexpr uint32_t MAX_LEG_MS              = 5000; 
constexpr int      FREE_READINGS           = 2;     
constexpr uint32_t FRONT_CHECK_INTERVAL_MS = 60;    
constexpr int      RETURN_SLOW_START_PCT = 80;      
constexpr uint32_t RETURN_EXTRA_MS       = 1500;    
constexpr uint32_t AXLE_TO_LINE_MS       = 300;     
constexpr uint32_t ALIGN_MIN_MS          = 150;     
constexpr uint32_t ALIGN_TIMEOUT_MS      = 1200;    
constexpr float    APPROX_SPEED_CM_PER_S = 25.0f;  



#define DEBUG_PRINT_DISTANCES       1
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 300;


constexpr int      OBSTACLE_CONFIRM_READINGS = 2;     
constexpr int      SIDE_RECHECKS             = 5;     
constexpr uint32_t SIDE_RECHECK_DELAY_MS     = 500;   
constexpr uint32_t SEEN_TIMEOUT_MS           = 1500;  
constexpr int RIGHT_TRIM_PCT = 100;
constexpr int LEFT_TRIM_PCT  = 100;

constexpr uint32_t SEARCH_STEP_MS  = 150;   
constexpr uint32_t SEARCH_CREEP_MS = 250; 
constexpr int      SEARCH_ROUNDS   = 4; 


#define PWM_FREQ_HZ         1000
#define PWM_MODE            LEDC_LOW_SPEED_MODE
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_RESOLUTION      LEDC_TIMER_8_BIT
#define RIGHT_PWM_CHANNEL   LEDC_CHANNEL_0
#define LEFT_PWM_CHANNEL    LEDC_CHANNEL_1
