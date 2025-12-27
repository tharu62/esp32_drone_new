#include "motor_controller.h"

float throttle_input = 0.0f;
float roll_input = 0.0f;
float pitch_input = 0.0f;
float yaw_input = 0.0f;

// Maximum resolution for motor speed (
// educated guess based on :
// (max_desired_angle - current_angle = 360.0f)*PID_GAIN = max_desired_rotation_rate [deg/sec] => estimated max angle_pid_controller_output error = 360.0f [deg],
// (max_desired_rotation_rate - current_rotation_rate = 360.0f)*PID_GAIN => estimated max rotation_rate_pid_controller error = 360.0f [deg/sec], 
//  max motor speed command = 360.0f => motor speed percent = (360.0f/360.0f)*100% = 100% .
// )
#define MAX_RES 360.0f 

#define PWM_GPIO_M1        33
#define PWM_GPIO_M2        34
#define PWM_GPIO_M3        35
#define PWM_GPIO_M4        36
#define PWM_FREQ_HZ     20000   // 20 kHz
#define TIMER_RES_HZ    1000000 // 1 MHz resolution

static mcpwm_timer_handle_t timer;
static mcpwm_oper_handle_t oper;
static mcpwm_cmpr_handle_t comparator;
static mcpwm_gen_handle_t generator;

// @todo: Implement motor controller initialization for all motors
void motor_controller_init(void)
{
    throttle_input = 0.0f;
    roll_input = 0.0f;
    pitch_input = 0.0f;
    yaw_input = 0.0f;

    /* 1. Create MCPWM timer */
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TIMER_RES_HZ,
        .period_ticks = TIMER_RES_HZ / PWM_FREQ_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    /* 2. Create operator */
    mcpwm_operator_config_t oper_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    /* 3. Create comparator */
    mcpwm_comparator_config_t cmp_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_config, &comparator));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0));

    /* 4. Create generator (GPIO output) */
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = PWM_GPIO_M1,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &generator));

    /* 5. Generator actions */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            comparator,
            MCPWM_GEN_ACTION_LOW)));

    /* 6. Start timer */
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    return;
}

// @todo: Implement motor speed setting logic for all motors
void motor_set_speed_percent(float percent, int motor_index)
{
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    uint32_t period = TIMER_RES_HZ / PWM_FREQ_HZ;
    uint32_t compare = (uint32_t)((percent / 100.0f) * period);

    ESP_ERROR_CHECK(
        mcpwm_comparator_set_compare_value(comparator, compare)
    ); 
}

// @todo: Implement motor controller logic for all motors
void motor_controller(float throttle, float* rotation_rate_output)
{
    float motor1_pwm = throttle + rotation_rate_output[1] + rotation_rate_output[0] - rotation_rate_output[2]; // Front Left
    float motor2_pwm = throttle + rotation_rate_output[1] - rotation_rate_output[0] + rotation_rate_output[2]; // Front Right
    float motor3_pwm = throttle - rotation_rate_output[1] - rotation_rate_output[0] - rotation_rate_output[2]; // Rear Right
    float motor4_pwm = throttle - rotation_rate_output[1] + rotation_rate_output[0] + rotation_rate_output[2]; // Rear Left


    motor_set_speed_percent((motor1_pwm/MAX_RES)*100.0f, 0);
    // motor_set_speed_percent((motor2_pwm/MAX_RES)*100.0f, 1);
    // motor_set_speed_percent((motor3_pwm/MAX_RES)*100.0f, 2);
    // motor_set_speed_percent((motor4_pwm/MAX_RES)*100.0f, 3);
    // ESP_LOGI("pwm : ", "%f [%%]", (motor1_pwm/MAX_RES)*100.0f);
    return;
}