#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "bdc_motor.h"

static const char *TAG = "forward_loop_test";


// PWM Configuration (Motor 1 and 2)
#define ENCODER_PULSES_PER_ROT       1550
#define BDC_MCPWM_TIMER_RESOLUTION_HZ  10000000 // 10MHz
#define BDC_MCPWM_FREQ_HZ              25000    // 25kHz
#define BDC_MCPWM_DUTY_TICK_MAX        400      // 100%

// Encoder 1 configuration (Motor 1)
#define ENCODER_GPIO_A                36
#define ENCODER_GPIO_B                35

// Encoder 2 configuration (Motor 2)
#define ENCODER2_GPIO_A               38
#define ENCODER2_GPIO_B               37

// Motor 1 PWM configuration (DRV8871)
#define BDC_MCPWM_GPIO_A               4
#define BDC_MCPWM_GPIO_B               5

// Motor 2 PWM configuration (DRV8871)
#define BDC2_MCPWM_GPIO_A              6
#define BDC2_MCPWM_GPIO_B              7

// PCNT units for encoders
static pcnt_unit_handle_t g_pcnt_unit = NULL;   // Motor 1 encoder
static pcnt_unit_handle_t g_pcnt_unit2 = NULL;  // Motor 2 encoder

typedef struct {
	bdc_motor_handle_t motor;
	int target_pulses;
	uint32_t duty_ticks;
	pcnt_unit_handle_t pcnt_unit; // Associated encoder counter unit
	int motor_id;                 // 1 or 2 for logging clarity
} rotate_args_t;

static void motor_rotate_n_task(void *arg)
{
	rotate_args_t *cfg = (rotate_args_t *)arg;
	bdc_motor_handle_t motor = cfg->motor;
	const int target_pulses = cfg->target_pulses; // 5 * ENCODER_PULSES_PER_ROT (5 * 1550 = 7750)
	const uint32_t duty_ticks = cfg->duty_ticks;
	pcnt_unit_handle_t pcnt = cfg->pcnt_unit;
	const int motor_id = cfg->motor_id;
	free(cfg);

	// Start from zero so PCNT count directly maps to pulses since start
	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt));
	ESP_ERROR_CHECK(bdc_motor_set_speed(motor, duty_ticks));
	ESP_ERROR_CHECK(bdc_motor_forward(motor));

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10));
		int count = 0;
		ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt, &count));
		float rotations = fabsf((float)count / (float)ENCODER_PULSES_PER_ROT);
		ESP_LOGI(TAG, "Motor %d: count=%d rot=%.2f/5.00 (PPR=%d target=%d)", motor_id, count, rotations, ENCODER_PULSES_PER_ROT, target_pulses);
		if (rotations >= 5.0f || abs(count) >= target_pulses) {
			// Immediately force both outputs low
			ESP_ERROR_CHECK(bdc_motor_set_speed(motor, 0));
			ESP_ERROR_CHECK(bdc_motor_coast(motor));
			break;
		}
	}

	// Ensure outputs remain low after completion
	int final_count = 0; ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt, &final_count));
	ESP_ERROR_CHECK(bdc_motor_set_speed(motor, 0));
	ESP_ERROR_CHECK(bdc_motor_coast(motor));
	ESP_LOGI(TAG, "Motor %d: Stopped at count=%d (~%.2f rotations, PPR=%d, target=%d).", motor_id,
		final_count, fabsf((float)final_count / (float)ENCODER_PULSES_PER_ROT), ENCODER_PULSES_PER_ROT, target_pulses);

	vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ROTATE EXACTLY 5 REV FORWARD USING PCNT (PPR=%d) ===", ENCODER_PULSES_PER_ROT);

    // Initialize PCNT for encoder - Motor 1
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &g_pcnt_unit));

    // Quadrature decoding using two channels (A/B)
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num = ENCODER_GPIO_A,
        .level_gpio_num = ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit, &chan_a_cfg, &chan_a));
    // Count both edges on A; direction from B level
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num = ENCODER_GPIO_B,
        .level_gpio_num = ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit, &chan_b_cfg, &chan_b));
    // Opposite mapping on B to realize X4
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Optional: add a small glitch filter to debounce
    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 500, // ignore pulses shorter than 500ns
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(g_pcnt_unit, &filter_cfg));

    ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt_unit));

    // Initialize PCNT for encoder - Motor 2
    pcnt_unit_config_t unit2_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit2_config, &g_pcnt_unit2));

    // Quadrature decoding using two channels (A/B) for Motor 2
    pcnt_chan_config_t chan2_a_cfg = {
        .edge_gpio_num = ENCODER2_GPIO_A,
        .level_gpio_num = ENCODER2_GPIO_B,
    };
    pcnt_channel_handle_t chan2_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit2, &chan2_a_cfg, &chan2_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan2_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan2_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    pcnt_chan_config_t chan2_b_cfg = {
        .edge_gpio_num = ENCODER2_GPIO_B,
        .level_gpio_num = ENCODER2_GPIO_A,
    };
    pcnt_channel_handle_t chan2_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit2, &chan2_b_cfg, &chan2_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan2_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan2_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Optional: same glitch filter for Motor 2
    pcnt_glitch_filter_config_t filter2_cfg = {
        .max_glitch_ns = 500,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(g_pcnt_unit2, &filter2_cfg));

    ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt_unit2));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_unit2));
    ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt_unit2));

    // Initialize motor driver (MCPWM) - Motor 1
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));

    // Initialize motor driver (MCPWM) - Motor 2
    bdc_motor_config_t motor2_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC2_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC2_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm2_config = {
        .group_id = 0, // same MCPWM group as Motor 1 for synchronous timing base
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor2 = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor2_config, &mcpwm2_config, &motor2));
    ESP_ERROR_CHECK(bdc_motor_enable(motor2));

    // Create tasks to rotate exactly 5 rotations forward for both motors
    const uint32_t duty_ticks = 280; // 70%
    rotate_args_t *args1 = (rotate_args_t *)malloc(sizeof(rotate_args_t));
    args1->motor = motor;
    args1->target_pulses = 5 * ENCODER_PULSES_PER_ROT;
    args1->duty_ticks = duty_ticks;
    args1->pcnt_unit = g_pcnt_unit;
    args1->motor_id = 1;
    xTaskCreate(motor_rotate_n_task, "rotate5_fwd_m1", 4096, args1, 5, NULL);

    rotate_args_t *args2 = (rotate_args_t *)malloc(sizeof(rotate_args_t));
    args2->motor = motor2;
    args2->target_pulses = 5 * ENCODER_PULSES_PER_ROT;
    args2->duty_ticks = duty_ticks;
    args2->pcnt_unit = g_pcnt_unit2;
    args2->motor_id = 2;
    xTaskCreate(motor_rotate_n_task, "rotate5_fwd_m2", 4096, args2, 5, NULL);

    // Let the task run; nothing else to do here
    while (1) {
    	vTaskDelay(pdMS_TO_TICKS(1000));
    }
}