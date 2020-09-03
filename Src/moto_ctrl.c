#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "mpu6050.h"
#include "moto_ctrl.h"

extern TIM_HandleTypeDef htim8;
extern mpu6050_t g_mpu6050;

float angle_control_p = ANGLE_CONTROL_P;
float angle_control_d = ANGLE_CONTROL_D;

moto_ctrl_t g_moto_ctrl;

void Moto_Ctrl_Init(void) {
    Motor_Stop();
    memset(&g_moto_ctrl, 0, sizeof(moto_ctrl_t));
    g_moto_ctrl.speed_set = 0;
}

void AngleControl(void) {
#ifdef KALMAN_FILTER_GYRO
    float p = (CAR_ANGLE_SET - g_mpu6050.Angle_Kalman) * angle_control_p;
    float d = (CAR_ANGLE_SPEED_SET - g_mpu6050.Gyro_Kalman) * angle_control_d;
#else
    float p = (CAR_ANGLE_SET - g_mpu6050.Angle_Complement_1st) * angle_control_p;
    float d = (CAR_ANGLE_SPEED_SET - g_mpu6050.gyro_scale_x) * angle_control_d;
#endif
    g_moto_ctrl.angle_ctrl_p = p;
    g_moto_ctrl.angle_ctrl_d = d;
    g_moto_ctrl.angle_ctrl = p + d;
}

void SpeedControl(void) {
    float diff;
    g_moto_ctrl.moto_pulse = (float) (g_moto_ctrl.left_moto_pulse + g_moto_ctrl.right_moto_pulse) / 2.f;
    g_moto_ctrl.speed = g_moto_ctrl.moto_pulse * CAR_SPEED_CONSTANT;
    diff = g_moto_ctrl.speed - g_moto_ctrl.speed_set;
    g_moto_ctrl.positon += (diff * SPEED_CONTROL_I);

    g_moto_ctrl.speed_ctrl_last = g_moto_ctrl.speed_ctrl_next;
    g_moto_ctrl.speed_ctrl_next = diff * SPEED_CONTROL_P + g_moto_ctrl.positon;
    g_moto_ctrl.speed_diff = g_moto_ctrl.speed_ctrl_next - g_moto_ctrl.speed_ctrl_last;

#ifdef SPEED_CTRL
	g_moto_ctrl.left_speed = g_moto_ctrl.left_moto_pulse * CAR_SPEED_CONSTANT;
	diff =  g_moto_ctrl.left_speed - g_moto_ctrl.speed_set ;
	g_moto_ctrl.left_positon += ( diff * SPEED_CONTROL_I ) ;

	g_moto_ctrl.left_speed_ctrl_last = g_moto_ctrl.left_speed_ctrl_next;
	g_moto_ctrl.left_speed_ctrl_next = diff * SPEED_CONTROL_P + g_moto_ctrl.left_positon;
	g_moto_ctrl.left_speed_diff = g_moto_ctrl.left_speed_ctrl_next - g_moto_ctrl.left_speed_ctrl_last;

	g_moto_ctrl.right_speed =  g_moto_ctrl.right_moto_pulse * CAR_SPEED_CONSTANT;
	diff = g_moto_ctrl.right_speed - g_moto_ctrl.speed_set;
	g_moto_ctrl.right_positon += (diff * SPEED_CONTROL_I);

	g_moto_ctrl.right_speed_ctrl_last = g_moto_ctrl.right_speed_ctrl_next;
	g_moto_ctrl.right_speed_ctrl_next = diff * SPEED_CONTROL_P + g_moto_ctrl.right_positon;
	g_moto_ctrl.right_speed_diff = g_moto_ctrl.right_speed_ctrl_next - g_moto_ctrl.right_speed_ctrl_last;
#endif
}

void SpeedControlOutput(uint8_t period) {
#ifdef SPEED_CTRL
    g_moto_ctrl.left_speed = g_moto_ctrl.left_speed_diff * (g_moto_ctrl.speed_ctrl_period + period)/SPEED_CONTROL_PERIOD +
                                                        g_moto_ctrl.left_speed_ctrl_last;
    g_moto_ctrl.right_speed = g_moto_ctrl.right_speed_diff * (g_moto_ctrl.speed_ctrl_period + period)/SPEED_CONTROL_PERIOD +
                                                        g_moto_ctrl.right_speed_ctrl_last;
#endif

    g_moto_ctrl.speed = g_moto_ctrl.speed_diff * (g_moto_ctrl.speed_ctrl_period + period) / SPEED_CONTROL_PERIOD +
                        g_moto_ctrl.speed_ctrl_last;

    if (g_moto_ctrl.speed_ctrl_period < (SPEED_CONTROL_PERIOD - period))
        g_moto_ctrl.speed_ctrl_period += period;
    else
        g_moto_ctrl.speed_ctrl_period = 0;
}

void DirectionControl(void) {
    g_moto_ctrl.direction_ctrl_last = g_moto_ctrl.direction_ctrl_next;

    if (g_moto_ctrl.track_in == 0x08 ||  // 110111
        g_moto_ctrl.track_in == 0x0C ||  // 110011
        g_moto_ctrl.track_in == 0x04) {  // 111011
        g_moto_ctrl.direction_ctrl_next = 0;
    } else if (g_moto_ctrl.track_in == 0x10 ||  // 101111
               g_moto_ctrl.track_in == 0x18) {  // 100111
        g_moto_ctrl.direction_ctrl_next = 10 * DIR_CONTROL_P + g_mpu6050.gyro_scale_z * DIR_CONTROL_D;
    } else if (g_moto_ctrl.track_in == 0x20 ||    // 011111
               g_moto_ctrl.track_in == 0x30 ||    // 001111
               g_moto_ctrl.track_in == 0x38) {    // 000111
        g_moto_ctrl.direction_ctrl_next = 20 * DIR_CONTROL_P + g_mpu6050.gyro_scale_z * DIR_CONTROL_D;
    } else if (g_moto_ctrl.track_in == 0x02 ||    // 111101
               g_moto_ctrl.track_in == 0x06) {    // 111001
        g_moto_ctrl.direction_ctrl_next = -10 * DIR_CONTROL_P + g_mpu6050.gyro_scale_z * DIR_CONTROL_D;
    } else if (g_moto_ctrl.track_in == 0x01 ||    // 111110
               g_moto_ctrl.track_in == 0x03 ||    // 111100
               g_moto_ctrl.track_in == 0x07) {    // 111000
        g_moto_ctrl.direction_ctrl_next = -20 * DIR_CONTROL_P + g_mpu6050.gyro_scale_z * DIR_CONTROL_D;
    } else if (g_moto_ctrl.track_in == 0x3F) {
        g_moto_ctrl.speed_set = 0;
        g_moto_ctrl.direction_ctrl_next = 0;
    } else {
        g_moto_ctrl.direction_ctrl_next = 0;
    }
    g_moto_ctrl.direction_diff = g_moto_ctrl.direction_ctrl_next - g_moto_ctrl.direction_ctrl_last;
}

void DirectionControlOutput(uint8_t period) {
    g_moto_ctrl.direction =
            g_moto_ctrl.direction_diff * (g_moto_ctrl.direction_ctrl_period + period) / DIR_CONTROL_PERIOD +
            g_moto_ctrl.direction_ctrl_last;

    if (g_moto_ctrl.direction_ctrl_period < (DIR_CONTROL_PERIOD - period))
        g_moto_ctrl.direction_ctrl_period += period;
    else
        g_moto_ctrl.direction_ctrl_period = 0;
}

void MotoOutput(void) {
#ifdef SPEED_CTRL
	g_moto_ctrl.left_ctrl = g_moto_ctrl.angle_ctrl - g_moto_ctrl.left_speed;
	g_moto_ctrl.right_ctrl = g_moto_ctrl.angle_ctrl - g_moto_ctrl.right_speed;
#else
#if 0
    g_moto_ctrl.left_ctrl = g_moto_ctrl.angle_ctrl - g_moto_ctrl.speed - g_moto_ctrl.direction;
    g_moto_ctrl.right_ctrl = g_moto_ctrl.angle_ctrl - g_moto_ctrl.speed + g_moto_ctrl.direction;
#else
    g_moto_ctrl.left_ctrl = g_moto_ctrl.angle_ctrl - g_moto_ctrl.speed;
    g_moto_ctrl.right_ctrl = g_moto_ctrl.angle_ctrl - g_moto_ctrl.speed;
#endif
#endif
}

void MotoSpeedOut(void) {
    float leftVal, rightVal;

#if 0
    leftVal = g_moto_ctrl.left_ctrl;
    rightVal = g_moto_ctrl.right_ctrl;
#else
    leftVal = -g_moto_ctrl.left_ctrl;
    rightVal = -g_moto_ctrl.right_ctrl;
#endif

    if (leftVal > 0) {
        leftVal += MOTOR_OUT_DEAD_VAL;
    } else if (leftVal < 0) {
        leftVal -= MOTOR_OUT_DEAD_VAL;
    }

    if (rightVal > 0) {
        rightVal += MOTOR_OUT_DEAD_VAL;
    } else if (rightVal < 0) {
        rightVal -= MOTOR_OUT_DEAD_VAL;
    }

    if (leftVal > MOTOR_OUT_MAX) {
        leftVal = MOTOR_OUT_MAX;
    } else if (leftVal < MOTOR_OUT_MIN) {
        leftVal = MOTOR_OUT_MIN;
    }

    if (rightVal > MOTOR_OUT_MAX) {
        rightVal = MOTOR_OUT_MAX;
    } else if (rightVal < MOTOR_OUT_MIN) {
        rightVal = MOTOR_OUT_MIN;
    }
    SetMotorVoltage(leftVal, rightVal);
}

// Set output voltage via PWM
void SetMotorVoltage(float fLeftVoltage, float fRightVoltage) {
    if (fLeftVoltage > 0) {
        left_backward((uint16_t) (fLeftVoltage * PWM_PERIOD));
    } else {
        left_forward((uint16_t) (-1 * fLeftVoltage * PWM_PERIOD));
    }
    if (fRightVoltage > 0) {
        right_backward((uint16_t) (fRightVoltage * PWM_PERIOD));
    } else {
        right_forward((uint16_t) (-1 * fRightVoltage * PWM_PERIOD));
    }
}
#define LEFT_PWM_CHANNEL TIM_CHANNEL_4
#define RIGHT_PWM_CHANNEL TIM_CHANNEL_3

void left_forward(uint16_t value) {
    HAL_TIM_PWM_Stop(&htim8, LEFT_PWM_CHANNEL);

    HAL_GPIO_WritePin(MT1_A_GPIO_Port, MT1_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MT1_B_GPIO_Port, MT1_B_Pin, GPIO_PIN_RESET);

    TIM_SetCompare4(TIM8, value);
    HAL_TIM_PWM_Start(&htim8, LEFT_PWM_CHANNEL);
}

void left_backward(uint16_t value) {
    HAL_TIM_PWM_Stop(&htim8, LEFT_PWM_CHANNEL);
    HAL_GPIO_WritePin(MT1_A_GPIO_Port, MT1_A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT1_B_GPIO_Port, MT1_B_Pin, GPIO_PIN_SET);

    TIM_SetCompare4(TIM8, value);
    HAL_TIM_PWM_Start(&htim8, LEFT_PWM_CHANNEL);
}

void right_forward(uint16_t value) {
    HAL_TIM_PWM_Stop(&htim8, RIGHT_PWM_CHANNEL);
    HAL_GPIO_WritePin(MT2_B_GPIO_Port, MT2_B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MT2_A_GPIO_Port, MT2_A_Pin, GPIO_PIN_RESET);

    TIM_SetCompare3(TIM8, value);
    HAL_TIM_PWM_Start(&htim8, RIGHT_PWM_CHANNEL);
}

void right_backward(uint16_t value) {
    HAL_TIM_PWM_Stop(&htim8, RIGHT_PWM_CHANNEL);
    HAL_GPIO_WritePin(MT2_B_GPIO_Port, MT2_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT2_A_GPIO_Port, MT2_A_Pin, GPIO_PIN_SET);

    TIM_SetCompare3(TIM8, value);
    HAL_TIM_PWM_Start(&htim8, RIGHT_PWM_CHANNEL);
}

void Motor_Stop(void) {
    HAL_TIM_PWM_Stop(&htim8, RIGHT_PWM_CHANNEL);
    HAL_TIM_PWM_Stop(&htim8, LEFT_PWM_CHANNEL);
    HAL_GPIO_WritePin(MT1_A_GPIO_Port, MT2_A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT1_B_GPIO_Port, MT2_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT2_A_GPIO_Port, MT2_A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT2_B_GPIO_Port, MT2_B_Pin, GPIO_PIN_RESET);
}

void TIM_SetCompare3(TIM_TypeDef *TIMx, uint16_t Compare) {
    /* Check the parameters */
    // assert_param(IS_TIM_LIST8_PERIPH(TIMx));
    /* Set the Capture Compare1 Register value */
    TIMx->CCR3 = Compare;
}

void TIM_SetCompare4(TIM_TypeDef *TIMx, uint16_t Compare) {
    /* Check the parameters */
    // assert_param(IS_TIM_LIST6_PERIPH(TIMx));
    /* Set the Capture Compare2 Register value */
    TIMx->CCR4 = Compare;
}
