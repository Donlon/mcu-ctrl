#ifndef _MOTO_CTRL_H
#define _MOTO_CTRL_H

#include "stm32f1xx_hal.h"

#define  CAR_ANGLE_SET        0.f
#define  CAR_ANGLE_SPEED_SET  0.f
#define  ANGLE_CONTROL_P      0.2f  // P=0.042 D=0.0015
#define  ANGLE_CONTROL_D      0.002f

extern float angle_control_p;
extern float angle_control_d;

#define  WHEEL_DIAMETER       82.0f  // 82mm
#define  WHEEL_PERIMETER      (PI*WHEEL_DIAMETER)
#define  ENCODE_CONSTANT      52.f  // 13*4
#define  MOTO_GEARBOX_RATE    4.4f  // 1:4.4 1360RPM
#define  SPEED_CONTROL_PERIOD 100.f // 100ms
#define  CAR_SPEED_CONSTANT   (1000.0f/SPEED_CONTROL_PERIOD/ENCODE_CONSTANT)
#define  SPEED_CONTROL_P      0.02f    //0.03
#define  SPEED_CONTROL_I      0.0000f

#define  DIR_CONTROL_PERIOD   10.f // 10ms
#define  DIR_CONTROL_P        0.1f
#define  DIR_CONTROL_D        0.01f

#define  CAR_POSITION_SET     0.f
#define  CAR_POSITION_MAX     8000.f
#define  CAR_POSITION_MIN     -8000.f

#define  MOTOR_OUT_DEAD_VAL   0.02f
#define  MOTOR_OUT_MAX        0.8f
#define  MOTOR_OUT_MIN        -0.8f
#define  PWM_PERIOD           2000.f

typedef struct {
    float angle_ctrl;

    float angle_ctrl_p;
    float angle_ctrl_d;

    // pulse/100ms
    int16_t left_moto_pulse;
    int16_t right_moto_pulse;

    float speed_set;

    float moto_pulse;
    float speed;
    float positon;
    float speed_ctrl_last;
    float speed_ctrl_next;
    float speed_diff;
    float speed_ctrl;

    float direction;
    float direction_ctrl_last;
    float direction_ctrl_next;
    float direction_diff;
    float direction_ctrl;

#ifdef SPEED_CTRL
    float left_speed;
    float right_speed;
    float left_positon;
    float left_speed_ctrl_last;
    float left_speed_ctrl_next;
    float left_speed_diff;
    float left_speed_ctrl;
    float right_positon;
    float right_speed_ctrl_last;
    float right_speed_ctrl_next;
    float right_speed_ctrl;
    float right_speed_diff;
#endif

    float speed_ctrl_period;
    float direction_ctrl_period;

    // for PWM output
    float left_ctrl;
    float right_ctrl;

    uint8_t track_in;

} moto_ctrl_t;


void Moto_Ctrl_Init(void);

void Motor_Stop(void);

void left_forward(uint16_t value);

void left_backward(uint16_t value);

void right_forward(uint16_t value);

void right_backward(uint16_t value);

void TIM_SetCompare3(TIM_TypeDef *TIMx, uint16_t Compare);

void TIM_SetCompare4(TIM_TypeDef *TIMx, uint16_t Compare);

void SetMotorVoltage(float fLeftVoltage, float fRightVoltage);

void AngleControl(void);

void SpeedControl(void);

void DirectionControl(void);

void SpeedControlOutput(uint8_t period);

void DirectionControlOutput(uint8_t period);

void MotoOutput(void);

void MotoSpeedOut(void);

#endif
