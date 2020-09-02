#include <stdio.h>
#include <string.h>
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"
#include "mpu6050.h"
#include "uart_osc.h"
#include "moto_ctrl.h"
#include "dot_matrix.h"
#include "speed.h"
#include "track_ir.h"

#define MAX_LED  4
extern mpu6050_t g_mpu6050;
uint32_t g_SysMode = 1;

extern moto_ctrl_t g_moto_ctrl;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
// Data process                                                             //
// Kalman_Filter                                                            //
// Complementary_Filter                                                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void Kalman_Filter(float angle, float gyro) {
    static const float Q_angle = 0.001;
    static const float Q_gyro = 0.003;
    static const char C_0 = 1;
    static const float R_angle = 0.5;

    static float Q_bias, angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] = {0, 0, 0, 0};
    static float PP[2][2] = {{1, 0},
                             {0, 1}};

    g_mpu6050.Angle_Kalman += (gyro - Q_bias) * DeltaTime;

    Pdot[0] = Q_angle - PP[0][1] - PP[1][0];
    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3] = Q_gyro;

    PP[0][0] += Pdot[0] * DeltaTime;
    PP[0][1] += Pdot[1] * DeltaTime;
    PP[1][0] += Pdot[2] * DeltaTime;
    PP[1][1] += Pdot[3] * DeltaTime;

    angle_err = angle - g_mpu6050.Angle_Kalman;

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    g_mpu6050.Angle_Kalman += K_0 * angle_err;
    Q_bias += K_1 * angle_err;
    g_mpu6050.Gyro_Kalman = gyro - Q_bias;
}

void Complementary_Filter_1st_Order(float angle, float gyro) {
    const float K1 = 0.02;
    g_mpu6050.Angle_Complement_1st = K1 * angle + (1 - K1) * (g_mpu6050.Angle_Complement_1st + gyro * DeltaTime);
}

void Complementary_Filter_2nd_Order(float angle, float gyro) {
    const float K1 = 0.2;
    float x1, x2, y1;

    x1 = (angle - g_mpu6050.Angle_Complement_2nd) * (1 - K1) * (1 - K1);
    y1 = y1 + x1 * DeltaTime;
    x2 = y1 + 2 * (1 - K1) * (angle - g_mpu6050.Angle_Complement_2nd) + gyro;
    g_mpu6050.Angle_Complement_2nd = g_mpu6050.Angle_Complement_2nd + x2 * DeltaTime;
}

void Complementary_Filter(float angle, float gyro) {
    g_mpu6050.Angle_Complement_1st += ((angle - g_mpu6050.Angle_Complement_1st) * 0.3 + gyro) * 0.01;
}

// index 0 : all , 1~4 : LED1~4 
// mode 0 : off , 1 : on , 2 : toggle
void ShowLED(uint32_t index, uint32_t mode) {
    if (mode == 0) {
        switch (index) {
            case 0 :
                HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
                break;
            case 1 :
                HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
                break;
            case 2 :
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
                break;
            case 3 :
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
                break;
            case 4 :
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
                break;
        }
    } else if (mode == 1) {
        switch (index) {
            case 0 :
                HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
                break;
            case 1 :
                HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
                break;
            case 2 :
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                break;
            case 3 :
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
                break;
            case 4 :
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
                break;
        }
    } else if (mode == 2) {
        switch (index) {
            case 0 :
                HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                break;
            case 1 :
                HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
                break;
            case 2 :
                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                break;
            case 3 :
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
                break;
            case 4 :
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                break;
        }
    }
}

uint32_t ReadUserButton0(void) {
    static uint8_t btn0_down = 0;
    static uint8_t btn0_up = 1;
    if (HAL_GPIO_ReadPin(BTN0_GPIO_Port, BTN0_Pin) == GPIO_PIN_RESET) {
        if (btn0_down == 1 && btn0_up == 1) {
            btn0_up = 0;
            return 1;
        } else {
            btn0_down = 1;
            HAL_Delay(50);
        }
    } else {
        btn0_down = 0;
        btn0_up = 1;
    }
    return 0;
}

uint32_t ReadUserButton1(void) {
    static uint8_t btn1_down = 0;
    static uint8_t btn1_up = 1;
    if (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_RESET) {
        if (btn1_down == 1 && btn1_up == 1) {
            btn1_up = 0;
            return 1;
        } else {
            btn1_down = 1;
            HAL_Delay(50);
        }
    } else {
        btn1_down = 0;
        btn1_up = 1;
    }
    return 0;
}

//////////////////////////////////////////////////////

void LoopLED(void) {
    static uint8_t index = 0;
    static uint32_t counter = 0;
    if (counter < 199) {
        counter++;
        return;
    }
    counter = 0;

    switch (g_SysMode) {
        case 0 :   // balance control
            if (index > 0) {
                index = 0;
                ShowLED(1, 1);
                ShowLED(2, 0);
                ShowLED(3, 0);
                ShowLED(4, 1);
            } else {
                ShowLED(1, 0);
                ShowLED(2, 1);
                ShowLED(3, 1);
                ShowLED(4, 0);
                index++;
            }
            break;
        case 1 :   // dot matrix
            if (index >= 3) {
                index = 0;
                ShowLED(1, 0);
                ShowLED(2, 0);
                ShowLED(3, 0);
                ShowLED(4, 1);
            } else if (index == 2) {
                ShowLED(1, 0);
                ShowLED(2, 0);
                ShowLED(3, 1);
                ShowLED(4, 0);
                index++;
            } else if (index == 1) {
                ShowLED(1, 0);
                ShowLED(2, 1);
                ShowLED(3, 0);
                ShowLED(4, 0);
                index++;
            } else if (index == 0) {
                ShowLED(1, 1);
                ShowLED(2, 0);
                ShowLED(3, 0);
                ShowLED(4, 0);
                index++;
            }
            break;
        case 2 :   // show osc wave
            if (index >= 6) {
                index = 0;
                ShowLED(1, 1);
                ShowLED(2, 0);
                ShowLED(3, 0);
                ShowLED(4, 0);
            } else if (index == 5 || index == 1) {
                ShowLED(1, 0);
                ShowLED(2, 1);
                ShowLED(3, 0);
                ShowLED(4, 0);
                index++;
            } else if (index == 4 || index == 2) {
                ShowLED(1, 0);
                ShowLED(2, 0);
                ShowLED(3, 1);
                ShowLED(4, 0);
                index++;
            } else if (index == 3) {
                ShowLED(1, 0);
                ShowLED(2, 0);
                ShowLED(3, 0);
                ShowLED(4, 1);
                index++;
            } else if (index == 0) {
                ShowLED(1, 1);
                ShowLED(2, 0);
                ShowLED(3, 0);
                ShowLED(4, 0);
                index++;
            }
            break;
    }
}


void System_Init(void) {
    Power_IR_Sensor(0);
    MPU6050_Init();
    Moto_Ctrl_Init();
    SpeedMesurement_Init();
}

void SetWorkMode(void) {
    switch (g_SysMode) {
        case 0 :   // balance control
            System_Init();
            IR_Sensor_Init();
            HAL_TIM_Base_Start_IT(&htim5); //time for applicationn
            HAL_TIM_Base_Start_IT(&htim6); //read sensor data and  control moto
            HAL_TIM_Base_Stop_IT(&htim7);  //read sensor data and  do not control moto
            break;
        case 1 :   // dot matrix
            System_Init();
            dot_matrix_init();
            HAL_TIM_Base_Start_IT(&htim5);
            HAL_TIM_Base_Stop_IT(&htim6);
            HAL_TIM_Base_Stop_IT(&htim7);
            break;
        case 2 :   // show osc wave
            System_Init();
            IR_Sensor_Init();
            HAL_TIM_Base_Start_IT(&htim5);
            HAL_TIM_Base_Start_IT(&htim6);
            HAL_TIM_Base_Stop_IT(&htim7);
            break;
    }
}


void UserTask(void) {
    if (ReadUserButton0() == 1) {
        if (g_SysMode < 2) g_SysMode++;
        else g_SysMode = 0;
        SetWorkMode();
    }

    switch (g_SysMode) {
        case 0 :
            break;
        case 1 :   // dot matrix
            show_dot_matrix();
            break;
        case 2 :

            Uart_OSC_ShowWave(g_mpu6050.accel_x, g_mpu6050.accel_y, g_mpu6050.accel_z, g_mpu6050.gyro_x);
//			Uart_OSC_ShowWave( g_mpu6050.angle_x , g_mpu6050.gyro_scale_y , g_mpu6050.Angle_Complement_1st, g_mpu6050.gyro_scale_z  ) ;
            break;
    }
//  CheckUartReceivedData();
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint8_t period_5ms = 0;
    static uint8_t period_100ms = 0;
    static uint32_t period_1s = 0;
    char databuf[128];

    if (htim == &htim5)  // 100ms
    {
        if (g_SysMode == 0) {
        } else if (g_SysMode == 1) {
            move_dot_matrix();
        } else if (g_SysMode == 2) {
            if (period_1s < 9) period_1s++;
            else period_1s = 0;
            if (period_1s == 0) {
                sprintf(databuf, "IR_IN=%X \n",
                        g_moto_ctrl.track_in);
                HAL_UART_Transmit(&huart1, databuf, strlen(databuf), 0xFFFF);

            }
        }

    } else if (htim == &htim6)  // 1ms
    {
        period_5ms++;
        if (period_5ms > 4) {
            period_5ms = 0;
            period_100ms++;
            if (period_100ms > 19) {
                period_100ms = 0;
            }
        }
        if (period_5ms == 0) {
            if (period_100ms == 0) {
                CalculateSpeed();
                SpeedControl();
            }
            SpeedControlOutput(5);
            DirectionControlOutput(5);
            MotoOutput();
            if (g_SysMode == 0) {
                MotoSpeedOut();
            }
            LoopLED();
            return;
        } else if (period_5ms == 1) {
            MPU6050_Get_Accel_Gyro_Temp();
            return;
        } else if (period_5ms == 2) {
            MPU6050_Data_Process();
            AngleControl();
            return;
        } else if (period_5ms == 3) {
            if ((period_100ms % 2) == 1) {
                Get_IR_Sensor();
                DirectionControl();
                Power_IR_Sensor(0);
            } else {
                Power_IR_Sensor(1);
            }
            return;
        }

    } else if (htim == &htim7) //5ms
    {
    }
}
