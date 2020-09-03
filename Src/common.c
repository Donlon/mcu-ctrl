#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "mpu6050.h"
#include "uart_osc.h"
#include "moto_ctrl.h"
#include "dot_matrix.h"
#include "speed.h"
#include "track_ir.h"

#define MAX_LED  4
extern mpu6050_t g_mpu6050;

typedef enum {
    BalanceControl = 0,
    DotMatrix = 1,
    OscWave = 2,
    ModeCount = 3,
} SysMode;

SysMode g_SysMode = BalanceControl;

extern moto_ctrl_t g_moto_ctrl;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
// Data process                                                             //
// Kalman_Filter                                                            //
// Complementary_Filter                                                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void Kalman_Filter(float angle, float gyro) {
    static const float Q_angle = 0.001f;
    static const float Q_gyro = 0.003f;
    static const char C_0 = 1;
    static const float R_angle = 0.5f;

    static float Q_bias, angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] = {
            0, 0, 0, 0
    };
    static float PP[2][2] = {
            {1, 0},
            {0, 1}
    };

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
    const float K1 = 0.02f;
    g_mpu6050.Angle_Complement_1st = K1 * angle + (1.f - K1) * (g_mpu6050.Angle_Complement_1st + gyro * DeltaTime);
}

void Complementary_Filter_2nd_Order(float angle, float gyro) {
    const float K1 = 0.2f;
    float x1, x2, y1;

    x1 = (angle - g_mpu6050.Angle_Complement_2nd) * (1.f - K1) * (1.f - K1);
    y1 = y1 + x1 * DeltaTime;
    x2 = y1 + 2.f * (1.f - K1) * (angle - g_mpu6050.Angle_Complement_2nd) + gyro;
    g_mpu6050.Angle_Complement_2nd = g_mpu6050.Angle_Complement_2nd + x2 * DeltaTime;
}

void Complementary_Filter(float angle, float gyro) {
    g_mpu6050.Angle_Complement_1st += ((angle - g_mpu6050.Angle_Complement_1st) * 0.3f + gyro) * 0.01f;
}

// index 0 : all , 1~4 : LED1~4 
// mode 0 : off , 1 : on , 2 : toggle
void ShowLED(uint32_t index, uint32_t mode) {
    if (mode == 0) {
        switch (index) {
            case 0:
                HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
                break;
            case 1:
                HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
                break;
            case 2:
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
                break;
            case 3:
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
                break;
            case 4:
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
                break;
            default:
                break;
        }
    } else if (mode == 1) {
        switch (index) {
            case 0:
                HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
                break;
            case 1:
                HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
                break;
            case 2:
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                break;
            case 3:
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
                break;
            case 4:
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
                break;
            default:
                break;
        }
    } else if (mode == 2) {
        switch (index) {
            case 0:
                HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                break;
            case 1:
                HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
                break;
            case 2:
                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                break;
            case 3:
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
                break;
            case 4:
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                break;
            default:
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

void LoopLED(void) { //  called per 100ms
    static uint8_t index = 0;
    static uint32_t counter = 0;
    if (counter <= 2) {
        counter++;
        return;
    }
    counter = 0;

    switch (g_SysMode) {
        case BalanceControl:   // balance control
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
        case DotMatrix:   // dot matrix
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
        case OscWave:   // show osc wave
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
        default:
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
        case BalanceControl:   // balance control
            System_Init();
            IR_Sensor_Init();
            HAL_TIM_Base_Start_IT(&htim5); //time for applicationn
            HAL_TIM_Base_Start_IT(&htim6); //read sensor data and  control moto
            HAL_TIM_Base_Stop_IT(&htim7);  //read sensor data and  do not control moto
            break;
        case DotMatrix:   // dot matrix
            System_Init();
            dot_matrix_init();
            HAL_TIM_Base_Start_IT(&htim5);
            HAL_TIM_Base_Stop_IT(&htim6);
            HAL_TIM_Base_Stop_IT(&htim7);
            break;
        case OscWave:   // show osc wave
            System_Init();
            IR_Sensor_Init();
            HAL_TIM_Base_Start_IT(&htim5);
            HAL_TIM_Base_Start_IT(&htim6);
            HAL_TIM_Base_Stop_IT(&htim7);
            break;
        default:
            break;
    }
}

#define RECV_BUF_SIZE 128

char recvBuf[RECV_BUF_SIZE];
uint8_t recvData = 0;
uint16_t recvLen = 0;

int recv_buf_processed = 1;

#define imin(a, b) (((a) < (b)) ? (a) : (b))

char *ParseUartCommand(char *str, int len) {
    if (len < 3) {
        return 0;
    }
    char numbuf[16];
    char item = str[0];
    //assert(str[1] == '=');
    char *p = str + 2;
    char *p_buf = numbuf;
    char *end = str + 2 + imin(len - 2, 15);
    while (p < end) {
        char ch = *p;
        if (ch == ';' || ch == '\r' || ch == '\n' || ch == ' ') {
            p++;
            break;
        }
        if (ch == '\0') {
            break;
        }
        *p_buf = ch;
        p++;
        p_buf++;
    }
    *p_buf = '\0';

    float num = atof(numbuf);
    switch (item) {
        case 'P':
        case 'p':
            angle_control_p = num;
            break;
        case 'D':
        case 'd':
            angle_control_d = num;
            break;
        default:
            break;
    }
    return p;
}

void ProcessUartRecvBuffer() {
    char *buf = recvBuf;
    int len = recvLen;
    while (len > 0) {
        char *new_buf = ParseUartCommand(buf, len);
        if (!new_buf) {
            break;
        }
        len -= (int) (new_buf - buf);
        buf = new_buf;
    }

    // recv_buf_processed = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart3.Instance) {
        if (recvLen < RECV_BUF_SIZE - 1) {
            if (recvData == '\r' || recvData == '\n') {
                if (recvLen > 0) {
                    ProcessUartRecvBuffer();
                }
                recvBuf[recvLen] = '\0';
                recvLen = 0;
            } else {
                recvBuf[recvLen++] = recvData;
            }
        } else { // Too long
            recvLen = 0;
        }
        HAL_UART_Receive_IT(&huart3, &recvData, 1);
    }
}

void UserTask(void) {
    if (ReadUserButton0() == 1) {
        g_SysMode++;
        if (g_SysMode >= ModeCount) {
            g_SysMode = BalanceControl;
        }
        SetWorkMode();
    }

    switch (g_SysMode) {
        case BalanceControl:
            break;
        case DotMatrix: // dot matrix
            show_dot_matrix();
            break;
        case OscWave:
            // Uart_OSC_ShowWave(g_mpu6050.accel_x, g_mpu6050.accel_y, g_mpu6050.accel_z, g_mpu6050.gyro_x);
            // Uart_OSC_ShowWave( g_mpu6050.angle_x , g_mpu6050.gyro_scale_y , g_mpu6050.Angle_Complement_1st, g_mpu6050.gyro_scale_z  ) ;
            break;
        default:
            break;
    }
    if (recv_buf_processed) {
        recv_buf_processed = 0;
        HAL_UART_Receive_IT(&huart3, &recvData, 1);
    }
    // CheckUartReceivedData();
}

void PrintIrSensorData(uint8_t data) {
#define IR_C(x) ((x) ? 'x' : '-')
    char s[8];
    s[0] = IR_C(data & 0x01u);
    s[1] = IR_C(data & 0x02u);
    s[2] = IR_C(data & 0x04u);
    s[3] = IR_C(data & 0x08u);
    s[4] = IR_C(data & 0x10u);
    s[5] = IR_C(data & 0x20u);
    printf("[IR]: %s\r\n", s);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint8_t period_5ms = 0;   // 1ms/0...4
    static uint8_t period_100ms = 0; // 5ms/0...19
    static uint32_t period_1s = 0;   // 100ms/0...9
    static uint32_t period_10s = 0;   // 1m/0...9
    static uint32_t secs = 0;

    if (htim == &htim5) { // 100ms
        period_1s++;
        if (period_1s >= 10) { // 1s
            period_1s = 0;
            period_10s++;
            secs++;
            if (period_10s >= 10) {
                period_10s = 0;
            }
        }
        switch (g_SysMode) {
            case BalanceControl:
                if (period_1s == 0 && (secs % 3) == 0) {
                    PrintIrSensorData(g_moto_ctrl.track_in);
                }
                if (period_1s == 0) {
                    float r;
                    if (g_moto_ctrl.right_moto_pulse == 0) {
                        r = -1.f;
                    } else {
                        r = (float) g_moto_ctrl.left_moto_pulse / (float) g_moto_ctrl.right_moto_pulse;
                    }
                    printf("[speed] motor pulse: L=%d, R=%d, ratio: %.2f...\r\n",
                           g_moto_ctrl.left_moto_pulse, g_moto_ctrl.right_moto_pulse, r);
                }
                break;
            case DotMatrix:
                move_dot_matrix();
                break;
            case OscWave:
                break;
            default:
                break;
        }
        LoopLED();
    } else if (htim == &htim6) { // 1ms
        period_5ms++;
        if (period_5ms >= 5) {
            period_5ms = 0;
            period_100ms++;
            if (period_100ms >= 20) {
                period_100ms = 0;
            }
        }
        switch (period_5ms) {
            case 0:
                if (period_100ms == 0) { // per 100ms
                    CalculateSpeed();
                    SpeedControl();
                }
                SpeedControlOutput(5);
                DirectionControlOutput(5);
                MotoOutput();
                if (g_SysMode == 0) {
                    MotoSpeedOut();
                }
                break;
            case 1:
                // read from sensor
                MPU6050_Get_Accel_Gyro_Temp();
                break;
            case 2:
                MPU6050_Data_Process();
                AngleControl();
                break;
            case 3:
                if ((period_100ms % 2) == 1) {
                    Get_IR_Sensor();
                    DirectionControl();
                    Power_IR_Sensor(0);
                } else {
                    Power_IR_Sensor(1);
                }
                break;
            case 4:
                Uart_OSC_ShowWave(g_mpu6050.Angle_Complement_1st,
                                  (int) (g_moto_ctrl.angle_ctrl * 10.f),
                                  (int) (g_moto_ctrl.angle_ctrl_p * 10.f),
                                  (int) (g_moto_ctrl.angle_ctrl_d * 10.f));
                break;
            default:
                break;
        }
    } else if (htim == &htim7) { //5ms
    }
}
