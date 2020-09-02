#include <stdio.h>
#include <string.h>
#include "main.h"

#include "usart.h"
#include "tim.h"
#include "moto_ctrl.h"
#include "speed.h"

extern moto_ctrl_t g_moto_ctrl;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
// Speed sensors decode by hardware                                         //
// Tim1 MT1 sensor , Tim3 MT2 sensor                                        //
// Tim7 calculate speed and distance by 100ms                               //
//////////////////////////////////////////////////////////////////////////////

void SpeedMesurement_Init(void) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void StopSpeedMesurement(void) {
    HAL_TIM_Base_Stop_IT(&htim7);

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
}

void CalculateSpeed(void) {
//  uint8_t databuf[128] ;
    // D=68mm , 3.1415*68/4/13/4.4
    g_moto_ctrl.right_moto_pulse = __HAL_TIM_GET_COUNTER(&htim1);
    g_moto_ctrl.left_moto_pulse = __HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

/*	
  char databuf[128] ;
	sprintf(databuf , "L=%d ;R=%d\n", 
					g_moto_ctrl.left_moto_pulse , g_moto_ctrl.right_moto_pulse );
	HAL_UART_Transmit(&huart1, databuf, strlen( databuf ), 0xFFFF);
*/
}
