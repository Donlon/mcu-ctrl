#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "bitband.h"
#include "track_ir.h"
#include "moto_ctrl.h"

extern moto_ctrl_t g_moto_ctrl ;

void Power_IR_Sensor( uint8_t on)
{
	if ( on == 1 ) 
	{
		PAO( 2 ) = 0;
	}
	else
	{
		PAO( 2 ) = 1;
	}
}
void Get_IR_Sensor( void )
{
  uint8_t result = 0 ;
	result |= PAI( 7 );
	result <<= 1;
	result |= PAI( 3 );
	result <<= 1;
	result |= PAI( 5 );
	result <<= 1;
	result |= PCI( 4 );
	result <<= 1;
	result |= PAI( 6 );
	result <<= 1;
	result |= PAI( 4 );

  //data0 = GPIOA->IDR & 0x00F8;	  //  PA3,4,5,6,7 -> IR 5,1,4,2,6
  //data1 = GPIOC->IDR & 0x0010;	  //  PC4   -> IR 3
	g_moto_ctrl.track_in = result;
}

void IR_Sensor_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_SPI_MspDeInit( &hspi1 );
  HAL_UART_MspDeInit( &huart2 );	
/*	
  HAL_GPIO_WritePin(IR_1_GPIO_Port, IR_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IR_2_GPIO_Port, IR_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IR_3_GPIO_Port, IR_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IR_4_GPIO_Port, IR_4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IR_5_GPIO_Port, IR_5_Pin, GPIO_PIN_RESET);
*/	
  HAL_GPIO_DeInit(IR_1_GPIO_Port, IR_1_Pin);
  HAL_GPIO_DeInit(IR_2_GPIO_Port, IR_2_Pin);
  HAL_GPIO_DeInit(IR_3_GPIO_Port, IR_3_Pin);
  HAL_GPIO_DeInit(IR_4_GPIO_Port, IR_4_Pin);
  HAL_GPIO_DeInit(IR_5_GPIO_Port, IR_5_Pin);
  HAL_GPIO_DeInit(IR_6_GPIO_Port, IR_6_Pin);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = IR_1_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_1_GPIO_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = IR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_2_GPIO_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = IR_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_3_GPIO_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = IR_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_4_GPIO_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = IR_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_5_GPIO_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = IR_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_5_GPIO_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = IR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IR_EN_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(IR_EN_GPIO_Port, IR_EN_Pin, GPIO_PIN_SET);

}
