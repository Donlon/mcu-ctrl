#ifndef __track_ir_H
#define __track_ir_H
#ifdef __cplusplus
 extern "C" {
#endif

#define IR_1_Pin GPIO_PIN_4
#define IR_1_GPIO_Port GPIOA
#define IR_2_Pin GPIO_PIN_6
#define IR_2_GPIO_Port GPIOA
#define IR_3_Pin GPIO_PIN_4
#define IR_3_GPIO_Port GPIOC
#define IR_4_Pin GPIO_PIN_5
#define IR_4_GPIO_Port GPIOA
#define IR_5_Pin GPIO_PIN_3
#define IR_5_GPIO_Port GPIOA
#define IR_6_Pin GPIO_PIN_7
#define IR_6_GPIO_Port GPIOA
#define IR_EN_Pin GPIO_PIN_2
#define IR_EN_GPIO_Port GPIOA

void  IR_Sensor_Init( void );
void Get_IR_Sensor( void );
void Power_IR_Sensor( uint8_t on);
#ifdef __cplusplus
}
#endif
#endif /*__track_ir_H*/
