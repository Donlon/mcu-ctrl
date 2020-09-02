#ifndef __COMMON_H
#define __COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

void Complementary_Filter (float angle, float gyro );
void Complementary_Filter_1st_Order (float angle, float gyro );  
void Complementary_Filter_2nd_Order(float angle, float gyro );  
void Kalman_Filter(float angle, float gyro);  

	
void LoopLED( void );
void System_Init( void );
void SetWorkMode( void );
void	UserTask( void );

	
#ifdef __cplusplus
}
#endif

#endif /* __COMMON_H */
