#include "main.h"
#include "usart.h"
#include "uart_osc.h"

UART_HandleTypeDef *g_uart_osc = &huart5;

typedef union {
    int16_t data16;
    int8_t data8[2];
} uart_osc_t;

uart_osc_t g_Uart_OSC_Data[4];

unsigned short CRC_CHECK(const uint8_t *Buf, unsigned char CRC_CNT) {
    unsigned short CRC_Temp;
    CRC_Temp = 0xffff;

    for (int i = 0; i < CRC_CNT; i++) {
        CRC_Temp ^= Buf[i];
        for (int j = 0; j < 8; j++) {
            if (CRC_Temp & 0x01u)
                CRC_Temp = (CRC_Temp >> 1u) ^ 0xa001u;
            else
                CRC_Temp = CRC_Temp >> 1u;
        }
    }
    return (CRC_Temp);
}

void Uart_OSC_Output_Data(void) {
    unsigned char databuf[10] = {0};

    for (int i = 0; i < 4; i++) {
        databuf[i * 2] = g_Uart_OSC_Data[i].data8[0];
        databuf[i * 2 + 1] = g_Uart_OSC_Data[i].data8[1];
    }

    unsigned short CRC16 = CRC_CHECK(databuf, 8);
    databuf[8] = CRC16 % 256;
    databuf[9] = CRC16 / 256;

    for (int i = 0; i < 10; i++) {
        HAL_UART_Transmit(g_uart_osc, (uint8_t *) &databuf[i], 1, 0xFFFF);
        while (HAL_UART_GetState(g_uart_osc) == HAL_UART_STATE_RESET);
    }
}

void Uart_OSC_ShowWave(int a, int b, int c, int d) {
#if 0
    char databuf[128] ;
    sprintf( databuf , "x=%d ,y=%d , z=%d \r\n" , a , b, c );
    HAL_UART_Transmit(&huart1, databuf, strlen( databuf ), 0xFFFF);
    return;
#endif
    g_Uart_OSC_Data[0].data16 = a;
    g_Uart_OSC_Data[1].data16 = b;
    g_Uart_OSC_Data[2].data16 = c;
    g_Uart_OSC_Data[3].data16 = d;
    Uart_OSC_Output_Data();
}

