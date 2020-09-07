#ifndef ESP8266_H
#define ESP8266_H

void esp8266_init_ap();

void esp8266_uart_handler(UART_HandleTypeDef *huart);

void esp8266_send_log(const char *string);

#endif // ESP8266_H
