#include <string.h>
#include <stdio.h>

#include "main.h"
#include "usart.h"

#include "esp8266.h"

UART_HandleTypeDef *uart;

typedef enum {
    WaitCmdResponse,
    RecvIpd,
    RecvData,
    Idle,
} Esp8266RxStatus;

Esp8266RxStatus rxStatus;

u_char isWaitingReply = 0;
u_char responseReceived = 0;

#define CMD_RECV_BUFFER_SIZE 128
#define DATA_RECV_BUFFER_SIZE 1024

char cmd_recv_buffer[CMD_RECV_BUFFER_SIZE];
uint16_t cmd_buffer_pos = 0;

char data_recv_buffer[DATA_RECV_BUFFER_SIZE];
uint16_t data_buffer_pos = 0;
int data_len;

uint8_t esp8266_recv_data = 0;

void esp8266_cmd_tx(const void *data, uint16_t len) {
    while (rxStatus == RecvIpd || rxStatus == RecvData){
    }

    HAL_UART_Transmit(uart, (uint8_t *) data, len, 0xFFFF);
    HAL_UART_Transmit(uart, (uint8_t *) "\r\n", 2, 0xFFFF);

    isWaitingReply = 1;
    responseReceived = 0;

    cmd_buffer_pos = 0;

    while (!responseReceived) {
    }

    isWaitingReply = 0;
}

void esp8266_cmd_tx_string(const char *string) {
    printf("[ESP8266] >> %s\r\n", string);
    esp8266_cmd_tx((uint8_t *) string, strlen(string));
}

void esp8266_process_cmd_reply() {
    // printf("[ESP8266] << %s\r\n", cmd_recv_buffer);
    if (strcmp(cmd_recv_buffer, "OK") == 0 ||
        strcmp(cmd_recv_buffer, "Failed") == 0) {
        responseReceived = 1;
    }
}

void esp8266_uart_handler(UART_HandleTypeDef *huart) {
    if (cmd_buffer_pos < CMD_RECV_BUFFER_SIZE - 1) {
        if (esp8266_recv_data != '\r' && esp8266_recv_data != '\n') {
            cmd_recv_buffer[cmd_buffer_pos++] = esp8266_recv_data;
        } else { // line ending
            if (cmd_buffer_pos > 0 && isWaitingReply) { // ignore response if !isWaitingReply
                cmd_recv_buffer[cmd_buffer_pos] = '\0';
                esp8266_process_cmd_reply();
            }
            cmd_buffer_pos = 0; // ignore previous response
        }
    } else { // Too long
        cmd_buffer_pos = 0;
        puts("[ESP8266] error: buffer too small.\r");
    }
    if ((rxStatus == WaitCmdResponse || rxStatus == Idle) && cmd_buffer_pos == 5) {
        if (strncmp(cmd_recv_buffer, "+IPD,", 5) == 0) {
            cmd_buffer_pos = 0;
            data_buffer_pos = 0;
            rxStatus = RecvIpd;
        }
    } else if (rxStatus == RecvIpd && esp8266_recv_data == ':') {
        int link_id;
        sscanf(cmd_recv_buffer, "%d,%d:", &link_id, &data_len);
        if (data_len > DATA_RECV_BUFFER_SIZE) {
            printf("[ESP8266] error: sent data too long(%d).\r", data_len);
            return;
        }
        rxStatus = RecvData;
        HAL_UART_Receive_IT(uart, (uint8_t *) data_recv_buffer, data_len);
    } else if (rxStatus == RecvData) {
        data_buffer_pos += data_len;
        rxStatus = Idle;
    }
    HAL_UART_Receive_IT(uart, &esp8266_recv_data, 1);
}

void esp8266_init_ap() {
    uart = &huart5;
    HAL_UART_Receive_IT(uart, &esp8266_recv_data, 1);

    esp8266_cmd_tx_string("AT+RST");
    esp8266_cmd_tx_string("AT+CWSAP_CUR=\"zgst-conn\",\"zeitgeist\",5,3");
    esp8266_cmd_tx_string("AT+CIPMUX=1");
    esp8266_cmd_tx_string("AT+CIPSERVER=1,9331");

    printf("[ESP8266] waiting for connection...\r\n");
}
