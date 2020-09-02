#include <string.h>

#include "spi.h"
#include "dot_matrix.h"

#define COL_NUM 8
#define SHOW_DELAY 10000
#define SHOW_SPEED 2

typedef enum {
    Vertical,
    Horizontal,
} MotionDirection;

MotionDirection motion_direction = Horizontal;

uint8_t g_dot_start = 0;
uint16_t g_dot_cnt = 0;

int display_buffer_available = 0;
uint8_t display_buffer[8] = {0, };

#define DISPLAY_CHARS 8
#define MATRIX_COL (DISPLAY_CHARS * 8)

uint8_t g_ShowData[MATRIX_COL] = {
        0x81, 0xFF, 0xFF, 0x89, 0x89, 0xFF, 0x76, 0x00,
        0x01, 0x7F, 0xFF, 0x80, 0x80, 0xFF, 0x7F, 0x01,
        0x00, 0x81, 0xFF, 0xFF, 0x91, 0x91, 0x1F, 0x0E,
        0x00, 0x07, 0x81, 0xFF, 0xFF, 0x81, 0x07, 0x00,

        0x81, 0xff, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3c,
        0x81, 0xff, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3c,
        0x81, 0xff, 0x8d, 0x0c, 0x0c, 0x8d, 0xff, 0x81,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

void dot_matrix_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_7);
    HAL_GPIO_DeInit(DOT_EN_GPIO_Port, DOT_EN_Pin);
    HAL_GPIO_DeInit(DOT_SHIFT_GPIO_Port, DOT_SHIFT_Pin);
    HAL_GPIO_DeInit(DOT_LAT_GPIO_Port, DOT_LAT_Pin);

    HAL_SPI_MspInit(&hspi1);

    HAL_GPIO_WritePin(DOT_EN_GPIO_Port, DOT_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DOT_SHIFT_GPIO_Port, DOT_SHIFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DOT_LAT_GPIO_Port, DOT_LAT_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = DOT_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DOT_EN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DOT_SHIFT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DOT_SHIFT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DOT_LAT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DOT_LAT_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DOT_LAT_GPIO_Port, DOT_LAT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DOT_EN_GPIO_Port, DOT_EN_Pin, GPIO_PIN_SET);
    for (uint8_t i = 0; i < COL_NUM; i++) {
        HAL_GPIO_WritePin(DOT_SHIFT_GPIO_Port, DOT_SHIFT_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DOT_SHIFT_GPIO_Port, DOT_SHIFT_Pin, GPIO_PIN_SET);
    }
}

void move_dot_matrix(void) {
    g_dot_cnt++;

    if (g_dot_cnt > SHOW_SPEED) { // SHOW_SPEED * 100ms
        g_dot_cnt = 0;
        if (g_dot_start < (MATRIX_COL - 1)) {
            g_dot_start++;
        } else {
            g_dot_start = 0;
        }

        if (motion_direction == Horizontal) {
            display_buffer_available = 0;
        }
    }
}

void show_dot_matrix(void) {
    if (motion_direction == Horizontal && !display_buffer_available) {
        memset(display_buffer, 0, 8);
        for (uint8_t x = 0; x < 8; x++) {
            for (uint8_t y = 0; y < 8; y++) {
                uint8_t x_cor = g_dot_start + x;
                if (x_cor >= MATRIX_COL) {
                    x_cor -= MATRIX_COL;
                }
                if (g_ShowData[x_cor] & (1u << y)) {
                    display_buffer[y] |= 1u << (7u - x);
                }
            }
        }
        display_buffer_available = 1;
    }

    for (uint32_t row = 0; row < COL_NUM; row++) {
        uint8_t *data_pos = 0;
        if (motion_direction == Horizontal) {
            data_pos = display_buffer + row;
        } else if (motion_direction == Vertical) {
            uint32_t j = g_dot_start + row;
            if (j > (MATRIX_COL - 1)) {
                j -= MATRIX_COL;
            }
            data_pos = display_buffer + j;
        }

        HAL_SPI_Transmit(&hspi1, data_pos, 1, 0);
        if (row == 0) {
            HAL_GPIO_WritePin(DOT_EN_GPIO_Port, DOT_EN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DOT_SHIFT_GPIO_Port, DOT_SHIFT_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DOT_SHIFT_GPIO_Port, DOT_SHIFT_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(DOT_EN_GPIO_Port, DOT_EN_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(DOT_SHIFT_GPIO_Port, DOT_SHIFT_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DOT_SHIFT_GPIO_Port, DOT_SHIFT_Pin, GPIO_PIN_SET);
        }
        HAL_GPIO_WritePin(DOT_LAT_GPIO_Port, DOT_LAT_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DOT_LAT_GPIO_Port, DOT_LAT_Pin, GPIO_PIN_SET);
        for (uint32_t k = 0; k < SHOW_DELAY; k++) {
        }
    }
}
