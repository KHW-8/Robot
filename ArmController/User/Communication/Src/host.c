#include "host.h"
// STD
#include <string.h>
// ArmController
#include "global.h"
#include "usart.h"

void test_host() {
    uint8_t arr[6] = { 0x68, 0x65, 0x6C, 0x6C, 0x6F }; // hello

    // Send packet
    for (uint32_t i = 0; i < 6; i++) {
        // Wait until TDR is empty (TXE flag is set)
        while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE));
        
        huart1.Instance->DR = (arr[i] & 0xFF);
    }
}

int send_msg_to_host(const char *buf) {
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);

    return OK;
}