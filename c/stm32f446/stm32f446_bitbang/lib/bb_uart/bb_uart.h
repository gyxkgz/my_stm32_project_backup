#pragma once
#include "common.h"
#include "tim.h"

#define SAMPLERATE 3    //1:normal rate,>1: oversample  
#define BAUDRATE 115200
bool bb_uart_rx_flag;
uint32_t bb_uart_txbuff[10*SAMPLERATE];
uint16_t bb_uart_rxbuff[10*SAMPLERATE];
void bb_uart_init(uint32_t *buff,uint16_t pin_index);
void bb_uart_data_set(uint32_t* buff,uint16_t pin_index, uint8_t value);
void bb_uart_data_clear(uint32_t *buff,uint16_t pin_index);
void bb_uart_tx();
void bb_uart_rx();
uint8_t  bb_uart_rx_decode(uint16_t *buff,uint16_t pin_index);