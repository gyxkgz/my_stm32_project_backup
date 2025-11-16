#include "bb_uart.h"

uint32_t bb_uart_txbuff[10*SAMPLERATE];
uint16_t bb_uart_rxbuff[10*SAMPLERATE];
bool bb_uart_rx_flag=false;

void bb_uart_init(uint32_t *buff,uint16_t pin_index){
    for(int i=0;i<SAMPLERATE;i++){
        buff[i]|=pin_index<<16;
        buff[9*SAMPLERATE+i]|=pin_index;
    }
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}
void bb_uart_data_set(uint32_t* buff,uint16_t pin_index, uint8_t value){
    for (int i=1;i<=8;i++)
    {
        if(value&0x1)
        {
            for(int j=0;j<SAMPLERATE;j++)
                buff[i*SAMPLERATE+j]|=pin_index;
        }else{
            for(int j=0;j<SAMPLERATE;j++)
                buff[i*SAMPLERATE+j]|=pin_index<<16;
        }
        value>>=1;
    }
}
void bb_uart_data_clear(uint32_t *buff,uint16_t pin_index){
    for (int i=1;i<=8;i++)
    {
        for(int j=0;j<SAMPLERATE;j++)
            buff[i*SAMPLERATE+j]&=~(pin_index|pin_index<<16);
    }
}
uint8_t  bb_uart_rx_decode(uint16_t *buff,uint16_t pin_index){
    uint8_t val=0;
    for(int i=1;i<=8;i++){
        if(buff[i*SAMPLERATE+SAMPLERATE/2]&pin_index){
            val|=1<<(i-1);
        }
    }
    return val;
}

void bb_uart_tx(){
    HAL_DMA_Start_IT(&hdma_tim1_ch1,(uint32_t)bb_uart_txbuff,(uint32_t)&GPIOC->BSRR,10*SAMPLERATE);   
    __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC1);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
}
void bb_uart_rx(){
    HAL_DMA_Start_IT(&hdma_tim1_ch2,(uint32_t)&GPIOC->IDR,(uint32_t)bb_uart_rxbuff,9*SAMPLERATE);      
    __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC2);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC2);
}

