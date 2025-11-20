/* Includes */
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cpu_utils.h"
#include "lwrb.h"

#ifdef TX_DEBUG
/*
 * Set this to `1` to enable DMA for TX for UART
 * Set this to `0` to send data using polling mode
 *
 * Observe output on UART and its CPU load
 */
#define USE_DMA_TX              0
#endif

/* Private function prototypes */
void SystemClock_Config(void);

/* USART related functions */
void usart_init(void);
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);
uint8_t usart_start_tx_dma_transfer(void);
/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/**
 * \brief           USART RX buffer for DMA to transfer every received byte
 * \note            Contains raw data that are about to be processed by different events
 */
uint8_t usart_rx_dma_buffer[64];
/* Buffer for data before transmitted over DMA */
#ifdef TX_DEBUG
void tx_debug_thread(void* arg);
static char buff[256];
/* Ring buffer for TX data */
lwrb_t usart_tx_buff;
uint8_t usart_tx_buff_data[1024];
volatile size_t usart_tx_dma_current_len;
/**
 * \brief           Long text to be transmitted
 */
static const char
long_string[] = ""
"Miusov, as a man man of breeding and deilcacy,"
"could not but feel some inwrd qualms, when he reached the Father Superior's with Ivan:"
"he felt ashamed of havin lost his temper."
"He felt that he ought to have disdaimed that despicable wretch,"
"Fyodor Pavlovitch, too much to have been upset by him in Father Zossima's cell,"
"and so to have forgotten himself. \"Teh monks were not to blame,"
"in any case,\" he reflected, on the steps."
"\"And if they\'re decent people here (and the Father Superior,"
"I understand, is a nobleman) why not be friendly and courteous withthem?"
"I won't argue, I'll fall in with everything, I'll win them by politness,"
"and show them that I've nothing to do with that Aesop, thta buffoon,"
"that Pierrot, and have merely been takken in over this affair, just as they have.\r\n"
"";
#endif
#ifdef USE_FREERTOS
const osThreadAttr_t blinkTask_attributes = {
  .name = "blinkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
void blink_task(void* arg);
void usart_rx_dma_thread(void* arg);
#ifdef USE_IRQ
osMessageQueueId_t usart_rx_dma_queue_id;
#endif
#endif

/**
 * \brief           Application entry point
 */
int
main(void) {
    /* MCU Configuration */

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    usart_init();
#ifdef TX_DEBUG
    lwrb_init(&usart_tx_buff, usart_tx_buff_data, sizeof(usart_tx_buff_data));
#endif
#ifdef USE_FREERTOS
    osKernelInitialize();
#ifdef USE_IRQ
    usart_rx_dma_queue_id = osMessageQueueNew(10, sizeof(void *), NULL);
#endif
#ifdef TX_DEBUG
    osThreadNew(tx_debug_thread, NULL, NULL);
#else
    /* Create new thread for USART RX DMA processing */
    osThreadNew(usart_rx_dma_thread, NULL, NULL);
#endif
    osThreadNew(blink_task,NULL,&blinkTask_attributes);
    osKernelStart();
#else //USE_FREERTOS
#ifdef USE_IRQ
    usart_send_string("USART DMA example: DMA HT & TC + USART IDLE LINE interrupts\r\n");
#else
    usart_send_string("USART DMA example: Polling\r\n");
#endif
    usart_send_string("Start sending data to STM32\r\n");
#ifdef USE_POLLING
    uint32_t currticks=HAL_GetTick();
    uint32_t pretick200=currticks;
#endif
    /* Infinite loop */
    while (1) {
        /* Nothing to process here */
        /* Everything is processed either by DMA or USART interrupts */
    #if defined(USE_POLLING)
        usart_rx_check();
        if(currticks-pretick200>200){
            pretick200=currticks;
            LL_GPIO_TogglePin(GPIOA,LL_GPIO_PIN_5);
        }
        currticks=HAL_GetTick();
    #else
        LL_GPIO_TogglePin(GPIOA,LL_GPIO_PIN_5);
        LL_mDelay(200);
    #endif
        /* Do task 1 */
        /* Do task 2 */
        /* Do task 3 */
        /* Do task 4 */
        /* Do task 5 */
    }
#endif

}
#ifdef USE_FREERTOS
#ifdef TX_DEBUG
void
tx_debug_thread(void* arg) {
    uint32_t ticks;
    uint16_t cpu_usage;
    /* Do other initializations if needed */

    /* Main application loop */
    ticks = osKernelGetTickCount();
    while (1) {
        /* Send test data */
        usart_send_string("\r\n-----\r\n");
        usart_send_string(long_string);
       // usart_send_string(long_string);
        usart_send_string("\r\n-----\r\n");

        /* Thread must be executed maximum once per second */
        ticks += 1000;
        osDelayUntil(ticks);

        cpu_usage = osGetCPUUsage();            /* Get CPU load */
        sprintf(buff, "CPU Load: %d%%\r\n", (int)cpu_usage);
        usart_send_string(buff);
    }
}
#endif
void blink_task(void* arg)
{
    while(1){
        LL_GPIO_TogglePin(GPIOA,LL_GPIO_PIN_5);
       // HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
        osDelay(500);
        //vTaskDelay(10);
    }
}
void
usart_rx_dma_thread(void* arg) {
    void* d;

    /* Notify user to start sending data */
#ifdef USE_POLLING
    usart_send_string("USART DMA example: Polling + RTOS\r\n");
#endif
#ifdef USE_IRQ
    usart_send_string("USART DMA example: DMA HT & TC + USART IDLE LINE IRQ + RTOS processing\r\n");
#endif
    usart_send_string("Start sending data to STM32\r\n");

    while (1) {
        /* Block thread and wait for event to process USART data */
#ifdef USE_IRQ
        osMessageQueueGet(usart_rx_dma_queue_id, &d, NULL, osWaitForever);
#endif
        /* Simply call processing function */
        usart_rx_check();
        (void)d;
    }
}
#endif

/**
 * \brief           Check for new data received with DMA
 *
 * User must select context to call this function from:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both context-es, exclusive access protection must be implemented
 * This mode is not advised as it usually means architecture design problems
 *
 * When IDLE interrupt is not present, application must rely only on thread context,
 * by manually calling function as quickly as possible, to make sure
 * data are read from raw buffer and processed.
 *
 * Not doing reads fast enough may cause DMA to overflow unread received bytes,
 * hence application will lost useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
void usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;
    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
        old_pos = pos;                          /* Save current position as old for next transfers */
    }
}

/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void
usart_process_data(const void* data, size_t len) {
    const uint8_t* d = data;
    
    /*
     * This function is called on DMA TC or HT events, and on UART IDLE (if enabled) event.
     * 
     * For the sake of this example, function does a loop-back data over UART in polling mode.
     * Check ringbuff RX-based example for implementation with TX & RX DMA transfer.
     */
    
    for (; len > 0; --len, ++d) {
        LL_USART_TransmitData8(USART2, *d);
        while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
    }
    while (!LL_USART_IsActiveFlag_TC(USART2)) {}
}

/**
 * \brief           Send string to USART
 * \param[in]       str: String to send
 */
void
usart_send_string(const char* str) {
     size_t len = strlen(str);
#if USE_DMA_TX   
    if (lwrb_get_free(&usart_tx_buff) >= len) {
        lwrb_write(&usart_tx_buff, str, len);
        usart_start_tx_dma_transfer();
    }
#else
    usart_process_data(str, len);
#endif
}

#ifdef TX_DEBUG
/**
 * \brief           Checks for data in buffer and starts transfer if not in progress
 */
uint8_t
usart_start_tx_dma_transfer(void) {
    uint32_t primask;
    uint8_t started = 0;

    /*
     * First check if transfer is currently in-active,
     * by examining the value of usart_tx_dma_current_len variable.
     *
     * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
     *
     * It is not necessary to disable the interrupts before checking the variable:
     *
     * When usart_tx_dma_current_len == 0
     *    - This function is called by either application or TX DMA interrupt
     *    - When called from interrupt, it was just reset before the call,
     *         indicating transfer just completed and ready for more
     *    - When called from an application, transfer was previously already in-active
     *         and immediate call from interrupt cannot happen at this moment
     *
     * When usart_tx_dma_current_len != 0
     *    - This function is called only by an application.
     *    - It will never be called from interrupt with usart_tx_dma_current_len != 0 condition
     *
     * Disabling interrupts before checking for next transfer is advised
     * only if multiple operating system threads can access to this function w/o
     * exclusive access protection (mutex) configured,
     * or if application calls this function from multiple interrupts.
     *
     * This example assumes worst use case scenario,
     * hence interrupts are disabled prior every check
     */
     primask = __get_PRIMASK();
     __disable_irq();
    if (usart_tx_dma_current_len == 0
            && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_buff)) > 0) {
        /* Limit maximal size to transmit at a time */
        if (usart_tx_dma_current_len > 32) {
            usart_tx_dma_current_len = 32;
        }
        /* Configure DMA */
        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, usart_tx_dma_current_len);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)lwrb_get_linear_block_read_address(&usart_tx_buff));

        /* Clear all flags */
        LL_DMA_ClearFlag_TC6(DMA1);
        LL_DMA_ClearFlag_HT6(DMA1);
        LL_DMA_ClearFlag_DME6(DMA1);
        LL_DMA_ClearFlag_FE6(DMA1);
        LL_DMA_ClearFlag_TE6(DMA1);

        /* Start transfer */
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
        started = 1;
    }
    __set_PRIMASK(primask);
    return started;
}
#endif

/**
 * \brief           USART2 Initialization Function
 */
void
usart_init(void) {
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /*
     * USART2 GPIO Configuration
     *
     * PA2  ------> USART2_TX
     * PA3  ------> USART2_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, LL_USART_DMA_GetRegAddr(USART2));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, ARRAY_LEN(usart_rx_dma_buffer));
#ifdef TX_DEBUG
  /* USART2_TX Init */
    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6, LL_USART_DMA_GetRegAddr(USART2));
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
        /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
      
#endif
#ifdef USE_IRQ
    /* Enable HT & TC interrupts */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_5);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
    /* DMA interrupt init */
    NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
#endif

    /* USART configuration */
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART2);
    LL_USART_EnableDMAReq_RX(USART2);
    LL_USART_EnableDMAReq_TX(USART2);   //使能串口DMA发送
#ifdef USE_IRQ
    LL_USART_EnableIT_IDLE(USART2);
    /* USART interrupt */
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(USART2_IRQn);
#endif
    /* Enable USART and DMA */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
    LL_USART_Enable(USART2);
}

/* Interrupt handlers here */
#ifdef TX_DEBUG
void
DMA1_Stream6_IRQHandler(void) {
    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_6) && LL_DMA_IsActiveFlag_TC6(DMA1)) {
        LL_DMA_ClearFlag_TC6(DMA1);             /* Clear transfer complete flag */
        lwrb_skip(&usart_tx_buff, usart_tx_dma_current_len);/* Data sent, ignore these */
        usart_tx_dma_current_len = 0;
        usart_start_tx_dma_transfer();          /* Try to send more data */
    }

    /* Implement other events when needed */
}
#endif

#ifdef USE_IRQ
/**
 * \brief           DMA1 stream5 interrupt handler for USART2 RX
 */
void DMA1_Stream5_IRQHandler(void) {
         //   usart_send_string("c\r\n");
#ifdef USE_FREERTOS
        void* d = (void *)1;
#endif
    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_5) && LL_DMA_IsActiveFlag_HT5(DMA1)) {
        LL_DMA_ClearFlag_HT5(DMA1);             /* Clear half-transfer complete flag */
#ifdef USE_FREERTOS
        osMessageQueuePut(usart_rx_dma_queue_id, &d, 0, 0); /* Write data to queue. Do not use wait function! */
#else
        usart_rx_check();                       /* Check for data to process */
#endif
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_5) && LL_DMA_IsActiveFlag_TC5(DMA1)) {
        LL_DMA_ClearFlag_TC5(DMA1);             /* Clear transfer complete flag */
#ifdef USE_FREERTOS
        osMessageQueuePut(usart_rx_dma_queue_id, &d, 0, 0);
#else
        usart_rx_check();                       /* Check for data to process */
#endif
    }
}

/**
 * \brief           USART2 global interrupt handler
 */
void USART2_IRQHandler(void) {
#ifdef USE_FREERTOS
        void* d = (void *)1;
#endif
    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(USART2) && LL_USART_IsActiveFlag_IDLE(USART2)) {
        LL_USART_ClearFlag_IDLE(USART2);        /* Clear IDLE line flag */
#ifdef USE_FREERTOS
        osMessageQueuePut(usart_rx_dma_queue_id, &d, 0, 0);
#else
        usart_rx_check();                       /* Check for data to process */
#endif
    }
}
#endif

/**
 * \brief           System Clock Configuration
 */
void
SystemClock_Config(void) {
    /* Configure flash latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3) {
        while (1) {}
    }

    /* Configure voltage scaling */
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    /* Configure HSI */
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1) {}

    /* Configure PLL */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 100, LL_RCC_PLLP_DIV_2);
    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1) {}

    /* Configure system prescalers */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

    /* Configure systick */
    LL_Init1msTick(100000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SYSTICK_EnableIT();
    LL_SetSystemCoreClock(100000000);
    LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}
