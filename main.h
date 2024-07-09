#ifndef H_MAIN
#define H_MAIN 1

#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"

#define USART_BOUDRATE 115200
//Settings
#define DELAY_PARSE_DATA 100 //ms
#define DEF_OUT 0//mV

//usr_flags
#define NEW_RECEIVE 1

void RCC_Init(void);
void DMA1_Stream1_Init(void);
void TIM2_Init(void);
void DAC_Init(void);
void DMA2_Stream5_Init(void);
void USART1_Init(void);

#endif