/*
File    : main.c
Software "Kurs STM32 PCBtech"
Lesson 9: DMA USART to Memory to DAC.
Student: antigo1989@gmail.com
*/

#include "main.h"
volatile uint8_t user_flags = 0x00;
volatile uint16_t delay_parse_data=0;
uint16_t bufferOUT[1] __attribute__((section(".fast"))) = {0};
char bufferIN[4] __attribute__((section(".fast")));


/*********************************main************************************/
int main(void) {
  //System Initial
  SystemInit();
  RCC_Init();
  __enable_irq();

  DAC_Init();

  USART1_Init();
  DMA2_Stream5_Init();
  
  SysTick_Config(168000);
  TIM2_Init();
  DMA1_Stream1_Init();


  while(1){
    if(delay_parse_data > DELAY_PARSE_DATA){
      delay_parse_data = 0;
      if(user_flags == NEW_RECEIVE){
        uint16_t buffer_tmp = atoi(bufferIN);
        buffer_tmp = buffer_tmp > 3300 ? 3300 : buffer_tmp;
        bufferOUT[0] = (uint16_t)buffer_tmp*4095/3300;
      }
    }
    __NOP();
  }
}
/**************************Interrupt function*****************************/
void SysTick_Handler(void){
  delay_parse_data++;
}

void DMA2_Stream5_IRQHandler(void){
  DMA2->HIFCR |= DMA_HIFCR_CTCIF5|DMA_HIFCR_CHTIF5|DMA_HIFCR_CFEIF5;
  DMA2_Stream5->CR |= DMA_SxCR_EN;
  user_flags = NEW_RECEIVE; 
}

void DMA1_Stream1_IRQHandler(void){
  DMA1->LIFCR |= DMA_LIFCR_CTCIF1|DMA_LIFCR_CHTIF1|DMA_LIFCR_CFEIF1;
  DMA1_Stream1->CR |= DMA_SxCR_EN; 
}
/****************************** function**********************************/
void DMA1_Stream1_Init(void){
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  DMA1_Stream1->PAR = (uint32_t)&DAC->DHR12R2;
  DMA1_Stream1->M0AR = (uint32_t)bufferOUT;
  DMA1_Stream1->NDTR = 1; //length(bufferOUT)
  DMA1_Stream1->FCR &= ~(DMA_SxFCR_DMDIS); //ohne FIFO
  DMA1_Stream1->CR &= ~(DMA_SxCR_CHSEL); //clear channel
  DMA1_Stream1->CR |= (3<<DMA_SxCR_CHSEL_Pos)|DMA_SxCR_PL_1; //channel 3 TIM2, priorety hight
  DMA1_Stream1->CR &= ~(DMA_SxCR_MBURST|DMA_SxCR_PBURST|DMA_SxCR_DBM);
  //DMA1_Stream1->CR &= ~(DMA_SxCR_MSIZE|DMA_SxCR_PSIZE); //clear
  DMA1_Stream1->CR |= DMA_SxCR_MSIZE_0|DMA_SxCR_PSIZE_0; //16 bit
  DMA1_Stream1->CR &= ~(DMA_SxCR_CIRC|DMA_SxCR_DIR|DMA_SxCR_PINC|DMA_SxCR_MINC); //per(inc=const) > mem (inc=const)
  DMA1_Stream1->CR |= DMA_SxCR_DIR_0|DMA_SxCR_TCIE;
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);; 
  //start transfer
  DMA1_Stream1->CR |= DMA_SxCR_EN;

}

void TIM2_Init(void){
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC = 419;
  TIM2->CR1 |= TIM_CR1_URS;
  TIM2->CR1 &= ~(TIM_CR1_CMS|TIM_CR1_DIR);
  TIM2->ARR = 1999;
  TIM2->DIER |= TIM_DIER_UDE;
  TIM2->CR1 |= TIM_CR1_CEN;
  TIM2->EGR |= TIM_EGR_UG;
}

void DAC_Init(void){
  //PA5
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= GPIO_MODER_MODE5;
  //DAC
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  DAC->CR |= DAC_CR_BOFF2|DAC_CR_EN2|DAC_CR_DMAEN2;
  DAC->DHR12R2 = (uint16_t)DEF_OUT*4095/3300;
  DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;
}

void DMA2_Stream5_Init(void){
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  DMA2_Stream5->PAR = (uint32_t)&(USART1->DR);
  DMA2_Stream5->M0AR = (uint32_t)bufferIN;
  DMA2_Stream5->NDTR = 4; //length(bufferIN)
  DMA2_Stream5->FCR &= ~(DMA_SxFCR_DMDIS); //ohne FIFO
  DMA2_Stream5->CR &= ~(DMA_SxCR_CHSEL); //clear channel
  DMA2_Stream5->CR |= (4<<DMA_SxCR_CHSEL_Pos)|DMA_SxCR_PL; //channel 4 USART1_Rx, priorety very hight
  DMA2_Stream5->CR &= ~(DMA_SxCR_MBURST|DMA_SxCR_PBURST|DMA_SxCR_DBM);
  DMA2_Stream5->CR &= ~(DMA_SxCR_MSIZE|DMA_SxCR_PSIZE); //8bit (char)
  DMA2_Stream5->CR &= ~(DMA_SxCR_CIRC|DMA_SxCR_DIR|DMA_SxCR_PINC); //per(inc=const) > mem
  DMA2_Stream5->CR |= DMA_SxCR_MINC|DMA_SxCR_TCIE; //mem(inc++)
  NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  //start transfer
  DMA2_Stream5->CR |= DMA_SxCR_EN;
}

void USART1_Init(void){
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  //PA9 and PA10
  GPIOA->MODER |= GPIO_MODER_MODE9_1|GPIO_MODER_MODE10_1;
  GPIOA->AFR[1] |= (7<<GPIO_AFRH_AFSEL9_Pos)|(7<<GPIO_AFRH_AFSEL10_Pos);
  // 84MHz / 115200bod / 16 = 45,57  M=45 (0x2D) F=0,57*16=9 (0x09)
  USART1->BRR = 0x02D9;
  USART1->CR1 |= USART_CR1_RE;
  USART1->CR1 &= ~(USART_CR1_M|USART_CR1_PCE);
  USART1->CR2 &= ~(USART_CR2_STOP);
  USART1->CR3 |= USART_CR3_DMAR;
  //Enable USART1
  USART1->CR1 |= USART_CR1_UE;
}

/*************************** End of file ****************************/
