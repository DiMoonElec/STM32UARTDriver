/******************************************************************************
File:   uart.c
MPU:    STM32F103xx
Ver     1.0
Date:   2019/06/26
Autor:  Sivokon Dmitriy aka DiMoon Electronics
*******************************************************************************
BSD 2-Clause License
Copyright (c) 2019, Sivokon Dmitriy aka DiMoon Electronics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "stm32f1xx.h"
#include "uart.h"


#if defined(UART1_USE_RING_BUFF)
#define RING_CODE_EN
#endif

#if defined(UART2_USE_RING_BUFF)
#define RING_CODE_EN
#endif

#if defined(UART3_USE_RING_BUFF)
#define RING_CODE_EN
#endif

#if defined(UART4_USE_RING_BUFF)
#define RING_CODE_EN
#endif


#ifdef RING_CODE_EN

#include "RingFIFO.h"

#endif

#define BRR_CALC(freq, baud)    ((freq)/(baud))


/******************************************************************************/

static int16_t _uart_init(USART_TypeDef *UARTx, const UARTInitStructure_t *init);
static void _uart_en(USART_TypeDef *UARTx);

#ifdef RING_CODE_EN

static void TXEIEnable(USART_TypeDef *UARTx);
static void TXEIDisable(USART_TypeDef *UARTx);
static void RXNEIEnable(USART_TypeDef *UARTx);

/******************************************************************************/

static int16_t ring_put(IRQn_Type IRQn, 
                        USART_TypeDef *UARTx, 
                        uint16_t BuffLen, 
                        RingBuff_t *fifo, 
                        uint8_t c);
                        
static int16_t ring_get(IRQn_Type IRQn, 
                        RingBuff_t *fifo);

#endif

/******************************************************************************/

#define RXNE(UARTx)     (UARTx->SR & USART_SR_RXNE)
#define TXE(UARTx)      (UARTx->SR & USART_SR_TXE)

#define RXNEIE(UARTx)   (UARTx->CR1 & USART_CR1_RXNEIE)

/******************************************************************************/

static void TxPinInit(__IO uint32_t *CRx, 
                      uint32_t GPIO_CRL_MODEx_Pos, 
                      uint32_t GPIO_CRL_CNFx_Pos);


static void RxPinInit(__IO uint32_t *CRx, 
                      __IO uint32_t *BSRR,
                      uint32_t GPIO_CRL_MODEx_Pos, 
                      uint32_t GPIO_CRL_CNFx_Pos,
                      uint32_t PinN);
/******************************************************************************/


#ifdef UART1_USE_RING_BUFF

static RingBuff_t tx_fifo1;
static uint8_t tx_buff1[UART1_TXBUFF_LENGHT];

static RingBuff_t rx_fifo1;
static uint8_t rx_buff1[UART1_RXBUFF_LENGHT];

#endif

#ifdef UART2_USE_RING_BUFF

static RingBuff_t tx_fifo2;
static uint8_t tx_buff2[UART2_TXBUFF_LENGHT];

static RingBuff_t rx_fifo2;
static uint8_t rx_buff2[UART2_RXBUFF_LENGHT];

#endif

#ifdef UART3_USE_RING_BUFF

static RingBuff_t tx_fifo3;
static uint8_t tx_buff3[UART3_TXBUFF_LENGHT];

static RingBuff_t rx_fifo3;
static uint8_t rx_buff3[UART3_RXBUFF_LENGHT];

#endif

#ifdef UART4_USE_RING_BUFF

static RingBuff_t tx_fifo4;
static uint8_t tx_buff4[UART4_TXBUFF_LENGHT];

static RingBuff_t rx_fifo4;
static uint8_t rx_buff4[UART4_RXBUFF_LENGHT];

#endif


/******************************************************************************/


int16_t UART_Init(uint8_t id, const UARTInitStructure_t *init)
{
  
#ifdef UART1_ENABLE
  
  /*
  ПОРТЫ:
        DEFAULT  REMAP
  TX1     PA9     PB6
  RX1     PA10    PB7
  
  ШИНА:
  APB2
  */
  
  if(id == 1)
  {
    //Включаем тактирование модуля UART
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    
    //Настройка портов

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
      
      //настройка TX (PA9)
    TxPinInit(&GPIOA->CRH, 
              GPIO_CRH_MODE9_Pos, 
              GPIO_CRH_CNF9_Pos);
      
      //настройка RX (PA10)
    RxPinInit(&GPIOA->CRH, 
              &GPIOA->BSRR, 
              GPIO_CRH_MODE10_Pos, 
              GPIO_CRH_CNF10_Pos, 
              10);
    
    //Сброс модуля
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    asm("nop");
    asm("nop");
    asm("nop");
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
    asm("nop");
    asm("nop");
    asm("nop");

    //Инициализация основных регистров UART
    if(_uart_init(USART1, init) < 0)
      return -1;
    
    
#ifdef UART1_USE_RING_BUFF
    RingBuffInit(&tx_fifo1, tx_buff1, UART1_TXBUFF_LENGHT);
    RingBuffInit(&rx_fifo1, rx_buff1, UART1_RXBUFF_LENGHT);
    
    RXNEIEnable(USART1);
    
    NVIC_EnableIRQ(USART1_IRQn);
    
#endif
    
    //Запускаем UART
    _uart_en(USART1);
    
    
    
    return 0;
  }
  
#endif

#ifdef UART2_ENABLE
  
  
  /*
  ПОРТЫ:
        DEFAULT  REMAP
  TX2     PA2     PD5
  RX2     PA3     PD6
  
  ШИНА:
  APB1
  
  */
  
  
  if(id == 2)
  {
    //Включаем тактирование модуля UART
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    
    //настройка TX (PA2)
    TxPinInit(&GPIOA->CRL, 
              GPIO_CRL_MODE2_Pos, 
              GPIO_CRL_CNF2_Pos);
    
    //настройка RX (PA3)
    RxPinInit(&GPIOA->CRL, 
              &GPIOA->BSRR, 
              GPIO_CRL_MODE3_Pos, 
              GPIO_CRL_CNF3_Pos, 
              3);
    
    //Сброс модуля
    RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
    asm("nop");
    asm("nop");
    asm("nop");
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
    asm("nop");
    asm("nop");
    asm("nop");
    
    
    //Инициализация основных регистров UART
    if(_uart_init(USART2, init) < 0)
      return -1;
    
#ifdef UART2_USE_RING_BUFF
    RingBuffInit(&tx_fifo2, tx_buff2, UART2_TXBUFF_LENGHT);
    RingBuffInit(&rx_fifo2, rx_buff2, UART2_RXBUFF_LENGHT);
    
    RXNEIEnable(USART2);
    
    NVIC_EnableIRQ(USART2_IRQn);
#endif
    
    //Запускаем UART
    _uart_en(USART2);
    
    
    return 0;
  }
  
#endif  
 
#ifdef UART3_ENABLE
    /*
  ПОРТЫ:
        DEFAULT  REMAP
  TX3    PB10     PD8
  RX3    PB11     PD9
  
  ШИНА:
  APB1
  
  */
  
  
  if(id == 3)
  {
    //Включаем тактирование модуля UART
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;    
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    
    //настройка TX (PB10)
    TxPinInit(&GPIOB->CRH, 
              GPIO_CRH_MODE10_Pos, 
              GPIO_CRH_CNF10_Pos);
    
    //настройка RX (PB11)
    RxPinInit(&GPIOB->CRH, 
              &GPIOB->BSRR, 
              GPIO_CRH_MODE11_Pos, 
              GPIO_CRH_CNF11_Pos, 
              11);
    
    //Сброс модуля
    RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
    asm("nop");
    asm("nop");
    asm("nop");
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
    asm("nop");
    asm("nop");
    asm("nop");
    
    //Инициализация основных регистров UART
    if(_uart_init(USART3, init) < 0)
      return -1;
    
#ifdef UART3_USE_RING_BUFF
    RingBuffInit(&tx_fifo3, tx_buff3, UART3_TXBUFF_LENGHT);
    RingBuffInit(&rx_fifo3, rx_buff3, UART3_RXBUFF_LENGHT);
    
    RXNEIEnable(USART3);
    
    NVIC_EnableIRQ(USART3_IRQn);
#endif
    
    //Запускаем UART
    _uart_en(USART3);
    
    
    return 0;
  }
  
#endif
  
  
#ifdef UART4_ENABLE
    /*
  ПОРТЫ:
  TX4    PC10  
  RX4    PC11  
  
  ШИНА:
  APB1
  */
  
  
  if(id == 4)
  {
    //Включаем тактирование модуля UART
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;    
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    
    //настройка TX (PC10)
    TxPinInit(&GPIOC->CRH, 
              GPIO_CRH_MODE10_Pos, 
              GPIO_CRH_CNF10_Pos);
    
    //настройка RX (PC11)
    RxPinInit(&GPIOC->CRH, 
              &GPIOC->BSRR, 
              GPIO_CRH_MODE11_Pos, 
              GPIO_CRH_CNF11_Pos, 
              11);
    
    //Сброс модуля
    RCC->APB1RSTR |= RCC_APB1RSTR_UART4RST;
    asm("nop");
    asm("nop");
    asm("nop");
    RCC->APB1RSTR &= ~RCC_APB1RSTR_UART4RST;
    asm("nop");
    asm("nop");
    asm("nop");
    
    //Инициализация основных регистров UART
    if(_uart_init(UART4, init) < 0)
      return -1;
    
#ifdef UART4_USE_RING_BUFF
    RingBuffInit(&tx_fifo4, tx_buff4, UART4_TXBUFF_LENGHT);
    RingBuffInit(&rx_fifo4, rx_buff4, UART4_RXBUFF_LENGHT);
    
    RXNEIEnable(UART4);
    
    NVIC_EnableIRQ(UART4_IRQn);
#endif
    
    //Запускаем UART
    _uart_en(UART4);
    
    
    return 0;
  }
  
#endif
  
  return -1;
  
}



int16_t UART_PutC(uint8_t id, char c)
{ 
  switch(id)
  {
  ////////////////////////////////////////
#ifdef UART1_ENABLE
  case 1:    
#ifdef UART1_USE_RING_BUFF
    return ring_put(USART1_IRQn, 
                    USART1, 
                    UART1_TXBUFF_LENGHT, 
                    &tx_fifo1, 
                    c);
#else
    
    if(TXE(USART1))
    {
      USART1->DR = c;
      return c;
    }
    else
      return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
#ifdef UART2_ENABLE
  case 2:    
#ifdef UART2_USE_RING_BUFF
    return ring_put(USART2_IRQn, 
                    USART2, 
                    UART2_TXBUFF_LENGHT, 
                    &tx_fifo2, 
                    c);
#else
    
    if(TXE(USART2))
    {
      USART2->DR = c;
      return c;
    }
    else
      return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
#ifdef UART3_ENABLE
  case 3:    
#ifdef UART3_USE_RING_BUFF
    return ring_put(USART3_IRQn, 
                    USART3, 
                    UART3_TXBUFF_LENGHT, 
                    &tx_fifo3, 
                    c);
#else
    
    if(TXE(USART3))
    {
      USART3->DR = c;
      return c;
    }
    else
      return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
  ////////////////////////////////////////
#ifdef UART4_ENABLE
  case 4:
#ifdef UART4_USE_RING_BUFF
    return ring_put(UART4_IRQn, 
                    UART4, 
                    UART4_TXBUFF_LENGHT, 
                    &tx_fifo4, 
                    c);
#else
    
    if(TXE(UART4))
    {
      UART4->DR = c;
      return c;
    }
    else
      return -1;
#endif
    break;
#endif
  }
  return -1;
}


int16_t UART_GetC(uint8_t id)
{
  
  switch(id)
  {
    
  ////////////////////////////////////////
#ifdef UART1_ENABLE
  case 1:
#ifdef UART1_USE_RING_BUFF
    return ring_get(USART1_IRQn, &rx_fifo1);
#else
    if(RXNE(USART1))
      return USART1->DR;
    else
      return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
#ifdef UART2_ENABLE
  case 2:
#ifdef UART2_USE_RING_BUFF
    return ring_get(USART2_IRQn, &rx_fifo2);
#else
    if(RXNE(USART2))
      return USART2->DR;
    else
      return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
#ifdef UART3_ENABLE
  case 3:
#ifdef UART3_USE_RING_BUFF
    return ring_get(USART3_IRQn, &rx_fifo3);
#else
    if(RXNE(USART3))
      return USART3->DR;
    else
      return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
#ifdef UART4_ENABLE
  case 4:
#ifdef UART4_USE_RING_BUFF
    return ring_get(UART4_IRQn, &rx_fifo4);
#else
    if(RXNE(UART4))
      return UART4->DR;
    else
      return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
  }
  
  return -1;
}

#ifdef RING_CODE_EN

int16_t UART_BytesToRead(uint8_t id)
{
  switch(id)
  {
  ////////////////////////////////////////
#ifdef UART1_ENABLE
  case 1:
#ifdef UART1_USE_RING_BUFF
    return RingBuffNumOfItems(&rx_fifo1);
#else
    return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
#ifdef UART2_ENABLE
  case 2:
#ifdef UART2_USE_RING_BUFF
    return RingBuffNumOfItems(&rx_fifo2);
#else
    return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
#ifdef UART3_ENABLE
  case 3:
#ifdef UART3_USE_RING_BUFF
    return RingBuffNumOfItems(&rx_fifo3);
#else
    return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
#ifdef UART4_ENABLE
  case 4:
#ifdef UART4_USE_RING_BUFF
    return RingBuffNumOfItems(&rx_fifo4);
#else
    return -1;
#endif
    break;
#endif
  ////////////////////////////////////////

  }
  
  return -1;
}


int16_t UART_BytesToWrite(uint8_t id)
{
  switch(id)
  {
  ////////////////////////////////////////
#ifdef UART1_ENABLE
  case 1:
#ifdef UART1_USE_RING_BUFF
    return RingBuffNumOfItems(&tx_fifo1);
#else
    return -1;
#endif
    break;
#endif
  ////////////////////////////////////////
#ifdef UART2_ENABLE
  case 2:
#ifdef UART2_USE_RING_BUFF
    return RingBuffNumOfItems(&tx_fifo2);
#else
    return -1;
#endif
    break;   
#endif
  ////////////////////////////////////////
#ifdef UART3_ENABLE
  case 3:
#ifdef UART3_USE_RING_BUFF
    return RingBuffNumOfItems(&tx_fifo3);
#else
    return -1;
#endif
    break;   
#endif
  ////////////////////////////////////////
#ifdef UART4_ENABLE
  case 4:
#ifdef UART4_USE_RING_BUFF
    return RingBuffNumOfItems(&tx_fifo4);
#else
    return -1;
#endif
    break;   
#endif
  ////////////////////////////////////////
    
  }
  
  return -1;
}


void UART_ReadBuffClear(uint8_t id)
{
  switch(id)
  {
  ////////////////////////////////////////
#ifdef UART1_USE_RING_BUFF
  case 1:
    RingBuffClear(&rx_fifo1);
    break;
#endif
  ////////////////////////////////////////
#ifdef UART2_USE_RING_BUFF
  case 2:
    RingBuffClear(&rx_fifo2);
    break;
#endif
  ////////////////////////////////////////
#ifdef UART3_USE_RING_BUFF
  case 3:
    RingBuffClear(&rx_fifo3);
    break;
#endif
  ////////////////////////////////////////
#ifdef UART4_USE_RING_BUFF
  case 4:
    RingBuffClear(&rx_fifo4);
    break;
#endif
  ////////////////////////////////////////
  }
}


void UART_WriteBuffClear(uint8_t id)
{
  switch(id)
  {
  ////////////////////////////////////////
#ifdef UART1_USE_RING_BUFF    
  case 1:
    RingBuffClear(&tx_fifo1);
    break;
#endif
  ////////////////////////////////////////
#ifdef UART2_USE_RING_BUFF    
  case 2:
    RingBuffClear(&tx_fifo2);
    break;
#endif
  ////////////////////////////////////////
#ifdef UART3_USE_RING_BUFF    
  case 3:
    RingBuffClear(&tx_fifo3);
    break;
#endif
  ////////////////////////////////////////
#ifdef UART4_USE_RING_BUFF   
  case 4:
    RingBuffClear(&tx_fifo4);
    break;
#endif
  ////////////////////////////////////////
  }
}

#endif

/******************************************************************************/
static int16_t _uart_init(USART_TypeDef *UARTx, const UARTInitStructure_t *init)
{
  uint32_t UARTx_CR1_VAL = 0;
  
  
  //установка скорости UART
  UARTx->BRR = (uint16_t)BRR_CALC(init->bus_freq, init->baud);
  
  
  UARTx_CR1_VAL |= USART_CR1_TE | USART_CR1_RE;
  
  ///////////////////////////////////////
  
  switch(init->data_bits)
  {
  case 8:
    break;
   
  case 9:
    UARTx_CR1_VAL |= USART_CR1_M;
    break;
    
  default:
   return -1;
  }
  
  
  ///////////////////////////////////////
  
  switch(init->parity)
  {
  case 0: //None
    break;
    
  case 1: //Even
    UARTx_CR1_VAL |= USART_CR1_PCE;
    break;
    
  case 2: //Odd
    UARTx_CR1_VAL |= USART_CR1_PCE | USART_CR1_PS;
    break;
    
  default:
   return -1;
  }
  
  UARTx->CR1 = UARTx_CR1_VAL;
  
  ///////////////////////////////////////
  
  switch(init->stop_bits)
  {
  case 1:
    break;
    
  case 2:
    UARTx->CR2 |= (0x02 << USART_CR2_STOP_Pos);
    break;
    
  default:
   return -1;
  }
  
  
  return 0;
}

static void _uart_en(USART_TypeDef *UARTx)
{
  UARTx->CR1 |= USART_CR1_UE;
}

/******************************************************************************/



static void TxPinInit(__IO uint32_t *CRx, 
                      uint32_t GPIO_CRL_MODEx_Pos, 
                      uint32_t GPIO_CRL_CNFx_Pos)
{
  *CRx &= ~((0x03 << GPIO_CRL_MODEx_Pos) | (0x03 << GPIO_CRL_CNFx_Pos));
  *CRx |= (0x01 << GPIO_CRL_MODEx_Pos) //01: Output mode, max speed 10 MHz.
        | (0x02 << GPIO_CRL_CNFx_Pos); //10: Alternate function output Push-pull
}


static void RxPinInit(__IO uint32_t *CRx, 
                      __IO uint32_t *BSRR,
                      uint32_t GPIO_CRL_MODEx_Pos, 
                      uint32_t GPIO_CRL_CNFx_Pos,
                      uint32_t PinN)
{
  *CRx &= ~((0x03 << GPIO_CRL_MODEx_Pos) | (0x03 << GPIO_CRL_CNFx_Pos));
  *CRx |= (0x00 << GPIO_CRL_MODEx_Pos) //00: Input mode (reset state)
    | (0x02 << GPIO_CRL_CNFx_Pos); //10: Input with pull-up / pull-down
  *BSRR = 1 << PinN; //pull-up
}


/******************************************************************************/

#ifdef RING_CODE_EN

static void TXEIEnable(USART_TypeDef *UARTx)
{
  UARTx->CR1 |= USART_CR1_TXEIE;
}

static void TXEIDisable(USART_TypeDef *UARTx)
{
  UARTx->CR1 &= ~USART_CR1_TXEIE;
}

static void RXNEIEnable(USART_TypeDef *UARTx)
{
  UARTx->CR1 |= USART_CR1_RXNEIE;
}

/******************************************************************************/

static int16_t ring_get(IRQn_Type IRQn, 
                        RingBuff_t *fifo)
{
  int16_t ret;
  
  NVIC_DisableIRQ(IRQn);
  ret = RingBuffGet(fifo);
  NVIC_EnableIRQ(IRQn);
  
  return ret;
}


static int16_t ring_put(IRQn_Type IRQn, 
                        USART_TypeDef *UARTx, 
                        uint16_t BuffLen, 
                        RingBuff_t *fifo, 
                        uint8_t c)
{
  int16_t ret;
  
  NVIC_DisableIRQ(IRQn);
  if((BuffLen - RingBuffNumOfItems(fifo)) > 0)
  {
    RingBuffPut(fifo, c);
    ret = c;
    TXEIEnable(UARTx); //запускаем передачу
  }
  else
  {
    ret = -1;
  }
  NVIC_EnableIRQ(IRQn);
  
  return ret;
}


#endif

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#ifdef UART1_USE_RING_BUFF

#define USARTx                  USART1
#define RX_FIFOx                rx_fifo1
#define TX_FIFOx                tx_fifo1
#define USARTx_IRQHandler       USART1_IRQHandler



void USARTx_IRQHandler(void)
{
  static uint16_t tmp;
  
  //Если прерывание приема разрешено и что-то получили
  if(RXNEIE(USARTx) && RXNE(USARTx))
  {
    tmp = USARTx->DR;
    RingBuffPut(&RX_FIFOx, tmp);
  }
  
  
  if(TXE(USARTx)) //Если передачтик пуст
  {
    if(RingBuffNumOfItems(&TX_FIFOx) > 0) //если есть, что передавать
    {
      tmp = RingBuffGet(&TX_FIFOx);
      USARTx->DR = (uint8_t)tmp;
    }
    else //если передавать нечего
    {
      TXEIDisable(USARTx); //отключаем прерывание
    }
  }
}

#undef USARTx
#undef RX_FIFOx
#undef TX_FIFOx
#undef USARTx_IRQHandler

#endif


#ifdef UART2_USE_RING_BUFF

#define USARTx                  USART2
#define RX_FIFOx                rx_fifo2
#define TX_FIFOx                tx_fifo2
#define USARTx_IRQHandler       USART2_IRQHandler



void USARTx_IRQHandler(void)
{
  static uint16_t tmp;
  
  //Если прерывание приема разрешено и что-то получили
  if(RXNEIE(USARTx) && RXNE(USARTx))
  {
    tmp = USARTx->DR;
    RingBuffPut(&RX_FIFOx, tmp);
  }
  
  
  if(TXE(USARTx)) //Если передачтик пуст
  {
    if(RingBuffNumOfItems(&TX_FIFOx) > 0) //если есть, что передавать
    {
      tmp = RingBuffGet(&TX_FIFOx);
      USARTx->DR = (uint8_t)tmp;
    }
    else //если передавать нечего
    {
      TXEIDisable(USARTx); //отключаем прерывание
    }
  }
}

#undef USARTx
#undef RX_FIFOx
#undef TX_FIFOx
#undef USARTx_IRQHandler

#endif


#ifdef UART3_USE_RING_BUFF

#define USARTx                  USART3
#define RX_FIFOx                rx_fifo3
#define TX_FIFOx                tx_fifo3
#define USARTx_IRQHandler       USART3_IRQHandler



void USARTx_IRQHandler(void)
{
  static uint16_t tmp;
  
  //Если прерывание приема разрешено и что-то получили
  if(RXNEIE(USARTx) && RXNE(USARTx))
  {
    tmp = USARTx->DR;
    RingBuffPut(&RX_FIFOx, tmp);
  }
  
  
  if(TXE(USARTx)) //Если передачтик пуст
  {
    if(RingBuffNumOfItems(&TX_FIFOx) > 0) //если есть, что передавать
    {
      tmp = RingBuffGet(&TX_FIFOx);
      USARTx->DR = (uint8_t)tmp;
    }
    else //если передавать нечего
    {
      TXEIDisable(USARTx); //отключаем прерывание
    }
  }
}

#undef USARTx
#undef RX_FIFOx
#undef TX_FIFOx
#undef USARTx_IRQHandler

#endif


#ifdef UART4_USE_RING_BUFF

#define USARTx                  UART4
#define RX_FIFOx                rx_fifo4
#define TX_FIFOx                tx_fifo4
#define USARTx_IRQHandler       UART4_IRQHandler



void USARTx_IRQHandler(void)
{
  static uint16_t tmp;
  
  //Если прерывание приема разрешено и что-то получили
  if(RXNEIE(USARTx) && RXNE(USARTx))
  {
    tmp = USARTx->DR;
    RingBuffPut(&RX_FIFOx, tmp);
  }
  
  
  if(TXE(USARTx)) //Если передачтик пуст
  {
    if(RingBuffNumOfItems(&TX_FIFOx) > 0) //если есть, что передавать
    {
      tmp = RingBuffGet(&TX_FIFOx);
      USARTx->DR = (uint8_t)tmp;
    }
    else //если передавать нечего
    {
      TXEIDisable(USARTx); //отключаем прерывание
    }
  }
}

#undef USARTx
#undef RX_FIFOx
#undef TX_FIFOx
#undef USARTx_IRQHandler

#endif

