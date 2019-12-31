/******************************************************************************
File:   uart.h
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

#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

#define UART1_ENABLE
#define UART2_ENABLE
#define UART3_ENABLE
//#define UART4_ENABLE


#ifdef UART1_ENABLE

#define UART1_USE_RING_BUFF
#define UART1_TXBUFF_LENGHT     16
#define UART1_RXBUFF_LENGHT     16

#endif

#ifdef UART2_ENABLE

#define UART2_USE_RING_BUFF
#define UART2_TXBUFF_LENGHT     16
#define UART2_RXBUFF_LENGHT     16

#endif


#ifdef UART3_ENABLE

#define UART3_USE_RING_BUFF
#define UART3_TXBUFF_LENGHT     16
#define UART3_RXBUFF_LENGHT     16

#endif

#ifdef UART4_ENABLE

#define UART4_USE_RING_BUFF
#define UART4_TXBUFF_LENGHT     16
#define UART4_RXBUFF_LENGHT     16

#endif

typedef struct
{
  uint32_t bus_freq;    //частота шины uart
  uint32_t baud;        //скорость передачи
  uint8_t data_bits;    //количество бит данных (8, 9)
  uint8_t stop_bits;    //количество стоп-бит (1 или 2)
  uint8_t parity;       //контроль четности (0 - нет, 1 - even, 2 - odd)
} UARTInitStructure_t;


//Инициализация UART
//
//id - номер порта
//init - структура инициализации UART
//
//Return: 0 - успех, -1 - ошибка инициализации
int16_t UART_Init(uint8_t id, const UARTInitStructure_t *init);

//Записать символ в буфер передатчика
//
//id - номер порта
//с - передававемый символ
//
//Retrun: с - успех, -1 - ошибка, буфер переполнен
int16_t UART_PutC(uint8_t id, char c);

//Прочитать симпов из буфера приемника
//
//id - номер порта
//
//Return: -1 - нет данных для чтения, 0..255 - прочитанный символ
int16_t UART_GetC(uint8_t id);

//Получить количество непрочитанных байт в буфере приемника
int16_t UART_BytesToRead(uint8_t id);

//Получить количество еще не отправленных байт в буфере передатчика
int16_t UART_BytesToWrite(uint8_t id);

//Очистить буфер приемника
void UART_ReadBuffClear(uint8_t id);

//Очистить буфер передатчика
void UART_WriteBuffClear(uint8_t id);


#endif

