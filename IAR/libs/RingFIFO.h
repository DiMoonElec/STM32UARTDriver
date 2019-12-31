/******************************************************************************
File:   RingFIFO.h
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


#ifndef __RING_FIFO_H__
#define __RING_FIFO_H__

#include <stdint.h>

typedef struct 
{
  uint16_t Len;
  uint16_t Head;
  uint16_t NumOfItems;
  uint8_t *Buff;
} RingBuff_t;


//Инициализация кольцевого буфера
//
//buff - указатель на структуру буфера
//Mem - массив, в котором будет хранится буфер
//Len - Длинна массива Mem
void RingBuffInit(RingBuff_t *buff, uint8_t *Mem, uint16_t Len);


//Положить элемент в буфер
//
//buff - указатель на структуру буфера
//val - записываемое значение
//
void RingBuffPut(RingBuff_t *buff, uint8_t val);


//Получить очередное значение из кольцевого буфера
//
//buff - указатель на структуру буфера
//
//Возвращает: 
// -1 - буфер пуст
// 0..255 - значение
int16_t RingBuffGet(RingBuff_t *buff);


//Плучить количество доступных элементов в буфере
//
//buff - указатель на структуру буфера
//
//Возвращает: количество занятых элементов
uint16_t RingBuffNumOfItems(RingBuff_t *buff);

//Удалить все элементы из буфера
//
//buff - указатель на структуру буфера
//
void RingBuffClear(RingBuff_t *buff);

                    
#endif