/******************************************************************************
File:   RingFIFO.c
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



#include "RingFIFO.h"

/******************************************************************************/

static void RingIncr(uint16_t *val, uint16_t max)
{
  (*val)++;

  if((*val) >= max)
    (*val) = 0;
}

static void Incr(uint16_t *val, uint16_t max)
{
  if((*val) < max)
    (*val)++;
}


static void Decr(uint16_t *val)
{
  if((*val) != 0)
    (*val)--;
}

/******************************************************************************/

void RingBuffInit(RingBuff_t *buff, uint8_t *Mem, uint16_t Len)
{
  buff->Head = 0;
  buff->NumOfItems = 0;
  
  buff->Len = Len;
  
  buff->Buff = Mem;
}


void RingBuffPut(RingBuff_t *buff, uint8_t val)
{
  buff->Buff[buff->Head] = val;
  RingIncr(&buff->Head, buff->Len);
  Incr(&buff->NumOfItems, buff->Len);
}


int16_t RingBuffGet(RingBuff_t *buff)
{
  uint8_t ret;
  int16_t tail;

  if(buff->NumOfItems == 0)
    return -1;


  tail = buff->Head - buff->NumOfItems;

  if(tail < 0)
    tail = buff->Len + tail;

  ret = buff->Buff[tail];

  Decr(&buff->NumOfItems);

  return ret;
}

uint16_t RingBuffNumOfItems(RingBuff_t *buff)
{
  return buff->NumOfItems;
}

void RingBuffClear(RingBuff_t *buff)
{
  buff->NumOfItems = 0;
}

