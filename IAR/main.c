#include <stdint.h>
#include "clock.h"
#include "uart.h"

static const UARTInitStructure_t UARTInitStr = 
{
  .bus_freq = 36000000,
  .baud = 19200,
  .data_bits = 8,
  .stop_bits = 1,
  .parity = 0,
};

void main()
{
  int16_t c;
  
  ClockInit();
  
  UART_Init(1, &UARTInitStr);
  
  UART_ReadBuffClear(1);
  UART_WriteBuffClear(1);
  
  for(;;)
  {
    c = UART_GetC(1);
    
    if(c != -1)
      UART_PutC(1, (char)c);
  }
}
