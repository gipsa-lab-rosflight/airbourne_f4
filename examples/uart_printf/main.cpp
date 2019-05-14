/*
 * Copyright (c) 2017, James Jackson
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifdef TARGET_REVO
#pragma message("TARGET_REVO")
#include "revo_f4.h"
#define UART_DEV UART3
#endif

#ifdef TARGET_AIRBOT
#pragma message("TARGET_AIRBOT")
#include "airbot_f4.h"
#define UART_DEV UART6
#endif

#include "uart.h"
#include "led.h"
#include "vcp.h"
#include "printf.h"

VCP *vcpPtr = NULL;

UART *uartPtr = NULL;

void rx_callback(uint8_t byte)
{
  uartPtr->put_byte(byte);

	printf("0x%02X\n", (int)byte);
}

static void _putc(void *p, char c)
{
  (void)p; // avoid compiler warning about unused variable
  vcpPtr->put_byte(c);
}

int main()
{
  systemInit();

	VCP vcp;
  vcp.init();
  vcpPtr = &vcp;
	init_printf(NULL, _putc);
	
  UART uart;
  uart.init(&uart_config[UART_DEV], 115200);
  uartPtr = &uart;
	
  uart.register_rx_callback(rx_callback);  // Uncomment to test callback version
	
  LED info;
  info.init(LED2_GPIO, LED2_PIN);
	delay(500);
  info.on();
	
  int i = 0;
  while (1)
  {
		printf("testing\n");
    uint8_t hello_string[9] = "testing\n";
    uart.write(hello_string, 8); // Uncomment to test Tx
    delay(1000);

    // Polling version (uncomment to test)
//    while (uart.rx_bytes_waiting())
//    {
//      uint8_t byte = uart.read_byte();
//      uartPtr->put_byte(byte);
//    }

  }
}
