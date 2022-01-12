//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2013-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#define _POSIX_C_SOURCE 200112L
#define DEBUG

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/flash.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Blinky (blinky)</h1>
//!
//! A very simple example that blinks the on-board LED using direct register
//! access.
//
//*****************************************************************************


//*****************************************************************************
//
// The variable g_ui32SysClock contains the system clock frequency in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

void replicate_code(void)
{
  extern char _text, _ehitext;
  uint32_t ptr = (uint32_t)&_text;
  uint32_t end = (uint32_t)&_ehitext;
  end -= 512 * 1024;
  uint32_t addr = ptr + 512 * 1024;

  if (memcmp((void*)ptr, (void*)addr, end-ptr) == 0) {
    // Data is already copied.
    return;
  }

  while(ptr < end) {
      int ret = ROM_FlashErase(addr);
      ASSERT(ret == 0);
      uint32_t len = end - ptr;
      if (len > 16*1024) len = 16*1024;
      ret = ROM_FlashProgram((uint32_t*)ptr, addr, len);
      ASSERT(ret == 0);
      ptr += len;
      addr += len;
  }
}

typedef struct Scenario {
  // Scenario number. 0 = EOF.
  uint8_t num;
  // If true, we are going to run the timer during the scenario. Otherwise only
  // the flash ops.
  bool timer_enable;
  // If true, we will trigger nested interrupts during the scenario.
  bool int_nesting_enable;
  // 0 = ROM, 1 = low flash, 2 = high flash
  uint8_t flash_commands;
  // if true, we're writing to high flash, if false, we're writing to low flash
  bool write_to_high;
} Scenario;

const char* flash_commands_to_str[] = {
  "ROM", "Low", "High"
};

// How many flash operations we should do during one scenario.
static const unsigned NUM_FUZZ = 2000;

// Length of the flash sector to which we direct our fuzz operations.
static const unsigned MAX_WORDS = 16*1024 / 4;

// Length of the maximum write we perform (in words).
static const unsigned MAX_LEN = 64 / 4;

// Indirect call type for flash program.
typedef int32_t
(*FlashProgram_t)(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count);

// Indirect call type for flash erase.
typedef int32_t
(*FlashErase_t)(uint32_t ui32Address);

// ROM flash program function.
int32_t call_ROM_FlashProgram(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count) {
  return ROM_FlashProgram(pui32Data, ui32Address, ui32Count);
}

int32_t call_ROM_FlashErase(uint32_t ui32Address) {
  return ROM_FlashErase(ui32Address);
}

extern int32_t HI_FlashProgram(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count);
extern int32_t HI_FlashErase(uint32_t ui32Address);

uint32_t data_write[64/4];

void run_scenario(Scenario* s) {
  uint32_t flash_base = 2*64*1024;
  if (s->write_to_high) flash_base += 512*1024;
  FlashProgram_t program;
  FlashErase_t erase;
  switch(s->flash_commands) {
    case 0:
      program = &call_ROM_FlashProgram;
      erase = &call_ROM_FlashErase;
      break;
    case 1:
      program = &FlashProgram; 
      erase = &FlashErase;
      break;
    case 2:
      program = &HI_FlashProgram; 
      erase = &HI_FlashErase;
      break;
    default:
      while(1);
  }

  if (s->timer_enable) {
    MAP_IntEnable(INT_TIMER0A);
  }
  if (s->int_nesting_enable) {
    MAP_IntEnable(INT_TIMER0B);
  }
  
  
  (*erase)(flash_base);
  unsigned int seed = 42;
  for (unsigned cnt = 0; cnt < NUM_FUZZ; ++cnt) {
    unsigned ofs = rand_r(&seed) % MAX_WORDS;
    unsigned len = rand_r(&seed) % MAX_LEN;
    if (ofs + len > MAX_WORDS) {
      len = MAX_WORDS - ofs;
    }
    uint32_t addr = flash_base + ofs * 4;
    uint32_t bytes = len * 4;
    uint32_t* p = (uint32_t*)addr;
    for (unsigned i = 0 ; i < len; i++) {
      data_write[i] = p[i] & rand_r(&seed);
    }
    (*program)(data_write, addr, bytes);
  }

  MAP_IntDisable(INT_TIMER0A);
  MAP_IntDisable(INT_TIMER0B);
  MAP_IntDisable(INT_TIMER1A);
  MAP_IntDisable(INT_TIMER1B);

  
}

//*****************************************************************************
//
// Interrupts that will hit during flash operations.
//
//*****************************************************************************
static unsigned counter = 0;
static uint8_t display = 0;

void Timer0AInterrupt(void) {
  ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  if (++counter > 5000) {
    counter = 0;
    display ^= 0xff;
    ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, display);
  }
  ROM_IntPendSet(INT_TIMER0B);
}

void Timer0BInterrupt(void) {
  ROM_IntPendClear(INT_TIMER0B);
  ++counter;
}

void __attribute__((section(".hiflash"))) Timer1AInterrupt(void) {
  ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  if (++counter > 5000) {
    counter = 0;
    display ^= 0xff;
    ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, display);
  }
  ROM_IntPendSet(INT_TIMER1B);
}

void __attribute__((section(".hiflash"))) Timer1BInterrupt(void) {
  ROM_IntPendClear(INT_TIMER1B);
  ++counter;
}


Scenario runs[] = {
// num,int, nest, prog, write_high
  {5, true, true, 1, false},
  {1, false, false, 0, true},
  {2, true, true, 0, true},
  {3, true, true, 1, true},
  {4, true, true, 2, true},
  {42, false, false, 2, true},
  {41, true, false, 2, true},
  {5, true, true, 1, false},
  {6, true, true, 2, false},
  {0}
};


//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
int
main(void)
{
    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                         SYSCTL_OSC_MAIN |
                                         SYSCTL_USE_PLL |
                                         SYSCTL_CFG_VCO_240), 120000000);

    replicate_code();

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Hello!
    //
    UARTprintf("Hello, world!\n");
    

    volatile uint32_t ui32Loop;

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }

    //
    // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    // Sets up interrupt timers.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 20000);

    // This interrupt should hit even during kernel operations.
    MAP_IntPrioritySet(INT_TIMER0A, 0);
    MAP_IntPrioritySet(INT_TIMER0B, 0x20);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);

    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock / 20000);

    // This interrupt should hit even during kernel operations.
    MAP_IntPrioritySet(INT_TIMER1A, 0);
    MAP_IntPrioritySet(INT_TIMER1B, 0x20);
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER1_BASE, TIMER_A);
    
    MAP_IntDisable(INT_TIMER0A);
    MAP_IntDisable(INT_TIMER0B);
    MAP_IntDisable(INT_TIMER1A);
    MAP_IntDisable(INT_TIMER1B);
    
    
    
    for (unsigned i = 0; runs[i].num; ++i) {
      UARTprintf("scenario %u: timer=%d nesting=%d flash=%s write=%s\n",
                 runs[i].num, runs[i].timer_enable, runs[i].int_nesting_enable,
                 flash_commands_to_str[runs[i].flash_commands],
                 runs[i].write_to_high ? "high": "low");
      run_scenario(runs+i);
    }
    
    UARTprintf("All Done!\n");
    
    
    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Turn on the LED.
        //
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

        //
        // Delay for a bit.
        //
        for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        {
        }

        //
        // Turn off the LED.
        //
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x0);

        //
        // Delay for a bit.
        //
        for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        {
        }
    }
}
