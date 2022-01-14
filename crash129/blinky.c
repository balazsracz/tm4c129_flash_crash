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
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
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

// Vector table copy in RAM.
uint32_t __attribute__((aligned(1024))) ram_vectors[256];

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
  // Copy interrupt vectors first
  extern uint32_t g_pfnVectors[];
  memcpy(ram_vectors, g_pfnVectors, sizeof(ram_vectors));
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
  unsigned num;
  // If true, we are going to run the timer during the scenario. Otherwise only
  // the flash ops.
  bool timer_enable;
  // If true, we will trigger nested interrupts during the scenario.
  bool int_nesting_enable;
  // 0 = ROM, 1 = low flash, 2 = high flash
  uint8_t flash_commands;
  // if true, we're writing to high flash, if false, we're writing to low flash
  bool write_to_high;
  // 0: low-flash interrupt, 1: high-flash interrupts
  uint8_t int_high;
  // 0: low-flash vector table, 1: high-flash vector table, 2: sram vector table
  uint8_t vect_select;
} Scenario;

const char* flash_commands_to_str[] = {
  "ROM", "Low", "High"
};
const char* vect_select_to_str[] = {
  "Low", "High", "RAM"
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
      UARTprintf("error in flash_command config\n");
      while(1);
  }

  switch(s->int_high) {
    case 0:
      if (s->timer_enable) {
        MAP_IntEnable(INT_TIMER0A);
      }
      if (s->int_nesting_enable) {
        MAP_IntEnable(INT_TIMER0B);
      }
      break;
    case 1:
      if (s->timer_enable) {
        MAP_IntEnable(INT_TIMER1A);
      }
      if (s->int_nesting_enable) {
        MAP_IntEnable(INT_TIMER1B);
      }
      break;
    default:
      UARTprintf("error in int_high config\n");
      while(1);
  }

  switch(s->vect_select) {
    case 0:
      HWREG(NVIC_VTABLE) = 0;
      break;
    case 1:
      HWREG(NVIC_VTABLE) = 512*1024;
      break;
    case 2:
      HWREG(NVIC_VTABLE) = (uint32_t)&(ram_vectors[0]);
      break;
    default:
      UARTprintf("error in vect_select config\n");
      while(1);
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


void print_scenario(Scenario* s) {
      UARTprintf("scenario %u: timer=%d nesting=%d flash=%s write=%s int=%s vect=%s ",
                 s->num, s->timer_enable, s->int_nesting_enable,
                 flash_commands_to_str[s->flash_commands],
                 s->write_to_high ? "high": "low",
                 s->int_high ? "high": "low",
                 vect_select_to_str[s->vect_select]);
}

/// Generates a scenario by going through all combinations seuquentially.
/// @param num where are we in the sequence number
/// @param s output argument: the generated scenario
/// @return true if successful, false if num is too high.
bool generate_scenario(unsigned num, Scenario* s)
{
  memset(s, 0, sizeof(*s));
  s->num = num;

#if 0  
  // Check timer
  switch(num%3) {
    case 0:
      s->timer_enable = 0;
      s->int_nesting_enable = 0;
      break;
    case 1:
      s->timer_enable = 1;
      s->int_nesting_enable = 0;
      break;
    case 2:
      s->timer_enable = 1;
      s->int_nesting_enable = 1;
      break;
  }
  num /= 3;
#else
  s->timer_enable = 1;
  s->int_nesting_enable = 1;
#endif  

#if 1  
  // Split on flash commands
  s->flash_commands = (num % 3);
  num /= 3;
#else
  s->flash_commands = 0; // ROM
#endif 

#if 1  
  // Split on Writing to high flash or low flash
  s->write_to_high = (num & 1) > 0;
  num /= 2;
#else
  s->write_to_high = 1; // always write to high flash
#endif
  
#if 1  
  // Split on int high or low
  s->int_high = (num & 1);
  num /= 2;
#else
  s->int_high = 0; // always low interrupt
#endif

#if 1  
  // Split on vector table select
  s->vect_select = (num % 3);
  num /= 3;
#else
  s->vect_select = 0; // default low vector table
#endif 
  
  // If we have bits left, then the scenario number is too high.
  return !(num > 0);
}

extern int blacklist[];

/// Checks if a scenario is in the blacklist.
/// @param num number of scenario
/// @return true if this is blacklisted.
bool is_blacklisted(unsigned num) {
  unsigned idx;  
  for (idx = 0; blacklist[idx] >=0 && blacklist[idx]!=(int)num; idx++)
  {
  }
  return blacklist[idx]==(int)num;
}

//*****************************************************************************
//
// Interrupts that will hit during flash operations.
//
//*****************************************************************************
static unsigned counter = 0;
static uint8_t display = 0;
static unsigned totalcount = 0;

void __attribute__((aligned(32),section(".ahead.0a"))) Timer0AInterrupt(void) {
  ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  if (++counter > 2000) {
    counter = 0;
    display ^= 0xff;
    ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, display);
  }
  ++totalcount;
  ROM_IntPendSet(INT_TIMER0B);
  __asm__ volatile("nop\n");
}

void __attribute__((aligned(32),section(".ahead.0b"))) Timer0BInterrupt(void) {
  ROM_IntPendClear(INT_TIMER0B);
  ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xff);
  for (int i = 1500; i; i--) {
    __asm__ volatile(" " : : "r"(i));
  }
  ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
  ++counter;
  __asm__ volatile("nop\n");
  __asm__ volatile("nop\n nop\n");
  //__asm__ volatile("nop\n nop\n");
  __asm__ volatile("nop\n nop\n");
}

void __attribute__((aligned(32),section(".hiflash"))) Timer1AInterrupt(void) {
  ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  ++totalcount;
  if (++counter > 2000) {
    counter = 0;
    display ^= 0xff;
    ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, display);
  }
  ROM_IntPendSet(INT_TIMER1B);
}

void __attribute__((aligned(32),section(".hiflash"))) Timer1BInterrupt(void) {
  ROM_IntPendClear(INT_TIMER1B);
  ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xff);
  for (int i = 1500; i; i--) {
    __asm__ volatile(" " : : "r"(i));
  }
  ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
  ++counter;
  __asm__ volatile("nop\n");
  __asm__ volatile("nop\n nop\n");
  //__asm__ volatile("nop\n nop\n");
  __asm__ volatile("nop\n nop\n");
}


Scenario runs[] = {
// num,int, nest, prog, write_high, int_high
  {1, false, false, 0, true},
  {2, false, false, 0, true, 1},
  {51, true, true, 0, false, 1, 2}, // ?

  {231, true, true, 0, true, 1, 2}, // ?
  {241, true, true, 0, false, 1, 2}, // ?
  {431, true, true, 1, false, 1, 2}, // ?
  {441, true, true, 2, false, 1, 2}, // ?
  {331, true, true, 1, true, 1, 2}, // ?
  {341, true, true, 2, true, 1, 2}, // ?
  
  {211, true, true, 0, true, 1}, // works
  {221, true, true, 0, false, 1}, // works
  {411, true, true, 1, false, 1}, // works
  {421, true, true, 2, false, 1}, // works
  {311, true, true, 1, true, 1}, // works
  {321, true, true, 2, true, 1}, // works

  {22, true, true, 0, false}, // works
  {41, true, true, 1, false}, // works
  {42, true, true, 2, false}, // works
  {31, true, true, 1, true}, // works
  {32, true, true, 2, true}, // works
  {21, true, true, 0, true}, // crashes
  {2, true, false, 0, true},
  {3, true, true, 1, true},
  {4, true, true, 2, true},
  {5, true, true, 1, false},
  {42, false, false, 2, true},
  {41, true, false, 2, true},
  {5, true, true, 1, false},
  {6, true, true, 2, false},
  {0}
};

// Which generated scenarios should we skip because they crash. terminated with
// -1.
int blacklist[] = {
  3,
  27,
  -1
};

unsigned button_nowait = 0;

void hw_preinit(void)
{
    // If enabled, waits for a button press to start. THis is helpful to attach
    // a debugger.
#if 1
    do
    {
      if (GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0) {
        button_nowait = 1;
      }
    } while (!button_nowait);
#endif
    //
    // Delay for a bit.
    //
    for(unsigned ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
    {
    }
}

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
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);

    //
    // Enable the GPIO input for the USR_SW1 button.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    MAP_GPIODirModeSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA,
                         GPIO_PIN_TYPE_STD_WPU);
    
    // Sets up interrupt timers.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 10000);

    // This interrupt should hit even during kernel operations.
    MAP_IntPrioritySet(INT_TIMER0A, 0);
    MAP_IntPrioritySet(INT_TIMER0B, 0x20);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);

    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock / 10000);

    // This interrupt should hit even during kernel operations.
    MAP_IntPrioritySet(INT_TIMER1A, 0);
    MAP_IntPrioritySet(INT_TIMER1B, 0x20);
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER1_BASE, TIMER_A);
    
    MAP_IntDisable(INT_TIMER0A);
    MAP_IntDisable(INT_TIMER0B);
    MAP_IntDisable(INT_TIMER1A);
    MAP_IntDisable(INT_TIMER1B);
    
    hw_preinit();
    
    if (false) {
      // Run static scenarios
      for (unsigned i = 0; runs[i].num; ++i) {
        totalcount = 0;
        print_scenario(runs+i);
        run_scenario(runs+i);
        UARTprintf("count=%u\n", totalcount);
      }
    } else {
      // Generate scenarios
      unsigned num = 0;
      Scenario s;
      while (generate_scenario(num, &s)) {
        totalcount = 0;
        print_scenario(&s);
        if (is_blacklisted(num)) {
          UARTprintf("blacklisted!\n", totalcount);
        } else {
          run_scenario(&s);
          UARTprintf("count=%u\n", totalcount);
        }
        num++;
      }
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
