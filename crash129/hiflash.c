/*
Copyright (c) 2022, Balazs Racz
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 - Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

 - Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#define FlashProgram __attribute__((section(".hiflash"))) HI_FlashProgram
#define FlashErase __attribute__((section(".hiflash"))) HI_FlashErase
#define FlashProtectGet __attribute__((section(".hiflash"))) HI_FlashProtectGet
#define FlashProtectSet __attribute__((section(".hiflash"))) HI_FlashProtectSet
#define FlashProtectSave __attribute__((section(".hiflash"))) HI_FlashProtectSave
#define FlashUserGet __attribute__((section(".hiflash"))) HI_FlashUserGet
#define FlashUserSet __attribute__((section(".hiflash"))) HI_FlashUserSet
#define FlashAllUserRegisterGet __attribute__((section(".hiflash"))) HI_FlashAllUserRegisterGet
#define FlashAllUserRegisterSet __attribute__((section(".hiflash"))) HI_FlashAllUserRegisterSet
#define FlashUserSave __attribute__((section(".hiflash"))) HI_FlashUserSave
#define FlashAllUserRegisterSave __attribute__((section(".hiflash"))) HI_FlashAllUserRegisterSave
#define FlashIntRegister __attribute__((section(".hiflash"))) HI_FlashIntRegister
#define FlashIntUnregister __attribute__((section(".hiflash"))) HI_FlashIntUnregister
#define FlashIntEnable __attribute__((section(".hiflash"))) HI_FlashIntEnable
#define FlashIntDisable __attribute__((section(".hiflash"))) HI_FlashIntDisable
#define FlashIntStatus __attribute__((section(".hiflash"))) HI_FlashIntStatus
#define FlashIntClear __attribute__((section(".hiflash"))) HI_FlashIntClear

#include "driverlib/flash.c"
