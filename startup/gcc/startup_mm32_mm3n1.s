/**************************************************************************//**
 * @file     startup_mm32_mm3n1.s
 * @brief    CMSIS-Core(M) Device Startup File for MM32F103 / MM32L3xx
 * @version  V2.0.0
 * @date     20. May 2019
 ******************************************************************************/
/*
 * Copyright (c) 2009-2019 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

.syntax  unified
.cpu cortex-m3

.section .vectors
.align   2
.globl   __Vectors
.globl   __Vectors_End
.globl   __Vectors_Size

__Vectors:
    .long   __StackTop;
    .long	Reset_Handler;                         ///< Reset Handler
    .long	NMI_Handler;                           ///< NMI Handler
    .long	HardFault_Handler;                     ///< Hard Fault Handler
    .long	MemManage_Handler;                     ///< MPU Fault Handler
    .long	BusFault_Handler;                      ///< Bus Fault Handler
    .long	UsageFault_Handler;                    ///< Usage Fault Handler
    .long	0;                     ///< Reserved
    .long	0;                     ///< Reserved
    .long	0;                     ///< Reserved
    .long	0;                     ///< Reserved
    .long	SVC_Handler;                           ///< SVCall Handler
    .long	DebugMon_Handler;                      ///< Debug Monitor Handler
    .long	0;                     ///< Reserved
    .long	PendSV_Handler;                        ///< PendSV Handler
    .long	SysTick_Handler;                       ///< SysTick Handler

// Peripheral handlers
    .long	WWDG_IRQHandler;                       ///<   0 Window Watchdog
    .long	PVD_IRQHandler ;                       ///< PVD through EXTI Line detect
    .long	TAMPER_IRQHandler;                     ///< Tamper
    .long	RTC_IRQHandler;                        ///< RTC
    .long	FLASH_IRQHandler;                      ///< Flash
    .long	RCC_IRQHandler;                        ///< RCC
    .long	EXTI0_IRQHandler;                      ///< EXTI Line 0
    .long	EXTI1_IRQHandler;                      ///< EXTI Line 1
    .long	EXTI2_IRQHandler;                      ///< EXTI Line 2
    .long	EXTI3_IRQHandler;                      ///< EXTI Line 3
    .long	EXTI4_IRQHandler;                      ///<  10 EXTI Line 4
    .long	DMA1_Channel1_IRQHandler;              ///< DMA1 Channel 1
    .long	DMA1_Channel2_IRQHandler;              ///< DMA1 Channel 2
    .long	DMA1_Channel3_IRQHandler;              ///< DMA1 Channel 3
    .long	DMA1_Channel4_IRQHandler;              ///< DMA1 Channel 4
    .long	DMA1_Channel5_IRQHandler;              ///< DMA1 Channel 5
    .long	DMA1_Channel6_IRQHandler;              ///< DMA1 Channel 6
    .long	DMA1_Channel7_IRQHandler;              ///< DMA1 Channel 7
    .long	ADC1_2_IRQHandler;                     ///< ADC1 & ADC2
    .long	USB_HP_CAN1_TX_IRQHandler;             ///< USB High Priority or CAN1 TX
    .long	0;					   ///<  20	
    .long	CAN1_RX1_IRQHandler;                   ///< CAN1 RX1
    .long	0;
    .long	EXTI9_5_IRQHandler;                    ///< EXTI Line 9..5
    .long	TIM1_BRK_IRQHandler;                   ///< TIM1 Break
    .long	TIM1_UP_IRQHandler;                    ///< TIM1 Update
    .long	TIM1_TRG_COM_IRQHandler;               ///< TIM1 Trigger and Commutation
    .long	TIM1_CC_IRQHandler;                    ///< TIM1 Capture Compare
    .long	TIM2_IRQHandler;                       ///< TIM2
    .long	TIM3_IRQHandler;                       ///< TIM3
    .long	TIM4_IRQHandler;                       ///< 30 TIM4
    .long	I2C1_EV_IRQHandler;                    ///< I2C1 Event
    .long	0;
    .long	I2C2_EV_IRQHandler;                    ///< I2C2 Event
    .long	0;
    .long	SPI1_IRQHandler;                       ///< SPI1
    .long	SPI2_IRQHandler;                       ///< SPI2
    .long	UART1_IRQHandler;                      ///< UART1
    .long	UART2_IRQHandler;                      ///< UART2
    .long	UART3_IRQHandler;                      ///< UART3
    .long	EXTI15_10_IRQHandler;                  ///< 40 EXTI Line 15..10
    .long	RTCAlarm_IRQHandler;                   ///< 41 RTC Alarm through EXTI Line
    .long	USBWakeUp_IRQHandler;                  ///< 42 USB Wakeup from suspend
    .long	Reserved10_Handler;                    ///< 43
    .long	Reserved11_Handler;                    ///< 44
    .long	AES_IRQHandler;                        ///< 45

//  .space   ( (224-46) * 4)                          /* Interrupts 46 .. 224 are left out */__Vectors_End:
                .equ     __Vectors_Size, __Vectors_End - __Vectors
                .size    __Vectors, . - __Vectors


                .thumb
                .section .text
                .align   2

                .thumb_func
                .type    Reset_Handler, %function
                .globl   Reset_Handler
                .fnstart
Reset_Handler:
                bl       SystemInit

                ldr      r4, =__copy_table_start__
                ldr      r5, =__copy_table_end__

.L_loop0:
                cmp      r4, r5
                bge      .L_loop0_done
                ldr      r1, [r4]
                ldr      r2, [r4, #4]
                ldr      r3, [r4, #8]

.L_loop0_0:
                subs     r3, #4
                ittt     ge
                ldrge    r0, [r1, r3]
                strge    r0, [r2, r3]
                bge      .L_loop0_0

                adds     r4, #12
                b        .L_loop0
.L_loop0_done:

                ldr      r3, =__zero_table_start__
                ldr      r4, =__zero_table_end__

.L_loop2:
                cmp      r3, r4
                bge      .L_loop2_done
                ldr      r1, [r3]
                ldr      r2, [r3, #4]
                movs     r0, 0

.L_loop2_0:
                subs     r2, #4
                itt      ge
                strge    r0, [r1, r2]
                bge      .L_loop2_0

                adds     r3, #8
                b        .L_loop2
.L_loop2_done:

                bl       _start

                .fnend
                .size    Reset_Handler, . - Reset_Handler


                .thumb_func
                .type    Default_Handler, %function
                .weak    Default_Handler
                .fnstart
Default_Handler:
                b        .
                .fnend
                .size    Default_Handler, . - Default_Handler

/* Macro to define default exception/interrupt handlers.
 * Default handler are weak symbols with an endless loop.
 * They can be overwritten by real handlers.
 */
                .macro   Set_Default_Handler  Handler_Name
                .weak    \Handler_Name
                .set     \Handler_Name, Default_Handler
                .endm



/* Default exception/interrupt handler */

	Set_Default_Handler  NMI_Handler
	Set_Default_Handler  HardFault_Handler
	Set_Default_Handler  MemManage_Handler
	Set_Default_Handler  BusFault_Handler
	Set_Default_Handler  UsageFault_Handler
	Set_Default_Handler  SVC_Handler
	Set_Default_Handler  DebugMon_Handler
	Set_Default_Handler  PendSV_Handler
	Set_Default_Handler  SysTick_Handler

    Set_Default_Handler  	WWDG_IRQHandler
    Set_Default_Handler  	PVD_IRQHandler 
    Set_Default_Handler  	TAMPER_IRQHandler
    Set_Default_Handler  	RTC_IRQHandler
    Set_Default_Handler  	FLASH_IRQHandler
    Set_Default_Handler  	RCC_IRQHandler
    Set_Default_Handler  	EXTI0_IRQHandler
    Set_Default_Handler  	EXTI1_IRQHandler
    Set_Default_Handler  	EXTI2_IRQHandler
    Set_Default_Handler  	EXTI3_IRQHandler
    Set_Default_Handler  	EXTI4_IRQHandler
    Set_Default_Handler  	DMA1_Channel1_IRQHandler
    Set_Default_Handler  	DMA1_Channel2_IRQHandler
    Set_Default_Handler  	DMA1_Channel3_IRQHandler
    Set_Default_Handler  	DMA1_Channel4_IRQHandler
    Set_Default_Handler  	DMA1_Channel5_IRQHandler
    Set_Default_Handler  	DMA1_Channel6_IRQHandler
    Set_Default_Handler  	DMA1_Channel7_IRQHandler
    Set_Default_Handler  	ADC1_2_IRQHandler
    Set_Default_Handler  	USB_HP_CAN1_TX_IRQHandler
    Set_Default_Handler  	CAN1_RX1_IRQHandler
    Set_Default_Handler  	EXTI9_5_IRQHandler
    Set_Default_Handler  	TIM1_BRK_IRQHandler
    Set_Default_Handler  	TIM1_UP_IRQHandler
    Set_Default_Handler  	TIM1_TRG_COM_IRQHandler
    Set_Default_Handler  	TIM1_CC_IRQHandler
    Set_Default_Handler  	TIM2_IRQHandler
    Set_Default_Handler  	TIM3_IRQHandler
    Set_Default_Handler  	TIM4_IRQHandler
    Set_Default_Handler  	I2C1_EV_IRQHandler
    Set_Default_Handler  	I2C2_EV_IRQHandler
    Set_Default_Handler  	SPI1_IRQHandler
    Set_Default_Handler  	SPI2_IRQHandler
    Set_Default_Handler  	UART1_IRQHandler
    Set_Default_Handler  	UART2_IRQHandler
    Set_Default_Handler  	UART3_IRQHandler
    Set_Default_Handler  	EXTI15_10_IRQHandler
    Set_Default_Handler  	RTCAlarm_IRQHandler
    Set_Default_Handler  	USBWakeUp_IRQHandler
    Set_Default_Handler  	AES_IRQHandler


.end








