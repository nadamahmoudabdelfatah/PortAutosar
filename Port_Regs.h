 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Regs.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver Registers
 *
 * Author: Nada Mahmoud
 ******************************************************************************/
#ifndef PORT_REG_H
#define PORT_REG_H

#include "Std_Types.h"

/*****************************************************************************
GPIO registers base address
*****************************************************************************/
#define GPIO_PORTA_REG_BASE_ADDRESS       (0x40004000)
#define GPIO_PORTB_REG_BASE_ADDRESS       (0x40005000)
#define GPIO_PORTC_REG_BASE_ADDRESS       (0x40006000)
#define GPIO_PORTD_REG_BASE_ADDRESS       (0x40007000)
#define GPIO_PORTE_REG_BASE_ADDRESS       (0x40024000)
#define GPIO_PORTF_REG_BASE_ADDRESS       (0x40025000)

/*****************************************************************************
GPIO offsets
*****************************************************************************/
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C

/*****************************************************************************
ADC Registers                            
*****************************************************************************/
#define PORT_RCGCADC_REG                  (*((volatile unsigned long *)0x400FE638))
#define PORT_RCGCGPIO_REG                 (*((volatile unsigned long *)0x400FE608))                
#define PORT_RCGC0_REG                    (*((volatile unsigned long *)0x400FE100))
#define PORT_RCC_REG                      (*((volatile unsigned long *)0x400FE060))

#define SYSCTL_REGCGC2_REG               (*((volatile uint32 *)0x400FE108))

#endif