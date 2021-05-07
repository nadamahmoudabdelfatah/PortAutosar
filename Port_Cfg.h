 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Nada Mahmoud
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)

/* Pre-compile option for Pin Direction API */
#define PORT_SET_PIN_DIRECTION_API               (STD_ON)
       
 /* Pre-compile option for Pin Direction change API */
#define PORT_PIN_DIRECTION_CHANGEABLE           (STD_ON)

 /* Pre-compile option for Pin Direction change API */
#define PORT_PIN_MODE_CHANGEABLE             (STD_ON)


/* Number of the configured Port Channels */
#define PORT_CONFIGURED_PORTS             (6U)

/* Number of the configured Port Channels */
#define PORT_CONFIGURED_CHANNLES              (43U)
   
/* Number of the configured Port Channels */
#define PORT_CONFIG_MODE_NUM                 (3U)

/* Channel Index in the array of structures in Port_PBcfg.c */
#define PORTConf_LED1_CHANNEL_ID_INDEX        (uint8)0x00
#define PORTConf_SW1_CHANNEL_ID_INDEX         (uint8)0x01

/* Port Configured Channel ID's */
#define PORTConf_LED1_CHANNEL_NUM             (Port_PinType)1 /* Pin 1 in PORTF */
#define PORTConf_SW1_CHANNEL_NUM              (Port_PinType)4 /* Pin 4 in PORTF */

/* Port Configured Port ID's  */
#define PortConf_PortA                (Port_PortType)0 /* PORTA */
#define PortConf_PortB                (Port_PortType)1 /* PORTB */
#define PortConf_PortC                (Port_PortType)2 /* PORTC */
#define PortConf_PortD                (Port_PortType)3 /* PORTD */
#define PortConf_PortE                (Port_PortType)4 /* PORTE */
#define PortConf_PortF                (Port_PortType)5 /* PORTF */

/* Port Configured Pin Index's  */

#define PA0                (Port_PinType)0 /* Pinindex0 */
#define PA1                (Port_PinType)1 /* Pinindex1 */
#define PA2                (Port_PinType)2 /* Pinindex2 */
#define PA3                (Port_PinType)3 /* Pinindex3 */
#define PA4                (Port_PinType)4 /* Pinindex4 */
#define PA5                (Port_PinType)5 /* Pinindex5 */
#define PA6                (Port_PinType)6 /* Pinindex6 */   
#define PA7                (Port_PinType)7 /* Pinindex7 */
#define PB0                (Port_PinType)8 /* Pinindex8 */
#define PB1                (Port_PinType)9 /* Pinindex9 */
#define PB2                (Port_PinType)10 /* Pinindex10 */
#define PB3                (Port_PinType)11 /* Pinindex11 */
#define PB4                (Port_PinType)12 /* Pinindex12 */
#define PB5                (Port_PinType)13 /* Pinindex13 */
#define PB6                (Port_PinType)14 /* Pinindex14 */   
#define PB7                (Port_PinType)15 /* Pinindex15 */
#define PC0                (Port_PinType)16 /* Pinindex16 */
#define PC1                (Port_PinType)17 /* Pinindex17 */
#define PC2                (Port_PinType)18 /* Pinindex18 */
#define PC3                (Port_PinType)19 /* Pinindex19 */
#define PC4                (Port_PinType)20 /* Pinindex20 */
#define PC5                (Port_PinType)21 /* Pinindex21 */
#define PC6                (Port_PinType)22 /* Pinindex22 */   
#define PC7                (Port_PinType)23 /* Pinindex23 */
#define PD0                (Port_PinType)24 /* Pinindex24 */
#define PD1                (Port_PinType)25 /* Pinindex25 */
#define PD2                (Port_PinType)26 /* Pinindex26 */
#define PD3                (Port_PinType)27 /* Pinindex27 */
#define PD4                (Port_PinType)28 /* Pinindex28 */
#define PD5                (Port_PinType)29 /* Pinindex29 */
#define PD6                (Port_PinType)30 /* Pinindex30 */
#define PD7                (Port_PinType)31 /* Pinindex31 */
#define PE0                (Port_PinType)32 /* Pinindex32 */
#define PE1                (Port_PinType)33 /* Pinindex33 */
#define PE2                (Port_PinType)34 /* Pinindex34 */
#define PE3                (Port_PinType)35 /* Pinindex35 */
#define PE4                (Port_PinType)36 /* Pinindex36 */
#define PE5                (Port_PinType)37 /* Pinindex37 */
#define PF0                (Port_PinType)38 /* Pinindex38 */ 
#define PF1                (Port_PinType)39 /* Pinindex39 */
#define PF2                (Port_PinType)40 /* Pinindex40 */
#define PF3                (Port_PinType)41 /* Pinindex41 */
#define PF4                (Port_PinType)42 /* Pinindex42 */    

/* Port Configured Channel Modes */
#define DIO_PMCx_BITS             (uint32)0x0000000F /* DIO_PMCx_BITS */
#define PWM_PMCx_BITS             (uint32)5          /* DIO_PMCx_BITS */   
#define PWM_CLOCK_ENABLE          (uint32)0x00100000 /*Enable PWM clock*/ 
#define ADC_MODULE0               (uint8)0
#define ADC_MODULE1               (uint8)1



#endif /* Port_CFG_H */
