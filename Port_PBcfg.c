
 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_PBcfg.c
 *
 * Description: Post Build Configuration Source file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Nada Mahmoud
 ******************************************************************************/
#include "Port.h"
#include "Port_Cfg.h"

/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PBcfg.c does not match the expected version"
#endif
/* PB structure used with Port_Init API */
const Port_ConfigType Port_Configuration = {
                                            PortConf_PortA , PA0, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortA , PA1, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortA , PA2, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortA , PA3, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortA , PA4, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortA , PA5, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortA , PA6, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortA , PA7, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            
                                            PortConf_PortB , PB0, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortB , PB1, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortB , PB2, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortB , PB3, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortB , PB4, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortB , PB5, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortB , PB6, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortB , PB7, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            
                                            PortConf_PortC , PC0, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortC , PC1, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortC , PC2, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortC , PC3, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortC , PC4, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortC , PC5, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortC , PC6, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortC , PC7, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            
                                            PortConf_PortD , PD0, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortD , PD1, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortD , PD2, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortD , PD3, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortD , PD4, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortD , PD5, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortD , PD6, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortD , PD7, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            
                                            PortConf_PortE , PE0, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortE , PE1, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortE , PE2, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortE , PE3, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortE , PE4, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortE , PE5, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            
                                            PortConf_PortF , PF0, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW , PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortF , PF1, PORT_PIN_OUT, PULL_UP, PORT_PIN_LEVEL_LOW, PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortF , PF2, PORT_PIN_OUT, PULL_UP, PORT_PIN_LEVEL_LOW, PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortF , PF3, PORT_PIN_OUT, PULL_UP, PORT_PIN_LEVEL_LOW, PORT_PIN_MODE_DIO , STD_ON, STD_ON,
                                            PortConf_PortF , PF4, PORT_PIN_IN, PULL_UP, PORT_PIN_LEVEL_LOW,  PORT_PIN_MODE_DIO , STD_ON, STD_ON

				         };
