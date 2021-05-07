 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Nada Mahmoud
 ******************************************************************************/
//revise errors
//revised errors in init, set pin, set direction
#include "Port.h"
#include "Port_Regs.h"
#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC const Port_ConfigType * Port_Configptr = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;
volatile uint32 * PortGpio_Ptr = NULL_PTR;
static const Port_ConfigChannel * Port_PortChannels = NULL_PTR;

/************************************************************************************
*Local fuctions declarations
************************************************************************************/
static void Port_Setpin(Port_PinType Pin, Port_PinType *Port_PinNum);
static void Port_Setport(Port_PinType Pin);
static void Port_Initdirection(Port_PinType Pin, Port_PinType Port_PinNum, const Port_ConfigType* ConfigPtr);
static void Port_Setmode(Port_PinType Pin, Port_PinType Port_PinNum, Port_PinModeType Mode);
/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initialize the Port module.
************************************************************************************/

//add mode

void Port_Init(const Port_ConfigType* ConfigPtr)
{

	Port_PinType Port_Pinindex=0;
	volatile uint32 delay = 0;
	Port_PinType Port_PinNum;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (NULL_PTR == ConfigPtr)
	{
                Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
				PORT_E_INIT_FAILED);
	}
	else
#endif
		//revise this end
	{
		Port_Configptr = ConfigPtr;
                Port_PortChannels = ConfigPtr->Channels;
		/*
		 * Set the module state to initialized and point to the PB configuration structure using a global pointer.
		 * This global pointer is global to be used by other functions to read the PB configuration structures
		 */
		Port_Status       = PORT_INITIALIZED;
		//    Port_PortChannels = ConfigPtr->Channels; /* address of the first Channels structure --> Channels[0] */
		for(Port_Pinindex=0;Port_Pinindex<PORT_CONFIGURED_CHANNLES;Port_Pinindex++)
		{               
			//revise this switch
			Port_Setport(Port_PortChannels[Port_Pinindex].pin_num);

			/* Enable clock for PORT and allow time for clock to start*/
			SYSCTL_REGCGC2_REG |= (1<<Port_PortChannels[Port_Pinindex].port_num);
			delay = SYSCTL_REGCGC2_REG;
			//revise

				//
			        Port_Setpin(Port_PortChannels[Port_Pinindex].pin_num, &Port_PinNum);
				//
				if( ((Port_PortChannels[Port_Pinindex].port_num == 3) && (Port_PinNum == 7)) || ((Port_PortChannels[Port_Pinindex].port_num == 5) && (Port_PinNum == 0)) ) /* PD7 or PF0 */
				{
					*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
					SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PinNum);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
				}
				else if( (Port_PortChannels[Port_Pinindex].port_num == 2) && (Port_PinNum <=3) ) /* PC0 to PC3 */
				{
					//REVISE 17:00:00 NO-58
					/* Do Nothing ...  this is the JTAG pins */
					continue;
				}
				else
				{
					/* Do Nothing ... No need to unlock the commit register for this pin */
				}
				Port_Setmode(Port_PortChannels[Port_Pinindex].pin_num, Port_PinNum, Port_PortChannels[Port_Pinindex].mode);
				//mode removed from here

				Port_Initdirection(Port_PortChannels[Port_Pinindex].pin_num, Port_PinNum, ConfigPtr);

			
		}

	}
}

/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Port Pin ID number , Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Sets the port pin direction.
************************************************************************************/

#if (PORT_SET_PIN_DIRECTION_API==STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
{
	Port_PinType Port_PinNum;
#if (PORT_DEV_ERROR_DETECT == STD_ON)

	if(Port_PortChannels[Pin].PortPinDirectionChangeable == STD_OFF)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
				PORT_E_MODE_UNCHANGEABLE);
	}
	if(Pin >= PORT_CONFIGURED_CHANNLES)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
				PORT_E_PARAM_PIN);
	}
	else
#endif
        {
		 Port_Setpin(Pin, &Port_PinNum);
	         Port_Setport(Pin);
		if(Direction == PORT_PIN_OUT)
		{
			//revise port gpio ptr
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinNum);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
		}
		else if(Direction == PORT_PIN_IN)
		{
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinNum);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

		} 
	}
}
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Sets the port pin direction.
************************************************************************************/
void Port_RefreshPortDirection( void )
{
	Port_PinType Port_Pinindex=0;
	Port_PinType Port_PinNum;
	for(Port_Pinindex=0;Port_Pinindex<PORT_CONFIGURED_CHANNLES;Port_Pinindex++)
	{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
          if(Port_PortChannels[Port_Pinindex].PortPinDirectionChangeable == STD_OFF)
		continue;

                
#endif  
		Port_Setpin(Port_PortChannels[Port_Pinindex].pin_num, &Port_PinNum);
		Port_Setport(Port_Pinindex);
		if(Port_PortChannels[Port_Pinindex].direction == PORT_PIN_OUT)
		{
			//check null pointer
			//revise port gpio ptr
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinNum);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
		}
		else if(Port_PortChannels[Port_Pinindex].direction == PORT_PIN_IN)
		{
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinNum);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

		}
	}

}

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): Pointer to where to store the version information of this module
* Return value: None
* Description: Function to Sets the port pin direction.
************************************************************************************/

#if (PORT_VERSION_INFO_API == STD_ON)
void Dio_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
	}
	else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}
}
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Port Pin ID number, New Port Pin mode to be set on port pin
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Sets the port pin direction.
************************************************************************************/

void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)    /* Report Errors */
	if (PORT_CONFIGURED_CHANNLES <= Pin)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
	}
	else
	{
		/* No Action Required */
	}
	if (PORT_CONFIG_MODE_NUM <= Mode)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
	}

	/* Report error if the pin mode is not changeable */
	if (Port_PortChannels[Pin].PortPinDirectionChangeable == STD_OFF)         
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
	}  
	else
#endif     

	{ 
          	Port_PinType Port_PinNum;
	        Port_Setpin(Pin, &Port_PinNum);
		Port_Setmode(Pin,Port_PinNum, Mode);
	}
}

/************************************************************************************
* Service Name: Port_Setpin
* Parameters (in): Pin number
* Parameters (inout): None
* Parameters (out): Pointer to pin index in the port
* Return value: None
* Description: Function to Setup the pin index in port for bit shifting
************************************************************************************/
static void Port_Setpin(Port_PinType Pin, Port_PinType* Port_PinNum)
{
	
	if(Pin == PA0 || Pin == PB0 || Pin == PC0 || Pin == PD0 || Pin == PE0 || Pin == PF0)
	{
		*Port_PinNum = 0;
	}
	else if(Pin == PA1 || Pin == PB1 || Pin == PC1 || Pin == PD1 || Pin == PE1 || Pin == PF1)
	{
		*Port_PinNum = 1;
	}
	else if(Pin == PA2 || Pin == PB2 || Pin == PC2 || Pin == PD2 || Pin == PE2 || Pin == PF2)
	{
		*Port_PinNum = 2;
	}
	else if(Pin == PA3 || Pin == PB3 || Pin == PC3 || Pin == PD3 || Pin == PE3 || Pin == PF3)
	{
		*Port_PinNum = 3;
	}
	else if(Pin == PA4 || Pin == PB4 || Pin == PC4 || Pin == PD4 || Pin == PE4 || Pin == PF4)
	{
		*Port_PinNum = 4;
	}
	else if(Pin == PA5 || Pin == PB5 || Pin == PC5 || Pin == PD5 || Pin == PE5)
	{
		*Port_PinNum = 5;
	}
	else if(Pin == PA6 || Pin == PB6 || Pin == PC6 || Pin == PD6)
	{
		*Port_PinNum = 6;
	}
	else if(Pin == PA7 || Pin == PB7 || Pin == PC7 || Pin == PD7)
	{
		*Port_PinNum = 7;
	}

}
/************************************************************************************
* Service Name: Port_Setport
* Parameters (in): Pin number
* Parameters (inout): None
* Parameters (out): Pointer to port
* Return value: None
* Description: Function to Setup the port for bit shifting
************************************************************************************/
static void Port_Setport(Port_PinType Pin)
{
	if(Pin == PA0 || Pin == PA1 || Pin == PA2 || Pin == PA3 || Pin == PA4 || Pin == PA5 || Pin == PA6 || Pin == PA7)
	{
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_REG_BASE_ADDRESS ;
	}
	else if(Pin == PB0 || Pin == PB1 || Pin == PB2 || Pin == PB3 || Pin == PB4 || Pin == PB5 || Pin == PB6 || Pin == PB7)
	{
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_REG_BASE_ADDRESS ;
	}
	else if(Pin == PC0 || Pin == PC1 || Pin == PC2 || Pin == PC3 || Pin == PC4 || Pin == PC5 || Pin == PC6 || Pin == PC7)
	{
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_REG_BASE_ADDRESS ;
	}
	else if(Pin == PD0 || Pin == PD1 || Pin == PD2 || Pin == PD3 || Pin == PD4 || Pin == PD5 || Pin == PD6 || Pin == PD7)
	{
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_REG_BASE_ADDRESS ;
	}
	else if(Pin == PE0 || Pin == PE1 || Pin == PE2 || Pin == PE3 || Pin == PE4 || Pin == PE5)
	{
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_REG_BASE_ADDRESS ;
	}
	else if(Pin == PF0 || Pin == PF1 || Pin == PF2 || Pin == PF3 || Pin == PF4)
	{
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_REG_BASE_ADDRESS ;
	}
}
/************************************************************************************
* Service Name: Port_Setmode
* Parameters (in): Pin number - Pin index - Mode
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the mode of a pin
************************************************************************************/

static void Port_Setmode(Port_PinType Pin, Port_PinType Port_PinNum, Port_PinModeType Mode)
{
	Port_Setport(Pin);
	switch (Mode)  
	{
	case DIO:
		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinNum);      /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */  
		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PinNum);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinNum);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(DIO_PMCx_BITS << (Port_PinNum * 4));     /* Clear the PMCx bits for this pin */
		break;
	case ADC:
		if(Pin != PE5 && Pin != PE4 && Pin != PE3 && Pin != PE2 && Pin != PE1 && Pin != PE0 && Pin != PD3 && Pin != PD2 && Pin != PD1 && Pin != PD0 && Pin != PB5 && Pin != PB4)
		{
			return;
		}
		else
		{  
			SET_BIT(PORT_RCGCADC_REG  , ADC_MODULE0);  
			SET_BIT(PORT_RCGCADC_REG  , ADC_MODULE1); 
			//revise clock
			//revise this too
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinNum);  
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinNum);      /* Clear the corresponding bit in the GPIODEN register to enable digital functionality on this pin */  
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PinNum);      /* Set the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
		}
		break;
	case PWM:
		if(Pin != PA6 && Pin != PA7 && Pin != PB4 && Pin != PB5 && Pin != PB6 && Pin != PD0 && Pin != PD1 && Pin != PD2 && Pin != PD6 && Pin != PC4 && Pin != PC5 && Pin != PE4 && Pin != PE5 && Pin != PF0 && Pin != PF1 && Pin != PF2 && Pin != PF3 && Pin != PF4)
		{
			return;
		}
		else
		{
			PORT_RCGC0_REG = PORT_RCGC0_REG & PWM_CLOCK_ENABLE;
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinNum);
			//revise
			*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) =  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) & (PWM_PMCx_BITS << (Port_PinNum * 4));
			//use pwm divide or not ?? revise
		}
	}
}
/************************************************************************************
* Service Name: Port_Initdirection
* Parameters (in): Pin number - Pin index - Pointer to configuration
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the direction of the pin - (input or output)
************************************************************************************/
static void Port_Initdirection(Port_PinType Pin, Port_PinType Port_PinNum, const Port_ConfigType* ConfigPtr)
{

	if(Port_PortChannels[Pin].direction == PORT_PIN_OUT)
	{
		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinNum);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

		if(Port_PortChannels[Pin].initial_value == STD_HIGH)
		{
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PinNum);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
		}
		else
		{
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PinNum);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
		}
	}
	else if(Port_PortChannels[Pin].direction == PORT_PIN_IN)
	{
		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinNum);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

		if(Port_PortChannels[Pin].resistor == PULL_UP)
		{
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PinNum);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
		}
		else if(Port_PortChannels[Pin].resistor == PULL_DOWN)
		{
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PinNum);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
		}
		else
		{
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PinNum);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PinNum);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
		}
	}
	else
	{
		/* Do Nothing */
	}
}
