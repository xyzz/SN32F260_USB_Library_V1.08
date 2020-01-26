/******************** (C) COPYRIGHT 2013 SONiX *******************************
* COMPANY: SONiX
* DATE:		 2019/11
* AUTHOR:	 SA1
* IC:			 SN32F26x
*____________________________________________________________________________
* REVISION	Date				User		Description
* 1.00			2016/07/21	Edan		1. First release
* 1.01      2016/08/01	Edan		1. Modify GPIO setting, SystemInit function(ILRC control flow)
* 1.02      2016/08/16	Edan		1. Modify USB IRQ_handler flow
* 1.03			2016/09/02	Edan		1. Reduce RAM size 
*																2. Add EMC protection mechanism (USBIRQ/Suspend)		
* 1.04			2016/09/19	Edan		1. Modify USB_EPnINFunction 
* 1.05			2017/05/17	Edan		1. Modify usbhw.c USBINT & add EFT protection mechanism  
* 1.06			2017/12/26  Edan		1. Add NotPinOut_GPIO_init function
* 1.07			2018/09/27  Edan		1. Modify usbhw.c 
* 1.08			2019/11/29  Tim			1. Modify usbhw.c 
*																2. Modify the SOF judging grammar in USB IRQ_handler flow
*																3. Modify the HID Descriptor for USB ISP
*______________________________________________________________
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS TIME TO MARKET.
* SONiX SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL 
* DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE
* AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN 
* IN CONNECTION WITH THEIR PRODUCTS.
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include	"SN32F260.h"
#include	"SN32F200_Def.h"
#include	"Utility/Utility.h"
#include	"Usb/usbram.h"
#include	"Usb/usbdesc.h"
#include	"Usb/usbhw.h"
#include	"Usb/usbuser.h"
#include	"Usb/usbepfunc.h"
#include	"UsbHid/hid.h"
#include	"UsbHid/hiduser.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/
#ifndef	SN32F268					//Do NOT Remove or Modify!!!
	#error Please install SONiX.SN32F2_DFP.1.2.9.pack or version >= 1.2.9
#endif
#define	PKG						SN32F268				//User SHALL modify the package on demand (SN32F268, SN32F267, SN32F265, SN32F2641, SN32F264, SN32F263) 


/*_____ M A C R O S ________________________________________________________*/


/*_____ F U N C T I O N S __________________________________________________*/
void SysTick_Init(void);
void NDT_Init(void);
void NotPinOut_GPIO_init(void);
	
/*****************************************************************************
* Function		: main
* Description	: USB HID demo code 
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/

int	_start (void)
{
	SystemInit();
	//1. User SHALL define PKG on demand.
	//2. User SHALL set the status of the GPIO which are NOT pin-out to input pull-up.	
#if 0
	NotPinOut_GPIO_init();		

	//Initial GPIO

	SN_GPIO0->MODE = 0xFFFF;
	SN_GPIO1->MODE = 0xFFFF;
	SN_GPIO2->MODE = 0xFFFF;
	SN_GPIO3->MODE = 0xFFFF;
	

	SN_GPIO0->DATA = 0xFFFF;	
	SN_GPIO1->DATA = 0xFFFF;		
	SN_GPIO2->DATA = 0xFFFF;
	SN_GPIO3->DATA = 0xFFFF;	
	
	
	SN_GPIO0->CFG = 0x00;  // Enable P0 internal pull-up resistor		
	SN_GPIO1->CFG = 0x00;  // Enable P1 internal pull-up resistor
	SN_GPIO2->CFG = 0x00;  // Enable P2 internal pull-up resistor	
	SN_GPIO3->CFG = 0x00;  // Enable P3 internal pull-up resistor	
#endif


	//****************USB Setting START******************//
	USB_Init();					/* USB Initialization */
	//****************USB Setting END**************//
	//** Setting For EFT Protect
	SysTick_Init();
	NDT_Init();
	//--------------------------------------------------------------------------
	//User Code starts HERE!!!
	while (1)
	{			
		if (sUSB_EumeData.wUSB_Status & mskBUSSUSPEND)	// Check BusSuspend
		{
			USB_Suspend();
		}
		#if (USB_LIBRARY_TYPE == USB_MOUSE_TYPE)
			USB_EPnINFunction(USB_EP1,&wUSB_MouseData,4);
		#else
			USB_EPnINFunction(USB_EP2,&wUSB_MouseData,5);
		#endif	
	}
	
}



/*****************************************************************************
* Function		 : SysTick_Init
* Description	 : For EFT Protection
* Input	(Global) : None
* Input	(Local)	 : None
* Output (Global): None
* Return (Local) : None
*****************************************************************************/
void	SysTick_Init (void)
{
	SysTick->LOAD = 0x000752FF;		//RELOAD = (system tick clock frequency ? 10 ms)/1000 -1
	SysTick->VAL = 0xFF; //__SYSTICK_CLEAR_COUNTER_AND_FLAG;
	SysTick->CTRL = 0x7;			//Enable SysTick timer and interrupt	
}
/*****************************************************************************
* Function		 : SysTick_Handler
* Description	 : For EFT Protection
* Input	(Global) : None
* Input	(Local)	 : None
* Output (Global): None
* Return (Local) : None
*****************************************************************************/
__irq void SysTick_Handler(void)
{
	if(bNDT_Flag)			//** Check NDT
	{
		if(SN_SYS0->NDTSTS_b.NDT5V_DET == 1)
		{
			dbNDT_Cnt = 0;
		}
		else
		{
		  dbNDT_Cnt++;
			if(dbNDT_Cnt == 90)//** 900ms
			{
				bNDT_Flag = 0;
			}
		}
	}	
}

/*****************************************************************************
* Function		 : NDT_Init
* Description	 : Noise Detect Init
* Input	(Global) : None
* Input	(Local)	 : None
* Output (Global): None
* Return (Local) : None
*****************************************************************************/
void NDT_Init(void)
{
	SN_SYS0->NDTCTRL = 0x2;					//** Enable NDT 5V
	NVIC_EnableIRQ(NDT_IRQn);
	SN_SYS0->ANTIEFT = 0x04;	
}
/*****************************************************************************
* Function		: NDT_IRQHandler
* Description	: ISR of NDT interrupt
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
__irq void NDT_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(NDT_IRQn);
	if (SN_SYS0->NDTSTS)				//** Check NDT 5V detected
	{
		bNDT_Flag = 1;
	}
	SN_SYS0->NDTSTS = 0x3;				//** Clear NDT 5V flag
}


/*****************************************************************************
* Function		: NotPinOut_GPIO_init
* Description	: Set the status of the GPIO which are NOT pin-out to input pull-up. 
* Input				: None
* Output			: None
* Return			: None
* Note				: 1. User SHALL define PKG on demand.
*****************************************************************************/
void	NotPinOut_GPIO_init(void)
{
#if (PKG == SN32F267)
	//set P2.9~P2.10 to input pull-up
	SN_GPIO2->CFG = 0xAAA82AAAA;
	
#elif (PKG == SN32F265)
	//set P0.6~P0.9 to input pull-up
	SN_GPIO0->CFG = 0xAAA00AAA;
	//set P2.0~P2.10 to input pull-up
	SN_GPIO2->CFG = 0x00000000;	
	//set P3.0 to input pull-up
	SN_GPIO3->CFG = 0xAAAAAAA8;
	
#elif (PKG == SN32F2641)
	//set P0.6~P0.9 to input pull-up
	SN_GPIO0->CFG = 0xAAA00AAA;
	//set P1.0~P1.1 to input pull-up
	SN_GPIO1->CFG = 0xAAAAAAA0;		
	//set P2.0~P2.10 to input pull-up
	SN_GPIO2->CFG = 0x00000000;	
	//set P3.0~P3.2 to input pull-up
	SN_GPIO3->CFG = 0xAAAAAA80;
	
#elif (PKG == SN32F264) 
	//set P0.6~P0.9 to input pull-up
	SN_GPIO0->CFG = 0xAAA00AAA;
	//set P2.0~P2.10 to input pull-up
	SN_GPIO2->CFG = 0x00000000;	
	//set P3.0~P3.4 to input pull-up
	SN_GPIO3->CFG = 0xAAAAA800;
	
#elif (PKG == SN32F263) 
	//set P0.6~P0.9, P0.14~P0.15 to input pull-up
	SN_GPIO0->CFG = 0x0AA00AAA;
	//set P1.0 to input pull-up
	SN_GPIO1->CFG = 0xAAAAAAA8;
	//set P2.0~P2.10 to input pull-up
	SN_GPIO2->CFG = 0x00000000;	
	//set P3.0~P3.4 to input pull-up
	SN_GPIO3->CFG = 0xAAAAA000;
	
#endif
}


/*****************************************************************************
* Function		: HardFault_Handler
* Description	: ISR of Hard fault interrupt
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
__irq void HardFault_Handler(void)
{
	NVIC_SystemReset();
}

__irq void P0_IRQHandler (void);

static void* vectors[0xC0/4] __attribute__((used, section (".vectors"))) = {
	[0] = (void*)0x20000800,
	[1] = _start,
	[15] = SysTick_Handler,
	[17] = USB_IRQHandler,
};
