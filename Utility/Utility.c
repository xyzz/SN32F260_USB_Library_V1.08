/******************** (C) COPYRIGHT 2013 SONiX *******************************
* COMPANY:			SONiX
* DATE:					2013/12
* AUTHOR:				SA1
* IC:						SN32F240/230/220
* DESCRIPTION:	Utility related functions.
*____________________________________________________________________________
* REVISION	Date				User		Description
* 1.0				2013/12/17	SA1			1. First release
*
*____________________________________________________________________________
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS TIME TO MARKET.
* SONiX SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL 
* DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE
* AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN 
* IN CONNECTION WITH THEIR PRODUCTS.
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "Utility.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/
#define CLK_PRESCALAR 	Fcpu_Div1
	#define Fcpu_Div1		0x0
	#define Fcpu_Div2 	0x1
	#define Fcpu_Div4 	0x2
	#define Fcpu_Div8 	0x3

//Define for Fcpu/8 , Fcpu/4 , Fcpu/2 , Fcpu/1
#define	UTILITY_10US_DELAY		80>>(CLK_PRESCALAR)	

/*_____ M A C R O S ________________________________________________________*/


/*_____ F U N C T I O N S __________________________________________________*/
/*****************************************************************************
* Function		: UT_INT_DelayNx10us
* Description	: Delay N x 10 us @ Fcpu = 48~6MHz
* Input			: N
* Output		: None
* Return		: None
* Note			: May have some deviation (+/-20%)
*							This Function work in INTERRPUT 
*****************************************************************************/
void UT_INT_DelayNx10us (uint32_t N)
{
	N*=UTILITY_10US_DELAY;			//10us @ Fcpu = 48~6MHz
  while (N--);
}


/*****************************************************************************
* Function		: UT_INT_DelayNms
* Description	: Delay N ms @ Fcpu = Fcpu = 48~6MHz
* Input			: N
* Output		: None
* Return		: None
* Note			: May have some deviation (+/-20%)
*							This Function work in INTERRPUT 
*****************************************************************************/
void UT_INT_DelayNms (uint32_t N)
{
	while (N--)
	{
		SN_WDT->FEED = 0x5AFA55AA;
		UT_INT_DelayNx10us(100);	//10us x 100 = 1ms
	}
}

/*****************************************************************************
* Function		: UT_Main_DelayNx10us
* Description	: Delay N x 10 us @ Fcpu = Fcpu = 48~6MHz
* Input			: N
* Output		: None
* Return		: None
* Note			: May have some deviation (+/-20%)
*							This Function work in Main Loop 
*****************************************************************************/
void UT_MAIN_DelayNx10us (uint32_t N)
{
	N*=UTILITY_10US_DELAY;			//10us @ Fcpu = 48~6MHz
  while (N--);
}


/*****************************************************************************
* Function		: UT_Main_DelayNms
* Description	: Delay N ms @ Fcpu = 12MHz
* Input			: N
* Output		: None
* Return		: None
* Note			: May have some deviation (+/-20%)
*							This Function work in Main Loop 
*****************************************************************************/
void UT_MAIN_DelayNms (uint32_t N)
{
	while (N--)
	{
		SN_WDT->FEED = 0x5AFA55AA;
		UT_MAIN_DelayNx10us(100);	//10us x 100 = 1ms
	}
}


