/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbhw.c
 * Purpose: USB Custom User Module
 * Version: V1.01
 * Date:		2013/11
 *------------------------------------------------------------------------------*/
#include	<SN32F260.h>
#include	"../type.h"
#include	"../Utility/Utility.h"

#include	"usbram.h"
#include	"usbhw.h"
#include	"usbuser.h"
#include	"usbdesc.h"
#include	"../UsbHid/hiduser.h"

/*****************************************************************************
* Description	:Setting USB for different Power domain
*****************************************************************************/
#define System_Power_Supply 			System_Work_at_5_0V					// only 3.3V, 5V
	#define System_Work_at_3_3V				0
	#define	System_Work_at_5_0V				1

/*****************************************************************************
* Function		: USB_Init
* Description	: 1. setting IDLE_TIME, REPORT_PROTOCOL, S_USB_EP0setupdata.wUSB_Status	
*								2. set EP1~EP6 FIFO RAM address.
*								3. save	EP1~EP6 FIFO RAM point address.
*								4. save EP1~EP6 Package Size.
*								5. Enable USB function and setting EP1~EP6 Direction.
*								6. NEVER REMOVE !! USB D+/D- Dischage 
*								7. Enable USB PHY and USB interrupt.
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_Init	(void)
{
	volatile uint32_t	*pRam;
	uint32_t 	wTmp;
	USB_StandardVar_Init();
	USB_HidVar_Init();
	
	// Enable USBCLKEN (USB PCLK)
	SN_SYS1->AHBCLKEN |= mskUSBCLK_EN;		

	NVIC_ClearPendingIRQ(USB_IRQn);	
	NVIC_EnableIRQ(USB_IRQn);
		
	// Initialize USB EP1~EP4 RAM Start address base on 64-bytes. 
	SN_USB->EP1BUFOS = EP1_BUFFER_OFFSET_VALUE;
	SN_USB->EP2BUFOS = EP2_BUFFER_OFFSET_VALUE;
	SN_USB->EP3BUFOS = EP3_BUFFER_OFFSET_VALUE;
	SN_USB->EP4BUFOS = EP4_BUFFER_OFFSET_VALUE;

	// Copy EP1~EP4 RAM Start address to array(wUSB_EPnOffset).	
	pRam = &wUSB_EPnOffset[0];
	*(pRam+0) =  EP1_BUFFER_OFFSET_VALUE;
	*(pRam+1) =  EP2_BUFFER_OFFSET_VALUE;
	*(pRam+2) =  EP3_BUFFER_OFFSET_VALUE;
	*(pRam+3) =  EP4_BUFFER_OFFSET_VALUE;

	// Initialize EP0~EP4 package size to array(wUSB_EPnPacketsize).	
	pRam = &wUSB_EPnPacketsize[0];
	*(pRam+0) = USB_EP0_PACKET_SIZE;
	*(pRam+1) = USB_EP1_PACKET_SIZE;
	*(pRam+2) = USB_EP2_PACKET_SIZE;
	*(pRam+3) = USB_EP3_PACKET_SIZE;
	*(pRam+4) = USB_EP4_PACKET_SIZE;

	// Enable the USB Interrupt 
	SN_USB->INTEN = (mskBUS_IE|mskUSB_IE|mskEPnACK_EN|mskBUSWK_IE);
	// BUS_DRVEN = 0, BUS_DP = 1, BUS_DN = 0 
	SN_USB->SGCTL = mskBUS_J_STATE;
	
	#if (System_Power_Supply == System_Work_at_5_0V)
			//----------------------------------//	
			//Setting USB for System work at 5V	//
			//----------------------------------//
			//VREG33_EN = 1, PHY_EN = 1, DPPU_EN = 1, SIE_EN = 1, ESD_EN = 1 
			wTmp = (mskVREG33_EN|mskPHY_EN|mskDPPU_EN|mskSIE_EN|mskESD_EN);
	
	#elif (System_Power_Supply == System_Work_at_3_3V)
			//------------------------------------//	
			//Setting USB for System work at 3.3V	//
			//------------------------------------//
			//VREG33_EN = 0, PHY_EN = 1, DPPU_EN = 1, SIE_EN = 1, ESD_EN = 1 
			wTmp = (mskVREG33_DIS|mskPHY_EN|mskDPPU_EN|mskSIE_EN|mskESD_EN);
	#endif
	
	////////////////////////////////
	//	setting EP1~EP4 Direction	//
	////////////////////////////////
	#if (USB_EP1_DIRECTION == USB_DIRECTION_OUT) 
		wTmp |= mskEP1_DIR;
	#endif
	#if (USB_EP2_DIRECTION == USB_DIRECTION_OUT)
		wTmp |= mskEP2_DIR;
	#endif
	#if (USB_EP3_DIRECTION == USB_DIRECTION_OUT)
		wTmp |= mskEP3_DIR;
	#endif
	#if (USB_EP4_DIRECTION == USB_DIRECTION_OUT)
		wTmp |= mskEP4_DIR;
	#endif

	//Delay for the connection between Device and Host
	UT_MAIN_DelayNms(50);
	//Setting USB Configuration Register
 	SN_USB->CFG = wTmp;

	//Setting PHY Parameter Register1 &2
	SN_USB->PHYPRM2 = 0x00004004;
	SN_USB->PHYPRM = 0x80000000;
	
	return;
}

/*****************************************************************************
* Function		: P0_IRQHandler
* Description	: P0_IRQHandler
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
__irq void P0_IRQHandler (void)
{	
	#if 0
		NVIC_ClearPendingIRQ(P0_IRQn);
		// Clear P0.1 wakeup interrupt
		SN_GPIO0->IC = 0x00000002;	
		//Setting status about Remote wakeup action   
		sUSB_EumeData.wUSB_Status |= mskREMOTE_WAKEUP_ACT;
		NVIC_DisableIRQ(P0_IRQn);
	#endif
}

/*****************************************************************************
* Function		: Remote_Wakeup_Setting
* Description	: Setting Remote Wakeup IO
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void Remote_Wakeup_Setting (void)
{
	#if 0
		//set P0.1	eage falling trigger
		SN_GPIO0->IS = 0x00;
		SN_GPIO0->IEV = 0x00000002;	
		SN_GPIO0->IC =0xFFFFFFFF;
		//Enable P0.1 as an wakeup IO
		SN_GPIO0->IE = 0x00000002;	
		NVIC_ClearPendingIRQ(P0_IRQn);			
		NVIC_EnableIRQ(P0_IRQn);
	#endif
}


/*****************************************************************************
* Function		: USB_IRQHandler
* Description	: USB Interrupt USB_BUS, USB SOF, USB_IE
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void USB_IRQHandler (void)
{
	uint32_t iwIntFlag;

	// Get Interrupt Status and clear immediately.
	iwIntFlag = SN_USB->INSTS;		
	//Reserved ERR_STEUP Flag, PRE_SETUP status
	SN_USB->INSTSC = 0xFEFBFFFF;
	
	//@20160902 add for EMC protection	
	if(iwIntFlag==0)
	{
		USB_ReturntoNormal();
		return;		
	}	
	
	/////////////////////////////////////////////////	
	/* Device Status Interrupt (Resume) 					 */
	/////////////////////////////////////////////////	
	if(iwIntFlag & mskBUS_RESUME)
	{	
		/* Resume */						
		USB_ReturntoNormal();		
		USB_ResumeEvent();
		if (iwIntFlag &  mskEP0_PRESETUP)
		{
			UT_INT_DelayNx10us(2);
			USB_EP0SetupEvent();
			return;
		}			
	}
	/////////////////////////////////////////////////
	/* Device Status Interrupt (SOF) 							 */
	/////////////////////////////////////////////////	
	if ((iwIntFlag & mskUSB_SOF) && (SN_USB->INTEN & mskUSB_SOF_IE))
	{	
		/* SOF */								
		USB_SOFEvent();
	}
	/////////////////////////////////////////////////
	/* Device Status Interrupt (BusReset, Suspend) */
	/////////////////////////////////////////////////
	if (iwIntFlag & (mskBUS_RESET|mskBUS_SUSPEND))
	{
		if (iwIntFlag & mskBUS_RESET) 
		{									
			/* BusReset */
			USB_ReturntoNormal();					
			USB_ResetEvent();
		}
		else if (iwIntFlag & mskBUS_SUSPEND)			
		{									
			/* Suspend */	
			USB_SuspendEvent();			
		}
	}
	/////////////////////////////////////////////////	
	/* Device Status Interrupt (SETUP, IN, OUT) 	 */
	/////////////////////////////////////////////////	
	else if (iwIntFlag & (mskEP0_SETUP|mskEP0_IN|mskEP0_OUT|mskEP0_IN_STALL|mskEP0_OUT_STALL)) 
	{	
		if (iwIntFlag &  mskEP0_SETUP)
		{									
			/* SETUP */
			USB_EP0SetupEvent();
		}
		else if (iwIntFlag &  mskEP0_IN)
		{									
			/* IN */
			USB_EP0InEvent();
		}
		else if (iwIntFlag & mskEP0_OUT)
		{									
			/* OUT */
			USB_EP0OutEvent();
		}
		else if (iwIntFlag & (mskEP0_IN_STALL|mskEP0_OUT_STALL))
		{
			/* EP0_IN_OUT_STALL */			
			SN_USB->INSTSC = (mskEP0_IN_STALL|mskEP0_OUT_STALL);
			USB_EPnStall(USB_EP0);
		}
	}
	/////////////////////////////////////////////////	
	/* Device Status Interrupt (EPnACK) 					 */
	/////////////////////////////////////////////////		
	else if (iwIntFlag & (mskEP4_ACK|mskEP3_ACK|mskEP2_ACK|mskEP1_ACK))
	{
		if (iwIntFlag & mskEP1_ACK)
		{
			/* EP1 ACK */
			USB_EP1AckEvent();
		}
		if (iwIntFlag & mskEP2_ACK)
		{									
			/* EP2 ACK */
			USB_EP2AckEvent();
		}	
		if (iwIntFlag & mskEP3_ACK)
		{									
			/* EP3 ACK */
			USB_EP3AckEvent();
		}
		if (iwIntFlag & mskEP4_ACK)
		{									
			/* EP4 ACK */
			USB_EP4AckEvent();
		}	
	}
  return;
}


/*****************************************************************************
* Function		: USB_Suspend
* Description	: USB Suspend state SYS_CLK is runing sleep mode. 
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_Suspend (void)
{	
	uint32_t i;
	// Clear BusSuspend
	sUSB_EumeData.wUSB_Status &= ~mskBUSSUSPEND;	
	
	if(bNDT_Flag == 1)				// Check NDT status
	{
		return;
	}
	
	for(i=0; i < 620000; i++ )
	{	
		SN_WDT->FEED = 0x5AfA55AA;	
		if(!(SN_USB->INSTS & mskBUS_SUSPEND))				// double check Suspend flag
		{
			return;
		}
	}	

	// double check Suspend flag for EMC protect
	if(!(SN_USB->INSTS & mskBUS_SUSPEND))
	{		
		return;
	}
	
	// disable ESD_EN & PHY_EN
	__USB_PHY_DISABLE;													
	
	//If system support remote wakeup ,setting Remote wakeup GPIOs
	if (sUSB_EumeData.wUSB_Status & mskREMOTE_WAKEUP)
	{	
		Remote_Wakeup_Setting();
	}
	
	//Delay for waitting IO stable, then go into sleep mode
	UT_MAIN_DelayNx10us(10);		

	//System switch into Slow mode(ILRC)
	USB_SwitchtoSlow();

	// double check Suspend flag for EMC protect
	if(!(SN_USB->INSTS & mskBUS_SUSPEND))
	{		
		USB_ReturntoNormal();			
		return;
	}	
	//Into Sleep mode, for saving power consumption in suspend (<500uA)
	SN_PMU->CTRL = 0x04;
	__WFI();
	
	//system wakeup from sleep mode, system clock(ILRC) switch into IHRC
	USB_ReturntoNormal();		
	if ((sUSB_EumeData.wUSB_Status & (mskREMOTE_WAKEUP | mskREMOTE_WAKEUP_ACT)) == (mskREMOTE_WAKEUP | mskREMOTE_WAKEUP_ACT) )
	{
		sUSB_EumeData.wUSB_Status &= ~mskREMOTE_WAKEUP_ACT;			
		if (sUSB_EumeData.wUSB_SetConfiguration == USB_CONFIG_VALUE)
		{						
			USB_RemoteWakeUp();
			return;
		}
	}	
	
}

/*****************************************************************************
* Function		: USB_EP1AckEvent
* Description	: USB Clear EP1 ACK interrupt status
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EP1AckEvent (void)
{
	__USB_CLRINSTS(mskEP1_ACK);
}

/*****************************************************************************
* Function		: USB_EP2AckEvent
* Description	: USB Clear EP2 ACK interrupt status
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EP2AckEvent (void)
{
	__USB_CLRINSTS(mskEP2_ACK);
}

/*****************************************************************************
* Function		: USB_EP3AckEvent
* Description	: USB Clear EP3 ACK interrupt status
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EP3AckEvent (void)
{
	__USB_CLRINSTS(mskEP3_ACK);
}

/*****************************************************************************
* Function		: USB_EP4AckEvent
* Description	: USB Clear EP4 ACK interrupt status
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EP4AckEvent (void)
{
	__USB_CLRINSTS(mskEP4_ACK);
}


/*****************************************************************************
* Function		: USB_ClrEPnToggle
* Description	: USB Clear EP1~EP6 toggle bit to DATA0
								write 1: toggle bit Auto. write0:clear EPn toggle bit to DATA0
* Input				: hwEPNum ->EP1~EP6
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_ClrEPnToggle	(uint32_t	hwEPNum)
{
	SN_USB->EPTOGGLE &= ~(0x1<<hwEPNum);
}

/*****************************************************************************
* Function		: USB_EPnDisable
* Description	: Disable EP1~EP4
* Input				: wEPNum 
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnDisable (uint32_t	wEPNum)
{
	volatile uint32_t	*pEPn_ptr;
	if(wEPNum > USB_EP4)
		return;
	pEPn_ptr = &SN_USB->EP0CTL + wEPNum;
	*pEPn_ptr = 0;								//SET DISABLE. No handshake IN/OUT token.
}

/*****************************************************************************
* Function		: USB_EPnNak
* Description	: SET EP1~EP4 is NAK. For IN will handshake NAK to IN token.
*																		For OUT will handshake NAK to OUT token.
* Input				: wEPNum 
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnNak (uint32_t	wEPNum)
{
	volatile	uint32_t	*pEPn_ptr;
	if(wEPNum > USB_EP4)
		return;
	pEPn_ptr = &SN_USB->EP0CTL + wEPNum;
	*pEPn_ptr = mskEPn_ENDP_EN;			//SET NAK
}

/*****************************************************************************
* Function		: USB_EPnAck
* Description	: SET EP1~EP4 is ACK. For IN will handshake bBytent to IN token.
*																		For OUT will handshake ACK to OUT token.
* Input				: wEPNum:EP1~EP4.
*								bBytecnt: Byte Number of Handshake. 
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnAck (uint32_t	wEPNum, uint8_t	bBytecnt)
{	
	volatile	uint32_t	*pEPn_ptr;
	if (wEPNum > USB_EP4)
		return;
	pEPn_ptr = &SN_USB->EP0CTL + wEPNum;
	*pEPn_ptr = (mskEPn_ENDP_EN|mskEPn_ENDP_STATE_ACK|bBytecnt);
	
}

/*****************************************************************************
* Function		: USB_EPnAck
* Description	: SET EP1~EP4 is STALL. For IN will handshake STALL to IN token.
*																			For OUT will handshake STALL to OUT token.
* Input				: wEPNum:EP1~EP4.
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnStall (uint32_t	wEPNum)
{
	volatile uint32_t	*pEPn_ptr;
	if(wEPNum > USB_EP4)				//wEPNum != EP0~EP4
		return;
	pEPn_ptr = &SN_USB->EP0CTL + wEPNum;
	if (wEPNum == USB_EP0)
	{
			if(SN_USB->INSTS & mskEP0_PRESETUP)		
				return;
	}
	*pEPn_ptr = (mskEPn_ENDP_EN|mskEPn_ENDP_STATE_STALL);
}

/*****************************************************************************
* Function		: USB_RemoteWakeUp
* Description	: USB Remote wakeup: USB D+/D- siganl is J-K state. 
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_RemoteWakeUp()
{
	__USB_JSTATE_DRIVER;			// J state ;Full speed D+ = 1, D- = 0
	USB_DelayJstate();
	__USB_KSTATE_DRIVER;			// K state ;Full speed D+ = 0, D- = 1
	USB_DelayKstate();
	SN_USB->SGCTL &= ~mskBUS_DRVEN;
}

/*****************************************************************************
* Function		: USB_DelayJstate
* Description	: For J state delay. about 180us
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	USB_DelayJstate()
{
	uint32_t	i;
	//for (i=0; i<30; i++);	// delay 180us
	i=30;
	while(i--);
}

/*****************************************************************************
* Function		: USB_DelayKstate
* Description	: For K state delay. about 14 ~ 14.5ms 
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	USB_DelayKstate()
{
	#define	K_REMOTE_STATE_DELAY 20500 //12ms @ 12MHz
	uint32_t	i;
	
	for (i=0; i < K_REMOTE_STATE_DELAY; i++);	// require delay 1ms ~ 15ms
}



/*****************************************************************************
* Function		: fnUSBINT_ReadFIFO
* Description	: 
* Input				: FIFO_Address
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	fnUSBINT_ReadFIFO(uint32_t FIFO_Address)
{	
		SN_USB->RWADDR = FIFO_Address;
		SN_USB->RWSTATUS = 0x02;		
		while (SN_USB->RWSTATUS &0x02);
		wUSBINT_ReadDataBuf = SN_USB->RWDATA;
}

/*****************************************************************************
* Function		: fnUSBINT_WriteFIFO
* Description	: 
* Input				: FIFO_Address,  FIFO_WriteData
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	fnUSBINT_WriteFIFO(uint32_t FIFO_Address, uint32_t FIFO_WriteData)
{	
		SN_USB->RWADDR = FIFO_Address;
		SN_USB->RWDATA = FIFO_WriteData;
		SN_USB->RWSTATUS = 0x01;
		while (SN_USB->RWSTATUS &0x01);
}		
/*****************************************************************************
* Function		: fnUSBMAIN_ReadFIFO
* Description	: 
* Input				: FIFO_Address2
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/

void	fnUSBMAIN_ReadFIFO(uint32_t FIFO_Address2)
{	
		SN_USB->RWADDR2 = FIFO_Address2;
		SN_USB->RWSTATUS2 = 0x02;		
		while (SN_USB->RWSTATUS2 &0x02);
		wUSBMAIN_ReadDataBuf = SN_USB->RWDATA2;
}

/*****************************************************************************
* Function		: fnUSBMAIN_WriteFIFO
* Description	: 
* Input				: FIFO_Address2,  FIFO_WriteData2
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/

void	fnUSBMAIN_WriteFIFO(uint32_t FIFO_Address2, uint32_t FIFO_WriteData2)
{	
		SN_USB->RWADDR2 = FIFO_Address2;
		SN_USB->RWDATA2 = FIFO_WriteData2;
		SN_USB->RWSTATUS2 = 0x01;
 		while (SN_USB->RWSTATUS2 &0x01);
}		

/*****************************************************************************
* Function		: USB_ReturntoNormal
* Description	: Enable USB IHRC and switch system into IHRC 
*								Enable PHY/ESD protect
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_ReturntoNormal	(void)
{	
		SN_SYS0->ANBCTRL |= 0x00000001;							// Enable IHRC 	
		if(((SN_FLASH->LPCTRL)&0xF)!=0x05)
		{	
			SystemInit();		
		}
		USB_WakeupEvent();	
}

/*****************************************************************************
* Function		: USB_SwitchtoSlow
* Description	: System switch into ILRC, and wait for stable
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_SwitchtoSlow	(void)
{	
	// switch ILRC
	SN_SYS0->CLKCFG = 0x01;		
	// switch SYSCLK/1 	
	SN_SYS0->AHBCP = 0;													
	//Setting Flash mode <24MHz
	SN_FLASH->LPCTRL = 0x5AFA0000;
	// check ILRC status
	while((SN_SYS0->CLKCFG & 0x10) != 0x10);
	// disable IHRC
  SN_SYS0->ANBCTRL &= 0xFFFFFFE;							
}


