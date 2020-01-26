/****************************************************************************
 ****************************************************************************
****************************************************************************/
#ifndef __USBHW_H__
#define __USBHW_H__



	/* AHB Clock Enable register <SYS1_AHBCLKEN> */
	#define	mskP0CLK_EN					(0x1<<0)
	#define	mskP1CLK_EN					(0x1<<1)
	#define	mskP2CLK_EN					(0x1<<2)
	#define	mskP3CLK_EN					(0x1<<3)
	#define	mskUSBCLK_EN				(0x1<<4)
	#define	mskCT16B0CLK_EN			(0x1<<6)	
	#define	mskCT16B1CLK_EN			(0x1<<7)		
	#define	mskSPI0CLK_EN				(0x1<<12)	
	#define	mskI2C0CLK_EN				(0x1<<21)
	#define	mskWDTCLK_EN				(0x1<<24)	
	
	/* USB Interrupt Enable Bit Definitions <USB_INTEN> */
	#define mskEP1_NAK_EN				(0x1<<0)
	#define mskEP2_NAK_EN				(0x1<<1)
	#define	mskEP3_NAK_EN				(0x1<<2)
	#define mskEP4_NAK_EN				(0x1<<3)
	#define mskEPnACK_EN				(0x1<<4)
	
	#define mskBUSWK_IE					(0x1<<28)	
	#define mskUSB_IE						(0x1<<29)
	#define mskUSB_SOF_IE				(0x1<<30)
	#define	mskBUS_IE						(0x1U<<31)

	/* USB Interrupt Event Status Bit Definitions <USB_INSTS/USB_INSTSC> */
	#define mskEP1_NAK					(0x1<<0)
	#define mskEP2_NAK					(0x1<<1)
	#define mskEP3_NAK					(0x1<<2)
	#define mskEP4_NAK					(0x1<<3)

	#define mskEP1_ACK					(0x1<<8)
	#define mskEP2_ACK					(0x1<<9)
	#define mskEP3_ACK					(0x1<<10)
	#define mskEP4_ACK					(0x1<<11)

	#define mskERR_TIMEOUT			(0x1<<17)
	#define mskERR_SETUP				(0x1<<18)
	#define mskEP0_OUT_STALL		(0x1<<19)
	#define mskEP0_IN_STALL			(0x1<<20)
	#define	mskEP0_OUT					(0x1<<21)
	#define mskEP0_IN						(0x1<<22)
	#define mskEP0_SETUP				(0x1<<23)
	#define mskEP0_PRESETUP			(0x1<<24)
	#define mskBUS_WAKEUP				(0x1<<25)
	#define	mskUSB_SOF					(0x1<<26)
	#define mskBUS_RESUME				(0x1<<29)
	#define mskBUS_SUSPEND			(0x1<<30)
	#define	mskBUS_RESET				(0x1U<<31)

	/* USB Device Address Bit Definitions <USB_ADDR> */
	#define mskUADDR						(0x7F<<0)

	/* USB Configuration Bit Definitions <USB_CFG> */
	#define mskEP1_DIR					(0x1<<0)
	#define mskEP2_DIR					(0x1<<1)
	#define mskEP3_DIR					(0x1<<2)
	#define mskEP4_DIR					(0x1<<3)

	#define mskDIS_PDEN					(0x1<<26)
	#define mskESD_EN						(0x1<<27)
	#define	mskSIE_EN						(0x1<<28)
	#define mskDPPU_EN				 	(0x1<<29)
	#define mskPHY_EN						(0x1<<30)
	#define	mskVREG33_EN				(0x1U<<31)
	#define mskVREG33_DIS				(0x0U<<31)
	
	/* USB Signal Control Bit Definitions <USB_SGCTL> */
	#define mskBUS_DRVEN					(0x1<<2)
	#define mskBUS_DPDN_STATE			(0x3<<0)
	#define mskBUS_J_STATE				(0x2<<0)			// D+ = 1, D- = 0
	#define mskBUS_K_STATE				(0x1<<0)			// D+ = 0, D- = 1
	#define mskBUS_SE0_STATE			(0x0<<0)			// D+ = 0, D- = 0
	#define mskBUS_SE1_STATE			(0x3<<0)			// D+ = 1, D- = 1
	#define mskBUS_IDLE_STATE			mskBUS_J_STATE

	/* USB Configuration Bit Definitions <USB_EPnCTL> */
	#define mskEPn_CNT						(0x1FF<<0)
	#define mskEP0_OUT_STALL_EN		(0x1<<27)
	#define mskEP0_IN_STALL_EN		(0x1<<28)
	#define mskEPn_ENDP_STATE			(0x3<<29)
	#define mskEPn_ENDP_STATE_ACK	(0x1<<29)
	#define mskEPn_ENDP_STATE_NAK	(0x0<<29)
	#define mskEPn_ENDP_STATE_STALL	(0x3<<29)
	#define mskEPn_ENDP_EN				(0x1U<<31)

	/* USB Endpoint Data Toggle Bit Definitions <USB_EPTOGGLE> */
	#define mskEP1_CLEAR_DATA0		(0x1<<0)
	#define mskEP2_CLEAR_DATA0		(0x1<<1)
	#define mskEP3_CLEAR_DATA0		(0x1<<2)
	#define mskEP4_CLEAR_DATA0		(0x1<<3)

	/* USB Endpoint n Buffer Offset Bit Definitions <USB_EPnBUFOS> */
	#define mskEPn_OFFSET					(0x1FF<<0)

	/* USB Frame Number Bit Definitions <USB_FRMNO> */
	#define mskFRAME_NO						(0x7FF<<0)

	/* Rx & Tx Packet Length Definitions */
	#define PKT_LNGTH_MASK				0x000003FF

	/* nUsb_Status Register Definitions */
	#define	mskBUSRESET							(0x1<<0)
	#define	mskBUSSUSPEND						(0x1<<1)
	#define	mskBUSRESUME						(0x1<<2)
	#define	mskREMOTEWAKEUP					(0x1<<3)
	#define	mskSETCONFIGURATION0CMD	(0x1<<4)
	#define	mskSETADDRESS						(0x1<<5)
	#define	mskSETADDRESSCMD				(0x1<<6)
	#define	mskREMOTE_WAKEUP				(0x1<<7)
	#define	mskDEV_FEATURE_CMD			(0x1<<8)
	#define	mskSET_REPORT_FLAG			(0x1<<9)
	#define	mskPROTOCOL_GET_REPORT	(0x1<<10)
	#define	mskPROTOCOL_SET_IDLE		(0x1<<11)
	#define	mskPROTOCOL_ARRIVAL			(0x1<<12)
	#define	mskSET_REPORT_DONE			(0x1<<13)
	#define	mskNOT_8BYTE_ENDDING		(0x1<<14)
	#define	mskSETUP_OUT						(0x1<<15)
	#define	mskSETUP_IN							(0x1<<16)
	#define	mskINITREPEAT						(0x1<<17)
	#define	mskREMOTE_WAKEUP_ACT		(0x1<<18)
	
	//ISP KERNEL MODE 
	#define	RETURN_KERNEL_0 0x5AA555AA
	#define	RETURN_KERNEL_1 0xCC3300FF
/*********Marco function***************/

	//USB device address set
	#define __USB_SETADDRESS(addr)  (SN_USB->ADDR = addr)			
	//USB INT status register clear
	#define	__USB_CLRINSTS(Clrflag)	(SN_USB->INSTSC = Clrflag)	
	//USB EP0_IN token set STALL
	#define	__USB_EP0INSTALL_EN		(SN_USB->EP0CTL |= mskEP0_IN_STALL_EN)		
	//USB EP0_OUT token set STALL
	#define	__USB_EP0OUTSTALL_EN		(SN_USB->EP0CTL |= mskEP0_OUT_STALL_EN)			
	//USB bus driver J state
	#define	__USB_JSTATE_DRIVER		(SN_USB->SGCTL = (mskBUS_DRVEN|mskBUS_J_STATE))	
	//USB bus driver K state
	#define	__USB_KSTATE_DRIVER		(SN_USB->SGCTL = (mskBUS_DRVEN|mskBUS_K_STATE))	
	//USB PHY set enable
	#define	__USB_PHY_ENABLE			(SN_USB->CFG |= (mskESD_EN|mskPHY_EN))				
	//USB PHY set Disable
	#define	__USB_PHY_DISABLE			(SN_USB->CFG &= ~(mskESD_EN|mskPHY_EN))				

/***************************************/

/* USB IRQ Functions*/
extern	void	USB_IRQHandler(void);
extern	void	USB_SOFEvent(void);
extern	void	USB_ResetEvent(void);
extern	void	USB_SuspendEvent(void);
extern	void	USB_ResumeEvent(void);
extern	void	USB_WakeupEvent(void);
extern	void	USB_EP0SetupEvent(void);
extern	void	USB_EP0InEvent(void);
extern	void	USB_EP0OutEvent(void);
extern	void	USB_EP1AckEvent(void);
extern	void	USB_EP2AckEvent(void);
extern	void	USB_EP3AckEvent(void);
extern	void	USB_EP4AckEvent(void);


/* USB Hardware Functions */
extern	void	USB_Init(void);
//extern	void	USB_ClrInsts(uint32_t	wClrflag);
extern	void 	USB_EPnDisable(uint32_t	wEPNum);
extern	void 	USB_EPnNak(uint32_t	wEPNum);
extern	void	USB_EPnAck(uint32_t	wEPNum, uint8_t	bBytecnt);
extern	void	USB_EPnStall(uint32_t	wEPNum);

extern	void	USB_Suspend(void);
extern	void	USB_RemoteWakeUp(void);
extern	void	USB_DelayJstate(void);
extern	void	USB_DelayKstate(void);
extern	void	USB_ClrEPnToggle(uint32_t	wEPNum);
extern	void 	USB_ReturntoNormal	(void);
extern	void 	USB_SwitchtoSlow	(void);

extern	uint32_t	USB_EPnReadByteData(uint32_t	wEPNum, uint32_t	wByteIndex);
extern	uint32_t	USB_EPnReadWordData(uint32_t	wEPNum, uint32_t	wWordIndex);
extern	void	USB_EPnWriteByteData(uint32_t	wEPNum, uint32_t	wByteIndex, uint32_t	wBytedata);
extern	void	USB_EPnWriteWordData(uint32_t	wEPNum, uint32_t	wWordIndex, uint32_t	wWorddata);

extern	void	fnUSBINT_ReadFIFO(uint32_t FIFO_Address);
extern	void	fnUSBINT_WriteFIFO(uint32_t FIFO_Address, uint32_t FIFO_WriteData );
extern	void	fnUSBMAIN_ReadFIFO(uint32_t FIFO_Address2);
extern	void	fnUSBMAIN_WriteFIFO(uint32_t FIFO_Address2, uint32_t FIFO_WriteData2 );
#endif  /* __USBHW_H__ */
