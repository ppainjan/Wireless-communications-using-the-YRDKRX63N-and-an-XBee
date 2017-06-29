/******************************************************************************
* File Name     : main.c
* Version       : 1.0
* Device(s)     : RX63N
* Tool-Chain    : Renesas RX Standard Toolchain 1.0.0
* OS            : None
* H/W Platform  : YRDKRX63N
* Description   : Empty application project
*                 This application provides a handy framework to serve as a
*                 generic basis for any application. The MCU startup procedures
*                 are provided, as well as some basic means for I/O. The LCD is 
*                 initialized, board switches and their interrupt ISR stubs are
*                 defined, and a simple loop blinks an LED to indicate that the
*                 board is operating.
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 22.09.2011     1.00        First release              
*******************************************************************************/

/*******************************************************************************
Includes   <System Includes> , "Project Includes"
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <machine.h>
#include "platform.h"
#include "r_switches.h"
#include "uart.h"
void uart_init();
void transmit(unsigned char *);
void transmit_data(unsigned char);
void delay(void);
/*******************************************************************************
* Prototypes for local functions
*******************************************************************************/
char data[19]={0x7E,0x00,0x0F,0x10,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x00,0x20,0xD1};
char new_char;
int count = 0;
char rcvd_data[28];
char disp_data = 0x20;
char disp[28];
char rcvd_char;
/******************************************************************************
* Function name: main
* Description  : Main program function
* Arguments    : none
* Return value : none
******************************************************************************/
void main(void)
{
    /* Used to pace toggling of the LED */
    uint32_t led_counter;
    
    /* Initialize LCD */
    lcd_initialize();
    
    /* Clear LCD */
    lcd_clear();
    
    /* Display message on LCD */
    lcd_display(LCD_LINE1, "  LAB3   ");
	//uart_init();
    /* 
        STDOUT is routed through the virtual console window tunneled through the JTAG debugger.
        Open the console window in 20HEW to see the output 
    */
    printf("This is the debug console\r\n");
    uart_init();
    /* The three pushbuttons on the YRDK board are tied to interrupt lines, set them up here */
    R_SWITCHES_Init();
    
	sprintf(disp, "%c", disp_data);
	lcd_display(LCD_LINE2, disp);
	delay();
	delay();
	delay();
	delay();
    /* This is the main loop.  It does nothing but toggle LED4 periodically */
    while (1)
    {
		if(SW1 == 0)
		{
			delay();
			if(SW1 == 0)
			{
				if (disp_data == 0x7E)
				{
					disp_data = 0x20;
					data[17] = disp_data;
					data[18] = 0xD1;
				}	
				else
				{	
					disp_data = disp_data + 0x01;
					data[17] = disp_data;
					data[18] = data[18] - 0x01;
				}
				lcd_display(LCD_LINE1,"Transmitted -");
				sprintf(disp, "%c", disp_data);
				lcd_display(LCD_LINE2, disp);
			}
		}
		delay();
		delay();
		if(SW2 == 0)
		{
			delay();
			if(SW2 == 0)
			{
				if (disp_data == 0x20)
				{
					disp_data = 0x7E;
					data[17] = disp_data;
					data[18] = 0x73;
				}
				else	
				{
					disp_data = disp_data - 0x01 ;
					data[17] = disp_data;
					data[18] = data[18] + 0x01;
				}
			}
			lcd_display(LCD_LINE1,"Transmitted -");
			sprintf(disp, "%c", disp_data);
			lcd_display(LCD_LINE2, disp);
		}
		delay();
		delay();
		if(SW3 == 0)
		{
			int i;
			//for (i=0;i<19;i++)
			//{
				
			//	new_char = data[i];
				transmit(data);
		//	}
			rcvd_char = rcvd_data[26];
			//sprintf(disp, "%c", rcvd_char);
			//lcd_display(LCD_LINE4, disp);
		}
		lcd_display(LCD_LINE3,"Received -");
		sprintf(disp, "%c", rcvd_char);
			lcd_display(LCD_LINE4, disp);
		delay();
		delay();
    }
} /* End of function main() */

/******************************************************************************
* Function name: sw1_callback
* Description  : Callback function that is executed when SW1 is pressed.
*                Called by sw1_isr in r_switches.c
* Arguments    : none
* Return value : none
******************************************************************************/
void sw1_callback(void)
{
    nop(); /* Add your code here. Note: this is executing inside an ISR. */
} /* End of function sw1_callback() */


/******************************************************************************
* Function name: sw2_callback
* Description  : Callback function that is executed when SW2 is pressed.
*                Called by sw2_isr in r_switches.c
* Arguments    : none
* Return value : none
******************************************************************************/
void sw2_callback(void)
{
    nop(); /* Add your code here. Note: this is executing inside an ISR. */
} /* End of function sw2_callback() */


/******************************************************************************
* Function name: sw3_callback
* Description  : Callback function that is executed when SW3 is pressed.
*                Called by sw3_isr in r_switches.c
* Arguments    : none
* Return value : none
******************************************************************************/
void sw3_callback(void)
{
    nop(); /* Add your code here. Note: this is executing inside an ISR. */
} /* End of function sw3_callback() */

void uart_init()
{
	 #ifdef PLATFORM_BOARD_RDKRX63N
	SYSTEM.PRCR.WORD = 0xA50B; /* Protect off */
    #endif
    
	/* clear ACSE Bit (All-Module Clock Stop Mode Enable) */	
	SYSTEM.MSTPCRA.BIT.ACSE = 0;  
	  
	/* Cancel stop state of SCI2 Peripheral to enable writing to it*/	
    MSTP(SCI6) = 0;	
    
    #ifdef PLATFORM_BOARD_RDKRX63N
	SYSTEM.PRCR.WORD = 0xA500; /* Protect on  */
    #endif     
    
    /* Clear bits TIE, RIE, RE, and TEIE in SCR to 0. Set CKE to internal. */
	SCI6.SCR.BYTE = 0x00;

    /* Set up the UART I/O port and pins. */
    MPC.P32PFS.BYTE  = 0x4A; /* PE1 is TxD2 */
    MPC.P33PFS.BYTE  = 0x4A; /* PE2 is RxD2 */
	
    PORT3.PDR.BIT.B2 = 1;    /* TxD12 is output. */
    PORT3.PDR.BIT.B3 = 0;    /* RxD12 is input. */
	
    PORT3.PMR.BIT.B2 = 1;    /* TxD12 is peripheral. */
    PORT3.PMR.BIT.B3 = 1;    /* RxD12 is peripheral. */
	
	SCI6.SMR.BYTE = 0x00;
    SCI6.SCMR.BIT.SMIF = 0;
	SCI6.SCMR.BIT.SDIR = 0; /* Set to 0 for serial communications interface mode. */
	SCI6.BRR = 155;
	    
    /* Enable RXI and TXI interrupts in SCI peripheral */
	SCI6.SCR.BIT.RIE  = 1;   /* Set Receive Interrupt (RX buffer full) enable. */
	SCI6.SCR.BIT.TIE  = 1;   /* Set Transmit Interrupt (TX data register empty) enable. */
    SCI6.SCR.BIT.TEIE = 1;   /* Set Transmit End interrupt enable */
	 IEN(SCI6, RXI6) = 1; 
     IEN(SCI6, TXI6) = 1; 
    
	/* Set interrupt prio for SCI. */
	IPR(SCI6, TXI6) = 3;
	IPR(SCI6, RXI6) = 3;
    /* Clear IR bits for TIE and RIE */
    IR(SCI6, RXI6) = 0;
    IR(SCI6, TXI6) = 0;
	SCI6.SCR.BYTE |= 0x30;
}

void transmit(unsigned char *data2)
{
	int j;
	for(j=0 ; j < 19 ; j++){
		transmit_data(data[j]);
	}

}
void transmit_data(unsigned char new_char){
		while(!(SCI6.SSR.BIT.TEND));
	//IEN(SCI6, TXI6) = 0;
	SCI6.TDR = new_char;
}

void delay(void) //Delay Function 
{
	volatile int i, j;
	for(i = 0; i < 1000; i++)
		for(j = 0; j < 100; j++);
}