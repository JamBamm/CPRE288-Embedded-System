/*
*
*   uart-interrupt.c
*
*
*
*   @author
*   @date
*/

// The "???" placeholders should be the same as in your uart.c file.
// The "?????" placeholders are new in this file and must be replaced.

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "uart-interrupt.h"

// These variables are declared as examples for your use in the interrupt handler.
//volatile char command_byteGo = '\0' ; // byte value for special character used as a command
//volatile char command_byteStop = '\0';
volatile char g_command_byte = '\0';
volatile bool g_command_ready = false;

#define TX_BUFFER_SIZE 256

volatile char tx_buffer[TX_BUFFER_SIZE];
volatile int tx_head = 0;
volatile int tx_tail = 0;


//external to be used in not only main but possibly other programs
volatile int command_flag = 0; // flag to tell the main program a special command was received

void uart_interrupt_init(void){
	//TODO
  //enable clock to GPIO port B
  SYSCTL_RCGCGPIO_R |= 0x02;

  //enable clock to UART1
  SYSCTL_RCGCUART_R |= 0x02;

  //wait for GPIOB and UART1 peripherals to be ready
  while ((SYSCTL_PRGPIO_R & 0x02) == 0) {};
  while ((SYSCTL_PRUART_R & 0x02) == 0) {};

  //enable digital functionality on port B pins
  GPIO_PORTB_DEN_R |= 0x03;

  //enable alternate functions on port B pins
  GPIO_PORTB_AFSEL_R |= 0x03;

  //enable UART1 Rx and Tx on port B pins
  GPIO_PORTB_PCTL_R = 0x00000011;

  //calculate baud rate
  uint16_t iBRD = 8; //16000000/ (16.0*115200)
  uint16_t fBRD = 44; //{16000000/ (16.0*115200)-8}*64+0.5

  //turn off UART1 while setting it up
  UART1_CTL_R &= ~0x01;

  //set baud rate
  //note: to take effect, there must be a write to LCRH after these assignments
  UART1_IBRD_R = iBRD;
  UART1_FBRD_R = fBRD;

  //set frame, 8 data bits, 1 stop bit, no parity, no FIFO
  //note: this write to LCRH must be after the BRD assignments
  UART1_LCRH_R = 0x60;

  //use system clock as source
  //note from the datasheet UARTCCC register description:
  //field is 0 (system clock) by default on reset
  //Good to be explicit in your code
  UART1_CC_R = 0x0;

  //////Enable interrupts

  //first clear RX interrupt flag (clear by writing 1 to ICR)
  UART1_ICR_R |= 0x10;

  //enable RX raw interrupts in interrupt mask register
  UART1_IM_R |= 0x10;

  //NVIC setup: set priority of UART1 interrupt to 1 in bits 21-23
  //Number in that section mean prioity
  NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;

  //NVIC setup: enable interrupt for UART1, IRQ #6, set bit 6
  NVIC_EN0_R |= 0x40;

  //tell CPU to use ISR handler for UART1 (see interrupt.h file)
  //from system header file: #define INT_UART1 22
  IntRegister(INT_UART1, UART1_Handler);

  //globally allow CPU to service interrupts (see interrupt.h file)
  IntMasterEnable();

  //re-enable UART1 and also enable RX, TX (three bits)
  //note from the datasheet UARTCTL register description:
  //RX and TX are enabled by default on reset
  //Good to be explicit in your code
  //Be careful to not clear RX and TX enable bits
  //(either preserve if already set or set them)
  UART1_CTL_R = 0x301;

}

void uart_sendChar(char data){
	// //TODO
	// //sends string doesnt exceed the limit
 //    while (UART1_FR_R & UART_FR_TXFF){}
 //    UART1_DR_R = data;
	//buffer method instead 
	int next_head = (tx_head + 1) % TX_BUFFER_SIZE;

    //wait if buffer full
    while (next_head == tx_tail) {}

	//is tx inturrpt disabled
    bool was_idle = ((UART1_IM_R & 0x20) == 0);
	
	//disable to edit
    UART1_IM_R &= ~0x20; 

	//add the new chars
    tx_buffer[tx_head] = data;
    tx_head = next_head;

    if (was_idle) {
        UART1_DR_R = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
    }

    //now enabled and ready to fire
    UART1_IM_R |= 0x20; 
}

char uart_receive(void){
	//DO NOT USE this busy-wait function if using RX interrupt
	// recieves char and waits for it until it arrives
    while (UART1_FR_R & 0x10) {}
    return (char)(UART1_DR_R & 0xFF);
	//return 0;
}

void uart_sendStr(const char *data){

	//TODO for reference see lcd_puts from lcd.c file
    if (!data) return;
		//handle some special chars
       while (*data)
       {
           if (*data == '\n')
           {
               uart_sendChar('\r');
               uart_sendChar('\n');
           }
           else
           {
               uart_sendChar(*data);
           }
           data++;
       }
}

// interrupt handler for receive interrupts
void UART1_Handler(void)
{	
    //check if handler called due to RX event
    if (UART1_MIS_R & 0x10)
    {
        // clear the RX trigger flag 
        UART1_ICR_R |= 0x10;

        //store
        g_command_byte = (char)(UART1_DR_R & 0xFF);
		g_command_ready = true;

		
		//echo
        uart_sendChar(g_command_byte);

    }
	//TX interrupt 
	if (UART1_MIS_R & 0x20) 
	{
		UART1_ICR_R |= 0x20; 

		if (tx_head != tx_tail) {
			UART1_DR_R = tx_buffer[tx_tail];
			tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
		} else {
			
			UART1_IM_R &= ~0x20;
		}
	}
}

