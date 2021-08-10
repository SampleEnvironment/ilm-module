/*
* ilm-module.c
*
* Created: 10.08.2021 11:34:03
* Author : qfj
*/

#include <avr/io.h>
#include <stdio.h>

#include "main.h"
#include "usart.h"
#include "xbee.h"


void init(void){
	usart_init(39);  //USART0 init with 9600 baud
	usart1_init(39); //USART1 init with 9600 baud
	xbee_init(NULL,NULL,0);
	
	//=========================================================================
	// Enable global interrupts
	//=========================================================================
	sei();

	
	
}


int main(void)
{
	init();
	
	/* Replace with your application code */
	while (1)
	{
	}
}

