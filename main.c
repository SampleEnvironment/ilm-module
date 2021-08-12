/*
* ilm-module.c
*
* Created: 10.08.2021 11:34:03
* Author : qfj
*/

#include <avr/io.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <string.h>

#include "main.h"
#include "usart.h"
#include "xbee.h"
#include "module_globals.h"
#include "status.h"
#include "xbee_utilities.h"



uint8_t 	sendbuffer[SINGLE_FRAME_LENGTH]; /**< @brief Send only Buffer used for sending data to the server for which no reply is expected i.e. Measurement data #CMD_send_data_91*/
uint8_t     answerbuffer[SINGLE_FRAME_LENGTH]; /**< @brief Contains Answer to the last sent Request*/

optionsType EEMEM eeOptions = {
	.ping_intervall = PING_INTERVALL_DEF,
	
	.t_transmission_min = T_TRANSMISSION_MIN_DEF,
	.t_transmission_max = T_TRANSMISSION_MAX_DEF,
	
	.helium_par.span = HE_SPAN_DEF,
	.helium_par.zero = HE_ZERO_DEF,
	
	.N2_1_par.span = N2_1_SPAN_DEF,
	.N2_1_par.zero = N2_1_ZERO_DEF,
	
	.N2_2_par.span = N2_2_SPAN_DEF,
	.N2_2_par.zero = N2_2_ZERO_DEF
	
};


optionsType Options = {
	.ping_intervall = PING_INTERVALL_DEF,
	
	.t_transmission_min = T_TRANSMISSION_MIN_DEF,
	.t_transmission_max = T_TRANSMISSION_MAX_DEF,
	
	.helium_par.span = HE_SPAN_DEF,
	.helium_par.zero = HE_ZERO_DEF,
	
	.N2_1_par.span = N2_1_SPAN_DEF,
	.N2_1_par.zero = N2_1_ZERO_DEF,
	
	.N2_2_par.span = N2_2_SPAN_DEF,
	.N2_2_par.zero = N2_2_ZERO_DEF
};





void init(void){
	
	version_INIT(FIRMWARE_VERSION,BRANCH_ID,FIRMWARE_VERSION);
	usart_init(39);  //USART0 init with 9600 baud
	usart1_init(39); //USART1 init with 9600 baud
	xbee_init(NULL,NULL,0);
	init_timer();
	
	adc_init(0);
	adc_init(1);
	adc_init(2);
	
	//=========================================================================
	// Enable global interrupts
	//=========================================================================
	sei();

	
	
}

/**
* @brief Initializes Timer1 of the controller so it send interrupts with a frequency of 1 Hz.
*
*
* @return void
*/
void init_timer(void)
{
	//Timer1 f ~ 1Hz
	TCCR1B |= (1<<WGM12) | (1<<CS12) | (1<<CS10);
	OCR1A = 5999;
	TIMSK1 |= (1<<OCIE1A);
	
	/*
	//Timer0 f ~ 40Hz T ~ 25ms
	TCCR0A |= (1<<WGM01);
	TCCR0B |= (1<<CS00) | (1<<CS02); // N = 1024
	OCR0A = 2;
	TIMSK0 |= (1<<OCIE1A);
	*/
}

void write_optsEEPROM(void){
	eeprom_update_block(&Options,  &eeOptions, sizeof(Options));
	
}

uint8_t read_optsEEPROM(void){
	optionsType OptionsBuff;
	
	eeprom_read_block(&OptionsBuff,&eeOptions,sizeof(OptionsBuff));
	
	uint8_t Val_outof_Bounds = 0;
	
	CHECK_BOUNDS(OptionsBuff.ping_intervall,PING_INTERVALL_MIN,PING_INTERVALL_MAX,PING_INTERVALL_DEF,Val_outof_Bounds);
	
	CHECK_BOUNDS(OptionsBuff.t_transmission_min,T_TRANSMISSION_MIN_MIN,T_TRANSMISSION_MIN_MAX,T_TRANSMISSION_MIN_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.t_transmission_max,T_TRANSMISSION_MAX_MIN,T_TRANSMISSION_MAX_MAX,T_TRANSMISSION_MAX_DEF,Val_outof_Bounds);
	
	
	CHECK_BOUNDS(OptionsBuff.helium_par.span,HE_SPAN_MIN,HE_SPAN_MAX,HE_SPAN_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.helium_par.zero,HE_ZERO_MIN,HE_ZERO_MAX,HE_ZERO_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.helium_par.delta,HE_DELTA_MIN,HE_DELTA_MAX,HE_DELTA_DEF,Val_outof_Bounds);
	
	CHECK_BOUNDS(OptionsBuff.N2_1_par.span,N2_1_SPAN_MIN,N2_1_SPAN_MAX,N2_1_SPAN_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.N2_1_par.zero,N2_1_ZERO_MIN,N2_1_ZERO_MAX,N2_1_ZERO_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.N2_1_par.delta,N2_1_DELTA_MIN,N2_1_DELTA_MAX,N2_1_DELTA_DEF,Val_outof_Bounds);
	
	CHECK_BOUNDS(OptionsBuff.N2_2_par.span,N2_2_SPAN_MIN,N2_2_SPAN_MAX,N2_2_SPAN_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.N2_2_par.zero,N2_2_ZERO_MIN,N2_2_ZERO_MAX,N2_2_ZERO_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.N2_2_par.delta,N2_2_DELTA_MIN,N2_2_DELTA_MAX,N2_2_DELTA_DEF,Val_outof_Bounds);
	
	if (OptionsBuff.t_transmission_min >= OptionsBuff.t_transmission_max * 60)
	{
		OptionsBuff.t_transmission_min = T_TRANSMISSION_MIN_DEF;
		OptionsBuff.t_transmission_max = T_TRANSMISSION_MAX_DEF;
		
		Val_outof_Bounds = 1 ;
	}
	
	memcpy(&Options,&OptionsBuff,sizeof(Options));
	
	return Val_outof_Bounds;
}

uint8_t xbee_send_login_msg(uint8_t db_cmd_type, uint8_t *buffer)
{

	uint8_t reply_Id = 0;
	
	buffer[0] = 0; // Statusbyte for login 0
	
	uint8_t number_trials = 1;

	
	while(number_trials)
	{
		reply_Id = xbee_send_request_only(db_cmd_type, buffer, 1);
		
		//#ifdef ALLOW_DEBUG
		//sprintf(print_temp,"%i",reply_Id);
		//LCD_InitScreen_AddLine(print_temp, 0);
		//_delay_ms(10000);
		//#endif
		
		
		if(reply_Id != 0xFF)
		{

			_delay_ms(1000);
			//sprintf(print_temp,"%i",frameBuffer[reply_Id].data_len);
			//LCD_InitScreen_AddLine(print_temp,0);
			
			if(frameBuffer[reply_Id].data_len == NUMBER_LOGIN_BYTES) {
				return reply_Id;	//good options
			}
			else {
				return 0xFF;  //bad options
			}
			
		}
		
		
		if(!(--number_trials))
		{
			//stop trying and go in error mode; no functionality available from here on
			_delay_ms(2000);
			return 0xFF;

		}

		
	}
	return 0xFF;
}


void set_Options(uint8_t * optBuffer){
	
	optionsType OptionsBuff ={
		.ping_intervall      =            ((uint16_t) optBuffer[0] << 8) | optBuffer[1] ,
		
		.t_transmission_min  =            ((uint16_t) optBuffer[2] << 8) | optBuffer[3] ,
		.t_transmission_max  =            ((uint16_t) optBuffer[4] << 8) | optBuffer[5] ,
		
		.helium_par.span     =		      ((double) (((uint16_t) optBuffer[6] << 8) | optBuffer[7]))/10 ,   // TODO genauigkeit von span und Zero ???
		.helium_par.zero     =		      ((double) (((uint16_t) optBuffer[8] << 8) | optBuffer[9]))/10 ,
		.helium_par.delta    =			  ((uint16_t) optBuffer[10] << 8) | optBuffer[11] ,
		
		.N2_1_par.span     =		      ((double) (((uint16_t) optBuffer[12] << 8) | optBuffer[13]))/10 ,
		.N2_1_par.span     =		      ((double) (((uint16_t) optBuffer[14] << 8) | optBuffer[15]))/10 ,
		.N2_1_par.delta    =			  ((uint16_t) optBuffer[16] << 8) | optBuffer[17] ,
		
		.N2_2_par.span     =		      ((double) (((uint16_t) optBuffer[18] << 8) | optBuffer[19]))/10 ,
		.N2_2_par.span     =		      ((double) (((uint16_t) optBuffer[20] << 8) | optBuffer[21]))/10,
		.N2_2_par.delta    =			  ((uint16_t) optBuffer[22] << 8) | optBuffer[23] 		
		
	};
	
	
	uint8_t Val_outof_Bounds = 0;
	
	CHECK_BOUNDS(OptionsBuff.ping_intervall,PING_INTERVALL_MIN,PING_INTERVALL_MAX,PING_INTERVALL_DEF,Val_outof_Bounds);

	CHECK_BOUNDS(OptionsBuff.t_transmission_min,T_TRANSMISSION_MIN_MIN,T_TRANSMISSION_MIN_MAX,T_TRANSMISSION_MIN_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.t_transmission_max,T_TRANSMISSION_MAX_MIN,T_TRANSMISSION_MAX_MAX,T_TRANSMISSION_MAX_DEF,Val_outof_Bounds);


	CHECK_BOUNDS(OptionsBuff.helium_par.span,HE_SPAN_MIN,HE_SPAN_MAX,HE_SPAN_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.helium_par.zero,HE_ZERO_MIN,HE_ZERO_MAX,HE_ZERO_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.helium_par.delta,HE_DELTA_MIN,HE_DELTA_MAX,HE_DELTA_DEF,Val_outof_Bounds);
	
	CHECK_BOUNDS(OptionsBuff.N2_1_par.span,N2_1_SPAN_MIN,N2_1_SPAN_MAX,N2_1_SPAN_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.N2_1_par.zero,N2_1_ZERO_MIN,N2_1_ZERO_MAX,N2_1_ZERO_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.N2_1_par.delta,N2_1_DELTA_MIN,N2_1_DELTA_MAX,N2_1_DELTA_DEF,Val_outof_Bounds);
	
	CHECK_BOUNDS(OptionsBuff.N2_2_par.span,N2_2_SPAN_MIN,N2_2_SPAN_MAX,N2_2_SPAN_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.N2_2_par.zero,N2_2_ZERO_MIN,N2_2_ZERO_MAX,N2_2_ZERO_DEF,Val_outof_Bounds);
	CHECK_BOUNDS(OptionsBuff.N2_2_par.delta,N2_2_DELTA_MIN,N2_2_DELTA_MAX,N2_2_DELTA_DEF,Val_outof_Bounds);

	if (OptionsBuff.t_transmission_min >= OptionsBuff.t_transmission_max * 60)
	{
		OptionsBuff.t_transmission_min = T_TRANSMISSION_MIN_DEF;
		OptionsBuff.t_transmission_max = T_TRANSMISSION_MAX_DEF;
		
		Val_outof_Bounds = 1 ;
	}
	
	
	if (!Val_outof_Bounds)
	{
		sendbuffer[0] = 0; // setting options was successful
		xbee_send_message(CMD_send_response_options_set_93,sendbuffer,1);
		memcpy(&Options,&OptionsBuff,sizeof(Options));
		return;
	}
	
	sendbuffer[0] = 1; // setting options was not successful
	xbee_send_message(CMD_send_response_options_set_93,sendbuffer,1);

	

}

int main(void)
{
	init();
	
	
	read_optsEEPROM();
	
	if(xbee_reset_connection())
	{
		if(xbee_get_server_adrr())
		{
			_delay_ms(2000);
			if(!CHECK_ERROR(NETWORK_ERROR))
			{

				_delay_ms(2000);
				//=========================================================================
				// Device Login
				//=========================================================================

				uint8_t reply_id = xbee_send_login_msg(CMD_send_registration_90, sendbuffer);
				
				if (reply_id!= 0xFF ){ // GOOD OPTIONS RECEIVED
					set_Options(frameBuffer[reply_id].data);
				}
				else // DEFECTIVE OPTIONS RECEIVED
				{
					SET_ERROR(OPTIONS_ERROR);
					SET_ERROR(INIT_OFFLINE_ERROR);
				}
				
			}
			else
			{ // No stable Connection was reached
				SET_ERROR(INIT_OFFLINE_ERROR);
			}


		}
	}
	
	/* Replace with your application code */
	while (1)
	{
	}
}


ISR(TIMER1_COMPA_vect)
{
	count_t_elapsed++;
}

