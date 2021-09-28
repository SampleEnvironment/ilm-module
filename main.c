/*
* ilm-module.c
*
* Created: 10.08.2021 11:34:03
* Author : qfj
*/

#include <avr/io.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>

#include "main.h"
#include "adwandler.h"
#include "usart.h"
#include "xbee.h"
#include "module_globals.h"
#include "status.h"
#include "xbee_utilities.h"
#include "DS3231M.h"
#include "LCD.h"


uint8_t 	sendbuffer[SINGLE_FRAME_LENGTH]; /**< @brief Send only Buffer used for sending data to the server for which no reply is expected i.e. Measurement data #CMD_send_data_91*/


optionsType EEMEM eeOptions = {
	.ping_intervall = PING_INTERVALL_DEF,
	
	.t_transmission_min = T_TRANSMISSION_MIN_DEF,
	.t_transmission_max = T_TRANSMISSION_MAX_DEF,
	
	.helium_par.span = HE_SPAN_DEF,
	.helium_par.zero = HE_ZERO_DEF,
	
	.N2_1_par.span = N2_1_SPAN_DEF,
	.N2_1_par.zero = N2_1_ZERO_DEF,
	
	.N2_2_par.span = N2_2_SPAN_DEF,
	.N2_2_par.zero = N2_2_ZERO_DEF,
	
	.measCycles = MEAS_CYCLES_DEF
	

	
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
	.N2_2_par.zero = N2_2_ZERO_DEF,
	
	.measCycles = MEAS_CYCLES_DEF
};


/************************************************************************/
/*  Time- and Pressurestamps                                            */
/************************************************************************/
/**
* @brief Time- and Pressurestamps
*
* Timestamps and Pressurevalues for last send/display-reset/ping - Event
*/
lastType last= {.time_send = 0,.time_ping = 0,.time_level_meas = 0};


// holds most recent meas data
MeasType current_meas;

deltaType delta = {
	.t_send = 0,
	.values_since_last_send.He = 0,
	.values_since_last_send.N2_1 = 0,
	.values_since_last_send.N2_2 = 0
};

/**
* @brief Measurement Period
*
* Time between Measurements (in s)
*/
const uint8_t Measure_Interval = 2;


/**
* @brief Checks current Connection Status. Possible states are "No Network" if no connection to a coordinator could be established. "No Server", if the Device is connected to a coordinator but, pings sent by #ping_server() are not answered. And "Online" if pings are answered by the server. The State is saved in #NetStatIndex
*
* @param dest_high high 32-bit of coordinator address
* @param dest_low  low  32-bit of coordinator address
*
* @return uint8_t 1 if online and 0 if there are any problems with the connection to the server
*/
uint8_t analyze_Connection(void)
{

	if (!xbee_reconnect())
	{
		//Associated
		CLEAR_ERROR(NETWORK_ERROR);
		CLEAR_ERROR(NO_REPLY_ERROR);
		
		if(!ping_server()){
			//offline
			//LCD_paint_info_line("NoServ",0);
			return 0;
		}
		else{
			//online;
			CLEAR_ERROR(NETWORK_ERROR);;
			CLEAR_ERROR(NO_REPLY_ERROR);;
			return 1;
		}
		
	}
	else
	{
		//offline
		//LCD_paint_info_line("NoNetw",0);
		return 0;
	}

}


// Pings Server and resets Time
/**
* @brief Pings the server in order to check if the connection is still live. If no Pong was received after #COM_TIMEOUT_TIME the Network error Bit is set in status#device.
*
* @param dest_high high 32-bit of coordinator address
* @param dest_low  low  32-bit of coordinator address
*
* @return uint8_t 1 if Ping successful and 0 if no Pong was received
*/
uint8_t ping_server(void)
{
	sendbuffer[0]= 0;
	uint8_t reply_id = xbee_send_request(PING_MSG,sendbuffer,1);
	if( 0xFF == reply_id)
	{
		_delay_ms(500);
		SET_ERROR(NETWORK_ERROR);

		return 0;
	}
	else
	{
		// Ping Successful --> time is set to the received time
		Time.tm_sec  = frameBuffer[reply_id].data[0];
		Time.tm_min  = frameBuffer[reply_id].data[1];
		Time.tm_hour = frameBuffer[reply_id].data[2];
		Time.tm_mday = frameBuffer[reply_id].data[3];
		Time.tm_mon  = frameBuffer[reply_id].data[4];
		Time.tm_year = frameBuffer[reply_id].data[5];
		
	}
	return 1;
}


/**
* @brief Initializes the interrupts of the controller.
*
*
* @return void
*/
void init_interrupts(void)
{
	EICRA |= (1<<ISC21);
	EIMSK |= (1<<INT2);
}


/**
* @brief Initializes the ports of the controller.
*
*
* @return void
*/
void init_ports(void)
{
	DDRC |= (1<<PINC4);
}

void paint_none(char* infoline,_Bool update){
	
}

void init(void){
	
	version_INIT(FIRMWARE_VERSION,BRANCH_ID,FIRMWARE_VERSION);
	
	//init_ports();
	
	_delay_ms(1000);
	usart_init(39);  //USART0 init with 9600 baud
	_delay_ms(100);

	//init_interrupts();
	xbee_init(&paint_none,NULL,0);
	_delay_ms(10);
	init_timer();
	_delay_ms(1000);
	adc_init(HELIUM);
	_delay_ms(100);
	adc_init(NITROGEN_1);
	_delay_ms(100);
	adc_init(NITROGEN_2);
	_delay_ms(100);
	
	//=========================================================================
	// Enable global interrupts
	//=========================================================================
	sei();

	xbee_hardware_version();
	
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
	
	
	if (!Val_outof_Bounds)
	{
		memcpy(&Options,&OptionsBuff,sizeof(Options));
	}
	
	
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


void read_channels(void){
	current_meas.He   = (readChannel(HELIUM,    Options.measCycles) * Options.helium_par.span) + Options.helium_par.zero;
	current_meas.N2_1 = (readChannel(NITROGEN_1,Options.measCycles) * Options.N2_1_par.span)   + Options.N2_1_par.zero;
	current_meas.N2_2 = (readChannel(NITROGEN_2,Options.measCycles) * Options.N2_2_par.span)   + Options.N2_2_par.zero;
	
	
	//update deltas
	//HELIUM
	//Avoid Overflow when subtracting
	if(current_meas.He > last.Measurement_on_send.He)
	{
		delta.values_since_last_send.He = current_meas.He - last.Measurement_on_send.He;
	}
	else
	{
		delta.values_since_last_send.He =  last.Measurement_on_send.He - current_meas.He;
	}
	
	
	//NITROGEN 1
	//Avoid Overflow when subtracting
	if(current_meas.N2_1 > last.Measurement_on_send.N2_1)
	{
		delta.values_since_last_send.N2_1 = current_meas.N2_1 - last.Measurement_on_send.N2_1;
	}
	else
	{
		delta.values_since_last_send.N2_1 =  last.Measurement_on_send.N2_1 - current_meas.N2_1;
	}
	
	//NITROGEN 2
	//Avoid Overflow when subtracting
	if(current_meas.N2_2 > last.Measurement_on_send.N2_2)
	{
		delta.values_since_last_send.N2_2 = current_meas.N2_2 - last.Measurement_on_send.N2_2;
	}
	else
	{
		delta.values_since_last_send.N2_2 =  last.Measurement_on_send.N2_2 - current_meas.N2_2;
	}

}

void Collect_Measurement_Data(void){
	
	
	
	uint16_t He_u16 = (uint16_t) (current_meas.He*10);
	
	uint16_t N2_1_u16 = (uint16_t) (current_meas.N2_1*10);
	
	uint16_t N2_2_u16 = (uint16_t) (current_meas.N2_2*10);
	
	uint16_t_to_Buffer(He_u16,sendbuffer,0);
	uint16_t_to_Buffer(N2_1_u16,sendbuffer,2);
	uint16_t_to_Buffer(N2_2_u16,sendbuffer,4);
	
	sendbuffer[6] = 0; //TODO status byte!!!
	
}


/**
* @brief Decodes incoming Messages from the Server and act accordingly
*
* @param reply_id index of the message in #frameBuffer which will be decoded
*
*
* @return void
*/
void execute_server_CMDS(uint8_t reply_id){
	switch (frameBuffer[reply_id].type)
	{
		//=================================================================
		case SET_OPTIONS_CMD:// set received Options
		set_Options((uint8_t*)frameBuffer[reply_id].data,SET_OPTIONS_CMD);
		break;
		
		//=================================================================
		case TRIGGER_MEAS_CMD: // Send Measurement Data immediately
		read_channels();
		Collect_Measurement_Data();
		xbee_send_message(TRIGGER_MEAS_CMD,sendbuffer,MEASUREMENT_MESSAGE_LENGTH);
		break;
		
		//=================================================================
		case GET_OPTIONS_CMD : ;// send current options to Server

		
		uint16_t_to_Buffer(Options.ping_intervall,sendbuffer,0);
		
		uint16_t_to_Buffer(Options.t_transmission_min,sendbuffer,2);
		uint16_t_to_Buffer(Options.t_transmission_max,sendbuffer,4);
		
		uint32_t_to_Buffer((uint16_t)(Options.helium_par.span*SPAN_ZERO_DECIMAL_PLACES),sendbuffer,6);
		uint32_t_to_Buffer((uint16_t)(Options.helium_par.zero*SPAN_ZERO_DECIMAL_PLACES),sendbuffer,10);
		uint16_t_to_Buffer(Options.helium_par.delta,sendbuffer,14);
		
		uint32_t_to_Buffer((uint16_t)(Options.N2_1_par.span*SPAN_ZERO_DECIMAL_PLACES),sendbuffer,16);
		uint32_t_to_Buffer((uint16_t)(Options.N2_1_par.zero*SPAN_ZERO_DECIMAL_PLACES),sendbuffer,20);
		uint16_t_to_Buffer(Options.N2_1_par.delta,sendbuffer,24);
		
		uint32_t_to_Buffer((uint16_t)(Options.N2_2_par.span*SPAN_ZERO_DECIMAL_PLACES),sendbuffer,26);
		uint32_t_to_Buffer((uint16_t)(Options.N2_2_par.zero*SPAN_ZERO_DECIMAL_PLACES),sendbuffer,30);
		uint16_t_to_Buffer(Options.N2_2_par.delta,sendbuffer,34);
		
		sendbuffer[36] = Options.measCycles;
		
		sendbuffer[37] = 0; // statusbyte
		

		xbee_send_message(GET_OPTIONS_CMD,sendbuffer,38);
		break;
		
		case SET_PING_INTERVALL_CMD:
		;
		uint8_t Val_outof_Bounds = 0;
		
		uint16_t buff_ping_Intervall  =            ((uint16_t) frameBuffer[reply_id].data[0] << 8) | frameBuffer[reply_id].data[1] ;
		
		CHECK_BOUNDS(buff_ping_Intervall,PING_INTERVALL_MIN,PING_INTERVALL_MAX,PING_INTERVALL_DEF,Val_outof_Bounds);
		if (!Val_outof_Bounds)
		{
			Options.ping_intervall = buff_ping_Intervall;
		}
		sendbuffer[0] = Val_outof_Bounds;
		
		//send status ack
		xbee_send_message(SET_PING_INTERVALL_CMD,sendbuffer,1);
		break;
		

	}
	//remove Frame from Buffer
	
	
	buffer_removeData(reply_id);
}

void uint16_t_to_Buffer(uint16_t var, uint8_t * buffer, uint8_t index){
	buffer[index]   =           var >> 8;
	buffer[++index] = (uint8_t) var;
}


void uint32_t_to_Buffer(uint32_t var, uint8_t * buffer, uint8_t index){
	buffer[index]   =           var >> 24;
	buffer[index]   =           var >> 16;
	buffer[index]   =           var >> 8;
	buffer[++index] = (uint8_t) var;
}

double span_zero_from_Buffer( uint8_t * buffer, uint8_t index){
	return ((double) (((uint32_t) buffer[index] << 24) |((uint32_t) buffer[index+1] << 16) |((uint32_t) buffer[index+2] << 8) | buffer[index+3]))/SPAN_ZERO_DECIMAL_PLACES;
}



void set_Options( uint8_t * optBuffer,uint8_t answer_code){
	
	optionsType OptionsBuff ={
		.ping_intervall      =            ((uint16_t) optBuffer[0] << 8) | optBuffer[1] ,
		
		.t_transmission_min  =            ((uint16_t) optBuffer[2] << 8) | optBuffer[3] ,
		.t_transmission_max  =            ((uint16_t) optBuffer[4] << 8) | optBuffer[5] ,
		
		.helium_par.span     =		      span_zero_from_Buffer(optBuffer,6),
		.helium_par.zero     =		      span_zero_from_Buffer(optBuffer,10) ,
		.helium_par.delta    =			  ((uint16_t) optBuffer[14] << 8) | optBuffer[15] ,
		
		.N2_1_par.span     =		      span_zero_from_Buffer(optBuffer,16),
		.N2_1_par.zero     =		      span_zero_from_Buffer(optBuffer,20) ,
		.N2_1_par.delta    =			  ((uint16_t) optBuffer[24] << 8) | optBuffer[25] ,
		
		.N2_2_par.span     =		      span_zero_from_Buffer(optBuffer,26) ,
		.N2_2_par.zero     =		      span_zero_from_Buffer(optBuffer,30) ,
		.N2_2_par.delta    =			  ((uint16_t) optBuffer[34] << 8) | optBuffer[35],
		
		.measCycles		   =              optBuffer[36]
		
		
		
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
		//no problem with received options --> save them in operational struct
		memcpy(&Options,&OptionsBuff,sizeof(Options));
		write_optsEEPROM();
		sendbuffer[0] = 0; // setting options was successful
		xbee_send_message(answer_code,sendbuffer,1);
		}else{
		
		sendbuffer[0] = 1; // setting options was not successful
		xbee_send_message(answer_code,sendbuffer,1);
	}
	


	

}

int main(void)
{
	
	
	init();

	//init_LCD();
	

	_delay_ms(4000);
	
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

				uint8_t reply_id = xbee_send_login_msg(LOGIN_MSG, sendbuffer);
				
				if (reply_id!= 0xFF ){ // GOOD OPTIONS RECEIVED
					
					set_Options((uint8_t*)frameBuffer[reply_id].data,OPTIONS_SET_ACK);
					
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

	//##########################################################################
	//				MAIN LOOP
	//##########################################################################
	while (1)
	{
		
		
		
		if((count_t_elapsed % Measure_Interval == 0) && ((count_t_elapsed - last.time_level_meas) > 1 )) //every Measure_Intervall
		{
			last.time_level_meas = count_t_elapsed;

			read_channels();
			

			
			
		}
		
		
		delta.t_send = count_t_elapsed - last.time_send;
		

		
		if(!CHECK_ERROR(NETWORK_ERROR)){
			// ONLINE
			
			
			//============================================================================================================================================
			// checks if either:
			// 1. the maximum time(t_transmission_max) has passed since last send  OR
			// 2. delta_HE is reached and at least t_transmission_min time has passed since last send
			// 3. delta_N2_1 is reached and at least t_transmission_min time has passed since last send
			// 4. delta_N2_2 is reached and at least t_transmission_min time has passed since last send
			
			
			if((delta.t_send >= Options.t_transmission_max * 60)|| //
			(delta.t_send >= Options.t_transmission_min && delta.values_since_last_send.He > Options.helium_par.delta)||
			(delta.t_send >= Options.t_transmission_min && delta.values_since_last_send.N2_1 > Options.N2_1_par.delta)||
			(delta.t_send >= Options.t_transmission_min && delta.values_since_last_send.N2_2 > Options.N2_2_par.delta))
			{

				

				Collect_Measurement_Data();

				// send Measurement Data to Server
				if( 0xFF ==xbee_send_request(MEAS_MSG,sendbuffer,MEASUREMENT_MESSAGE_LENGTH))
				{
					SET_ERROR(NETWORK_ERROR);
					analyze_Connection();
				}
				
				
				// Reset for delta calculation
				delta.values_since_last_send.He = 0;
				delta.values_since_last_send.N2_1 = 0;
				delta.values_since_last_send.N2_2 = 0;
				
				// Reset for delta calculation
				last.Measurement_on_send.He = current_meas.He;
				last.Measurement_on_send.N2_1 = current_meas.N2_1;
				last.Measurement_on_send.N2_2 = current_meas.N2_2;
				
				
				// Reset for time since last sent i.e. delta_t_send
				last.time_send = count_t_elapsed;
				

			}

			//==============================================================================================
			//   PING
			//==========================================================
			// since pressure/temp is measured every 5s and ping is done every 60+2 seconds to ensure they dont get triggerd at the same Second
			if (((count_t_elapsed % Options.ping_intervall) == 2) && ((count_t_elapsed - last.time_ping) > 5 ))
			{
				last.time_ping = count_t_elapsed;
				
				
				if(!ping_server()){
					if(!analyze_Connection()){
						continue;
					}
				}
				
				
			}
			
			//==============================================================================================
			//   EXECUTE SERVER COMMANDS
			//==========================================================
			
			uint8_t reply_id;
			reply_id  = xbee_hasReply(LAST_NON_CMD_MSG,GREATER_THAN);
			if (reply_id != 0xFF){
				execute_server_CMDS(reply_id);
			}

			// execute Server Commands
			
		}
		else
		{
			// Offline
			//========================================================
			//     RECONNECT
			//========================================================
			// try to Reconnect after every ping_intervall (Reconnect_after_time)
			//========================================================
			if (count_t_elapsed % Options.ping_intervall == 2){
				if (!xbee_reconnect())
				{
					//Associated
					CLEAR_ERROR(NETWORK_ERROR);
					CLEAR_ERROR(NO_REPLY_ERROR);
					if(!ping_server()){
						CLEAR_ERROR(NETWORK_ERROR);
						CLEAR_ERROR(NO_REPLY_ERROR);
					}
					
				}
			}


			
			
			
			
			
			
			
			
		}
	}
}

//=========================================================================
// Interrupts
//=========================================================================

ISR(TIMER1_COMPA_vect)
{
	count_t_elapsed++;
}

