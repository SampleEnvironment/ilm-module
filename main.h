/*
 * main.h
 *
 * Created: 10.08.2021 16:47:29
 *  Author: qfj
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#define FIRMWARE_VERSION  2 /**< @brief  Software Version  */

#define BRANCH_ID 1


#define CHECK_BOUNDS(VAR,MIN,MAX,DEF,FLAG) if((VAR < MIN) || (VAR > MAX || isnan(VAR))){VAR = DEF; FLAG = 1;};


#define NUMBER_LOGIN_BYTES 4 //TODO

#define PING_INTERVALL_DEF 60*10
#define PING_INTERVALL_MIN 60
#define PING_INTERVALL_MAX 60*120

#define T_TRANSMISSION_MIN_DEF 20	 //20s
#define T_TRANSMISSION_MIN_MIN 1	 //1s
#define T_TRANSMISSION_MIN_MAX 60*10 //10min

#define T_TRANSMISSION_MAX_DEF 20    //20min
#define T_TRANSMISSION_MAX_MIN 1     //1min
#define T_TRANSMISSION_MAX_MAX 60*10 //10h

#define N2_1_SPAN_DEF  1		
#define N2_1_SPAN_MIN  0.001
#define N2_1_SPAN_MAX  2000

#define N2_1_ZERO_DEF  0
#define N2_1_ZERO_MIN  (-2000)
#define N2_1_ZERO_MAX  ( 2000)

#define N2_2_SPAN_DEF  1
#define N2_2_SPAN_MIN  0.001
#define N2_2_SPAN_MAX  2000

#define N2_2_ZERO_DEF  0
#define N2_2_ZERO_MIN  (-2000)
#define N2_2_ZERO_MAX  ( 2000)


#define HE_SPAN_DEF  1
#define HE_SPAN_MIN  0.001
#define HE_SPAN_MAX  2000

#define HE_ZERO_DEF  0
#define HE_ZERO_MIN  (-2000)
#define HE_ZERO_MAX  ( 2000)

#define N2_1_DELTA_DEF  10
#define N2_1_DELTA_MIN  0
#define N2_1_DELTA_MAX  1000

#define N2_2_DELTA_DEF  10
#define N2_2_DELTA_MIN  0
#define N2_2_DELTA_MAX  1000

#define HE_DELTA_DEF    10
#define HE_DELTA_MIN    0
#define HE_DELTA_MAX    1000


#define SPAN_ZERO_DECIMAL_PLACES 10 // --> 0.1

typedef struct {
	double span;
	double zero;
	uint16_t delta;
}ADCparamsType;


typedef struct {
	uint16_t t_transmission_min;	/**< @brief Minimal time between Measurement-Data Transmissions (in seconds) */
	uint16_t t_transmission_max;	/**< @brief Maximum time between Measurement-Data Transmissions (in minutes) */
	uint16_t ping_intervall;        /**< @brief time intervall between pings to the server (in seconds) */
	
	ADCparamsType helium_par;
	ADCparamsType N2_1_par;
	ADCparamsType N2_2_par;
}optionsType;


typedef struct {
	double He;
	double N2_1;
	double N2_2;	
} MeasType;

/**
* @brief Time Pressure or Temperature deltas
*
* All Variables in ILM-Module representing a difference in Time, or measured value since the last Measuerement/Send/Reset
*/
typedef struct {
	MeasType values_since_last_send;       /**< @brief difference in he and n2 channels since last send */
	uint32_t t_send;					   /**< @brief Time since last sending of Data */
} deltaType;


/**
* @brief Pressure- & Timestamps of previous Events
*
* Absolute Times/Pressure of the last time an action was executetd i.e: Pinging the Server
*/
typedef struct {
	MeasType Measurement_on_send;      /**< @brief Measured he and N2 Values last time Data was sent/stored to the Server/memory */
	MeasType Measurement;
	uint32_t time_send;				   /**< @brief Absolute time of last sending of Data*/
	uint32_t time_ping;				   /**< @brief Absolute time of last Ping*/
	uint32_t time_level_meas;  /**< @brief Absolute time of last Temp and Pressure Measurement */
}lastType;





void init_timer(void);
void init(void);
void write_optsEEPROM(void);
uint8_t read_optsEEPROM(void);
void set_Options(uint8_t * optBuffer);
uint8_t xbee_send_login_msg(uint8_t db_cmd_type, uint8_t *buffer);
uint8_t ping_server(void);
void execute_server_CMDS(uint8_t reply_id);
void uint16_t_to_Buffer(uint16_t var, uint8_t * buffer, uint8_t index);


#endif /* MAIN_H_ */