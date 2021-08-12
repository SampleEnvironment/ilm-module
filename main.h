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




void init_timer(void);
void init(void);
void write_optsEEPROM(void);
uint8_t read_optsEEPROM(void);
void set_Options(uint8_t * optBuffer);
uint8_t xbee_send_login_msg(uint8_t db_cmd_type, uint8_t *buffer);



#endif /* MAIN_H_ */