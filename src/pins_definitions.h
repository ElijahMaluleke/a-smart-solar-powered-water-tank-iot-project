/********************************************************************************/
//  
 
 
// 
/* P0 signal */ 					/* Function */	   /* Peripheral */  /* Default connected on P0  */
/* PORT P0 */	
#define PORT_P0_PIN00 				0 // ( UART0_TXD ) |                 |        Yes 
#define PORT_P0_PIN01				1 // ( UART0_RXD ) |                 |        Yes 
#define PORT_P0_PIN02				2 // ( UART0_RST ) |                 |        Yes 
#define PORT_P0_PIN03				3 // ( UART0_CST ) |                 |        Yes 
#define PORT_P0_PIN04				4 // ( BUTTON 3 )  |   Button 3      |        Yes 


/* P0 signal */ 					/* Function */		 	/* Peripheral */ 	/* Default connected on P1  */
/* PORT P1  */
#define PORT_P1_PIN00 				0 // ( 32.768 kHz, XL1 ) |                    |     No, solder bridges must be configured     
#define PORT_P1_PIN01				1 // ( 32.768 kHz, XL2 ) |                    |     No, solder bridges must be configured      
#define PORT_P1_PIN02				2 // ( NFC1 )            |                    |     No, 0R resistors must be configured  
#define PORT_P1_PIN03				3 // ( NFC2 )            |                    |     No, 0R resistors must be configured  
#define PORT_P1_PIN04 				4 // ( UART1_TXD )  AIN0 |                    |     Yes 
#define PORT_P1_PIN05				5 // ( UART1_RXD )  AIN1 |                    |     Yes  
#define PORT_P1_PIN06				6 // ( UART1_RST )  AIN2 |                    |     Yes  
#define PORT_P1_PIN07				7 // ( UART1_CTS )  AIN3 |                    |     Yes  
#define PORT_P1_PIN08 				8 // ( BUTTON 2 )        |    Button 2        |     Yes  
#define PORT_P1_PIN09				9 // ( BUTTON 1 )        |    Button 1        |     Yes  
#define PORT_P1_PIN10				10 // ( LED 1 )          |    LED 1           |     Yes
#define PORT_P1_PIN11				11 // ( )           AIN4 |                    |     Yes  
#define PORT_P1_PIN12				12 // ( )           AIN5 |                    |     Yes   
#define PORT_P1_PIN13				13 // ( BUTTON 0 )  AIN6 |    Button 0        |     Yes  
#define PORT_P1_PIN14				14 // ( LED 3 )     AIN7 |    LED 3           |     Yes  


/* GPIO */							/* Flash memory pin */	/* Solder bridge for memory use (default:shorted) */ /* Solder bridge for GPIO use (default: open) */
/* P2 signal */  					/* Function */			/* Peripheral */	/* Default connected on P2 */
/* PORT P2 */	
#define PORT_P2_PIN00 				0 // ( SIO_3/HOLD )   |           SB16         |         SB17
#define PORT_P2_PIN01				1 // ( CLK )          |           SB12         |         SB18
#define PORT_P2_PIN02				2 // ( SIO_0/SI )     |           SB13         |         SB19  
#define PORT_P2_PIN03				3 // ( SIO_2/WP )     |           SB15         |         SB20     
#define PORT_P2_PIN04 				4 // ( SIO_1/SO )     |           SB14         |         SB21    
#define PORT_P2_PIN05				5 // ( CS )           |           SB11         |         SB22
#define PORT_P2_PIN06				6 // ( Trace CLK)     |                        |     Yes  
#define PORT_P2_PIN07				7 // ( Trace [0] )    |   LED 2                |     Yes  
#define PORT_P2_PIN08 				8 // ( Trace [1] )    |                        |     Yes  
#define PORT_P2_PIN09				9 // ( Trace [2] )    |   LED 0                |     Yes  
#define PORT_P2_PIN10				10 // ( Trace [3] )   |                        |     Yes  

/********************************************************************************/
// 

// user leds 
#define LED_ONE	 	 				PORT_P2_PIN09 //  
#define LED_TWO 					PORT_P1_PIN10 //  
#define LED_THREE	  	  	  		PORT_P2_PIN07 // 
#define LED_FOUR 			 		PORT_P1_PIN14 // 
// user buttons
#define USER_BUTTON_ONE		 		PORT_P1_PIN13 // 
#define USER_BUTTON_TWO		 		PORT_P1_PIN09 //  
#define USER_BUTTON_THREE		 	PORT_P1_PIN08 //  
#define USER_BUTTON_FOUR		 	PORT_P0_PIN04 // 

/********************************************************************************/
//  
 

// get ultrasonic range sensor
#define O_ECHO_ULTRASONIC_SENSOR 	PORT_P1_PIN11 // Output pin
#define O_TRIG_ULTRASONIC_SENSOR 	PORT_P2_PIN01 // Input pin

// get ultrasonic range sensor
#define W_ECHO_ULTRASONIC_SENSOR 	PORT_P1_PIN08 // Output pin ( BUTTON 2 )
#define W_TRIG_ULTRASONIC_SENSOR 	PORT_P1_PIN09 // Input pin ( BUTTON 1 ) 
    
// buzzer
#define BUZZER						PORT_P2_PIN00 // 
// pir sensor
#define	PIR_MODULE_PIN				PORT_P1_PIN12 //   
// water valves
#define WATER_VALVE_IN				PORT_P2_PIN02 // water valve in
#define WATER_VALVE_OUT				PORT_P2_PIN03 // water valve out
// water flow sensor
#define WATER_FLOW_SENSOR			PORT_P2_PIN04 //  
// water level sensor
//#define WATER_LEVEL_SENSOR			PORT_P2_PIN05 //
// oled display pins
#define OLED_I2C_SCL  				PORT_P2_PIN06 //
#define OLED_I2C_SDL  				PORT_P2_PIN08 // 
 
//#define PORT_P0_PIN04 // ( BUTTON 3 )  
//#define PORT_P1_PIN08 // ( BUTTON 2 )
//#define PORT_P1_PIN09 // ( BUTTON 1 ) 
#define WATER_LEVEL_SENSOR			PORT_P1_PIN13 // ( BUTTON 0 )  AIN6 

/********************************************************************************/
//  

#define SAADC_CHANNEL_0  			0 // 	 
#define SAADC_CHANNEL_1  			1 // 	 
#define SAADC_CHANNEL_2  			2 // 	 
#define SAADC_CHANNEL_3  			3 // 	 
#define SAADC_CHANNEL_4  			4 // 	 
#define SAADC_CHANNEL_5  			5 // 	 
#define SAADC_CHANNEL_6  			6 // 	 
#define SAADC_CHANNEL_7  			7 // 	 

/********************************************************************************/
//  
 

#define SENSORS_STATUS_LED	    	LED_ONE   // 
#define BAT_STATUS_LED          	LED_TWO   // 
#define PIR_MODULE_LED				LED_THREE // 
#define USER_LED                	LED_THREE // 

// 
/********************************************************************************/ 
