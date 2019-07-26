//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### HARD.H ################################
//---------------------------------------------
#ifndef _HARD_H_
#define _HARD_H_

//--- Defines For Configuration ----------------------------
//--- Hardware Board Version -------------------------------
#define VER_1_0    //version original

#define VOUT_SETPOINT    VOUT_200V
#define I_SETPOINT    IOUT_3A

#define VBIAS_HIGH    VBIAS_25V
#define VBIAS_LOW     VBIAS_08V
#define VBIAS_START   VBIAS_10V

#define VLINE_START_THRESHOLD    VLINE_20V
#define VOUT_MAX_THRESHOLD    VOUT_240V
#define VOUT_MIN_THRESHOLD    VOUT_160V


//--- Configuration for Hardware Versions ------------------
#ifdef VER_2_0
#define HARDWARE_VERSION_2_0
#define SOFTWARE_VERSION_2_0
#endif

#ifdef VER_1_0
#define HARDWARE_VERSION_1_0
#define SOFTWARE_VERSION_1_0
#endif


//---- Features Configuration ----------------
//features are activeted here and annouced in hard.c
#define FEATURES

// SOFTWARE Features -------------------------
//-- Types of programs ----------
#define DRIVER_MODE
// #define HARD_TEST_MODE
// #define HARD_TEST_MODE_LINE_SYNC
// #define HARD_TEST_MODE_CONDUCTION_ANGLE
// #define HARD_TEST_MODE_RECT_SINUSOIDAL
// #define HARD_TEST_MODE_DISABLE_PWM
// #define HARD_TEST_MODE_ADC_SENSE

//-- Types of led indications ----------
#define USE_LED_FOR_MAIN_STATES
// #define USE_LED_FOR_MAINS_SYNC
// #define USE_LED_FOR_POSITIVE_VOLTAGE
// #define USE_LED_AS_TIM1_CH3
// #define USE_LED_FOR_SIGNAL

//-- Frequency selection ----------
// #define USE_FREQ_70KHZ    //max pwm: 686
#define USE_FREQ_48KHZ    //max pwm: 1000

//-- Types of Interrupts ----------
// #define WITH_AC_SYNC_INT
// #define WITH_OVERCURRENT_SHUTDOWN

//-- Types of Optoisolatar used ----------
#define OPTO_KB817
// #define OPTO_FOD8801


//---- End of Features Configuration ----------

//--- Stringtify Utils -----------------------
#define STRING_CONCAT(str1,str2) #str1 " " #str2
#define STRING_CONCAT_NEW_LINE(str1,str2) xstr(str1) #str2 "\n"
#define xstr_macro(s) str_macro(s)
#define str_macro(s) #s

//--- Hardware Welcome Code ------------------//
#ifdef HARDWARE_VERSION_1_0
#define HARD "Hardware V: 1.0\n"
#endif

//--- Software Welcome Code ------------------//
#ifdef SOFTWARE_VERSION_1_0
#define SOFT "Software V: 1.0\n"
#endif


//-------- Others Configurations depending on the formers ------------
#ifdef INVERTER_MODE_PURE_SINUSOIDAL
#ifndef INVERTER_MODE
#define INVERTER_MODE
#endif
#endif
//-------- Hysteresis Conf ------------------------

//-------- PWM Conf ------------------------

//-------- End Of Defines For Configuration ------

#define VBIAS_25V    698
#define VBIAS_12V    346
#define VBIAS_10V    279
#define VBIAS_08V    223
//bias @12V 1.14V -> 346  ;;medido 5-7-19

#define VOUT_100V    230
#define VOUT_160V    368
#define VOUT_200V    460
#define VOUT_240V    552
#define VOUT_300V    692
#define VOUT_350V    805
#define VOUT_400V    920    

#if defined OPTO_KB817
#define IOUT_3A    610
#define IOUT_2A    406    //esto da 2.24A en frio 8-7-19
#define IOUT_1A    203
//Iup @2.38A 1.56V -> 484  ;;medido 5-7-2019
#elif defined OPTO_FOD8801
#define IOUT_3A    915
#define IOUT_2A    610    //esto da 2.07A en frio 10-7-19
#define IOUT_1A    305
#else
#error "define the opto in hard.h"
#endif

#define VLINE_20V    55



#if (defined USE_FREQ_70KHZ)
#define SOFT_START_CNT_ROOF    140
#elif (defined USE_FREQ_48KHZ)
#define SOFT_START_CNT_ROOF    96
#else
#error "select FREQ on hard.h"
#endif

//------- PIN CONFIG ----------------------
#ifdef VER_1_0
//GPIOA pin0	Vbias_Sense
//GPIOA pin1	Vup
//GPIOA pin2	I_Sense
//GPIOA pin3	Iup
//GPIOA pin4	V220_Sense, Vline_Sense

//GPIOA pin5    NC

//GPIOA pin6    TIM3_CH1 (CTRL_MOSFET)

//GPIOA pin7    
//GPIOB pin0    
//GPIOB pin1	
//GPIOA pin8	
//GPIOA pin9    NC

//GPIOA pin10	LED
#define LED ((GPIOA->ODR & 0x0400) != 0)
#define LED_ON	GPIOA->BSRR = 0x00000400
#define LED_OFF GPIOA->BSRR = 0x04000000

//GPIOA pin11	
//GPIOA pin12	
//GPIOA pin13	
//GPIOA pin14	
//GPIOA pin15    NC

//GPIOB pin3	
//GPIOB pin4	
//GPIOB pin5	
//GPIOB pin6
//GPIOB pin7	NC
#endif

//------- END OF PIN CONFIG -------------------


//DRIVER States
typedef enum
{
    POWER_UP = 0,
    SOFT_START,
    VOLTAGE_MODE,
    CURRENT_MODE,
    OUTPUT_OVERVOLTAGE,
    INPUT_OVERVOLTAGE,
    OVERCURRENT,
    BIAS_OVERVOLTAGE,
    AUTO_RESTART,
    POWER_DOWN
    
} driver_states_t;


//ESTADOS DEL LED
typedef enum
{    
    START_BLINKING = 0,
    WAIT_TO_OFF,
    WAIT_TO_ON,
    WAIT_NEW_CYCLE
} led_state_t;


//Estados Externos de LED BLINKING
#define LED_NO_BLINKING               0
#define LED_STANDBY                   1
#define LED_VOLTAGE_MODE              2
#define LED_CURRENT_MODE              3
#define LED_JUMPER_PROTECTED          4
#define LED_VIN_ERROR                 5
#define LED_OVERCURRENT_POS           6
#define LED_OVERCURRENT_NEG           7



/* Module Functions ------------------------------------------------------------*/
unsigned short GetHysteresis (unsigned char);
unsigned char GetNew1to10 (unsigned short);
void UpdateVGrid (void);
void UpdateIGrid (void);
unsigned short GetVGrid (void);
unsigned short GetIGrid (void);
unsigned short PowerCalc (unsigned short, unsigned short);
unsigned short PowerCalcMean8 (unsigned short * p);
void ShowPower (char *, unsigned short, unsigned int, unsigned int);
void ChangeLed (unsigned char);
void UpdateLed (void);
unsigned short UpdateDMAX (unsigned short);
unsigned short UpdateDMAXSF (unsigned short);
unsigned short UpdateDmaxLout (unsigned short);
unsigned short VoutTicksToVoltage (unsigned short);
unsigned short VinTicksToVoltage (unsigned short);
unsigned short Hard_GetDmaxLout (unsigned short, unsigned short);
void WelcomeCodeFeatures (char *);

void Hard_Reset_Voltage_Filter (void);
unsigned char Hard_Update_Voltage_Sense (void);
unsigned char Hard_Get_Conduction_Angle (void);
unsigned short Hard_Get_Hidden_Value (void);

#endif /* _HARD_H_ */
