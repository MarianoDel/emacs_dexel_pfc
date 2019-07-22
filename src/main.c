//-----------------------------------------------------
// #### PFC 150W PROJECT  F030 - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### MAIN.C ########################################
//-----------------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "gpio.h"
#include "tim.h"
#include "uart.h"
#include "hard.h"

#include "core_cm0.h"
#include "adc.h"
#include "dma.h"
#include "flash_program.h"
#include "dsp.h"
#include "it.h"
#include "sync.h"


// Externals -----------------------------------------------

// -- Externals from or for Serial Port --------------------
// volatile unsigned char tx1buff [SIZEOF_DATA];
// volatile unsigned char rx1buff [SIZEOF_DATA];
// volatile unsigned char usart1_have_data = 0;

// -- Externals from or for the ADC ------------------------
volatile unsigned short adc_ch [ADC_CHANNEL_QUANTITY];
volatile unsigned char seq_ready = 0;

// -- Externals for the timers -----------------------------
volatile unsigned short timer_led = 0;

// -- Externals used for analog or digital filters ---------
// volatile unsigned short take_temp_sample = 0;




// Globals -------------------------------------------------
volatile unsigned char overcurrent_shutdown = 0;
// volatile short d = 0;
// short ez1 = 0;
// short ez2 = 0;
// // unsigned short dmax = 0;
// unsigned short last_d = 0;
// #define DELTA_D    2


// ------- de los timers -------
volatile unsigned short wait_ms_var = 0;
volatile unsigned short timer_standby;
//volatile unsigned char display_timer;
// volatile unsigned short timer_meas;
volatile unsigned char timer_filters = 0;
// volatile unsigned short dmax_permited = 0;

#ifdef HARD_TEST_MODE_RECT_SINUSOIDAL
#define USE_SIGNAL_CURRENT_SIN_1_A
#define SIZEOF_SIGNAL 240

unsigned short mem_signal [SIZEOF_SIGNAL] = {13,26,39,52,65,78,91,104,117,130,
                                             143,156,169,182,195,207,220,233,246,258,
                                             271,284,296,309,321,333,346,358,370,382,
                                             394,406,418,430,442,453,465,477,488,499,
                                             511,522,533,544,555,566,577,587,598,608,
                                             619,629,639,649,659,669,678,688,697,707,
                                             716,725,734,743,751,760,768,777,785,793,
                                             801,809,816,824,831,838,845,852,859,866,
                                             872,878,884,891,896,902,908,913,918,923,
                                             928,933,938,942,946,951,955,958,962,965,
                                             969,972,975,978,980,983,985,987,989,991,
                                             993,994,995,996,997,998,999,999,999,1000,
                                             999,999,999,998,997,996,995,994,993,991,
                                             989,987,985,983,980,978,975,972,969,965,
                                             962,958,955,951,946,942,938,933,928,923,
                                             918,913,908,902,896,891,884,878,872,866,
                                             859,852,845,838,831,824,816,809,801,793,
                                             785,777,768,760,751,743,734,725,716,707,
                                             697,688,678,669,659,649,639,629,619,608,
                                             598,587,577,566,555,544,533,522,511,499,
                                             488,477,465,453,442,430,418,406,394,382,
                                             370,358,346,333,321,309,296,284,271,258,
                                             246,233,220,207,195,182,169,156,143,130,
                                             117,104,91,78,65,52,39,26,13,0};
#endif

unsigned short * p_signal;

// Module Functions ----------------------------------------
void TimingDelay_Decrement (void);
// extern void EXTI4_15_IRQHandler(void);



//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
    unsigned char i;
    unsigned short ii;

    driver_states_t driver_state = AUTO_RESTART;
    unsigned char soft_start_cnt = 0;
    short d = 0;
    short ez1 = 0;
    short ez2 = 0;


    //GPIO Configuration.
    GPIO_Config();

    //ACTIVAR SYSTICK TIMER
    if (SysTick_Config(48000))
    {
        while (1)	/* Capture error */
        {
            if (LED)
                LED_OFF;
            else
                LED_ON;

            for (i = 0; i < 255; i++)
            {
                asm (	"nop \n\t"
                        "nop \n\t"
                        "nop \n\t" );
            }
        }
    }

//---------- Pruebas de Hardware --------//    
    // EXTIOff ();
    
    TIM_3_Init();    //Used for mosfet channels control and ADC synchro
#ifdef USE_LED_AS_TIM1_CH3
    TIM_1_Init();
#endif
    // TIM_16_Init();    //free running with tick: 1us
    // TIM16Enable();
    // TIM_17_Init();    //with int, tick: 1us
    MA32Circular_Reset();
    
    CTRL_MOSFET(DUTY_NONE);
    
    //ADC and DMA configuration
    AdcConfig();
    DMAConfig();
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CR |= ADC_CR_ADSTART;
    //end of ADC & DMA

#ifdef HARD_TEST_MODE_DISABLE_PWM
    CTRL_MOSFET(DUTY_100_PERCENT);
    while (1);
#endif
    
#ifdef HARD_TEST_MODE_RECT_SINUSOIDAL
    p_signal = mem_signal;

    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            //aca la senial (el ultimo punto) termina en 0
            if (p_signal < &mem_signal[(SIZEOF_SIGNAL - 1)])
            {
                p_signal++;
            }
            else
            {
                p_signal = mem_signal;
#ifdef USE_LED_FOR_SIGNAL
                if (LED)
                    LED_OFF;
                else
                    LED_ON;
#endif
            }
            CTRL_MOSFET(DUTY_100_PERCENT - *p_signal);
        }
    }
#endif    // HARD_TEST_MODE_RECT_SINUSOIDAL

#ifdef HARD_TEST_MODE_ADC_SENSE
    //disable pwm
    CTRL_MOSFET(DUTY_100_PERCENT);
    
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            // CTRL_LED(Vout_Sense);
            CTRL_LED(Vline_Sense);
        }
    }
#endif
    
#ifdef HARD_TEST_MODE_CONDUCTION_ANGLE
    Hard_Reset_Voltage_Filter();
    
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;

            ii = Hard_Update_Voltage_Sense();

            if (ii)
            {
                ii = 0;
                CTRL_LED(Hard_Get_Conduction_Angle());
                // CTRL_LED(Hard_Get_Hidden_Value());
            }
        }
    }
#endif    // HARD_TEST_MODE_CONDUCTION_ANGLE

#ifdef HARD_TEST_MODE_LINE_SYNC
    Hard_Reset_Voltage_Filter();

    // CTRL_MOSFET(DUTY_10_PERCENT);
    // CTRL_LED(DUTY_50_PERCENT);
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            Hard_Update_Voltage_Filter(Vline_Sense);
        }

        Hard_Update_Voltage_Sense();
    }
#endif
    
#ifdef HARD_TEST_MODE
    ChangeLed(LED_STANDBY);
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            // if (LED)
            //     LED_OFF;
            // else
            //     LED_ON;
            // CTRL_MOSFET(Vbias_Sense);
            // CTRL_MOSFET(Vup);
            // CTRL_MOSFET(I_Sense);
            // CTRL_MOSFET(Iup);
            CTRL_MOSFET(V220_Sense);
        }
        UpdateLed();
    }
#endif
    
    
    //--- Production Program ----------
#ifdef DRIVER_MODE
    Hard_Reset_Voltage_Filter();
    
    while (1)
    {
        switch (driver_state)
        {
        case POWER_UP:
            //TODO: revisar tambien 220V
            // if ((Vbias_Sense > VBIAS_START) && (Vline_Sense > VLINE_START_THRESHOLD))
            if (Vbias_Sense > VBIAS_START)
                driver_state = SOFT_START;
            
            break;

        case SOFT_START:
            if (sequence_ready)
            {
                sequence_ready_reset;
                soft_start_cnt++;

                //reviso no pasarme de corriente de salida
                if (Iup < I_SETPOINT)
                {
                    //reviso no pasarme de tension
                    if (Vup < V_SETPOINT)
                    {
                        //hago un soft start respecto de la corriente y/o tension de salida
                        if (soft_start_cnt > SOFT_START_CNT_ROOF)    //update cada 2ms aprox.
                        {
                            soft_start_cnt = 0;
                    
                            if (d < DUTY_FOR_DMAX)
                            {
                                d++;
                                CTRL_MOSFET(d);
                            }
                            else
                            {
                                ChangeLed(LED_VOLTAGE_MODE);
                                driver_state = VOLTAGE_MODE;
                            }
                        }
                    }
                    else
                    {
                        ChangeLed(LED_VOLTAGE_MODE);
                        driver_state = VOLTAGE_MODE;
                    }
                }                    
                else
                {
                    ChangeLed(LED_CURRENT_MODE);
                    driver_state = CURRENT_MODE;
                }
            }
            break;

        case AUTO_RESTART:
            CTRL_MOSFET(DUTY_NONE);
            d = 0;
            ez1 = 0;
            ez2 = 0;
            ChangeLed(LED_STANDBY);
            driver_state = POWER_UP;
            break;
        
        case VOLTAGE_MODE:
            if (sequence_ready)
            {
                sequence_ready_reset;

                //reviso cambio de modo
                if (Iup < I_SETPOINT)   
                {                
                    d = PID_roof (V_SETPOINT, Vup, d, &ez1, &ez2);
                    if (d > 0)    //d puede tomar valores negativos
                    {
                        if (d > DUTY_FOR_DMAX)
                            d = DUTY_FOR_DMAX;
                    }
                    else
                        d = DUTY_NONE;

                    CTRL_MOSFET(d);
                }
                else     //cambio a lazo I
                {
                    ChangeLed(LED_CURRENT_MODE);
                    driver_state = CURRENT_MODE;
                }

                if (Hard_Update_Voltage_Sense())
                    CTRL_LED(Hard_Get_Conduction_Angle());
            }
            break;

        case CURRENT_MODE:
            if (sequence_ready)
            {
                sequence_ready_reset;

                //reviso cambio de modo
                if (Vup < V_SETPOINT)   
                {                
                    d = PID_roof (I_SETPOINT, Iup, d, &ez1, &ez2);
                    if (d > 0)    //d puede tomar valores negativos
                    {
                        if (d > DUTY_FOR_DMAX)
                            d = DUTY_FOR_DMAX;
                    }
                    else
                        d = DUTY_NONE;

                    CTRL_MOSFET(d);
                }
                else     //cambio a lazo V
                {
                    ChangeLed(LED_VOLTAGE_MODE);
                    driver_state = VOLTAGE_MODE;
                }

                if (Hard_Update_Voltage_Sense())
                    CTRL_LED(Hard_Get_Conduction_Angle());
            }
            break;
            
        case OUTPUT_OVERVOLTAGE:
            if (!timer_standby)
                driver_state = AUTO_RESTART;
            break;

        case INPUT_OVERVOLTAGE:
            if (!timer_standby)
                driver_state = AUTO_RESTART;                
            break;
            
        case OVERCURRENT:
            if (!timer_standby)
                driver_state = AUTO_RESTART;                
            break;

        case BIAS_OVERVOLTAGE:
            if (!timer_standby)
                driver_state = AUTO_RESTART;                
            break;            

        case POWER_DOWN:
            if (!timer_standby)
                driver_state = AUTO_RESTART;                
            break;

        }

        //Cosas que no tienen tanto que ver con las muestras o el estado del programa
        // Hard_Update_Voltage_Sense();
        
#ifdef USE_LED_FOR_MAIN_STATES
        UpdateLed();
#endif
        
    }
    
#endif    // DRIVER_MODE
    
    return 0;
}

//--- End of Main ---//


void TimingDelay_Decrement(void)
{
    if (wait_ms_var)
        wait_ms_var--;

    if (timer_standby)
        timer_standby--;

    // if (take_temp_sample)
    //     take_temp_sample--;

    // if (timer_meas)
    //     timer_meas--;

    if (timer_led)
        timer_led--;

    if (timer_filters)
        timer_filters--;

#ifdef INVERTER_ONLY_SYNC_AND_POLARITY
    if (timer_no_sync)
        timer_no_sync--;
#endif
    
    // //cuenta de a 1 minuto
    // if (secs > 59999)	//pasaron 1 min
    // {
    // 	minutes++;
    // 	secs = 0;
    // }
    // else
    // 	secs++;
    //
    // if (minutes > 60)
    // {
    // 	hours++;
    // 	minutes = 0;
    // }


}

#define AC_SYNC_Int        (EXTI->PR & 0x00000100)
#define AC_SYNC_Set        (EXTI->IMR |= 0x00000100)
#define AC_SYNC_Reset      (EXTI->IMR &= ~0x00000100)
#define AC_SYNC_Ack        (EXTI->PR |= 0x00000100)

#define AC_SYNC_Int_Rising          (EXTI->RTSR & 0x00000100)
#define AC_SYNC_Int_Rising_Set      (EXTI->RTSR |= 0x00000100)
#define AC_SYNC_Int_Rising_Reset    (EXTI->RTSR &= ~0x00000100)

#define AC_SYNC_Int_Falling          (EXTI->FTSR & 0x00000100)
#define AC_SYNC_Int_Falling_Set      (EXTI->FTSR |= 0x00000100)
#define AC_SYNC_Int_Falling_Reset    (EXTI->FTSR &= ~0x00000100)

#define OVERCURRENT_POS_Int        (EXTI->PR & 0x00000010)
#define OVERCURRENT_POS_Ack        (EXTI->PR |= 0x00000010)
#define OVERCURRENT_NEG_Int        (EXTI->PR & 0x00000020)
#define OVERCURRENT_NEG_Ack        (EXTI->PR |= 0x00000020)

void EXTI4_15_IRQHandler(void)
{
#ifdef WITH_AC_SYNC_INT
    if (AC_SYNC_Int)
    {
        if (AC_SYNC_Int_Rising)
        {
            //reseteo tim
            delta_t2 = TIM16->CNT;
            TIM16->CNT = 0;
            AC_SYNC_Int_Rising_Reset;
            AC_SYNC_Int_Falling_Set;

            SYNC_Rising_Edge_Handler();
        }
        else if (AC_SYNC_Int_Falling)
        {
            delta_t1 = TIM16->CNT;
            AC_SYNC_Int_Falling_Reset;
            AC_SYNC_Int_Rising_Set;
            // ac_sync_int_flag = 1;
            
            SYNC_Falling_Edge_Handler();
        }
        AC_SYNC_Ack;
    }
#endif
    
#ifdef WITH_OVERCURRENT_SHUTDOWN
    if (OVERCURRENT_POS_Int)
    {
        HIGH_LEFT(DUTY_NONE);
        //TODO: trabar el TIM3 aca!!!
        overcurrent_shutdown = 1;
        OVERCURRENT_POS_Ack;
    }

    if (OVERCURRENT_NEG_Int)
    {
        HIGH_RIGHT(DUTY_NONE);
        //TODO: trabar el TIM3 aca!!!
        overcurrent_shutdown = 2;
        OVERCURRENT_NEG_Ack;
    }
#endif
}

//------ EOF -------//
