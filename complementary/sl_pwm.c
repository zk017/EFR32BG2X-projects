/***************************************************************************//**
 * @file
 * @brief PWM Driver
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "sl_pwm.h"

#include "em_gpio.h"
#include "em_bus.h"
#include "em_cmu.h"

static CMU_Clock_TypeDef get_timer_clock(TIMER_TypeDef *timer)
{
#if defined(_CMU_HFCLKSEL_MASK) || defined(_CMU_CMD_HFCLKSEL_MASK)
    CMU_Clock_TypeDef timer_clock = cmuClock_HF;
#elif defined(_CMU_SYSCLKCTRL_MASK)
    CMU_Clock_TypeDef timer_clock = cmuClock_SYSCLK;
#else
#error "Unknown root of clock tree"
#endif

    switch ((uint32_t)timer)
    {
#if defined(TIMER0_BASE)
    case TIMER0_BASE:
        timer_clock = cmuClock_TIMER0;
        break;
#endif
#if defined(TIMER1_BASE)
    case TIMER1_BASE:
        timer_clock = cmuClock_TIMER1;
        break;
#endif
#if defined(TIMER2_BASE)
    case TIMER2_BASE:
        timer_clock = cmuClock_TIMER2;
        break;
#endif
#if defined(TIMER3_BASE)
    case TIMER3_BASE:
        timer_clock = cmuClock_TIMER3;
        break;
#endif
#if defined(TIMER4_BASE)
    case TIMER4_BASE:
        timer_clock = cmuClock_TIMER4;
        break;
#endif
#if defined(TIMER5_BASE)
    case TIMER5_BASE:
        timer_clock = cmuClock_TIMER5;
        break;
#endif
#if defined(TIMER6_BASE)
    case TIMER6_BASE:
        timer_clock = cmuClock_TIMER6;
        break;
#endif
#if defined(WTIMER0_BASE)
    case WTIMER0_BASE:
        timer_clock = cmuClock_WTIMER0;
        break;
#endif
#if defined(WTIMER1_BASE)
    case WTIMER1_BASE:
        timer_clock = cmuClock_WTIMER1;
        break;
#endif
#if defined(WTIMER2_BASE)
    case WTIMER2_BASE:
        timer_clock = cmuClock_WTIMER2;
        break;
#endif
#if defined(WTIMER3_BASE)
    case WTIMER3_BASE:
        timer_clock = cmuClock_WTIMER3;
        break;
#endif
    default:
        EFM_ASSERT(0);
        break;
    }
    return timer_clock;
}

sl_status_t sl_pwm_init(sl_pwm_instance_t *pwm, sl_pwm_config_t *config)
{
    CMU_Clock_TypeDef timer_clock = get_timer_clock(pwm->timer);
    CMU_ClockEnable(timer_clock, true);

    // Set PWM pin as output
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinModeSet((GPIO_Port_TypeDef)pwm->port,
                    pwm->pin,
                    gpioModePushPull,
                    config->polarity);

    // Set CC channel parameters
    TIMER_InitCC_TypeDef channel_init = TIMER_INITCC_DEFAULT;
    channel_init.mode = timerCCModePWM;
    channel_init.cmoa = timerOutputActionToggle;
    channel_init.edge = timerEdgeBoth;
    channel_init.outInvert = (config->polarity == PWM_ACTIVE_LOW);
    TIMER_InitCC(pwm->timer, pwm->channel, &channel_init);

    // Configure CC channel pinout
#if defined(_TIMER_ROUTE_MASK)
    BUS_RegMaskedWrite(&pwm->timer->ROUTE,
                       _TIMER_ROUTE_LOCATION_MASK,
                       pwm->location << _TIMER_ROUTE_LOCATION_SHIFT);
#elif defined(_TIMER_ROUTELOC0_MASK)
    BUS_RegMaskedWrite(&pwm->timer->ROUTELOC0,
                       _TIMER_ROUTELOC0_CC0LOC_MASK << (pwm->channel * 8U),
                       pwm->location << (pwm->channel * 8U));
#elif defined(_GPIO_TIMER_ROUTEEN_MASK)
    volatile uint32_t *route_register = &GPIO->TIMERROUTE[TIMER_NUM(pwm->timer)].CC0ROUTE;
    route_register += pwm->channel;
    *route_register = (pwm->port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                      | (pwm->pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
#else
#error "Unknown route setting"
#endif

    // Configure TIMER frequency
    uint32_t top = (CMU_ClockFreqGet(timer_clock) / (config->frequency)) - 1U;
    TIMER_TopSet(pwm->timer, top);

    // Set initial duty cycle to 0%
    TIMER_CompareSet(pwm->timer, pwm->channel, 0U);

    // Initialize TIMER
    TIMER_Init_TypeDef timer_init = TIMER_INIT_DEFAULT;
    TIMER_Init(pwm->timer, &timer_init);

    return SL_STATUS_OK;
}

//--------------------------
#include "log.h"




#define OUT_FREQ 10000

void aaaaa_bbbb(void)
{
	uint32_t top=0;
	  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;


	  //-------------------------------
	  GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 0);
	  GPIO_PinModeSet(gpioPortC, 3, gpioModePushPull, 0);
	  // Don't start counter on initialization
	  timerInit.enable = false;

	  // Configure capture/compare channel for output compare
	  timerCCInit.mode = timerCCModePWM; //timerCCModeCompare;
	  timerCCInit.cofoa = timerOutputActionToggle;
	  timerCCInit.outInvert = PWM_ACTIVE_HIGH;



	  // Route CC0 output to PB4
	  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CDTI0PEN;
	  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortC << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
	                    | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
	  GPIO->TIMERROUTE[0].CDTI0ROUTE = (gpioPortC << _GPIO_TIMER_CDTI0ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CDTI0ROUTE_PIN_SHIFT);
	  TIMER_InitCC(TIMER0, 0, &timerCCInit);

	  //--------------------------
	  TIMER_InitDTI_TypeDef init_DTI = TIMER_INITDTI_DEFAULT;
	  init_DTI.activeLowOut = true;
	  init_DTI.invertComplementaryOut = false;
	  init_DTI.riseTime = 1;
	  init_DTI.fallTime = 1;
	  TIMER_InitDTI(TIMER0, &init_DTI);

	  TIMER_Init(TIMER0, &timerInit);
	  /*
	   * Set the TOP register value.  Each time the counter overflows TOP
	   * is one half of the signal period.
	   */
	  //timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
	  //topValue = timerFreq / (OUT_FREQ) - 1;

	  top = (CMU_ClockFreqGet(cmuClock_TIMER0) / OUT_FREQ) - 1U;
	  TIMER_TopSet(TIMER0, top);

	  TIMER_CompareSet(TIMER0, 0, top * 20 / 100);

	  // Now start the TIMER
	  //TIMER_Enable(TIMER0, true);
}

#if 1
void cccc_dddd(void)
{

		uint32_t top=0;
		  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
		  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;


		  //-------------------------------
		  CMU_Clock_TypeDef timer_clock = cmuClock_TIMER0;
		  CMU_ClockEnable(timer_clock, true);
		  CMU_ClockEnable(cmuClock_GPIO, true);

		  GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 0);
		  GPIO_PinModeSet(gpioPortC, 3, gpioModePushPull, 0);
		  GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0);
		  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
		  // Don't start counter on initialization
		  timerInit.enable = false;

		  // Configure capture/compare channel for output compare
		  timerCCInit.mode = timerCCModePWM; //timerCCModeCompare;
		  timerCCInit.cofoa = timerOutputActionToggle;
		  timerCCInit.outInvert = PWM_ACTIVE_HIGH;



		  // Route CC0 output to PB4
		  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CDTI0PEN|GPIO_TIMER_ROUTEEN_CC1PEN | GPIO_TIMER_ROUTEEN_CDTI1PEN;;
		  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortC << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)| (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
		  GPIO->TIMERROUTE[0].CDTI0ROUTE = (gpioPortC << _GPIO_TIMER_CDTI0ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CDTI0ROUTE_PIN_SHIFT);
		  //-------------------------------------------
		  //GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC1PEN | GPIO_TIMER_ROUTEEN_CDTI1PEN;
		  GPIO->TIMERROUTE[0].CC1ROUTE = (gpioPortC << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)| (1 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
		  GPIO->TIMERROUTE[0].CDTI1ROUTE = (gpioPortA << _GPIO_TIMER_CDTI1ROUTE_PORT_SHIFT) | (0 << _GPIO_TIMER_CDTI1ROUTE_PIN_SHIFT);

		  TIMER_InitCC(TIMER0, 0, &timerCCInit);
		  TIMER_InitCC(TIMER0, 1, &timerCCInit);

		  //--------------------------
		  TIMER_InitDTI_TypeDef init_DTI = TIMER_INITDTI_DEFAULT;
		  init_DTI.activeLowOut = true;
		  init_DTI.invertComplementaryOut = false;
		  init_DTI.riseTime = 1;
		  init_DTI.fallTime = 1;
		  //--------
		  init_DTI.outputsEnableMask =
		          TIMER_DTOGEN_DTOGCC0EN
		          | TIMER_DTOGEN_DTOGCDTI0EN
		          | TIMER_DTOGEN_DTOGCC1EN
		          | TIMER_DTOGEN_DTOGCDTI1EN;
		  TIMER_InitDTI(TIMER0, &init_DTI);

		  TIMER_Init(TIMER0, &timerInit);
		  /*
		   * Set the TOP register value.  Each time the counter overflows TOP
		   * is one half of the signal period.
		   */
		  //timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
		  //topValue = timerFreq / (OUT_FREQ) - 1;

		  top = (CMU_ClockFreqGet(cmuClock_TIMER0) / OUT_FREQ) - 1U;
		  TIMER_TopSet(TIMER0, top);

		  TIMER_CompareSet(TIMER0, 0, top * 20 / 100);
		  TIMER_CompareSet(TIMER0, 1, top * 40 / 100);
		  // Now start the TIMER
		  TIMER_Enable(TIMER0, true);

}
#else
void cccc_dddd(void)  //@zk20240906--use here code .
{

		uint32_t top=0;
		  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
		  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;


		  //-------------------------------
		  CMU_Clock_TypeDef timer_clock = cmuClock_TIMER1;
		  CMU_ClockEnable(timer_clock, true);
		  CMU_ClockEnable(cmuClock_GPIO, true);

		  GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0);
		  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
		  // Don't start counter on initialization
		  timerInit.enable = false;

		  // Configure capture/compare channel for output compare
		  timerCCInit.mode = timerCCModePWM; //timerCCModeCompare;
		  timerCCInit.cofoa = timerOutputActionToggle;
		  timerCCInit.outInvert = PWM_ACTIVE_HIGH;



		  // Route CC0 output to PB4
		  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CDTI0PEN;
		  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortC << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)| (1 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
		  GPIO->TIMERROUTE[0].CDTI0ROUTE = (gpioPortA << _GPIO_TIMER_CDTI0ROUTE_PORT_SHIFT) | (0 << _GPIO_TIMER_CDTI0ROUTE_PIN_SHIFT);

		  TIMER_InitCC(TIMER1, 0, &timerCCInit);

		  //--------------------------
		  TIMER_InitDTI_TypeDef init_DTI = TIMER_INITDTI_DEFAULT;
		  init_DTI.activeLowOut = true;
		  init_DTI.invertComplementaryOut = false;
		  init_DTI.riseTime = 1;
		  init_DTI.fallTime = 1;
		  TIMER_InitDTI(TIMER1, &init_DTI);

		  TIMER_Init(TIMER1, &timerInit);
		  /*
		   * Set the TOP register value.  Each time the counter overflows TOP
		   * is one half of the signal period.
		   */
		  //timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
		  //topValue = timerFreq / (OUT_FREQ) - 1;

		  top = (CMU_ClockFreqGet(cmuClock_TIMER1) / OUT_FREQ) - 1U;
		  TIMER_TopSet(TIMER1, top);

		  TIMER_CompareSet(TIMER1, 0, top * 20 / 100);

		  // Now start the TIMER
		  TIMER_Enable(TIMER1, true);

}
#endif
#if 0 //OK
void dddd_eeee(void)
{

	uint32_t top=0;
		  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
		  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;


		  //-------------------------------

		  GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0);
		  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
		  // Don't start counter on initialization
		  timerInit.enable = false;

		  // Configure capture/compare channel for output compare
		  timerCCInit.mode = timerCCModePWM; //timerCCModeCompare;
		  timerCCInit.cofoa = timerOutputActionToggle;
		  timerCCInit.outInvert = PWM_ACTIVE_HIGH;



		  // Route CC0 output to PB4
		  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CDTI0PEN;
		  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortC << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
		                    | (1 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
		  GPIO->TIMERROUTE[0].CDTI0ROUTE = (gpioPortA << _GPIO_TIMER_CDTI0ROUTE_PORT_SHIFT) | (0 << _GPIO_TIMER_CDTI0ROUTE_PIN_SHIFT);
		  TIMER_InitCC(TIMER0, 0, &timerCCInit);

		  //--------------------------
		  TIMER_InitDTI_TypeDef init_DTI = TIMER_INITDTI_DEFAULT;
		  init_DTI.activeLowOut = true;
		  init_DTI.invertComplementaryOut = false;
		  init_DTI.riseTime = 1;
		  init_DTI.fallTime = 1;
		  TIMER_InitDTI(TIMER0, &init_DTI);

		  TIMER_Init(TIMER0, &timerInit);
		  /*
		   * Set the TOP register value.  Each time the counter overflows TOP
		   * is one half of the signal period.
		   */
		  //timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
		  //topValue = timerFreq / (OUT_FREQ) - 1;

		  top = (CMU_ClockFreqGet(cmuClock_TIMER0) / OUT_FREQ) - 1U;
		  TIMER_TopSet(TIMER0, top);

		  TIMER_CompareSet(TIMER0, 0, top * 20 / 100);

		  // Now start the TIMER
		  TIMER_Enable(TIMER0, true);
}
#endif
void dddd_eeee(void)
{

	uint32_t top=0;
		  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
		  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;


		  //-------------------------------
		  CMU_Clock_TypeDef timer_clock = cmuClock_TIMER0;
		  CMU_ClockEnable(timer_clock, true);
		  CMU_ClockEnable(cmuClock_GPIO, true);

		  GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0);
		  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
		  // Don't start counter on initialization
		  timerInit.enable = false;

		  // Configure capture/compare channel for output compare
		  timerCCInit.mode = timerCCModePWM; //timerCCModeCompare;
		  timerCCInit.cofoa = timerOutputActionToggle;
		  timerCCInit.outInvert = PWM_ACTIVE_HIGH;



		  // Route CC0 output to PB4
		  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CDTI0PEN|GPIO_TIMER_ROUTEEN_CC1PEN | GPIO_TIMER_ROUTEEN_CDTI1PEN;;
		  GPIO->TIMERROUTE[0].CC1ROUTE = (gpioPortC << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)| (1 << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
		  GPIO->TIMERROUTE[0].CDTI1ROUTE = (gpioPortA << _GPIO_TIMER_CDTI1ROUTE_PORT_SHIFT) | (0 << _GPIO_TIMER_CDTI1ROUTE_PIN_SHIFT);
		  TIMER_InitCC(TIMER0, 1, &timerCCInit);

		  //--------------------------
		  TIMER_InitDTI_TypeDef init_DTI = TIMER_INITDTI_DEFAULT;
		  init_DTI.activeLowOut = true;
		  init_DTI.invertComplementaryOut = false;
		  init_DTI.riseTime = 1;
		  init_DTI.fallTime = 1;
		  TIMER_InitDTI(TIMER0, &init_DTI);

		  TIMER_Init(TIMER0, &timerInit);
		  /*
		   * Set the TOP register value.  Each time the counter overflows TOP
		   * is one half of the signal period.
		   */
		  //timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
		  //topValue = timerFreq / (OUT_FREQ) - 1;

		  top = (CMU_ClockFreqGet(cmuClock_TIMER0) / OUT_FREQ) - 1U;
		  TIMER_TopSet(TIMER0, top);

		  TIMER_CompareSet(TIMER0, 1, top * 20 / 100);

		  // Now start the TIMER
		  TIMER_Enable(TIMER0, true);
}

#if 1
void init_h2_l2(void)
{
	uint32_t top=0;
		  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
		  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;


		  //-------------------------------
		  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
		  GPIO_PinModeSet(gpioPortA, 5, gpioModePushPull, 0);
		  // Don't start counter on initialization
		  timerInit.enable = false;

		  // Configure capture/compare channel for output compare
		  timerCCInit.mode = timerCCModePWM; //timerCCModeCompare;
		  timerCCInit.cofoa = timerOutputActionToggle;
		  timerCCInit.outInvert = PWM_ACTIVE_HIGH;



		  // Route CC0 output to PB4
		  GPIO->TIMERROUTE[1].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CDTI0PEN;
		  GPIO->TIMERROUTE[1].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
		                    | (3 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
		  GPIO->TIMERROUTE[1].CDTI0ROUTE = (gpioPortA << _GPIO_TIMER_CDTI0ROUTE_PORT_SHIFT) | (5 << _GPIO_TIMER_CDTI0ROUTE_PIN_SHIFT);
		  TIMER_InitCC(TIMER0, 0, &timerCCInit);

		  //--------------------------
		  TIMER_InitDTI_TypeDef init_DTI = TIMER_INITDTI_DEFAULT;
		  init_DTI.activeLowOut = true;
		  init_DTI.invertComplementaryOut = false;
		  init_DTI.riseTime = 1;
		  init_DTI.fallTime = 1;
		  TIMER_InitDTI(TIMER0, &init_DTI);

		  TIMER_Init(TIMER0, &timerInit);
		  /*
		   * Set the TOP register value.  Each time the counter overflows TOP
		   * is one half of the signal period.
		   */
		  //timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
		  //topValue = timerFreq / (OUT_FREQ) - 1;

		  top = (CMU_ClockFreqGet(cmuClock_TIMER0) / OUT_FREQ) - 1U;
		  TIMER_TopSet(TIMER0, top);

		  TIMER_CompareSet(TIMER0, 0, top * 20 / 100);

		  // Now start the TIMER
		  TIMER_Enable(TIMER0, true);
}
#endif

void my_setH1_L1_duty(uint8_t duty)
{
	uint32_t top = 0;
	if(duty>=100) duty = 100;
	top = (CMU_ClockFreqGet(cmuClock_TIMER0) / OUT_FREQ) - 1U;
	TIMER_TopSet(TIMER0, top);
	TIMER_CompareSet(TIMER0, 0, top * duty / 100);
	TIMER_CompareSet(TIMER0, 1, top * (100-duty) / 100);
}


#if 0 //this OK.
#define OUT_FREQ 10000
void aaaaa_bbbb(void)
{
	uint32_t top=0;
	  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;


	  //-------------------------------
	  GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 0);
	  // Don't start counter on initialization
	  //timerInit.enable = false;

	  // Configure capture/compare channel for output compare
	  timerCCInit.mode = timerCCModePWM; //timerCCModeCompare;
	  timerCCInit.cofoa = timerOutputActionToggle;
	  timerCCInit.outInvert = PWM_ACTIVE_HIGH;



	  // Route CC0 output to PB4
	  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CDTI0PEN;
	  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortC << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
	                    | (2 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

	  TIMER_InitCC(TIMER0, 0, &timerCCInit);

	  //--------------------------
	  GPIO_PinModeSet(gpioPortC, 3, gpioModePushPull, 0);
	  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CDTI0PEN;
	  GPIO->TIMERROUTE[0].CDTI0ROUTE = (gpioPortC << _GPIO_TIMER_CDTI0ROUTE_PORT_SHIFT) | (3 << _GPIO_TIMER_CDTI0ROUTE_PIN_SHIFT);

	  TIMER_InitDTI_TypeDef init_DTI = TIMER_INITDTI_DEFAULT;
	  init_DTI.activeLowOut = true;
	  init_DTI.invertComplementaryOut = false;
	  init_DTI.riseTime = 1;
	  init_DTI.fallTime = 1;
	  TIMER_InitDTI(TIMER0, &init_DTI);

	  TIMER_Init(TIMER0, &timerInit);
	  /*
	   * Set the TOP register value.  Each time the counter overflows TOP
	   * is one half of the signal period.
	   */
	  //timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
	  //topValue = timerFreq / (OUT_FREQ) - 1;

	  top = (CMU_ClockFreqGet(cmuClock_TIMER0) / OUT_FREQ) - 1U;
	  TIMER_TopSet(TIMER0, top);

	  TIMER_CompareSet(TIMER0, 0, top * 20 / 100);

	  // Now start the TIMER
	  TIMER_Enable(TIMER0, true);
}

#endif


//---------------
sl_status_t sl_pwm_deinit(sl_pwm_instance_t *pwm)
{
    // Reset TIMER routes
    sl_pwm_stop(pwm);

#if defined(_TIMER_ROUTE_MASK)
    BUS_RegMaskedClear(&pwm->timer->ROUTE, _TIMER_ROUTE_LOCATION_MASK);
#elif defined(_TIMER_ROUTELOC0_MASK)
    BUS_RegMaskedClear(&pwm->timer->ROUTELOC0, _TIMER_ROUTELOC0_CC0LOC_MASK << (pwm->channel * 8));
#elif defined(_GPIO_TIMER_ROUTEEN_MASK)
    volatile uint32_t *route_register = &GPIO->TIMERROUTE[TIMER_NUM(pwm->timer)].CC0ROUTE;
    route_register += pwm->channel;
    *route_register = 0;
#else
#error "Unknown route setting"
#endif

    // Reset TIMER
    TIMER_Reset(pwm->timer);

    // Reset GPIO
    GPIO_PinModeSet((GPIO_Port_TypeDef)pwm->port,
                    pwm->pin,
                    gpioModeDisabled,
                    0);

    CMU_Clock_TypeDef timer_clock = get_timer_clock(pwm->timer);
    CMU_ClockEnable(timer_clock, false);

    return SL_STATUS_OK;
}

void sl_pwm_start(sl_pwm_instance_t *pwm)
{
    // Enable PWM output
#if defined(_TIMER_ROUTE_MASK)
    BUS_RegMaskedSet(&pwm->timer->ROUTE,
                     1 << (pwm->channel + _TIMER_ROUTE_CC0PEN_SHIFT));
#elif defined(_TIMER_ROUTELOC0_MASK)
    BUS_RegMaskedSet(&pwm->timer->ROUTEPEN,
                     1 << (pwm->channel + _TIMER_ROUTEPEN_CC0PEN_SHIFT));
#elif defined(_GPIO_TIMER_ROUTEEN_MASK)
    GPIO->TIMERROUTE_SET[TIMER_NUM(pwm->timer)].ROUTEEN = 1 << (pwm->channel + _GPIO_TIMER_ROUTEEN_CC0PEN_SHIFT);
#else
#error "Unknown route setting"
#endif
}

void sl_pwm_stop(sl_pwm_instance_t *pwm)
{
    // Disable PWM output
#if defined(_TIMER_ROUTE_MASK)
    BUS_RegMaskedClear(&pwm->timer->ROUTE,
                       1 << (pwm->channel + _TIMER_ROUTE_CC0PEN_SHIFT));
#elif defined(_TIMER_ROUTELOC0_MASK)
    BUS_RegMaskedClear(&pwm->timer->ROUTEPEN,
                       1 << (pwm->channel + _TIMER_ROUTEPEN_CC0PEN_SHIFT));
#elif defined(_GPIO_TIMER_ROUTEEN_MASK)
    GPIO->TIMERROUTE_CLR[TIMER_NUM(pwm->timer)].ROUTEEN = 1 << (pwm->channel + _GPIO_TIMER_ROUTEEN_CC0PEN_SHIFT);
#else
#error "Unknown route setting"
#endif

    // Keep timer running in case other channels are in use
}

void sl_pwm_set_duty_cycle(sl_pwm_instance_t *pwm, uint8_t percent)
{
    uint32_t top = TIMER_TopGet(pwm->timer);

    // Set compare value
    TIMER_CompareBufSet(pwm->timer, pwm->channel, (top * percent) / 100);
}

uint8_t sl_pwm_get_duty_cycle(sl_pwm_instance_t *pwm)
{
    uint32_t top = TIMER_TopGet(pwm->timer);
    uint32_t compare = TIMER_CaptureGet(pwm->timer, pwm->channel);

    uint8_t percent = (uint8_t)((compare * 100) / top);

    return percent;
}

//--------------------------------------------
void toggle_cc0_polarity(sl_pwm_instance_t *pwm, bool toggle)
{
	//static uint8_t g_L1_polarity_counts = 0, g_L2_polarity_counts=0;

	if(toggle==true) //change polarity
	{
		pwm->timer->CC[0].CTRL |= 0x04;
	}
	else //shouldnt change
	{
		pwm->timer->CC[0].CTRL &= 0xFFFFFFFB;
	}
}

//--------------------------------------------
void toggle_cc1_polarity(sl_pwm_instance_t *pwm, bool toggle)
{
	//static uint8_t g_L1_polarity_counts = 0, g_L2_polarity_counts=0;

	if(toggle==true) //change polarity
	{
		pwm->timer->CC[1].CTRL |= 0x04;
	}
	else //shouldnt change
	{
		pwm->timer->CC[1].CTRL &= 0xFFFFFFFB;
	}
}



