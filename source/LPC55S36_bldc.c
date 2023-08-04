/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    LPC55S36_bldc.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "freemaster.h"
#include "pin_mux.h"
#include "peripherals.h"
#include "board.h"
#include "clock_config.h"
#include "LPC55S36.h"
#include "fsl_debug_console.h"
#include "fsl_common_arm.h"
#include "fsl_common.h"
#include "fsl_power.h"
#include "fsl_lpadc.h"
#include "fsl_inputmux.h"
#include "freemaster_serial_miniusart.h"
#include "fsl_device_registers.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

#define LED_RED_SET 				GPIO->CLR[1]|=led_red
#define LED_RED_CLEAR 				GPIO->SET[1]|=led_red
#define LED_GREEN_SET 				GPIO->SET[0]|=led_green
#define LED_GREEN_CLEAR 			GPIO->CLR[0]|=led_green
#define HALL_A_DETECTED 			GPIO->W[0][13]==0xffffffff
#define HALL_B_DETECTED 			GPIO->W[0][14]==0xffffffff
#define HALL_C_DETECTED 			GPIO->W[1][11]==0xffffffff
#define GPIO_1_21_SET 				GPIO->SET[1]|=0x00200000
#define GPIO_1_21_CLEAR 			GPIO->CLR[1]|=0x00200000
#define GPIO_1_21_TOGGLE			GPIO->NOT[1]|=0x00200000
#define PWM0_INIT_150M_10K			(int16_t)(-7500);
#define PWM0_VAL1_150M_10K			(int16_t)(7499);
#define PWM_OUT_ENABLE				PWM0->OUTEN=0x0FF0;
#define PWM_OUT_DISABLE				PWM0->OUTEN=0x0000;
#define UDCBUS_SCALE				4.59900e-4F
#define UDCBUS_HIGH					16.0F
#define UDCBUS_LOW					8.0F


#define M1_END_OF_ISR \
    {                 \
        __DSB();      \
        __ISB();      \
    }
#define SYSTICK_START_COUNT() 		(SysTick->VAL = SysTick->LOAD)
#define SYSTICK_STOP_COUNT(par1)   \
    uint32_t val  = SysTick->VAL;  \
    uint32_t load = SysTick->LOAD; \
    par1          = load - val


const uint32_t 	led_red					=0x10000000;
const uint32_t 	led_green				=0x00400000;
const uint16_t 	UdcBus_high				=0x00003A98;
const uint16_t 	UdcBus_low				=0x000025E0;
uint32_t 		sector_old				=0;
uint32_t 		sector					=0;
uint32_t 		SlowLoopCnt				=0;
uint32_t 		SW3Cnt					=0;
uint16_t 		UdcBus_Frac16			=0;
uint16_t 		IdcBus_Frac16			=0;
uint32_t 		SpeedCnt[7]				={0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};
uint16_t		Duty					=0x2000;
uint32_t 		g_ui32NumberOfCycles    = 0U;
uint32_t 		g_ui32MaxNumberOfCycles = 0U;
uint32_t 		Ctimer1_old			 	= 0U;
uint32_t 		cmtflag				 	= 0U;
float			Speed_User_Cmd			=1000.0f;

/* Commutation table */
uint16_t SectorCtrlCW[14]=
					{
							0x0000,0x0000,
							0x0330,0x0004,//1
							0x0550,0x0400,//2
							0x0660,0x0400,//3
							0x0660,0x0040,//4
							0x0550,0x0004,//5
							0x0330,0x0040,//6
					};
uint16_t SectorCtrlCCW[14]=
					{
							0x0000,0x0000,
							0x0330,0x0040,//1
							0x0550,0x0004,//2
							0x0660,0x0040,//3
							0x0660,0x0400,//4
							0x0550,0x0400,//5
							0x0330,0x0004,//6
					};

struct
{
	float		SpeedMeasure;
	float		SpeedCmd;
	float		SpeedRampUp;
	float		SpeedRampDown;
	float		SpeedRamp;
	float		UserSpeedCmd;
	float		SpeedErr;
	float		PI_P;
	float		PI_I;
	float		PI_Lim;
	float		PI_Pout;
	float		PI_Iout;
	float		PI_PIout;
	float		UdcBus_Float;
	int16_t		Direction;
} BldcCtl;

void InitCTIMER0(void);
void InitCTIMER1(void);
void InitFlexPWM(void);
void InitADC(void);
void InitDAC(void);
void InitHscmp(void);
void InitPINT(void);
void InitGINT(void);
void SetPwmDuty(float);
void SetPwmSector(uint16_t);
void Communication(void);
uint16_t ReadHall(void);

enum
{
    AppFault = 0,
    AppInit  = 1,
    AppStop  = 2,
    AppRun   = 3,
} AppState;

/*
 * @brief   Application entry point.
 */
void main(void)
{
    uint32_t ui32PrimaskReg;
    ui32PrimaskReg = DisableGlobalIRQ();

    /* Init board hardware. */
    BOARD_InitBootClocks();//generate by ConfigTolls
    BOARD_InitBootPins();//generate by ConfigTolls
    BOARD_InitPeripherals();

    SYSCON->AHBCLKCTRLSET[0]		|=SYSCON_AHBCLKCTRL0_MUX_MASK;
    INPUTMUX_AttachSignal(INPUTMUX, 0U, kINPUTMUX_Ctimer0M3ToAdc0Trigger);//CTIMER0 trigger ADC 1ms a time
    INPUTMUX_AttachSignal(INPUTMUX, 0U, kINPUTMUX_Hscmp0OutToPwm0FaultTrigger);//hardware protect lock PWM
    //BOARD_InitBootPeripherals();

    /* Init systcik to calculate code efficiency*/
    SysTick->LOAD = 0xFFFFFF;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    InitCTIMER0();//1kHz control loop
    InitCTIMER1();//record communication time
    InitFlexPWM();//PWM generator
    InitADC();//sample DC voltage
    InitDAC();//DC current hardware protect
    InitPINT();//generate communication interrupt from GPIO
    InitGINT();//control state machine through button

#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    //BOARD_InitDebugConsole();
#endif

    AppState=AppInit;
	GINT0->PORT_ENA[0]				|=GINT_PORT_ENA_ENA17_MASK;//enable GINT from Pin0_17
    EnableGlobalIRQ(ui32PrimaskReg);

    SetPwmDuty(0.0f);

    /* Enter an infinite loop, just incrementing a counter. */
    while(1)
    {
        /* FreeMASTER Polling function */
        FMSTR_Poll();
    }
}

void ADC0_IRQHandler(void)
{
	SYSTICK_START_COUNT();
	UdcBus_Frac16					=(uint16_t)(0x0000FFFF&ADC0->RESFIFO[0]);//read conversion value
	BldcCtl.UdcBus_Float			=(float)(UdcBus_Frac16*UDCBUS_SCALE);//trans to float

    if(BldcCtl.UdcBus_Float<UDCBUS_LOW || BldcCtl.UdcBus_Float>UDCBUS_HIGH)
    {
    	AppState=AppFault;
    	LED_GREEN_CLEAR;
    	PWM_OUT_DISABLE;
    }//bus voltage protect

    if(AppState==AppInit)//initial control parameter
    {
    	BldcCtl.SpeedMeasure		=0;
    	BldcCtl.SpeedCmd			=0;
    	BldcCtl.SpeedErr			=0;
    	BldcCtl.PI_P				=1.2e-4f;
    	BldcCtl.PI_I				=1e-5;
    	BldcCtl.PI_Lim				=0.9f;
    	BldcCtl.PI_Pout				=0;
    	BldcCtl.PI_Iout				=0;
    	BldcCtl.PI_PIout			=0;
    	BldcCtl.UdcBus_Float		=0;
    	BldcCtl.SpeedRampUp			=50.0f;
    	BldcCtl.SpeedRampDown		=50.0f;
    	BldcCtl.UserSpeedCmd		=Speed_User_Cmd;
    	ReadHall();
    	SetPwmSector(0U);
    	AppState=AppStop;
    }

    else if(AppState==AppStop ||AppState==AppFault)//initial control parameter
    {
    	BldcCtl.SpeedMeasure		=0;
    	BldcCtl.SpeedCmd			=0;
    	BldcCtl.SpeedErr			=0;
    	cmtflag						=1U;
    	PWM_OUT_DISABLE	;
    }

    else if(AppState==AppRun)
    {
    	if(cmtflag==1U)
    	{
    		SetPwmSector(ReadHall());
    		cmtflag=0U;
    	}
    	BldcCtl.SpeedMeasure		=(22500000.0f)/(SpeedCnt[1]+SpeedCnt[2]+SpeedCnt[3]+SpeedCnt[4]+SpeedCnt[5]+SpeedCnt[6]);//calculate speed(rpm)

    	if(BldcCtl.UserSpeedCmd<0)BldcCtl.SpeedCmd=-BldcCtl.UserSpeedCmd;
    	else BldcCtl.SpeedCmd			=BldcCtl.UserSpeedCmd;

    	if(BldcCtl.SpeedRamp<(BldcCtl.SpeedCmd-BldcCtl.SpeedRampUp))BldcCtl.SpeedRamp+=BldcCtl.SpeedRampUp;
    	else if(BldcCtl.SpeedRamp>(BldcCtl.SpeedCmd+BldcCtl.SpeedRampDown))BldcCtl.SpeedRamp-=BldcCtl.SpeedRampDown;
    	else BldcCtl.SpeedRamp=BldcCtl.SpeedCmd;

    	BldcCtl.SpeedErr			=BldcCtl.SpeedRamp-BldcCtl.SpeedMeasure;

    	BldcCtl.PI_Pout				=BldcCtl.SpeedErr*BldcCtl.PI_P;
    	BldcCtl.PI_Iout				=BldcCtl.SpeedErr*BldcCtl.PI_I+BldcCtl.PI_Iout;

    	if(BldcCtl.PI_Pout>BldcCtl.PI_Lim)BldcCtl.PI_Pout=BldcCtl.PI_Lim;
    	else if(BldcCtl.PI_Pout<0.1f)BldcCtl.PI_Pout=0.1f;
    	if(BldcCtl.PI_Iout>BldcCtl.PI_Lim)BldcCtl.PI_Iout=BldcCtl.PI_Lim;
    	else if(BldcCtl.PI_Iout<0.1f)BldcCtl.PI_Iout=0.1f;

    	BldcCtl.PI_PIout			=BldcCtl.PI_Pout+BldcCtl.PI_Iout;

    	if(BldcCtl.PI_PIout>BldcCtl.PI_Lim)BldcCtl.PI_PIout=BldcCtl.PI_Lim;
    	else if(BldcCtl.PI_PIout<0.1f)BldcCtl.PI_PIout=0.1f;

    	if(BldcCtl.Direction == -1)BldcCtl.SpeedMeasure=-BldcCtl.SpeedMeasure;

    	SetPwmDuty(BldcCtl.PI_PIout);
    }

    SYSTICK_STOP_COUNT(g_ui32NumberOfCycles);
      g_ui32MaxNumberOfCycles =
          g_ui32NumberOfCycles > g_ui32MaxNumberOfCycles ? g_ui32NumberOfCycles : g_ui32MaxNumberOfCycles;

    /* Call FreeMASTER recorder */
    FMSTR_Recorder(0);

    ADC0->STAT |= (uint32_t)(1U << 9);
    M1_END_OF_ISR;
}

void PIN_INT0_IRQHandler(void)
{
	Communication();
    PINT->IST 					|= 0x00000001;//clear PINT0_0 flag
    PINT->FALL 					|= 0x00000001;//clear PINT0_0 falling edge flag
    PINT->RISE 					|= 0x00000001;//clear PINT0_0 rising edge flag
    M1_END_OF_ISR;
}

void PIN_INT1_IRQHandler(void)
{
	Communication();
    PINT->IST 					|= 0x00000002;//clear PINT0_1 flag
    PINT->FALL 					|= 0x00000002;//clear PINT0_1 falling edge flag
    PINT->RISE 					|= 0x00000002;//clear PINT0_1 rising edge flag
    M1_END_OF_ISR;
}

void PIN_INT2_IRQHandler(void)
{
	Communication();
    PINT->IST 					|= 0x00000004;//clear PINT0_2 flag
    PINT->FALL 					|= 0x00000004;//clear PINT0_2 falling edge flag
    PINT->RISE 					|= 0x00000004;//clear PINT0_2 rising edge flag
    M1_END_OF_ISR;
}

void GINT0_IRQHandler(void)
{
	SW3Cnt++;
	PRINTF("GINT0 event detected %d\r\n",(uint32_t)(SW3Cnt));
	PRINTF("UdcBus_Frac16=%d\r\n",(uint32_t)(UdcBus_Frac16));
	switch (AppState)
	{
	case AppInit:
		AppState=AppStop;
		PWM_OUT_DISABLE;
		break;
	case AppStop:
		AppState=AppRun;
		Communication();
		LED_GREEN_SET;
		PWM_OUT_ENABLE;
		break;
	case AppRun:
		AppState=AppStop;
    	BldcCtl.SpeedMeasure		=0;
    	BldcCtl.SpeedErr			=0;
    	BldcCtl.PI_Pout				=0;
    	BldcCtl.PI_Iout				=0;
    	BldcCtl.PI_PIout			=0;
		LED_GREEN_CLEAR;
		PWM_OUT_DISABLE;
		break;
	default:
		AppState=AppStop;
		PWM_OUT_DISABLE;
		break;
	}

	GINT0->CTRL					|=GINT_CTRL_INT_MASK;
    M1_END_OF_ISR;
}

void InitCTIMER0(void)
{

	SYSCON->AHBCLKCTRLSET[1]	|=SYSCON_AHBCLKCTRL1_TIMER0_MASK;//enable CTIMER0 interface clock in SYSCON
    CTIMER0->MCR				=CTIMER_MCR_MR3R_MASK;//reset when match
    CTIMER0->MR[3]				=(uint32_t)(CLOCK_GetFreq(kCLOCK_FroHf)/(1000U));//1kHz
    CTIMER0->PWMC				|=CTIMER_PWMC_PWMEN3_MASK;//enable trigger of MATCH[3]
	CTIMER0->TCR				|=CTIMER_TCR_CEN_MASK;//enable CTIMER0
}

void InitCTIMER1(void)
{
	SYSCON->AHBCLKCTRLSET[1]	=SYSCON_AHBCLKCTRL1_TIMER1_MASK;//enable CTIMER1 interface clock in SYSCON
	CTIMER1->PR					=0;
	CTIMER1->TCR				|=CTIMER_TCR_CEN_MASK;//enable CTIMER1
}

void InitFlexPWM(void)
{
	SYSCON->PWM0SUBCTL 			=(SYSCON_PWM0SUBCTL_CLK0_EN_MASK | SYSCON_PWM0SUBCTL_CLK1_EN_MASK
								| SYSCON_PWM0SUBCTL_CLK2_EN_MASK | SYSCON_PWM0SUBCTL_CLK3_EN_MASK);//enable FlexPWM submodule 0~3 function clock source in SYSCON
	SYSCON->AHBCLKCTRLSET[3]	|=SYSCON_AHBCLKCTRL3_PWM0_MASK;//enable interface clock of FlexPWM

    for(int i=0;i<3;i++)
    {
    	PWM0->SM[i].INIT 		= PWM0_INIT_150M_10K;
    	PWM0->SM[i].VAL1 		= PWM0_VAL1_150M_10K;
    	PWM0->SM[i].VAL2 		= (uint16_t)(-3750);//edge-align PWM
    	PWM0->SM[i].VAL3 		= (uint16_t)(3749);//initial PWM duty 0.5
    	PWM0->SM[i].DTCNT0 		= (int16_t)(150);//1us dead time on rising edge
    	PWM0->SM[i].DTCNT1 		= (int16_t)(150);//1us dead time on falling edge
    	PWM0->SM[i].DISMAP[0] 	= 0xF111U;//PWM output in fault status
    	PWM0->SM[i].CTRL2		|=0x0010;
    }

    //PWM0->SM[0].VAL4 			= PWM0_INIT_150M_10K;
    //PWM0->SM[0].TCTRL 			|= PWM_TCTRL_OUT_TRIG_EN(1 << 4);

    PWM0->FSTS 					= (PWM0->FSTS & ~PWM_FSTS_FFULL_MASK) | PWM_FSTS_FFULL(0x1);//PWM outputs are re-enabled at the start of a full cycle
    PWM0->FFILT 				= (PWM0->FFILT & ~PWM_FFILT_FILT_PER_MASK) | PWM_FFILT_FILT_PER(2);//Fault Filter Period
    PWM0->FCTRL 				=0xF0F0;//A logic 1 on the fault input indicates a fault condition.
    PWM0->FSTS 					|=0x000F;//Cause a simulated fault

    PWM0->MCTRL 				|=((PWM0->MCTRL & ~PWM_MCTRL_CLDOK_MASK) | PWM_MCTRL_CLDOK(0xF));//Clear Load Okay
    PWM0->MCTRL 				|=((PWM0->MCTRL & ~PWM_MCTRL_LDOK_MASK) | PWM_MCTRL_LDOK(0xF));//Load Okay
    PWM0->MCTRL 				|=((PWM0->MCTRL & ~PWM_MCTRL_RUN_MASK) | PWM_MCTRL_RUN(0xF));//enable submodule 0~3

    PWM_OUT_DISABLE;
}

void InitADC(void)
{
	SYSCON->ADC0CLKDIV			|=SYSCON_ADC0CLKDIV_RESET_MASK;//Reset ADC clock
	SYSCON->ADC0CLKDIV			=0x00000001;//divide ADC0 clock by 2, ADC0 clock no more than 50MHz
	SYSCON->ADC0CLKSEL			=0x00000002;//ADC0 clock FRO 96MHz
	POWER_DisablePD(kPDRUNCFG_PD_VREF); // Disable VREF power down */
	SYSCON->AHBCLKCTRLSET[0]	|=SYSCON_AHBCLKCTRL0_ADC0_MASK;//enable ADC0 interface clock in SYSCON

    ADC0->CTRL 					|= ADC_CTRL_RST_MASK;
    ADC0->CTRL 					&= ~ADC_CTRL_RST_MASK;//Resets all internal logic and registers, except the Control Register. Remains set until cleared by software.
    ADC0->CTRL 					|= ADC_CTRL_RSTFIFO0_MASK;//Reset FIFO 0
    ADC0->CTRL 					|= ADC_CTRL_RSTFIFO1_MASK;//Reset FIFO 0
    ADC0->CTRL 					&= ~ADC_CTRL_ADCEN_MASK;//ADC enable
    ADC0->CTRL 					&= ~ADC_CTRL_DOZEN_MASK;//Doze enable
    ADC0->CTRL 					&= ~ADC_CTRL_CAL_AVGS_MASK;//single conversion
    ADC0->CFG 					=ADC_CFG_PWREN_MASK|ADC_CFG_PUDLY(0x80)|ADC_CFG_PWRSEL_MASK ;//ADC Analog Pre-Enable;power up delay 512 ADCK;High power setting.
    ADC0->PAUSE					=0x0000000;//No pause
    ADC0->FCTRL[0] 				&= ~ADC_FCTRL_FWMARK_MASK;
    ADC0->FCTRL[1] 				&= ~ADC_FCTRL_FWMARK_MASK;
    ADC0->CTRL 					|= ADC_CTRL_ADCEN_MASK;//enable ADC0

    LPADC_DoOffsetCalibration(ADC0);
    LPADC_DoAutoCalibration(ADC0);//Do auto calibration

    ADC0->CMD[0].CMDL			=0x00000000|ADC_CMDL_ADCH(1U)|ADC_CMDL_CTYPE(kLPADC_SampleChannelSingleEndSideA);//DC voltage; single-end sample
    ADC0->CMD[0].CMDH			=0x00000000|ADC_CMDH_AVGS(kLPADC_HardwareAverageCount16)//16 conversions averaged
    										|ADC_CMDH_STS(kLPADC_SampleTimeADCK11);//11.5 ADCK cycles total sample tim
    ADC0->TCTRL[0]				=0x00000000|ADC_TCTRL_TCMD(1U)|ADC_TCTRL_HTEN_MASK;//hardware trigger CMD1
    ADC0->IE					=0x00000000|ADC_IE_TCOMP_IE_MASK;//Trigger completion interrupts are enabled for every trigger source.

    NVIC_SetPriority(ADC0_IRQn, 255U);
    NVIC_EnableIRQ(ADC0_IRQn);
}

void InitDAC(void)
{
	SYSCON->DAC[0].CLKSEL		=SYSCON_DAC_CLKSEL_SEL(0U);//DAC0 clock main clock
	SYSCON->DAC[0].CLKDIV		=SYSCON_DAC_CLKDIV_RESET_MASK;//Resets the divider counter.
	SYSCON->DAC[0].CLKDIV		=SYSCON_DAC_CLKDIV_DIV(11U);//divide ADC0 clock by 12,
	PMC->PDRUNCFGCLR1 			= kPDRUNCFG_PD_DAC0 ;//clear DAC0 stop mode
	SYSCON->AHBCLKCTRLSET[0]	|=SYSCON_AHBCLKCTRL0_DAC0_MASK;//enable DAC0 interface clock
	DAC0->RCR					|=LPDAC_RCR_SWRST_MASK;
	DAC0->RCR					&=~LPDAC_RCR_SWRST_MASK;//Resets all DAC registers and internal logic
	DAC0->RCR					|=LPDAC_RCR_FIFORST_MASK;
	DAC0->RCR					&=~LPDAC_RCR_FIFORST_MASK;//Resets the FIFO pointers and flags in register FSR
	DAC0->GCR					=0x00000000|LPDAC_GCR_BUF_SPD_CTRL_MASK|LPDAC_GCR_BUF_EN_MASK
										|LPDAC_GCR_LATCH_CYC(1U)|LPDAC_GCR_IREF_ZTC_EXT_SEL_MASK;
								//low power mode;Opamp is used as buffer;Sync time is 2 RCLK cycles;External ZTC current reference selected
	DAC0->FCR 					= LPDAC_FCR_WML(0U);//not enable watermark

	DAC0->GCR 					|= LPDAC_GCR_DACEN_MASK;//DAC system is enabled.
	DAC0->DATA 					= LPDAC_DATA_DATA(
								( 5.0f <= 0.0f ? 2048U : ((uint16_t)(5.0f * (4096.0f / (5.0f * 3.3f))) + 2048U) ));//5A hardware DC current protect threshold

}

void InitHscmp(void)
{
	POWER_DisablePD(kPDRUNCFG_PD_HSCMP0);
    POWER_DisablePD(kPDRUNCFG_PD_CMPBIAS);
    SYSCON->AHBCLKCTRLSET[3]	|=SYSCON_AHBCLKCTRL3_HSCMP0_MASK;//enable HSCMP interface clock

    /* Input minus = External DAC, input plus = analog mux in3, high power/high speed mode */
    HSCMP0->CCR2 				|= (uint32_t)(HSCMP_CCR2_MSEL(5U) | HSCMP_CCR2_PSEL(3U) | HSCMP_CCR2_CMP_HPMD_MASK);

    /* HSCMP enable */
    HSCMP0->CCR0 				|= HSCMP_CCR0_CMP_EN_MASK;
}

void InitPINT(void)
{
	SYSCON->AHBCLKCTRLSET[0]	|=SYSCON_AHBCLKCTRL0_PINT_MASK;//enable PINT0 interface clock in SYSCON
	SYSCON->PRESETCTRLSET[0]	|=SYSCON_PRESETCTRL0_PINT_RST_MASK;
	SYSCON->PRESETCTRLCLR[0]	|=SYSCON_PRESETCTRL0_PINT_RST_MASK;

	INPUTMUX->PINTSEL[0]		=INPUTMUX_PINTSEL_INTPIN(0xD);//interrupt PINT0_0 from P0_13
	INPUTMUX->PINTSEL[1]		=INPUTMUX_PINTSEL_INTPIN(0xE);//interrupt PINT0_1 from P0_14
	INPUTMUX->PINTSEL[2]		=INPUTMUX_PINTSEL_INTPIN(0x2B);//interrupt PINT0_2 from P0_11

	PINT->ISEL					=0x00;//edge-sensitive interrupt
	PINT->RISE					|=PINT_RISE_RDET(0x07);//generate interrupt on rising edge
	PINT->FALL					|=PINT_FALL_FDET(0x07);//generate interrupt on falling edge
	PINT->IST					|=PINT_FALL_FDET(0x07);//clear rising edge and falling edge flag
	PINT->SIENF					|=PINT_IENF_ENAF(0x07);
	PINT->SIENR					|=PINT_IENF_ENAF(0x07);

	EnableIRQ(PIN_INT0_IRQn);
	EnableIRQ(PIN_INT1_IRQn);
	EnableIRQ(PIN_INT2_IRQn);
}

void InitGINT()
{
	SYSCON->AHBCLKCTRLSET[0]	|=SYSCON_AHBCLKCTRL0_GINT_MASK;//enable GINT0 interface clock in SYSCON
	SYSCON->PRESETCTRLSET[0]	|=SYSCON_PRESETCTRL0_GINT_RST_MASK;
	SYSCON->PRESETCTRLCLR[0]	|=SYSCON_PRESETCTRL0_GINT_RST_MASK;
	GINT0->CTRL					&=~GINT_CTRL_TRIG_MASK;
	GINT0->PORT_ENA[0]			=0x00000000;
	GINT0->PORT_POL[0]			=0x00000000;
	GINT0->CTRL					|=GINT_CTRL_INT_MASK;
	EnableIRQ(GINT0_IRQn);
}

uint16_t hall_a=0;
uint16_t hall_b=0;
uint16_t hall_c=0;
uint16_t hall_sector=0;
uint16_t ReadHall(void)
{
    uint16_t sector_temp=0;

    if(HALL_A_DETECTED)
    	{
    	hall_a=1;
    	sector_temp+=4;
    	}
    else hall_a=0;

    if(HALL_B_DETECTED)
    	{
    	hall_b=1;
    	sector_temp+=2;
    	}
    else hall_b=0;

    if(HALL_C_DETECTED)
    	{
    	hall_c=1;
    	sector_temp+=1;
    	}
    else hall_c=0;
    hall_sector=hall_a*4+hall_b*2+hall_c;
    return sector_temp;
}

void SetPwmDuty(float duty)
{
	int16_t valtmp;
	valtmp						=(int16_t)(duty*3750.0f);
	PWM0->SM[0].VAL2 			=(uint16_t)(3750+valtmp);
	PWM0->SM[0].VAL3 			=(uint16_t)(-3750-valtmp);
	PWM0->SM[1].VAL2 			=(uint16_t)(3750+valtmp);
	PWM0->SM[1].VAL3 			=(uint16_t)(-3750-valtmp);
	PWM0->SM[2].VAL2 			=(uint16_t)(3750+valtmp);
	PWM0->SM[2].VAL3 			=(uint16_t)(-3750-valtmp);
	PWM0->MCTRL 				|= PWM_MCTRL_LDOK(15);
}

void SetPwmSector(uint16_t sector)
{
	if(BldcCtl.UserSpeedCmd>0)
	{
		PWM0->OUTEN					= SectorCtrlCW[sector*2];
		PWM0->DTSRCSEL 				= SectorCtrlCW[2 * sector+1];
	}
    else
    {
    	PWM0->OUTEN					= SectorCtrlCCW[sector*2];
        PWM0->DTSRCSEL 				= SectorCtrlCCW[2 * sector+1];
    }

	PWM0->MCTRL 				|= PWM_MCTRL_LDOK(15);
}

void Communication(void)
{
	sector=ReadHall();
	if(AppState==AppRun)
	{
	SetPwmSector(sector);
	SpeedCnt[sector_old]		=CTIMER1->TC-Ctimer1_old;
	Ctimer1_old					=CTIMER1->TC;
	if(((sector==3) && (sector_old==1))
	 ||((sector==1) && (sector_old==5))
	 ||((sector==5) && (sector_old==4))
	 ||((sector==4) && (sector_old==6))
	 ||((sector==6) && (sector_old==2))
	 ||((sector==2) && (sector_old==3)))BldcCtl.Direction=-1;
	else BldcCtl.Direction=1;
	sector_old					=sector;
	}
}
