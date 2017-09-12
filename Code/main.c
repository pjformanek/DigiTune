/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "arm_math.h"

/***************************************************************************************************
 * 					TypeDefs and Variables
 ***************************************************************************************************/

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure_LED;
GPIO_InitTypeDef  GPIO_InitStructure_PWM;
ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
NVIC_InitTypeDef NVIC_InitStructure_TIM2;
NVIC_InitTypeDef NVIC_InitStructure_TIM5;
TIM_TimeBaseInitTypeDef TIM2_TimeBaseStructure;
TIM_TimeBaseInitTypeDef TIM5_TimeBaseStructure;
arm_rfft_instance_q15 RealFFT_Instance;
arm_cfft_radix4_instance_q15 MyComplexFFT_Instance;

#define SampleRate (2000)			//based from the TIM2 config
#define NumSamples (2048)			//number of samples or bins

//variables for ADC
volatile int16_t *Activebuffer;
volatile int16_t *Backbuffer;
volatile int16_t InBuffer1[NumSamples];
volatile int16_t InBuffer2[NumSamples];
volatile uint8_t CaptureActive = 0;
volatile uint16_t CurrentSample = 0;
volatile uint8_t BufferPtr = 0;

//variables for FFT
volatile q15_t FFTBuffer[NumSamples*2];
volatile q15_t FFTMag[NumSamples];
uint16_t binValue = 0;
uint16_t frequency=0;
uint16_t binNum = 0;

//variables for PWM
volatile uint16_t PWM_Counter = 0;
volatile uint16_t PWM_Dir;
volatile char GuitarString = 'E';
volatile int Begin = 0;

//variables for motor control and direction
uint8_t Stop=150;
uint8_t Up=145;
uint8_t Down=155;
uint16_t i=0;
int check=1;
int tune=0;
int flip=0;
int activate=0;

//bitmaps for painting LCD screen
extern const unsigned char Emap;
extern const unsigned char Amap;
extern const unsigned char Dmap;
extern const unsigned char Gmap;
extern const unsigned char Bmap;
extern const unsigned char emap;


/* Private function prototypes -----------------------------------------------*/
static void TP_Config(void);

/***************************************************************************************************
 * 					Main
 ***************************************************************************************************/
int main(void)
{
  static TP_STATE* TP_State; 
    
/***************************************************************************************************
 * 					Initializations
 ***************************************************************************************************/

  PWM_Dir = Stop;

  	void RCC_Configuration(void)
  	{
  		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
  		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  	}

  	    RCC_Configuration();

  	    void GPIO_Configuration(void)
  	{
  		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  		GPIO_Init(GPIOC, &GPIO_InitStructure);

  		GPIO_InitStructure_PWM.GPIO_Pin=GPIO_Pin_12;
  		GPIO_InitStructure_PWM.GPIO_Mode=GPIO_Mode_OUT;
  		GPIO_InitStructure_PWM.GPIO_OType=GPIO_OType_PP;
  		GPIO_InitStructure_PWM.GPIO_Speed=GPIO_Speed_100MHz;
  		GPIO_InitStructure_PWM.GPIO_PuPd=GPIO_PuPd_UP;
  		GPIO_Init(GPIOC,&GPIO_InitStructure_PWM);

  		GPIO_InitStructure_LED.GPIO_Pin = GPIO_Pin_2;
  		GPIO_InitStructure_LED.GPIO_Mode = GPIO_Mode_OUT;
  		GPIO_InitStructure_LED.GPIO_OType = GPIO_OType_PP;
  		GPIO_InitStructure_LED.GPIO_Speed = GPIO_Speed_100MHz;
  		GPIO_InitStructure_LED.GPIO_PuPd = GPIO_PuPd_NOPULL;
  		GPIO_Init(GPIOG, &GPIO_InitStructure_LED);
  	}


  	ADC_DeInit(); //restore ADC structure to power on default

  	void ADC_Configuration(void)
  	{
  		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; // 2 half-words one by one, 1 then 2
  		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  		ADC_CommonInit(&ADC_CommonInitStructure);

  		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  		ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1 Channel
  		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
  		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
  		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  		ADC_InitStructure.ADC_NbrOfConversion = 1;
  		ADC_Init(ADC3, &ADC_InitStructure);
  		ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);
  		ADC_Cmd(ADC3, ENABLE);
  	}

  	void TIM2_Configuration(void)
  	{
  		TIM_TimeBaseStructInit(&TIM2_TimeBaseStructure);
  		TIM2_TimeBaseStructure.TIM_Period = (45000000 / 2000) - 1; // 1 KHz, from 48 MHz TIM2CLK (ie APB1 = HCLK/4, TIM2 = APB2 = HCLK/2)
  		TIM2_TimeBaseStructure.TIM_Prescaler = 0;
  		TIM2_TimeBaseStructure.TIM_ClockDivision = 0;
  		TIM2_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  		TIM_TimeBaseInit(TIM2, &TIM2_TimeBaseStructure);
  		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  		TIM_Cmd(TIM2, ENABLE);
  }

  	void TIM5_Configuration()
  	{
  		TIM_TimeBaseStructInit(&TIM5_TimeBaseStructure);
  		TIM5_TimeBaseStructure.TIM_Period = (90000000 / 100000) - 1;// for a 0.1ms tick, then set the IRQ handler to count to 199 for a 20ms period
  		TIM5_TimeBaseStructure.TIM_Prescaler = 0;
  		TIM5_TimeBaseStructure.TIM_ClockDivision = 0;
  		TIM5_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  		TIM5_TimeBaseStructure.TIM_RepetitionCounter = 0;
  		TIM_TimeBaseInit(TIM5, &TIM5_TimeBaseStructure);
  		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
  		TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  		TIM_Cmd(TIM5, ENABLE);
  	}

  	void NVIC_Configuration(void)
  	{
  		NVIC_InitStructure_TIM2.NVIC_IRQChannel = TIM2_IRQn;
  		NVIC_InitStructure_TIM2.NVIC_IRQChannelPreemptionPriority = 1;
  		NVIC_InitStructure_TIM2.NVIC_IRQChannelSubPriority = 1;
  		NVIC_InitStructure_TIM2.NVIC_IRQChannelCmd = ENABLE;
  		NVIC_Init(&NVIC_InitStructure_TIM2);

  		NVIC_InitStructure_TIM5.NVIC_IRQChannel = TIM5_IRQn;
  		NVIC_InitStructure_TIM5.NVIC_IRQChannelPreemptionPriority = 0;
  		NVIC_InitStructure_TIM5.NVIC_IRQChannelSubPriority = 0;
  		NVIC_InitStructure_TIM5.NVIC_IRQChannelCmd = ENABLE;
  		NVIC_Init(&NVIC_InitStructure_TIM5);
  	}


  	    GPIO_Configuration();

  	    NVIC_Configuration();

  	    TIM2_Configuration();

  	    TIM5_Configuration();

  	    ADC_Configuration();

  /* LCD initialization */
  LCD_Init();
  
  /* LCD Layer initialization */
  LCD_LayerInit();
    
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD foreground layer */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);
  
  /* Touch Panel configuration */
  TP_Config();

  BufferPtr = 0;
  Activebuffer = InBuffer1;
  Backbuffer = InBuffer2;

  arm_rfft_init_q15(&RealFFT_Instance,
          			&MyComplexFFT_Instance,
          			NumSamples,
          			0,
          			1);

/***************************************************************************************************
 * 					The magic
 ***************************************************************************************************/
  while (1)
  {
	  //Wait for the Background capture to complete
	  		while(CaptureActive == 1)
	  		{
	  		}

	  		//Swap the Active/Background Buffer ---> Ping Pong!
	  		if( BufferPtr == 0)
	  		{
	  			BufferPtr = 1;
	  			Activebuffer = InBuffer2;
	  			Backbuffer = InBuffer1;
	  		}
	  		else
	  		{
	  			BufferPtr = 0;
	  			Activebuffer = InBuffer1;
	  			Backbuffer = InBuffer2;
	  		}

	  		//Start the background capture on the new buffer
	  		CaptureActive = 1;

	  		//Passing ADC sample array to the FFT function
	      	arm_rfft_q15(&RealFFT_Instance,
	      	    	    (q15_t *)Activebuffer,
	      	    	    (q15_t *)FFTBuffer);

	      	//Scaling, due to the FFT function
	  		for(i = 0; i < (NumSamples*2); i++)
	  			{
	  				FFTBuffer[i] <<= 2;
	  			}

	  	   //Calculating peak magnitudes
	     	   arm_cmplx_mag_q15((q15_t *)FFTBuffer,
	   	  		   	   	   	 (q15_t *)FFTMag,
	      	  	     	 		  NumSamples);


	     	   //Parsing array for the first significant peak which is our fundamental frequency
	     	   for(i=10; (i<1024 && !tune); i++)
	     	   {
	     		   if(FFTMag[i]>20)
	     		   {
	     			   binValue = FFTMag[i];
	     			   binNum = i;
	     			   tune=1;
	     		   }
	     	   }

	     	   //Calculating which frequency has the strongest magnitude
	         frequency = (binNum*((float32_t)(2*SampleRate)/NumSamples));

/********************************************************************************************************************************************
 * 							Tuning
 ********************************************************************************************************************************************/
if(activate)
{
	if(check)
	{
	         	   switch (GuitarString)
	         	   {
	         		case 'E':
	         			if(tune) //checks for a valid signal magnitude
	         			{
	         				if(frequency == 82 || (frequency < 84 && frequency > 80)) //if desired frequency, don't move
	         				{
	         					PWM_Dir = Stop;
	         					check = 0;
	         					LCD_SetTextColor(LCD_COLOR_GREEN); //draws a green border when string is tuned
	         					LCD_DrawFullRect(0, 0, 118, 15);
	         					LCD_DrawFullRect(0, 183, 118, 15);
	         					LCD_DrawFullRect(0, 0, 15, 198);
	         					LCD_DrawFullRect(103, 0, 15, 198);
	         					break;
	         				}
	         				else if(frequency > 90) //If way over, quickly get it close
	         				{
	         					PWM_Dir = Down;
	         					LCD_SetTextColor(LCD_COLOR_YELLOW);
	         					LCD_DrawFullRect(0, 0, 118, 15);
	         				    LCD_DrawFullRect(0, 183, 118, 15);
	         				    LCD_DrawFullRect(0, 0, 15, 198);
	         				    LCD_DrawFullRect(103, 0, 15, 198);
	         					break;
	         				}
	         				else if(frequency > 82) //Slow Down and dial it in
	         				{
	         					PWM_Dir = Down-3;
	         					LCD_SetTextColor(LCD_COLOR_YELLOW);
	         				    LCD_DrawFullRect(0, 0, 118, 15);
	         				    LCD_DrawFullRect(0, 183, 118, 15);
	         					LCD_DrawFullRect(0, 0, 15, 198);
	         				    LCD_DrawFullRect(103, 0, 15, 198);
	         					break;
	         				}
	         				else if(frequency < 75) //Same as above but in opposite direction
	         				{
	         					PWM_Dir = Up;
	         					LCD_SetTextColor(LCD_COLOR_YELLOW);
	         				    LCD_DrawFullRect(0, 0, 118, 15);
	         				    LCD_DrawFullRect(0, 183, 118, 15);
	         				    LCD_DrawFullRect(0, 0, 15, 198);
	         				    LCD_DrawFullRect(103, 0, 15, 198);
	         					break;
	         				}
	         				else if(frequency < 82)
	         				{
	         					PWM_Dir = Up+2;
	         					LCD_SetTextColor(LCD_COLOR_YELLOW);
	         				    LCD_DrawFullRect(0, 0, 118, 15);
	         				    LCD_DrawFullRect(0, 183, 118, 15);
	         				    LCD_DrawFullRect(0, 0, 15, 198);
	         				    LCD_DrawFullRect(103, 0, 15, 198);
	         					break;
	         				}
	         			}
	         		case 'A':
	         			if(tune)
	         			{
	         				if(frequency == 110 || (frequency < 112 && frequency > 108))
	         				{
	         					PWM_Dir = Stop;
	         					check=0;
	         					LCD_SetTextColor(LCD_COLOR_GREEN);
	         				    LCD_DrawFullRect(0, 0, 118, 15);
	         				    LCD_DrawFullRect(0, 183, 118, 15);
	         				    LCD_DrawFullRect(0, 0, 15, 198);
	         					LCD_DrawFullRect(103, 0, 15, 198);
	         					break;
	         				}
	         				else if(frequency > 120)
	         				{
	         				   PWM_Dir = Down;
	         				   LCD_SetTextColor(LCD_COLOR_YELLOW);
	         				   LCD_DrawFullRect(0, 0, 118, 15);
	         				   LCD_DrawFullRect(0, 183, 118, 15);
	         				   LCD_DrawFullRect(0, 0, 15, 198);
	         				   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}
	         				else if(frequency > 110)
	         				{
	         				   PWM_Dir = Down-3;
	         				   LCD_SetTextColor(LCD_COLOR_YELLOW);
	         				   LCD_DrawFullRect(0, 0, 118, 15);
							   LCD_DrawFullRect(0, 183, 118, 15);
							   LCD_DrawFullRect(0, 0, 15, 198);
							   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}
	         				else if(frequency < 100)
	         				{
	         				   PWM_Dir = Up;
	         				   LCD_SetTextColor(LCD_COLOR_YELLOW);
							   LCD_DrawFullRect(0, 0, 118, 15);
							   LCD_DrawFullRect(0, 183, 118, 15);
							   LCD_DrawFullRect(0, 0, 15, 198);
							   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}
	         				else if(frequency < 110)
	         				{
	         				   PWM_Dir = Up+2;
	         				   LCD_SetTextColor(LCD_COLOR_YELLOW);
							   LCD_DrawFullRect(0, 0, 118, 15);
							   LCD_DrawFullRect(0, 183, 118, 15);
							   LCD_DrawFullRect(0, 0, 15, 198);
							   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}
	         			}
	         		case 'D':
	         			if(tune)
	         			{
	         				if(frequency == 147 || (frequency < 149 && frequency > 145))
	         				{
	         				    PWM_Dir = Stop;
	         				    check=0;
	         				    LCD_SetTextColor(LCD_COLOR_GREEN);
								LCD_DrawFullRect(0, 0, 118, 15);
								LCD_DrawFullRect(0, 183, 118, 15);
								LCD_DrawFullRect(0, 0, 15, 198);
								LCD_DrawFullRect(103, 0, 15, 198);
	         				    break;
	         				}
	         				else if(frequency > 157)
	         				{   PWM_Dir = Down;
	         				    LCD_SetTextColor(LCD_COLOR_YELLOW);
							    LCD_DrawFullRect(0, 0, 118, 15);
							    LCD_DrawFullRect(0, 183, 118, 15);
							    LCD_DrawFullRect(0, 0, 15, 198);
							    LCD_DrawFullRect(103, 0, 15, 198);
	         				    break;
	         				}
	         				else if(frequency > 147)
	         				{   PWM_Dir = Down-3;
	         				    LCD_SetTextColor(LCD_COLOR_YELLOW);
						        LCD_DrawFullRect(0, 0, 118, 15);
							    LCD_DrawFullRect(0, 183, 118, 15);
							    LCD_DrawFullRect(0, 0, 15, 198);
						        LCD_DrawFullRect(103, 0, 15, 198);
	         				    break;
	         				}
	         				if(frequency < 137)
	         				{
	         				   PWM_Dir = Up;
	         				   LCD_SetTextColor(LCD_COLOR_YELLOW);
							   LCD_DrawFullRect(0, 0, 118, 15);
							   LCD_DrawFullRect(0, 183, 118, 15);
							   LCD_DrawFullRect(0, 0, 15, 198);
							   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}
	         				if(frequency < 147)
	         				{
	         				   PWM_Dir = Up+3;
	         				   LCD_SetTextColor(LCD_COLOR_YELLOW);
							   LCD_DrawFullRect(0, 0, 118, 15);
							   LCD_DrawFullRect(0, 183, 118, 15);
							   LCD_DrawFullRect(0, 0, 15, 198);
							   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}
	         			}
	         		case 'G':
	         			if(tune)
	         			{
	         				if(frequency == 196 || (frequency < 198 && frequency > 194))
	         				{
	         				   PWM_Dir = Stop;
	         				   check=0;
	         				   LCD_SetTextColor(LCD_COLOR_GREEN);
							   LCD_DrawFullRect(0, 0, 118, 15);
							   LCD_DrawFullRect(0, 183, 118, 15);
							   LCD_DrawFullRect(0, 0, 15, 198);
							   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}
	         				else if(frequency > 206)
	         				{
	         				   PWM_Dir = Down;
	         				  LCD_SetTextColor(LCD_COLOR_YELLOW);
							  LCD_DrawFullRect(0, 0, 118, 15);
							  LCD_DrawFullRect(0, 183, 118, 15);
							  LCD_DrawFullRect(0, 0, 15, 198);
							  LCD_DrawFullRect(103, 0, 15, 198);
	         				  break;
	         				}
	         				else if(frequency > 196)
	         			    {
	         					PWM_Dir = Down-4;
	         					LCD_SetTextColor(LCD_COLOR_YELLOW);
								LCD_DrawFullRect(0, 0, 118, 15);
								LCD_DrawFullRect(0, 183, 118, 15);
								LCD_DrawFullRect(0, 0, 15, 198);
								LCD_DrawFullRect(103, 0, 15, 198);
	         					break;
	         				}
	         				else if(frequency < 186)
	         				{
	         					PWM_Dir = Up;
	         					LCD_SetTextColor(LCD_COLOR_YELLOW);
								LCD_DrawFullRect(0, 0, 118, 15);
								LCD_DrawFullRect(0, 183, 118, 15);
								LCD_DrawFullRect(0, 0, 15, 198);
								LCD_DrawFullRect(103, 0, 15, 198);
	         					break;
	         				}
	         				else if(frequency < 196)
	         				{
	         				   PWM_Dir = Up+3;
	         				   LCD_SetTextColor(LCD_COLOR_YELLOW);
							   LCD_DrawFullRect(0, 0, 118, 15);
							   LCD_DrawFullRect(0, 183, 118, 15);
							   LCD_DrawFullRect(0, 0, 15, 198);
							   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}
	         			}
	         		case 'B':
	         			if(tune)
	         			{
	         				if(frequency == 247 || (frequency < 249 && frequency > 245))
	         				{
	         				  PWM_Dir = Stop;
	         				  check=0;
	         				  LCD_SetTextColor(LCD_COLOR_GREEN);
							  LCD_DrawFullRect(0, 0, 118, 15);
							  LCD_DrawFullRect(0, 183, 118, 15);
							  LCD_DrawFullRect(0, 0, 15, 198);
							  LCD_DrawFullRect(103, 0, 15, 198);
	         				  break;
	         				}
	         				else if(frequency > 257)
	         				{
	         				   PWM_Dir = Down;
	         				   LCD_SetTextColor(LCD_COLOR_YELLOW);
							   LCD_DrawFullRect(0, 0, 118, 15);
						   	   LCD_DrawFullRect(0, 183, 118, 15);
							   LCD_DrawFullRect(0, 0, 15, 198);
							   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}
	         				else if(frequency > 247)
	         				{
	         				   PWM_Dir = Down-4;
	         				  LCD_SetTextColor(LCD_COLOR_YELLOW);
							  LCD_DrawFullRect(0, 0, 118, 15);
							  LCD_DrawFullRect(0, 183, 118, 15);
							  LCD_DrawFullRect(0, 0, 15, 198);
							  LCD_DrawFullRect(103, 0, 15, 198);
	         				  break;
	         				}
	         				else if(frequency < 237)
	         				{
	         					PWM_Dir = Up;
	         					LCD_SetTextColor(LCD_COLOR_YELLOW);
								LCD_DrawFullRect(0, 0, 118, 15);
								LCD_DrawFullRect(0, 183, 118, 15);
								LCD_DrawFullRect(0, 0, 15, 198);
								LCD_DrawFullRect(103, 0, 15, 198);
	         				}
	         				else if(frequency < 247)
	         				{
	         				   PWM_Dir = Up+3;
	         				  LCD_SetTextColor(LCD_COLOR_YELLOW);
							  LCD_DrawFullRect(0, 0, 118, 15);
							  LCD_DrawFullRect(0, 183, 118, 15);
							  LCD_DrawFullRect(0, 0, 15, 198);
							  LCD_DrawFullRect(103, 0, 15, 198);
	         				  break;
	         				}
	         			}
	         		case 'e':
	         			if(tune)
	         			{
	         				if(frequency == 330 || (frequency < 332 && frequency > 328))
	         				{
	         				   PWM_Dir = Stop;
	         				   check=0;
	         				   LCD_SetTextColor(LCD_COLOR_GREEN);
							   LCD_DrawFullRect(0, 0, 118, 15);
							   LCD_DrawFullRect(0, 183, 118, 15);
							   LCD_DrawFullRect(0, 0, 15, 198);
							   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}
	         				else if(frequency < 320)
	         				{
	         					PWM_Dir = Up;
	         					LCD_SetTextColor(LCD_COLOR_YELLOW);
								LCD_DrawFullRect(0, 0, 118, 15);
								LCD_DrawFullRect(0, 183, 118, 15);
								LCD_DrawFullRect(0, 0, 15, 198);
								LCD_DrawFullRect(103, 0, 15, 198);
	         					break;
	         				}
	         				else if(frequency < 330)
	         				{
	         				   PWM_Dir = Up+3;
	         				  LCD_SetTextColor(LCD_COLOR_YELLOW);
							  LCD_DrawFullRect(0, 0, 118, 15);
							  LCD_DrawFullRect(0, 183, 118, 15);
							  LCD_DrawFullRect(0, 0, 15, 198);
							  LCD_DrawFullRect(103, 0, 15, 198);
	         				  break;
	         				}
	         				else if(frequency > 340)
	         				{
	         					PWM_Dir = Down;
	         					LCD_SetTextColor(LCD_COLOR_YELLOW);
								LCD_DrawFullRect(0, 0, 118, 15);
								LCD_DrawFullRect(0, 183, 118, 15);
								LCD_DrawFullRect(0, 0, 15, 198);
								LCD_DrawFullRect(103, 0, 15, 198);
	         					break;
	         				}
	         				else if(frequency > 330)
	         				{
	         				   PWM_Dir = Down-4;
	         				   LCD_SetTextColor(LCD_COLOR_YELLOW);
							   LCD_DrawFullRect(0, 0, 118, 15);
							   LCD_DrawFullRect(0, 183, 118, 15);
							   LCD_DrawFullRect(0, 0, 15, 198);
							   LCD_DrawFullRect(103, 0, 15, 198);
	         				   break;
	         				}

	         			}
	         		default :
	         			   PWM_Dir = Stop;
	         			   break;
	         		}
	}

}
else
{
	PWM_Dir = Stop;
}

//end of guitar tuning code. Activate needs to be on


	        	   //Reset the variables and repeat
	               binValue = 0;
	               binNum = 0;
	               tune=0;


/***************************************************************************************************
 * 							Touch Panel Code
 ***************************************************************************************************/
    TP_State = IOE_TP_GetState();
    
    if ((TP_State->TouchDetected) && (TP_State->Y <= 280) && (TP_State->Y >= 200) && (TP_State->X >= 5) && (TP_State->X <= 110))
        {
        	switch(GuitarString)
        	{
        	case 'E': GuitarString='A';
        			  LCD_WriteBMP(&Amap);
        			  break;

        	case 'A': GuitarString='D';
        			  LCD_WriteBMP(&Dmap);
        			  break;

        	case 'D': GuitarString='G';
        			  LCD_WriteBMP(&Gmap);
        			  break;

        	case 'G': GuitarString='B';
        			  LCD_WriteBMP(&Bmap);
        			  break;

        	case 'B': GuitarString='e';
        			  LCD_WriteBMP(&emap);
        			  break;

        	case 'e': GuitarString='E';
        			  LCD_WriteBMP(&Emap);
        			  break;

        	}

        	if(activate)
        	{
               LCD_SetTextColor(LCD_COLOR_BLUE2);
      	       LCD_DrawFullRect(122, 82, 116, 15);
       	       LCD_DrawFullRect(122, 183, 116, 15);
       	       LCD_DrawFullRect(122, 82, 15, 116);
      	       LCD_DrawFullRect(225, 82, 15, 116);
            }

        	if(flip)
        	{
        	   LCD_SetTextColor(LCD_COLOR_MAGENTA);
        	   LCD_DrawFullRect(122, 0, 116, 10);
        	   LCD_DrawFullRect(122, 69, 116, 10);
        	   LCD_DrawFullRect(122, 0, 10, 79);
        	   LCD_DrawFullRect(230, 0, 10, 79);
        	}

        	check=1;

        }

        if ((TP_State->TouchDetected) && (TP_State->Y <= 280) && (TP_State->Y >= 200) && (TP_State->X >= 125) && (TP_State->X <= 235))
        {
        	switch(GuitarString)
        	{
        	case 'E': GuitarString='e';
        			  LCD_WriteBMP(&emap);
        			  break;

        	case 'A': GuitarString='E';
        			  LCD_WriteBMP(&Emap);
        	    	  break;

        	case 'D': GuitarString='A';
        			  LCD_WriteBMP(&Amap);
        	    	  break;

            case 'G': GuitarString='D';
            		  LCD_WriteBMP(&Dmap);
        	    	  break;

        	case 'B': GuitarString='G';
        			  LCD_WriteBMP(&Gmap);
        	    	  break;

        	case 'e': GuitarString='B';
        			  LCD_WriteBMP(&Bmap);
        	    	  break;
        	}

        	if(activate)
        	{
        	   LCD_SetTextColor(LCD_COLOR_BLUE2);
        	   LCD_DrawFullRect(122, 82, 116, 15);
        	   LCD_DrawFullRect(122, 183, 116, 15);
        	   LCD_DrawFullRect(122, 82, 15, 116);
        	   LCD_DrawFullRect(225, 82, 15, 116);
        	}

        	if(flip)
        	{
        	    LCD_SetTextColor(LCD_COLOR_MAGENTA);
        	    LCD_DrawFullRect(122, 0, 116, 10);
        	    LCD_DrawFullRect(122, 69, 116, 10);
        	    LCD_DrawFullRect(122, 0, 10, 79);
        	    LCD_DrawFullRect(230, 0, 10, 79);
        	}

        	check=1;

        }

    if ((TP_State->TouchDetected) && (TP_State->Y <= 75) && (TP_State->Y >= 5) && (TP_State->X >= 125) && (TP_State->X <= 235))
    {
        	if(flip)
        	{
        		LCD_SetTextColor(LCD_COLOR_WHITE);
        		LCD_DrawFullRect(122, 0, 116, 10);
        		LCD_DrawFullRect(122, 69, 116, 10);
        		LCD_DrawFullRect(122, 0, 10, 79);
        		LCD_DrawFullRect(230, 0, 10, 79);
        		flip=0;
        		Up=145;
        		Down=155;
        	}
        	else
        	{
        		LCD_SetTextColor(LCD_COLOR_MAGENTA);
        		LCD_DrawFullRect(122, 0, 116, 10);
        		LCD_DrawFullRect(122, 69, 116, 10);
         		LCD_DrawFullRect(122, 0, 10, 79);
        		LCD_DrawFullRect(230, 0, 10, 79);
        		flip=1;
        		Up=155;
        		Down=145;
        	}
    }

    if ((TP_State->TouchDetected) && (TP_State->Y <= 195) && (TP_State->Y >= 85) && (TP_State->X >= 125) && (TP_State->X <= 235))
    {
    	if(activate)
    	{
    		activate=0;
    		LCD_SetTextColor(LCD_COLOR_WHITE);
    		LCD_DrawFullRect(122, 82, 116, 15);
    		LCD_DrawFullRect(122, 183, 116, 15);
    		LCD_DrawFullRect(122, 82, 15, 116);
       		LCD_DrawFullRect(225, 82, 15, 116);
    	}
    	else
    	{
    		activate=1;
    		LCD_SetTextColor(LCD_COLOR_BLUE2);
    		LCD_DrawFullRect(122, 82, 116, 15);
    		LCD_DrawFullRect(122, 183, 116, 15);
    		LCD_DrawFullRect(122, 82, 15, 116);
    		LCD_DrawFullRect(225, 82, 15, 116);
    	}
    }

    if ((TP_State->TouchDetected) && (TP_State->Y <= 195) && (TP_State->Y >= 5) && (TP_State->X >= 5) && (TP_State->X <= 115))
    {
    	LCD_SetTextColor(LCD_COLOR_WHITE);
        LCD_DrawFullRect(0, 0, 118, 15);
        LCD_DrawFullRect(0, 183, 118, 15);
        LCD_DrawFullRect(0, 0, 15, 198);
    	LCD_DrawFullRect(103, 0, 15, 198);
    	check=1;
    }


  }
/***************************************************************************************************
 * 					End of while loop
 ***************************************************************************************************/

}



/***************************************************************************************************
 * 					Initial Touch Screen Layout
 ***************************************************************************************************/
static void TP_Config(void)
{
  /* Clear the LCD */ 
  LCD_Clear(LCD_COLOR_WHITE);
  
  /* Configure the IO Expander */
  if (IOE_Config() == IOE_OK)
  {
	  LCD_WriteBMP(&Emap);
  }  

}


