#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "delay.h"

uint16_t buttonState = 0;
uint16_t flag = 0;
uint16_t potValue = 0;
uint16_t mapValue = 0;

void  gpioConfig(){
	
      GPIO_InitTypeDef GPIOInitStructure;
	
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); 
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	
	    
	    //base pin + leds
	    GPIOInitStructure.GPIO_Mode = GPIO_Mode_Out_PP;               
	    GPIOInitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;      
	    GPIOInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	    GPIO_Init(GPIOB,&GPIOInitStructure);
	
	    //button
	    GPIOInitStructure.GPIO_Mode = GPIO_Mode_IPD;               
	    GPIOInitStructure.GPIO_Pin = GPIO_Pin_6;      
	  
	    GPIO_Init(GPIOC,&GPIOInitStructure);
	
	    //pwm pin
			GPIOInitStructure.GPIO_Mode = GPIO_Mode_AF_PP;               
	    GPIOInitStructure.GPIO_Pin = GPIO_Pin_0;      
	    GPIOInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	    GPIO_Init(GPIOA,&GPIOInitStructure);
			
			//pot pin
	    GPIOInitStructure.GPIO_Mode = GPIO_Mode_AIN;
	    GPIOInitStructure.GPIO_Pin = GPIO_Pin_1;  
	
	     GPIO_Init(GPIOA,&GPIOInitStructure);
	    
}

void timerConfig(){
  
      TIM_TimeBaseInitTypeDef TIMERInitStructure;
	
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	    TIMERInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;                   
	    TIMERInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    
	    TIMERInitStructure.TIM_Period = 2399;                       
	    TIMERInitStructure.TIM_Prescaler = 10;                    
	    TIMERInitStructure.TIM_RepetitionCounter = 0;
	
	    TIM_TimeBaseInit(TIM2,&TIMERInitStructure);
      TIM_Cmd(TIM2,ENABLE);
	
}

void pwmConfig(uint32_t timPulse){

      TIM_OCInitTypeDef TIMER_OCInitStructure;
	    
	    TIMER_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                  //PWM1 %75 high ile baslar , PWM2 ise %25 high ile baslar . 
      TIMER_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;         //yukardakiyle ayni mantik biz yüksek baslamasini sagladik.
      TIMER_OCInitStructure.TIM_OutputState =  TIM_OutputState_Enable;
	    TIMER_OCInitStructure.TIM_Pulse = timPulse;
	
	    TIM_OC1Init(TIM2,&TIMER_OCInitStructure);
	    TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
}

static void adcConfig(){

  ADC_InitTypeDef ADCInitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

  ADCInitStructure.ADC_ContinuousConvMode = ENABLE;
	ADCInitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADCInitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADCInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADCInitStructure.ADC_NbrOfChannel = 1;
	ADCInitStructure.ADC_ScanConvMode = DISABLE;
	
	ADC_Init(ADC1,&ADCInitStructure);
	ADC_Cmd(ADC1,ENABLE);
	
}

static uint16_t readADC(){

   ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_55Cycles5);

	 ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	
	 while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	
   return ADC_GetConversionValue(ADC1);
		
}

float map(float adcValue,float max,float min,float conMax,float conMin){

   return adcValue*((conMax-conMin)/(max-min));

}


int main(){
	
	  gpioConfig();
	  timerConfig();
	  DelayInit();
	  adcConfig();
	
	  while(1){

    potValue = readADC();
		mapValue = map(potValue,4095,0,2399,0);
		pwmConfig(mapValue);


//motor hiz kontrolu			
//		for(int i=0;i<2399;i++){
//		
//		pwmConfig(i);
//		delay_ms(100);
//			
//		}
		

//kalici butonla motor surme			
//			buttonState = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6);

//		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)!=0);

//			if(buttonState == 1){
//		
//		  if(flag == 0){
//			
//			flag = 1;
//			GPIO_SetBits(GPIOB,GPIO_Pin_0 | GPIO_Pin_2);     //pin 0 motor pin
//			GPIO_ResetBits(GPIOB,GPIO_Pin_1);
//			
//			}
//			
//			else{
//			
//			flag = 0;
//			GPIO_SetBits(GPIOB,GPIO_Pin_1);
//			GPIO_ResetBits(GPIOB,GPIO_Pin_0 | GPIO_Pin_2);              //pin 0 motor pin
//				
//			}
//			
//			delay_ms(1000);
//		
//		}
			
			
			
//butonla motor kontrol			
			
//		  if(buttonState == 1){
//			
//			GPIO_SetBits(GPIOB,GPIO_Pin_0 | GPIO_Pin_2);
//			GPIO_ResetBits(GPIOB,GPIO_Pin_1);
//			
//			}
//			
//			else{
//			
//		  GPIO_SetBits(GPIOB,GPIO_Pin_1);
//			GPIO_ResetBits(GPIOB,GPIO_Pin_0 | GPIO_Pin_2);
//			
//			}
			
			
			 
//butonun calismasi
			
//		GPIO_SetBits(GPIOB,GPIO_Pin_0);
//		delay_ms(1000);
//		GPIO_ResetBits(GPIOB,GPIO_Pin_0);
//		delay_ms(1000);
		
		}

}


