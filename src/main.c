#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "assert.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx.h"
#include "math.h"


// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/* Constants */
#define myTIM2_PRESCALER ((uint16_t)0x0000)    // Clock prescaler for TIM2 timer: noprescaling
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)   // overflow maximum
#define RESISTANCE_MAX ((uint16_t)5000)        // resistance maximum value
#define myTIM3_PRESCALER ((uint16_t) 0xBB80)   // Clock prescaler for TIM3 timer: 48Mhz/48K (0xBB80 = 48000) = 1Khz prescaling
#define myTIM3_PERIOD ((uint32_t) 0xFFFFFFFF)  // Maximum possible setting for overflow
#define TRUE ((int) 1)
#define FALSE (~TRUE)

/* Variables */
float resistance = 0;
float frequency = 0;
volatile uint32_t finishedCounting = 0;
uint16_t clear = 0x0001;
uint16_t autoIncrement = (uint16_t) (0x3 << 10);
uint16_t displayOn  = (uint16_t) (0x3 << 11);
uint16_t functionSet = (uint16_t) (0x7 << 13); // two lines of eight characters, 8-bit interface
uint16_t firstLine = 0x0080;
uint16_t secondLine = 0x00C0;
uint8_t HChar = (uint8_t)'H';
uint8_t zChar = (uint8_t)'z';
uint8_t FChar = (uint8_t)'F';
uint8_t semiColonChar = (uint8_t) ':';
uint8_t RChar = (uint8_t) 'R';
uint8_t OChar = (uint8_t)'O';
uint8_t hChar = (uint8_t) 'h';
uint8_t mChar = (uint8_t) 'm';
uint8_t sChar = (uint8_t) 's';
uint8_t dotChar = (uint8_t) '.';
uint8_t spaceChar = (uint8_t) ' ';
char stringFreq[5];
char stringRes[5];

/* Function Prototypes */
void myGPIO_Init(void);     // Initializes GPIO ports
void myADC_DAC_Init(void);  // Initialize ADC and DAC
void myTIM2_Init(void);     // Initializes Tim2 for use with frequency reading (times signal period)
void myTIM3_Init(void);     // Initializes timer3 used for delays
void myEXTI_Init(void);     // Initializes external interrupt to get frequency
void delay(int);            // delay function
void initDisplay(void);     // Initializes LCD
void sendCommand(uint16_t, int); // send command to LCD
void runDisplay(void);      // Send values to display


int main(int argc, char* argv[]){
	myGPIO_Init();
	myADC_DAC_Init();
	myTIM3_Init();
	myEXTI_Init();
	myTIM2_Init();
	initDisplay();

    uint16_t rawAnalog = 0;

	while (1){
		while((ADC1->ISR & ADC_ISR_EOC) == 0); // Check that it is end of conversion
		ADC1->ISR &= ~(ADC_ISR_EOC);           // Clear status conversion flag
		rawAnalog = ADC1->DR;
		DAC->DHR12R1 = rawAnalog;              // Send value to optocoupler and to 555 timer
		resistance = (float)rawAnalog/4096 * RESISTANCE_MAX;
		delay(100);
		runDisplay();                          // Display frequency and resistance on LCD
		delay(1000);
	}
}

void myADC_DAC_Init(){
	/* ADC */
	RCC->APB2ENR|= RCC_APB2ENR_ADCEN;         // Enable ADC clock
	ADC1->CFGR1 |= ADC_CFGR1_CONT;            // Set ADC in continuous mode
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;          // Set ADC to overwrite last conversion result
	ADC1->CHSELR = ADC_CHSELR_CHSEL11;        // Select channel 11 for pin 1 to be input channel
	ADC1->CR |= ADC_CR_ADEN;                  // Enable ADC
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0); // Wait for ADC to be ready to start operation
	ADC1->CR |= ADC_CR_ADSTART;               // Start ADC measurements

	/* DAC */
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;        // Enable DAC clock
	DAC->CR |= DAC_CR_EN1;                    // Enable the only DAC channel as PA4
}



void myGPIO_Init(){
	/* Port A */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;    // Enables GPIOA clock
	GPIOA->MODER &= ~(GPIO_MODER_MODER0); // Sets PA0 as input with no pull up or pull down
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0); // Ensures no pull up/down
    GPIOA->MODER &= ~(GPIO_MODER_MODER1); // Set PA1 as input for 555 timer with no pull up or pull down
    GPIOA->MODER &= ~(GPIO_PUPDR_PUPDR1);
	GPIOA->MODER |=  (GPIO_MODER_MODER4); // Set PA4 as analog function with no pull up or pull down
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

    /* Port B */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;    // Enable GPIOB clock
	GPIOB->MODER &= ~(GPIO_MODER_MODER7); // Set PB7 as input from LCD
	GPIOB->MODER |= 0x55551500;           // Set other port B pins as output for LCD display
	GPIOB->PUPDR =  (0X0<<31);            // Set no pull up or pull down for port B

	/* Port C */
	RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;   // Enable port C GPIO clock for ADC
	GPIOC->MODER |= (GPIO_MODER_MODER1);  // Set PC1 as Analog function
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1); // Set no pull up/pull down
	GPIOC->MODER |=(0x1 << 16|0x1 << 18); // Set PC8 and PC9 as output for LEDS
}

void myTIM2_Init(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    // Enable clock for TIM2 peripheral
	TIM2->CR1 = ((uint16_t) 0x008C);       // Set as count up, stop on overflow, enable update events, interrupt on overflow only
	TIM2->PSC = myTIM2_PRESCALER;          // Set timer2 prescaler
	TIM2->ARR = myTIM2_PERIOD;             // Set timer2 count value
	TIM2->EGR = ((uint16_t) 0x0001);       // Update the timer registers
	NVIC_SetPriority(TIM2_IRQn, 0);        // Assign timer 2 interrupt priority in NVIC
	NVIC_EnableIRQ(TIM2_IRQn);             // Enable timer2 interrupt in NVIC
	TIM2->DIER |= TIM_DIER_UIE;            // Enable interrupt generation
}

void TIM2_IRQHandler() {
	// Check if update interrupt flag is set
	if ((TIM2->SR & TIM_SR_UIF) != 0) {
		trace_printf("\n Timer Overflow \n");
		TIM2->SR &= ~(TIM_SR_UIF); // Clear update interrupt flag
		TIM2->CR1 |= TIM_CR1_CEN; // Restart timer
	}
}


void EXTI0_1_IRQHandler(){
	    uint32_t count = 0;
	    float freq = 0;
	    // Check if EXTI1 interrupt pending flag is set
		if ((EXTI->PR & EXTI_PR_PR1) != 0)
		{
			if (finishedCounting == 1)
			{
				TIM2 -> CNT = 0X0;         // Reset counter register
				TIM2 -> CR1 = TIM_CR1_CEN; // Start timer
				finishedCounting = 0;      // Clear counting flag
			}
			else
			{
				TIM2 -> CR1 &= ~(TIM_CR1_CEN);       // Stop timer
				count = TIM2->CNT;                   // Get current count
				freq =(float)SystemCoreClock / count; // Get the frequency from count
				frequency = freq;
				finishedCounting = 1;      // Set counting flag to done
			}
	 	EXTI->PR |= EXTI_PR_PR1;    // Clear EXTI1 interrupt pending flag by writing 1 to PR bit
	}
}


void myTIM3_Init(){
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;     // Enable clock for TIM3 peripheral
    TIM3->CR1 = ((uint16_t) 0x006C);        // Configure TIM3: buffer auto-reload, count up, stop on overflow, enable update events, interrupt on overflow only
    TIM3->PSC = myTIM3_PRESCALER;           // Set clock prescaler value
    TIM3->EGR |= ((uint16_t) 0x0001);       // Update timer registers
}

void myEXTI_Init() {
	SYSCFG->EXTICR[0] = (0x1 << 7);         // Sets EXTI 1 to PA1
	EXTI->RTSR = (0x1 << 1);                // Sets rising edge trigger TR1
	EXTI->IMR = (0x1 << 1);                 // Unmasks MR1 for interrupt line 1
	NVIC_SetPriority(EXTI0_1_IRQn, 0);      // Assign interrupt priority in NVIC
	NVIC_EnableIRQ(EXTI0_1_IRQn);           //Enables interrupt in NVIC
}

void delay(int milliSeconds){
    TIM3->CNT =  0x0;            	        // Clear timer
    TIM3->ARR =  milliSeconds;       		// Set wait time to milliseconds
    TIM3->EGR |= (uint16_t) 0x0001;         // Update registers
    TIM3->CR1 |= (uint16_t) 0x0001;         // Start timer
    while((TIM3->SR & 0x0001) == 0);   	    // Wait for timer to reach desired time interval
    TIM3->SR &= ~(TIM_SR_UIF);              // Clear update interrupt flag
    TIM3->CR1 &= ~0x0001;                   // Stop timer
}

void initDisplay()
{
	sendCommand(functionSet, FALSE);
	sendCommand(displayOn, FALSE);
	sendCommand(autoIncrement, FALSE);
	sendCommand(clear, FALSE);
}

void sendCommand(uint16_t cmd, int isData)
{
	if (isData == TRUE)                        // Send data with RS = 1 or a normal command
	{
		GPIOB->ODR = ((uint16_t) cmd << 8) | (0x1 << 5);
	}
	else
	{
		GPIOB->ODR = ((uint16_t) cmd << 8);
	}
	delay(20);
    GPIOB->ODR |= ((uint16_t) 0x1 << 4);       // Assert handshake enable
	delay(20);
	while(((GPIOB->IDR) &  GPIO_Pin_7) == 0);  // Wait for done to be asserted
	GPIOB->ODR &= ~((uint16_t) 0x1 << 4);      // De-assert enable
	while(((GPIOB->IDR) & GPIO_Pin_7) != 0);   // Wait for handshake done signal (from PB7)
	delay(20);
}


void runDisplay()
{
	sprintf(stringFreq,"%04d", (int)frequency);
	sprintf(stringRes,"%04d", (int)resistance);
	sendCommand(firstLine, FALSE);
	sendCommand(FChar, TRUE);
	sendCommand(semiColonChar, TRUE);
	for(int i = 0; i < 4; i++)
	{
		sendCommand(stringFreq[i], TRUE);
	}
	sendCommand(spaceChar, TRUE);
	sendCommand(HChar, TRUE);
	sendCommand(zChar, TRUE);
	sendCommand(secondLine, FALSE);
	sendCommand(RChar, TRUE);
	sendCommand(semiColonChar, TRUE);

	for(int i=0; i < 4 ; i++)
	{
		sendCommand(stringRes[i], TRUE);
	}
	sendCommand(spaceChar, TRUE);
	sendCommand(OChar, TRUE);
	sendCommand(hChar, TRUE);
	sendCommand(mChar, TRUE);
	sendCommand(sChar, TRUE);
}

#pragma GCC diagnostic pop
// ----------------------------------------------------------------------------
