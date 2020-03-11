// StepperMotorController.c starter file EE319K Lab 5
// Runs on TM4C123
// Finite state machine to operate a stepper motor.  
// Jonathan Valvano
// 1/17/2020

// Hardware connections (External: two input buttons and four outputs to stepper motor)
//  PA5 is Wash input  (1 means pressed, 0 means not pressed)
//  PA4 is Wiper input  (1 means pressed, 0 means not pressed)
//  PE5 is Water pump output (toggle means washing)
//  PE4-0 are stepper motor outputs 
//  PF1 PF2 or PF3 control the LED on Launchpad used as a heartbeat
//  PB6 is LED output (1 activates external LED on protoboard)

#include "SysTick.h"
#include "TExaS.h"
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

volatile int delay;

#define Rest	0			//define each state
#define Wiper1_1	1
#define Wiper2_1	2
#define Wiper4_1	3
#define Wiper8_1	4
#define Wiper16_1	5
#define Wiper1_2	6
#define Wiper2_2	7
#define Wiper4_2	8
#define Wiper8_2	9
#define Wiper16_2	10
#define Wiper8_3	11
#define Wiper4_3	12
#define Wiper2_3	13
#define Wiper1_3	14
#define Wiper16_4	15
#define Wiper8_4	16
#define Wiper4_4	17
#define Wiper2_4	18
#define Wiper1_4	19
#define Clean1_1	20
#define Clean2_1	21
#define Clean4_1	22
#define Clean8_1	23
#define Clean16_1	24
#define Clean1_2	25
#define Clean2_2	26
#define Clean4_2	27
#define Clean8_2	28
#define Clean16_2	29
#define Clean8_3	30
#define Clean4_3	31
#define Clean2_3	32
#define Clean1_3	33
#define Clean16_4	34
#define Clean8_4	35
#define Clean4_4	36
#define Clean2_4	37
#define Clean1_4	38
#define LED_OFF		0
#define LED_ON		64

typedef struct state {		//define the struct
		uint8_t Next[4];
		uint8_t LED;
		uint8_t output;
		uint8_t delay;
	} State_t;

	const State_t FSM[39] = {
		{/*Rest*/ {Rest, Wiper1_1, Clean1_1, Clean1_1}, LED_OFF, 0, 5},
		{/*Wiper1_1*/ {Wiper2_1, Wiper2_1, Clean2_1, Clean2_1}, LED_OFF, 1, 5},
		{/*Wiper2_1*/ {Wiper4_1, Wiper4_1, Clean4_1, Clean4_1}, LED_OFF, 2, 5},
		{/*Wiper4_1*/ {Wiper8_1, Wiper8_1, Clean8_1, Clean8_1}, LED_OFF, 4, 5},
		{/*Wiper8_1*/ {Wiper16_1, Wiper16_1, Clean16_1, Clean16_1}, LED_OFF, 8, 5},
		{/*Wiper16_1*/ {Wiper1_2, Wiper1_2, Clean1_2, Clean1_2}, LED_OFF, 16, 5},
		{/*Wiper1_2*/ {Wiper2_2, Wiper2_2, Clean2_2, Clean2_2}, LED_OFF, 1, 5},
		{/*Wiper2_2*/ {Wiper4_2, Wiper4_2, Clean4_2, Clean4_2}, LED_OFF, 2, 5},
		{/*Wiper4_2*/ {Wiper8_2, Wiper8_2, Clean8_2, Clean8_2}, LED_OFF, 4, 5},
		{/*Wiper8_2*/ {Wiper16_2, Wiper16_2, Clean16_2, Clean16_2}, LED_OFF, 8, 10},
		{/*Wiper16_2*/ {Wiper8_3, Wiper8_3, Clean8_3, Clean8_3}, LED_OFF, 16, 5},
		{/*Wiper8_3*/ {Wiper4_3, Wiper4_3, Clean4_3, Clean4_3}, LED_OFF, 8, 5},
		{/*Wiper4_3*/ {Wiper2_3, Wiper2_3, Clean2_3, Clean2_3}, LED_OFF, 4, 5},
		{/*Wiper2_3*/ {Wiper1_3, Wiper1_3, Clean1_3, Clean1_3}, LED_OFF, 2, 5},
		{/*Wiper1_3*/ {Wiper16_4, Wiper16_4, Clean16_4, Clean16_4}, LED_OFF, 1, 5},
		{/*Wiper16_4*/ {Wiper8_4, Wiper8_4, Clean8_4, Clean8_4}, LED_OFF, 16, 5},
		{/*Wiper8_4*/ {Wiper4_4, Wiper4_4, Clean4_4, Clean4_4}, LED_OFF, 8, 5},
		{/*Wiper4_4*/ {Wiper2_4, Wiper2_4, Clean2_4, Clean2_4}, LED_OFF, 4, 5},
		{/*Wiper2_4*/ {Wiper1_4, Wiper1_4, Clean1_4, Clean1_4}, LED_OFF, 2, 5},
		{/*Wiper1_4*/ {Rest, Wiper2_1, Clean2_1, Clean2_1}, LED_OFF, 1, 20},
		{/*Clean1_1*/ {Wiper2_1, Wiper2_1, Clean2_1, Clean2_1}, LED_ON, 0, 5},
		{/*Clean2_1*/ {Wiper4_1, Wiper4_1, Clean4_1, Clean4_1}, LED_ON, 2, 5},
		{/*Clean4_1*/ {Wiper8_1, Wiper8_1, Clean8_1, Clean8_1}, LED_OFF, 4, 5},
		{/*Clean8_1*/ {Wiper16_1, Wiper16_1, Clean16_1, Clean16_1}, LED_OFF, 8, 5},
		{/*Clean16_1*/ {Wiper1_2, Wiper1_2, Clean1_2, Clean1_2}, LED_ON, 16, 5},
		{/*Clean1_2*/ {Wiper2_2, Wiper2_2, Clean2_2, Clean2_2}, LED_ON, 1, 5},
		{/*Clean2_2*/ {Wiper4_2, Wiper4_2, Clean4_2, Clean4_2}, LED_OFF, 2, 5},
		{/*Clean4_2*/ {Wiper8_2, Wiper8_2, Clean8_2, Clean8_2}, LED_OFF, 4, 5},
		{/*Clean8_2*/ {Wiper16_2, Wiper16_2, Clean16_2, Clean16_2}, LED_ON, 8, 5},
		{/*Clean16_2*/ {Wiper8_3, Wiper8_3, Clean8_3, Clean8_3}, LED_ON, 16, 20},
		{/*Clean8_3*/ {Wiper4_3, Wiper4_3, Clean4_3, Clean4_3}, LED_OFF, 8, 5},
		{/*Clean4_3*/ {Wiper2_3, Wiper2_3, Clean2_3, Clean2_3}, LED_OFF, 4, 5},
		{/*Clean2_3*/ {Wiper1_3, Wiper1_3, Clean1_3, Clean1_3}, LED_ON, 2, 5},
		{/*Clean1_3*/ {Wiper16_4, Wiper16_4, Clean16_4, Clean16_4}, LED_ON, 1, 5},
		{/*Clean16_4*/ {Wiper8_4, Wiper8_4, Clean8_4, Clean8_4}, LED_OFF, 16, 5},
		{/*Clean8_4*/ {Wiper4_4, Wiper4_4, Clean4_4, Clean4_4}, LED_OFF, 8, 5},
		{/*Clean4_4*/ {Wiper2_4, Wiper2_4, Clean2_4, Clean2_4}, LED_ON, 4, 5},
		{/*Clean2_4*/ {Wiper1_4, Wiper1_4, Clean1_4, Clean1_4}, LED_ON, 2, 10},
		{/*Clean1_4*/ {Rest, Wiper2_1, Clean2_1, Clean2_1}, LED_OFF, 1, 5}
	};			//define the elements of each state

	uint8_t current = 0; //temp variables
	uint8_t input_t = 0;

void EnableInterrupts(void);
// edit the following only if you need to move pins from PA4, PE3-0      
// logic analyzer on the real board
#define PA4       (*((volatile unsigned long *)0x40004040))
#define PE50      (*((volatile unsigned long *)0x400240FC))
void SendDataToLogicAnalyzer(void){
  UART0_DR_R = 0x80|(PA4<<2)|PE50;
}

int main(void){ 
  TExaS_Init(&SendDataToLogicAnalyzer);    // activate logic analyzer and set system clock to 80 MHz
  SysTick_Init();   
// you initialize your system here
	SYSCTL_RCGCGPIO_R |= 0x33; //initialize each port used
	delay++;
	
	GPIO_PORTE_DIR_R |= 0x1F;
	GPIO_PORTE_DEN_R |= 0x1F;
	
	GPIO_PORTA_DIR_R &= 0xCF;
	GPIO_PORTA_DEN_R |= 0x30;
	
	GPIO_PORTB_DIR_R |= 0x40;
	GPIO_PORTB_DEN_R |= 0x40;
	
	GPIO_PORTF_DIR_R |= 0x02;
	GPIO_PORTF_DEN_R |= 0x02;
	
	
	EnableInterrupts();   
  while(1){
// output
// wait
// input
// next		
		GPIO_PORTF_DATA_R ^= 0x02; //toggle the heartbeat LED
		
		GPIO_PORTE_DATA_R = FSM[current].output; //output to the motor
		GPIO_PORTB_DATA_R = FSM[current].LED; //toggle the light when it is on
		
		SysTick_Wait10ms(FSM[current].delay); //delay for how ever long each state runs for
		
		input_t = (GPIO_PORTA_DATA_R)/16;		//take the inputs and shift them right
		current = FSM[current].Next[input_t]; //use the new value as the index of the next state graph
  }
}


