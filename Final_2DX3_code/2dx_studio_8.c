/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;
int step=0;
volatile int button0 = 0; //scan
volatile int button1 = 0; //produce graph
uint8_t byteData, sensorState = 0;
uint16_t wordData;
uint16_t Distance;
uint8_t dataReady;

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
//    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
    I2C0_MTPR_R = 6;                                        						// 8) configure for 100 kbps clock
        
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// Port H bits 0 to 3
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};      // Allow time for clock to stabilize

	GPIO_PORTH_DIR_R = GPIO_PORTH_DIR_R | 0b00001111;            // Enable PH0 to PH3 as inputs
	GPIO_PORTH_AFSEL_R &= ~0x0F;     								// disable alt funct on Port H pins (PH0-PH3)
	GPIO_PORTH_DEN_R |= 0b00001111;            // Enable PH0 to PH3 digital pins
	GPIO_PORTH_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port H	pins (PH0-PH3)	
	return;
}

void PortE_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;                 // Activate the clock for Port E
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0){};      // Allow time for clock to stabilize

	GPIO_PORTE_DIR_R = GPIO_PORTE_DIR_R | 0b00000001;            // Enable PE0 as inputs
	GPIO_PORTE_AFSEL_R &= ~0x01;     								// disable alt funct on Port E pins (PH0-PH3)
	GPIO_PORTE_DEN_R |= 0b00000001;            // Enable PE0 digital pins
	GPIO_PORTE_AMSEL_R &= ~0x01;     								// disable analog functionality on Port E
	return;
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortM_Init(void){
	//Use PortM pins (PM0-PM1) for output (External Buttons)
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R &= ~0x03;        								// configure Port M pins (PM0-PM1) as inputs
  GPIO_PORTM_AFSEL_R &= ~0x03;     								// disable alt funct on Port M pins (PM0-PM1)
  GPIO_PORTM_DEN_R |= 0x03;        								// enable digital I/O on Port M pins (PM0-PM1)
																									// configure Port M as GPIO
	GPIO_PORTM_PCTL_R &= ~0x000000FF;	
  GPIO_PORTM_AMSEL_R &= ~0x03;     								// disable analog functionality on Port M	pins (PM0-PM1)	
	return;
}

void PortM_Interrupt_Init(void){

		GPIO_PORTM_IS_R &= ~0x03;     						// (Step 1) PM1/PM0 is Edge-sensitive 
		GPIO_PORTM_IBE_R &= ~0x03;    						//     			PM1/PM0 is not triggered by both edges 
		GPIO_PORTM_IEV_R &= ~0x03;    						//     			PM1/PM0 is falling edge event 
		GPIO_PORTM_ICR_R = 0x03;    					// 					Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTM_IM_R |= 0x03;      					// 					Arm interrupt on PM1/PM0 by setting proper bit in IM register
    
		NVIC_EN2_R = 0x00000100;            					// (Step 2) Enable interrupt 72 in NVIC (which is in Register EN1)
	
		NVIC_PRI18_R = (NVIC_PRI18_R & 0xFFFFFF00) | 0x000000A0;  									// (Step 4) Set interrupt priority to 5

		EnableInt();																	// (Step 3) Enable Global Interrupt. lets go!
}

void GPIOM_IRQHandler(void){
    uint32_t status = GPIO_PORTM_MIS_R;

    GPIO_PORTM_ICR_R = status;   // clear whatever fired

    if(status & 0x01){
        button0 = 1;
    }

    if(status & 0x02){
        button1 = 1;
    }

}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void rotateStepCW(void){
    uint32_t delay = 100000;

    for(int i = 0; i < 16; i++){
        GPIO_PORTH_DATA_R = 0b00000011;
        SysTick_Wait(delay);

        GPIO_PORTH_DATA_R = 0b00000110;
        SysTick_Wait(delay);

        GPIO_PORTH_DATA_R = 0b00001100;
        SysTick_Wait(delay);

        GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait(delay);
    }
}

void returnHome(void){
    uint32_t delay = 100000;

    for(int i = 0; i < 512; i++){
        GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait(delay);

        GPIO_PORTH_DATA_R = 0b00001100;
        SysTick_Wait(delay);

        GPIO_PORTH_DATA_R = 0b00000110;
        SysTick_Wait(delay);

        GPIO_PORTH_DATA_R = 0b00000011;
        SysTick_Wait(delay);
			  GPIO_PORTF_DATA_R ^= 0b00010000;			
    }
}

void full_scan() {
	status = VL53L1X_StartRanging(dev);
	Status_Check("StartRanging", status);

	UART_printf("SCAN_START\r\n");
	UART_printf("Angle,Distance\r\n");
	for(step = 0; step < 32; step++) {
				dataReady = 0;
				

        while(dataReady == 0) {
            status = VL53L1X_CheckForDataReady(dev, &dataReady);
            VL53L1_WaitMs(dev, 5);
        }

        status = VL53L1X_GetDistance(dev, &Distance);

        status = VL53L1X_ClearInterrupt(dev);
				FlashLED2(1);

        sprintf(printf_buffer, "%f,%u\r\n", (step*11.25), Distance);
        UART_printf(printf_buffer);

        FlashLED1(1);
        SysTick_Wait10ms(10);


        rotateStepCW();
        SysTick_Wait10ms(20);
    }

    status = VL53L1X_StopRanging(dev);
    Status_Check("StopRanging", status);

    UART_printf("SCAN_END\r\n");

    returnHome();
    button0 = 0;
}

void clock_Check(){
		GPIO_PORTE_DATA_R ^= 0b00000001;
		SysTick_Wait10ms(1);
}

//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************

int main(void) {

    PLL_Init();
    SysTick_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();
    PortH_Init();
    PortG_Init();
		PortE_Init();
		PortM_Init();
		PortM_Interrupt_Init();		

    UART_printf("Program Begins\r\n");

    VL53L1X_XSHUT();
    SysTick_Wait10ms(10);

    status = VL53L1X_GetSensorId(dev, &wordData);
    sprintf(printf_buffer, "Sensor ID: 0x%X\r\n", wordData);
    UART_printf(printf_buffer);

    while(sensorState == 0) {
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10);
    }

    FlashAllLEDs();

    status = VL53L1X_ClearInterrupt(dev);

    status = VL53L1X_SensorInit(dev);
    Status_Check("SensorInit", status);

    status = VL53L1X_SetDistanceMode(dev, 2);
    Status_Check("SetDistanceMode", status);

    status = VL53L1X_SetTimingBudgetInMs(dev, 100);
    Status_Check("SetTimingBudgetInMs", status);

    status = VL53L1X_SetInterMeasurementInMs(dev, 150);
    Status_Check("SetInterMeasurementInMs", status);

    while(1) {
			//WaitForInt();									
			// Call WaitForInt() function to wait for interrupts
			clock_Check(); 								//clock pin is PE0
			if(button0 == 1) {
				full_scan();
			}
			
			if(button1 == 1) {
					UART_printf("STOP_SCAN\r\n");
					button1 = 0;
			}
    }

}
