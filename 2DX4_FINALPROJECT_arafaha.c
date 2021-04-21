/* Name: Abd Elelah Arafah
mac id: arafaha
student number: 400197623
*/


#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Data Acknowledge
#define I2C_MCS_ADRACK          0x00000004  // Address Acknowledge
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up

uint16_t	dev=0x52;
int status=0;
volatile int IntCount;
#define isInterrupt 1 //device working in interrupt mode

void UART_Init(void);
uint16_t debugArray[100];
void PortE0_Init(void){//Row pin
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // activate the clock for Port E
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){}; // allow time for clock to stabilize
 GPIO_PORTE_DEN_R = 0b00000001; // Enabled as digital outputs
 GPIO_PORTE_DIR_R = 0b00000001;
return;
}
void PortL4_Init(void) { //external LED that shuts off when motor is on and is on when the motor is off
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10; //activate clock for Port L
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){}; // allow time for clock to stabilize
	GPIO_PORTL_DEN_R = 0b00010000; // Enabled both as digital outputs
	GPIO_PORTL_DIR_R = 0b00010000;
return;
}
void PortM0_Init(void){//column pin
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; //activate the clock for Port M
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){}; //allow time for clock to stabilize
	GPIO_PORTM_DIR_R = 0b00000000; // make Pins input
	GPIO_PORTM_DEN_R = 0b00000001;
return;
}

void PortN2N3N4N5_Init(void){//The pins used for motor
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; //activate the clock for Port N
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
	GPIO_PORTN_DIR_R=0b00111100; //make pins intput
	GPIO_PORTN_DEN_R=0b00111100;
return;
}

void PortF0_Init(void){//Student unique D4 on-board LED
 SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; //activate the clock for Port F
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};
	GPIO_PORTF_DIR_R=0b00000001; //make pins output
	GPIO_PORTF_DEN_R=0b00000001;
return;
}

void Clockwise(int Delay){//allows for fullstep clockwise motion of the motor
	SysTick_Wait10ms(Delay);
	GPIO_PORTN_DATA_R = 0b00110000;
	SysTick_Wait10ms(Delay);
	GPIO_PORTN_DATA_R = 0b00011000;
	SysTick_Wait10ms(Delay);
	GPIO_PORTN_DATA_R = 0b00001100;
	SysTick_Wait10ms(Delay);
	GPIO_PORTN_DATA_R = 0b00100100;
}
void I2C_Init(void)
{
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// stablization
    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only
    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;// 9) master function enablation
    I2C0_MTPR_R = 0b0000000000000101000000000111011;// 8) configure for 100 kbps clock
}

//The tof sensor's reset is using XSHUT (PG0)
void PortG_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;// activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};// allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;// make HiZ
    GPIO_PORTG_AFSEL_R &= ~0x01;// disable alt funct on PG0
    GPIO_PORTG_DEN_R |= 0x01;// enable digital I/O on PG0
    // configure PG0 as GPIO
    GPIO_PORTG_AMSEL_R &= ~0x01;// disable analog functionality on PN0
    return;
}


void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;// make the PG0 pin outp.
    GPIO_PORTG_DATA_R &= 0b11111110;//PG0 == 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;// make PG0 input as HiZ
    
}

int main(void) 
{
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;//tof measurements variable initialization
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;// This is the main measurement utilized
  uint8_t dataReady;
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PLL_Init();
	SysTick_Init();
	PortE0_Init();//Function call ---> Row pin
	PortM0_Init();//Function call ---> Column pin
	PortF0_Init();//Function call ---> D4 on board pin
	PortN2N3N4N5_Init();//Function call ---> Motor rotation pins
	PortL4_Init(); //Function call ---> External LED pins
	int counter = 0; //counter used to recognize the angle reached by motor
	int rotations = 0; //counts the number of 360 turns / plane visualization
	

	UART_printf("Program Starts\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"Abd Elelah's Project code...%d\r\n",mynumber);
	UART_printf(printf_buffer);
	
  /*basic I2C read functions to check the I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);// The model ID!
  myByteArray[i++] = byteData;
  status = VL53L1_RdByte(dev, 0x0110, &byteData);// The module type!
  myByteArray[i++] = byteData;
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);
	sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);//prints specific addresses of model and module of tof
	UART_printf(printf_buffer);
	
	// BOOTS THE TOF SENSOR UP
	while(sensorState==0)
	{
		status = VL53L1X_BootState(dev, &sensorState);//boots sensor
		SysTick_Wait10ms(3);
  }
	FlashAllLEDs(); //flashes all LEDs when sensor is booted
	UART_printf("The ToF Chip Booted!!!\r\n");
	UART_printf("Please wait..........\r\n");
	status = VL53L1X_ClearInterrupt(dev); // clear interrupt
  //functions called to set up the sensor with default settings
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
  status = VL53L1X_StartRanging(dev);   // Enables ranging
	Status_Check("StartRanging", status);
   
	while(1)//pooling
	{

  //Row 1
  GPIO_PORTE_DATA_R =  0b00000000;
  GPIO_PORTL_DATA_R =  0b00010000;
	
  while((GPIO_PORTM_DATA_R&0b00000001)==0)
	{//Checks when button is pressed
	
	  GPIO_PORTL_DATA_R =  0b00000000;

	  for(int i = 0; i < 512; i++)//knowing that there is 512 steps utilized (*4*8 in the Clockwise function - Aka 4096 total) steps for motor total in one rotation
	  {
			  if((counter + 1)%8 == 0)//checks whenever 5.625 degrees of rotation is reached
				{
						GPIO_PORTF_DATA_R =  0b00000001;//Flashes the on-board LED
	          SysTick_Wait10ms(5);
	          GPIO_PORTF_DATA_R =  0b00000000;
					  while (dataReady == 0)
						{
							status = VL53L1X_CheckForDataReady(dev, &dataReady);//ready for measurements!!
							VL53L1_WaitMs(dev, 5);
	          }

            dataReady = 0;
	          status = VL53L1X_GetDistance(dev, &Distance);//takes a measuremnt each 5.625 degree
	          status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
            sprintf(printf_buffer,"%u,\n",Distance);//sends measurement to UART
            UART_printf(printf_buffer);
						SysTick_Wait10ms(5);
				}
				Clockwise(5);//Rotates the motor in clockwise direction
				counter++;
		}
		if (rotations== 4)//when 5 360s are reached stop ranging with the tof sensor
		{
					status = VL53L1X_StopRanging(dev);//api function - stops ranging
		}
		rotations++;
	 }
  }
	
}
