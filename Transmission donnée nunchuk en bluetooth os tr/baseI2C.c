/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "osObjects.h"                      // RTOS object definitions
#include "cmsis_os.h"                       // CMSIS RTOS header file
#include "Driver_I2C.h"                 // ::CMSIS Driver:I2C
#include "stdio.h"
#include "Board_GLCD.h"                 // ::Board Support:Graphic LCD
#include "GLCD_Config.h"
#include "GPIO.h" 
#include "LPC17xx.h"
#include "Driver_USART.h"
#include "stdlib.h"
#include <string.h>

#define SLAVE_I2C_ADDR       0xXX			// Adresse esclave sur 7 bits

void Thread1 (void const *argument);                             // thread function
osThreadId tid_Thread1;                                          // thread id
osThreadDef (Thread1, osPriorityNormal, 1, 0);                   // thread object

osMutexId mid_Thread_Mutex;                                     // mutex id
osMutexDef (SampleMutex);                                       // mutex name definition

extern ARM_DRIVER_I2C Driver_I2C2;
extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;
extern ARM_DRIVER_USART Driver_USART1;

uint8_t DeviceAddr;
char retour[6], tab[10], maValeur, chaine[20], chaine1[20];


void write1byte(unsigned char composant, unsigned char registre, unsigned char valeur);

void Init_I2C(void){   //initialisation de la transmission I2C
	Driver_I2C2.Initialize(NULL);
	Driver_I2C2.PowerControl(ARM_POWER_FULL);
	Driver_I2C2.Control(	ARM_I2C_BUS_SPEED,				// 2nd argument = débit
							ARM_I2C_BUS_SPEED_STANDARD  );	// 100 kHz
	Driver_I2C2.Control(	ARM_I2C_BUS_CLEAR,
							0 );
}

void Init_UART(void){
	Driver_USART1.Initialize(NULL);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(	ARM_USART_MODE_ASYNCHRONOUS |
							ARM_USART_DATA_BITS_8		|
							ARM_USART_STOP_BITS_1		|
							ARM_USART_PARITY_NONE		|
							ARM_USART_FLOW_CONTROL_NONE,
							115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX,1);
}


int main (void){
	osKernelInitialize ();
	
	Init_I2C();
	Init_UART();
	GLCD_Initialize();
  GLCD_ClearScreen();
  GLCD_SetFont(&GLCD_Font_16x24);
	
	DeviceAddr = 0x52;
	write1byte(DeviceAddr,0xf0,0x55); //init 1er registre
	write1byte(DeviceAddr,0xfb,0x00);	//init 2e registre

	tid_Thread1 = osThreadCreate (osThread(Thread1), NULL);
	
	mid_Thread_Mutex = osMutexCreate (osMutex (SampleMutex));
	
	osKernelStart ();
}
	
void Thread1 (void const *argument) {

	char InitCmd=0;
		
  while (1) {
    Driver_I2C2.MasterTransmit (DeviceAddr, &InitCmd, 1, false);// commande de conversion
		while (Driver_I2C2.GetStatus().busy == 1); // attente fin transmission
	
		Driver_I2C2.MasterReceive (0x52 ,retour, 6, false);		// recupération des data false = avec stop 
		while (Driver_I2C2.GetStatus().busy == 1);		// attente fin transmission
		
		//Affichage joystick axe X
		osMutexWait (mid_Thread_Mutex, osWaitForever);
		sprintf(chaine,"Joystick X = %03d", retour[0] ); 
		GLCD_DrawString(1,1,(unsigned char*)chaine);// affichage nunchuk
		sprintf(chaine,"\r\nX%03d\r\n", retour[0] );
		Driver_USART1.Send(chaine,strlen(chaine));
		
		//Affichage joystick axe Y
		sprintf(chaine,"Joystick Y = %03d",  retour[1]); 
		GLCD_DrawString(1,24,(unsigned char*)chaine);
		sprintf(chaine,"Y%03d\r\n", retour[1] );
		Driver_USART1.Send(chaine,strlen(chaine));
		osMutexRelease (mid_Thread_Mutex);
		
		//
		if ((retour[5]&0x01)==0x00)// analyser l'etat desv boutons C et Z
		{
				osMutexWait (mid_Thread_Mutex, osWaitForever);
				GLCD_DrawString(1,72,"Btn z : pressed");	
				Driver_USART1.Send("Z1",strlen("Z1"));					//envoi UART vers module BT maitre
				osMutexRelease (mid_Thread_Mutex);
		}
		else
		{
				osMutexWait (mid_Thread_Mutex, osWaitForever);
				GLCD_DrawString(1,72,"Btn z : release");
				Driver_USART1.Send("Z0",2);					//envoi UART vers module BT maitre
				osMutexRelease (mid_Thread_Mutex);
		}
		if ((retour[5]&0x02)==0x00)
		{
				osMutexWait (mid_Thread_Mutex, osWaitForever);
				GLCD_DrawString(1,96,"Btn c : pressed");
				Driver_USART1.Send("C1",2);					//envoi UART vers module BT maitre
				osMutexRelease (mid_Thread_Mutex);
		}
		else
		{
				osMutexWait (mid_Thread_Mutex, osWaitForever);
				GLCD_DrawString(1,96,"Btn c : release");
				Driver_USART1.Send("C0",2);					//envoi UART vers module BT maitre
				osMutexRelease (mid_Thread_Mutex);
		}
	}
}


void write1byte(unsigned char composant, unsigned char registre, unsigned char valeur) fonction d'ecriture I2C
{
	char tab[2];
		
	tab[0] = registre;
	tab[1] = valeur;
	
	Driver_I2C2.MasterTransmit (composant, tab, 2, false);// ecriture
	while (Driver_I2C2.GetStatus().busy == 1);// attente fin transmission
}
