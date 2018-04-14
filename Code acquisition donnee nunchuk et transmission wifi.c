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


void Init_WiFi(void);																			
void sendCommand(char * command, int tempo_ms);

void Thread1 (void const *argument);                             // thread function
osThreadId tid_Thread1;                                          // thread id
osThreadDef (Thread1, osPriorityNormal, 1, 0);                   // thread object
                   

void Thread_R (void const *argument);                             // thread function Receive
osThreadId tid_Thread_R;                                          // thread id
osThreadDef (Thread_R, osPriorityNormal, 1, 0);                   // thread object

osMutexId mid_Thread_Mutex;                                     // mutex id
osMutexDef (SampleMutex);                                       // mutex name definition

extern ARM_DRIVER_I2C Driver_I2C2;
extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;
extern ARM_DRIVER_USART Driver_USART1;

uint8_t DeviceAddr;
char retour[6], tab[10], maValeur, chaine[20], chaine1[20];


void write1byte(unsigned char composant, unsigned char registre, unsigned char valeur);// prototype de la fonction écriture I2C
void Init_I2C(void){   //initialisation de la transmission I2C
	Driver_I2C2.Initialize(NULL);
	Driver_I2C2.PowerControl(ARM_POWER_FULL);
	Driver_I2C2.Control(	ARM_I2C_BUS_SPEED,				// 2nd argument = débit
				ARM_I2C_BUS_SPEED_STANDARD  );	// 100 kHz
	Driver_I2C2.Control(	ARM_I2C_BUS_CLEAR,
							0 );
}
//fonction de CB lancee si l'on a reçu une trame  ou envoye une trame symbolisé par des  Events T ou R
void event_UART(uint32_t event)
{
	switch (event) {
		
		case ARM_USART_EVENT_RECEIVE_COMPLETE : 	osSignalSet(tid_Thread_R, 0x01);
																							break;
		
		case ARM_USART_EVENT_SEND_COMPLETE  : 	osSignalSet(tid_Thread1, 0x02);
																							break;
		
		default : break;
	}
}
void Init_UART(void){  // fonction initialisation de l'UART
	Driver_USART1.Initialize(event_UART);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(	ARM_USART_MODE_ASYNCHRONOUS |
							ARM_USART_FLOW_CONTROL_NONE   |
							ARM_USART_DATA_BITS_8		|
							ARM_USART_STOP_BITS_1		|
							ARM_USART_PARITY_NONE		,							
							115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX,1);
}



int main (void){
	osKernelInitialize ();
	
	Init_I2C();// initialisation de l'I2C
	Init_UART();// initialisation de l'UART
	
	GLCD_Initialize();
        GLCD_ClearScreen();
        GLCD_SetFont(&GLCD_Font_6x8);
	
	DeviceAddr = 0x52;								//Adresse de la nunchuk noire
	write1byte(DeviceAddr,0xf0,0x55); //init 1er registre
	write1byte(DeviceAddr,0xfb,0x00);	//init 2e registre

	tid_Thread1 = osThreadCreate (osThread(Thread1), NULL); // création tache 
	tid_Thread_R = osThreadCreate (osThread(Thread_R), NULL);// création tache reception des réponses du module wifi
	//mid_Thread_Mutex = osMutexCreate (osMutex (SampleMutex));
	
	osKernelStart ();// lancer horloge TR
	osDelay(osWaitForever);//mise en sommeil du main
}
	
void Thread1 (void const *argument) {

	char InitCmd=0;
		Init_WiFi();// initialisation du module wifi
  while (1) {
    Driver_I2C2.MasterTransmit (DeviceAddr, &InitCmd, 1, false);// commande de conversion
		while (Driver_I2C2.GetStatus().busy == 1); // attente fin transmission
	
		Driver_I2C2.MasterReceive (0x52 ,retour, 6, false);		// recupération des data false = avec stop 
		while (Driver_I2C2.GetStatus().busy == 1);		// attente fin transmission
		
		//Affichage joystick axe X

		//osMutexWait (mid_Thread_Mutex, osWaitForever);

		sprintf(chaine,"N%03d%03d", retour[0], retour[1] ); 			//création de la chaine envoyée
		 sendCommand("AT+CIPSEND=7\r\n",500); 		//nombre d'cotets à envoyer 
                 sendCommand(chaine,500);                            //envoyé la chaine 
		
                //GLCD_DrawString(1,1,(unsigned char*)chaine);// affichage des données de la nunchuk
                //sprintf(chaine,"\r\nX%03d\r\n", retour[0] );
		
		
		//Affichage joystick axe Y
	     //	sprintf(chaine,"Joystick Y = %03d",  retour[1]); 
		//GLCD_DrawString(1,24,(unsigned char*)chaine);
		//sprintf(chaine,"Y%03d\r\n", retour[1] );
	     //	Driver_USART1.Send(chaine,strlen(chaine));
		//osMutexRelease (mid_Thread_Mutex);
		
		//
//		if ((retour[5]&0x01)==0x00) // recuperation des valeurs du bouton C et Z de la nunchuk non utilise dans le projet
//		{
//				osMutexWait (mid_Thread_Mutex, osWaitForever);
//				GLCD_DrawString(1,72,"Btn z : pressed");	
//				Driver_USART1.Send("Z1",strlen("Z1"));					//envoi UART vers module BT maitre non utilise
//				osMutexRelease (mid_Thread_Mutex);
//		}
//		else
//		{
//				osMutexWait (mid_Thread_Mutex, osWaitForever);
//				//GLCD_DrawString(1,72,"Btn z : release");
//				Driver_USART1.Send("Z0",2);					//envoi UART vers module BT maitre non utilise
//				osMutexRelease (mid_Thread_Mutex);
//		}
//		if ((retour[5]&0x02)==0x00)
//		{
//				osMutexWait (mid_Thread_Mutex, osWaitForever);
//				//GLCD_DrawString(1,96,"Btn c : pressed");
//				Driver_USART1.Send("C1",2);					//envoi UART vers module BT maitre non utilise
//				osMutexRelease (mid_Thread_Mutex);
//		}
//		else
//		{
//				osMutexWait (mid_Thread_Mutex, osWaitForever);
//				//GLCD_DrawString(1,96,"Btn c : release");
//				Driver_USART1.Send("C0",2);					//envoi UART vers module BT maitre non utilise
//				osMutexRelease (mid_Thread_Mutex);
//		}
	}
}
void Thread_R (void const *argument) { // tache permettant d'afficher les réponses du module wifi 

	char RxChar;
	int ligne;
	int i=0;	// i pour position colonne caractère
	char RxBuf[200];
	
  while (1) {
		Driver_USART1.Receive(&RxChar,1);		// A mettre ds boucle pour recevoir 
		osSignalWait(0x01, osWaitForever);	// sommeil attente reception
		
		RxBuf[i]=RxChar;
		i++;
		//Suivant le caractère récupéré
		switch(RxChar)
		{
			case 0x0D: 		//Un retour chariot? On ne le conserve pas...
				i--;
				break;
			case 0x0A:										//Un saut de ligne?
				RxBuf[i-1]=0;											//=> Fin de ligne, donc, on "cloture" la chaine de caractères
				GLCD_DrawString(1,ligne,RxBuf);	//On l'affiche (peut etre trop long, donc perte des caractères suivants??)
				ligne+=10;										//On "saute" une ligne de l'afficheur LCD
			  if(ligne>240)
				{
					ligne=1;
					GLCD_ClearScreen();
					osDelay(2000);
				}
				i=0;													//On se remet au début du buffer de réception pour la prochaine ligne à recevoir
				break;
		}
  }
}


void write1byte(unsigned char composant, unsigned char registre, unsigned char valeur) // fonction permettant de réaliser une ecriture en I2C avec la nunchuk
{
	char tab[2];
		
	tab[0] = registre;
	tab[1] = valeur;
	
	Driver_I2C2.MasterTransmit (composant, tab, 2, false);// ecriture
	while (Driver_I2C2.GetStatus().busy == 1);// attente fin transmission
}

void Init_WiFi(void) // fonction d'initialisation du module wifi
{
	// reset module
	sendCommand("AT+RST\r\n",7000); 
//	
//	// disconnect from any Access Point
//	sendCommand("AT+CWQAP\r\n",4000); 
////	
	sendCommand("AT+CWMODE=3\r\n",7000);
////	
//  // configure as Station 
	sendCommand("AT+CWJAP=\"IT2R1\",\"It2r2018\"\r\n",7000);
//	
	sendCommand("AT+CIFSR\r\n",2000);
	
	//Connect to YOUR Access Point
	sendCommand("AT+CIPSTART=\"TCP\",\"192.168.0.100\",333\r\n",7000); 
	
}

void sendCommand(char * command, int tempo_ms) // fonction permettant d'envoyer les commandes au module wifi
{
	int len;
	len = strlen (command);
	Driver_USART1.Send(command,len); // send the read character to the esp8266
	osSignalWait(0x02, osWaitForever);		// sommeil fin emission
	osDelay(tempo_ms);		// attente traitement retour
}
