#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <wiringPi.h>

#include "EasyCAT.h"									 // EasyCAT library to interface     

#define LOBYTE(x) ((unsigned char) ((x) & 0xff))
#define HIBYTE(x) ((unsigned char) ((x) >> 8 & 0xff))

EasyCAT EASYCAT;										 // EasyCAT istantiation

unsigned short ContaUp;									 // used for sawthoot test generation
unsigned short ContaDown;								 //

unsigned long counter;

void callback() {
	counter++;
	memcpy(&EASYCAT.BufferIn.Byte[28], &counter, sizeof(counter));
	EASYCAT.MainTask();									// execute the EasyCAT task
}

int main()
{
	 char cValue;										 // used to read the output buffer 

	 printf("Size of counter: %d\n", sizeof(counter));

     ContaUp = 0x0000;                                   //
     ContaDown = 0x0000;                                 //
	counter=0;
	
	 //---- initialize the EasyCAT board -----

     if (EASYCAT.Init() == true)						 // initialization
     {
     	  printf("inizialized\n");							 // succesfully completed

		  if (wiringPiSetup () == -1)
			return -1;
		wiringPiISR(0, INT_EDGE_RISING, callback);
     }
     else											 	 // initialization failed   
     {							
       printf("inizialization failed\n");				 // the EasyCAT board was not recognized
	   return -1;
     }			
														 // In the main loop we must call ciclically the 
                                                         // EasyCAT task and our application
                                                         //
                                                         // This allows the bidirectional exachange of the data
                                                         // between the EtherCAT master and our application
                                                         //
                                                         // The EasyCAT cycle and the Master cycle are asynchronous
                                                         //     
		                                                 // The delay allows us to set the EasyCAT cycle time  
                                                         // according to the needs of our application
                                                         //
                                                         // For user interface applications a cycle time of 100mS,
                                                         // or even more, is appropriate, but, for data processing 
                                                         // applications, a faster cycle time may be required
                                                         //
                                                         // In this case we can also completely eliminate this
                                                         // delay in order to obtain the fastest possible response

	while (1)
	{  
		  // --- test sawtooth generation --- 

		  ContaUp++;											// we increment the variable ContaUp  
		  ContaDown--;											// and decrement ContaDown

																// we use these variables to create sawtooth,
																// with different slopes and periods, for
																// test pourpose, in input Bytes 2,3,4,5,30,31

		  EASYCAT.BufferIn.Byte[2] = LOBYTE(ContaUp);           // slow rising slope
   		  EASYCAT.BufferIn.Byte[3] = HIBYTE(ContaUp);           // extremly slow rising slope
    
		  EASYCAT.BufferIn.Byte[4] = LOBYTE(ContaDown);         // slow falling slope
   		  EASYCAT.BufferIn.Byte[5] = HIBYTE(ContaDown);         // extremly slow falling slope
    		
		  // EASYCAT.BufferIn.Byte[30] = LOBYTE(ContaUp) << 2;		// medium speed rising slope
		  // EASYCAT.BufferIn.Byte[31] = LOBYTE(ContaDown) << 2;	// medium speed falling slope    

		  // --- eight bits management ---

		  cValue = EASYCAT.BufferOut.Byte[0];					// we read the input bit status reading the first byte from output buffer

		  
//	 	  EASYCAT.MainTask();									// execute the EasyCAT task


			sleep(60);
//		  usleep(1000000);										// delay of 100mS
	}
}
