#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <wiringPi.h>

#include "EasyCAT.h"

//#define LOBYTE(x) ((unsigned char) ((x) & 0xff))
//#define HIBYTE(x) ((unsigned char) ((x) >> 8 & 0xff))

EasyCAT EASYCAT;

unsigned long counter;

void callback() {
    counter++;
    memcpy(&EASYCAT.BufferIn.Byte[28], &counter, sizeof(counter));
    EASYCAT.MainTask();
}

int main() {
    counter = 0;

    if (EASYCAT.Init()) {
        if (wiringPiSetup() == -1)
            return -1;

        wiringPiISR(0, INT_EDGE_RISING, callback);

        printf("Init done\n");
    } else                                                 // initialization failed
    {
        printf("Init failed!\n");                 // the EasyCAT board was not recognized
        return -1;
    }

    while (1) {
        // we just loop forever because we've set the callback to handle the maintask
        sleep(60);
    }
}
