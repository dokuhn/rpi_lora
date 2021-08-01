/*******************************************************************************
 *
 * 
 *
 * 
 *
 *******************************************************************************/
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <stdlib.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "sx1276.h"


int readline(int fd, char *buf, int nbytes)
{
    int numread = 0;
    int returnval;
    while (numread < nbytes - 1)
    {
        returnval = read(fd, buf + numread, 1);
        if ((returnval == -1) && (errno == EINTR))
            continue;
        if ((returnval == 0) && (numread == 0))
            return 0;
        if (returnval == 0)
            break;
        if (returnval == -1)
            return -1;
        numread++;
        if (buf[numread - 1] == '\n')
        {
            buf[numread] = '\0';
            return numread;
        }
    }
    errno = EINVAL;
    return -1;
}





int main (int argc, char *argv[]) {

    int buffer_size = 256;
    char line_buffer[buffer_size];

    if (argc < 2) {
        printf ("Usage: argv[0] sender|rec [message]\n");
        fflush(stdout);
        exit(1);
    }

    wiringPiSetup () ;
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);

    wiringPiSPISetup(CHANNEL, 500000);

    SetupLoRa();


    if (!strcmp("sender", argv[1])) {
        opmodeLora();
        // enter standby mode (required for FIFO loading))
        opmode(OPMODE_STANDBY);

        writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

        configPower(23);

        printf("Send packets at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
        printf("------------------\n");
	fflush(stdout);

        //if (argc > 2)
        //    strncpy((char *)hello, argv[2], sizeof(hello));

        while( readline(STDIN_FILENO, line_buffer, sizeof(line_buffer)) > 1 ){

            txlora((byte*)line_buffer, (byte)strlen(line_buffer));
            delay(500);
        }

    } else {

        // radio init
        opmodeLora();
        opmode(OPMODE_STANDBY);
        opmode(OPMODE_RX);
        printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
        printf("------------------\n");
        fflush(stdout);

	void (*fun_ptr2receivePackageISR)(void) = &isr_handler;

	wiringPiISR(dio0, INT_EDGE_RISING, fun_ptr2receivePackageISR);

        while(1) {
            delay(5000);
        }

    }

    return (0);
}
