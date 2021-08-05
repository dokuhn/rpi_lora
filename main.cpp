/*******************************************************************************
 *
 * 
 *
 * 
 *
 *******************************************************************************/

extern "C" {

    #include <errno.h>
    #include <stdio.h>
    #include <unistd.h>
    #include <string.h>
    #include <sys/types.h>
    #include <sys/time.h>
    #include <stdlib.h>

    #include <wiringPi.h>
    #include <wiringPiSPI.h>
}

#include "sx1276.hpp"




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


extern "C" void isr_handler_wrapper(sx1276* ptr2sx1276Inst)
{
    ptr2sx1276Inst->isr_handler();

}



int main (int argc, char *argv[]) {

    int buffer_size = 256;
    char line_buffer[buffer_size];

    sx1276 sx1276Inst;

    if (argc < 2) {
        printf ("Usage: argv[0] sender|rec [message]\n");
        fflush(stdout);
        exit(1);
    }

    wiringPiSetup () ;
    pinMode(sx1276Inst.ssPin, OUTPUT);
    pinMode(sx1276Inst.dio0, INPUT);
    pinMode(sx1276Inst.RST, OUTPUT);

    wiringPiSPISetup(sx1276Inst.CHANNEL, 500000);

    sx1276Inst.SetupLoRa();


    if (!strcmp("sender", argv[1])) {
        sx1276Inst.opmodeLora();
        // enter standby mode (required for FIFO loading))
        sx1276Inst.opmode(OPMODE_STANDBY);

        sx1276Inst.writeReg(RegPaRamp, (sx1276Inst.readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

        sx1276Inst.configPower(23);

        printf("Send packets at SF%i on %.6lf Mhz.\n", sx1276Inst.sf,(double)sx1276Inst.freq/1000000);
        printf("------------------\n");
	    fflush(stdout);

        //if (argc > 2)
        //    strncpy((char *)hello, argv[2], sizeof(hello));

        while( readline(STDIN_FILENO, line_buffer, sizeof(line_buffer)) > 1 ){

            sx1276Inst.txlora((byte*)line_buffer, (byte)strlen(line_buffer));
            delay(500);
        }

    } else {

        // radio init
        sx1276Inst.opmodeLora();
        sx1276Inst.opmode(OPMODE_STANDBY);
        sx1276Inst.opmode(OPMODE_RX);
        printf("Listening at SF%i on %.6lf Mhz.\n", sx1276Inst.sf,(double)sx1276Inst.freq/1000000);
        printf("------------------\n");
        fflush(stdout);

	    // void (*fun_ptr2isr_handler)(SX1276* ptr2sx1276Inst) = &isr_handler_wrapper;

	    // wiringPiISR(sx1276Inst.dio0, INT_EDGE_RISING, fun_ptr2isr_handler);

        while(1) {
            sx1276Inst.receivepacket(); 
            delay(1);
        }

    }

    return (0);
}
