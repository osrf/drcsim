/**
 * \file parametertest.c
 *
 * get and set motor parameters
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include "../../handle_lib/include/handle_lib/handleTypes.hpp"
#include "../../handle_lib/include/handle_lib/packetParser.hpp"

//#define BAUDRATE B115200
#define BAUDRATE B1152000

#define MODEMDEVICE "/dev/ttyS0"

#define BUFFER 50



volatile bool run = true;
void sigint_handler(int s)
{
    run = false;
};

int main(int argc, char** argv)
{
    int fd; // serial file descriptor

    printf("setting up signal handler\n");
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART; //required or segfault on CTRL-C
    if (sigaction(SIGINT, &sa, NULL) == -1) 
    {
        perror("sigaction");
        exit(1);
    }

    printf("trying to open\n");
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY); 
    if (fd <0 ) 
    {
        printf("cannot open device\n");
        perror(MODEMDEVICE); 
        exit(-1); 
    }
    printf("device open\n");
    

    /////////////////////////////////
    struct termios options;
    tcgetattr(fd, &options);
    cfmakeraw(&options);
    cfsetspeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    
    // 8 data bits, no parity, 1 stop bit
    options.c_cflag &= ~PARENB; // no parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit, not 2
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag &= ~CRTSCTS; // no CTS/RTS hardware flow control

    options.c_cc[VTIME]    = 3;  // 0.3 second timeout
    options.c_cc[VMIN]     = 0;  // don't block for a number of chars

    tcflush(fd, TCIOFLUSH); // flushes both directions

    tcsetattr(fd, TCSANOW, &options);

    //////////////////////////////

    unsigned char send_buff[COMMAND_PACKET_LENGTH];
    unsigned char recv_buff[BUFFER];
    char read_buff[20];

    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    memset(recv_buff, 0, BUFFER);
    memset(read_buff, 0, 20);

    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, DATA_COLLECTION_MOTORHALL_BITMASK); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    int rsp = readResponse(fd);
    if (rsp != 0)
        printf("WARNING: stop collection returned: %02X\n", rsp);

    int parameter = 0;
    int board = 0;
    int action = 0;
    
    while (run)
    {
        printf("\n\n");
        printf("-------------------------------\n");
        printf(" [1] Get\n");
        printf(" [2] Set\n");
        printf("[99] !EXIT!\n");
        printf("Action: ");
        fgets(read_buff, 20, stdin);
        action = atoi(read_buff);

        if (action == 99)
            break;
        
        printf("\n");
        if (action == 1)
            printf(" [] All parameters\n");
        printf(" [0] K_pT\n");
        printf(" [1] K_iT\n");
        printf(" [2] K_dT\n");
        printf(" [3] K_pV\n");
        printf(" [4] K_iV\n");
        printf(" [5] K_dV\n");
        printf(" [6] K_pP\n");
        printf(" [7] K_iP\n");
        printf(" [8] K_dP\n");
        printf(" [9] R_Tw0\n");
        printf("[10] R_th1\n");
        printf("[11] T_L+\n");
        printf("[12] T_L-\n");
        printf("[13] Tau_W\n");
        printf("[14] T_MAX\n");
        printf("[15] alpha_Cu\n");
        printf("[16] t_off\n");
        printf("[17] T_TARGET\n");
        printf("[18] RPM_MAX\n");
        printf("[19] RSV\n");
        printf("[20] V_MAX\n");
        printf("[99] !EXIT!\n");
        printf("Parameter: ");
        fgets(read_buff, 20, stdin);

        int from_p;
        int to_p;

        if (read_buff[0] == '\n')
        {
            parameter = -1;
            from_p = 0;
            to_p = 21;
        }
        else
        {
            parameter = atoi(read_buff);
            from_p = parameter;
            to_p = parameter+1;
        }
        
        if (parameter == 99)
            break;
        
        float setvalue = 0;
        if (action == 2)
        {
            printf("\n");
            printf("Value: ");
            fgets(read_buff, 20, stdin);
            setvalue = atof(read_buff);
        }
        
        printf("\n");
        printf(" [1] Motor 1\n");
        printf(" [2] Motor 2\n");
        printf(" [3] Motor 3\n");
        printf(" [4] Motor 4\n");
        printf("[99] !EXIT!\n");
        printf("Board: ");
        fgets(read_buff, 20, stdin);
        board = atoi(read_buff);
    
        if (board == 99)
            break;
    
        int destination;
        int responding;
        switch (board)
        {
            case 1: destination = DESTINATION_MOTOR1; responding=RESPONDING_DEVICES_FIRST_MOTOR1_BITMASK; break;
            case 2: destination = DESTINATION_MOTOR2; responding=RESPONDING_DEVICES_FIRST_MOTOR2_BITMASK; break;
            case 3: destination = DESTINATION_MOTOR3; responding=RESPONDING_DEVICES_SECOND_MOTOR1_BITMASK; break;
            case 4: destination = DESTINATION_MOTOR4; responding=RESPONDING_DEVICES_SECOND_MOTOR2_BITMASK; break;
            default:
                printf("UNKNOWN DESTINATION\n");
                board = 99;
        }

        if (board == 99)
            break;
        
        printf("\n");
        
        for (int i=from_p; i<to_p; i++)
        {
            memset(send_buff, 0, COMMAND_PACKET_LENGTH);
            memset(recv_buff, 0, BUFFER);
            
            if (action == 1)
            {
                send_buff[DESTINATION_HEADER_OFFSET] = destination;
                send_buff[COMMAND_OFFSET] = MOTOR_PARAMETER_RE_L_OPCODE | i;
                //setPayload(send_buff, sensor);
                send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
                write(fd, send_buff, COMMAND_PACKET_LENGTH);
        
                printf("Param: %d  Sent: ", i);
                for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
                    printf("%02X ", send_buff[i]);

                int readlen = 0;
                int code = readResponse(fd, recv_buff, BUFFER, readlen);
            
                printf("  Got: ");
                for (int i=0; i<readlen; i++)
                    printf("%02X ", recv_buff[i]);
                printf("  :  ");
            
                if (code != 0)
                {
                    printf("ERROR %02X\n", code);
                }
                else
                {
                    float val = parseParameter(&(recv_buff[RESPONSE_PAYLOAD_OFFSET]));
                    printf("%f\n", val);
                }
            }
            else if (action == 2)
            {
                send_buff[DESTINATION_HEADER_OFFSET] = destination;
                send_buff[COMMAND_OFFSET] = MOTOR_PARAMETER_WR_L_OPCODE | parameter;
                setPayloadf(send_buff, setvalue);
                send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
                write(fd, send_buff, COMMAND_PACKET_LENGTH);
                
                printf("Param: %d  Sent: ", i);
                for (int i=0; i<COMMAND_PACKET_LENGTH; i++)
                    printf("%02X ", send_buff[i]);

                int readlen = 0;
                int code = readResponse(fd, recv_buff, BUFFER, readlen);
            
                printf("  Got: ");
                for (int i=0; i<readlen; i++)
                    printf("%02X ", recv_buff[i]);
                printf("  :  ");
            
                if (code != 0)
                {
                    printf("ERROR %02X\n", code);
                }
            }
        }

    } //end while

    //tcsetattr(fd,TCSANOW, &oldtio);
    
    printf("closing\n");
    close(fd);
    printf("exiting\n");
    exit(0);
}
