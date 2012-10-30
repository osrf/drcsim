/**
 * \file controller.cpp
 *
 * Overo control code
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <pthread.h>
#include <ctype.h> //isprint

#include <fcntl.h>
#include <termios.h>
#include <math.h> // round

// this is not compiled with rosmake so this is a little more awkward
#include "../../handle_lib/include/handle_lib/handleTypes.hpp"
#include "../../handle_lib/include/handle_lib/packetParser.hpp"
#include "../../handle_lib/include/handle_lib/handleControl.hpp"
#include "../../handle_lib/include/handle_lib/handleTcp.hpp"
// #include "handle_lib/handleTypes.hpp"
// #include "handle_lib/packetParser.hpp"
// #include "handle_lib/handleControl.hpp"
// #include "handle_lib/handleTcp.hpp"

#define MOTOR_DEADBAND 25 // in units of hall encoder ticks
#define MOTOR_MIN_RPM  200 // in units of RPM
#define MOTOR_MAX_RPM  12000 // in units of RPM
#define MOTOR_P        15.0  // proportional gain for finger motors
#define SPREAD_DEADBAND 15 //75 // in units of magnetic encoder ticks
#define DO_THERMAL

//how many sensor messages to wait for before declaring a motor message unsuccessful
#define CMD_WAIT_TRIES 10

// how many commands to skip when commands are the same as last time
#define MAX_SKIP 10

//#define BAUDRATE B115200
#define  BAUDRATE B1152000
//#define BAUDRATE B2000000
#define MODEMDEVICE "/dev/ttyS0"
#define BUFFER 1000

// parameters affecting thermal limit override
#define OVERRIDE_HISTORY    50   // size of the history buffer to store (how fast to get into override)
#define ERROR_TO_OVERRIDE   1000 // position target - current position must be greater than this to trigger override
#define MAX_DELTA_NO_CHANGE 10   // the sum of deltas in the history must be smaller than this to trigger override
#define OVERRIDE_ITERATIONS 150   // how many iterations to be at 0 current (how long to be in override)

#ifdef DO_THERMAL
struct override_t
{
    int pos_history[OVERRIDE_HISTORY];
    int err_history[OVERRIDE_HISTORY];
    int history_i;
    int last_error;
    int count;
};
override_t motor_override[4];
#endif

HandSensors sensor_data;

pthread_mutex_t control_mutex = PTHREAD_MUTEX_INITIALIZER;
HandleCommand control;
HandleCommand lastcmd;
int skip_count[5];

// calibration data
int offset[5];
int last_spread = 0;
bool spread_calibrated = false;
int last_angle[3];
bool angle_calibrated[3];

int hz = 200;
bool debug_time = false;
bool debug_motors = false;
bool debug_errors = false;
int voltage_cap = 150;
bool include_accel = false;

int sign(float val)
{
    if (val == 0)
        return 0;
    if (val > 0)
        return 1;
    return -1;
};
int sign(int val)
{
    if (val == 0)
        return 0;
    if (val > 0)
        return 1;
    return -1;
};

/// Look through 'control', return true if any commands are valid.
// index will be moved to point to the next valid command.
// does not lock mutex.
bool anyValid(int& index)
{
    for (int i=0; i<5; i++)
    {
        //index = (index+i)%5;
        if (control.motorCommand[(index+i)%5].valid)
        {
            index = (index+i)%5;
            return true;
        }
    }
    return false;
};

volatile bool run = true;
volatile int socketfd = -1;
volatile int tcp_fd = -1;
void sigint_handler(int s)
{
    printf("SIGINT received\n");
    run = false;
    if (socketfd != -1)
    {
        close(socketfd);
        socketfd = -1;
    }
    if (tcp_fd != -1)
    {
        close(tcp_fd);
        tcp_fd = -1;
    }
};

void sigpipe_handler(int s)
{
    printf("SIGPIPE received\n");
};


// bang-bang control for finger spread motor
int spreadControl(int target)
{
    int error = target - sensor_data.fingerSpread;
    if (abs(error) < SPREAD_DEADBAND)
        return 0;
    else if (error > 0)
        return 1;
    else
        return -1;
};


// motor 0-4
// target is the target encoder count.
// return value to set motor at.  
// Value will be capped at motor min and max values (-9000 to -200, 200 to 9000).
// Motor 4 is a bang controller, (-1 to 1) is sufficient.
int motorControl(int motor, int target)
{
    if (motor == 4)
        return spreadControl(target);
    
    float error = target - sensor_data.motorHallEncoder[motorChain_to_fingerNumber[motor]];

    // thermal limit override
#ifdef DO_THERMAL
    
    motor_override[motor].history_i++;
    motor_override[motor].history_i = motor_override[motor].history_i % OVERRIDE_HISTORY;
    
    motor_override[motor].pos_history[motor_override[motor].history_i] = 
        sensor_data.motorHallEncoder[motorChain_to_fingerNumber[motor]];

    motor_override[motor].err_history[motor_override[motor].history_i] = (int)error;
    
    //if (motor==3)  printf("%d  %d\n", (int)error, target);

    bool override = false;

    if (abs(error) > ERROR_TO_OVERRIDE)
    {
        int err_sum = 0;
        for (int i=0; i<OVERRIDE_HISTORY; i++)
            if (abs(motor_override[motor].err_history[i]) > ERROR_TO_OVERRIDE)
                err_sum += sign(motor_override[motor].err_history[i]);
        
        int delta_sum = 0;
        for (int i=0; i<(OVERRIDE_HISTORY-1); i++)
            if (abs(motor_override[motor].pos_history[i] - motor_override[motor].pos_history[i+1]) < MAX_DELTA_NO_CHANGE)
                delta_sum++;

        //if (motor==3) printf("             %d  %d  %d  %d\n", err_sum, delta_sum, sign(motor_override[motor].last_error), sign(error));
        
        if (abs(err_sum) >= OVERRIDE_HISTORY && delta_sum >= (OVERRIDE_HISTORY-3))
        {
            if (motor_override[motor].last_error == 0 ||
                sign(motor_override[motor].last_error) == sign(error))
            {
                //if (motor==3) printf("            OVERRIDE %d\n", motor_override[motor].count);
                motor_override[motor].last_error = error;
                error = 0;
                override = true;
                
                // if we have been overridden too long, release it for a short pulse.
                if (motor_override[motor].count++ > OVERRIDE_ITERATIONS)
                {
                    //if (motor==3) printf("          disable\n");
                    for (int i=0; i<OVERRIDE_HISTORY; i++)
                        motor_override[motor].err_history[i] = 0;
                }
            }
        }
    }
    
    if (!override)
    {
        //if (motor==3) printf("no\n");
        motor_override[motor].count = 0;
        motor_override[motor].last_error = 0;
    }
    
#endif //end thermal overide
    
    if (abs(error) < MOTOR_DEADBAND)
    {
        return 0;
    }
    else
    {
        int p_control = (int)round(error * (float)MOTOR_P);
        
        // hard limits
        if (p_control > MOTOR_MAX_RPM)
            p_control = MOTOR_MAX_RPM;
        if (p_control < -MOTOR_MAX_RPM)
            p_control = -MOTOR_MAX_RPM;
        if (p_control > 0 && p_control < MOTOR_MIN_RPM)
            p_control = MOTOR_MIN_RPM;
        if (p_control < 0 && p_control > -MOTOR_MIN_RPM)
            p_control = -MOTOR_MIN_RPM;
        
        return p_control;
    }
};

/// Set a parameter on the motor [0-3]
// returns error code.
//  0 = success
// -2 = unknown motor
// -3 = write error
int writeMotorParameter(int fd, int motor, float value, short parameter)
{
    unsigned char buff[COMMAND_PACKET_LENGTH];
    memset(buff, 0, COMMAND_PACKET_LENGTH);

    switch (motor)
    {
        case 0:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR1;
            break;
        case 1:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR2;
            break;
        case 2:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR3;
            break;
        case 3:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR4;
            break;
        default:
            return -2;
    }
    
    buff[COMMAND_OFFSET] = MOTOR_PARAMETER_WR_L_OPCODE | parameter;
    setPayloadf(buff, value);
    buff[CHECKSUM_OFFSET] = computeChecksum(buff, COMMAND_PACKET_LENGTH-1);
    
    if (write(fd, buff, COMMAND_PACKET_LENGTH) != COMMAND_PACKET_LENGTH)
        return -3;
    
    return 0;
};

/// Set the motor [0-4] to the desired speed value.
// Note that value can be negative.
// Value will be capped at 2 bytes (65535).
// type is ignored for motor 4.
// returns error code.
//  0 = success
// -2 = unknown motor
// -3 = write error
int writeMotor(int fd, int motor, int value, CommandType type = MOTOR_VELOCITY)
{
    //printf("writing motor %d with %d\n", motor, value);
    
    unsigned char buff[COMMAND_PACKET_LENGTH];
    memset(buff, 0, COMMAND_PACKET_LENGTH);
    
    switch (motor)
    {
        case 0:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR1;
            break;
        case 1:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR2;
            break;
        case 2:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR3;
            break;
        case 3:
            buff[DESTINATION_HEADER_OFFSET] = DESTINATION_MOTOR4;
            break;
        case 4:
            buff[DESTINATION_HEADER_OFFSET] = PALM_CHAINADDRESS;
            break;
        default:
            return -2;
    }
    
    if (motor == 4)
        buff[COMMAND_OFFSET] = FINGER_COMMAND_OPCODE;
    else
    {
        switch (type)
        {
            case MOTOR_CURRENT:
                buff[COMMAND_OFFSET] = MOTOR_COMMAND_OPCODE | MOTOR_COMMAND_CURRENT;
                break;
            case MOTOR_VOLTAGE:
                buff[COMMAND_OFFSET] = MOTOR_COMMAND_OPCODE | MOTOR_COMMAND_VOLTAGE;
                break;
            case MOTOR_VELOCITY:
            case MOTOR_POSITION:
            default:
                buff[COMMAND_OFFSET] = MOTOR_COMMAND_OPCODE | MOTOR_COMMAND_VELOCITY;
                break;
        }
    }
    
    if (value == 0)
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_STOP;
    else if (value > 0)
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_FORWARD;
    else
        buff[COMMAND_OFFSET] |= MOTOR_COMMAND_REVERSE;
    
    if (value < 0)
        value = -value;
    
    if (value > 65535)
        value = 65535;
    
    setPayload(buff, value & 0xFFFF);
    buff[CHECKSUM_OFFSET] = computeChecksum(buff, COMMAND_PACKET_LENGTH-1);

    if (write(fd, buff, COMMAND_PACKET_LENGTH) != COMMAND_PACKET_LENGTH)
        return -3;
    
    return 0;
};



void* serial_thread(void*)
{
    printf("serial thread started\n");
    
    int fd; // serial file descriptor

    // ofstream myfile;
    // myfile.open("/home/root/code.log", ios::app);
    // myfile << "serial thread started\n";
    // myfile.close();

    printf("opening serial port\n");
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY); // | O_NONBLOCK); 
    if (fd <0 ) 
    {
        printf("ERROR: cannot open serial port\n");
        perror(MODEMDEVICE); 
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "ERROR: cannot open serial port\n";
        // myfile.close();
        pthread_exit(NULL);
        exit(-1); 
    }
    printf("serial port open\n");
    
    printf("configuring serial port\n");
    struct termios options;
    tcgetattr(fd, &options);
    cfmakeraw(&options);
    cfsetspeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    
    // 8 data bits, no parity, 1 stop bit
    options.c_cflag &= ~PARENB; // no parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit, not 2
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits

    options.c_cflag &= ~CRTSCTS; // no CTS/RTS hardware flow control
    options.c_cc[VTIME]    = 10;  // 0.5 second timeout
    options.c_cc[VMIN]     = 1;  // block for 1 char

    tcflush(fd, TCIOFLUSH); // flush both directions
    tcsetattr(fd, TCSANOW, &options);
    printf("serial port configured\n");
    
    // myfile.open("/home/root/code.log", ios::app);
    // myfile << "configuring hardware\n";
    // myfile.close();

    unsigned char send_buff[COMMAND_PACKET_LENGTH];

    int sensor_mask = DATA_COLLECTION_ALL_BITMASK & 
        (~DATA_COLLECTION_ACCELERATION_BITMASK) & // no accelerometers because they break the distal tactile sensors
        (~DATA_COLLECTION_DYNAMIC_BITMASK) & // no PVDF because it is too hard to mold
        (~DATA_COLLECTION_TENSION_BITMASK) & // no cable tension sensors
        (~DATA_COLLECTION_EXTERNALSUPPLY_BITMASK); // no voltages because they are not needed
    
    if (include_accel)
        sensor_mask |= DATA_COLLECTION_ACCELERATION_BITMASK;
    
    printf("Sensor mask: 0x%04X\n", sensor_mask);
    
    printf("configuring hardware ");

    int rsp;
    
    //   
    // set sample period
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = SET_SAMPLE_PERIOD_OPCODE; // set sample period
    setPayload(send_buff, 1000000/hz);
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    rsp = readResponse(fd);
    if (rsp < 0)
    {
        printf("\nWARNING: set sample period returned: %02X\n", rsp);
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "WARNING: set sample period returned error\n";
        // myfile.close();
    }
    else
    {
        printf(".");
        fflush(stdout);
    }
    
    if (!run)
    {
        // stop collection
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
        send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
        usleep(2000);
        // stop motors
        for(int i=0; i<5; i++)
        {
            writeMotor(fd, i, 0);
            usleep(2000);
        }
        printf("serial: closing\n");
        close(fd);
        printf("serial thread stopping\n");
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "ERROR serial thread stopping\n";
        // myfile.close();
        pthread_exit(NULL);
    }
    
    //
    // increase max motor rpm
    //    
    for (int i=0; i<4; i++)
    {
        if (writeMotorParameter(fd, i, MOTOR_MAX_RPM, PARAMETER_MAXIMUM_RPM) != 0)
            printf("\nWARNING: write motor %d max rpm parameter failed\n", i);
        else if (readResponse(fd) < 0)
            printf("\nWARNING: set motor %d max rpm parameter returned: %02X\n", i, rsp);
        else
        {
            printf(".");
            fflush(stdout);
        }

        if (!run)
        {
            // stop collection
            memset(send_buff, 0, COMMAND_PACKET_LENGTH);
            send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
            send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
            send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
            write(fd, send_buff, COMMAND_PACKET_LENGTH);
            usleep(2000);
            // stop motors
            for(int i=0; i<5; i++)
            {
                writeMotor(fd, i, 0);
                usleep(2000);
            }
            printf("serial: closing\n");
            close(fd);
            printf("serial thread stopping\n");
            // myfile.open("/home/root/code.log", ios::app);
            // myfile << "ERROR serial thread stopping\n";
            // myfile.close();
            pthread_exit(NULL);
        }
    }
    
    //
    // decrease max motor voltage
    //    
    for (int i=0; i<4; i++)
    {
        if (writeMotorParameter(fd, i, voltage_cap, PARAMETER_MAXIMUM_COMMAND) != 0)
            printf("\nWARNING: write motor %d max voltage parameter failed\n", i);
        else if (readResponse(fd) < 0)
            printf("\nWARNING: set motor %d max voltage parameter returned: %02X\n", i, rsp);
        else
        {
            printf(".");
            fflush(stdout);
        }

        if (!run)
        {
            // stop collection
            memset(send_buff, 0, COMMAND_PACKET_LENGTH);
            send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
            send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
            send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
            write(fd, send_buff, COMMAND_PACKET_LENGTH);
            usleep(2000);
            // stop motors
            for(int i=0; i<5; i++)
            {
                writeMotor(fd, i, 0);
                usleep(2000);
            }
            printf("serial: closing\n");
            close(fd);
            printf("serial thread stopping\n");
            // myfile.open("/home/root/code.log", ios::app);
            // myfile << "ERROR serial thread stopping\n";
            // myfile.close();
            pthread_exit(NULL);
        }
    }
    
    //
    // set sample args
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = SET_SAMPLE_ARGS_OPCODE; // set sample args
    setPayload(send_buff, sensor_mask);
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    rsp = readResponse(fd);
    if (rsp < 0)
    {
        printf("\nWARNING: set sample args returned: %02X\n", rsp);
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "WARNING: set sample args returned error\n";
        // myfile.close();
    }
    else
    {
        printf(".");
        fflush(stdout);
    }
    
    if (!run)
    {
        // stop collection
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
        send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
        usleep(2000);
        // stop motors
        for(int i=0; i<5; i++)
        {
            writeMotor(fd, i, 0);
            usleep(2000);
        }
        printf("serial: closing\n");
        close(fd);
        printf("serial thread stopping\n");
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "ERROR serial thread stopping\n";
        // myfile.close();
        pthread_exit(NULL);
    }

    //
    // start collection
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = START_COLLECTION_OPCODE; // start collection
    //setPayload(send_buff, 0); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    rsp = readResponse(fd);
    if (rsp < 0)
    {
        printf("\nWARNING: start collection returned: %02X\n", rsp);
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "WARNING: start collection returned error\n";
        // myfile.close();
    }
    else
    {
        printf(".");
        fflush(stdout);
    }

    if (!run)
    {
        // stop collection
        memset(send_buff, 0, COMMAND_PACKET_LENGTH);
        send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
        send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
        send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
        write(fd, send_buff, COMMAND_PACKET_LENGTH);
        usleep(2000);
        // stop motors
        for(int i=0; i<5; i++)
        {
            writeMotor(fd, i, 0);
            usleep(2000);
        }
        printf("serial: closing\n");
        close(fd);
        printf("serial thread stopping\n");
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "ERROR serial thread stopping\n";
        // myfile.close();
        pthread_exit(NULL);
    }
    
    printf("\nDONE CONFIGURING HARDWARE\n");

    // myfile.open("/home/root/code.log", ios::app);
    // myfile << "DONE CONFIGURING HARDWARE\n";
    // myfile.close();

    unsigned char recv_buff[BUFFER];
    HandSensors tmpdata;
    HandSensorsValid valid;
    int count = 0;
    int command_index = 0;
    // int spreadcount = 0;
    // int spreadreal = 0;

    timeval tv_now;
    timeval tv_last_motor;
    timeval tv_last_sensor;
    timeval tv_last_loop;
    gettimeofday(&tv_now, NULL);
    gettimeofday(&tv_last_motor, NULL);
    gettimeofday(&tv_last_sensor, NULL);
    gettimeofday(&tv_last_loop, NULL);
    int motor_time_buff[100];
    int sensor_time_buff[100];
    int loop_time_buff[100];
    int motor_time_index = 1;
    int sensor_time_index = 1;
    int loop_time_index = 1;
    bool motor_time_disp = false;
    bool sensor_time_disp = false;
    int errorcount = 0;
    
    for (int i=0; i<100; i++)
    {
        motor_time_buff[i] = 0;
        sensor_time_buff[i] = 0;
        loop_time_buff[i] = 0;
    }

    while (run)
    {
        pthread_mutex_lock( &control_mutex );
        if (count++ % 2 == 0 && anyValid(command_index))
        {
            if (debug_motors)
            {
                printf("CMD: ");
                for (int i=0; i<5; i++)
                {
                    if (control.motorCommand[i].valid)
                        printf("%05d ", control.motorCommand[i].value);
                    else
                        printf("_____ ");
                }
                printf("\n");

                printf("LST: ");
                for (int i=0; i<5; i++)
                {
                    if (lastcmd.motorCommand[i].valid)
                        printf("%05d ", lastcmd.motorCommand[i].value);
                    else
                        printf("_____ ");
                }
                printf("\n");
            
                printf("CNT: ");
                for (int i=0; i<5; i++)
                    printf("%05d ", skip_count[i]);
                printf("\n");

                printf("TYP: ");
                for (int i=0; i<5; i++)
                {
                    if (i == command_index)
                    {
                        if (control.motorCommand[i].type == MOTOR_POSITION)
                            printf("PPPPP ");
                        else if (control.motorCommand[i].type == MOTOR_VELOCITY)
                            printf("RRRRR ");
                        else if (control.motorCommand[i].type == MOTOR_CURRENT)
                            printf("CCCCC ");
                        else if (control.motorCommand[i].type == MOTOR_VOLTAGE)
                            printf("VVVVV ");
                        else if (control.motorCommand[i].type == PARAMETER_SET)
                            printf("SSSSS ");
                        else
                            printf("***** ");
                    }
                    else
                        printf("_____ ");
                }
                printf("\n"); 
            }

            control.motorCommand[command_index].valid = false;
            MotorCommand command = control.motorCommand[command_index];
            pthread_mutex_unlock( &control_mutex );
            
            int set = 0;
            if (command.type == MOTOR_POSITION)
                set = motorControl(command_index, command.value); // does P control, caps at 200, 12000
            else //if (command.type == MOTOR_VELOCITY)
            {
                if (command_index == 4) // eliminate jitter for spread motor
                {
                    if (command.value > 5)
                        set = 1;
                    else if (command.value < -5)
                        set = -1;
                    else 
                        set = 0;
                }
                else
                    set = command.value;
            }
            // else
            // {
            //     printf("UNKNOWN COMMAND\n");
            // }
            
            if (debug_motors)
            {
                printf("AAT: ");            
                for (int i=0; i<5; i++)
                {
                    if (i < 4)
                        printf("%05d ", sensor_data.motorHallEncoder[motorChain_to_fingerNumber[i]]);
                    else
                        printf("%05d ", sensor_data.fingerSpread);
                }
                printf("\n"); 
            
                printf("SET: ");
                for (int i=0; i<5; i++)
                {
                    if (i == command_index)
                        printf("%05d ", set);
                    else
                        printf("_____ ");
                }
                printf("\n"); 
            }

            bool skip = false;
            if (lastcmd.motorCommand[command_index].valid && 
                lastcmd.motorCommand[command_index].type == command.type && 
                lastcmd.motorCommand[command_index].value == set)
            {
                // this command is the same as the last
                
                skip_count[command_index]++;
                
                if (skip_count[command_index] < MAX_SKIP)
                {
                    // actually skip
                    
                    if (debug_motors)
                    {
                        printf("SKIPPING\n");
                    }
                    
                    skip = true;
                    count--; // so we stay in motor command mode
                }
            }
            
            if (!skip)
            {
                skip_count[command_index] = 0;
                
                if (debug_motors)
                {
                    printf("SETTING\n");
                }
            
                lastcmd.motorCommand[command_index].valid = true;
                lastcmd.motorCommand[command_index].type = command.type;
                lastcmd.motorCommand[command_index].value = set;
            
                int trynum = 0;
                int ret = 0;
                
                // if (command.type == PARAMETER_SET)
                //     ret = writeMotorParameter(fd, command_index, set, command.parameter);
                // else
                ret = writeMotor(fd, command_index, set, command.type);

                // error writing to device, don't listen for response, just put command back in queue
                if (ret != 0)
                {
                    printf("\n\nwrite error: %02X\n", ret);
                    trynum = CMD_WAIT_TRIES+1;
                }

                // listen for up to 5 incoming messages to hear expected response
                while (trynum++ < CMD_WAIT_TRIES)
                {
                    //printf("listen response try %d\n", trynum);
                
                    tmpdata.reset();
                    valid.reset();
                    bool sensorMsg;
                    int code = parseResponse(fd, sensor_mask, sensorMsg, tmpdata, valid, recv_buff, BUFFER);
                    if (code == 0) // no error
                    {
                        if (debug_motors)
                        {
                            printf("&");
                        }
                        if (sensorMsg)
                        {
                            if (debug_time)
                            {
                                gettimeofday(&tv_now, NULL);
                                sensor_time_buff[sensor_time_index++ % 100] = 
                                    1000000*(tv_now.tv_sec - tv_last_sensor.tv_sec) + (tv_now.tv_usec - tv_last_sensor.tv_usec);
                                tv_last_sensor = tv_now;
                                sensor_time_disp = true;
                            }
                        
                            // motor encoder calibration
                            for (int i=0; i<4; i++)
                                tmpdata.motorHallEncoder[i] -= offset[i];

                            // spread encoder
                            if (valid.fingerSpread)
                            {
                                if (!spread_calibrated)
                                {
                                    last_spread = tmpdata.fingerSpread;
                                    spread_calibrated = true;
                                }
                                int delta = tmpdata.fingerSpread - last_spread;
                                if (abs(delta) < 512)
                                {
                                    last_spread = tmpdata.fingerSpread;
                                    tmpdata.fingerSpread = sensor_data.fingerSpread + delta;
                                }
                                else
                                {
                                    int bottom = last_spread + (1024-tmpdata.fingerSpread);
                                    int top = tmpdata.fingerSpread + (1024-last_spread);
                                    last_spread = tmpdata.fingerSpread;
                                    if (bottom<top) // crossing 0
                                        tmpdata.fingerSpread = sensor_data.fingerSpread - bottom;
                                    else // crossing 1024
                                        tmpdata.fingerSpread = sensor_data.fingerSpread + top;
                                }
                            }
                            
                            for (int i=0; i<3; i++)
                            {
                                if (valid.proximalJointAngle[i])
                                {
                                    if (!angle_calibrated[i])
                                    {
                                        last_angle[i] = tmpdata.proximalJointAngle[i];
                                        angle_calibrated[i] = true;
                                    }
                                    int delta = tmpdata.proximalJointAngle[i] - last_angle[i];
                                    if (abs(delta) < 512)
                                    {
                                        last_angle[i] = tmpdata.proximalJointAngle[i];
                                        tmpdata.proximalJointAngle[i] = sensor_data.proximalJointAngle[i] + delta;
                                    }
                                    else
                                    {
                                        int bottom = last_angle[i] + (1024-tmpdata.proximalJointAngle[i]);
                                        int top = tmpdata.proximalJointAngle[i] + (1024-last_angle[i]);
                                        last_angle[i] = tmpdata.proximalJointAngle[i];
                                        if (bottom<top) // crossing 0
                                            tmpdata.proximalJointAngle[i] = sensor_data.proximalJointAngle[i] - bottom;
                                        else // crossing 1024
                                            tmpdata.proximalJointAngle[i] = sensor_data.proximalJointAngle[i] + top;
                                    }   
                                }
                            }

                            sensor_data.update(tmpdata, valid);
                        
                            if (tcp_fd >= 0)
                            {
                                HandPacket packet(sensor_data);
                                int len = packet.pack(recv_buff);
                                if (sendall(tcp_fd, recv_buff, len, 0) == -1)
                                    tcp_fd = -1;
                            }
                        }
                        else
                        {
                            if (debug_motors)
                            {
                                printf("@");
                            }
                            // Got motor command response
                            if (debug_time)
                            {
                                gettimeofday(&tv_now, NULL);
                                motor_time_buff[motor_time_index++ % 100] = 
                                    1000000*(tv_now.tv_sec - tv_last_motor.tv_sec) + (tv_now.tv_usec - tv_last_motor.tv_usec);
                                tv_last_motor = tv_now;
                                motor_time_disp = true;
                            }
                            // if we ever implement a GET type message, this is where we would parse it.
                            break;
                        }
                    }
                    else // error
                    {
                        if (debug_errors)
                        {
                            printf("\n\ncode error: %02X\n", code);
                        }
                        if (sensorMsg)
                        {
                            // do nothing, keep waiting for response.
                            // sometimes sensor messages can return an error code if there is a data length issue.
                        }
                        else
                        {
                            // got an error from our command
                            // break loop with error
                            trynum = CMD_WAIT_TRIES+1;
                            break;
                        }
                    }
                }
            
                if (trynum >= CMD_WAIT_TRIES) // either did not hear a response, or got an error
                {
                    if (debug_time)
                    {
                        errorcount++;
                    }
                    
                    if (debug_errors)
                    {
                        printf("\nno response or error\n");
                    }
                    // put back in queue to try again
                    pthread_mutex_lock( &control_mutex );
                    lastcmd.motorCommand[command_index].valid = false;
                    if (!control.motorCommand[command_index].valid) // make sure newer data is not already set
                        control.motorCommand[command_index].valid = true;
                    pthread_mutex_unlock( &control_mutex );
                }
            }
            
            // re-enable position command
            if (command.type == MOTOR_POSITION)
            {
                pthread_mutex_lock( &control_mutex );
                if (!control.motorCommand[command_index].valid) // make sure newer data is not already set
                    control.motorCommand[command_index].valid = true;
                pthread_mutex_unlock( &control_mutex );
            }

            command_index++;
            command_index = command_index % 5;

            // // special case spread motor
            // if (command_index == 4)
            // {
            //     if (debug_motors)
            //     {
            //         printf("case 4\n");
            //     }
            //     pthread_mutex_lock( &control_mutex );
            //     if (++spreadcount % 2 == 0)
            //     {
            //         if (debug_motors)
            //         {
            //             printf("case 4 abort\n");
            //         }
            //         command_index = 0; //increment
            //         spreadcount = 0;
            //         control.motorCommand[4].value = spreadreal;
            //     }
            //     else if (control.motorCommand[4].value == 0)
            //     {
            //         if (debug_motors)
            //         {
            //             printf("case 4 zero\n");
            //         }
            //         spreadcount = 0;
            //         command_index = 0; //increment
            //     }
            //     else
            //     {
            //         if (debug_motors)
            //         {
            //             printf("case 4 repeat\n");
            //         }
            //         spreadreal = control.motorCommand[4].value;
            //         if (control.motorCommand[4].type == MOTOR_POSITION)
            //             control.motorCommand[4].value = sensor_data.fingerSpread;
            //         else
            //             control.motorCommand[4].value = 0;
            //         control.motorCommand[4].valid = true;
            //     }
            //     pthread_mutex_unlock( &control_mutex );
            // }
            // else
            // {
            //     command_index++;
            //     command_index = command_index % 5;
            // }

        }
        else // only listen for sensor data
        {
            pthread_mutex_unlock( &control_mutex );
            
            tmpdata.reset();
            valid.reset();
            bool sensorMsg;
            int code = parseResponse(fd, sensor_mask, sensorMsg, tmpdata, valid, recv_buff, BUFFER);
            if (code == 0) // no error
            {
                if (sensorMsg)
                {
                    if (debug_motors)
                    {
                        printf("$");
                    }
                    if (debug_time)
                    {
                        gettimeofday(&tv_now, NULL);
                        sensor_time_buff[sensor_time_index++ % 100] = 
                            1000000*(tv_now.tv_sec - tv_last_sensor.tv_sec) + (tv_now.tv_usec - tv_last_sensor.tv_usec);
                        tv_last_sensor = tv_now;
                        sensor_time_disp = true;
                    }
                    // calibration
                    for (int i=0; i<4; i++)
                        tmpdata.motorHallEncoder[i] -= offset[i];
                    
                    // spread encoder
                    if (valid.fingerSpread)
                    {
                        if (!spread_calibrated)
                        {
                            //printf("calibrating spread\n");
                            last_spread = tmpdata.fingerSpread;
                            spread_calibrated = true;
                        }
                        int delta = tmpdata.fingerSpread - last_spread;
                        if (abs(delta) < 512)
                        {
                            last_spread = tmpdata.fingerSpread;
                            tmpdata.fingerSpread = sensor_data.fingerSpread + delta;
                            
                        }
                        else
                        {
                            int bottom = last_spread + (1024-tmpdata.fingerSpread);
                            int top = tmpdata.fingerSpread + (1024-last_spread);
                            last_spread = tmpdata.fingerSpread;
                            if (bottom<top) // crossing 0
                                tmpdata.fingerSpread = sensor_data.fingerSpread - bottom;
                            else // crossing 1024
                                tmpdata.fingerSpread = sensor_data.fingerSpread + top;
                        }
                    }
                    
                    for (int i=0; i<3; i++)
                    {
                        if (valid.proximalJointAngle[i])
                        {
                            if (!angle_calibrated[i])
                            {
                                last_angle[i] = tmpdata.proximalJointAngle[i];
                                angle_calibrated[i] = true;
                            }
                            int delta = tmpdata.proximalJointAngle[i] - last_angle[i];
                            if (abs(delta) < 512)
                            {
                                last_angle[i] = tmpdata.proximalJointAngle[i];
                                tmpdata.proximalJointAngle[i] = sensor_data.proximalJointAngle[i] + delta;
                            }
                            else
                            {
                                int bottom = last_angle[i] + (1024-tmpdata.proximalJointAngle[i]);
                                int top = tmpdata.proximalJointAngle[i] + (1024-last_angle[i]);
                                last_angle[i] = tmpdata.proximalJointAngle[i];
                                if (bottom<top) // crossing 0
                                    tmpdata.proximalJointAngle[i] = sensor_data.proximalJointAngle[i] - bottom;
                                else // crossing 1024
                                    tmpdata.proximalJointAngle[i] = sensor_data.proximalJointAngle[i] + top;
                            }   
                        }
                    }

                    sensor_data.update(tmpdata, valid);

                    if (tcp_fd >= 0)
                    {
                        HandPacket packet(sensor_data);
                        int len = packet.pack(recv_buff);
                        if (sendall(tcp_fd, recv_buff, len, 0) == -1)
                            tcp_fd = -1;
                    }
                }
                else
                {
                    if (debug_errors)
                    {
                        printf("\n\nunwarranted response\n");
                    }
                }
            }
            else
            {
                if (debug_time)
                {
                    errorcount++;
                }
                if (debug_errors)
                {
                    printf("\n\ncode error: %02X\n", code);
                }
            }
        }
        if (debug_time)
        {
            gettimeofday(&tv_now, NULL);
            loop_time_buff[loop_time_index++ % 100] = 
                1000000*(tv_now.tv_sec - tv_last_loop.tv_sec) + (tv_now.tv_usec - tv_last_loop.tv_usec);
            tv_last_loop = tv_now;
        
            if (loop_time_index % 100 == 0)
            {
                double laverage = 0;
                for (int i=0; i<100; i++)
                    laverage += loop_time_buff[i];
                laverage /= 100.0;

                double saverage = 0;
                for (int i=0; i<100; i++)
                    saverage += sensor_time_buff[i];
                saverage /= 100.0;
            
                double maverage = 0;
                for (int i=0; i<100; i++)
                    maverage += motor_time_buff[i];
                maverage /= 100.0;
            
                printf("%.2f  %.2f  %.2f  %d\n", 1000000.0/laverage, 1000000.0/saverage, 1000000.0/maverage, errorcount);
            
                errorcount = 0;
            }
        }
    }
    
    //
    // stop collection
    //
    memset(send_buff, 0, COMMAND_PACKET_LENGTH);
    send_buff[DESTINATION_HEADER_OFFSET] = DESTINATION_PALM; // directed to palm board
    send_buff[COMMAND_OFFSET] = STOP_COLLECTION_OPCODE; // stop collection
    //setPayload(send_buff, 0); // no payload
    send_buff[CHECKSUM_OFFSET] = computeChecksum(send_buff, COMMAND_PACKET_LENGTH-1);
    write(fd, send_buff, COMMAND_PACKET_LENGTH);
    usleep(2000);
    
    // stop motors
    for(int i=0; i<5; i++)
        writeMotor(fd, i, 0);
    
    printf("serial: closing\n");
    close(fd);
    
    pthread_mutex_lock( &control_mutex );
    for (int i=0; i<5; i++)
    {
        control.motorCommand[i].valid = false;
        lastcmd.motorCommand[i].valid = false;
    }
    pthread_mutex_unlock( &control_mutex );
    
    printf("serial thread stopping\n");
    // myfile.open("/home/root/code.log", ios::app);
    // myfile << "serial thread stopping\n";
    // myfile.close();
    pthread_exit(NULL);
};


int init_socket(const char* const port)
{
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int yes=1;
    
    // ofstream myfile;
    // myfile.open("/home/root/code.log", ios::app);
    // myfile << "init_socket()\n";
    // myfile.close();
    
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    int rv = getaddrinfo(NULL, port, &hints, &servinfo);
    if (rv != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "ERROR getaddrinfo\n";
        // myfile.close();
        return -1;
    }
        
    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                             p->ai_protocol)) == -1) {
            perror("server: socket");
            // myfile.open("/home/root/code.log", ios::app);
            // myfile << "WARNING: socket in use?\n";
            // myfile.close();
            continue;
        }

        //fcntl(sockfd, F_SETFL, O_NONBLOCK);  // set to non-blocking

        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
                       sizeof(int)) == -1) {
            perror("setsockopt");
            // myfile.open("/home/root/code.log", ios::app);
            // myfile << "ERROR setsockopt\n";
            // myfile.close();
            return -1;
        }

        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("server: bind");
            // myfile.open("/home/root/code.log", ios::app);
            // myfile << "WARNING: can't bind socket?\n";
            // myfile.close();
            continue;
        }

        break;
    }

    if (p == NULL)  {
        fprintf(stderr, "server: failed to bind\n");
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "ERROR: server: failed to bind\n";
        // myfile.close();
        return -1;
    }

    freeaddrinfo(servinfo); // all done with this structure

    if (listen(sockfd, 10) == -1)
    {
        perror("listen");
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "ERROR: listen\n";
        // myfile.close();
        return -1;
    }
    
    // myfile.open("/home/root/code.log", ios::app);
    // myfile << "init_socket returning " << sockfd << "\n";
    // myfile.close();

    return sockfd;
}


int main(int argc, char** argv)
{
    //
    // init globals
    //
    for (int i=0; i<5; i++)
    {
        offset[i] = 0;
        skip_count[i] = 0;
    }
    last_spread = 0;
    spread_calibrated = false;
    for (int i=0; i<3; i++)
    {
        last_angle[i] = 0;
        angle_calibrated[i] = false;
    }
    
#ifdef DO_THERMAL
    for (int i=0; i<4; i++)
    {
        motor_override[i].history_i = 0;
        motor_override[i].last_error = 0;
        motor_override[i].count = 0;
        for (int j=0; j<OVERRIDE_HISTORY; j++)
        {
            motor_override[i].pos_history[j] = 0;
            motor_override[i].err_history[j] = 0;
        }
        //motor_override[i].history[0] = MAX_DELTA_NO_CHANGE;
    }
#endif

    //
    // parse command line options
    //
    int c;
    while ((c = getopt (argc, argv, "r:tmev:ah")) != -1)
    {
        switch (c)
        {
            case 'r':
                hz = atoi(optarg);
                if (hz <= 0)
                {
                    fprintf(stderr, "WARNING: value '%d' is outside of range for option 'r'\n", hz);
                    hz = 200;
                }
                break;
            case 't':
                debug_time = true;
                break;
            case 'm':
                debug_motors = true;
                break;
            case 'e':
                debug_errors = true;
                break;
            case 'v':
                voltage_cap = atoi(optarg);
                if (voltage_cap < 1 || voltage_cap > 255)
                {
                    fprintf(stderr, "WARNING: value '%d' is outside of range for option 'v'\n", voltage_cap);
                    voltage_cap = 150;
                }
                break;
            case 'a':
                include_accel = true;
                break;
            case 'h':
                printf("usage: handle_controller [options]\n");
                printf("options:\n");
                printf("  -r <hz>  Set sensor rate in hz.  Default: 200\n");
                printf("  -t       Debug: print measured rates.  Default: false\n");
                printf("  -m       Debug: print motor info.  Default: false\n");
                printf("  -e       Debug: print error codes.  Default: false\n");
                printf("  -v <cap> Cap max motor voltage.  Range 0-255. Default: 150\n");
                printf("  -a       Include accelerometers in sensor suite.  Default: false\n");
                printf("  -h       Display this help message and exit.  Default: false\n");
                exit(0);
                break;
            case '?':
                if (optopt == 'r' || optopt == 'v')
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    fprintf(stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
                fprintf(stderr, "\n");
                fprintf(stderr, "usage: handle_controller [options]\n");
                fprintf(stderr, "options:\n");
                fprintf(stderr, "  -r <hz>  Set sensor rate in hz.  Default: 200\n");
                fprintf(stderr, "  -t       Debug: print measured rates.  Default: false\n");
                fprintf(stderr, "  -m       Debug: print motor info.  Default: false\n");
                fprintf(stderr, "  -e       Debug: print error codes.  Default: false\n");
                fprintf(stderr, "  -v <cap> Cap max motor voltage.  Range 0-255. Default: 150\n");
                fprintf(stderr, "  -a       Include accelerometers in sensor suite.  Default: false\n");
                fprintf(stderr, "  -h       Display this help message and exit.  Default: false\n");
                exit(1);
            default:
                exit(2);
        }
    }

    printf("command line options:\n");
    printf("  hz = %d\n", hz);
    printf("  debug time = %d\n", debug_time);
    printf("  debug motors = %d\n", debug_motors);
    printf("  debug errors = %d\n", debug_errors);
    printf("  voltage cap = %d\n", voltage_cap);
    printf("  include accel = %d\n", include_accel);
    
    // // USAGE: handle_controller [hz] [debug_time] [debug_motors] [debug_errors] [voltage_cap]
    // if (argc > 1)
    //     hz = atoi(argv[1]);
    // if (argc > 2)
    //     debug_time = atoi(argv[2]) != 0;
    // if (argc > 3)
    //     debug_motors = atoi(argv[3]) != 0;
    // if (argc > 4)
    //     debug_errors = atoi(argv[4]) != 0;
    // if (argc > 5)
    //     voltage_cap = atoi(argv[5]) != 0;
    
    // if (hz <= 0)
    //     hz = 200;
    
    //setlogmask (LOG_UPTO (LOG_NOTICE));
    //openlog ("handle_controller", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);
    // ofstream myfile;

    printf("setting up signal handlers\n");
    //syslog (LOG_NOTICE, "setting up signal handlers");
    // myfile.open("/home/root/code.log", ios::app);
    // myfile << "setting up signal handlers\n";
    // myfile.close();
    
    
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART; //required or segfault on CTRL-C
    if (sigaction(SIGINT, &sa, NULL) == -1) 
    {
        //syslog (LOG_NOTICE, "SIGINT error");
        //closelog ();
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "SIGINT error\n";
        // myfile.close();
        
        perror("sigaction");
        exit(1);
    }

    // this seems to be required even though the body of the handler is empty
    struct sigaction sa2;
    sa2.sa_handler = sigpipe_handler; // print something when we get SIGPIPE signal
    sigemptyset(&sa2.sa_mask);
    sa2.sa_flags = SA_RESTART;
    if (sigaction(SIGPIPE, &sa2, NULL) == -1) 
    {
        //syslog (LOG_NOTICE, "SIGPIPE error");
        //closelog ();
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "SIGPIPE error\n";
        // myfile.close();

        perror("sigaction");
        exit(1);
    }

    // if (argc == 1) // if launched with no arguments
    // {    
    //     printf("sleeping for 1 second to let microcontrollers stabilize\n");
    //     //syslog (LOG_NOTICE, "sleeping for 1 second to let microcontrollers stabilize");

    //     // myfile.open("/home/root/code.log", ios::app);
    //     // myfile << "sleeping for 1 second to let microcontrollers stabilize\n";
    //     // myfile.close();
        
    //     sleep(1);
    // }

    pthread_t thread;
    
    if (pthread_create(&thread, NULL, serial_thread, NULL))
    {
        printf("ERROR: unable to create serial thread\n");
        //syslog (LOG_NOTICE, "ERROR: unable to create serial thread");
        //closelog ();

        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "ERROR: unable to create serial thread\n";
        // myfile.close();
        
        exit(-1);
    }

    socketfd = init_socket(HANDLE_PORT);
    
    //printf("server: waiting for connections...\n");
    
    while(run) 
    {
        // main accept() loop
        
        printf("server: waiting to accept\n");
        //syslog (LOG_NOTICE, "server: waiting to accept");

        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "server: waiting to accept\n";
        // myfile.close();

        struct sockaddr_storage their_addr; // connector's address information
        
        socklen_t sin_size = sizeof(sockaddr_storage); //their_addr;
        //int fd = accept4(socketfd, (struct sockaddr *)&their_addr, &sin_size, SOCK_NONBLOCK);
        int fd = accept(socketfd, (struct sockaddr *)&their_addr, &sin_size);
        //printf("accept returned %d\n", fd);
        if (fd == -1) 
        {
            // myfile.open("/home/root/code.log", ios::app);
            // myfile << "WARNING accept returned -1\n";
            // myfile.close();
            continue;
        }
        
        tcp_fd = fd;
        
        printf("receiver: starting\n");
        //syslog (LOG_NOTICE, "receiver: starting");
        
        // myfile.open("/home/root/code.log", ios::app);
        // myfile << "receiver: starting with tcp_fd " << tcp_fd <<"\n";
        // myfile.close();

        unsigned char rbuff[100];
        HandleCommand cmd;
        int len = cmd.pack(rbuff); // dummy pack to get serialization length.
        
        while (tcp_fd >= 0)
        {
            int ret = recvall(tcp_fd, rbuff, len);
            if (ret < 0)
            {
                tcp_fd = -1;
            }
            else 
            {
                cmd.unpack(rbuff);
                //printf("receiver: got %d bytes\n", ret);
                
                if (cmd.calibrate)
                {
                    pthread_mutex_lock( &control_mutex );
                    for (int i=0; i<4; i++)
                        offset[i] = sensor_data.motorHallEncoder[i] + offset[i];
                    for (int i=0; i<5; i++)
                    {
                        control.motorCommand[i].valid = false;
                        lastcmd.motorCommand[i].valid = false;
                    }
                    pthread_mutex_unlock( &control_mutex );
                    spread_calibrated = false;
                    sensor_data.fingerSpread = 0;
                    for (int i=0; i<3; i++)
                    {
                        sensor_data.proximalJointAngle[i] = 0;
                        angle_calibrated[i] = false;
                    }
                }
                else
                {
                    pthread_mutex_lock( &control_mutex );
                    if (debug_motors)
                    {
                        printf("GOT: ");
                    }
                    for (int i=0; i<5; i++)
                    {
                        if (cmd.motorCommand[i].valid)
                        {
                            // convert finger number into motor number
                            control.motorCommand[fingerNumber_to_motorChain[i]] = cmd.motorCommand[i];
                            if (debug_motors)
                            {      
                                printf("%05d ", cmd.motorCommand[i].value);
                            }
                        }
                        else
                        {
                            if (debug_motors)
                            {      
                                printf("_____ ");
                            }
                        }
                    }
                    if (debug_motors)
                    {      
                        printf("\n");
                    }
                    pthread_mutex_unlock( &control_mutex );
                }
            }
        }
        
        printf("receiver: stopping motors\n");
        pthread_mutex_lock( &control_mutex );
        for (int i=0; i<5; i++)
        {
            control.motorCommand[i].valid = true;
            control.motorCommand[i].type = MOTOR_VELOCITY;
            control.motorCommand[i].value = 0;
            lastcmd.motorCommand[i].valid = false;
        }
        pthread_mutex_unlock( &control_mutex );
        
        usleep(1000000);

        printf("receiver: invalidating motors\n");
        pthread_mutex_lock( &control_mutex );
        for (int i=0; i<5; i++)
        {
            control.motorCommand[i].valid = false;
            lastcmd.motorCommand[i].valid = false;
        }
        pthread_mutex_unlock( &control_mutex );
        
        printf("receiver: closing\n");
        close(fd);
    }
    
    pthread_join(thread, NULL);
    
    printf("exiting main\n");
    //syslog (LOG_NOTICE, "exiting main");
    //closelog ();
    
    // myfile.open("/home/root/code.log", ios::app);
    // myfile << "exiting main\n";
    // myfile.close();
    
    return 0;
}
