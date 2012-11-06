/**
 * \file annanLib.cpp
 *
 * library to read data from annan's 5 slider board
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#include "../include/annan_control/annanLib.hpp"

// #include <sys/types.h>
// #include <sys/stat.h>
#include <fcntl.h> //O_RDWR
#include <termios.h> //termios
#include <stdio.h> //printf
#include <string.h> //memmove strchr
// #include <strings.h>
#include <unistd.h> //read close
#include <stdlib.h> //atoi

//#include <pthread.h>

#define TOSS_FIRST_N 5
//#define DEBUG_ANNAN

static bool athread_run = true;
static int athread_fd = -1;
static annan_cb_t athread_cb = NULL;
static pthread_t athread;
// int spreadstate = 0;
// int spreadval = 0;
static int readcount = 0;

static CommandType lastState = PARAMETER_SET;
static int zeroPoint[5] = {0, 0, 0, 0, 0};

AnnanBoard parseAnnanData(char* data)
{
    char* ptr = data;
    char* ptr2;
    AnnanBoard vals;

    ptr = strchr(ptr, '\n');

    //printf("first \\n %d\n", (ptr-data));

    for (int i=0; i<5; i++)
    {
        ptr = strchr(ptr, ':');
        if (ptr==NULL)
            return vals;
        //printf("next : %d\n", (ptr-data));
        ptr = ptr + 1;
        
        ptr2 = strchr(ptr, '\n');
        if (ptr2==NULL)
            return vals;
        //printf("next \\n %d\n", (ptr2-data));
        *ptr2 = '\0';
        
        vals.Slider[i] = atoi(ptr);
        //printf("value %d\n", vals.Slider[i]);
        ptr = ptr2+1;
        *ptr2 = '_';
    }

    for (int i=0; i<4; i++)
    {
        ptr = strchr(ptr, ':');
        if (ptr==NULL)
            return vals;
        ptr = ptr + 1;
        
        ptr2 = strchr(ptr, '\n');
        if (ptr2==NULL)
            return vals;
        *ptr2 = '\0';
        
        vals.Button[i] = (bool)atoi(ptr);
        ptr = ptr2+1;
        *ptr2 = '_';
    }
    return vals;
};

void printAnnanData(const AnnanBoard data)
{
    printf("------------\n");
    for (int i=0; i<5; i++)
        printf("Slider[%d] = %d\n", i, data.Slider[i]);
    for (int i=0; i<4; i++)
        printf("Button[%d] = %d\n", i, data.Button[i]);
};


int annan_connect(const char* const device)
{
    struct termios tio;
    
    athread_fd = open(device, O_RDWR | O_NOCTTY ); 
    if (athread_fd <0 ) 
    {
#ifdef DEBUG_ANNAN
        printf("ERROR: cannot open serial port\n");
#endif
        return -1;
    }
    
    /* set input mode (non-canonical, no echo,...) */
    bzero(&tio, sizeof(tio));
    tio.c_cflag = ANNAN_BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
        
    tio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    tio.c_cc[VMIN]     = READ_CHUNK;   /* blocking read until 5 chars received */
        
    tcflush(athread_fd, TCIFLUSH);
    tcsetattr(athread_fd,TCSANOW,&tio);
    return 0;
};

void* annan_thread(void*)
{
    char buf[ANNAN_BUFFER];
    memset(buf, 0, ANNAN_BUFFER);
    int token = 0;
    bool aligned = false;
    bool expectwipe = true;
    
    while (athread_run)
    {
            if (token > ANNAN_BUFFER-READ_CHUNK)
            {
#ifdef DEBUG_ANNAN
                if (!expectwipe)
                    printf("\n\n\nWARNING: emergency wipe\n");
#endif
                token=0;
                memset(buf, 0, ANNAN_BUFFER);
                aligned = false;
                expectwipe = false;
            }
        
            int res = read(athread_fd, &(buf[token]), ANNAN_BUFFER-token-1);   /* returns after 5 chars have been input */
            if (res < 0)
            {
#ifdef DEBUG_ANNAN
                printf("\n\n\nWARNING: read returned %d\n", res);
                //run_=false;
#endif
            }
            else if (res > 0)
            {
                token += res;
                buf[token]='\0';

                if (!aligned)
                {
                    // find first '$'
                    char* start = strchr(buf, '$');
                    if (start != NULL)
                    {
                        // move data to start of buf[], cutting off '$'
                        aligned = true;
                        int start_pos = (start - buf);
                        memmove(buf, start+1, token - start_pos);
                        token = token - start_pos - 1;
                        buf[token] = '\0';
                    }
                }
            
                if (aligned)
                {
                    // find next '$'
                    char* end = strchr(buf, '$');
                    while (end != NULL)
                    {
                        int len = end-buf;
                        if (len < 52 || len > 67)
                        {
                            // length of data is wrong, possible data corruption
#ifdef DEBUG_ANNAN
                            printf("\n\n\nWARNING: end-buf = %d\n", len);
#endif
                            token=0;
                            memset(buf, 0, ANNAN_BUFFER);
                            aligned = false;
                            break;
                        }
                        
                        // throw away the first few readings because they are junk
                        readcount++;
                        if (readcount > TOSS_FIRST_N)
                        {
                            AnnanBoard vals = parseAnnanData(buf);
#ifdef DEBUG_ANNAN
                            printAnnanData(vals);
#endif
                            if (athread_cb)
                                athread_cb(vals);
                        }
                        
                        int start_pos = (end - buf);
                        memmove(buf, end+1, token - start_pos);
                        token = token - start_pos - 1;
                        buf[token] = '\0';

                        end = strchr(buf, '$');
                    }
                }
            }
        }

    close(athread_fd);
    pthread_exit(NULL);
};
    
int annan_start(annan_cb_t callback)
{
    athread_cb = callback;
    return pthread_create(&athread, NULL, annan_thread, NULL);
};

int annan_stop()
{
    athread_run = false;
    // wait for thread to finish
    return pthread_join(athread, NULL);
};


HandleCommand annan_to_cmd(const AnnanBoard& data)
{
    HandleCommand cmd;
    if (data.Button[0]) //calibrate
    {
        cmd.calibrate = true;
        lastState = PARAMETER_SET;
    }
    else if (data.Button[3]) // position mode
    {
        if (lastState != MOTOR_POSITION)
        {
            for (int i=0; i<5; i++)
                zeroPoint[i] = MIN_SLIDER; //data.Slider[i];
            lastState = MOTOR_POSITION;
            //zeroPoint[4] = MAX_SLIDER/2;
        }
        
        for (int i=0; i<4; i++)
        {
            cmd.motorCommand[annanToFinger[i]].type = MOTOR_POSITION;
            cmd.motorCommand[annanToFinger[i]].valid = true;
            cmd.motorCommand[annanToFinger[i]].value = (data.Slider[i]-zeroPoint[i]) * (float)MAX_MOTOR_POSITION / (float)MAX_SLIDER;
        }
        
        // handle spread slightly different
        {
            cmd.motorCommand[4].type = MOTOR_POSITION;
            cmd.motorCommand[4].value = (data.Slider[4]-zeroPoint[4]) * (float)MAX_SPREAD_POSITION / (float)MAX_SLIDER;
            cmd.motorCommand[4].valid = true;
            
            // // spread is always in velocity mode
            // {
            //     cmd.motorCommand[4].type = MOTOR_VELOCITY;
            //     cmd.motorCommand[4].valid = true;
            
            //     if (data.Slider[4] > zeroPoint[4]-DEADBAND && data.Slider[4] < zeroPoint[4]+DEADBAND)
            //         cmd.motorCommand[4].value = 0;
            //     else if (data.Slider[4] >= zeroPoint[4]+DEADBAND)
            //         cmd.motorCommand[4].value = (data.Slider[4]-zeroPoint[4]-DEADBAND) * (float)MAX_MOTOR_VELOCITY / (float)(MAX_SLIDER-zeroPoint[4]-DEADBAND);
            //     else
            //         cmd.motorCommand[4].value = (data.Slider[4]-zeroPoint[4]+DEADBAND) * (float)MAX_MOTOR_VELOCITY / (float)(MAX_SLIDER-zeroPoint[4]-DEADBAND);
            // }
        }
        
    }
    else if (data.Button[2]) // velocity mode
    {
        if (lastState != MOTOR_VELOCITY)
        {
            for (int i=0; i<5; i++)
                zeroPoint[i] = data.Slider[i];
            lastState = MOTOR_VELOCITY;
        }

        for (int i=0; i<5; i++)
        {
            cmd.motorCommand[annanToFinger[i]].type = MOTOR_VELOCITY;
            cmd.motorCommand[annanToFinger[i]].valid = true;
            
            if (data.Slider[i] > zeroPoint[i]-DEADBAND && data.Slider[i] < zeroPoint[i]+DEADBAND)
                cmd.motorCommand[annanToFinger[i]].value = 0;
            else if (data.Slider[i] >= zeroPoint[i]+DEADBAND)
                cmd.motorCommand[annanToFinger[i]].value = (data.Slider[i]-zeroPoint[i]-DEADBAND) * (float)MAX_MOTOR_VELOCITY / (float)(MAX_SLIDER-zeroPoint[i]-DEADBAND);
            else
                cmd.motorCommand[annanToFinger[i]].value = (data.Slider[i]-zeroPoint[i]+DEADBAND) * (float)MAX_MOTOR_VELOCITY / (float)(MAX_SLIDER-zeroPoint[i]-DEADBAND);
        }
        
        // safe mode
        if (data.Button[1])
        {
            for (int i=0; i<5; i++)
            {
                cmd.motorCommand[annanToFinger[i]].value /= 4;
            }
        }
    }
    // else if (data.Button[1]) // voltage mode
    // {
    //     if (lastState != MOTOR_VOLTAGE)
    //     {
    //         for (int i=0; i<5; i++)
    //             zeroPoint[i] = data.Slider[i];
    //         lastState = MOTOR_VOLTAGE;
    //     }

    //     for (int i=0; i<5; i++)
    //     {
    //         cmd.motorCommand[annanToFinger[i]].type = MOTOR_VOLTAGE;
    //         cmd.motorCommand[annanToFinger[i]].valid = true;
            
    //         if (data.Slider[i] > zeroPoint[i]-DEADBAND && data.Slider[i] < zeroPoint[i]+DEADBAND)
    //             cmd.motorCommand[annanToFinger[i]].value = 0;
    //         else if (data.Slider[i] >= zeroPoint[i]+DEADBAND)
    //             cmd.motorCommand[annanToFinger[i]].value = (data.Slider[i]-zeroPoint[i]-DEADBAND) * (float)MAX_MOTOR_VOLTAGE / (float)(MAX_SLIDER-zeroPoint[i]-DEADBAND);
    //         else
    //             cmd.motorCommand[annanToFinger[i]].value = (data.Slider[i]-zeroPoint[i]+DEADBAND) * (float)MAX_MOTOR_VOLTAGE / (float)(MAX_SLIDER-zeroPoint[i]-DEADBAND);
    //     }
    // }
    // else if (data.Button[1]) // current mode
    // {
    //     if (lastState != MOTOR_CURRENT)
    //     {
    //         for (int i=0; i<5; i++)
    //             zeroPoint[i] = data.Slider[i];
    //         lastState = MOTOR_CURRENT;
    //     }

    //     for (int i=0; i<5; i++)
    //     {
    //         cmd.motorCommand[annanToFinger[i]].type = MOTOR_CURRENT;
    //         cmd.motorCommand[annanToFinger[i]].valid = true;
            
    //         if (data.Slider[i] > zeroPoint[i]-DEADBAND && data.Slider[i] < zeroPoint[i]+DEADBAND)
    //             cmd.motorCommand[annanToFinger[i]].value = 0;
    //         else if (data.Slider[i] >= zeroPoint[i]+DEADBAND)
    //             cmd.motorCommand[annanToFinger[i]].value = (data.Slider[i]-zeroPoint[i]-DEADBAND) * (float)MAX_MOTOR_CURRENT / (float)(MAX_SLIDER-zeroPoint[i]-DEADBAND);
    //         else
    //             cmd.motorCommand[annanToFinger[i]].value = (data.Slider[i]-zeroPoint[i]+DEADBAND) * (float)MAX_MOTOR_CURRENT / (float)(MAX_SLIDER-zeroPoint[i]-DEADBAND);
    //     }
    // }
    else // no mode button pressed
    {
        lastState = PARAMETER_SET;
        for (int i=0; i<5; i++)
        {
            cmd.motorCommand[annanToFinger[i]].type = MOTOR_VELOCITY;
            cmd.motorCommand[annanToFinger[i]].valid = true;
            cmd.motorCommand[annanToFinger[i]].value = 0;
        }
    }
    
    return cmd;
};
