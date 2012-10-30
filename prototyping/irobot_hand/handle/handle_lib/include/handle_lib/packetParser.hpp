/**
 * \file packetParser.h
 *
 * Functions for parsing commands coming from traffic cop microcontroller.
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#ifndef PACKET_PARSER_H
#define PACKET_PARSER_H

#include "handleTypes.hpp"
#include <unistd.h> //read close


// Define this to spew lots of data to console.
//#define DEBUG_PARSER

#ifdef DEBUG_PARSER
#include <string.h>
#endif

/// Parse the sensor data from a motor board.
// Take data from the buffer and put it into the HandSensors type.
//
// \param buff  The buffer of data from the microcontroller.  The first byte 
//              should already be lined up with the appropriate data.
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
// \finger  The finger number.  (not motor number).  Range 0-3.  Note that 
//          despite the hand having only 3 fingers, there are 4 'fingers' here.
//          This is because there are 2 motor boards for the thumb.
//
// Returns the number of bytes parsed, or -1 on error.
int parseMotor(const unsigned char* const buff, 
               const unsigned int sensor_mask, 
               HandSensors& data,
               HandSensorsValid& valid,
               const unsigned int finger) //0 to 3
{
    if (finger > 3)
        return -1;
    
    int parsed = 0;
    
    // NOTE the order of these statements matters

    if (sensor_mask & DATA_COLLECTION_TENSION_BITMASK)
    {
        // The 2 thumb motor boards don't have cable tension sensors.
        // They still return data though, the data is just junk.
        if (finger == 0 || finger == 1)
        {
            data.cableTension[finger].sensor1 = ((float)((buff[parsed+1] << 8) | buff[parsed])) * 0.00122;
            data.cableTension[finger].sensor2 = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
            valid.cableTension[finger] = true;
        }
        parsed += 4;
    }

    if (sensor_mask & DATA_COLLECTION_MOTORCURRENT_BITMASK)
    {
        data.motorCurrent[finger] = ((float)((buff[parsed+1] << 8) | buff[parsed])) / 1000.0;
        valid.motorCurrent[finger] = true;
        parsed += 2;
    }

    if (sensor_mask & DATA_COLLECTION_MOTORSTATORTEMP_BITMASK)
    {
        data.motorHousingTemp[finger] = ((float)((buff[parsed+1] << 8) | buff[parsed])) / 100.0;
        valid.motorHousingTemp[finger] = true;
        parsed += 2;
    }

    if (sensor_mask & DATA_COLLECTION_MOTORVELOCITY_BITMASK)
    {
        data.motorVelocity[finger] = (buff[parsed+1] << 8) | buff[parsed];
        valid.motorVelocity[finger] = true;
        parsed += 2;
    }

    if (sensor_mask & DATA_COLLECTION_MOTORWINDINGTEMP_BITMASK)
    {
        data.motorWindingTemp[finger] = ((float)((buff[parsed+1] << 8) | buff[parsed])) / 100.0;
        valid.motorWindingTemp[finger] = true;
        parsed += 2;
    }
    
    if (sensor_mask & DATA_COLLECTION_MOTORHALL_BITMASK)
    {
        data.motorHallEncoder[finger] = (buff[parsed+1] << 8) | buff[parsed];
        valid.motorHallEncoder[finger] = true;

        // Turn signed short into signed int
        //if (data.motorHallEncoder[finger] & 0x8000)
        //    data.motorHallEncoder[finger] |= 0xFFFF0000;

        parsed += 2;
    }
    
    return parsed;
};

/// Parse the sensor data from the palm traffic cop board. 
// Take data from the buffer and put it into the HandSensors type.
//
// \param buff  The buffer of data from the microcontroller.  The first byte 
//              should already be lined up with the appropriate data.
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
//
// Returns the number of bytes parsed, or -1 on error.
int parsePalmCop(const unsigned char* const buff, 
                 const unsigned int sensor_mask, 
                 HandSensors& data,
                 HandSensorsValid& valid)
{
    int parsed = 0;
    
    if (sensor_mask & DATA_COLLECTION_FINGERROTATION_BITMASK)
    {
        data.fingerSpread = (buff[parsed+1] << 8) | buff[parsed];
        valid.fingerSpread = true;
        parsed += 2;
    }

    if (sensor_mask & DATA_COLLECTION_MOTORCURRENT_BITMASK)
    {
        data.motorCurrent[4] = (((float)((buff[parsed+1] << 8) | buff[parsed])) * 0.00122 - 1.25) * 0.134;
        valid.cableTension[4] = true;
        parsed += 2;
    }
    
    if (sensor_mask & DATA_COLLECTION_MOTORSTATORTEMP_BITMASK)
    {
        data.motorHousingTemp[4] = ((float)((buff[parsed+1] << 8) | buff[parsed])) / 100.0;
        valid.motorHousingTemp[4] = true;
        parsed += 2;
    }
    
    if (sensor_mask & DATA_COLLECTION_EXTERNALSUPPLY_BITMASK)
    {
        data.voltage.volts33 = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122 * 2.0;
        data.voltage.volts12 = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122 * 6.0;
        data.voltage.volts48 = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122 * 31.0;
        valid.voltage = true;
        parsed += 6;
    }
    
    if (sensor_mask & DATA_COLLECTION_AIRTEMPERATURE_BITMASK)
    {
        data.airTemp = ((float)((buff[parsed+1] << 8) | buff[parsed])) / 100.0;
        valid.airTemp = true;
        parsed += 2;
    }
    
    return parsed;
};

/// Parse the sensor data from a proximal board. 
// Take data from the buffer and put it into the HandSensors type.
//
// \param buff  The buffer of data from the microcontroller.  The first byte 
//              should already be lined up with the appropriate data.
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
// \finger The finger number. Range 0-2.
//
// Returns the number of bytes parsed, or -1 on error.
int parseProximal(const unsigned char* const buff,
                  const unsigned int sensor_mask,
                  HandSensors& data,
                  HandSensorsValid& valid,
                  const unsigned int finger) //0 to 2
{
    if (finger > 2)
        return -1;
    
    int parsed = 0;
    
    if (sensor_mask & DATA_COLLECTION_DYNAMIC_BITMASK)
    {
        data.fingerPVDF[finger].proximal[0] = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122;
        data.fingerPVDF[finger].proximal[1] = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
        data.fingerPVDF[finger].proximal[2] = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122;
        valid.fingerPVDF.proximal[finger] = true;
        parsed += 6;
    }
    
    if (sensor_mask & DATA_COLLECTION_DISTALJOINT_BITMASK)
    {
        data.distalJointAngle[finger].proximal[0] = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122;
        data.distalJointAngle[finger].proximal[1] = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
        data.distalJointAngle[finger].proximal[2] = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122;
        data.distalJointAngle[finger].proximal[3] = ((float)((buff[parsed+7] << 8) | buff[parsed+6])) * 0.00122;
        valid.distalJointAngle.proximal[finger] = true;
        parsed += 8;
    }
    
    if (sensor_mask & DATA_COLLECTION_PROXIMALJOINT_BITMASK)
    {
        data.proximalJointAngle[finger] = (buff[parsed+1] << 8) | buff[parsed];
        valid.proximalJointAngle[finger] = true;
        parsed += 2;
    }

    if (sensor_mask & DATA_COLLECTION_TACTILE_BITMASK)
    {
        for (int i=0; i<12; i++)
        {
            data.fingerTactile[finger].proximal[i] = (float)((buff[parsed+2*i+1] << 8) | buff[parsed+2*i]);
            // Turn signed short into signed int
            //if (data.fingerTactile[finger].proximal[i] & 0x8000)
            //    data.fingerTactile[finger].proximal[i] |= 0xFFFF0000; //hack
        }
        //data.fingerTactile[finger].proximal[i] = (float)buff[parsed+i] * 0.255 + 50.0;
        //data.fingerTactile[finger].proximal[i] = (float)buff[parsed+i];
        //data.fingerTactile[finger].proximal[i] = (float)((int8_t)buff[parsed+i]) * 0.255 + 50.0;
        //data.fingerTactile[finger].proximal[i] = (float)buff[parsed+i];
        //data.fingerTactile[finger].proximal[i] = (float)buff[parsed+i] * 0.255 + 50.0;
        valid.fingerTactile.proximal[finger] = true;
        parsed += 24; //12;
    }
    
    return parsed;
};

/// Parse the sensor data from a distal board. 
// Take data from the buffer and put it into the HandSensors type.
//
// \param buff  The buffer of data from the microcontroller.  The first byte 
//              should already be lined up with the appropriate data.
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
// \finger The finger number. Range 0-2.
//
// Returns the number of bytes parsed, or -1 on error.
int parseDistal(const unsigned char* const buff,
                const unsigned int sensor_mask,
                HandSensors& data,
                HandSensorsValid& valid,
                const unsigned int finger) //0 to 2
{
    if (finger > 2)
        return -1;
    
    int parsed = 0;
    
    if (sensor_mask & DATA_COLLECTION_ACCELERATION_BITMASK)
    {
        data.fingerAcceleration[finger].x = (buff[parsed+1] << 8) | buff[parsed+0];
        data.fingerAcceleration[finger].y = (buff[parsed+3] << 8) | buff[parsed+2];
        data.fingerAcceleration[finger].z = (buff[parsed+5] << 8) | buff[parsed+4];
        valid.fingerAcceleration[finger] = true;
        parsed += 6;
    }
    
    if (sensor_mask & DATA_COLLECTION_DYNAMIC_BITMASK)
    {
        data.fingerPVDF[finger].distal[0] = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122;
        data.fingerPVDF[finger].distal[1] = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
        data.fingerPVDF[finger].distal[2] = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122;
        valid.fingerPVDF.distal[finger] = true;
        parsed += 6;
    }
    
    if (sensor_mask & DATA_COLLECTION_DISTALJOINT_BITMASK)
    {
        data.distalJointAngle[finger].distal[0] = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122;
        data.distalJointAngle[finger].distal[1] = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
        data.distalJointAngle[finger].distal[2] = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122;
        data.distalJointAngle[finger].distal[3] = ((float)((buff[parsed+7] << 8) | buff[parsed+6])) * 0.00122;
        valid.distalJointAngle.distal[finger] = true;
        parsed += 8;
    }
    
    if (sensor_mask & DATA_COLLECTION_TACTILE_BITMASK)
    {
        for (int i=0; i<10; i++)
            data.fingerTactile[finger].distal[i] = (float)((buff[parsed+2*i+1] << 8) | buff[parsed+2*i]);
        //data.fingerTactile[finger].distal[i] = (float)buff[parsed+i] * 0.255 + 50.0;
        valid.fingerTactile.distal[finger] = true;
        parsed += 20;//10;
    }
    
    return parsed;
};

/// Parse the sensor data from the palm tactile sensor board. 
// Take data from the buffer and put it into the HandSensors type.
//
// \param buff  The buffer of data from the microcontroller.  The first byte 
//              should already be lined up with the appropriate data.
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
//
// Returns the number of bytes parsed, or -1 on error.
int parseTactile(const unsigned char* const buff,
                 const unsigned int sensor_mask,
                 HandSensors& data,
                 HandSensorsValid& valid)
{
    int parsed = 0;
    
    if (sensor_mask & DATA_COLLECTION_DYNAMIC_BITMASK)
    {
        data.palmPVDF[0] = ((float)((buff[parsed+1] << 8) | buff[parsed+0])) * 0.00122;
        data.palmPVDF[1] = ((float)((buff[parsed+3] << 8) | buff[parsed+2])) * 0.00122;
        data.palmPVDF[2] = ((float)((buff[parsed+5] << 8) | buff[parsed+4])) * 0.00122;
        data.palmPVDF[3] = ((float)((buff[parsed+7] << 8) | buff[parsed+6])) * 0.00122;
        valid.palmPVDF = true;
        parsed += 8;
    }
    
    if (sensor_mask & DATA_COLLECTION_TACTILE_BITMASK)
    {
        for (int i=0; i<48; i++)
            data.palmTactile[i] = (float)((buff[parsed+2*i+1] << 8) | buff[parsed+2*i]);
        
        //data.palmTactile[i] = (float)buff[parsed+i] * 0.255 + 50.0;
        valid.palmTactile = true;
        parsed += 48*2;
    }
    
    return parsed;
};

/// Read the data from the sensor payload and load it into the HandSensors 
// type.  Calls the above specialized parsers for each board type.
//
// \param buff  The buffer of data from the microcontroller. 
// \param sensor_mask  The bitmask of sensors requested.
// \param device_mask  The bitmask of responding devices.
// \param[out] data   The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor. 
//
// Returns the number of bytes parsed, or -1 on error.
int parseData(const unsigned char* const buff, 
              int sensor_mask, 
              int device_mask,
              HandSensors& data,
              HandSensorsValid& valid)
{
#ifdef DEBUG_PARSER
    printf("parseData\n");
#endif
    int parsed = 0;
    
    // palm (traffic cop)
    if (device_mask & RESPONDING_DEVICES_PALM_BITMASK)
    {
        int tmpparsed = parsePalmCop(&(buff[parsed]), sensor_mask, data, valid);
        if (tmpparsed < 0)
        {
            printf("ERROR PARSING PALM BOARD DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    // finger 1 proximal
    if (device_mask & RESPONDING_DEVICES_FIRST_PROX_BITMASK)
    {
        int tmpparsed = parseProximal(&(buff[parsed]), sensor_mask, data, valid, fingerChain_to_fingerNumber[0]);
        if (parsed < 0)
        {
            printf("ERROR PARSING FINGER 1 PROXIMAL DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    // finger 1 distal
    if (device_mask & RESPONDING_DEVICES_FIRST_DIST_BITMASK)
    {
        int tmpparsed = parseDistal(&(buff[parsed]), sensor_mask, data, valid, fingerChain_to_fingerNumber[0]);
        if (parsed < 0)
        {
            printf("ERROR PARSING FINGER 1 DISTAL DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    // finger 2 proximal
    if (device_mask & RESPONDING_DEVICES_SECOND_PROX_BITMASK)
    {
        int tmpparsed = parseProximal(&(buff[parsed]), sensor_mask, data, valid, fingerChain_to_fingerNumber[1]);
        if (parsed < 0)
        {
            printf("ERROR PARSING FINGER 2 PROXIMAL DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    // finger 2 distal
    if (device_mask & RESPONDING_DEVICES_SECOND_DIST_BITMASK)
    {
        int tmpparsed = parseDistal(&(buff[parsed]), sensor_mask, data, valid, fingerChain_to_fingerNumber[1]);
        if (parsed < 0)
        {
            printf("ERROR PARSING FINGER 2 DISTAL DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    // finger 3 proximal
    if (device_mask & RESPONDING_DEVICES_THIRD_PROX_BITMASK)
    {
        int tmpparsed = parseProximal(&(buff[parsed]), sensor_mask, data, valid, fingerChain_to_fingerNumber[2]);
        if (parsed < 0)
        {
            printf("ERROR PARSING FINGER 3 PROXIMAL DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    // finger 3 distal
    if (device_mask & RESPONDING_DEVICES_THIRD_DIST_BITMASK)
    {
        int tmpparsed = parseDistal(&(buff[parsed]), sensor_mask, data, valid, fingerChain_to_fingerNumber[2]);
        if (parsed < 0)
        {
            printf("ERROR PARSING FINGER 3 DISTAL DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    // motor 1
    if (device_mask & RESPONDING_DEVICES_FIRST_MOTOR1_BITMASK)
    {
        int tmpparsed = parseMotor(&(buff[parsed]), sensor_mask, data, valid, motorChain_to_fingerNumber[0]);
        if (parsed < 0)
        {
            printf("ERROR PARSING MOTOR 1 DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }

    // motor 2
    if (device_mask & RESPONDING_DEVICES_FIRST_MOTOR2_BITMASK)
    {
        int tmpparsed = parseMotor(&(buff[parsed]), sensor_mask, data, valid, motorChain_to_fingerNumber[1]);
        if (parsed < 0)
        {
            printf("ERROR PARSING MOTOR 2 DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    // motor 3
    if (device_mask & RESPONDING_DEVICES_SECOND_MOTOR1_BITMASK)
    {
        int tmpparsed = parseMotor(&(buff[parsed]), sensor_mask, data, valid, motorChain_to_fingerNumber[2]);
        if (parsed < 0)
        {
            printf("ERROR PARSING MOTOR 3 DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    // motor 4
    if (device_mask & RESPONDING_DEVICES_SECOND_MOTOR2_BITMASK)
    {
        int tmpparsed = parseMotor(&(buff[parsed]), sensor_mask, data, valid, motorChain_to_fingerNumber[3]);
        if (parsed < 0)
        {
            printf("ERROR PARSING MOTOR 4 DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }

    // tactile
    if (device_mask & RESPONDING_DEVICES_TACTILE_BITMASK)
    {
        int tmpparsed = parseTactile(&(buff[parsed]), sensor_mask, data, valid);
        if (parsed < 0)
        {
            printf("ERROR PARSING TACTILE DATA\n");
            return -1;
        }
        parsed += tmpparsed;
    }
    
    return parsed;
};


/// Just like standard serial read(), but blocking for up to n chars.  
// Blocks for up to 1 second.
//
// \param fd   The serial file descriptor to use.
// \param[out] buff  Where to put the data.
// \param n    Read this number of bytes.
// \param timeout  Number of microseconds to wait for.
//
// Returns the number of bytes read, or -1 on timeout.
int blockingread(int fd, unsigned char* buff, int n, unsigned int timeout = 1000000)
{
    int rd = 0;
    unsigned int time = 0;
    while (rd < n)
    {
        //printf("%d ", time);
        
        int r = read(fd, &(buff[rd]), n-rd);
        if (r <= 0)
        {
            time += 100;
            if (time >= timeout)
                return -1;
            usleep(100);
            continue;
        }
        rd += r;
    }
    return rd;
};

/// Read a standard command response from palm traffic cop microcontroller.
//
// \param fd   The serial file descriptor.
// \param[out] len  Returns entire length of message (including initial 2 
//                  bytes for packet length).
//
// Returns error code. (0 on success, larger on error)
int readResponse(int fd, int& len)
{
    unsigned char smbuff[2];
    len = 0;
    
    // first read packet size
    int ret = blockingread(fd, smbuff, 2);
    if (ret <= 0)
        return -1;
    int packetlen = (smbuff[1] << 8) | smbuff[0];
#ifdef DEBUG_PARSER
    printf("read %d bytes\n", ret);
    printf("packet len is: %d\n", packetlen);
#endif
    unsigned char* buff = new unsigned char[packetlen+2];
    buff[0] = smbuff[0];
    buff[1] = smbuff[1];

    // now read packet
    ret = blockingread(fd, &(buff[2]), packetlen);
    if (ret <= 0)
    {
        delete[] buff;
        return -1;
    }
#ifdef DEBUG_PARSER
    printf("read packet of length: %d\n", ret);
#endif
    // verify checksum
    if (computeChecksum(buff, packetlen+2) != 0)
    {
        delete[] buff;
        return 0x11;
    }
    // todo: verify reflected command byte?
    
#ifdef DEBUG_PARSER
    for (int i=0; i<packetlen+2; i++)
        printf("%02X ", buff[i]);
    printf("\n");
#endif

    len = packetlen+2;
    ret = buff[RESPONSE_STATUSCODE_OFFSET];
    delete[] buff;
    return ret;
};


/// Read a standard command response from palm traffic cop microcontroller.
// Same as above readResponse, but does not give you the length of the read
//  message
//
// \param fd   The serial file descriptor.
//
// Returns error code. (0 on success, larger on error)
int readResponse(int fd)
{
    int len;
    return readResponse(fd, len);
};

/// Read a standard command response from palm traffic cop microcontroller.
// This version gives you the response packet to inspect.
//
// \param fd   The serial file descriptor.
// \param buff  Where to put the response.
// \param maxlen  the max length of the buffer.
// \param[out] len  Returns entire length of message (including initial 2 
//                  bytes for packet length).
//
// Returns error code. (0 on success, larger on error)
int readResponse(int fd, unsigned char* buff, int maxlen, int& len)
{
    len = 0;
    
    // first read packet size
    int ret = blockingread(fd, buff, (2<maxlen)?2:maxlen);
    if (ret < 2)
        return -1;

    int packetlen = (buff[1] << 8) | buff[0];

#ifdef DEBUG_PARSER
    printf("read %d bytes\n", ret);
    printf("packet len is: %d\n", packetlen);
#endif
    
    // now read packet
    ret = blockingread(fd, &(buff[2]), (packetlen<maxlen-2)?packetlen:maxlen-2);

#ifdef DEBUG_PARSER
    printf("read packet of length: %d\n", ret);
#endif
    
    if (ret < packetlen)
        return -1;

    // verify checksum
    if (computeChecksum(buff, packetlen+2) != 0)
        return 0x11;
    // todo: verify reflected command byte?
    
#ifdef DEBUG_PARSER
    for (int i=0; i<packetlen+2; i++)
        printf("%02X ", buff[i]);
    printf("\n");
#endif

    len = packetlen+2;
    ret = buff[RESPONSE_STATUSCODE_OFFSET];
    return ret;
};


// read sensor data 
// void readSensors(int fd, char* buff, int sensor_mask)
// {
//     // first read packet size
//     int ret = blockingread(fd, buff, 2);
//     if (ret < 0)
//     {
//         //printf("READ ERROR\n");
//         return;
//     }
//     //return r1;
//     int packetlen = (buff[1] << 8) | buff[0];
    
//     printf("read %d bytes\n", ret);
//     printf("packet len is: %d\n", packetlen);
    
//     // now read packet
//     ret = blockingread(fd, &(buff[2]), packetlen);
//     if (ret < 0)
//     {
//         //printf("READ ERROR\n");
//         return;
//     }
//     //return r1;

//     printf("read packet of length: %d\n", ret);

//     // verify checksum
//     if (computeChecksum(buff, packetlen+2) != 0)
//         printf("CHECKSUM ERROR\n");
//     // return 0x11;
    
//     if (buff[RESPONSE_BROADCAST_REFLECTEDOPCODE] != DATA_COLLECTION_OPCODE)
//         printf("OPCODE REFLECTION ERROR\n");
    
//     int respondingDevices = (buff[RESPONSE_BROADCAST_RESPONDINGDEVICES+1] << 8) | buff[RESPONSE_BROADCAST_RESPONDINGDEVICES];
//     int datalen = computeDataSize(sensor_mask, respondingDevices);
//     if (datalen != packetlen-4)
//     {
//         printf("\n\n\nDATA LENGTH ERROR\n");
//         printf("responding Devices = 0x%04X\n", respondingDevices);
//         printf("sensor mask        = 0x%04X\n", sensor_mask);
//         printf("computed Data Size = 0x%02X\n", datalen);
//         printf("packet len - 4 = %d\n", packetlen-4);
//     }

//     for (int i=0; i<packetlen+2; i++)
//         printf("%02X ", buff[i]);
//     printf("\n");
    
//     //return buff[RESPONSE_STATUSCODE_OFFSET];
// };


// /// Read response or sensor data from MCU.
// // If it is sensor data, sensorMsg will be set to true, and data and valid bits will be filled in to passed in references.
// // If it is a motor response, sensorMsg will be false
// // returns an error code, 0 for success.
// int parseResponse(int fd, int sensor_mask, bool& sensorMsg, HandSensors& data, HandSensorsValid& valid)
// {
// #ifdef DEBUG_PARSER
//     printf("parseResponse\n");
// #endif

//     char smbuff[2];
//     sensorMsg = false;
// #ifdef DEBUG_PARSER
//     printf("entering first blocking read\n");
// #endif
//     // first read packet size
//     int ret = blockingread(fd, smbuff, 2);
//     if (ret <= 0)
//         return -1;
    
//     // packetlen is length of packet without leading 2 size bytes
//     int packetlen = (smbuff[1] << 8) | smbuff[0];

// #ifdef DEBUG_PARSER
//     printf("read %d bytes\n", ret);
//     printf("packet len is: %d\n", packetlen);
// #endif
//     char* buff = new char[packetlen+2];
//     buff[0] = smbuff[0];
//     buff[1] = smbuff[1];

//     // now read packet
//     ret = blockingread(fd, &(buff[2]), packetlen);
//     if (ret <= 0)
//     {
//         delete[] buff;
//         return -1;
//     }
// #ifdef DEBUG_PARSER
//     printf("read packet of length: %d\n", ret);
// #endif
//     // verify checksum
//     if (computeChecksum(buff, packetlen+2) != 0)
//     {
//         delete[] buff;
//         return 0x11;
//     }

//     if (buff[RESPONSE_REFLECTEDOPCODE_OFFSET] == DATA_COLLECTION_OPCODE)
//     {
//         // sensor message
//         sensorMsg = true;
        
//         // determine which boards responded
//         int respondingDevices = (buff[RESPONSE_BROADCAST_RESPONDINGDEVICES+1] << 8) | buff[RESPONSE_BROADCAST_RESPONDINGDEVICES];
        
//         // determine how large the expected reply should be
//         int datalen = computeDataSize(sensor_mask, respondingDevices);
        
//         // data length error
//         if (datalen != packetlen-4)
//         {
// #ifdef DEBUG_PARSER
//             printf("\n\ndata length error\n");
//             printf("sensor mask: %d\n", sensor_mask);
//             printf("device mask: %d\n", respondingDevices);
//             printf("computed datalen: %d\n", datalen);
//             printf("packetlen-4: %d\n", packetlen-4);
// #endif
//             delete[] buff;
//             return 0x12;
//         }
        
//         int parsed = parseData(&(buff[RESPONSE_BROADCAST_PAYLOAD]), sensor_mask, respondingDevices, data, valid);

//         // i don't think this should happen, but just in case.
//         if (parsed != datalen)
//         {
// #ifdef DEBUG_PARSER
//             printf("\n\nparsed length error\n");
//             printf("sensor mask: %d\n", sensor_mask);
//             printf("device mask: %d\n", respondingDevices);
//             printf("computed datalen: %d\n", datalen);
//             printf("packetlen-4: %d\n", packetlen-4);
//             printf("parsed length: %d\n", parsed);
// #endif
//             delete[] buff;
//             return 0x13;
//         }

// #ifdef DEBUG_PARSER        
//         printf("Sensor mask: %04X\n", sensor_mask);
//         printf("Device mask: %04X\n", respondingDevices);
//         for (int i=0; i<packetlen+2; i++)
//             printf("%02X ", buff[i]);
//         printf("\n");
// #endif
//         // success
//         delete[] buff;
//         return 0x00;
//     }
//     else
//     {
//         // command response
//         sensorMsg = false;

// #ifdef DEBUG_PARSER        
//         printf("Response code: %02X\n", buff[RESPONSE_STATUSCODE_OFFSET]);
// #endif
       
//         // success, return error code (if any).
//         ret = buff[RESPONSE_STATUSCODE_OFFSET];
//         delete[] buff;
//         return ret;
//     }
// };
                  

/// Read response or parse sensor data from palm traffic cop microcontroller.
// If it is sensor data, sensorMsg will be set to true, and data and valid 
// bits will be filled in to passed in references. If it is a motor response, 
// sensorMsg will be false.
//
// \param fd  The serial file descriptor
// \param sensor_mask  The bitmask of sensors requested.
// \param[out] sensorMsg  Will be set to true if message was a sensor message.
// \param[out] data  The HandSensors type where to load the data.
// \param[out] valid  The booleans in this type will be set to true if there is
//                    data for that sensor.
// \param[out] buff  A buffer to use when recieving data.
// \param maxlen  The size of the buffer.
//
// Returns an error code, 0 for success.
int parseResponse(int fd, int sensor_mask, bool& sensorMsg, HandSensors& data, HandSensorsValid& valid, unsigned char* buff, int maxlen)
{
#ifdef DEBUG_PARSER
    printf("parseResponse\n");
#endif

    sensorMsg = false;
#ifdef DEBUG_PARSER
    printf("entering first blocking read\n");
#endif
    // first read packet size
    int ret = blockingread(fd, buff, 2);
    if (ret <= 0)
    {
#ifdef DEBUG_PARSER
        //printf("ERROR: %d: %s\n", errno, strerror( errno ) );
        // EAGAIN = 11
        // EWOULDBLOCK = 11
        // EBADF = 9
        // EFAULT = 14
        // EINTR = 4
        // EINVAL = 22
        // EINVAL = 22
        // EIO = 5
        // EISDIR = 21
#endif
        return -1;
    }
    
    // packetlen is length of packet without leading 2 size bytes
    int packetlen = (buff[1] << 8) | buff[0];

#ifdef DEBUG_PARSER
    printf("read %d bytes\n", ret);
    printf("packet len is: %d\n", packetlen);
#endif
    
    if (packetlen+2 > maxlen)
        return -2;

    // now read packet
    ret = blockingread(fd, &(buff[2]), packetlen);
    if (ret <= 0)
    {
#ifdef DEBUG_PARSER
        //printf("ERROR: %s\n", strerror( errno ) );
#endif
        return -1;
    }
#ifdef DEBUG_PARSER
    printf("read packet of length: %d\n", ret);
#endif
    // verify checksum
    if (computeChecksum(buff, packetlen+2) != 0)
    {
        return 0x11;
    }

    if (buff[RESPONSE_REFLECTEDOPCODE_OFFSET] == DATA_COLLECTION_OPCODE)
    {
        // sensor message
        sensorMsg = true;
        
        // determine which boards responded
        int respondingDevices = (buff[RESPONSE_BROADCAST_RESPONDINGDEVICES+1] << 8) | buff[RESPONSE_BROADCAST_RESPONDINGDEVICES];
        
        // determine how large the expected reply should be
        int datalen = computeDataSize(sensor_mask, respondingDevices);
        
        // data length error
        if (datalen != packetlen-4)
        {
#ifdef DEBUG_PARSER
            printf("\n\ndata length error\n");
            printf("sensor mask: %04X\n", sensor_mask);
            printf("device mask: %04X\n", respondingDevices);
            printf("computed datalen: %d\n", datalen);
            printf("packetlen-4: %d\n", packetlen-4);
#endif
            return 0x12;
        }
        
        int parsed = parseData(&(buff[RESPONSE_BROADCAST_PAYLOAD]), sensor_mask, respondingDevices, data, valid);

        // i don't think this should happen, but just in case.
        if (parsed != datalen)
        {
#ifdef DEBUG_PARSER
            printf("\n\nparsed length error\n");
            printf("sensor mask: %d\n", sensor_mask);
            printf("device mask: %d\n", respondingDevices);
            printf("computed datalen: %d\n", datalen);
            printf("packetlen-4: %d\n", packetlen-4);
            printf("parsed length: %d\n", parsed);
#endif
            return 0x13;
        }

#ifdef DEBUG_PARSER        
        printf("Sensor mask: %04X\n", sensor_mask);
        printf("Device mask: %04X\n", respondingDevices);
        for (int i=0; i<packetlen+2; i++)
            printf("%02X ", buff[i]);
        printf("\n");
#endif
        // success
        return 0x00;
    }
    else
    {
        // command response
        sensorMsg = false;

#ifdef DEBUG_PARSER        
        printf("Response code: %02X\n", buff[RESPONSE_STATUSCODE_OFFSET]);
#endif
       
        // success, return error code (if any).
        ret = buff[RESPONSE_STATUSCODE_OFFSET];
        return ret;
    }
};

/// Parse a motor parameter from a response payload.
// Note that this is big-endian, despite the rest of the code being little endian.
//
// \param buff  The response payload
//
// Returns the floating point value.
float parseParameter(unsigned char* buff)
{
    float x = 0;
    unsigned char* ptr = (unsigned char*)(&x);
    *ptr = buff[0]; ptr++;
    *ptr = buff[1]; ptr++;
    *ptr = buff[2]; ptr++;
    *ptr = buff[3];
    return x;
};



#endif

