/**
 * \file handleTypes.hpp
 *
 * Mostly #defines from firmware.  With some helper functions for checksums 
 * and setting the payload.  Also has data length computation, finger to motor
 * number mapping, and the HandSensors type.
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

#ifndef HANDLE_TYPES_H
#define HANDLE_TYPES_H

#include <stdint.h> //short
#include <stdio.h> //printf
#include <sys/time.h> //timeval

#include "packing.hpp" //pack unpack

// Definitions from:
// 120306-2 Palm Micro FW Release Rev00/PLMMCU_0_00/src/C1482-SRC-PLMMCU-0_router.h

#define RESPONSE_PACKETSIZE_OFFSET_LSB 0
#define RESPONSE_PACKETSIZE_OFFSET_MSB 1
#define RESPONSE_REFLECTEDOPCODE_OFFSET 2
#define RESPONSE_STATUSCODE_OFFSET 3
#define RESPONSE_PAYLOAD_OFFSET 4

#define RESPONSE_BROADCAST_PACKETSIZE_LSB 0
#define RESPONSE_BROADCAST_PACKETSIZE_MSB 1
#define RESPONSE_BROADCAST_REFLECTEDOPCODE 2
#define RESPONSE_BROADCAST_RESPONDINGDEVICES 3
#define RESPONSE_BROADCAST_PAYLOAD 5

#define STATUS_OK 0x00
#define CHECKSUM_ERROR 0x01
#define UNKNOWN_COMMAND 0x02
#define BAD_ADDRESS 0x03
#define TIMEOUT_ERROR 0x04
#define DELAYED_ERROR 0x80
#define MANDATORY_COOLDOWN 0x81
#define OUT_OF_RANGE 0x82

#define DESTINATION_HEADER_OFFSET 0
#define COMMAND_OFFSET 1
#define PAYLOAD_OFFSET 2
#define CHECKSUM_OFFSET 6
#define COMMAND_PACKET_LENGTH 7 // BA

#define CHAINADDRESS_BITMASK 0xF0
#define PALM_CHAINADDRESS    0x00
#define FINGER1_CHAINADDRESS 0x10
#define FINGER2_CHAINADDRESS 0x20
#define FINGER3_CHAINADDRESS 0x30
#define MOTOR1_CHAINADDRESS  0x40 // motors 1 and 2
#define MOTOR2_CHAINADDRESS  0x50 // motors 3 and 4
#define TACTILE_CHAINADDRESS 0x60

// this set is all BA
#define CHAININDEX_BITMASK      0x0F
#define PALM_CHAININDEX         0x00
#define FINGER1_PROX_CHAININDEX 0x00
#define FINGER1_DIST_CHAININDEX 0x01
#define FINGER2_PROX_CHAININDEX 0x00
#define FINGER2_DIST_CHAININDEX 0x01
#define FINGER3_PROX_CHAININDEX 0x00
#define FINGER3_DIST_CHAININDEX 0x01
#define MOTOR1_CHAININDEX       0x00
#define MOTOR2_CHAININDEX       0x01
#define MOTOR3_CHAININDEX       0x00
#define MOTOR4_CHAININDEX       0x01
#define TACTILE_CHAININDEX      0x00

// this set is all BA
#define DESTINATION_BROADCAST      0xFF
#define DESTINATION_PALM           PALM_CHAINADDRESS | PALM_CHAININDEX
#define DESTINATION_FINGER1_PROX   FINGER1_CHAINADDRESS | FINGER1_PROX_CHAININDEX
#define DESTINATION_FINGER1_DIST   FINGER1_CHAINADDRESS | FINGER1_DIST_CHAININDEX
#define DESTINATION_FINGER2_PROX   FINGER2_CHAINADDRESS | FINGER2_PROX_CHAININDEX
#define DESTINATION_FINGER2_DIST   FINGER2_CHAINADDRESS | FINGER2_DIST_CHAININDEX
#define DESTINATION_FINGER3_PROX   FINGER3_CHAINADDRESS | FINGER3_PROX_CHAININDEX
#define DESTINATION_FINGER3_DIST   FINGER3_CHAINADDRESS | FINGER3_DIST_CHAININDEX
#define DESTINATION_MOTOR1         MOTOR1_CHAINADDRESS | MOTOR1_CHAININDEX
#define DESTINATION_MOTOR2         MOTOR1_CHAINADDRESS | MOTOR2_CHAININDEX
#define DESTINATION_MOTOR3         MOTOR2_CHAINADDRESS | MOTOR3_CHAININDEX
#define DESTINATION_MOTOR4         MOTOR2_CHAINADDRESS | MOTOR4_CHAININDEX
#define DESTINATION_TACTILE        TACTILE_CHAINADDRESS | TACTILE_CHAININDEX

// from python test code
// DESTINATION_PALM = 0x00
// DESTINATION_TACTILE = 0x60
// DESTINATION_BROADCAST = 0xFF
// DESTINATION_PROX1 = 0x10

#define CHAININDEX_BITMASK 0x0F
#define NUMCHAINS 6

#define OPCODE_BITMASK 0xF0

#define DATA_COLLECTION_OPCODE      0x00
#define SET_SAMPLE_PERIOD_OPCODE    0x10
#define SET_SAMPLE_ARGS_OPCODE      0x20
#define START_COLLECTION_OPCODE     0x30
#define STOP_COLLECTION_OPCODE      0x40
#define FINGER_COMMAND_OPCODE       0x50
#define MOTOR_PARAMETER_RE_L_OPCODE 0x60
#define MOTOR_PARAMETER_RE_H_OPCODE 0x70
#define MOTOR_PARAMETER_WR_L_OPCODE 0x80
#define MOTOR_PARAMETER_WR_H_OPCODE 0x90
#define MOTOR_COMMAND_OPCODE        0xA0

#define PARAMETER_ADDRESS_BITMASK 0x1F

#define PARAMETER_TORQUE_KP        0
#define PARAMETER_TORQUE_KI        1
#define PARAMETER_TORQUE_KD        2
#define PARAMETER_VELOCITY_KP      3
#define PARAMETER_VELOCITY_KI      4
#define PARAMETER_VELOCITY_KD      5
#define PARAMETER_POWER_KP         6
#define PARAMETER_POWER_KI         7
#define PARAMETER_POWER_KD         8
#define PARAMETER_WINDING_R        9
#define PARAMETER_THERMAL_R       10
#define PARAMETER_T_PLUS          11
#define PARAMETER_T_MINUS         12
#define PARAMETER_WINDING_TAU     13
#define PARAMETER_T_MAX           14
#define PARAMETER_CU_ALPHA        15
#define PARAMETER_OFF_TIME        16
#define PARAMETER_T_TARGET        17
#define PARAMETER_MAXIMUM_RPM     18
#define PARAMETER_SPEED_CONSTANT  19
#define PARAMETER_MAXIMUM_COMMAND 20

#define DATA_COLLECTION_ALL_BITMASK              0xFFFF
#define DATA_COLLECTION_ACCELERATION_BITMASK     0x8000
#define DATA_COLLECTION_DYNAMIC_BITMASK          0x4000
#define DATA_COLLECTION_TENSION_BITMASK          0x2000
#define DATA_COLLECTION_DISTALJOINT_BITMASK      0x1000
#define DATA_COLLECTION_PROXIMALJOINT_BITMASK    0x0800
#define DATA_COLLECTION_FINGERROTATION_BITMASK   0x0400
#define DATA_COLLECTION_TACTILE_BITMASK          0x0200
#define DATA_COLLECTION_MOTORCURRENT_BITMASK     0x0100
#define DATA_COLLECTION_MOTORSTATORTEMP_BITMASK  0x0080
#define DATA_COLLECTION_MOTORVELOCITY_BITMASK    0x0040
#define DATA_COLLECTION_EXTERNALSUPPLY_BITMASK   0x0020
#define DATA_COLLECTION_AIRTEMPERATURE_BITMASK   0x0010
#define DATA_COLLECTION_MOTORWINDINGTEMP_BITMASK 0x0008
#define DATA_COLLECTION_MOTORHALL_BITMASK        0x0004 //BA

#define RESPONDING_DEVICES_PALM_BITMASK          0x0001 // 1
#define RESPONDING_DEVICES_FIRST_PROX_BITMASK    0x0002 // 2
#define RESPONDING_DEVICES_FIRST_DIST_BITMASK    0x0004 // 3
#define RESPONDING_DEVICES_SECOND_PROX_BITMASK   0x0008 // 4
#define RESPONDING_DEVICES_SECOND_DIST_BITMASK   0x0010 // 5
#define RESPONDING_DEVICES_THIRD_PROX_BITMASK    0x0020 // 6
#define RESPONDING_DEVICES_THIRD_DIST_BITMASK    0x0040 // 7
#define RESPONDING_DEVICES_FIRST_MOTOR1_BITMASK  0x0080 // 8
#define RESPONDING_DEVICES_FIRST_MOTOR2_BITMASK  0x0100 // 9
#define RESPONDING_DEVICES_SECOND_MOTOR1_BITMASK 0x0200 // 10
#define RESPONDING_DEVICES_SECOND_MOTOR2_BITMASK 0x0400 // 11
#define RESPONDING_DEVICES_TACTILE_BITMASK       0x0800 // 12

#define MOTOR_COMMAND_DIRECTION_BITMASK 0x0C
#define MOTOR_COMMAND_STOP 0x00
#define MOTOR_COMMAND_REVERSE 0x04
#define MOTOR_COMMAND_FORWARD 0x08 // BA: these were initially swapped.  making it match the document makes the behavior correct.

#define MOTOR_COMMAND_SCHEME_BITMASK 0x03
#define MOTOR_COMMAND_CURRENT 0x01
#define MOTOR_COMMAND_VOLTAGE 0x02
#define MOTOR_COMMAND_VELOCITY 0x03

#define N_BOARDS 12
#define N_BOARD_TYPES 5
#define N_SENSOR_TYPES 14

/************************************************************************
 * Returns the appropriate checksum for the first packetSize bytes of 
 * packetBuffer as a shortcut, feeding an entire packet to this routine should
 * result in a return value of zero for a proper packet.
 ************************************************************************/
int computeChecksum(const unsigned char* const packetBuffer, int packetSize)
{
	unsigned int accumulator = 0;
	for(int i=0;i<packetSize;i++)
	{
		accumulator += packetBuffer[i];
	}	
	
	return (0x00 - (0xFF & accumulator));
}

/// Set the payload with an integer value.
//
// \param packetBuffer A pointer to the packet to send, not the payload part.
// \param payload  The integer value to send.
void setPayload(unsigned char* const packetBuffer, unsigned int payload)
{
    packetBuffer[PAYLOAD_OFFSET]     = (char)((payload >> 0) & 0xFF);
    packetBuffer[PAYLOAD_OFFSET + 1] = (char)((payload >> 8) & 0xFF);
    packetBuffer[PAYLOAD_OFFSET + 2] = (char)((payload >> 16) & 0xFF);
    packetBuffer[PAYLOAD_OFFSET + 3] = (char)((payload >> 24) & 0xFF);
};

/// Set the payload with a floating point value.
// Note that this is big-endian despite the rest of the code being little-endian.
//
// \param packetBuffer A pointer to the packet to send, not the payload part.
// \param payload  The floating point value to send.
void setPayloadf(unsigned char* const packetBuffer, float payload)
{
    unsigned char* ptr = (unsigned char*)(&payload) ;
    packetBuffer[PAYLOAD_OFFSET + 0] = *ptr; ptr++;
    packetBuffer[PAYLOAD_OFFSET + 1] = *ptr; ptr++;
    packetBuffer[PAYLOAD_OFFSET + 2] = *ptr; ptr++;
    packetBuffer[PAYLOAD_OFFSET + 3] = *ptr;
};

// lookup table to convert device index into device type for dataSizeMatrix[].
// see tables 1, 7 and 8.
const uint8_t deviceType[N_BOARDS] = {4, 1, 0, 1, 0, 1, 0, 2, 2, 2, 2, 3};

// matrix representing different sensor types, owner boards, and data widths.
// rows of sensor type, columns of board type.
//
// columns:
// distal board,   proximal board,   motor board,   palm (tact) board,   traffic cop (palm) board
//
// rows: 
// Motor Hall, Motor winding temp, Air temp, External supply voltage, Motor Velocity, Stator temp, Motor Current, Tactile Array, Finger Rotation, Proximal Joint Angle, Distal Joint Angle, Cable Tension, Dynamic, Acceleration
//
// see tables 1, 7 and 8.
const uint8_t dataSizeMatrix[N_BOARD_TYPES * N_SENSOR_TYPES] = {
    0,    0,   2,    0,   0, // Motor Hall
    0,    0,   2,    0,   0, // Motor winding temp
    0,    0,   0,    0,   2, // Air temp
    0,    0,   0,    0,   6, // External supply voltage
    0,    0,   2,    0,   0, // Motor Velocity
    0,    0,   2,    0,   2, // Stator temp
    0,    0,   2,    0,   2, // Motor Current
    20,   24,  0,   96,   0, // Tactile Array
    0,    0,   0,    0,   2, // Finger Rotation
    0,    2,   0,    0,   0, // Proximal Joint Angle
    8,    8,   0,    0,   0, // Distal Joint Angle
    0,    0,   4,    0,   0, // Cable Tension
    6,    6,   0,    8,   0, // Dynamic
    6,    0,   0,    0,   0};// Acceleration

/// Determine how large the reply should be.  
// Based on the sensors requested and which devices responded.
int computeDataSize(int sensor_mask, int device_mask)
{
    int size = 0;
    for (int i=0; i<N_BOARDS; i++)
    {
        if (device_mask & (0x1 << i))
        {
            for (int j=0; j<N_SENSOR_TYPES; j++)
            {
                if (sensor_mask & (0x1 << (j+(16-N_SENSOR_TYPES))))
                    size += dataSizeMatrix[N_BOARD_TYPES * j + deviceType[i]];
            }
        }
    }
    
    return size;
};

struct HandVoltage
{
    float volts33;
    float volts12;
    float volts48;
};

struct FingerTactile
{
    short distal[10];
    short proximal[12];
};

struct FingerDistalAngle
{
    float distal[4];
    float proximal[4];
};

struct FingerCableTension
{
    float sensor1;
    float sensor2;
};

struct FingerPVDF
{
    float distal[3];
    float proximal[3];
};

struct FingerAcceleration
{
    short x;
    short y;
    short z;
};

struct FingerValid
{
    bool distal[3];
    bool proximal[3];
};

/// Class with dirty bits for all of the hand sensor data.
class HandSensorsValid
{
 public:
    bool motorHallEncoder[4];
    bool motorWindingTemp[4];
    bool airTemp;
    bool voltage;
    bool motorVelocity[4];
    bool motorHousingTemp[5];
    bool motorCurrent[5];
    FingerValid fingerTactile;
    bool palmTactile;
    bool fingerSpread;
    bool proximalJointAngle[3];
    FingerValid distalJointAngle;
    bool cableTension[2];
    FingerValid fingerPVDF;
    bool palmPVDF;
    bool fingerAcceleration[3];
    
    /// Constructor.
    // Initialize data to false, or true if passed in.
    HandSensorsValid(bool valid = false)
    {
        reset(valid);
    }
    
    /// Re-initialize data to false, or true if passed in.
    void reset(bool valid = false)
    {
        for (int i=0; i<2; i++)
            cableTension[i] = valid;
        
        for (int i=0; i<3; i++)
        {
            proximalJointAngle[i] = valid;
            fingerAcceleration[i] = valid;
            fingerTactile.distal[i] = valid;
            fingerTactile.proximal[i] = valid;
            distalJointAngle.distal[i] = valid;
            distalJointAngle.proximal[i] = valid;
            fingerPVDF.distal[i] = valid;
            fingerPVDF.proximal[i] = valid;
        }
        
        for (int i=0; i<4; i++)
        {
            motorHallEncoder[i] = valid;
            motorWindingTemp[i] = valid;
            motorVelocity[i] = valid;
        }
        
        for (int i=0; i<5; i++)
        {
            motorHousingTemp[i] = valid;
            motorCurrent[i] = valid;
        }
        
        airTemp = valid;
        voltage = valid;
        palmTactile = valid;
        fingerSpread = valid;
        palmPVDF = valid;
    };

    /// Print data
    void print() const
    {
        printf("Motor Hall: ");
        for (int i=0; i<4; i++)
            printf("%d ", motorHallEncoder[i]);
        printf("\n");
        
        printf("Motor Winding Temp: ");
        for (int i=0; i<4; i++)
            printf("%d ", motorWindingTemp[i]);
        printf("\n");
        
        printf("Air Temp: ");
        printf("%d ", airTemp);
        printf("\n");
        
        printf("Voltage: ");
        printf("%d ", voltage);
        printf("\n");
        
        printf("Motor Velocity: ");
        for (int i=0; i<4; i++)
            printf("%d ", motorVelocity[i]);
        printf("\n");
        
        printf("Motor Housing Temp: ");
        for (int i=0; i<5; i++)
            printf("%d ", motorHousingTemp[i]);
        printf("\n");
        
        printf("Motor Current: ");
        for (int i=0; i<5; i++)
            printf("%d ", motorCurrent[i]);
        printf("\n");
        
        printf("Tactile Array:\n");
        printf("  Palm: ");
        printf("%d ", palmTactile);
        printf("\n");
        for (int i=0; i<3; i++)
        {
            printf("  F%d:\n", i+1); 
            printf("    Prox: ");
            for (int j=0; j<12; j++)
                printf("%d ", fingerTactile.proximal[i]);
            printf("\n");
            printf("    Dist: ");
            for (int j=0; j<10; j++)
                printf("%d ", fingerTactile.distal[i]);
            printf("\n");
        }

        printf("Finger Spread: ");
        printf("%d ", fingerSpread);
        printf("\n");
        
        printf("Proximal Joint Angle: ");
        for (int i=0; i<3; i++)
            printf("%d ", proximalJointAngle[i]);
        printf("\n");

        printf("Distal Joint Angle:\n");
        for (int i=0; i<3; i++)
        {
            printf("  F%d:\n", i+1); 
            printf("    Prox: ");
            printf("%d ", distalJointAngle.proximal[i]);
            printf("\n");
            printf("    Dist: ");
            printf("%d ", distalJointAngle.distal[i]);
            printf("\n");
        }
        
        printf("Cable Tension:\n");
        for (int i=0; i<2; i++)
        {
            printf("  F%d: ", i+1); 
            printf("%d ", cableTension[i]);
            printf("%d ", cableTension[i]);
            printf("\n");
        }
        
        printf("PVDF:\n");
        printf("  Palm: ");
        printf("%d ", palmPVDF);
        printf("\n");
        for (int i=0; i<3; i++)
        {
            printf("  F%d:\n", i+1); 
            printf("    Prox: ");
            printf("%d ", fingerPVDF.proximal[i]);
            printf("\n");
            printf("    Dist: ");
            printf("%d ", fingerPVDF.distal[i]);
            printf("\n");
        }

        printf("Finger Acceleration:\n");
        for (int i=0; i<3; i++)
        {
            printf("  F%d: ", i+1); 
            printf("%d ", fingerAcceleration[i]);
            printf("\n");
        }
    };
};

/// Class containing all hand sensor data.
// size: 796
class HandSensors
{
 public:
    short motorHallEncoder[4]; // really only a short, but signed int16_t seems to break on the OMAP
    float motorWindingTemp[4];
    float airTemp;
    HandVoltage voltage;
    int motorVelocity[4];
    float motorHousingTemp[5];
    float motorCurrent[5];
    FingerTactile fingerTactile[3];
    short palmTactile[48];
    int fingerSpread;
    int proximalJointAngle[3];
    FingerDistalAngle distalJointAngle[3];
    FingerCableTension cableTension[2];
    FingerPVDF fingerPVDF[3];
    float palmPVDF[4];
    FingerAcceleration fingerAcceleration[3];
    
    /// Constructor.  Set all data to 0.
    HandSensors()
    {
        reset();
    }
    
    /// set all data to 0
    void reset()
    {
        for (int i=0; i<2; i++)
        {
            cableTension[i].sensor1 = 0;
            cableTension[i].sensor2 = 0;
        }
        
        for (int i=0; i<3; i++)
        {
            for (int j=0; j<10; j++)
                fingerTactile[i].distal[j] = 0;
            for (int j=0; j<12; j++)
                fingerTactile[i].proximal[j] = 0;
            
            proximalJointAngle[i] = 0;
            
            for (int j=0; j<4; j++)
            {
                distalJointAngle[i].distal[j] = 0;
                distalJointAngle[i].proximal[j] = 0;
            }
            
            for (int j=0; j<3; j++)
            {
                fingerPVDF[i].distal[j] = 0;
                fingerPVDF[i].proximal[j] = 0;
            }
            
            fingerAcceleration[i].x = 0;
            fingerAcceleration[i].y = 0;
            fingerAcceleration[i].z = 0;
        }
        
        for (int i=0; i<4; i++)
        {
            this->motorHallEncoder[i] = 0;
            motorWindingTemp[i] = 0;
            motorVelocity[i] = 0;
            palmPVDF[i] = 0;
        }
        
        for (int i=0; i<5; i++)
        {
            motorHousingTemp[i] = 0;
            motorCurrent[i] = 0;
        }
        
        for (int i=0; i<48; i++)
            palmTactile[i] = 0;
        
        airTemp = 0;
        voltage.volts33 = 0;
        voltage.volts12 = 0;
        voltage.volts48 = 0;
        fingerSpread = 0;
    };
    
    /// Merge in only valid data from other source
    void update(const HandSensors& other, const HandSensorsValid& valid)
    {
        for (int i=0; i<2; i++)
        {
            if (valid.cableTension[i])
            {
                cableTension[i].sensor1 = other.cableTension[i].sensor1;
                cableTension[i].sensor2 = other.cableTension[i].sensor2;
            }
        }
        
        for (int i=0; i<3; i++)
        {
            if (valid.fingerTactile.distal[i])
                for (int j=0; j<10; j++)
                    fingerTactile[i].distal[j] = other.fingerTactile[i].distal[j];

            if (valid.fingerTactile.proximal[i])
                for (int j=0; j<12; j++)
                    fingerTactile[i].proximal[j] = other.fingerTactile[i].proximal[j];
            
            if (valid.proximalJointAngle[i])
                proximalJointAngle[i] = other.proximalJointAngle[i];
            
            if (valid.distalJointAngle.distal[i])
                for (int j=0; j<4; j++)                    
                    distalJointAngle[i].distal[j] = other.distalJointAngle[i].distal[j];

            if (valid.distalJointAngle.proximal[i])
                for (int j=0; j<4; j++) 
                    distalJointAngle[i].proximal[j] = other.distalJointAngle[i].proximal[j];

            if (valid.fingerPVDF.distal[i])
                for (int j=0; j<3; j++)
                    fingerPVDF[i].distal[j] = other.fingerPVDF[i].distal[j];

            if (valid.fingerPVDF.proximal[i])
                for (int j=0; j<3; j++)
                    fingerPVDF[i].proximal[j] = other.fingerPVDF[i].proximal[j];
            
            if (valid.fingerAcceleration[i])
            {
                fingerAcceleration[i].x = other.fingerAcceleration[i].x;
                fingerAcceleration[i].y = other.fingerAcceleration[i].y;
                fingerAcceleration[i].z = other.fingerAcceleration[i].z;
            }
        }
        
        for (int i=0; i<4; i++)
        {
            if (valid.motorHallEncoder[i])
                this->motorHallEncoder[i] = other.motorHallEncoder[i];
            
            if (valid.motorWindingTemp[i])
                motorWindingTemp[i] = other.motorWindingTemp[i];
            
            if (valid.motorVelocity[i])
                motorVelocity[i] = other.motorVelocity[i];
        }
        
        if (valid.palmPVDF)
            for (int i=0; i<4; i++)
                palmPVDF[i] = other.palmPVDF[i];
        
        for (int i=0; i<5; i++)
        {
            if (valid.motorHousingTemp[i])
                motorHousingTemp[i] = other.motorHousingTemp[i];
            
            if (valid.motorCurrent[i])
                motorCurrent[i] = other.motorCurrent[i];
        }
        
        if (valid.palmTactile)
            for (int i=0; i<48; i++)
                palmTactile[i] = other.palmTactile[i];

        if (valid.airTemp)
            airTemp = other.airTemp;
        
        if (valid.voltage)
        {
            voltage.volts33 = other.voltage.volts33;
            voltage.volts12 = other.voltage.volts12;
            voltage.volts48 = other.voltage.volts48;
        }
        
        if (valid.fingerSpread)
            fingerSpread = other.fingerSpread;
        
    };
    
    /// Print only valid data
    // non valid data will be printed as blank lines
    void print(const HandSensorsValid& valid) const
    {
        printf("Motor Hall: ");
        for (int i=0; i<4; i++)
            pval_d(motorHallEncoder[i], valid.motorHallEncoder[i]);
        printf("\n");
        
        printf("Motor Winding Temp: ");
        for (int i=0; i<4; i++)
            pval_f(motorWindingTemp[i], valid.motorWindingTemp[i]);
        printf("\n");
        
        printf("Air Temp: ");
        pval_f(airTemp, valid.airTemp);
        printf("\n");
        
        printf("Voltage: ");
        pval_f(voltage.volts33, valid.voltage);
        pval_f(voltage.volts12, valid.voltage);
        pval_f(voltage.volts48, valid.voltage);
        printf("\n");
        
        printf("Motor Velocity: ");
        for (int i=0; i<4; i++)
            pval_d(motorVelocity[i], valid.motorVelocity[i]);
        printf("\n");
        
        printf("Motor Housing Temp: ");
        for (int i=0; i<5; i++)
            pval_d(motorHousingTemp[i], valid.motorHousingTemp[i]);
        printf("\n");
        
        printf("Motor Current: ");
        for (int i=0; i<5; i++)
            pval_f(motorCurrent[i], valid.motorCurrent[i]);
        printf("\n");
        
        printf("Tactile Array:\n");
        printf("  Palm: ");
        for (int j=0; j<48; j++)
            pval_d(palmTactile[j], valid.palmTactile);
        printf("\n");
        for (int i=0; i<3; i++)
        {
            printf("  F%d:\n", i+1); 
            printf("    Prox: ");
            for (int j=0; j<12; j++)
                pval_d(fingerTactile[i].proximal[j], valid.fingerTactile.proximal[i]);
            printf("\n");
            printf("    Dist: ");
            for (int j=0; j<10; j++)
                pval_d(fingerTactile[i].distal[j], valid.fingerTactile.distal[i]);
            printf("\n");
        }

        printf("Finger Spread: ");
        pval_d(fingerSpread, valid.fingerSpread);
        printf("\n");
        
        printf("Proximal Joint Angle: ");
        for (int i=0; i<3; i++)
            pval_d(proximalJointAngle[i], valid.proximalJointAngle[i]);
        printf("\n");

        printf("Distal Joint Angle:\n");
        for (int i=0; i<3; i++)
        {
            printf("  F%d:\n", i+1); 
            printf("    Prox: ");
            for (int j=0; j<4; j++)
                pval_f(distalJointAngle[i].proximal[j], valid.distalJointAngle.proximal[i]);
            printf("\n");
            printf("    Dist: ");
            for (int j=0; j<4; j++)
                pval_f(distalJointAngle[i].distal[j], valid.distalJointAngle.distal[i]);
            printf("\n");
        }
        
        printf("Cable Tension:\n");
        for (int i=0; i<2; i++)
        {
            printf("  F%d: ", i+1); 
            pval_f(cableTension[i].sensor1, valid.cableTension[i]);
            pval_f(cableTension[i].sensor2, valid.cableTension[i]);
            printf("\n");
        }
        
        printf("PVDF:\n");
        printf("  Palm: ");
        for (int j=0; j<4; j++)
            pval_f(palmPVDF[j], valid.palmPVDF);
        printf("\n");
        for (int i=0; i<3; i++)
        {
            printf("  F%d:\n", i+1); 
            printf("    Prox: ");
            for (int j=0; j<3; j++)
                pval_f(fingerPVDF[i].proximal[j], valid.fingerPVDF.proximal[i]);
            printf("\n");
            printf("    Dist: ");
            for (int j=0; j<3; j++)
                pval_f(fingerPVDF[i].distal[j], valid.fingerPVDF.distal[i]);
            printf("\n");
        }

        printf("Finger Acceleration:\n");
        for (int i=0; i<3; i++)
        {
            printf("  F%d: ", i+1); 
            pval_d(fingerAcceleration[i].x, valid.fingerAcceleration[i]);
            pval_d(fingerAcceleration[i].y, valid.fingerAcceleration[i]);
            pval_d(fingerAcceleration[i].z, valid.fingerAcceleration[i]);
            printf("\n");
        }
    };

    /// Print only valid data
    // non-valid data will not be printed
    void printonly(const HandSensorsValid* const valid) const
    {
        printonly(*valid);
    };

    /// Print only valid data
    // non-valid data will not be printed
    void printonly(const HandSensorsValid& valid) const
    {
        bool anyvalid = false;
        
        // motor hall
        for (int i=0; i<4; i++)
        {
            if (valid.motorHallEncoder[i])
            {
                anyvalid = true;
                break;
            }
        }
        if (anyvalid)
        {
            printf("Motor Hall: ");
            for (int i=0; i<4; i++)
                pval_d(motorHallEncoder[i], valid.motorHallEncoder[i]);
            printf("\n");
        }

        // motor winding temp
        anyvalid = false;
        for (int i=0; i<4; i++)
        {
            if (valid.motorWindingTemp[i])
            {
                anyvalid = true;
                break;
            }
        }
        if (anyvalid)
        {
            printf("Motor Winding Temp: ");
            for (int i=0; i<4; i++)
                pval_f(motorWindingTemp[i], valid.motorWindingTemp[i]);
            printf("\n");
        }

        if (valid.airTemp)
        {
            printf("Air Temp: ");
            pval_f(airTemp, valid.airTemp);
            printf("\n");
        }
        
        //todo        
        // printf("Voltage: ");
        // pval_f(voltage.volts33, valid.voltage);
        // pval_f(voltage.volts12, valid.voltage);
        // pval_f(voltage.volts48, valid.voltage);
        // printf("\n");
        
        // motor velocity
        anyvalid = false;
        for (int i=0; i<4; i++)
        {
            if (valid.motorVelocity[i])
            {
                anyvalid = true;
                break;
            }
        }
        if (anyvalid)
        {
            printf("Motor Velocity: ");
            for (int i=0; i<4; i++)
                pval_d(motorVelocity[i], valid.motorVelocity[i]);
            printf("\n");
        }
        
        // motor housing temp
        anyvalid = false;
        for (int i=0; i<5; i++)
        {
            if (valid.motorHousingTemp[i])
            {
                anyvalid = true;
                break;
            }
        }
        if (anyvalid)
        {
            printf("Motor Housing Temp: ");
            for (int i=0; i<5; i++)
                pval_d(motorHousingTemp[i], valid.motorHousingTemp[i]);
            printf("\n");
        }

        // motor current
        anyvalid = false;
        for (int i=0; i<5; i++)
        {
            if (valid.motorCurrent[i])
            {
                anyvalid = true;
                break;
            }
        }
        if (anyvalid)
        {
            printf("Motor Current: ");
            for (int i=0; i<5; i++)
                pval_f(motorCurrent[i], valid.motorCurrent[i]);
            printf("\n");
        }

        if (valid.palmTactile)
        {
            printf("Palm Tactile: ");
            for (int j=0; j<48; j++)
                pval_d(palmTactile[j], valid.palmTactile);
            printf("\n");
        }
        for (int i=0; i<3; i++)
        {
            if (valid.fingerTactile.proximal[i])
            {
                printf("F%d Proximal Tactile: ", i+1);
                for (int j=0; j<12; j++)
                    pval_d(fingerTactile[i].proximal[j], valid.fingerTactile.proximal[i]);
                printf("\n"); 
            }
        }
        for (int i=0; i<3; i++)
        {
            if (valid.fingerTactile.distal[i])
            {
                printf("F%d Distal Tactile: ", i+1);
                for (int j=0; j<10; j++)
                    pval_d(fingerTactile[i].distal[j], valid.fingerTactile.distal[i]);
                printf("\n"); 
            }
        }

        // printf("Finger Spread: ");
        // pval_d(fingerSpread, valid.fingerSpread);
        // printf("\n");
        
        // printf("Proximal Joint Angle: ");
        // for (int i=0; i<3; i++)
        //     pval_d(proximalJointAngle[i], valid.proximalJointAngle[i]);
        // printf("\n");

        // printf("Distal Joint Angle:\n");
        // for (int i=0; i<3; i++)
        // {
        //     printf("  F%d:\n", i+1); 
        //     printf("    Prox: ");
        //     for (int j=0; j<4; j++)
        //         pval_f(distalJointAngle[i].proximal[j], valid.distalJointAngle.proximal[i]);
        //     printf("\n");
        //     printf("    Dist: ");
        //     for (int j=0; j<4; j++)
        //         pval_f(distalJointAngle[i].distal[j], valid.distalJointAngle.distal[i]);
        //     printf("\n");
        // }
        
        // printf("Cable Tension:\n");
        // for (int i=0; i<2; i++)
        // {
        //     printf("  F%d: ", i+1); 
        //     pval_f(cableTension[i].sensor1, valid.cableTension[i]);
        //     pval_f(cableTension[i].sensor2, valid.cableTension[i]);
        //     printf("\n");
        // }
        
        // printf("PVDF:\n");
        // printf("  Palm: ");
        // for (int j=0; j<4; j++)
        //     pval_f(palmPVDF[j], valid.palmPVDF);
        // printf("\n");
        // for (int i=0; i<3; i++)
        // {
        //     printf("  F%d:\n", i+1); 
        //     printf("    Prox: ");
        //     for (int j=0; j<3; j++)
        //         pval_f(fingerPVDF[i].proximal[j], valid.fingerPVDF.proximal[i]);
        //     printf("\n");
        //     printf("    Dist: ");
        //     for (int j=0; j<3; j++)
        //         pval_f(fingerPVDF[i].distal[j], valid.fingerPVDF.distal[i]);
        //     printf("\n");
        // }
        
        for (int i=0; i<3; i++)
        {
            if (valid.fingerAcceleration[i])
            {
                printf("F%d Acceleration: ", i+1); 
                pval_d(fingerAcceleration[i].x, valid.fingerAcceleration[i]);
                pval_d(fingerAcceleration[i].y, valid.fingerAcceleration[i]);
                pval_d(fingerAcceleration[i].z, valid.fingerAcceleration[i]);
                printf("\n");
            }
        }
    };

    /// Print all data
    void print() const
    {
        this->print(HandSensorsValid(true));
    };

    /// pack data into this byte buffer for transfer over the wire
    //
    // with no cableTension, voltages, or PVDF, packed length = 442 bytes
    //
    // @return the number of bytes packed
    int pack(unsigned char* buffer) const
    {
        unsigned char* ptr = buffer;
        
        // for (int i=0; i<2; i++)
        // {
        //     ptr += pack(ptr, cableTension[i].sensor1);
        //     ptr += pack(ptr, cableTension[i].sensor2);
        // }
        
        for (int i=0; i<3; i++)
        {
            for (int j=0; j<10; j++)
                ptr += ::pack(ptr, fingerTactile[i].distal[j]);
            for (int j=0; j<12; j++)
                ptr += ::pack(ptr, fingerTactile[i].proximal[j]);
            
            ptr += ::pack(ptr, proximalJointAngle[i]);
            
            for (int j=0; j<4; j++)
            {
                ptr += ::pack(ptr, distalJointAngle[i].distal[j]);
                ptr += ::pack(ptr, distalJointAngle[i].proximal[j]);
            }
            
            // for (int j=0; j<3; j++)
            // {
            //     ptr += ::pack(ptr, fingerPVDF[i].distal[j]);
            //     ptr += ::pack(ptr, fingerPVDF[i].proximal[j]);
            // }
            
            ptr += ::pack(ptr, fingerAcceleration[i].x);
            ptr += ::pack(ptr, fingerAcceleration[i].y);
            ptr += ::pack(ptr, fingerAcceleration[i].z);
        }
        
        for (int i=0; i<4; i++)
        {
            ptr += ::pack(ptr, motorHallEncoder[i]);
            ptr += ::pack(ptr, motorWindingTemp[i]);
            ptr += ::pack(ptr, motorVelocity[i]);
            // ptr += ::pack(ptr, palmPVDF[i]);
        }
        
        for (int i=0; i<5; i++)
        {
            ptr += ::pack(ptr, motorHousingTemp[i]);
            ptr += ::pack(ptr, motorCurrent[i]);
        }
        
        for (int i=0; i<48; i++)
            ptr += ::pack(ptr, palmTactile[i]);
        
        ptr += ::pack(ptr, airTemp);
        // ptr += ::pack(ptr, voltage.volts33);
        // ptr += ::pack(ptr, voltage.volts12);
        // ptr += ::pack(ptr, voltage.volts48);
        ptr += ::pack(ptr, fingerSpread);

        return (ptr-buffer);
    };

    /// unpack data from byte buffer
    //
    // with no cableTension, voltages, or PVDF, packed length = 442 bytes
    //
    // @return the number of bytes unpacked
    int unpack(const unsigned char* const buffer)
    {
        unsigned char* ptr = (unsigned char*)buffer;
        
        // for (int i=0; i<2; i++)
        // {
        //     ptr += ::unpack(ptr, &cableTension[i].sensor1);
        //     ptr += ::unpack(ptr, &cableTension[i].sensor2);
        // }
        
        for (int i=0; i<3; i++)
        {
            for (int j=0; j<10; j++)
                ptr += ::unpack(ptr, &fingerTactile[i].distal[j]);
            for (int j=0; j<12; j++)
                ptr += ::unpack(ptr, &fingerTactile[i].proximal[j]);
            
            ptr += ::unpack(ptr, &proximalJointAngle[i]);
            
            for (int j=0; j<4; j++)
            {
                ptr += ::unpack(ptr, &distalJointAngle[i].distal[j]);
                ptr += ::unpack(ptr, &distalJointAngle[i].proximal[j]);
            }
            
            // for (int j=0; j<3; j++)
            // {
            //     ptr += ::unpack(ptr, &fingerPVDF[i].distal[j]);
            //     ptr += ::unpack(ptr, &fingerPVDF[i].proximal[j]);
            // }
            
            ptr += ::unpack(ptr, &fingerAcceleration[i].x);
            ptr += ::unpack(ptr, &fingerAcceleration[i].y);
            ptr += ::unpack(ptr, &fingerAcceleration[i].z);
        }
        
        for (int i=0; i<4; i++)
        {
            ptr += ::unpack(ptr, &motorHallEncoder[i]);
            ptr += ::unpack(ptr, &motorWindingTemp[i]);
            ptr += ::unpack(ptr, &motorVelocity[i]);
            // ptr += ::unpack(ptr, &palmPVDF[i]);
        }
        
        for (int i=0; i<5; i++)
        {
            ptr += ::unpack(ptr, &motorHousingTemp[i]);
            ptr += ::unpack(ptr, &motorCurrent[i]);
        }
        
        for (int i=0; i<48; i++)
            ptr += ::unpack(ptr, &palmTactile[i]);
        
        ptr += ::unpack(ptr, &airTemp);
        // ptr += ::unpack(ptr, &voltage.volts33);
        // ptr += ::unpack(ptr, &voltage.volts12);
        // ptr += ::unpack(ptr, &voltage.volts48);
        ptr += ::unpack(ptr, &fingerSpread);

        return (ptr-buffer);
    };

 protected:
    
    /// Helper function for print() routines
    // For decimal values.
    void pval_d(int val, bool valid) const
    {
        if (valid)
            printf("%4d ", val);
        else
            printf("____ ");
    };
    
    /// Helper function for print() routines.
    // For float values.
    void pval_f(float val, bool valid) const
    {
        if (valid)
            printf("%4.2f ", val);
        else
            printf("____ ");
    };
};

class HandPacket
{
public:
    HandSensors data;
    timeval stamp;
    
    HandPacket()
    { };
    
    HandPacket(HandSensors data)
    {
        this->data = data;
        gettimeofday(&stamp, NULL);
    };
    
    int pack(unsigned char* buffer) const
    {
        unsigned char* ptr = buffer;
        ptr += ::pack(ptr, (int)stamp.tv_sec);
        ptr += ::pack(ptr, (int)stamp.tv_usec);
        ptr += data.pack(ptr);
        return (ptr-buffer);
    };
    
    int unpack(const unsigned char* const buffer)
    {
        unsigned char* ptr = (unsigned char*)buffer;
        ptr += ::unpack(ptr, (int*)(&(stamp.tv_sec)));
        ptr += ::unpack(ptr, (int*)(&(stamp.tv_usec)));
        ptr += data.unpack(ptr);
        return (ptr-buffer);
    };
};

// convert motor board indecies to final finger numbers
const uint8_t motorChain_to_fingerNumber[5] = {2, 1, 3, 0, 4};

// convert finger number to motor board number
const uint8_t fingerNumber_to_motorChain[5] = {3, 1, 0, 2, 4};

// convert the hardware finger chain number to actual finger number
const uint8_t fingerChain_to_fingerNumber[3] = {1, 0, 2};

#endif

