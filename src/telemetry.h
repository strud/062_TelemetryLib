#define telemetry

#include "radio_structs.h"

//------------------------------------------------------------------------------
// Radio Packet types (constants)
//------------------------------------------------------------------------------

#define PAC_TYPE_DUMMY         1
#define PAC_TYPE_GPS_FLIGHT    2
#define PAC_TYPE_GPS_PEDESTRIAN 3
#define PAC_TYPE_GPS_BASE_STATION  4
#define PAC_TYPE_GROUND_MOBILE     5
#define PAC_TYPE_ASCII             6
#define PAC_TYPE_IMU            10
#define PAC_TYPE_GSE_FULL       11
#define PAC_TYPE_GSE_BRIEF      12
#define PAC_TYPE_GSE_PENDANT_CMD  13
#define PAC_TYPE_SYSTEM_QUERY   14
#define PAC_TYPE_ROCKET         15


#define PAC_TYPE_SYSTEM_STATUS       20
#define PAC_TYPE_THROUGH_TRAFFIC     30
#define PAC_TYPE_COMMAND             50
#define PAC_TYPE_LEGACY1        100

//------------------------------------------------------------------------------
// Packet lengths by type
//------------------------------------------------------------------------------
#define PAC_LENGTH_GROUND_MOBILE  56

#define PAC_LENGTH_DUMMY             1
#define PAC_LENGTH_GPS_FLIGHT        2
#define PAC_LENGTH_GPS_PEDESTRIAN    3
#define PAC_LENGTH_GPS_BASE_STATION  4
#define PAC_LENGTH_GROUND_MOBILE     56
#define PAC_LENGTH_ASCII             6
#define PAC_LENGTH_IMU               10
#define PAC_LENGTH_GSE_FULL          11
#define PAC_LENGTH_GSE_BRIEF         12
#define PAC_LENGTH_GSE_PENDANT_CMD   13

#define PAC_LENGTH_SYSTEM_STATUS     20
#define PAC_LENGTH_THROUGH_TRAFFIC   30
#define PAC_LENGTH_COMMAND           50
#define PAC_LENGTH_LEGACY1           100

// Commands
#define PAC_CMD_RESTART               1
#define PAC_CMD_RESET_STATE_MACHINE   2
#define PAC_CMD_INITIALISE            3
#define PAC_CMD_SET_OFFSET            4
#define PAC_CMD_SET_SCALE             5
#define PAC_CMD_RADIO_POWER_INC_POS   10
#define PAC_CMD_RADIO_POWER_INC_NEG   11
#define PAC_CMD_RADIO_CHANNEL_INC_POS 20
#define PAC_CMD_RADIO_CHANNEL_INC_NEG 21

#define GNSS_CHIPSET_SAM_M8            1
#define GNSS_CHIPSET_SAM_M10           2
#define GNSS_CHIPSET_L80               3
#define GNSS_CHIPSET_NEO_M6            4    
#define GNSS_CHIPSET_NEO_M7            4    
#define GNSS_CHIPSET_NEO_M8            4    
#define GNSS_CHIPSET_NEO_M9            4    

// defines used for calcs
#define WALKING_PACE_THRESHOLD  0.75    // m/s

// Telemetry related data structures
// Common definitions used amongst multiple projects

#define telemetry_structures

struct  motionMetricsType {
       float VelX, VelY, VelZ;
       float AccelX, AccelY, AccelZ;
       float Theta, Phi, Zeta;
       float ThetaDot,PhiDot, ZetaDot;
       float MagX, MagY, MagZ;
       float baroAltitude;
       float baroPressure;
       float baroTemperature;
};

// Remote station parameters derived from gnss
struct gnssMetricsType {
       unsigned char chipset; // #define indicating GPS chipset used
       unsigned char lockStatus;
       long Lat;           // 1e-7 deg
       long Lon;           // 1e-7 deg
       float fLat;         // deg
       float fLon;         // deg
       float fAlt;         // altitude above sea level m
       float previousfLat; // deg
       float previousfLon; // deg
       long Alt;           // mm
       long Heading;       // units 1e-5 deg
       float fHeading;     // deg
       long VelNorth;      // units cm/s
       long VelEast;       // units cm/s
       long VelDown;       // units cm/s
       long velocity;      // units cm/s magnitude of velocity
       long groundSpeed;   // units mm/s  
       unsigned long hAcc; // units 0.1mm horizontal accuracy estimate
       unsigned long vAcc; // units 0.1mm vertical accuracy estimate
       long CurrentTime, PreviousTime; // units s since epoch
       unsigned char satellites;  // number of satellites
       long hMSL;              // altitude or height above mean sea level 
       unsigned char newdata;  // boolean used to indicate data has been updated
}; 

struct packetDetailsType {
    unsigned char type;
    unsigned int sourceID;
    unsigned int targetID;
    unsigned char length;
    unsigned char rssi;
    float frssi;
    float snr;
    unsigned int checksum;
    unsigned char validPacket;
    unsigned char unhandledPacketType;
    unsigned long rxTime;
    unsigned char payload[256];
    unsigned char newData;
};

struct flightComputerStatusType {
    unsigned int state;
    unsigned char continuity;
    unsigned char outputs;
    char          txtStatus[10];
};
struct dvrStatusType {
    unsigned int cardSize;    // card size MB
    unsigned int remainingMem; // remaining memory MB
    unsigned int state;        // state of camera     
    unsigned char dvrType;     // type of dvr 
};

struct telemetryConfigurationType {
       unsigned int localDeviceID;                 // unique 16bit ID
       unsigned int remoteDeviceID;                // unique 16bit ID
       unsigned char DeviceType;                   // 1: Flight computer
                                                   // 2. Base station
                                                   // 3: tracker
                                                   // 4: slave node
                                                   // 255: unconfigured
       unsigned char OBI2CEEPromAvailable;
       unsigned char OBAccelPolarity;
       unsigned char OBAccelRange;
       unsigned char AccelAvailable;
       unsigned char MagAvailable;
       unsigned char GyroAvailable;
       unsigned char gnssAvailable;
       unsigned char radioAvailable;
       unsigned char allowRemoteCommands;
       unsigned int  TransmitPeriod;                // time between transmissions - units 100ms, 2 bytes

       unsigned char Mode;                         // 0 - Rx only
                                                   // 1 - Simple Modem ie radio <-> UART bridge
                                                   // 2 - GPS transponder
                                                   // 3 - test beacon
                                                   // 4 - base station
                                                   // 5 - ground station mobile
};
// System structure - contains information relating to identity, product type etc
struct  systemType {
       char Name[20];
       unsigned int identity;
       unsigned char uniqueID64[65];
       unsigned char uniqueID8[9];
       unsigned char productType;
       unsigned char fwVersion;
       unsigned char hwVersion;
       int      state;
       unsigned int  Vbus;
       gnssMetricsType gnssMetrics;
       motionMetricsType motioniMetrics;
       telemetryConfigurationType configuration;
       radioModemType radioModemSettings;
       flightComputerStatusType flightComputerStatus;
       dvrStatusType dvrStatus;
};

struct trackingMetricsType {
       float bearing;       // deg
       float magHeading;    // deg
       float gnssHeading;   // deg
       float newGnssHeading;   // deg
       unsigned long consecSampleCountAboveThresh; // number of gnss consecutive samples where ground speed > 2km/h
       char gnssHeadingValid;
       float localGroundSpeed;  // m/s
       float range;         // m
       float elevation;     // deg
       float bearingOffset; // deg
       float TrackCommand;         // deg
       float timeDelta;     // 
       float bearingRate;   // deg/s
       float elevationRate; // deg/s
       float rangeRate;     // m/s
};

unsigned int BuildRadioPacket(packetDetailsType *Packet, systemType *System, short int target);
int ParseBinaryPacket(unsigned char *localbuf, packetDetailsType *Packet, telemetryConfigurationType *System);


