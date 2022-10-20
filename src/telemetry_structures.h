// Telemetry related data structures
// Common definitions used amongst multiple projects

#include "radio_structs.h"
#define telemetry_structures

struct  motionMetricsType {
       long VelX, VelY, VelZ;
       int AccelX, AccelY, AccelZ;
       float Theta, Phi, Zeta;
       int ThetaDot,PhiDot, ZetaDot;
       int MagX, MagY, MagZ;
};

// Remote station parameters derived from gnss
struct gnssMetricsType {
       unsigned int lockStatus;
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
       unsigned long hAcc; // units mm horizontal accuracy estimate
       unsigned long vAcc; // units mm vertical accuracy estimate
       long CurrentTime, PreviousTime; // units s since epoch
       unsigned int satellites;  // number of satellites
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