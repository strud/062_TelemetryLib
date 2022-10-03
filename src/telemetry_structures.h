// Telemetry related data structures
// Common definitions used amongst multiple projects

#define telemetry_structures

typedef struct trackingMetricsType{
       float bearing;       // deg
       float range;         // m
       float elevation;     // deg
       float bearingOffset; // deg
       float TrackCommand;         // deg
};

typedef struct  motionMetricsType {
       long VelX, VelY, VelZ;
       int AccelX, AccelY, AccelZ;
       float Theta, Phi, Zeta;
       int ThetaDot,PhiDot, ZetaDot;
       int MagX, MagY, MagZ;
};

// Remote station parameters derived from gnss
typedef struct gnssMetricsType {
       unsigned int lockStatus;
       long Lat;           // 1e-7 deg
       long Lon;           // 1e-7 deg
       float fLat;         // deg
       float fLon;         // deg
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

typedef struct PacketDetailsType {
    unsigned char type;
    unsigned int sourceID;
    uint16_t    targetID;
    unsigned char length;
    unsigned char rssi;
    unsigned int checksum;
    unsigned char validPacket;
    unsigned char unhandledPacketType;
    unsigned long rxTime;
    unsigned char payload[256];
};

// System structure - contains information relating to identity, product type etc
typedef struct  systemType {
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
};