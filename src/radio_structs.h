// packet related data structures
// Common definitions used across multiple projects

// rev 0.1  9-7-2016 CDS
// Copy existing structures from Rider tracker base station
#ifndef gnss_structures
   #include "gnss_structures.h"
#endif

#define radio_structs

#define RADIO_BAND1_LOWER_F    413
#define RADIO_BAND1_DEFAULT_F    433
#define RADIO_BAND1_UPPER_F    453
#define RADIO_BAND2_LOWER_F    450
#define RADIO_BAND2_DEFAULT_F    470
#define RADIO_BAND2_UPPER_F    490
#define RADIO_BAND3_LOWER_F    848
#define RADIO_BAND3_DEFAULT_F    868
#define RADIO_BAND3_UPPER_F    888
#define RADIO_BAND4_LOWER_F    901
#define RADIO_BAND4_DEFAULT_F    915
#define RADIO_BAND4_UPPER_F    929


typedef struct radioModemType {
    unsigned char TransmitEnable;               // 1 : Enable, any other value - disabled
    unsigned char RadioEnable;
    unsigned char DeviceModel;
    unsigned char DeviceVersion;
    unsigned char Channel;
    float    BaseFreq;
    unsigned char Band;
    int RxSensitivity;  // dBm
    char antGain;   // dB
    char txPower;   // dBm
};
    

typedef struct  LinkStatsType {
        float local_rssi_average;
        float remote_rssi_average;
        float link_margin_average;
        unsigned long totalTxPackets;
        unsigned long totalRxPackets;
        unsigned long oldTotalRxPackets;
        unsigned long oltTotalTxPackets;
        unsigned long totalLostPackets;
        unsigned long badPackets;
        float badPacketsPercent;
        float lostPacketsPercent;
        float responseTime;
        float responseTimeAverage;
        unsigned char responseTimeouts;
        unsigned char linkOK;
        float packetsPerMinute;

};