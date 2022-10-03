// Module configuration structure

#define generic_system_config

#define DEVICE_TYPE_FLIGHT_COMPUTER     1
#define DEVICE_TYPE_BASE_STATION        2
#define DEVICE_TYPE_TRACKER             3
#define DEVICE_TYPE_SLAVE_NODE          4
#define DEVICE_TYPE_STATIONARY_SENSOR   5
#define DEVICE_TYPE_FARTRAK_MINI        6
#define DEVICE_TYPE_FARTRAK             7
#define DEVICE_TYPE_ROCTRAK_DIGITAL     8
#define DEVICE_TYPE_ROCTRAK_GROUNDSTATION     9
#define DEVICE_TYPE_ROCTRAK_APRS        10  
#define DEVICE_TYPE_UNCONFIGURED        255

// #defines for allowable modes of operation
#define MODE_RX_ONLY                     0
#define MODE_SIMPLE_MODEM                1
#define MODE_GPS_TRANSPONDER             2
#define MODE_TEST_TRANSPONDER            3
#define MODE_BASE_STATION_FIXED          4
#define MODE_BASE_STATION_MOBILE         5

typedef struct telemetryConfigurationType {
       unsigned int deviceID;                      // unique 16bit ID
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