#define telemetry

#include "telemetry_structures.h"
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

// defines used for calcs
#define WALKING_PACE_THRESHOLD  0.75    // m/s



#ifndef USE_INTERNAL_PACKET_HANDLERS

void BuildRadioPacket(PacketDetailsType *Packet, telemetryConfigurationType *System);
int ParseBinaryPacket(unsigned char *localbuf, PacketDetailsType *Packet, telemetryConfigurationType *System);

#endif

