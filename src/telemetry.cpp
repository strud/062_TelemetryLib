
//#include "generic_system_config.h"
#include <Arduino.h>
#include "telemetry.h"
#define byte  uint8_t

//------------------------------------------------------------------------------
// BuildRadioPacket
// Parameters 
// pointer to Packet (type PacketDetailsType) :This is the packet
// structure that will have the payload generated 'into'
// pointer to System (type SystemType) - this contains the information from which
// the packet payload is generated from
//------------------------------------------------------------------------------

unsigned int BuildRadioPacket(packetDetailsType *Packet, systemType *System, short int target)
{   unsigned char ctemp;
    unsigned char * p_ctemp;
    long lpactemp, ltemp;
    int i, itemp;
    float ftemp;
    unsigned int utemp, index;

    for (i=0;i<256;i++) Packet->payload[i]=0; // blast old packet away
    //-------------------------------------------------------------------
    // Common bytes for all types
    index=0;
    memcpy(&Packet->payload[index], (unsigned char*)(&System->identity), 2); // Identity of source system
    index+=2;
    memcpy(&Packet->payload[index], (unsigned char*)(&target), 2);   // Identity of target system
    index+=2;
    memcpy(&Packet->payload[index], (unsigned char*)(&Packet->type), 1); // First byte describes the type of packet
    index+=1;
    //-------------------------------------------------------------------

    switch (Packet->type) {

     case PAC_TYPE_COMMAND : {
           // Tell the remote station to do something.....
           // Format : LSB first
           // Description      bytes index     type         units
           // command          2    3,4        int          none
           // argument1        2    5,6        int
           // argument2        2    7,8        int
           // argument3        2    9,10       int
           // argument4        2    11,12      int
           // checksum         1    13         byte
     break;
     }
     case PAC_TYPE_ROCKET: {

     // Description      bytes           type         units    
     // GNSS Chipset      1              byte 
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.chipset), 1);index+=1;
     // Altitude         4            float          m         
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.fAlt), 4);index+=4;
     // Lattitude        4            float         deg 
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.fLat), 4);index+=4;
     // Longitude        4            float         deg 
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.fLon), 4);index+=4;
     // Satellites       1               byte        none
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.satellites), 1);index++;
     // Lock             1               byte        boolean
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.lockStatus), 1);index++;
     // VBus             2              uint         mV
     memcpy(&Packet->payload[index], (unsigned char*)(&System->Vbus), 2);index+=2;
     // GPS speed (magnitude of vector sum)           cm/s
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.velocity), 4);index+=4;
     // GPS down velocity                             cm/s
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.VelDown), 4);index+=4;
     // GPS Heading                                   deg 
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.fHeading), 4);index+=4;
     // Horizontal accuracy estimate      unsigned long, mm
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.hAcc), 4);index+=4;
     // Vertical accuracy estimate        unsigned long, mm
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.vAcc), 4);index+=4;
     // GPS Ground speed                  unsigned in cm/s
     memcpy(&Packet->payload[index], (unsigned char*)(&System->gnssMetrics.groundSpeed), 4);index+=4;
     
     // Motion & attitude metrics
     //Accelerations
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.AccelX), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.AccelY), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.AccelZ), 4);index+=4;
     // Velocities
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.VelX), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.VelY), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.VelZ), 4);index+=4;
     // magnetic field
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.MagX), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.MagY), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.MagZ), 4);index+=4;
     // Angles
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.Phi), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.Theta), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.Zeta), 4);index+=4;
     // Angular rate 
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.PhiDot), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.ThetaDot), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.ZetaDot), 4);index+=4;
     // Barometer
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.baroAltitude), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.baroPressure), 4);index+=4;
     memcpy(&Packet->payload[index], (unsigned char*)(&System->motioniMetrics.baroTemperature), 4);index+=4;

     // Continuity status   1 byte
     memcpy(&Packet->payload[index], (unsigned char*)(&System->flightComputerStatus.continuity), 1);index+=1;
     // Output Status       1 byte  
     memcpy(&Packet->payload[index], (unsigned char*)(&System->flightComputerStatus.outputs), 1);index+=1;
     // State machine value unsigned int
     memcpy(&Packet->payload[index], (unsigned char*)(&System->flightComputerStatus.state), 2);index+=2;
     

     }
    case PAC_TYPE_GROUND_MOBILE: {

           Packet->length = PAC_LENGTH_GROUND_MOBILE;
           // of interest for rider tracker project

           // Format : LSB first
           // Description      bytes index     type         units
           // X accel          2    3,4        int          0.1 m/s/s
           // Y accel          2    5,6        int          0.1 m/s/s
           // Z accel          2    7,8        int          0.1 m/s/s
           // Theta            2    9,10        int          deg
           // Phi              2    11,12      int          deg
           // Zeta             2    13,14      int          deg
           // Thetadot         2    15,16      int          deg/s
           // Phidot           2    17,18      int          deg/s
           // Zetadot          2    19,20      int          deg/s
           // MagX             2    21,22      int          mGauss
           // MagY             2    23,24      int          mGauss
           // MaxZ             2    25,26      int          mGauss
           // Lattitude        4    27,28,29,30 long       deg  1e-7
           // Longitude        4    31,32,33,34 long       deg  1e-7
           // Satellites       1    35          byte        none
           // Lock             1    36          byte        boolean
           // VBus             1    37          uchar       100mV
           // gpsSpeed         4    38,39,40,41 long         cm/s
           // gpsHeading       4    42,43,44,45 long        1e-5 deg
           // gpshAcc          4    46,47,48,49 unsigned long, mm
           // gpsvAcc          4    50,51,52,53 unsigned long, mm
           // state            2    54, 55      unsigned int
           // checksum         1    56

          
     break;
     }


     case PAC_TYPE_LEGACY1 : {     // does not conform to new packet structure template
/*
    // SendCString(DEBUG_PORT, "packetising\r\n");
      // as per the first implementation ie no identifier, no packet type
           // Format : LSB first
     // Description      bytes           type         units
     // Altitude         2    0,1        int          100ft
     ltemp = System->gnss->hMSL/305; // convert to feet
     itemp = (int)ltemp;  // hMSL units mm
     p_ctemp = &itemp;
     pacBuff[0]= p_ctemp[0];
     pacBuff[1]= p_ctemp[1];

     // Lattitude        4    26,27,28,29 float       deg e-7
     //p_ctemp = &gps_lat;
     p_ctemp = System->gnss->Lat;
//     for (i=0;i<4;i++) pacBuff[i+26] = p_ctemp[i];
     pacBuff[26] = p_ctemp[0];
     pacBuff[27] = p_ctemp[1];
     pacBuff[28] = p_ctemp[2];
     pacBuff[29] = p_ctemp[3];

     // Longitude        4    30,31,32,33 float       deg e-7
//     p_ctemp = &gps_long;
     p_ctemp = System->gnss->Lon;
//     for (i=0;i<4;i++) pacBuff[i+30] = p_ctemp[i];
     pacBuff[30] = p_ctemp[0];
     pacBuff[31] = p_ctemp[1];
     pacBuff[32] = p_ctemp[2];
     pacBuff[33] = p_ctemp[3];

     // Satellites       1    34          byte        none
     pacBuff[34]= System->gnss->satellites;
     // Lock             1    35          byte        boolean
     pacBuff[35] = System->gnss->lockStatus;

     // VBus             2    36,37       int         mV
     pacBuff[36]=0;

     // Continuity status
     pacBuff[37]=0;

     // GPS speed (magnitude of vector sum)           m/s
     ltemp = System->gnss->velocity/100;
     itemp = (int)ltemp;
     p_ctemp = &itemp;
     pacBuff[38] = p_ctemp[0];
     pacBuff[39] = p_ctemp[1];

     // GPS down velocity                             m/s
     ltemp = System->gnss->VelDown/100;
     itemp = (int)ltemp;
     p_ctemp = &itemp;
     pacBuff[40] = p_ctemp[0];
     pacBuff[41] = p_ctemp[1];

     // GPS Heading                                   deg e-5
     p_ctemp = &System->gnss->Heading;
     pacBuff[42] = p_ctemp[0];
     pacBuff[43] = p_ctemp[1];
     pacBuff[44] = p_ctemp[2];
     pacBuff[45] = p_ctemp[3];

     // Distance from Pad (m) (over ground) unsigned int, m
//     p_ctemp =  &gnss_distToPad;
//     pacBuff[46] = p_ctemp[0];
//     pacBuff[47] = p_ctemp[1];

     // Bearing to Pad (deg)               unsigned int, deg e-2
//     p_ctemp = &gnss_padBearing;
//     pacBuff[48] = p_ctemp[0];
//     pacBuff[49] = p_ctemp[1];

     // Horizontal accuracy estimate      unsigned long, mm
     p_ctemp = &System->gnss->hAcc;
     pacBuff[50] = p_ctemp[0];
     pacBuff[51] = p_ctemp[1];
     pacBuff[52] = p_ctemp[2];
     pacBuff[53] = p_ctemp[3];

     // Vertical accuracy estimate        unsigned long, mm
     p_ctemp = &System->gnss->vAcc;
     pacBuff[54] = p_ctemp[0];
     pacBuff[55] = p_ctemp[1];
     pacBuff[56] = p_ctemp[2];
     pacBuff[57] = p_ctemp[3];

     // GPS Ground speed                  unsigned in cm/s
//     utemp = (unsigned int)gpsGroundSpeed/100;          // convert to m/s
     utemp = 0;          // convert to m/s
     p_ctemp = &utemp;
     pacBuff[58] = p_ctemp[0];
     pacBuff[59] = p_ctemp[1];

     // State machine value                unsigned int  no units
     p_ctemp = &System->state;
     pacBuff[60] = p_ctemp[0];
     pacBuff[61] = p_ctemp[1];
                                              */
     break;
     }
     case PAC_TYPE_DUMMY : {
     // Construct test packet
     // Format : LSB first
     // Description      bytes           type         units


     break;
     }

     case PAC_TYPE_GPS_FLIGHT : {


     break;
     }
     case PAC_TYPE_GPS_PEDESTRIAN : {



     break;
     }
     case PAC_TYPE_GPS_BASE_STATION : {



     break;
     }
     case PAC_TYPE_IMU : {



     break;
     }



     case PAC_TYPE_SYSTEM_STATUS : {

          

     break;
     }

     case PAC_TYPE_THROUGH_TRAFFIC : {


     break;
     }

     default : {
             // leave packet blank
      break;
     }

     }
     return index;
     
}

int ParseBinaryPacket(unsigned char *localbuf, packetDetailsType *Packet, systemType *System){
 unsigned char * p_byte;
 int i_temp, i, index;
 float f_temp;
 
         // There is a packet ready then extract the payload !
         // Reset current packet vaules
         Packet->validPacket =0;

         // Read in bytes common to all packet types
         for (i=0;i<256;i++) Packet->payload[i]=0; // blast old packet away
          //-------------------------------------------------------------------
          // Common bytes for all types
          index=0;
          memcpy((unsigned char*)(&System->identity), &Packet->payload[index], 2); // Identity of source system
          index+=2;
          memcpy((unsigned char*)(&Packet->sourceID), &Packet->payload[index], 2);   // Identity of source system
          index+=2;
          memcpy((unsigned char*)(&Packet->type), &Packet->payload[index], 1);
          index+=1;
         
         switch(Packet->type){

         case  PAC_TYPE_GSE_FULL : {
              Packet->unhandledPacketType=0;

          break;
         }

         case PAC_TYPE_GSE_BRIEF  : {  //
              Packet->unhandledPacketType=0;
          break;
         }
         
         case PAC_TYPE_GROUND_MOBILE  : {  //
              Packet->unhandledPacketType=0;
              /*
               // Format : LSB first
               // Description      bytes index     type         units
               // X accel          2    3,4        int          0.1 m/s/s
               // Y accel          2    5,6        int          0.1 m/s/s
               // Z accel          2    7,8        int          0.1 m/s/s
               // Theta            2    9,10       int          0.01 deg
               // Phi              2    11,12      int          0.01 deg
               // Zeta             2    13,14      int          0.01 deg
               // Thetadot         2    15,16      int          deg/s
               // Phidot           2    17,18      int          deg/s
               // Zetadot          2    19,20      int          deg/s
               // MagX             2    21,22      int          mGauss
               // MagY             2    23,24      int          mGauss
               // MaxZ             2    25,26      int          mGauss
               // Lattitude        4    27,28,29,30 long       deg  1e-7
               // Longitude        4    31,32,33,34 long       deg  1e-7
               // Satellites       1    35          byte        none
               // Lock             1    36          byte        boolean
               // VBus             1    37          uchar       100mV
               // gpsSpeed         4    38,39,40,41 long         cm/s
               // gpsHeading       4    42,43,44,45 long        1e-5 deg
               // gpshAcc          4    46,47,48,49 unsigned long, mm
               // gpsvAcc          4    50,51,52,53 unsigned long, mm
               // state            2    54, 55      unsigned int
               // checksum         1    56

               p_byte = &System->inertial.AccelX;
               p_byte[0] = Packet->payload[3];
               p_byte[1] = Packet->payload[4];
               p_byte = &System->inertial.AccelY;
               p_byte[0] = Packet->payload[5];
               p_byte[1] = Packet->payload[6];
               p_byte = &System->inertial.AccelZ;
               p_byte[0] = Packet->payload[7];
               p_byte[1] = Packet->payload[8];
               
               p_byte =  i_temp;
               p_byte[0] = Packet->payload[9];
               p_byte[1] = Packet->payload[10];
               System->inertial.Theta = (float)i_temp*100.0; // convert to units of degrees

               p_byte =  i_temp;
               p_byte[0] = Packet->payload[11];
               p_byte[1] = Packet->payload[12];
               System->inertial.Phi = (float)i_temp*100.0; // convert to units of degrees
               
               p_byte =  i_temp;
               p_byte[0] = Packet->payload[13];
               p_byte[1] = Packet->payload[14];
               System->inertial.Zeta = (float)i_temp*100.0;  // convert to units of degrees

               p_byte = &System->inertial.ThetaDot;
               p_byte[0] = Packet->payload[15];
               p_byte[1] = Packet->payload[16];
               
               p_byte = &System->inertial.PhiDot;
               p_byte[0] = Packet->payload[17];
               p_byte[1] = Packet->payload[18];
               
               p_byte = &System->inertial.ZetaDot;
               p_byte[0] = Packet->payload[19];
               p_byte[1] = Packet->payload[20];

               p_byte = &System->inertial.MagX;
               p_byte[0] = Packet->payload[21];
               p_byte[1] = Packet->payload[22];
               p_byte = &System->inertial.MagY;
               p_byte[0] = Packet->payload[23];
               p_byte[1] = Packet->payload[24];
               p_byte = &System->inertial.MagZ;
               p_byte[0] = Packet->payload[25];
               p_byte[1] = Packet->payload[26];

               p_byte = &System->gnss.Lat;
               p_byte[0] = Packet->payload[27];
               p_byte[1] = Packet->payload[28];
               p_byte[2] = Packet->payload[29];
               p_byte[3] = Packet->payload[30];

               p_byte = &System->gnss.Lon;
               p_byte[0] = Packet->payload[31];
               p_byte[1] = Packet->payload[32];
               p_byte[2] = Packet->payload[33];
               p_byte[3] = Packet->payload[34];

               System->gnss.satellites = Packet->payload[35];
               System->gnss.lockStatus = Packet->payload[36];
               System->Vbus = Packet->payload[37];
               
               // gpsSpeed         4    38,39,40,41       int         m/s
               p_byte = &System->gnss.velocity;
               p_byte[0] = Packet->payload[38];
               p_byte[1] = Packet->payload[39];
               p_byte[0] = Packet->payload[40];
               p_byte[1] = Packet->payload[41];
               
               // gpsHeading       4    42,43,44,45 long        1e-5 deg
               p_byte = &System->gnss->Heading;
               p_byte[0]=Packet->payload[42];
               p_byte[1]=Packet->payload[43];
               p_byte[2]=Packet->payload[44];
               p_byte[3]=Packet->payload[45];

               // Horizontal accuracy estimate      unsigned long, mm
               p_byte = &System->gnss->hAcc;
               p_byte[0]=Packet->payload[46];
               p_byte[1]=Packet->payload[47];
               p_byte[2]=Packet->payload[48];
               p_byte[3]=Packet->payload[49];

               // Vertical accuracy estimate        unsigned long, mm
               p_byte = &System->gnss->vAcc;
               p_byte[0]=Packet->payload[50];
               p_byte[1]=Packet->payload[51];
               p_byte[2]=Packet->payload[52];
               p_byte[3]=Packet->payload[53];

               p_byte = &System->state;
               p_byte[0]=Packet->payload[54];
               p_byte[1]=Packet->payload[55];

               Packet->sentChecksum = Packet->payload[56];
               
               // Calculate 1byte checksum for ourselves
               Packet->checksum=0;
               for(i=0;i< Packet->length-1;i++){
                Packet->checksum += Packet->payload[i];
               }
               if (Packet->checksum == Packet->sentChecksum)
                  Packet->validPacket=1;
               else
                  Packet->validPacket=0;
                  
               Packet->newPacket = 1;
               */     
          break;
         }

         case PAC_TYPE_COMMAND  : {
              // command has come in from someone, check who and what the command is

         break;
         }
         case PAC_TYPE_ROCKET: {

               // Description      bytes           type         units    
               // GNSS Chipset      1              byte 
               memcpy((unsigned char*)(&System->gnssMetrics.chipset), &Packet->payload[index], 1);index+=1;
               // Altitude         4            float          m         
               memcpy((unsigned char*)(&System->gnssMetrics.fAlt), &Packet->payload[index], 4);index+=4;
               // Lattitude        4            float         deg 
               memcpy((unsigned char*)(&System->gnssMetrics.fLat), &Packet->payload[index], 4);index+=4;
               // Longitude        4            float         deg 
               memcpy((unsigned char*)(&System->gnssMetrics.fLon), &Packet->payload[index], 4);index+=4;
               // Satellites       1               byte        none
               memcpy((unsigned char*)(&System->gnssMetrics.satellites), &Packet->payload[index], 1);index++;
               // Lock             1               byte        boolean
               memcpy((unsigned char*)(&System->gnssMetrics.lockStatus), &Packet->payload[index], 1);index++;
               // VBus             2              uint         mV
               memcpy((unsigned char*)(&System->Vbus), &Packet->payload[index], 2);index+=2;
               // GPS speed (magnitude of vector sum)           cm/s
               memcpy((unsigned char*)(&System->gnssMetrics.velocity), &Packet->payload[index],4);index+=4;
               // GPS down velocity                             cm/s
               memcpy((unsigned char*)(&System->gnssMetrics.VelDown), &Packet->payload[index], 4);index+=4;
               // GPS Heading                                   deg 
               memcpy((unsigned char*)(&System->gnssMetrics.fHeading), &Packet->payload[index], 4);index+=4;
               // Horizontal accuracy estimate      unsigned long, mm
               memcpy((unsigned char*)(&System->gnssMetrics.hAcc), &Packet->payload[index], 4);index+=4;
               // Vertical accuracy estimate        unsigned long, mm
               memcpy((unsigned char*)(&System->gnssMetrics.vAcc), &Packet->payload[index], 4);index+=4;
               // GPS Ground speed                  unsigned in cm/s
               memcpy((unsigned char*)(&System->gnssMetrics.groundSpeed), &Packet->payload[index], 4);index+=4;
               
               // Motion & attitude metrics
               //Accelerations
               memcpy((unsigned char*)(&System->motioniMetrics.AccelX), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.AccelY), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.AccelZ), &Packet->payload[index], 4);index+=4;
               // Velocities
               memcpy((unsigned char*)(&System->motioniMetrics.VelX), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.VelY), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.VelZ), &Packet->payload[index], 4);index+=4;
               // magnetic field
               memcpy((unsigned char*)(&System->motioniMetrics.MagX), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.MagY), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.MagZ), &Packet->payload[index], 4);index+=4;
               // Angles
               memcpy((unsigned char*)(&System->motioniMetrics.Phi), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.Theta), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.Zeta), &Packet->payload[index], 4);index+=4;
               // Angular rate 
               memcpy((unsigned char*)(&System->motioniMetrics.PhiDot), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.ThetaDot), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.ZetaDot), &Packet->payload[index], 4);index+=4;
               // Barometer
               memcpy((unsigned char*)(&System->motioniMetrics.baroAltitude), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.baroPressure), &Packet->payload[index], 4);index+=4;
               memcpy((unsigned char*)(&System->motioniMetrics.baroTemperature), &Packet->payload[index], 4);index+=4;

               // Continuity status   1 byte
               memcpy((unsigned char*)(&System->flightComputerStatus.continuity), &Packet->payload[index], 1);index+=1;
               // Output Status       1 byte  
               memcpy((unsigned char*)(&System->flightComputerStatus.outputs), &Packet->payload[index], 1);index+=1;
               // State machine value unsigned int
               memcpy((unsigned char*)(&System->flightComputerStatus.state), &Packet->payload[index], 2);index+=2;
               Packet->newData=1;
     } 

         default : {
                 // unrecognised pacekt type
                 Packet->unhandledPacketType=1;
                 Packet->validPacket=0;
         }
      } // end of packet type switch
     return Packet->unhandledPacketType;
}