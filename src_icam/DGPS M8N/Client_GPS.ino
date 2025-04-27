#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WiFiUdp.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
*/
static const int RXPin = 17, TXPin = 16;
static const uint32_t GPSBaud = 9600;

// --- Configuration Wi-Fi (identique au point d’accès SoftAP) ---
const char* ssid = "BASE_GPS";
const char* password = "base_gps";

// --- Configuration UDP ---
WiFiUDP udp;
const int udpPort = 4210;
char incomingPacket[512];  // Buffer de réception

const long baudRates[] = { 9600, 19200, 38400, 57600, 115200, 230400, 460800 };
const int numBauds = sizeof(baudRates) / sizeof(baudRates[0]);

const unsigned char UBLOX_REVERT[] PROGMEM = {
  // 0 - Config UBX-CFG-CFG - Revert to default Configuration
  0xB5,0x62,0x06,0x09,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x03,0x1B,0x9A  // REVERT to default configuration
};

const unsigned char UBLOX_CONFIG_PORT[] PROGMEM = {  
  // 0 - Config UBX-PORT - GPS COM-UART1
  //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x84,0x03,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x7E,0xB8
  0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xBA,0x4E   // DGPS      UART1 - 115200 - IN:UBX - OUT:UBX+NEMA
  //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x23,0x00,0x00,0x00,0x00,0x00,0xDA,0x0E   // RTK-BASE  UART1 - 115200 - IN:UBX - OUT:UBX+NEMA+RTCM3
  //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x23,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xDA,0x52   // RTK-ROVER UART1 - 115200 - IN:UBX+NEMA+RTCM3 - OUT:UBX
  //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x23,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xDC,0x5E   // RTK-ROVER UART1 - 115200 - IN:UBX+NEMA+RTCM3 - OUT:UBX+NMEA
};

const unsigned char UBLOX_MSG_INIT[] PROGMEM = {
  // 1 - Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // 2 - Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // 3 - Enable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // 4 - Rate
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
  
  // 5 - Configuration permanente
  // Il serait peut-êre possible de sauvegarder la configuration pour éviter de repasser par cette procédure en cas 
  // de déconnection du GPS ?
  // UBX-CFG-CFG (save current config as permanent config)
  // 0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x1D,0xAB // save current config as permanent config

   // End - Configuration
};

const unsigned char UBX_HEADER[]        = { 0xB5, 0x62 };  // Entête sur deux octetsspécifique des modules U-Blox
const unsigned char NAV_POSLLH_HEADER[] = { 0x01, 0x02 }; // Message POSLLH
const unsigned char NAV_STATUS_HEADER[] = { 0x01, 0x03 }; // Message STATUS
const unsigned char NAV_PVT_HEADER[]    = { 0x01, 0x07 }; // Message PVT

enum _ubxMsgType {  // Les trois messages partagent le même espace en mémoire
  MT_NONE,
  MT_NAV_POSLLH,
  MT_NAV_STATUS,
  MT_NAV_PVT
};

struct NAV_POSLLH {   // Message élémentaire Voir documentation U-Blox page 
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

struct NAV_STATUS {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  unsigned char gpsFix;
  char flags;
  char fixStat;
  char flags2;
  unsigned long ttff;
  unsigned long msss;
};

// Ne fonctionne pas avec U-Blox M6N ou M7N.
// Sur U-Blox M8N module veillez au numéro de version du firmware
struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  
  unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)  
  unsigned short year;         // Year (UTC) 
  unsigned char month;         // Month, range 1..12 (UTC)
  unsigned char day;           // Day of month, range 1..31 (UTC)
  unsigned char hour;          // Hour of day, range 0..23 (UTC)
  unsigned char minute;        // Minute of hour, range 0..59 (UTC)
  unsigned char second;        // Seconds of minute, range 0..60 (UTC)
  char valid;                  // Validity Flags (see graphic below)
  unsigned long tAcc;          // Time accuracy estimate (UTC) (ns)
  long nano;                   // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;       // GNSSfix Type, range 0..5
  char flags;                  // Fix Status Flags
  unsigned char reserved1;     // reserved
  unsigned char numSV;         // Number of satellites used in Nav Solution
  
  long lon;                    // Longitude (deg)
  long lat;                    // Latitude (deg)
  long height;                 // Height above Ellipsoid (mm)
  long hMSL;                   // Height above mean sea level (mm)
  unsigned long hAcc;          // Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;          // Vertical Accuracy Estimate (mm)
  
  long velN;                   // NED north velocity (mm/s)
  long velE;                   // NED east velocity (mm/s)
  long velD;                   // NED down velocity (mm/s)
  long gSpeed;                 // Ground Speed (2-D) (mm/s)
  long heading;                // Heading of motion 2-D (deg)
  unsigned long sAcc;          // Speed Accuracy Estimate
  unsigned long headingAcc;    // Heading Accuracy Estimate
  unsigned short pDOP;         // Position dilution of precision
  short reserved2;             // Reserved
  unsigned long reserved3;     // Reserved
                // New firmware U-Blox M8N
  long headVeh;         // I4 deg Heading of vehicle (2-D), this is only valid when headVehValid is set, otherwise the output is set to the heading of motion
  short magDec;         // I2 1e-2 magDec deg Magnetic declination. Only supported in ADR 4.10 and later.
  unsigned short magAcc;    // U2 1e-2 magAcc deg Magnetic declination accuracy. Only supported in ADR 4.10 and later.
};

union UBXMessage {
  NAV_POSLLH navPosllh;
  NAV_STATUS navStatus;
  NAV_PVT    navPvt;
};

UBXMessage ubxMessage;

// GPS latitude et longitude in classical float representation
double gpsLat;
double gpsLon;


// The serial connection to the GPS device
char state;
bool config_gps_ok;
bool valid_ack;

bool waitForAck(byte msgClass, byte msgID, uint16_t timeout);


void setup()
{
  Serial.begin(115200);

    // Connexion au réseau Wi-Fi créé par l'autre ESP32
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connexion au Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    if (WiFi.status() == WL_IDLE_STATUS){
      Serial.print("."); 
    }if (WiFi.status() == WL_CONNECT_FAILED){
      Serial.println("Mauvais mot de passe, ou erreur de connexion");
    }if (WiFi.status() == WL_NO_SSID_AVAIL){
      Serial.println("Le réseau Wi-Fi demandé n’a pas été trouvé");
    }
    delay(500);
  }

  Serial.println("\nConnecté au Wi-Fi !");
  Serial.print("Adresse IP locale : ");
  Serial.println(WiFi.localIP());

  // Démarrage du client UDP
  udp.begin(udpPort);
  Serial.print("En écoute UDP sur le port ");
  Serial.println(udpPort);
  
  Serial.printf("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
  Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());  

  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();

  long br = detect_gps_baudrate();
  Serial.println(br);
  
  config_gps_ok = false;
}

void loop()
{  
  float delta_lat;
  float delta_lon;

    int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0';  // Ajout du caractère de fin de chaîne
    }

    //Serial.print("Message reçu : ");
    //Serial.println(incomingPacket);
    
    String message = String (incomingPacket);
    
    int commaIndex = message.indexOf(',');

    // Extraire les deux valeurs
    String firstValue =message.substring(0, commaIndex);
    String secondValue = message.substring(commaIndex + 1);
  
    // Convertir en entier si besoin
    delta_lat = firstValue.toFloat();
    delta_lon = secondValue.toFloat();
  
    // Affichage
    //Serial.println("Valeur 1 : " + String(delta_lat));
    //Serial.println("Valeur 2 : " + String(delta_lon));
   }
  
  long br;
  
  if( Serial.available() > 0 ) {
    state = Serial.read();
    Serial.print("state="); Serial.println(state);

    //=== Revert to Default Configuration ===
    Serial.println("UBX-GPS Reset and setup PORT confiugration");
    sendUBX( UBLOX_REVERT, sizeof(UBLOX_REVERT) );
    detect_gps_baudrate(); 

    //=== Config PORT to 115200 Bauds ===//
    Serial.println("UBX-GPS Config PORT configuration");
    sendUBX( UBLOX_CONFIG_PORT, sizeof(UBLOX_CONFIG_PORT) );
    br = detect_gps_baudrate(); 
    // Wait For ACK - 0x06 = CFG, 0x00 = CFG-PRT
    while( br != 115200 ) { 
        Serial.println("ACK not received");
        br = detect_gps_baudrate();
        sendUBX( UBLOX_CONFIG_PORT, sizeof(UBLOX_CONFIG_PORT) );
        br = detect_gps_baudrate();
    }
    // Send configuration data in UBX protocol
    Serial.println("UBX-GPS Configuration");
    sendUBX( UBLOX_MSG_INIT, sizeof(UBLOX_MSG_INIT) );

    Serial.println();
    Serial.println("Config GPS Terminated => OK.");
    delay(1000);
    config_gps_ok = true;
  }

  if( config_gps_ok ) {
    int msgType = processGPS();

    if ( msgType == MT_NAV_STATUS ) {
      //Serial.print("[STATUS] gpsFix:");    Serial.print(ubxMessage.navStatus.gpsFix);
      Serial.println();
    }
    else if ( msgType == MT_NAV_POSLLH ) {
      float GPS_lat = ubxMessage.navPosllh.lat;
      float GPS_lon = ubxMessage.navPosllh.lon;

      float lat_corrige = GPS_lat + delta_lat;
      float lon_corrige = GPS_lon + delta_lon;

      Serial.print(delta_lat);
      Serial.print(",");
      Serial.print(delta_lon);
      Serial.print(",");
      Serial.print(GPS_lat);
      Serial.print(",");
      Serial.print(GPS_lon);
      Serial.print(",");
      Serial.print(lat_corrige);
      Serial.print(",");
      Serial.println(lon_corrige);
      
      //Serial.print("[POSLLH] iTOW:"); Serial.print(ubxMessage.navPosllh.iTOW);
      //Serial.print(" lat/lon: "); Serial.print(ubxMessage.navPosllh.lat); Serial.print(","); Serial.print(ubxMessage.navPosllh.lon);
      //Serial.print(" hAcc: ");    Serial.print(ubxMessage.navPosllh.hAcc/1000.0f);
      //Serial.println();
    }
    else if ( msgType == MT_NAV_PVT ) {
      /*Serial.print("[PVT] SV: "); Serial.print(ubxMessage.navPvt.numSV);
      Serial.print(" fixType: "); Serial.print(ubxMessage.navPvt.fixType);
      Serial.print(" Date:");     Serial.print(ubxMessage.navPvt.year); Serial.print("/"); Serial.print(ubxMessage.navPvt.month); Serial.print("/"); Serial.print(ubxMessage.navPvt.day); Serial.print(" "); Serial.print(ubxMessage.navPvt.hour); Serial.print(":"); Serial.print(ubxMessage.navPvt.minute); Serial.print(":"); Serial.print(ubxMessage.navPvt.second);
      // Eviter les arrondis qui peuvent faire perdre de la précision
      Serial.print(" lat/lon: "); Serial.print(ubxMessage.navPvt.lat); Serial.print(","); Serial.print(ubxMessage.navPvt.lon);
      Serial.print(" gSpeed: ");  Serial.print(ubxMessage.navPvt.gSpeed/1000.0f);
      Serial.print(" heading: "); Serial.print(ubxMessage.navPvt.heading/100000.0f);
      Serial.print(" hAcc: ");    Serial.print(ubxMessage.navPvt.hAcc/1000.0f);
      Serial.println();*/    
    } else {
      //Serial.println('.');
      delay(1000);
    }
  }
}

long detect_gps_baudrate()
{
  long baudrate = 0;
  uint8_t ubxFound = 0;
  uint8_t nmeaFound = 0;
  
  for (int i = 0; i < numBauds; i++) {
    baudrate = baudRates[i];
    Serial2.flush();
    Serial2.end();
    Serial.print("Trying GPS at "); Serial.print(baudrate); Serial.print("... ");
    Serial2.begin(baudrate, SERIAL_8N1, RXPin, TXPin);

    bool found = false;
    unsigned long start = millis();
    while( millis() - start < 1000 ) {
      if( Serial2.available() ) {
        char c = Serial2.read();
        
        // NMEA detection: look for '$G'
        if( nmeaFound == 0 && c == '$') {
          ubxFound = 0;
          nmeaFound++;
        } else if( nmeaFound == 1 && c == 'G') {
          nmeaFound++;
          break;
        }
        // UBX detection: look for 0xB5 0x62
        else if( ubxFound == 0 && c == 0xB5) {
          nmeaFound = 0;
          ubxFound++;
        } else if (ubxFound == 1 && c == 0x62) {
          ubxFound++;
          break;
        }
        // Reset Wrong UBX or NMEA Frame
        else {
          nmeaFound = 0;
          ubxFound = 0;
        }
      }
    }

    if( nmeaFound >= 2 || ubxFound >= 2) {
      Serial.println("Found GPS!");
      break;
    } else {
      Serial.println("No data.");
      baudrate = 0;
    }
  }

  return baudrate;
}

bool waitForAck(byte msgClass, byte msgID, uint16_t timeout = 1000)
{
  byte ackPacket[10]; // UBX-ACK-ACK is always 10 bytes
  byte expectedAck[] = {
    0xB5, 0x62,     // Sync chars
    0x05, 0x01,     // Class, ID (ACK-ACK)
    0x02, 0x00,     // Payload length = 2
    msgClass, msgID // Payload: class and ID of original message
    // Followed by checksum
  };

  unsigned long start = millis();
  uint8_t i = 0;

  while (millis() - start < timeout) {
    if (Serial2.available()) {
      byte b = Serial2.read();

      if (i < 10) {
        ackPacket[i++] = b;
      }

      // Once we have the full packet
      if (i == 10) {
        // Compare first 8 bytes
        bool match = true;
        for (uint8_t j = 0; j < 8; j++) {
          if (ackPacket[j] != expectedAck[j]) {
            match = false;
            break;
          }
        }

        // Verify checksum
        if (match) {
          byte ckA = 0, ckB = 0;
          for (int j = 2; j < 8; j++) {
            ckA += ackPacket[j];
            ckB += ckA;
          }
          if (ckA == ackPacket[8] && ckB == ackPacket[9]) {
            return true; // ACK-ACK received
          }
        }

        // Shift buffer to look for next packet if needed
        i = 0;
      }
    }
  }

  return false; // Timeout
}

void sendUBX( const byte *msg, uint8_t len )
{
    bool valid_ack = false;
    
    for(uint8_t i = 0; i < len; i++) {                        
      byte c = pgm_read_byte( msg+i );
      Serial2.write( c );
      Serial.print( c, HEX );        
    }
    Serial2.flush();
    Serial.println();
}

// The last two bytes of the message is a checksum value, used to confirm that the received payload is valid.
// The procedure used to calculate this is given as pseudo-code in the uBlox manual.
void calcChecksum(unsigned char* CK, int msgSize) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char*)(&ubxMessage))[i];
    CK[1] += CK[0];
  }
}


// Compares the first two bytes of the ubxMessage struct with a specific message header.
// Returns true if the two bytes match.
boolean compareMsgHeader(const unsigned char* msgHeader) {
  unsigned char* ptr = (unsigned char*)(&ubxMessage);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}


// Reads in bytes from the GPS module and checks to see if a valid message has been constructed.
// Returns the type of the message found if successful, or MT_NONE if no message was found.
// After a successful return the contents of the ubxMessage union will be valid, for the 
// message type that was found. Note that further calls to this function can invalidate the
// message content, so you must use the obtained values before calling this function again.
int processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  
  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);

  while ( Serial2.available() ) {  
    byte c = Serial2.read();    
    //Serial.write(c);
    
    if ( fpos < 2 ) {
      // For the first two bytes we are simply looking for a match with the UBX header bytes (0xB5,0x62)
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0; // Reset to beginning state.
    }
    else {
      // If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
      // and we are now reading in the bytes that make up the payload.
      
      // Place the incoming byte into the ubxMessage struct. The position is fpos-2 because
      // the struct does not include the initial two-byte header (UBX_HEADER).
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&ubxMessage))[fpos-2] = c;

      fpos++;
      
      if ( fpos == 4 ) {
        // We have just received the second byte of the message type header, 
        // so now we can check to see what kind of message it is.
        if ( compareMsgHeader(NAV_POSLLH_HEADER) ) {
          currentMsgType = MT_NAV_POSLLH;
          payloadSize = sizeof(NAV_POSLLH);
        }
        else if ( compareMsgHeader(NAV_STATUS_HEADER) ) {
          currentMsgType = MT_NAV_STATUS;
          payloadSize = sizeof(NAV_STATUS);
        }
        else if ( compareMsgHeader(NAV_PVT_HEADER) ) {
          currentMsgType = MT_NAV_PVT;
          payloadSize = sizeof(NAV_PVT);
        }
        else {
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }

      if ( fpos == (payloadSize+2) ) {
        // All payload bytes have now been received, so we can calculate the 
        // expected checksum value to compare with the next two incoming bytes.
        calcChecksum(checksum, payloadSize);
      }
      else if ( fpos == (payloadSize+3) ) {
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if ( c != checksum[0] ) {
          // Checksum doesn't match, reset to beginning state and try again.
          fpos = 0; 
        }
      }
      else if ( fpos == (payloadSize+4) ) {
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0; // We will reset the state regardless of whether the checksum matches.
        if ( c == checksum[1] ) {
          // Checksum matches, we have a valid message.
          return currentMsgType; 
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        // We have now read more bytes than both the expected payload and checksum 
        // together, so something went wrong. Reset to beginning state and try again.
        fpos = 0;
      }
    }
  }
  
  return MT_NONE;
}
