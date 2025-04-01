// UBX_GPS_MultipleMessages.ino
// Programmation des modules GPS U-Blox pour récupérer les positions GPS
// La syntaxe des messages est tirée du logiciel U-Center de U-Blox
// Commentaires en Anglais sur Youtube
// https://www.youtube.com/watch?v=TwhCX0c8Xe0
// https://www.youtube.com/watch?v=ylxwOg2pXrc

// Sketches par @iforce2d
// https://www.iforce2d.net/sketches/

/********
Dans le fichier source UBX_GPS_SetMessageOutputs.ino on trouvera comment configurer 
un module GPS U-Blox M6N, M7N, M8N ( ? et M9N) connecté aux broches GPSRX et GPSTX d'une carte Arduino

Dans le fichier source UBX_GPS.ino on trouvera comment lire les trames   
UBX-NAV-POSLLH (Geodetic position solution) )

Dans le fichier source UBX_GPS_MultipleMessage.ino on trouvera comment lire les trames   
UBX-NAV-POSLLH (Geodetic position solution) )et UBX-NAV-STATUS (Receiver navigation status)

Dans le fichier source UBX_GPS_NAV_PVT.ino on trouvera comment lire les trames   
UBX-NAV-PVT (Navigation position velocity time solution)

Ce code utilise la bibliothèque <SoftwareSerial.h>
https://docs.arduino.cc/learn/built-in-libraries/software-serial/
qui permet d'adresser des ports série supplémentaires sur les broches numériques (digital pins) d'une carte Arduino.

ATTENTION : Veillez à éviter les conflits de ports série.
Il sera probablement nécessaire de choisir d'autres broches (pins) si vous utilisez
aussi une connexion série pour le débugage USB
https://stackoverflow.com/questions/75050941/esp32-connected-to-gps-module-no-serial-out-unless-holding-down-reset-button

Pour le format des données U-Blox consulter le document 
https://docs.google.com/spreadsheets/d/14Fo6mMFCGLDnGiOhI6vzVnZVtHBz8d1JU0cUR86ovSs/edit?gid=0#gid=0

Pour ajouter d'autres types de messages consultez le logiciel U-Center de U-Blox
https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf?utm_content=UBX-13003221

*********/


/*************************** U-Blox data type ********************************

Short name	Type			Bytes	Arduino type	
U1			Unsigned char	1		unsigned char	
I1			Signed char		1		char	
X1			Bitfield		1		unsigned char	
U2			Unsigned short	2		unsigned short	
I2			Signed short	2		short	
X2			Bitfield		2		unsigned short	
U4			Unsigned long	4		unsigned long	
I4			Signed long		4		long	
X4			Bitfield		4		unsigned long	
R4			Float			4		float	
R8			Double			8		?	You could try two 4-byte types, to at least read the data from the GPS module. 
CH			Char			1		char	

*****************************************************************************/
#include <SoftwareSerial.h>

#define GPSRX 3
#define GPSTX 5

// Connect the GPS RX/TX to arduino pins 3 and 5
SoftwareSerial serial = SoftwareSerial(GPSRX,GPSTX);

const unsigned char UBX_HEADER[]        = { 0xB5, 0x62 };	// Entête sur deux octetsspécifique des modules U-Blox
const unsigned char NAV_POSLLH_HEADER[] = { 0x01, 0x02 };	// Message POSLLH
const unsigned char NAV_STATUS_HEADER[] = { 0x01, 0x03 };	// Message STATUS

enum _ubxMsgType {	// Les trois messages partagent le même espace en mémoire
  MT_NONE,
  MT_NAV_POSLLH,
  MT_NAV_STATUS
};

struct NAV_POSLLH {		// Message élémentaire Voir documentation U-Blox page 
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

union UBXMessage {
  NAV_POSLLH navPosllh;
  NAV_STATUS navStatus;
};

UBXMessage ubxMessage;

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

  while ( serial.available() ) {
    
    byte c = serial.read();    
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

void setup() 
{
  Serial.begin(9600);
  serial.begin(9600);
}

long lat;
long lon;

void loop() {
  int msgType = processGPS();
  if ( msgType == MT_NAV_POSLLH ) {
    Serial.print("iTOW:");      Serial.print(ubxMessage.navPosllh.iTOW);
    Serial.print(" lat/lon: "); Serial.print(ubxMessage.navPosllh.lat); Serial.print(","); Serial.print(ubxMessage.navPosllh.lon);
    Serial.print(" hAcc: ");    Serial.print(ubxMessage.navPosllh.hAcc/1000.0f);
    Serial.println();
  }
  else if ( msgType == MT_NAV_STATUS ) {
    Serial.print("gpsFix:");    Serial.print(ubxMessage.navStatus.gpsFix);
    Serial.println();
  }
}

