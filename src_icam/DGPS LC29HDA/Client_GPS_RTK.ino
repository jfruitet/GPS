#include <WiFi.h>
#include <WiFiUdp.h>

#define MAX_BUFFER 1024
#define DEBUG_GPS_INFO false

// --- GPS (SoftwareSerial) ---
const byte rxPin = 16;
const byte txPin = 17;
//SoftwareSerial gps(rxPin, txPin);

// --- Configuration Wi-Fi (identique au point d’accès SoftAP) ---
const char* ssid = "BASE_GPS";
const char* password = "base_gps";

// --- Configuration UDP ---
WiFiUDP udp;
const int udpPort = 4210;
char incomingPacket[MAX_BUFFER+1];  // Buffer de réception

//--- Data Decodage GPS ---
size_t bufferIndex = 0;
uint8_t rawBuffer[MAX_BUFFER+1];
bool receivingRTCM = false;
bool receivingNMEA = false;
bool validFrame = false;

// ======================= CRC24Q TABLE POUR RTCM ========================
static const unsigned crc24q[256] = {
    0x00000000, 0x01864CFB, 0x028AD50D, 0x030C99F6,
    0x0493E6E1, 0x0515AA1A, 0x061933EC, 0x079F7F17,
    0x08A18139, 0x0927CDC2, 0x0A2B5434, 0x0BAD18CF,
    0x0C3267D8, 0x0DB42B23, 0x0EB8B2D5, 0x0F3EFE2E,
    0x10C54E89, 0x11430272, 0x124F9B84, 0x13C9D77F,
    0x1456A868, 0x15D0E493, 0x16DC7D65, 0x175A319E,
    0x1864CFB0, 0x19E2834B, 0x1AEE1ABD, 0x1B685646,
    0x1CF72951, 0x1D7165AA, 0x1E7DFC5C, 0x1FFBB0A7,
    0x200CD1E9, 0x218A9D12, 0x228604E4, 0x2300481F,
    0x249F3708, 0x25197BF3, 0x2615E205, 0x2793AEFE,
    0x28AD50D0, 0x292B1C2B, 0x2A2785DD, 0x2BA1C926,
    0x2C3EB631, 0x2DB8FACA, 0x2EB4633C, 0x2F322FC7,
    0x30C99F60, 0x314FD39B, 0x32434A6D, 0x33C50696,
    0x345A7981, 0x35DC357A, 0x36D0AC8C, 0x3756E077,
    0x38681E59, 0x39EE52A2, 0x3AE2CB54, 0x3B6487AF,
    0x3CFBF8B8, 0x3D7DB443, 0x3E712DB5, 0x3FF7614E,
    0x4019A3D2, 0x419FEF29, 0x429376DF, 0x43153A24,
    0x448A4533, 0x450C09C8, 0x4600903E, 0x4786DCC5,
    0x48B822EB, 0x493E6E10, 0x4A32F7E6, 0x4BB4BB1D,
    0x4C2BC40A, 0x4DAD88F1, 0x4EA11107, 0x4F275DFC,
    0x50DCED5B, 0x515AA1A0, 0x52563856, 0x53D074AD,
    0x544F0BBA, 0x55C94741, 0x56C5DEB7, 0x5743924C,
    0x587D6C62, 0x59FB2099, 0x5AF7B96F, 0x5B71F594,
    0x5CEE8A83, 0x5D68C678, 0x5E645F8E, 0x5FE21375,
    0x6015723B, 0x61933EC0, 0x629FA736, 0x6319EBCD,
    0x648694DA, 0x6500D821, 0x660C41D7, 0x678A0D2C,
    0x68B4F302, 0x6932BFF9, 0x6A3E260F, 0x6BB86AF4,
    0x6C2715E3, 0x6DA15918, 0x6EADC0EE, 0x6F2B8C15,
    0x70D03CB2, 0x71567049, 0x725AE9BF, 0x73DCA544,
    0x7443DA53, 0x75C596A8, 0x76C90F5E, 0x774F43A5,
    0x7871BD8B, 0x79F7F170, 0x7AFB6886, 0x7B7D247D,
    0x7CE25B6A, 0x7D641791, 0x7E688E67, 0x7FEEC29C,
    0x803347A4, 0x81B50B5F, 0x82B992A9, 0x833FDE52,
    0x84A0A145, 0x8526EDBE, 0x862A7448, 0x87AC38B3,
    0x8892C69D, 0x89148A66, 0x8A181390, 0x8B9E5F6B,
    0x8C01207C, 0x8D876C87, 0x8E8BF571, 0x8F0DB98A,
    0x90F6092D, 0x917045D6, 0x927CDC20, 0x93FA90DB,
    0x9465EFCC, 0x95E3A337, 0x96EF3AC1, 0x9769763A,
    0x98578814, 0x99D1C4EF, 0x9ADD5D19, 0x9B5B11E2,
    0x9CC46EF5, 0x9D42220E, 0x9E4EBBF8, 0x9FC8F703,
    0xA03F964D, 0xA1B9DAB6, 0xA2B54340, 0xA3330FBB,
    0xA4AC70AC, 0xA52A3C57, 0xA626A5A1, 0xA7A0E95A,
    0xA89E1774, 0xA9185B8F, 0xAA14C279, 0xAB928E82,
    0xAC0DF195, 0xAD8BBD6E, 0xAE872498, 0xAF016863,
    0xB0FAD8C4, 0xB17C943F, 0xB2700DC9, 0xB3F64132,
    0xB4693E25, 0xB5EF72DE, 0xB6E3EB28, 0xB765A7D3,
    0xB85B59FD, 0xB9DD1506, 0xBAD18CF0, 0xBB57C00B,
    0xBCC8BF1C, 0xBD4EF3E7, 0xBE426A11, 0xBFC426EA,
    0xC02AE476, 0xC1ACA88D, 0xC2A0317B, 0xC3267D80,
    0xC4B90297, 0xC53F4E6C, 0xC633D79A, 0xC7B59B61,
    0xC88B654F, 0xC90D29B4, 0xCA01B042, 0xCB87FCB9,
    0xCC1883AE, 0xCD9ECF55, 0xCE9256A3, 0xCF141A58,
    0xD0EFAAFF, 0xD169E604, 0xD2657FF2, 0xD3E33309,
    0xD47C4C1E, 0xD5FA00E5, 0xD6F69913, 0xD770D5E8,
    0xD84E2BC6, 0xD9C8673D, 0xDAC4FECB, 0xDB42B230,
    0xDCDDCD27, 0xDD5B81DC, 0xDE57182A, 0xDFD154D1,
    0xE026359F, 0xE1A07964, 0xE2ACE092, 0xE32AAC69,
    0xE4B5D37E, 0xE5339F85, 0xE63F0673, 0xE7B94A88,
    0xE887B4A6, 0xE901F85D, 0xEA0D61AB, 0xEB8B2D50,
    0xEC145247, 0xED921EBC, 0xEE9E874A, 0xEF18CBB1,
    0xF0E37B16, 0xF16537ED, 0xF269AE1B, 0xF3EFE2E0,
    0xF4709DF7, 0xF5F6D10C, 0xF6FA48FA, 0xF77C0401,
    0xF842FA2F, 0xF9C4B6D4, 0xFAC82F22, 0xFB4E63D9,
    0xFCD11CCE, 0xFD575035, 0xFE5BC9C3, 0xFFDD8538,
};


uint32_t computeCRC24Q(const uint8_t *data, size_t length) {
    uint32_t crc = 0;
    for (int i = 0; i < length; i++) {
        crc = (crc << 8) ^ crc24q[data[i] ^ (unsigned char)(crc>>16)];
    }
    return (crc & 0x00ffffff);
}



void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, rxPin, txPin);

  // Configuration de GPS en mode Rover RTK
  Serial2.println("$PQTMRESTOREPAR*13");            // Restore parametres par défaut
  Serial2.println("$PQTMCFGRCVRMODE,W,1*2A");       // Configure en mode Rover RTK
  Serial2.println("$PQTMSAVEPAR*5A");               // Sauvegarde de la configuration
  Serial2.println("$PAIR023*3B");                   // Reboot Module
  delay(2000);
  Serial2.println("$PAIR062,1,0*3C");               // Turn off GLL messages
  Serial2.println("$PAIR062,2,0*3C");               // Turn off GSA messages
  Serial2.println("$PAIR062,3,0*3D");               // Turn off GSV messages
  Serial2.println("$PAIR062,5,0*3B");               // Turn off VTG messages
  Serial2.println("$PAIR062,0,01*0F");              // Enable GCA NMEA Messages

  //Serial2.println("$PAIR100,1,0*3A");
  //Serial2.println("$PAIR400,1*23");
  Serial2.println("$PQTMCFGNMEADP,W,3,8,3,3,3,3*39"); // Set decimal precision for NMEA
  //Serial2.println("$PAIR050,200*21");              // Set ouput interval to 200ms
  Serial2.println("$PQTMSAVEPAR*5A");               // Sauvegarde de la configuration

  
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
}

void loop()
{ 
  char hexa[3] = { 0,0,0 };

  /***********************************************************/
  /*** UDP Packet Analysis ***/
  /***********************************************************/
  int packetSize = udp.parsePacket();
  if( packetSize ) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if( len >= 6 ) {
      // Formatage
      incomingPacket[len] = '\0';  // Ajout du caractère de fin de chaîne
      Serial.print('.');
/*      Serial.println( len );
      Serial.print(" Message reçu : ");

      String debugMesg = "";
      for( uint8_t i = 0; i < len; i++ ) {
        sprintf(hexa, "%02X", incomingPacket[i]);
        debugMesg += hexa;
        debugMesg += ' ';
      }
      Serial.println( debugMesg );
*/      
      //--- Write RTCM to GPS ---
      Serial2.write(incomingPacket);
    }
  }

  /***********************************************************/
  /*** Decodage GPS ***/
  /***********************************************************/
  while( Serial2.available() )
  {
    uint8_t byteIn = Serial2.read();
    // Empile tant que le buffer n'est pas plein
    if( bufferIndex < MAX_BUFFER )
        rawBuffer[bufferIndex++] = byteIn;    

    // Initialisation de la trame
    if( bufferIndex == 1 ) {
        receivingRTCM = ( byteIn == 0xD3 );
        receivingNMEA = ( byteIn == '$' );
        continue;
    }
    // Détection de fin de trame
    else if( (bufferIndex >= MAX_BUFFER) ||
             (bufferIndex >= 2 && rawBuffer[bufferIndex-2] == 0x0D && rawBuffer[bufferIndex-1] == 0x0A ) ||
             (receivingRTCM && bufferIndex >= 6 && bufferIndex >= (((rawBuffer[1] & 0x03) << 8) | rawBuffer[2]) + 6 )
            )
    {   // Store end of Trame
        rawBuffer[bufferIndex] = 0;
        frameDebug_Display();

        // Decode/parse la trame NMEA (pour affichage ccord) et RTCM (pour envoi RTK en UDP au Rover)
        if( receivingNMEA )
            validFrame = parseNMEA( rawBuffer, bufferIndex );
        else if( receivingRTCM )
            validFrame = parseRTCM3( rawBuffer, bufferIndex );
        else
            validFrame = invalidFrame( "Invalid niNMEA-niRTCM ", (const char *)rawBuffer );
        
        // Clean Reception Buffer
        receivingRTCM = false;
        receivingNMEA = false;
        bufferIndex = 0;
        break;
    }
  }
  
  // Si l’utilisateur envoie une commande depuis le PC via le port série
  if( Serial.available() ) {
    String mesg = "";
    while( Serial.available() ) {
      mesg += (char)Serial.read();
      delay(5);
    }
    Serial.println(mesg);
    mesg = addNMEAchecksum(mesg);
    Serial.println(mesg);
    Serial2.println(mesg);  // Transmet au GPS
  }

  delay(10); // petite pause pour ne pas saturer la boucle
}


bool invalidFrame( const char *err_msg, const char *buffer ) {
  if( DEBUG_GPS_INFO )
  { // Debug Print Buffer
    Serial.println( err_msg ); 
    Serial.println( buffer ); 
  }
  return false;
}

void frameDebug_Display() {
  char hexa[3] = { 0,0,0 };

  if( DEBUG_GPS_INFO )
  { // Debug Print Buffer
    String debugMesg = "";
    for( uint8_t i = 0; i < bufferIndex; i++ ) {
      sprintf(hexa, "%02X", rawBuffer[i]);
      debugMesg += hexa;
      debugMesg += ' ';
    }
    Serial.println( (char *)rawBuffer );
    //Serial.println( debugMesg );
  }
}


// ======================= CRC NMEA ========================
String addNMEAchecksum(const String& message) {
  uint8_t checksum = 0;
  char checksumStr[4];

  // Calculer et Formater le checksum en deux chiffres hexadécimaux
  for (int i = 1; i < message.length(); i++)
    checksum ^= message[i];
  sprintf(checksumStr, "*%02X", checksum);

  // Construire la trame complète
  String fullSentence = message + String(checksumStr) + "\r\n";
  return fullSentence;
}

// ======================= CRC NMEA ========================
bool checkNMEACRC( const String &nmea ) {
  int asterisk = nmea.indexOf('*');
  if (asterisk == -1 || asterisk + 2 >= nmea.length()) return false;

  uint8_t checksum = 0;
  for (int i = 1; i < asterisk; i++) {
    checksum ^= nmea[i];
  }

  String crcStr = nmea.substring(asterisk + 1, asterisk + 3);
  uint8_t crcFromStr = strtol(crcStr.c_str(), NULL, 16);
  return (checksum == crcFromStr);
}

// ======================= PARSEUR NMEA ========================
bool parseNMEA( const uint8_t *rtcmBuffer, size_t length ) {
  String nmea = String(rtcmBuffer, length);
  String message = "";
  
  if( checkNMEACRC( nmea ) )
  {
    // Trame NMEA valide    
    if( nmea.startsWith("$GNGGA") ) {
      message = "Position Fix (GGA): ";
      extractNMEACoordinate( nmea );
    } else if( nmea.startsWith("$GNRMC") ) {
      message = "Recommended Minimum (RMC): ";
      //extractNMEACoordinate( nmea );
    } else if( nmea.startsWith("$PQTM") ) {
      message = "PQTM > ";
      Serial.print( message );
      Serial.println( nmea );
    } else if( nmea.startsWith("$PAIR") ) {
      message = "PAIR > ";
      Serial.print( message );
      Serial.println( nmea );
    } else {
      message = "Other NMEA: ";
    }

    if( DEBUG_GPS_INFO )
    { // Debug Print NMEA
      Serial.print( message );
      Serial.println( nmea );
    }

    // Autres types ici
    return true;
  }
  else
  { // Trame NMEA invalide
    invalidFrame( "Wrong CRC - NMEA > ", nmea.c_str() );
  }

  return false;
}

void extractNMEACoordinate( const String nmea ) {
  if( nmea.length() < 6 ) return;

  // === Identifier le type de trame (GGA, RMC...) sans le préfixe GNSS ===
  String nType = nmea.substring(3, 6);

  // ==== EXTRACT LAT/LON ====
  if( (nmea.startsWith("$GNGGA")) || (nmea.startsWith("$GNRMC")) ) {
    // On découpe la trame en morceaux
    int fieldIndex = 0;
    String fields[20];  // max 20 champs
    int start = 0;

    for (int i = 0; i < nmea.length(); i++) {
      if( nmea.charAt(i) == ',' || nmea.charAt(i) == '*' ) {
        fields[fieldIndex++] = nmea.substring(start, i);
        start = i + 1;
      }
    }

    // GGA: $GNGGA,time,lat,N/S,lon,E/W,...
    // RMC: $GNRMC,time,status,lat,N/S,lon,E/W,...
    int fixIndex = 6;
    int latIndex = nType == "GGA"? 2 : 3;
    int lonIndex = latIndex + 2;

    String mesg = "GPS: ";
    // Insert GPS Fix value
    if( nmea.startsWith("$GNGGA") && fields[fixIndex].length() > 0 ) {
      // Convertir en entier pour switch
      int code = fields[fixIndex].toInt();
      switch (code) {
        case 0: mesg += "(0) NO Fix"; break;
        case 1: mesg += "(1) Fix GPS - "; break;
        case 2: mesg += "(2) Fix DGPS - "; break;
        case 4: mesg += "(4) Fix RTK - "; break;
        case 5: mesg += "(5) Fix RTK flottant -"; break;
        default: mesg+= "(" + String(code) + ") Fix inconnu "; break;
      }
    }

    // Insert GPS latidue et longitude
    if( fields[latIndex].length() > 0 && fields[lonIndex].length() > 0 ) {
      double latitude = convertNMEACoordinate(fields[latIndex], fields[latIndex + 1]);
      double longitude = convertNMEACoordinate(fields[lonIndex], fields[lonIndex + 1]);

      mesg += " Lat,Lon : ";
      mesg += String(latitude, 10);
      mesg += " , ";
      mesg += String(longitude, 10);
    }

    // Output GPS position data
    Serial.println( mesg );
  }
}

double convertNMEACoordinate(String nmeaCoord, String direction) {
  if( nmeaCoord.length() < 4 ) return 0.0;

  // Exemple : 4807.038,N → 48°07.038'
  double raw = nmeaCoord.toFloat();
  int deg = int(raw / 100);
  double min = raw - (deg * 100);
  double dec = deg + (min / 60.0);

  if (direction == "S" || direction == "W") {
    dec *= -1.0;
  }

  return dec;
}

// ========== Décodage binaire RTCM3 simplifié ==========
bool parseRTCM3(const uint8_t *rtcmBuffer, size_t length)
{ 
  if( length < 6 ) return false;  // trop court

  // Reprend la longueur depuis l'entête
  uint16_t rtcmLength = ((rtcmBuffer[1] & 0x03) << 8) | rtcmBuffer[2];
  
  // Calcul le CRC de la trame => L = 3 header + N + 3 CRC
  //Serial.println("LEN"); Serial.println(length); Serial.println(rtcmLength + 6);
  if( length == rtcmLength + 6 )
  { 
    uint32_t crcRx = (rtcmBuffer[length - 3] << 16) |
                     (rtcmBuffer[length - 2] << 8 ) |
                      rtcmBuffer[length - 1];
    uint32_t crcCalc = computeCRC24Q( rtcmBuffer, rtcmLength + 3 );
    //Serial.println("CRC"); Serial.println(crcRx); Serial.println(crcCalc);

    // Si la trame RTCM est valide
    if( crcRx == crcCalc )
    {
      if( DEBUG_GPS_INFO )
      {
        // Décalage pour le début des données après 3 octets d'en-tête
        const uint8_t *payload = &rtcmBuffer[3];

        // Message type = bits 0 à 11
        uint16_t msgType = getBits(payload, 0, 12);
        Serial.print("RTCM MsgType: ");
        Serial.println(msgType);

        switch (msgType) {
          case 1005:
            decodeRTCM1005(payload, length - 6);
            break;
          case 1077:
            Serial.println("Observations GNSS (1077) - non décodé ici");
            break;
          default:
            Serial.print("Msg RTCM inconnu (");
            Serial.print(msgType);
            Serial.print("), taille=");
            Serial.println(length);
            break;
        }
      }

      return true;
    }
    else {
      invalidFrame( "RTCM - CRC FAIL - ", (const char*)rtcmBuffer );
    }
  } else {
      invalidFrame( "RTCM - Invalid Length - ", (const char*)rtcmBuffer );
    }

  return false;
}

// Lecture bits utils RTCM3
uint32_t getBits(const uint8_t *data, int startBit, int bitLen) {
  uint32_t result = 0;
  for (int i = 0; i < bitLen; i++) {
    int bit = startBit + i;
    int byteIdx = bit / 8;
    int bitIdx = 7 - (bit % 8);
    result <<= 1;
    result |= (data[byteIdx] >> bitIdx) & 0x01;
  }
  return result;
}

// ========== Décodage message RTCM 1005 (Station de base) ==========
void decodeRTCM1005(const uint8_t *payload, size_t len) {
  uint16_t stationID = getBits(payload, 12, 12);
  uint8_t  system = getBits(payload, 24, 6);
  uint8_t  refStation = getBits(payload, 30, 1);
  int32_t  x = getBits(payload, 36, 38);
  int32_t  y = getBits(payload, 74, 38);
  int32_t  z = getBits(payload, 112, 38);

  Serial.println("RTCM 1005 - Station Reference");
  Serial.print(" - Station ID: "); Serial.println(stationID);
  Serial.print(" - ITRF System: "); Serial.println(system);
  Serial.print(" - Reference Station: "); Serial.println(refStation);
  Serial.print(" - ECEF X: "); Serial.println((double)x / 10000.0, 7);
  Serial.print(" - ECEF Y: "); Serial.println((double)y / 10000.0, 7);
  Serial.print(" - ECEF Z: "); Serial.println((double)z / 10000.0, 7);
}

