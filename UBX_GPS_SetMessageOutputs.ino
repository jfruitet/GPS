// Programmation des modules GPS U-Blox
// UBX_GPS_MultipleMessage.ino
// Tous les messages peuvent être vérifiés en utilisant le logiciel U-Center de U-Blox
// Commentaires en Anglais sur Youtube
// https://www.youtube.com/watch?v=TwhCX0c8Xe0
// https://www.youtube.com/watch?v=ylxwOg2pXrc

// Sketches par @iforce2d
// https://www.iforce2d.net/sketches/

/********
Configuration d'un module GPS U-Blox M6N, M7N, M8N et M9N
Evite de passer par U-Center 2 pour configurer un GPS U-Blox
@iforce2d says:
Making a non-permanent change via the program at run-time means the end-user never needs to fiddle with any message settings, 
you can use a brand-new module right out of the box and it will work, and you can switch your GPS modules to any other 
usage and back without caring about how they are set up. There is no need to use the u-center app at all if you 
are game to read the datasheet and work out what the message should be, but it's a hell of a lot easier 
and less error-prone to just copy and paste it in (and much easier when making a video like this to demonstrate 
what the hell is going on :).

Ce code initialise le module GPS connecté aux broches 3 et 5 d'une carte Arduino

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

Principe du code
Des messages sont envoyés (en écriture) sur le port série du GPS pour 
1) Déconnecter le protocole NMEA et supprimer une série de messages non pertinents
2) Faire un reset des messages UBX utiles
3) Activer les messages UBX utiles
4) Sélectionner un taux d'échantillonage de 10 hz
5) A tester : sauvegarder la configuration de façon permanente

Pour ajouter d'autres fonctionnalités et 
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

// Connect the GPS RX/TX to arduino pins 3 and 5
#define GPSRX 3
#define GPSTX 5

SoftwareSerial serial = SoftwareSerial(GPSRX,GPSTX);

const unsigned char UBLOX_INIT[] PROGMEM = {
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
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // 4 - Rate
  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
  
  // 5 - Configuration permanente
  // Il serait peut-êre possible de sauvegarder la configuration pour éviter de repasser par cette procédure en cas 
  // de déconnection du GPS ?
  // UBX-CFG-CFG (save current config as permanent config)
  // 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB

};

void setup()
{
  serial.begin(9600);

  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    serial.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); 	// Simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
				// Voir la remarque dans les commentaires de la vidéo N°1
				// @ColinMcCormack
				// It's worth noting that the device will send back an ACK message (0x05) upon receipt 
				// of a configuration message. 
				// If you waited until that came in before sending the next one, you'd not have to put in delays. 

  }
}

void loop() {}















