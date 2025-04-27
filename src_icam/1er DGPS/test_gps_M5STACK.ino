#include <TinyGPS++.h>
#include <HardwareSerial.h>

static const int RXPin = 32, TXPin = 26;  // Broches UART du GPS
static const uint32_t GPSBaud = 9600;     // Vitesse de communication du GPS

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Utilisation du second port série

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Serial.println("Recherche du signal GPS...");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());  // Lecture des données GPS
  }

  if (gps.location.isUpdated()) {  // Si de nouvelles coordonnées sont disponibles
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    Serial.println("------------------------");
  } else {
    Serial.println("Aucun signal GPS reçu. En attente d'une connexion aux satellites...");
  }

  delay(2000);  // Petite pause pour éviter un affichage trop rapide
}
 
