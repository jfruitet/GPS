#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0x9F, 0x98, 0x1C}; // Adresse MAC du récepteur
float latitude;
float delta_lat;
float curent_lat;
float longitude;
float delta_long;
float curent_long;


static const int RXPin = 32, TXPin = 26;  // Broches UART du GPS
static const uint32_t GPSBaud = 9600;     // Vitesse de communication du GPS

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Utilisation du second port série

typedef struct struct_message {
    char text[64]; // Taille augmentée pour stocker la chaîne complète de coordonnées
} struct_message;

struct_message myData;

// Callback pour suivre l'état de l'envoi
void sentCallback(const uint8_t *mac, esp_now_send_status_t status) {
    Serial.print("État de l'envoi : ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Succès" : "Échec");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    latitude = 47.274942;
    longitude = -1.506522; 

    // Afficher l'adresse MAC de l'émetteur
    Serial.print("Adresse MAC de l'émetteur : ");
    Serial.println(WiFi.macAddress());
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    Serial.println("Recherche du signal GPS...");

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_register_send_cb(sentCallback);

    // Ajouter le récepteur en tant que peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Échec d'ajout du récepteur !");
        return;
    }
    Serial.println("Récepteur ajouté avec succès !");
}

void loop() {
    esp_err_t result;  // Déclaration de la variable result avant son utilisation

    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());  // Lecture des données GPS
    }
  
    if (gps.location.isUpdated()) {  // Si de nouvelles coordonnées sont disponibles
      // Conversion de la longitude et latitude en chaîne de caractères
      curent_lat = gps.location.lat();
      curent_long = gps.location.lng();

      delta_lat = latitude - curent_lat;
      delta_long = longitude - curent_long;
      
      snprintf(myData.text, sizeof(myData.text), "Delta Lng: %f, Delta Lat: %f", delta_long, delta_lat);
      result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
      Serial.print("Delta longitude: ");
      Serial.print(delta_long, 10);  // 6 décimales
      Serial.print(", Delta latitude: ");
      Serial.println(delta_lat, 10);  // 6 décimales + retour à la ligne
          
    } else {
      Serial.println("Aucun signal GPS reçu. En attente d'une connexion aux satellites...");
      // Envoi d'un message vide ou d'un message d'erreur
      strcpy(myData.text, "Delta Lng: 0.000000, Delta Lat: 0.000000");
      result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    }

    delay(2000);  // Petite pause pour éviter un affichage trop rapide
    Serial.println(myData.text);
    Serial.println();
}
