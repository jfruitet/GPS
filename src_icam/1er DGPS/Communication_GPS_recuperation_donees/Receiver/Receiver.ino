#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <iostream>
#include <cstdio>  // Pour sscanf
#include <Arduino.h>
#include <vector> 

float longitude;
float current_long;
double delta_long;
float latitude;
float current_lat;
double delta_lat;
int long_donnees = 20;
std::vector<float> liste_current_coor_lat;
std::vector<float> liste_current_coor_lng;
std::vector<float> liste_corrige_coor_lat;
std::vector<float> liste_corrige_coor_lng;


static const int RXPin = 32, TXPin = 26;  // Broches UART du GPS
static const uint32_t GPSBaud = 9600;     // Vitesse de communication du GPS

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Utilisation du second port série

typedef struct struct_message {
    char text[41];
} struct_message;

struct_message myData;

// Callback pour la réception des messages
void receiveCallback(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    //Serial.print("Message reçu de : ");
    //Serial.println(WiFi.macAddress()); // Affiche l'adresse MAC de l'expéditeur

    memcpy(&myData, incomingData, sizeof(myData));
    //Serial.print("Message : ");
    String message = myData.text;
    //Serial.println(message);
    
    double deltaLng, deltaLat;

    // Extraction des valeurs
    int startLng = message.indexOf(":") + 2; // Trouver le début du premier nombre
    int endLng = message.indexOf(",");       // Fin du premier nombre
    int startLat = message.lastIndexOf(":") + 2; // Début du deuxième nombre

    // Conversion des sous-chaînes en double
    delta_long = message.substring(startLng, endLng).toDouble();
    delta_lat = message.substring(startLat).toDouble();
    
    // Affichage des résultats
    //Serial.print("Delta Lng: ");
    //Serial.println(delta_long, 6);
    //Serial.print("Delta Lat: ");
    //Serial.println(delta_lat, 6);
}

void afficherListe(const std::vector<float>& Liste, const String& nom) {
    Serial.print(nom + ": ");
    for (size_t i = 0; i < Liste.size(); i++) {
        Serial.print(Liste[i], 6);  // Affiche avec 6 décimales
        if (i < Liste.size() - 1) {
            Serial.print(", ");  // Virgule entre les éléments
        }
    }
    Serial.println();  // Nouvelle ligne après le dernier élément
}

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    WiFi.mode(WIFI_STA);

    // Afficher l'adresse MAC du récepteur
    Serial.print("Adresse MAC du récepteur : ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_register_recv_cb(receiveCallback);
}

void loop() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());  // Lecture des données GPS
    }

    if (gps.location.isUpdated()) {  // Si de nouvelles coordonnées sont disponibles
        current_lat = gps.location.lat();
        current_long = gps.location.lng();
        //liste_current_coor_lat.push_back(current_lat);
        //liste_current_coor_lng.push_back(current_long);

        //if (liste_current_coor_lat.size() > long_donnees) {
            //liste_current_coor_lat.erase(liste_current_coor_lat.begin());  // Supprime le premier élément pour garder une taille max de 10
        //}
        //if (liste_current_coor_lng.size() > long_donnees) {
            //liste_current_coor_lng.erase(liste_current_coor_lng.begin());  // Supprime le premier élément pour garder une taille max de 10
        //}
        //afficherListe(liste_current_coor_lat, "current_lat");
        //afficherListe(liste_current_coor_lng, "current_lng");
        
        //Serial.print("Longitude: ");
        //Serial.print(current_long, 6);  // 6 décimales
        //Serial.print(", Latitude: ");
        //Serial.println(current_lat, 6);  // 6 décimales + retour à la ligne

        longitude = current_long + delta_long;
        latitude = current_lat + delta_lat;

        //Serial.print("Longitude corrigée: ");
        //Serial.print(longitude, 6);  
        //Serial.print(", Latitude corrigée: ");
        //Serial.println(latitude, 6);

        //liste_corrige_coor_lat.push_back(latitude);
        //liste_corrige_coor_lng.push_back(longitude);

        //if (liste_corrige_coor_lat.size() > long_donnees) {
            //liste_corrige_coor_lat.erase(liste_corrige_coor_lat.begin());  // Supprime le premier élément pour garder une taille max de 10
        //}
        //if (liste_corrige_coor_lng.size() > long_donnees) {
            //liste_corrige_coor_lng.erase(liste_corrige_coor_lng.begin());  // Supprime le premier élément pour garder une taille max de 10
        //}
        //afficherListe(liste_corrige_coor_lat, "corrige_lat");
        //afficherListe(liste_corrige_coor_lng, "corrige_lng");

        Serial.print("1");
        Serial.print(" ");
        Serial.print(delta_lat, 10);
        Serial.print(" ");
        Serial.print(delta_long, 10);
        Serial.print(" ");
        Serial.print(current_lat, 10);
        Serial.print(" ");
        Serial.print(current_long, 10);
        Serial.print(" ");
        Serial.print(latitude, 10);
        Serial.print(" ");
        Serial.println(longitude, 10);
    } 
    else {
        //Serial.println("Aucun signal GPS reçu. En attente d'une connexion aux satellites...");
        Serial.println("0 00.0000000000 00.0000000000 00.0000000000 00.0000000000 00.0000000000 00.0000000000");
    }
    //Serial.println("------------------------");
    delay(2000);  // Petite pause pour éviter un affichage trop rapide
}
