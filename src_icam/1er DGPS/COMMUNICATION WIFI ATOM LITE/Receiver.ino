#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
    char text[32];
} struct_message;

struct_message myData;

// Callback pour la réception des messages
void receiveCallback(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    Serial.print("Message reçu de : ");
    Serial.println(WiFi.macAddress()); // Affiche l'adresse MAC de l'expéditeur

    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("Message : ");
    Serial.println(myData.text);
}

void setup() {
    Serial.begin(115200);
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

void loop() {}
