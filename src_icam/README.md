# RoBoNav - Mineure ICAM 2024 - 2025

Agathe D., Thed K.

Ce rapport présente l'avancement du projet RoBoNav au 24/04/2025

Les dossiers fournissent les versions de code source pour les différentes étapes de notre projet.

### Objectifs

Implanter un DGPS avec deux modules GPS U-Blox MN8, l'un comme base fixe à une position connue, l'autre en mode rover.

L'algorithme consiste à transmettre au rover les écarts en longitude et latitude entre la position connue et la position captée par la base.

Tester l'apport de GPS RTK, en mode DGNSS et en mode RTK  

![](C:\Users\jf\AppData\Roaming\marktext\images\2025-04-27-16-17-14-image.png)

### Matériels

Chip ESP32 WROOM, GPS U-Blox MN8, antenne premier prix

Chip ESP32 WROOM, GPS RTK Quectel L29HDA, antenne hélicoïdale L1, L5 

### Sources

* Dossier **1er DGPS** : 2 Atom Lite à base de ESP32 Pico et 2 GPS M5Stack

* Dossier **DGPS M8N** : 2 ESP32 WROOM et 2 GPS U-Blox MN8N  

* Dossier **DGPS LC29HDA** : 2 ESP32 WROOM et 2 GPS Quectel LC29HDA RTK


