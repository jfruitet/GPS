# GPS

## Présentation

Pour configurer un GPS –  vitesse de transfert (baud rates), protocole, fréquence d’acquisition, etc. –  on peut relier le module GPS à port série d'un contrôleur de vol Pixhawk,  connecter celui-ci par câble USB au PC sur lequel on aura installé au préalable  les logiciels **U-Center** de U-Blox et **Mission Planner**.

Une fois la configuration du module effectuée avec U-Center, Mission Planer permet de vérifier que les données sont correctement lues et placées sur une cartographie de la zone de travail. **Il faut alors sauvegarder la configuration avec U-Center*** et réinstaller le GPS sur la carte Arduino ou ESP32… 

Cette procédure doit être répétée pour chaque module GPS de chaque rover. En cas de changement de module, il faut la reprendre entièrement, ce qui n'est pas du tout plug & play !

Alternativement, et c'est ce que ce code permet, on intégre au code source du firmware une séquence de configuration en binaire UBX.  

### Protocoles de lecture des données GPS

Les modules GPS U-Blox sont programmables, mais seulement avec le format propriétaire binaire UBX qui permet de lire des données et d’en envoyer au GPS, en particulier pour le configurer ; les autres protocoles (NMEA, …) sont uniquement en lecture.

Nous allons privilégier le protocole UBX qui est plus compact et évite de parser des chaînes de caractères pour interpréter les messages MNEA.

## Configuration et lecture des messages des module GPS U-Blox

Les sources ci-après sont reprises in extenso des vidéos Youtube publiées en 2015 par @iforce2d
 https://www.youtube.com/watch?v=TwhCX0c8Xe0
 https://www.youtube.com/watch?v=ylxwOg2pXrc

On trouvera nombre de croquis utiles sur la page de @iforce2d
 https://www.iforce2d.net/sketches/

dont ceux qui ont servi à rédiger ce code.

Dans le fichier source *UBX_GPS_SetMessageOutputs.ino* on trouve **comment configurer** un module GPS U-Blox M6N, M7N, M8N ( ? M9N, M10 ) connecté aux broches RX et TX d'une carte Arduino.

Dans le fichier source *UBX_GPS.ino* on trouve comment lire les trames  **UBX-NAV-POSLLH** (Geodetic position solution). 

Dans le fichier source *UBX_GPS_MultipleMessage.ino* on trouve comment lire  deux messages différents : **UBX-NAV-POSLLH** (Geodetic position solution)  et **UBX-NAV-STATUS** (Receiver navigation status).

Dans le fichier source *UBX_GPS_NAV_PVT.ino* on trouvecomm ent lire les trames   **UBX-NAV-PVT** (Navigation position velocity time solution).




### U-Blox data type

| Nom court | Type           | Octets | Type de données Arduino                                                         |
| --------- | -------------- | ------ | ------------------------------------------------------------------------------- |
| U1        | Unsigned char  | 1      | unsigned char                                                                   |
| I1        | Signed char    | 1      | char                                                                            |
| X1        | Bitfield       | 1      | unsigned char                                                                   |
| U2        | Unsigned short | 1      | unsigned short                                                                  |
| I2        | Signed short   | 2      | short                                                                           |
| X2        | Bitfield       | 2      | unsigned short                                                                  |
| U4        | Unsigned long  | 4      | unsigned long                                                                   |
| I4        | Signed long    | 4      | long                                                                            |
| X4        | Bitfield       | 4      | uunsigned long                                                                  |
| R4        | Float          | 4      | float                                                                           |
| R8        | Double         | 8      | ? You could try two 4-byte types, to at least read the data from the GPS module |
| CH        | CHAR           | &      | CHAR                                                                            |
|           |                |        |                                                                                 |

### Nota Bene

Lors de la connexion série du GPS U-BLOX N8M à la carte Arduino (ou ESP32) il faut veiller à [utiliser des broches différentes](https://stackoverflow.com/questions/75050941/esp32-connected-to-gps-module-no-serial-out-unless-holding-down-reset-button) de celles consacrées à la connexion série USB pour qu’elles n’entrent pas en conflit. 

Quand un seul port série est disponible, le port matériel (hardware port) est assigné aux connexions USB et au débogage.

Il faut alors assigner deux autres broches au GPS pour éviter les conflits de port série. Soit, si la carte le permet, utiliser deux autres broches numériques (digital pins : ~PIN) soit utiliser une bibliothèque comme <SoftwareSerial.h>
 https://docs.arduino.cc/learn/built-in-libraries/software-serial/

qui permet d'adresser des ports série supplémentaires sur les broches d'une carte Arduino.

## Messages UBX utiles

Documentation constructeur pour les GPS U-Blox Neo M8N

https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf?utm_content=UBX-13003221

On y trouvera la liste des commandes UBX acceptées par cette famille de modules GPS.

A vrai dire les seules commandes qui nous seront utiles portent sur la configuration et la lecture des positions GPS : 

UBX-CFG-PRT (Port configuration for UART ports)  
UBX-NAV-POSLLH (Geodetic position solution)

UBX-NAV-STATUS (Receiver navigation status)

UBX-NAV-PVT (Navigation position velocity time solution) à utiliser à partir des module UBX NEO M8N

On peut suivre la méthode proposée par @iforce2d dans ses vidéos pour lire d'autres trames.
