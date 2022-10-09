/*******************************************************
  GraiNMaster
  système de contrôle de température de la cuve
  Débuté le 24/11/2015 par Nicolas Morival
  Testé et compilé sur Arduino IDE v1.6.8
*******************************************************/

#define version  " --V1.005-- "

/*
  v1.000 le 14/05/2016
  V1.002 le 08/10/2016
  nettoyage du code, suppression des morceaux de code inutiles (relatifs à la SD)
  suppression du paramètre sur SD detect : cette broche est une interruption fixée par le hardware
  ajout de la possibilité d'utiliser une sonde DS18B20
  v1.003 le 09/11/2017
  correction de bugs sur l'initialisation de l'eeprom (bug relevé par artiche)
  mise en fonction du buzzer (suggestion artiche)
  v1.004 le 15/04/2019 - DaOurZ
  Ajout de la possibilité de deux sondes de température
  "Simplification" du code
  Désactivation possible de l'EEPROM, mode Debug, du buzzer, du défilement
  Changement de la température et du temps restant à la volée pour chaque palier en cours
  Sortie propre du programme en fin de brassage
  Création d'un fichier de log différent à chaque brassage
  Redéfinition des informations affichées à l'écran (avec témoin d'atteinte du palier)
*/


//bibliothèques
#include <SPI.h>
#include "SdFat.h"
#include <ArduinoJson.h>
#include <PID_v1.h> //gestion de l'algo PID
//#include <avr/pgmspace.h>
#include "Timer.h" // gestion des timers
#include <jm_LCM2004A_I2C.h> //gestion du LCD
#include <avr/io.h>
#include <avr/interrupt.h> //gestion des interruptions
#include <Wire.h>

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 rtc;

char RtcTime[9] = "xxhxxmxx";
char RtcDate[11] = "xx/xx/xxxx";
DateTime now;





// Json config
struct Config {
  //valeurs de l'hysteresis pour passage en PID fin ou grossier
  byte hysteresis_Pos =  1;
  byte hysteresis_Neg  = 1;

  String myPaliers[2][7] = {
    {"Prechauffe", "Palier 1  ", "Palier 2  ", "Palier 3  ", "Palier 4  ", "Mash-out  ", "Ebullition"},   // Nom des paliers en version longue (10 caractères)
    {"PREC",         "PAL1",     "PAL2",     "PAL3",     "PAL4",     "MOUT",     "EBUL"      }    // Nom des paliers en version courte (4 caractères)
  };

  float myTemperatures[7] = {
    40,     // = thetastart, température de préchauffe de la cuve
    40.25,  // = theta1, température empâtage céréales non maltées
    52.5,   // = theta2, température protéines
    62.0,   // = theta3, température de sacharification B amylase
    68.75,  // = theta4, température de sacharification A amylase
    75.75,  // = thetaMO, température de mash out
    103.0   // = theta5, température d'ébullition
  }; // Températures pour chaque myPaliers

  byte myTempo[7] {
    0,      // tempo_A, pour l'affichage temporaire
    0,      // tempo1, palier empâtage céréales non maltées
    15,     // tempo2, palier protéinique
    35,     // tempo3, palier de sacharification B amylase
    30,     // tempo4, palier de sacharification A amylase
    5,      // palier de Mash Out
    70      // tempo5, ébullition
  }; // Temporisation pour chaque myPaliers

  // Règlages par défaut du PID
  // Ces constantes sont correctes pour une cuve de 27L 2500W non isolée type kitchen chef

  byte P_strong = 100;
  float I_strong = 0.0;
  byte D_strong = 16;

  byte P_weak = 80;
  float I_weak = 0.02;
  byte D_weak = 8;

  byte PID_OFFSET = 1; // on ruse le PID pour lui faire décaller la température cible d'une valeur constante par exemple +1° donc si l'utilisateur vise 62° et que la température mesurée est de 61° le PID croit qu'il a atteint les 62° et coupe la chauffe. Ainsi l'overshoot est limité.
  // en théorie, avec les paramètres réglés au top, ce paramètre peut être remis à zéro.

  byte Beep_PIN = 40;
  byte ledPin = 13;

};

const char *filename = "/config.txt";  // <- SD library uses 8.3 filenames
Config config;                         // <- global configuration object

double Kp = config.P_strong; // multiplicateur de l'erreur différentielle de température.
double Ki = config.I_strong; //coef de correction inverse
double Kd = config.D_strong; //coef de dérivée

#define DEBUG           // Décommenter pour afficher les informations de deboguage (PID sur le LCD)
#define MAX31865
#define MAX31865_2

float temp1;
float temp2;


#define DEFIL           // Commenter pour ne pas faire alterner le texte sur la deuxième ligne pour afficher le temps restant total avant la fin de l'ébullition
#define PIDDEBUG        // Commenter pour enlever l'information du PID sur l'écran lors de la chauffe

#ifdef PIDDEBUG
#define PID_DEBUG(x) + String(x)
#else
#define PID_DEBUG(x)
#endif

#ifdef DEBUG

void printdebug(String str)
{
  Serial.println(String(RtcDate) + " " + String(RtcTime) + " : " + str);
}

#define DEBUGPRINTLN(x) printdebug(x)
#else
#define DEBUGPRINTLN(x)
#endif

#define BLINK() ledState = !ledState; digitalWrite(config.ledPin, ledState);

#ifdef DEFIL
bool defilement = 0;    // Position du défilement / 1 = température et température à atteindre / 0 = température et temps restant / s'échange à chaque écriture sur la carte SD, donc toutes les 10 s environ
#endif

byte DETECT = 2 , GATE = 3; //Broches utilisées pour le pilotage du triac -> detect : impulsion du passage au point zéro, gate : gachette du triac
byte PULSE = 4  ; //triac gate pulse : largeur de l'impulsion pour activer le triac, en nombre de cycles d'horloge
int HALF_WAVE = 560;// //nombre de "tics" d'horloge à chaque demi onde 50Hz . 625 théoriques, mais 560 réels .

#if defined(MAX31865)
#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(42, 43, 44, 45);

#if defined(MAX31865_2)
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(38, 39, 40, 41);
#endif

// use hardware SPI, just pass in the CS pin
// Adafruit_MAX31865 thermo = Adafruit_MAX31865(42);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
#endif

// constantes pour les touches reliées au module LCD
#define btnUP     22
#define btnDOWN   23
#define btnLEFT   24
#define btnRIGHT  25
#define btnSELECT 26
#define btnNONE   0

int btnUPValue = HIGH;
int btnDOWNValue = HIGH;
int btnLEFTValue = HIGH;
int btnRIGHTValue = HIGH;
int btnSELECTValue = HIGH;

//  pins utilisées par le LCD
jm_LCM2004A_I2C lcd(0x27);

// valeurs utilisées pour configurer le LCD
byte backLight   = 10;    // LCD Panel Backlight LED connected to digital pin 10
int lcd_key     = 0;
int adc_key_in  = 0;

byte ledState = LOW;

// variables du programme
volatile unsigned int menu = 0;
byte submenu = 0;
byte startprog = 0; //marqueur pour indiquer que le programme s'éxécute
byte jump = 0; //pour indiquer qu'on va sauter un menu

byte annuler = 0; //pour annuler les opérations 0 = fonctionnement normal  / -1 on recule d'une étape / 1 on avance d'une étape

byte secondes = 0;
unsigned long secondes_reel = 0; //temps écoulé depuis le début du programme, utilisé pour le data log
unsigned int minutes = 0; //minutes écoulées
unsigned long total_time = 0; //cumul des temps programme
unsigned long temps_pause = 0; //si le programme est en pause, compte le temps écoulé
bool palier_atteint = false;
int logRecord_ID;   // Timer ID pour la fonction logRecord, pour un arrêt propre du programme (on veut le stopper avant d'arrêter le programme pour éviter toute écriture quand on ddébranche l'Arduino)

unsigned int cooling = 0; // compteur du temps de refroidissement

//int therm; //variable pour le relevé de température

// double chauffe_reel = 0, tx_chauffe, theta_mesure, theta_PID, theta_objectif = 20;
float tx_chauffe;
float theta_PID, theta_mesure, theta_objectif = 20;

PID myPID((double*)&theta_PID, (double*)&tx_chauffe, (double*)&theta_objectif, Kp, Ki, Kd, DIRECT); //déclaration du PID

#define BeepONState  LOW
#define BeepOFFState  HIGH
#define BeepON(s) BeepON(s)
byte beep_duration = 0;   // temps du bip : s / 128, 128 ms étant le temps entre chaque appel de sel_menu, la plus rapide des périodes pour vérifier que le bip est en route ou non
void BeepON(unsigned int s) {
  digitalWrite(config.Beep_PIN, BeepONState);
  beep_duration = (byte) ceil(s / 128);   // la durée du bip est comptée par période de 128 ms (arrondies à l'entier supérieur) - sel_menu, qui va vérifier si le bip est en route ou non, est appelée toutes les 128 ms
}
void BeepOFF() {
  digitalWrite(config.Beep_PIN, BeepOFFState);
}


//réglages des timers
Timer T;

//déclaratioon du fichier pour la SD

// SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
const uint8_t chipSelect = SS;

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3

// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
#define SPI_SPEED SD_SCK_MHZ(4)

//------------------------------------------------------------------------------
#if SD_FAT_TYPE == 0
SdFat sd;
File myFile;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 myFile;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile myFile;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile myFile;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

char datalogFile[40]; // Fichier de log, à chaque brassage différent

char degre = (char)176;  // char 176 = ° (signe degré) en unicode

void setTime()
{

  int setrtc[] = { now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second()};
  char item_ind[6][21] = {
    "^^                 ",
    "   ^^              ",
    "      ^^^^         ",
    "           ^^      ",
    "              ^^   ",
    "                 ^^"
  };
  int item = 0;
  char str[20];

  CLS();

  lcd_key = btnNONE;
  delay(1500);

  while ( lcd_key != btnSELECT )
  {

    CLS();           // move to position 0 on the first line
    lcd.print("Reglage date / heure");
    lcd.setCursor(0, 1);           // move to position 0 on the second line
    sprintf(str, "%02d-%02d-%04d %02dh%02dm%02d", setrtc[0], setrtc[1], setrtc[2], setrtc[3], setrtc[4], setrtc[5]);
    lcd.print(str);
    lcd.setCursor(0, 2);           // move to position 0 on the second line
    lcd.print(item_ind[item] );
    lcd.setCursor(0, 3);           // move to position 0 on the second line
    lcd.print("<< >> ok>Select");

    while ( lcd_key == btnNONE )
    {
      delay(200);
      lcd_key = read_LCD_buttons();  // read the buttons
    }

    switch (lcd_key)
    {
      case btnNONE:
        {
          break;
        }

      case btnLEFT:
        {
          item = item - 1;
          break;
        }

      case btnRIGHT:
        {
          item = item + 1;
          break;
        }

      case btnDOWN:
        {
          DEBUGPRINTLN ("Down !");
          setrtc[item] = setrtc[item] - 1;
          break;
        }

      case btnUP:
        {
          DEBUGPRINTLN ("Up !");
          setrtc[item] = setrtc[item] + 1;
          break;
        }

      case btnSELECT:
        {
          DEBUGPRINTLN ("Change Time !");
          rtc.adjust(DateTime(setrtc[2], setrtc[1], setrtc[0], setrtc[3], setrtc[4], setrtc[5]));
          return;
        }
    }

    lcd_key = btnNONE;

  }
}

void setup()
{
#if defined(DEBUG)
  Serial.begin(9600);
  DEBUGPRINTLN  ("Startup ok");
#endif

  // setup buttons
  pinMode(btnUP, INPUT_PULLUP);
  pinMode(btnDOWN, INPUT_PULLUP);
  pinMode(btnLEFT, INPUT_PULLUP);
  pinMode(btnRIGHT, INPUT_PULLUP);
  pinMode(btnSELECT, INPUT_PULLUP);

  // ECRAN DE PRESENTATION ----------------------------
  Wire.begin();
  lcd.begin();

  if (! rtc.begin()) {
    DEBUGPRINTLN("Couldn't find RTC");
    while (1);
  }
  if (! rtc.isrunning()) {
    DEBUGPRINTLN ("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2022, 1, 21, 3, 0, 0));
  }
  else
  {
    DEBUGPRINTLN ("RTC is running!");

    now = rtc.now();

    sprintf(RtcTime, "%02dh%02dm%02d", now.hour(), now.minute(), now.second());
    sprintf(RtcDate, "%02d-%02d-%04d", now.day(), now.month(), now.year());

    DEBUGPRINTLN("Time" );

    CLS();           // move to position 0 on the first line
    lcd.print("Date Heure OK ?");
    lcd.setCursor(0, 1);           // move to position 0 on the second line
    lcd.print(String(RtcDate) + " " + String( RtcTime));
    lcd.setCursor(0, 3);           // move to position 0 on the second line
    lcd.print("Select pour Changer!");
    while ( lcd_key == btnNONE)
    {
      delay(1000);
      lcd_key = read_LCD_buttons();  // read the buttons
    }

    switch (lcd_key)
    {
      case btnNONE:
        {
          break;
        }

      case btnSELECT:
        {
          DEBUGPRINTLN ("Change Time !");

          rtc.adjust(DateTime(0, 0, 0, 0, 0, 0));
          rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

          setTime();

          now = rtc.now();

          sprintf(RtcTime, "%02dh%02dm%02d", now.hour(), now.minute(), now.second());
          sprintf(RtcDate, "%02d-%02d-%04d", now.day(), now.month(), now.year());
          break;
        }

    }


  }


#if defined(MAX31865)
  DEBUGPRINTLN("Adafruit MAX31865 PT100");
  thermo.begin(MAX31865_3WIRE);  // set to 3WIRE or 4WIRE as necessary
#endif

#if defined(MAX31865_2)
  DEBUGPRINTLN("Adafruit MAX31865 PT100 Sonde 2");
  thermo2.begin(MAX31865_3WIRE);  // set to 3WIRE or 4WIRE as necessary
#endif
  //pinMode(53, OUTPUT); //la pin 53 est normalement attribuée à la fonction SPI Slave Select (SS)Permet d'activer ou désactiver le périphérique
  // Suivant les modules SD, il peut être nécessaire de commenter cette ligne (c'est le cas de mon module même si c'est pas logique)



  // création des caractères spéciaux
  byte beer1[8] = {
    B10001,
    B11111,
    B10001,
    B10001,
    B10001,
    B10001,
    B11111,
  };
  byte beer2[8] = {
    B00000,
    B11100,
    B00100,
    B00100,
    B00100,
    B00100,
    B11000,
  };

  lcd.createChar(0, beer1);
  lcd.createChar(1, beer2);
  CLS();           // fonction qui efface le LCD et met le curseur au début
  lcd.print(" - GraiN.Master - ");
  DEBUGPRINTLN  ("Print ok");
  lcd.setCursor(0, 1);
  lcd.write(byte(0));
  lcd.write(byte(1));
  lcd.print(version);
  lcd.write(byte(0));
  lcd.write(byte(1));
  delay(2000);

  // set up des pins
  pinMode(DETECT, INPUT);     //détection du passage au pont zéro / zero cross detect
  digitalWrite(DETECT, HIGH); // enable pull-up resistor
  pinMode(GATE, OUTPUT);      //triac gate control
  pinMode(config.Beep_PIN, OUTPUT);  // Déclaration du bipper

  pinMode(backLight, INPUT); //set backlight pin to input to avoid MCU overcurent on pin 10
  // pinMode(11, OUTPUT);     // pin11 = MOSI sur uno
  pinMode(config.ledPin, OUTPUT);

  // set up Timer1
  //(see ATMEGA 328 data sheet pg 134 for more details)
  OCR1A = 25;      //initialize the comparator
  TIMSK1 = 0x03;    //enable comparator A and overflow interrupts
  TCCR1A = 0x00;    //timer control registers set for
  TCCR1B = 0x00;    //normal operation, timer disabled

  myPID.SetOutputLimits(0, 255);

  // pinMode(10, OUTPUT);



  CLS();          // move cursor to beginning of line "0"
  lcd.print("initialise  SD  "); // print a simple message
  delay(600);
  if (!sd.begin(53)) {
    lcd.home();           // move cursor to beginning of line "0"
    lcd.print("ERREUR SD CARD "); // print a simple message
    delay(6000);
  }

  CLS();           // move cursor to beginning of line "0"
  lcd.print(" Lecture param "); // print a simple message

  if ( sd.exists(filename)) {

    // Should load default config if run for the first time
    DEBUGPRINTLN("Loading configuration...");
    loadConfiguration(filename, config);
    DEBUGPRINTLN("Loaded...");
  }
  else
  {

    // Create configuration file
    DEBUGPRINTLN("Saving configuration...");
    saveConfiguration(filename, config);
    DEBUGPRINTLN("Saved...");

  }

  // Dump config file
  DEBUGPRINTLN("Print config file...");
  printFile(filename);
  DEBUGPRINTLN("End Print config file...");

  sprintf(datalogFile, "log_%s_%s.txt", RtcDate, RtcTime); // Le nom du fichier, avec un numéro pseudo-aléatoire pour éviter de réécrire sur le même à chaque brassin
  DEBUGPRINTLN(strcat("Datalogfile = " , datalogFile));

  digitalWrite(config.ledPin, 1);
  myFile = sd.open(datalogFile, O_WRITE | O_CREAT | O_AT_END);
  if (myFile) {
    DEBUGPRINTLN(strcat("Writing info to Datalogfile = " , datalogFile));
    myFile.print("Temps (secondes)");
    myFile.print("; ");
    myFile.print("Palier en cours");
    myFile.print("; ");
    myFile.print("Objectif de temperature (°C)");
    myFile.print("; ");
    myFile.print("Temperature mesuree (°C)");
    myFile.print("; ");
    myFile.print("PID - taux de chauffe (0 - 255)");
    myFile.print('\n');
    myFile.close();
  }
  else
  {
    DEBUGPRINTLN(strcat("Error Writing info to Datalogfile = ", datalogFile));
  }

  digitalWrite(config.ledPin, 0);

  T.every(2205, lecture); // constante de temps pour le timer 1 - lecture de la sonde
  T.every(128, sel_menu); // constante de temps pour le timer 2 - lecture des touches
  T.every(1024, LCD_upd); // timer 3 - mise à jour écran
  T.every(1000, horloge); // timer 4 - compte les secondes et minutes
  T.every(350, regle_chauffe); //timer 5 - fréquence de mise à jour de la chauffe
  logRecord_ID = T.every(10002, logRecord); //enregiste sur la carte SD

  OCR1A = 500;
  CLS();
  LCD_upd();
  // set up zero crossing interrupt
  attachInterrupt(digitalPinToInterrupt(2), zeroCrossingInterrupt, CHANGE);
  //IRQ0 is pin 2. Call zeroCrossingInterrupt
  //on rising signal

  menu = 0;

}

//Boucle principale ========================================

void loop()
{

  T.update();

  //gestion des sauts d'étapes manuels
  if (annuler == -1)
  {
    menu--; //on était au menu X on retourne au X-1
    annuler = 0;
    logRecord();
  }
  else {
    if (annuler == 1)
    {
      menu++;
      jump = 0; //par sécurité on ne veut pas de double saut au cas où le passage au palier suivant était prévu au même moment
      annuler = 0;
      logRecord();
      palier_atteint = false; // le palier n'est pas encore atteint, il vient de commencer
    }
  }

  //gestion des sauts d'étapes automatiques
  if (jump == 1) {
    menu++;
    jump = 0;
    logRecord();
    BeepON(1500);
    palier_atteint = false; // le palier n'est pas encore atteint, il vient de commencer
  }

} //fin Boucle principale =================================

// Sélection de la température et du temps du palier
void paliers_temp(int numpalier, String *line1, String *line2) {
  *line1 = config.myPaliers[0][numpalier] + " ? ";
  if (submenu == 0) {
    *line2 = " > T : " + String(config.myTemperatures[numpalier]) + degre + "  " + String(config.myTempo[numpalier]) + " min";
  }
  else {
    *line2 = " T : " + String(config.myTemperatures[numpalier]) + degre + " > " + String(config.myTempo[numpalier]) + " min";
  }
}

// Sauter le palier en cours
void pass_palier(int numpalier, String *line1, String *line2) {
  *line1 = "SAUTER " + config.myPaliers[0][numpalier] + " ? ";
  *line2 = "SEL = OUI autre = NO";
}

// Annuler le palier en cours
void restart_palier(int numpalier, String *line1, String *line2) {
  *line1 = "RESTART " + config.myPaliers[0][numpalier] + " ? ";
  *line2 = "SEL = OUI autre = NO";
}

void display_palier(int numpalier, String *line1, String *line2) {
  unsigned int t = 0;
  // L'objectif de température est celui du palier en cours
  theta_objectif = config.myTemperatures[numpalier];
  // On ajoute tous les temps des paliers pour connaître le temps restant
  for (int i = 1; i <= numpalier; i++) {
    t += config.myTempo[i];
  }
  DEBUGPRINTLN(strcat(strcat("t : ", t), strcat( " / minutes : ", minutes)));
  // Si le temps n'est pas dépassé (t = temps total qu'il faut atteindre depuis le premier palier jusqu'à celui en cours)
  if (t > minutes) {
    config.myTempo[0] = t - minutes;
  }
  else {  // le palier est fini : on saute au palier suivant automatiquement
    jump = 1;
    BeepON(200);
    palier_atteint = false;
    return;
  }
  *line1 = config.myPaliers[1][numpalier] + "(" + String(config.myTempo[numpalier]) + ")";

  // on affiche un caractère différent s'il y a chauffe (" > ") ou si le palier a déjà été atteint (" : ")
  if (palier_atteint) {
    *line1 = *line1 + " : " + String(config.myTempo[0]) + " min";
  }
  else {
    *line1 = *line1 + " > " + String(config.myTempo[0]) + " min";
  }

  *line2 = String((int)theta_mesure) + " / " + String((int)config.myTemperatures[numpalier]);
#ifdef DEFIL
  if (defilement) { // on affiche alternativement la température en cours et la température à atteindre, ou...
#endif
    *line2 = *line2 + degre + "C ";
    *line2 = *line2 PID_DEBUG("PID : " + String((int)ceil(tx_chauffe)) + "  "); // Ne va s'ajouter que si on active la fonction de debug (désactivation : enlever #define PIDDEBUG)
#ifdef DEFIL
  }
  else {  // ... la température en cours et le temps total restant estimé avant la fin de l'ébullition, sans le temps inter-palier néanmoins
    t = 0;
    for (int i = numpalier + 1; i <= 6; i++) {
      t += config.myTempo[i];
    }
    t += config.myTempo[0];
    *line2 = *line2 + " REST : " + String(t) + "min";
  }
#endif
}   // Fin Display_palier

void LCD_upd() // affiche les infos à l'écran *********************************************************
{
  String Ligne1;
  String Ligne2;
  //  CLS(); // on efface l'écran et on place le curseur au début de la première ligne

  switch (menu)
  {
    case 0:   // Sélection de température de préchauffage
      {
        Ligne1 = config.myPaliers[0][menu] + " ? ";
        Ligne2 = " > T" + String(config.myTemperatures[0]) + degre + "C";
        break;
      }


    case 1:   // Sélection de températures des paliers 1 à 6 - inclus Mash-out et ébullition
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
      {
        paliers_temp(menu, &Ligne1, &Ligne2);
        //fin sous menu
        break;
      }


    case 7:
      { // Possibilité de ne pas sauvegarder sur carte SD nos règlages (pour quelque raison que ce soit)
        Ligne1 = "Sauvegarder ?   ";
        Ligne2 = "SEL = oui DROIT = no";
        break;
      }

    case 8:   // préchauffage
      {
        theta_objectif = config.myTemperatures[0];
        Ligne1 = config.myPaliers[1][0];
        Ligne2 = String((int)theta_mesure) + " / " + String((int)config.myTemperatures[0]) + degre + "C ";
        Ligne2 = Ligne2 PID_DEBUG("PID : " + String(tx_chauffe)); // Ne va s'ajouter que si on active la fonction de debug (désactivation : commenter #define DEBUG)
        break;
      }
    case 9:   // palier 1
    case 10:  // palier 2
    case 11:  // palier 3
    case 12:  // palier 4
    case 13:  // Mash-out
    case 14:  // Ebullition
      {

        display_palier(menu - 8, &Ligne1, &Ligne2);
        break;
      }

    case 15: // refroidissement
      {
        Ligne1 = "Refroidissement";
        theta_objectif = 0;
        config.myTempo[0] = (minutes - cooling);
        Ligne2 = "T : " + String(((int)theta_mesure)) + degre + " / " + String(config.myTempo[0]) + " min";
        break;
      }

    case 109:   // Demander si on veut recommencer le palier en cours
    case 110:
    case 111:
    case 112:
    case 113:
    case 114:
      {
        restart_palier(menu - 108, &Ligne1, &Ligne2);
        break;
      }

    case 115:   // Recommencer le refroidissement
      {
        Ligne1 = "RESTART REFROID ? ";
        Ligne2 = "SEL = OUI autre = NO";
        break;
      }

    case 209:   // Demander si on veut passer le palier en cours
    case 210:
    case 211:
    case 212:
    case 213:
    case 214:
      {
        pass_palier(menu - 208, &Ligne1, &Ligne2);
        break;
      }
    case 215:   // On a demander à passer le refroidissement : fin du brassage
      {
        Ligne1 = "FIN DU BRASSAGE ? ";
        Ligne2 = "SEL = OUI autre = NO";
        break;
      }
    default :
      {
        Ligne1 = "Erreur de menu";
        Ligne2 = "menu " + String(menu);
        break;
      }

  }

  // padding with nothing
  char Char1[21];
  char Char2[21];

  sprintf(Char1, "%-20s", Ligne1.c_str());
  sprintf(Char2, "%-20s", Ligne2.c_str());

  lcd.setCursor(0, 0);
  lcd.print(Char1);
  lcd.setCursor(0, 1);
  lcd.print(Char2);
  lcd.setCursor(0, 2);
  lcd.print("T1 : " + String(temp1) + " T2: " + String(temp2));
  lcd.setCursor(0, 3);
  lcd.print(String(RtcDate) + " " + String(RtcTime));


} //************************************************************************************************************************************************************************FIN LCD

// read the buttons
int read_LCD_buttons() //cette fonction renvoie la touche appuyée
{
  btnUPValue = digitalRead(btnUP);
  btnDOWNValue = digitalRead(btnDOWN);
  btnLEFTValue = digitalRead(btnLEFT);
  btnRIGHTValue = digitalRead(btnRIGHT);
  btnSELECTValue = digitalRead(btnSELECT);

  if (btnUPValue == LOW)   return btnUP;
  if (btnDOWNValue == LOW)  return btnDOWN;
  if (btnLEFTValue == LOW)  return btnLEFT;
  if (btnRIGHTValue == LOW)  return btnRIGHT;
  if (btnSELECTValue == LOW)  return btnSELECT;

  return btnNONE;  // when all others fail, return this...
} // fin read the buttons

float gettemp()
{
  float ttt;

#if defined(MAX31865)
  //uint16_t rtd = thermo.readRTD();

  //Serial.print("RTD value: "); Serial.println(rtd);
  //float ratio = rtd;
  //ratio /= 32768;
  //Serial.print("Ratio = "); Serial.println(ratio, 8);
  //Serial.print("Resistance = "); Serial.println(RREF * ratio, 8);
  //Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  //Serial.print("Fault 0x"); Serial.println(fault, HEX);
  if (fault) {

    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      DEBUGPRINTLN("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      DEBUGPRINTLN("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      DEBUGPRINTLN("REFIN - > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      DEBUGPRINTLN("REFIN - < 0.85 x Bias - FORCE - open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      DEBUGPRINTLN("RTDIN - < 0.85 x Bias - FORCE - open");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      DEBUGPRINTLN("Under / Over voltage");
    }
    thermo.clearFault();
  }
  else
  {
    temp1 = thermo.temperature(RNOMINAL, RREF);
    ttt = temp1;
  }
#endif

#if defined(MAX31865_2)
  //uint16_t rtd = thermo.readRTD();

  //Serial.print("RTD value: "); Serial.println(rtd);
  //float ratio = rtd;
  //ratio /= 32768;
  //Serial.print("Ratio = "); Serial.println(ratio, 8);
  //Serial.print("Resistance = "); Serial.println(RREF * ratio, 8);
  //Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault2 = thermo2.readFault();
  //Serial.print("Fault 0x"); Serial.println(fault, HEX);
  if (fault2) {

    if (fault2 & MAX31865_FAULT_HIGHTHRESH) {
      DEBUGPRINTLN("RTD High Threshold");
    }
    if (fault2 & MAX31865_FAULT_LOWTHRESH) {
      DEBUGPRINTLN("RTD Low Threshold");
    }
    if (fault2 & MAX31865_FAULT_REFINLOW) {
      DEBUGPRINTLN("REFIN - > 0.85 x Bias");
    }
    if (fault2 & MAX31865_FAULT_REFINHIGH) {
      DEBUGPRINTLN("REFIN - < 0.85 x Bias - FORCE - open");
    }
    if (fault2 & MAX31865_FAULT_RTDINLOW) {
      DEBUGPRINTLN("RTDIN - < 0.85 x Bias - FORCE - open");
    }
    if (fault2 & MAX31865_FAULT_OVUV) {
      DEBUGPRINTLN("Under / Over voltage");
    }
    thermo2.clearFault();
  }
  else
  {
    temp2 = thermo2.temperature(RNOMINAL, RREF);
    ttt = ( temp1 + temp2 ) / 2 ;
  }
#endif

  myPID.Compute(); //on Lance un calcul du PID. le calcul se fait en même temps que le rafraichissement des variables de température

  return ttt;
}

void CLS() // efface l'écran et place le curseur au début de la première ligne
{
  lcd.clear();
  lcd.setCursor(0, 0);
}

void horloge() //fonction appelée une fois par seconde . On profite de cette fonction pour certaines fonctionalités actualisées une fois par seconde
{

  now = rtc.now();

  sprintf(RtcTime, "%02dh%02dm%02d", now.hour(), now.minute(), now.second());
  sprintf(RtcDate, "%02d-%02d-%04d", now.day(), now.month(), now.year());

  secondes_reel++;

  if (palier_atteint) { // on incrémente le compteur minutes toutes les 60s, mais seulement si on a atteint le palier - permet d'éviter les erreurs de mesure de la sonde qui parfois affiche des valeurs farfelues :
    // si la lecture de la sonde de température donne quelque chose de négatif au moment où on rentre dans ce sous-programme, on pourrait ne pas incrémenter les minutes, et donc faire durer trop longtemps le palier

    secondes++; //on compte le temps écoulé pendant que le palier est en cours

    BLINK(); //on fait clignoter une fois par seconde pour montrer que le palier est lancé
  }

  if (theta_objectif == 0) //cas uniquement présent lors du refroidissement
  {
    secondes++;
  }

  if (secondes >= 60) {
    secondes = 0;

    // Si le menu correpsond à un palier en cours (sauf préchauffage), on incrémente les minutes
    if (menu >= 9) {
      minutes++;
      DEBUGPRINTLN(strcat("Minutes écoulées : : " , minutes));
    }
  }

  if (theta_mesure > (theta_objectif - config.hysteresis_Neg) && theta_mesure < (theta_objectif + config.hysteresis_Pos)) // si on est compris entre les deux valeurs d'hysteresis, on est dans la bonne plage de température. on réduit la force du PID
  {
    Kp = config.P_weak; // multiplicateur de l'erreur différentielle de température.
    Ki = config.I_weak; //coef de correction intégrale
    Kd = config.D_weak; //coef de dérivée

  }
  else {
    Kp = config.P_strong; // multiplicateur de l'erreur différentielle de température. 1° d'écart = Kp% de chauffe
    Ki = config.I_strong; //coef de correction intégrale
    Kd = config.D_strong; //coef de dérivée
  }
  myPID.SetTunings(Kp, Ki, Kd);
}

void lecture()
{
  theta_mesure = gettemp();
  if (theta_mesure > (theta_objectif - config.hysteresis_Neg) && theta_mesure < (theta_objectif + config.hysteresis_Pos)) {
    theta_PID = theta_mesure; // si on est proche de la température de consigne, on supprime l'offset de sécurité
    palier_atteint = true;    // on annonce que le palier est atteint : c'est lui qui va déterminer si on va incrémenter les minutes pour le palier en cours, pour éviter les erreurs de mesure de la sonde
  }
  else {
    // Si on n'est pas à l'ébullition...
    if (menu != 14) {
      theta_PID = theta_mesure + config.PID_OFFSET;
    } else {
      theta_PID = theta_mesure;
    }

  }
}

void start_brew() {
  digitalWrite(config.ledPin, LOW);
  myPID.SetMode(AUTOMATIC);

  //////////////////////////////////
  // on compte le temps que va prendre le programme
  startprog = 1;
  total_time = 0;
  for (int i = 1; i < 7; i++) {
    total_time += config.myTempo[i];
    DEBUGPRINTLN("Temps palier[" + String(i) + "] : " + String(config.myTempo[i]));
  }
  DEBUGPRINTLN("Temps total prévu : " + String(total_time));
  CLS();           // move to position 0 on the first line
  lcd.print("    BRASSAGE    ");
  lcd.setCursor(0, 1);           // move to position 0 on the second line
  lcd.print("    IMMINENT !  ");
  delay(800);
  CLS();
  lcd.print("Duree prevue :  ");
  lcd.setCursor(0, 1);           // move to position 0 on the second line
  lcd.print(total_time);
  lcd.print(" min");
  delay(1000);

  digitalWrite(config.ledPin, LOW);

  menu = 8; //saute au menu suivant (départ programme)
  CLS();
  LCD_upd();

  return;
}

// +++++++++++++++++++++++++++++++++++++++++       GESTION DES MENUS                   ++++++++++++++++++++++++++++++++++++++++++++++
void sel_menu()
{

  byte m;

  lcd_key = read_LCD_buttons();  // read the buttons

  if (menu >= 100) { // gestion des sauts de programme en cas d'annulation la valeur est >100  en cas de saut menu > 200
    switch (lcd_key)
    {
      case btnNONE:
        {
          break;
        }

      case btnSELECT:
        {
          switch (menu)
          {
            case 108:
              {
                CLS();
                lcd.print("saut erreur 108");
                menu = 8;
                delay(150);
                break;
              }
            case 109: //on a demandé à recommencer l'étape palier 1
            case 110: //on a demandé à recommencer l'étape palier 2
            case 111: //on a demandé à recommencer l'étape palier 3
            case 112: //on a demandé à recommencer l'étape palier 4
            case 113: //on a demandé à recommencer l'étape rincage mash out
            case 114: //on a demandé à recommencer l'étape ébu
            case 115: //on a demandé à recommencer l'étape refroidissement/whirlpool
              {
                m = menu - 108;
                CLS();
                annuler = -1; // on était au menu X (menu - 109) on retourne au X-1 (menu - 110)
                minutes = 0;  //avant le palier 1 (donc l'étape de préchauffage) le temps écoulé était à 0, et
                //avant le palier 2 (donc le palier 1) le temps écoulé était aussi égal à 0

                config.myTempo[0] = 0;


                for (int i = m; i >= 2; i--) { // on ne réinitialise les minutes que pour les paliers > 1
                  minutes += config.myTempo[(i - 1)];    // on rajoute les durées des paliers précédents uniquement, car on recommence au début du palier m
                }
                cooling = minutes;
                delay(150);
                palier_atteint = false;   // on ne sait jamais, on va prétendre que la température du palier n'a pas été atteinte
                break;
              }

            case 208: //on a demandé à sauter l'étape préchauffage (ne devrait pas arriver)
              {
                CLS();
                lcd.print("saut erreur 208");
                menu = 8;
                delay(1000);
                BeepON(50);
                LCD_upd();
                break;
              }

            case 209: //on a demandé à sauter l'étape palier 1
            case 210: //on a demandé à sauter l'étape palier 2
            case 211: //on a demandé à sauter l'étape palier 3
            case 212: //on a demandé à sauter l'étape palier 4
            case 213: //on a demandé à sauter l'étape mash out
            case 214: //on a demandé à sauter l'étape ebullition
              {
                m = menu - 208;
                minutes = 0;
                for (int i = m; i >= 1; i--) { // on ne réinitialise les minutes que pour les paliers > 1
                  minutes += config.myTempo[i];    // on rajoute les durées des paliers précédents et du palier en cours, car on va recommencer au début du palier m + 1
                }
                config.myTempo[0] = 0;
                jump = 0;
                annuler = 1;
                cooling = minutes; // sauter l'étape ébullition (cooling sera mis à jour de toute façon quand on sera à la fin de l'ébullition, pas de problème à l'écraser maintenant)
                delay(150);
                palier_atteint = false;   // on ne sait jamais, on va prétendre que la température du palier n'a pas été atteinte
                break;
              }
            case 215:   // on a fini le brassage : on demande à "sauter" le refroidissement/whirlpool pour finir dans un état permettant d'arrêter proprement l'automate
              {
                T.stop(logRecord_ID); // On arrête le timer lié à l'écriture sur la carte SD, pour éviter qu'une écriture impromptue ne soit en cours quand on débranche l'Arduino
                logRecord();  // On appelle pour la dernière fois l'écriture sur la carte SD, pour logger la fin du brassage (dernier événement du log)
                CLS();
                lcd.print("Fin de BRASSAGE");
                lcd.setCursor(0, 1);
                lcd.print("!!! HAVE FUN !!!");
                delay(800);
                while (1);  // Arrêt propre : plus aucune action ne sera réalisée par la suite (donc pas d'erreur d'écriture sur carte SD par exemple)
                break;
              }
              break;
          } //fin switch menu
        }//fin bouton select

      case btnUP: // on annule le saut
      case btnDOWN: // on annule le saut
      case btnLEFT: // on annule le saut
      case btnRIGHT: // on annule le saut
        {
          CLS();
          jump = 0;
          delay(500);
          if (menu > 199)
          {
            menu -= 200;
          }
          else {
            menu -= 100;
          }
          break;
        }
        break;
    }// fin switch lcd key
  } else { //Si on n'a pas demandé de saut

    // gestion des menus
    switch (lcd_key)               // depending on which button was pushed, we perform an action
    {

      case btnNONE:
        {
          break;
        }

      case btnUP:
        {

          if (startprog == 0) //si on est encore en phase de programation, on peut naviguer du menu 0 à 7
          {
            if (menu < 7)
            {
              menu++;

              delay(150);
              LCD_upd();
            }
          }
          else { //le programme est lancé
            if (menu > 7) // On demande à sauter l'étape en cours
            {


              if (menu >= 9 && menu < 16) //si on est au menu 9 ou + on considère que c'est un saut de programme (y inclus pour le refroidissement, pour la fin du programme)
              {
                // small_Beep = 1;
                BeepON(200);

                menu += 200;
              }
              if (menu == 8) menu ++; //si on est dans le menu 8 (préchauffe) , le saut est autorisé sans condition
              delay(150);
              LCD_upd();
            }

          }//fin else
          break;
        }
      case btnDOWN:
        {

          if (startprog == 0) //si on est encore en phase de programation, on peut naviguer du 7 à 0
          {
            if (menu > 0) {
              menu--;
              delay(150);
              LCD_upd();
            }
          }
          else {
            if (menu > 8) { //si on est en phase brassage (a partir de menu 9)

              // DEMANDER SI ON SOUHAITE RECOMMENCER L'ETAPE EN COURS
              // small_Beep = 1;
              BeepON(200);

              menu += 100;
              delay(150);
              LCD_upd();
            }

          }

          break;
        }
      case btnLEFT:
        {
          //faire un switch menu
          switch (menu)
          {
            case 0: //menu température de préchauffe
            case 1: //menu réglage palier 1
            case 2: //menu réglage palier 2
            case 3: //menu réglage palier 3
            case 4: //menu réglage palier 4
            case 5:  //menu réglage Mash out - rinçage
            case 6: // ébullition
              {
                if (submenu == 0) {
                  if (config.myTemperatures[menu] > 20) config.myTemperatures[menu] -= 0.5;
                }
                else {
                  if (config.myTempo[menu] > 0) config.myTempo[menu]--;
                }
                LCD_upd();
                break;
              }

            case 7: //sauvegarde
              {
                break;
              }
            case 8: //préchauffe
            case 9: //palier 1
            case 10://palier 2
            case 11://palier 3
            case 12://palier 4
            case 13://rinçage - mash-out
            case 14://ébullition
              {
                m = menu - 8;
                DEBUGPRINTLN("Valeur submenu pour le menu " + String(m) + " : " + String(submenu));
                if (submenu == 0) {
                  if (config.myTemperatures[m] > 20) config.myTemperatures[m] -= 0.5;
                }
                else {
                  if (config.myTempo[m] > 2) config.myTempo[m]--;
                }
                LCD_upd();
                break;
              }

            case 15://mode manuel
            case 16://mode erreur
              {
                break;
              }
          }
          break;
        }
      case btnRIGHT:
        {
          //faire un switch menu
          switch (menu)
          {
            case 0: //menu température de préchauffe
            case 1: //menu réglage palier 1
            case 2: //menu réglage palier 2
            case 3: //menu réglage palier 3
            case 4: //menu réglage palier 4
            case 5:  //menu réglage Mash out - rinçage
            case 6: // ébullition
              {
                if (submenu == 0) {
                  if (config.myTemperatures[menu] <= 110) config.myTemperatures[menu] += 0.5;
                }
                else {
                  config.myTempo[menu]++;
                }
                LCD_upd();
                break;

              }

            case 7: // On a demandé à ne pas faire la sauvegarde
              {
                start_brew();
                break;
              }
            case 8: //préchauffe
            case 9: //palier 1
            case 10://palier 2
            case 11://palier 3
            case 12://palier 4
            case 13://rinçage mash out
            case 14://ébullition
              {
                m = menu - 8;
                DEBUGPRINTLN("Valeur submenu pour le menu " + String(m) + " : " + String(submenu));
                if (submenu == 0) {
                  if (config.myTemperatures[m] < 110) config.myTemperatures[m] += 0.5;
                }
                else {
                  config.myTempo[m]++;
                }
                LCD_upd();
                break;
              }

            case 15://mode manuel
            case 16://mode erreur
              {
                break;
              }
            default:
              {
                CLS();
                lcd.print("Erreur de menu");
                lcd.setCursor(0, 1);           // move to position 0 on the second line
                lcd.print("menu ");
                lcd.print(menu);
                break;
              }
          }
          break;
        }
      case btnSELECT:
        {
          switch (menu)
          {

            case 1: // menu paliers 1 à 4
            case 2:
            case 3:
            case 4:
            case 6: // ébullition
              {
                submenu = !submenu;
                LCD_upd();
                break;
              }

            case 0: //menu température de préchauffe
            case 5: //menu rinçage
              {
                LCD_upd();
                break;
              }

            case 7: // sauvegarde
              {

                digitalWrite(config.ledPin, HIGH);

                saveConfiguration(filename, config);

                myFile = sd.open(datalogFile, O_WRITE | O_CREAT | O_AT_END);
                DEBUGPRINTLN(strcat("Write on " , datalogFile));
                if (myFile) {
                  myFile.print("----GraiN.Master----\n");
                  myFile.print(version);
                  myFile.println("\nDate / Heure : " + String(RtcDate) + " " + String(RtcTime));
                  myFile.print("\nTempérature de départ : ");
                  myFile.print(config.myTemperatures[0]);
                  myFile.print("°C\n");

                  for (int i = 1; i < 7; i++) {
                    myFile.print(config.myPaliers[0][i]);
                    myFile.print(config.myTempo[i]);
                    myFile.print(" min - ");
                    myFile.print(config.myTemperatures[i]);
                    myFile.print("°C\n");
                  }

                  myFile.print("\n\n");
                  myFile.print("Temps (s), Température cible °C, Température °C, PID (0 - 255)\n");
                  myFile.close();
                }
                else
                {
                  DEBUGPRINTLN(strcat("Error Writing info to Datalogfile = " , datalogFile));
                }

                DEBUGPRINTLN("----GraiN.Master----");
                DEBUGPRINTLN(version);
                DEBUGPRINTLN("Température de départ : " + String(config.myTemperatures[0]) + "°C"  );

#ifdef DEBUG
                for (int i = 1; i < 7; i++) {
                  DEBUGPRINTLN(config.myPaliers[0][i]);
                  DEBUGPRINTLN(String(config.myTempo[i]) + " min - " + String(config.myTemperatures[i]) + "°C");
                }
#endif
                lcd.home();           // move to position 0 on the first line
                lcd.print("    PROGRAMME   ");
                lcd.setCursor(0, 1);           // move to position 0 on the second line
                lcd.print("   SAUVEGARDE ! ");


                delay(600);
                start_brew();
                break;

              }
            case 8: //palier 1
            case 9: //palier 2
            case 10: //palier 3
            case 11://palier 3
            case 12://palier 4
            case 13://rinçage - mash-out
            case 14://ébullition
              {
                submenu = !submenu;   // Permet de modifier la durée ou la température du palier en cours
                break;
              }

            default:
              {
                CLS();
                lcd.print("Erreur de menu");
                lcd.setCursor(0, 1);           // move to position 0 on the second line
                lcd.print("menu ");
                lcd.print(menu);
                break;
              }

              break;
          } //fin switch menu
          break;
        } //fin buton select


      default:
        {
          CLS();
          lcd.print("Erreur de menu");
          lcd.setCursor(0, 1);           // move to position 0 on the second line
          lcd.print("menu ");
          lcd.print(menu);
          break;
        }

        break;
    } //fin switch lcd key
  } //fin du else (pas de saut)
}

void regle_chauffe()
{

  if (tx_chauffe >= 250)
  {
    // Si on est dans un des paliers de chauffe, on veut une courbe de température douce, qui gagne environ 1° toutes les minutes (pour éviter de violenter notre moût), le TRIAC à fond va beaucoup plus fort
    // On ne modifie pas tx_chauffe pour éviter tout effet de bord
    // Ce n'est valide que si le palier n'a pas été atteint. Si ce n'est pas le cas, c'est qu'on a eu une brusque chute de température lors du palier et qu'il faut au plus vite revenir à notre température !
    if (menu >= 9 && menu <= 13 && !palier_atteint) {  // menu 8 = préchauffage, menu 9 = palier 1, menu 13 = mash-out, menu 14 = ébullition
      PROPORTIONNAL();
      Triac_pilot(Power_out(220));
    }
    // Si on est autre part (préchauffage ou ébullition), on y va à fond
    else {
      FULL_ON();
    }
  }
  else {

    if (tx_chauffe <= 5)
    {
      FULL_OFF();
    }
    else
    {
      PROPORTIONNAL();
      Triac_pilot(Power_out(tx_chauffe));
    }
  }

}
//Interrupt Service Routines

void zeroCrossingInterrupt() { //zero cross detect
  TCCR1B = 0x04; //start timer with divide by 256 input
  TCNT1 = 0;   //reset timer - count from zero
}

ISR(TIMER1_COMPA_vect) { //comparator match
  digitalWrite(GATE, HIGH); //set triac gate to high
  TCNT1 = 65536 - PULSE;    //trigger pulse width
}

ISR(TIMER1_OVF_vect) { //timer1 overflow
  digitalWrite(GATE, LOW); //turn off triac gate
  TCCR1B = 0x00;          //disable timer stopd unintended triggers
}

void Triac_pilot(int conduction_time) { // pilote le triac
  if (conduction_time > HALF_WAVE) conduction_time = HALF_WAVE;
  OCR1A = conduction_time;     //set the compare register brightness desired.
}


int Power_out(int Power) //calcule le temps de conduction nécessaire pour obtenir la puissance demandée. Power va de 0 à 255
{

  int CT;
  CT = 540 - (Power * 2);
  return CT;
}
void FULL_ON()
{
  // set up zero crossing interrupt
  detachInterrupt(digitalPinToInterrupt(2));  // équivalent à 0
  digitalWrite(GATE, HIGH);
  //IRQ0 is pin 2
}
void FULL_OFF()
{
  // set up zero crossing interrupt
  detachInterrupt(digitalPinToInterrupt(2));  // équivalent à 0
  digitalWrite(GATE, LOW);
  //IRQ0 is pin 2
}

void PROPORTIONNAL()
{
  attachInterrupt(digitalPinToInterrupt(2), zeroCrossingInterrupt, CHANGE);
}

void logRecord()
{
  byte numpalier = 0;
#ifdef DEFIL
  defilement = !defilement;    // Position du défilement / 1 = température et température à atteindre / 0 = température et temps restant / s'échange à chaque écriture sur la carte SD, donc toutes les 10 s environ
#endif
  if (menu > 200)
    numpalier = menu - 208;
  else if (menu > 100)
    numpalier = menu - 108;
  else if (menu <= 7)
    numpalier = 0;
  else numpalier = menu - 8;
  digitalWrite(config.ledPin, 1);
  myFile = sd.open(datalogFile, O_WRITE | O_CREAT | O_AT_END);
  if (myFile) {
    myFile.println("\nDate / Heure : " + String(RtcDate) + " " + String(RtcTime));
    myFile.print(secondes_reel);
    myFile.print("; ");
    if (numpalier != 7)
      myFile.print(config.myPaliers[0][numpalier]);
    else myFile.print("Refroidissement");
    myFile.print("; ");
    myFile.print(theta_objectif);
    myFile.print("; ");
    myFile.print(theta_mesure);
    myFile.print("; ");
    myFile.print(tx_chauffe);
    myFile.print('\n');
    myFile.close();
  }
  else
  {
    DEBUGPRINTLN(strcat("Error Writing info to Datalogfile = ", datalogFile));
  }

  digitalWrite(config.ledPin, 0);
  DEBUGPRINTLN("Secondes réelles depuis le début : " + String(secondes_reel));
  DEBUGPRINTLN("Theta_Objectif : " + String(theta_objectif));
  DEBUGPRINTLN("Theta_mesure : " + String(theta_mesure));
  DEBUGPRINTLN("Taux chauffe (PID) : " + String(tx_chauffe));
  //CLS();                      // On ne refresh l'écran que toutes les 10 secondes, pour éviter l'effet stroboscopique

}




// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config) {
  // Open file for reading
  myFile = sd.open(filename);

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<4096> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, myFile);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  config.hysteresis_Pos = doc["hysteresis_Pos"];
  config.hysteresis_Neg = doc["hysteresis_Neg"];
  copyArray( doc["myPaliers"] , config.myPaliers );
  copyArray( doc["myTemperatures"], config.myTemperatures );
  copyArray( doc["myTempo"], config.myTempo );
  config.P_strong =  doc["P_strong"];
  config.I_strong =  doc["I_strong"];
  config.D_strong =  doc["D_strong"];
  config.P_weak = doc["P_weak"];
  config.I_weak = doc["I_weak"];
  config.D_weak = doc["D_weak"];
  config.PID_OFFSET = doc["PID_OFFSET"];
  config.Beep_PIN = doc["Beep_PIN"];
  config.ledPin = doc["ledPin"];

  // Close the file (Curiously, File's destructor doesn't close the file)
  //myFile.close();
}

// Saves the configuration to a file
void saveConfiguration(const char *filename, const Config &config) {
  // Delete existing file, otherwise the configuration is appended to the file
  sd.remove(filename);

  // Open file for writing
  File myFile = sd.open(filename, O_WRITE | O_CREAT | O_AT_END);
  if (!myFile) {
    Serial.println("Failed to create file");
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<4096> doc;
  Serial.println("Allocated");
  // Set the values in the document
  doc["hysteresis_Pos"] = config.hysteresis_Pos;
  doc["hysteresis_Neg"] = config.hysteresis_Neg;
  copyArray( config.myPaliers, doc["myPaliers"] );
  copyArray( config.myTemperatures, doc["myTemperatures"] );
  copyArray( config.myTempo, doc["myTempo"] );
  doc["P_strong"] = config.P_strong;
  doc["I_strong"] = config.I_strong;
  doc["D_strong"] = config.D_strong;
  doc["P_weak"] = config.P_weak;
  doc["I_weak"] = config.I_weak;
  doc["D_weak"] = config.D_weak;
  doc["PID_OFFSET"] = config.PID_OFFSET;
  doc["Beep_PIN"] = config.Beep_PIN;
  doc["ledPin"] = config.ledPin;

Serial.println("Assigned");

  // Serialize JSON to file
  if (serializeJson(doc, myFile) == 0) {
    Serial.println("Failed to write to file");
  }

  // Close the file
  Serial.println("Close file");
  //myFile.close();
  Serial.println("Closed file");
}

// Prints the content of a file to the Serial
void printFile(const char *filename) {
  // Open file for reading
  myFile = sd.open(filename);
  if (!myFile) {
    Serial.println("Failed to read file");
    return;
  }

  // Extract each characters by one by one
  while (myFile.available()) {
    Serial.print((char)myFile.read());
  }
  Serial.println();

  // Close the file
  myFile.close();
}
