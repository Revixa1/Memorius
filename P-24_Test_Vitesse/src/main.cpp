#include <Arduino.h>
#include <librobus.h>
#include <math.h>
#include "LCD.h"

#define CIRCONFERENCE_ENTRE_2_ROUES (18.60000000000000000000000000 * PI) // la circonférence en cm du cercle fait par les deux roues lors d'une rotation de 1 tour
#define PULSE_PAR_CM (3200.0000 / 23.9389000000000000000000000000)         // nombre de pulse par tour/circonférence d'une roue en cm
#define TOUR_DEGRE 360.0000                          // nombre de degré dans 1 tour

void setup() 
{
  lcd.begin(16,2);
  lcd.init();
  lcd.backlight();
  //Creation de caractere
  lcd.createChar(0,eyes);
  lcd.createChar(1,bouche_centre);
  lcd.createChar(2,bouche_niveau1_gauche);
  lcd.createChar(3,bouche_niveau2_gauche);
  lcd.createChar(4,bouche_niveau3_gauche);
  lcd.createChar(5,bouche_niveau1_droit);
  lcd.createChar(6,bouche_niveau2_droit);
  lcd.createChar(7,bouche_niveau3_droit);
  lcd.home();
}

void loop() 
{
  char texte1[]="Memorius";
  char texte2[]="Equipe P24";
  PrintLCD(0,texte1);
  PrintLCD(1,texte2);
  delay(5000);
  Afficher_yeux_bouche();
  delay(5000);
}