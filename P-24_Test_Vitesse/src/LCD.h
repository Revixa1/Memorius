#include <Arduino.h>
#include <wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);//dimension ecran lcd

/*Definition caractere*/
byte eyes[8]=
{
  B11111,
  B10001,
  B10001,
  B10101,
  B10101,
  B10001,
  B10001,
  B11111
};

byte bouche_centre[8]=
{
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111
};

byte bouche_niveau1_gauche[8]=
{
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11100,
  B00111,
  B00000
};

byte bouche_niveau2_gauche[8]=
{
  B00000,
  B00000,
  B00000,
  B11100,
  B00111,
  B00000,
  B00000,
  B00000
};

byte bouche_niveau3_gauche[8]=
{
  B10000,
  B11100,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

byte bouche_niveau1_droit[8]=
{
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00111,
  B11100,
  B00000
};

byte bouche_niveau2_droit[8]=
{
  B00000,
  B00000,
  B00000,
  B00111,
  B11100,
  B00000,
  B00000,
  B00000
};

byte bouche_niveau3_droit[8]=
{
  B00001,
  B00111,
  B11100,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

/*Declaration fonctions*/
int Longueur_chaine(char texte[]);
void PrintLCD(int ligne, char texte[]);//ligne 0 ou 1 seulement
void Afficher_yeux_bouche(void);

void setup() 
{
  lcd.begin(16,2);
  lcd.init();
  lcd.backlight();
  /*Creation de caractere*/
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

/*Fonctions*/
void PrintLCD(int ligne, char texte[])//ligne 0 ou 1 seulement
{
  int Curseur=0;//position curseur

  if(Longueur_chaine(texte)<16)//Si chaine plus petit que 16 caractere
  {
    Curseur=(16-(Longueur_chaine(texte)))/2;
  }
  else //Si chaine 16 caractere ou plus
  {
    Curseur=0;
  }

  lcd.setCursor(Curseur,ligne);
  lcd.print(texte);
}

int Longueur_chaine(char texte[])
{
  int Longueur_chaine=0;

  while(texte[Longueur_chaine]!='\0')
  {
    Longueur_chaine=Longueur_chaine+1;
  }
  Longueur_chaine=Longueur_chaine-1; //longueur chaine a partir de 0
  return Longueur_chaine;
}

void Afficher_yeux_bouche(void)
{
  lcd.clear();
  lcd.setCursor(6,0);
  lcd.write(0);//print yeux
  lcd.setCursor(9,0);
  lcd.write(0);//print yeux
  lcd.setCursor(4,1);
  lcd.write(4);//print bouche_niveau3_gauche
  lcd.setCursor(5,1);
  lcd.write(3);//print bouche_niveau2_gauche
  lcd.setCursor(6,1);
  lcd.write(2);//print bouche_niveau1_gauche
  lcd.setCursor(7,1);
  lcd.write(1);//print bouche_centre
  lcd.setCursor(8,1);
  lcd.write(1);//print bouche_centre
  lcd.setCursor(9,1);
  lcd.write(5);//print bouche_niveau1_droit
  lcd.setCursor(10,1);
  lcd.write(6);//print bouche_niveau2_droit
  lcd.setCursor(11,1);
  lcd.write(7);//print bouche_niveau3_droit
}