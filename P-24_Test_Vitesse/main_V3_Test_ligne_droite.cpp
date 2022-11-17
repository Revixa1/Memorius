#include <Arduino.h>
#include <librobus.h>

int LS_Arriere=28;

float encodeur_gauche_compteur=0;
float encodeur_droit_compteur=0;
float Temps_delais=1000.00;
float vitesse_TourParSeconde=0.00;
float vitesse_MetreParSeconde=0.0000;
int nbrintervale=1000;
int distance_parcourir=29000;
int a=0;
bool ok_gauche=0;
bool ok_droit=0;

void setup() {
  // put your setup code here,o run once:
  BoardInit();
  pinMode(LS_Arriere, INPUT);
}

void loop() {

}