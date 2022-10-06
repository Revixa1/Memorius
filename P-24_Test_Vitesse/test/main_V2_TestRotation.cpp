#include <Arduino.h>
#include <librobus.h>

int LS_Arriere=28;

float encodeur_gauche_compteur=0;
float encodeur_droit_compteur=0;
float Temps_delais=1000.00;
float vitesse_TourParSeconde=0.00;
float vitesse_MetreParSeconde=0.0000;
int nbrintervale=30;
int distance_parcourir=1974;
int a=0;
bool ok_gauche=0;
bool ok_droit=0;

void setup() {
  // put your setup code here,o run once:
  BoardInit();
  pinMode(LS_Arriere, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(LS_Arriere)==HIGH){
    for(int i=1;i<=nbrintervale;i=i+a){
    a=0;
    if(-1*ENCODER_Read(0)<=(distance_parcourir/nbrintervale)){
      MOTOR_SetSpeed(0,-0.1);
      ok_gauche=0;
    }

      if(ENCODER_Read(1)<=(distance_parcourir/nbrintervale)){
      MOTOR_SetSpeed(1,0.1);
      ok_droit=0;
      }
     
        
     if(-1*ENCODER_Read(0)>=distance_parcourir/nbrintervale){
      MOTOR_SetSpeed(0,0);
      
      ok_gauche=1;
    }

    if(ENCODER_Read(1)>=(distance_parcourir/nbrintervale)){
      MOTOR_SetSpeed(1,0);
      
      ok_droit=1;
    }
  
    if(ok_droit==1 and ok_gauche==1){
    ENCODER_ReadReset(0);
    ENCODER_ReadReset(1);
    a=1;
    }
    }
  }
 
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
}