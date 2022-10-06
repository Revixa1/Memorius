#include <Arduino.h>
#include <librobus.h>

int LS_Arriere=28;

float encodeur_gauche_compteur=0;
float encodeur_droit_compteur=0;
float Temps_delais=1000.00;
float vitesse_TourParSeconde=0.00;
float vitesse_MetreParSeconde=0.0000;
int nbrintervale=20;
int distance_parcourir=29000;

void setup() {
  // put your setup code here,o run once:
  BoardInit();
  pinMode(LS_Arriere, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(LS_Arriere)==HIGH){
   // for(i=0;i<=nbrintervale;i)
    
    
    
    MOTOR_SetSpeed(0,0.5);
    MOTOR_SetSpeed(1,0.5);
    delay(Temps_delais);
    
    /*
     if(ENCODER_Read(0)>=29000){
      MOTOR_SetSpeed(0,0);
      ENCODER_ReadReset(0);
    }

    if(ENCODER_Read(1)>=29000){
      MOTOR_SetSpeed(1,0);
      ENCODER_ReadReset(1);
    }
  */
    

    
    encodeur_gauche_compteur=ENCODER_ReadReset(0);
    encodeur_droit_compteur=ENCODER_ReadReset(1);

    vitesse_TourParSeconde=(encodeur_droit_compteur/3200.00)/(Temps_delais/1000.00);
    vitesse_MetreParSeconde=vitesse_TourParSeconde*0.2394;

    Serial.print("PulseGauche= ");
    Serial.print(encodeur_gauche_compteur);
    Serial.print(" | PulseDroit= ");
    Serial.print(encodeur_droit_compteur);
    Serial.print(" | Vitesse: ");
    Serial.print("ToursParSecondes= ");
    Serial.print(vitesse_TourParSeconde);
    Serial.print(" MetresParSecondes= ");
    Serial.println(vitesse_MetreParSeconde);
    
  }
 
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
  
}