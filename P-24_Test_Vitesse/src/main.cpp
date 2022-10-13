#include <Arduino.h>
#include <librobus.h>
#include <math.h>

#define CIRCONFERENCE_ENTRE_2_ROUES (18.70000000000000000000000000 * PI) // la circonférence en cm du cercle fait par les deux roues lors d'une rotation de 1 tour
#define PULSE_PAR_CM (3200.0000 / 23.9389000000000000000000000000)         // nombre de pulse par tour/circonférence d'une roue en cm
#define TOUR_DEGRE 360.0000                          // nombre de degré dans 1 tour

#define LINE 1
#define ANGLE 0
#define DIM_M 27
#define DIM_N 2

// ================================================================================================================================
//                                                  Constantes et variables de distance
// ================================================================================================================================

#define TRACK_WIDTH 18.7              // Distance [cm] entre les deux roues (24A: 18.7 cm, 24B: 18.6 cm)
#define WHEEL_DIA_IN 3                // Diamètre [po] des roues
#define NB_PULSES_PER_REV_WHEEL 3200  // Nombre de pulses envoyés par les encodeurs pour une révolution des roues
#define IN_TO_CM 2.54                 // Constante de conversion: pouces -> cm

#define COURSE_WIDTH 45               // Largeur constante du parcours [cm]

/* Définition des termes calculés concernant les roues */
#define CIRC_WHEEL_CM (PI * WHEEL_DIA_IN * IN_TO_CM)              // Circonférence [cm] des roues
#define DIST_PER_PULSE (CIRC_WHEEL_CM / NB_PULSES_PER_REV_WHEEL)  // Distance parcourue [cm] pour un pulse de l'encodeur

int LS_Arriere = 28; // pin de la limite switch arrière

long int dist2PulsesLin(int type, float value);
void deplacement(float prevAngle, float distance, float nextAngle, float angle); // prototype de la fonction de déplacement

const float MATRICE_P[DIM_M][DIM_N] = 
  {
    {LINE,100},
    {ANGLE,-90},
    {LINE,55},
    {ANGLE,90},
    {LINE,2},
    {ANGLE,90},
    {LINE,10-3},
    {ANGLE,-90-4},
    {LINE,165},
    {ANGLE,90},
    {LINE,0.05},
    {ANGLE,-90+1},
    {LINE,100+4},
    {ANGLE,180},
    {LINE,100+4},
    {ANGLE,90},
    {LINE,0.05},
    {ANGLE,-90-1},
    {LINE,165},
    {ANGLE,90},
    {LINE,10-3},
    {ANGLE,-90},
    {LINE,2},
    {ANGLE,-90},
    {LINE, 55},
    {ANGLE, 90},
    {LINE,100+4}
  };

void setup()
{

  BoardInit();                // fonction d'initialisation de librobus
  pinMode(LS_Arriere, INPUT); // activer la pin de la limite switch arrière en tant qu'entrée
  Serial.begin(115200);
}

void loop()
{
  // Défi du parcours
  if(ROBUS_IsBumper(3)) // Condition de départ
  {
    delay(200);
    for(int m=0;m<DIM_M;m++)
    {
      if(MATRICE_P[m][0] == LINE)  // Translation
      {
        if(m == 0)  // Départ
        {
          deplacement(0, MATRICE_P[0][1], MATRICE_P[1][1], 0);
        }
        else if(m == DIM_M - 1) // Arrivée
        {
          deplacement(MATRICE_P[m-1][1], MATRICE_P[m][1], 0, 0);
        }
        else  // Toutes les autres étapes
        {
          deplacement(MATRICE_P[m-1][1], MATRICE_P[m][1], MATRICE_P[m+1][1], 0);
        }
      }
      else  // Rotation
      {
        deplacement(0, 0, 0, MATRICE_P[m][1]);
      }
    }
  }
}

long int dist2PulsesLin(int type, float value)
{
  long int dist2Pulses;
  if(type == LINE)  // Pour les distances linéaires mesurées (lignes)
  {
    dist2Pulses = floor(abs(value)*PULSE_PAR_CM + 0.5);
  }
  else if(type == ANGLE)  // Pour les distances linéaires à parcourir dans les coins du parcours
  {
    if(value == 180)
    {
      dist2Pulses = 0;
    }
    else
    {
      dist2Pulses = floor(COURSE_WIDTH/2*tan(abs(value)*DEG_TO_RAD/2)*PULSE_PAR_CM + 0.5);
    }
  }
 // Serial.print("Distance ligne calculée: "); Serial.println(dist2Pulses);
  return dist2Pulses;
}

void deplacement(float prevAngle, float distance, float nextAngle, float angle)
{ // déclaration de la fonction de déplacement

  const float distance_en_pulse = dist2PulsesLin(ANGLE, prevAngle) + dist2PulsesLin(LINE, distance) + dist2PulsesLin(ANGLE, nextAngle);  //transformation de la distance en cm vers une distance en pulses
  const int sens_rotation = angle / abs(angle);//variable pour déterminer le sens de rotation
  const int arc_en_pulse = (abs(angle) / TOUR_DEGRE) * CIRCONFERENCE_ENTRE_2_ROUES * PULSE_PAR_CM; //transformation de l'angle degrés vers la distance d'un arc en pulse/*
  /*
  Serial.print("Sensrotation  ");
  Serial.print(sens_rotation);
  Serial.print("  arc_en_pulse  ");
  Serial.print(arc_en_pulse);
  */
  
  float vitesse = 0.25; //vitesse lors de la rotation
  float acceleration=vitesse;//variable pour l'accélération lors d'une ligne droite
  float somme_pulse_gauche = 0;//déclaration + reset de la distance parcourue par le moteur gauche
  int somme_pulse_droit=0;
  //int Temps_de_delais = 50;
  int encodeur_voulu = 0; //déclaration + reset du SetPoint du PID
  int encodeur_actuel=0;  //déclaration + reset du ProcessValue du PID
  int erreur_KP = 0;      //déclaration + reset de l'erreur proportionnel du PID
  int erreur_KI = 0;
  //static int erreur_KI_rot_gauche = 0;
 // static int erreur_KI_rot_droit = 0;      //déclaration + reset de l'erreur intégrale du PID
  float KP = 0.00001;       //Valeur pour tunner le gain proportionnelle du PID
  float KI = 0.0000005;//0.0005;           //Valeur pour tunner le gain de l'intégrale du PID
  float ajout_de_vitesse = 0; //déclaration + reset de la réponse du PID
  int cycle = 1;          //déclaration + reset du nombre de cycles du PID
  int briser=0;
  int briser_droit=0;
  int briser_gauche=0;

  if (angle != 0)
  {
    ENCODER_Reset(0); //on reset les encodeurs avant de bouger
    ENCODER_Reset(1);
    MOTOR_SetSpeed(0, sens_rotation * vitesse);//on démare les moteur à la vitesse de départ et dans le sens de rotation approprié. Angle - = vitesse roue gauche est négative
    MOTOR_SetSpeed(1, -1 * sens_rotation * vitesse);//Angle positif = vitesse roue droite est négative

    if(sens_rotation>0){//rotation vers la droite
      while (somme_pulse_droit <= arc_en_pulse && somme_pulse_gauche <= arc_en_pulse)//tant que la distance de l'arc n'est pas atteinte, on continue la rotation vers la droite
    {
      MOTOR_SetSpeed(0, sens_rotation * (vitesse + ajout_de_vitesse));//on change la vitesse du moteur gauche selon la réponse du PID
    //  Serial.println(sens_rotation * (vitesse + ajout_de_vitesse));
      while(50>ENCODER_Read(1) && 50>-1*ENCODER_Read(1) && briser<arc_en_pulse && briser_gauche<arc_en_pulse){briser= -1*ENCODER_Read(1)+somme_pulse_droit;briser_gauche= ENCODER_Read(0)+somme_pulse_gauche;}//on attend au moins 100 pulse du moteur droit avant d'entrer dans le PID

      //Section PID
      encodeur_voulu = (-1*sens_rotation*ENCODER_ReadReset(1));//lire la distance parcourue par la roue droite depuis le dernier reset. La valeur sera toujour positive
      encodeur_actuel =  sens_rotation*ENCODER_ReadReset(0);//lire la distance parcourue par la roue gauche depuis le dernier reset. La valeur sera toujour positive
      erreur_KP = encodeur_voulu - encodeur_actuel;//on calcule l'erreur proportionnelle
      somme_pulse_droit= somme_pulse_droit + encodeur_voulu;
      somme_pulse_gauche = somme_pulse_gauche + encodeur_actuel;//on trouve notre distance complète depuis le début du déplacement
      erreur_KI += somme_pulse_droit - somme_pulse_gauche;//  (cycle * encodeur_voulu)on calcule l'erreur de l'intégralle entre les deux courbes
      ajout_de_vitesse = KI * erreur_KI + KP * erreur_KP;//on calcule la réponse de la boucle PID. ICI nous avons seulment une réponse qui travaile sur le gain proportionnel et sur l'intégralle du procédé
      cycle++;//on incrémente le cycle
     
      
      Serial.print("pulse_gauche_rotation:");
      Serial.print(encodeur_actuel);
      Serial.print(",");
      Serial.print("pulse_droit_rotation:");
      Serial.println(encodeur_voulu);
      
      //Serial.print(" somme_pulse_gauche_rotation: ");
      //Serial.println(somme_pulse_gauche);
      /*
      Serial.print("  arc_en_pulse  ");
  Serial.print(arc_en_pulse);
  Serial.print(",");
  Serial.print("  somme_gauche:  ");
  Serial.print(somme_pulse_gauche);
  Serial.print(",");
  Serial.print("  somme_droit:  ");
  Serial.print(somme_pulse_droit);
  Serial.println(",");
      */
      
    }

    }
    else{//rotation vers la gauche
    while (somme_pulse_droit <= arc_en_pulse && somme_pulse_gauche <= arc_en_pulse)//tant que la distance de l'arc n'est pas atteinte, on continue la rotation vers la gauche
    {
      MOTOR_SetSpeed(0, sens_rotation * (vitesse + ajout_de_vitesse));//on change la vitesse du moteur gauche selon la réponse du PID
     // Serial.println(sens_rotation * (vitesse + ajout_de_vitesse));
      while(50>ENCODER_Read(1) && 50>-1*ENCODER_Read(1) && briser_droit<arc_en_pulse && briser_gauche<arc_en_pulse){briser_droit= ENCODER_Read(1)+somme_pulse_droit;briser_gauche= ENCODER_Read(0)+somme_pulse_gauche;}//on attend au moins 100 pulse du moteur droit avant d'entrer dans le PID

      //Section PID
      encodeur_voulu = (-1*sens_rotation*ENCODER_ReadReset(1));//lire la distance parcourue par la roue droite depuis le dernier reset. La valeur sera toujour positive
      encodeur_actuel =  sens_rotation*ENCODER_ReadReset(0);//lire la distance parcourue par la roue gauche depuis le dernier reset. La valeur sera toujour positive
      erreur_KP = encodeur_voulu - encodeur_actuel;//on calcule l'erreur proportionnelle
      somme_pulse_droit= somme_pulse_droit + encodeur_voulu;
      somme_pulse_gauche = somme_pulse_gauche + encodeur_actuel;//on trouve notre distance complète depuis le début du déplacement
      erreur_KI += somme_pulse_droit - somme_pulse_gauche;//  (cycle * encodeur_voulu)on calcule l'erreur de l'intégralle entre les deux courbes
      ajout_de_vitesse = KI * erreur_KI + KP * erreur_KP;//on calcule la réponse de la boucle PID. ICI nous avons seulment une réponse qui travaile sur le gain proportionnel et sur l'intégralle du procédé
      cycle++;//on incrémente le cycle
     
      
      Serial.print("pulse_gauche_rotation:");
      Serial.print(encodeur_actuel);
      Serial.print(",");
      Serial.print("pulse_droit_rotation:");
      Serial.println(encodeur_voulu);
      
      //Serial.print(" somme_pulse_gauche_rotation: ");
      //Serial.println(somme_pulse_gauche);
/*
      Serial.print("  arc_en_pulse  ");
  Serial.print(arc_en_pulse);
  Serial.print(",");
  Serial.print("  somme_gauche:  ");
  Serial.print(somme_pulse_gauche);
  Serial.print(",");
  Serial.print("  somme_droit:  ");
  Serial.print(somme_pulse_droit);
  Serial.println(",");
      */
      
    }
    }
    MOTOR_SetSpeed(0, 0);//on arrête les moteur une fois arrivé
    MOTOR_SetSpeed(1, 0);
    delay(70);//un délais pour une transition "smooth" (peu être changé)
  }
  /*
  Serial.print("  arc_en_pulse  ");
  Serial.print(arc_en_pulse);
 // Serial.print(",");
  Serial.print("  somme_gauche:  ");
  Serial.print(somme_pulse_gauche);
  //Serial.print(",");
  Serial.print("  somme_droit:  ");
  Serial.print(somme_pulse_droit);
  //Serial.println(",");
  */
  /*
  Serial.print("  Diff_g=  ");
  Serial.print(arc_en_pulse-somme_pulse_gauche);
  Serial.print("  Diff_d=  ");
  Serial.print(arc_en_pulse-somme_pulse_droit);
  Serial.print("  Diff_d_g=  ");
  Serial.println(somme_pulse_droit-somme_pulse_gauche);
  */
  vitesse = 0.2; // vitesse de départ lors d'une ligne droite
  acceleration=vitesse; //variable pour l'accélération lors d'une ligne droite
  somme_pulse_gauche = 0; //reset de la distance parcourue par le moteur gauche
  somme_pulse_droit=0;
  //Temps_de_delais = 50;
  encodeur_voulu = 0; //reset du SetPoint du PID
  encodeur_actuel=0; //reset du ProcessValue du PID
  erreur_KP = 0; //reset de l'erreur proportionnel du PID
  erreur_KI = 0; //reset de l'erreur intégrale du PID
  KP = 0.0000001; //Valeur pour tunner le gain proportionnelle du PID
  KI = 0.0005; //Valeur pour tunner le gain de l'intégrale du PID(0.00000000001)
  ajout_de_vitesse = 0; //reset de la réponse du PID
  cycle = 1; //reset du nombre de cycles du PID
  briser=0;
  if (distance != 0) //si une distance est spécifié faire un mouvement en ligne droite de cette distance
  {
    ENCODER_Reset(0); //on reset les encodeurs avant de bouger
    ENCODER_Reset(1);
    MOTOR_SetSpeed(0, vitesse);//on démare les moteur à la vitesse de départ
    MOTOR_SetSpeed(1, vitesse);
    

    while (somme_pulse_droit <= distance_en_pulse)//tant que la distance n'est pas atteinte, faire un déplacement vers l'avant
    {
      MOTOR_SetSpeed(0, acceleration + ajout_de_vitesse);//on change la vitesse des moteurs selon une courbe d'accéleration écrit plus bas ET on change la vitesse du moteur gauche selon la réponse du PID
      MOTOR_SetSpeed(1, acceleration);

      while(100>ENCODER_Read(1) && briser<distance_en_pulse){briser= ENCODER_Read(1)+somme_pulse_droit;}//on attend au moins 100 pulses avant de passer dans le PID
    
      //Section PID
      encodeur_voulu = ENCODER_ReadReset(1);//on lit notre SetPoint et on reset le compteur de son encodeur
      encodeur_actuel= ENCODER_ReadReset(0);//on lit notre ProcessValue et on reset le compteur de son encodeur
      erreur_KP = encodeur_voulu - encodeur_actuel;//on calcule l'erreur proportionnelle
      somme_pulse_droit= somme_pulse_droit + encodeur_voulu;
      somme_pulse_gauche = somme_pulse_gauche + encodeur_actuel;//on trouve notre distance complète depuis le début du déplacement
      erreur_KI = (cycle * encodeur_voulu) - somme_pulse_gauche;//on calcule l'erreur de l'intégralle entre les deux courbes
      ajout_de_vitesse = KI * erreur_KI + KP * erreur_KP;//on calcule la réponse de la boucle PID. ICI nous avons seulment une réponse qui travaile sur le gain proportionnel et sur l'intégralle du procédé
      cycle++;//on incrémente le cycle
      
      /*
      Serial.print("pulse_gauche_rotation:");
      Serial.print((encodeur_actuel));
      Serial.print(",");
      Serial.print("pulse_droit_rotation:");
      Serial.println(encodeur_voulu);
      */
     // Serial.print(",");
      
      //Section accélération
       if(somme_pulse_gauche<=distance_en_pulse/3 && acceleration<0.8)//pour le premier tier de la distance, accélérer avec une pente constante
      {
        acceleration+=0.01; //acceleration=0.65*(somme_pulse_gauche/(distance_en_pulse/3))+0.2
      }
       if((somme_pulse_gauche>distance_en_pulse/3 && somme_pulse_gauche<2*distance_en_pulse/3))//entre le 1/3 et le 2/3 de la distance garder une vitesse constante
      {
      
      }
       if((somme_pulse_gauche/100)>(distance_en_pulse/100)-((acceleration-0.2)/0.020) && acceleration>0.15)//pour le dernier tier de la distance, décélérer avec une pente constante
      {
       acceleration-=0.020; //acceleration-=1*(((distance_en_pulse/100)-(somme_pulse_gauche/100))*((distance_en_pulse/100)-(somme_pulse_gauche/100))); // acceleration=-1.2*((somme_pulse_gauche-(2*distance_en_pulse/3))/(2*distance_en_pulse/3))+acceleration
      }
      /*
      Serial.print("somme_pulse_gauche ");
      Serial.print(somme_pulse_gauche);
      Serial.print(",");
      Serial.print("somme_pulse_droit ");
      Serial.print(somme_pulse_droit);
      Serial.print(",");
      Serial.print(" distance_en_pulse  ");
      Serial.println(distance_en_pulse);
      //Serial.print(",");
      */
      /*
      Serial.print("  acceleration  ");
      Serial.println(acceleration);
      */
    }
    MOTOR_SetSpeed(0, 0); //une fois le trajet complèté arrêter les moteurs
    MOTOR_SetSpeed(1, 0);
    delay(70); // laisser un délais pour une transition "smooth" entre deux déplacements
  }
}
