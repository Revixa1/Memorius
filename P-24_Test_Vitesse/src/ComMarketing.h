#include <Arduino.h>
//#define TEST_COM
//0=Serial.read

//Lucius
//1=Arrivé personne
//2=Fini Récompense

//Dobby
//3=Fini Personne
//4=Recompanse 1
//5=Recompanse 2
unsigned long ComTime=0;
unsigned long PrevComTime=0;

int comMarketing(char tache){

char readSerial[2]={0,0};

if(tache=='0'){
    Serial1.flush();
    while (tache=='0'){
    if(Serial1.available()>0){
    for(int i=0;Serial1.available()>0;i++){
    readSerial[i] =  Serial1.read();
    }
    tache=readSerial[0];
    }
    }
    Serial1.write('0');
    #ifdef TEST_COM
    Serial.println(tache);
    #endif
    return tache;
}

if(tache!='0'){
    Serial1.flush();
    PrevComTime=millis();
    
    while(tache!='0'){

        ComTime=millis();

        if(ComTime-PrevComTime>1000 and tache!='0'){
         #ifndef TEST_COM
         Serial1.write(tache);
         #endif
    
         #ifdef TEST_COM
         Serial.print( tache);
         #endif
         PrevComTime=ComTime;
        } else  if(Serial1.available()>0){
    for(int i=0;Serial1.available()>0;i++){
    readSerial[i] =  Serial1.read();
    }
    tache=readSerial[0];
    }


        }
    }
    return tache;
}


