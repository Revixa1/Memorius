#include <Arduino.h>
#define TEST_COM
//0=Serial.read

//Lucius
//1=Arrivé personne
//2=Fini Récompense

//Dobby
//3=Fini Personne
//4=Recompanse 1
//5=Recompanse 2


int comMarketing(char tache){

char readSerial[2]={0,0};

if(tache=='0'){
    
    if(Serial.available()>0){
    for(int i=0;Serial.available()>0;i++){
    readSerial[i] =  Serial.read();
    }
    tache=readSerial[0];
    }
    
    #ifdef TEST_COM
    Serial.println(tache);
    #endif
    return tache;
}

if(tache!='0'){
    #ifndef TEST_COM
    Serial.write(tache);
    #endif
    #ifdef TEST_COM
    Serial.print( tache);
    #endif
    return tache;
}


}