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
//6=Recompanse 3
unsigned long ComTime=0;
unsigned long PrevComTime=0;

int comMarketing(int Com){

char readSerial[30];

Serial1.begin(9600);

if(Com=='0'){
    
    while (Com=='0'){
    if(Serial.available()>0){delay(10);
    for(int i=0;Serial.available()>0;i++){
    readSerial[i] =  Serial.read();
    }
    Com=readSerial[0];
    }
    }

    Serial.write('0');
    #ifdef TEST_COM
    Serial.print("read");
    Serial.println((char)Com);
    #endif
    return Com;
}

if(Com!='0'){
    
    PrevComTime=millis();

    while(Com!='0'){

        ComTime=millis();

        if(ComTime-PrevComTime>200 and Com!='0'){
         
         Serial.write((char)Com);
         
         
         #ifdef TEST_COM
         Serial.print("write");
         Serial.println((char)Com);
         #endif
         PrevComTime=ComTime;
        } 
        else  if(Serial.available()>0){delay(10);
            for(int i=0;Serial.available()>0;i++){
                 readSerial[i] =  Serial.read();
            }
            Com=readSerial[0];
        }


        }
    }
    #ifdef TEST_COM
    Serial.println((char)Com);
    #endif
    return Com;
}


