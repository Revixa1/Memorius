#include <Arduino.h>
#include <librobus.h>

int i=0;
bool memoire[54];
bool actuel[54];
void setup() {
  // put your setup code here,o run once:
  BoardInit();
  for(i=0;i<53;i++){
    pinMode(i, INPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(digitalRead(28));
  for(i=0;i<53;i++){
    actuel[i]=digitalRead(i);
    if(memoire[i]!=actuel[i] and i!=1 ){
      Serial.println(String("Pin ") + i + " = " + actuel[i]);
    //Serial.print(i);
    //Serial.print(" = ");
    //Serial.println(actuel[i]);
  memoire[i]=actuel[i];
    }
   // delay(250);
    
  }
  
}