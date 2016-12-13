#include "Encoder.h"

Encoder encoder;
Encoder encoder2;

bool ledStatus = true;

void setup() 
{
  Serial.begin(9600);

  encoder.begin(A3, A4);
  encoder2.begin(4,5);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);


  encoder2.setCallback(blinkLED);
  
  Serial.println("Done Init...");
}


void loop() 
{
 
  if (encoder.changed())
  {
    Serial.println(encoder.getCount());
    
    encoder.reset();
  }

  if (encoder2.changed())
  {
    Serial.println(encoder2.getCount());
    
    encoder2.reset();
  }

  // set the builtin LED
  digitalWrite(LED_BUILTIN, ledStatus?HIGH:LOW);

}

// simple callback function
void blinkLED()
{
  ledStatus = !ledStatus;
}

