#include <Stepper.h>

struct Message{
  unsigned char header[4];
  float angularZ;
  long spin;
  long fire;
};

#define PIN_FIRE 1
#define PIN_SPIN 2

#define STEPS 200

Stepper stepper(STEPS, 8, 9, 10, 11);

volatile float angularZ;
volatile int spin;
volatile int fire;

void setup() {
  angularZ = 0.0;
  spin = 0;
  fire = 0;
  
  stepper.setSpeed(0);
  Serial.begin(9600);
}

void loop() {

  angularZ = PI;
  
  if (angularZ != 0.0){
    /* 
     *  angularZ is radians / second
     *  convert to rotations per minute
     */
     if(angularZ > 0.0)
      stepper.step(1);
     else
      stepper.step(-1);

  }else{
     stepper.setSpeed(0);
  }

  if (fire)
    digitalWrite(PIN_FIRE, HIGH);
  else
    digitalWrite(PIN_FIRE, LOW);

  if (spin)
    digitalWrite(PIN_SPIN, HIGH);
  else
    digitalWrite(PIN_SPIN, LOW);
}


void serialEvent() {

  Message message;  
  
  while (Serial.available()) {
    Serial.readBytes((char*)&message, sizeof(Message));

    if(message.header[0] == 0xDE && 
       message.header[1] == 0xAD && 
       message.header[2] == 0xBE && 
       message.header[3] == 0xEF){

       angularZ = message.angularZ;
       spin = message.spin;
       fire = message.fire;

       long rotationsPerMinute = abs(angularZ) * 60.0 * 1.0/(2.0*PI);
       stepper.setSpeed(rotationsPerMinute);
    }
    
  }
}
