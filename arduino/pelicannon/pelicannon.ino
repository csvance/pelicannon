#include <Stepper.h>

#define ENABLE_STEPPER_PITCH 0

#define STEPPER_ANGLE(steps, stepsRot) 2*PI*((steps % stepsRot) / stepsRot)
#define LEAST_ANGLE_DIFF(x, y) atan2(sin(x-y), cos(x-y))
#define ROTATIONS_PER_MINUTE(av) abs(av) * 60.0 * 1.0/(2.0*PI)

struct RecvMessage{
  unsigned char header[4];
  double speedPitch;
  double speedYaw;
  long spin;
  long fire;
  long goal;
  double goalPitch;
  double goalYaw;
};

struct SendMessage{
  unsigned char header[4];
  double pitch;
  double yaw;
};

#define PIN_FIRE 1
#define PIN_SPIN 2



#define STEPS_YAW 200
Stepper stepperYaw(STEPS_YAW, 8, 9, 10, 11);
int currentStepsYaw;

#if ENABLE_STEPPER_PITCH
#define STEPS_PITCH 200
#error "Configure stepperPitch pins!"
Stepper stepperPitch(STEPS_PITCH, 8, 9, 10, 11);
int currentStepsPitch;
#endif

bool validMessageRecieved;
RecvMessage state;

void setup() {
  validMessageRecieved = false;

  currentStepsYaw = 0;
#if ENABLE_STEPPER_PITCH
  currentStepsPitch = 0;
#endif
  
  allStop();

  Serial.begin(9600);

  // Send initial packet to finish setup
  sendPosition();
}

void allStop(){
  digitalWrite(PIN_SPIN, LOW);
  digitalWrite(PIN_FIRE, LOW);
  stepperYaw.setSpeed(0);
#if ENABLE_STEPPER_PITCH
  stepperPitch.setSpeed(0);
#endif
}

void loop() {

  bool sendUpdate = false;

  if(validMessageRecieved){

      /* Goal mode is enabled, stop moving once we reach the goal */
      if(state.goal){

        double thetaYaw = STEPPER_ANGLE(currentStepsYaw, STEPS_YAW);
        if( LEAST_ANGLE_DIFF(thetaYaw, state.goalYaw) > 2*PI/100 )
            state.speedYaw = 0.0;

        
#if ENABLE_STEPPER_PITCH
        double thetaPitch = STEPPER_ANGLE(currentStepsPitch, STEPS_PITCH);
        if( LEAST_ANGLE_DIFF(thetaPitch, state.goalPitch) > 2*PI/100 )
            state.speedPitch = 0.0;
#endif

      }

      /* Handle Z stepper motor */
      stepperYaw.setSpeed(ROTATIONS_PER_MINUTE(state.speedYaw));

      if(state.speedYaw > 0.0){
        stepperYaw.step(1);
        currentStepsYaw++;
        sendUpdate = true;
      }else if (state.speedYaw < 0.0){
        stepperYaw.step(-1);
        currentStepsYaw--;
        sendUpdate = true;
      }else{
        stepperYaw.setSpeed(0.0);
      }

#if ENABLE_STEPPER_PITCH
      /* Handle XY stepper motor */
      stepperPitch.setSpeed(ROTATIONS_PER_MINUTE(state.speedPitch));

      if(state.speedPitch > 0.0){
        stepperPitch.step(1);
        currentStepsPitch++;
        sendUpdate = true;
      }else if (state.speedPitch < 0.0){
        stepperPitch.step(-1);
        currentStepsPitch--;
        sendUpdate = true;
      }else{
        stepperPitch.setSpeed(0.0);
      }
#endif

      if (state.fire)
        digitalWrite(PIN_FIRE, HIGH);
      else
        digitalWrite(PIN_FIRE, LOW);
      
      if (state.spin)
        digitalWrite(PIN_SPIN, HIGH);
      else
        digitalWrite(PIN_SPIN, LOW);
    
  }else{
    /* Invalid message, stop everything */
    allStop();
  }

  if (sendUpdate)
    sendPosition();
  
}


void sendPosition(){
  SendMessage message;

  message.header[0] = 0xDE;
  message.header[1] = 0xAD;
  message.header[2] = 0xBE;
  message.header[3] = 0xEF;

  message.yaw = STEPPER_ANGLE(currentStepsYaw, STEPS_YAW);

#if ENABLE_STEPPER_PITCH
  message.pitch = STEPPER_ANGLE(currentStepsPitch, STEPS_PITCH);
#else
  message.pitch = 0.0;
#endif

  Serial.write((char*)&message, sizeof(SendMessage));
}


void serialEvent() {

  RecvMessage message;  

  while (Serial.available()) {
    Serial.readBytes((char*)&message, sizeof(RecvMessage));

    if(message.header[0] == 0xDE && 
       message.header[1] == 0xAD && 
       message.header[2] == 0xBE && 
       message.header[3] == 0xEF){

       memcpy((void*)&state, (void*)&message, sizeof(RecvMessage));

       validMessageRecieved = true;
    }
    
  }
}
