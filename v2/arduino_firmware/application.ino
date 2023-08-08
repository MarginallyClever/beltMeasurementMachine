#include "config.h"
//-----------------------------------------------------------------------------

#define ADDRESS_EVERYONE 0x7F  // 0b00001111111

#define COB_NMT_CONTROL 0x000
#define COB_SDO_SEND    0x580
#define COB_SDO_RECEIVE 0x600

#define MAKE_COB_ID(functionCode,address) (functionCode | address)
#define COB_GET_FUNCTION_CODE(cobID)      (cobID&0x780)  // 0b11110000000
#define COB_GET_ADDRESS(cobID)            (cobID&0x07F)  // 0b00001111111


#define NUM_AXIES (6)

//-----------------------------------------------------------------------------

char axies[NUM_AXIES]  = {'X','Y','Z','U','V','W'};
float nextPos[NUM_AXIES];
float lastHeard[NUM_AXIES];
uint32_t lastReq=0;
float velocityDegPerS = 5;
//-----------------------------------------------------------------------------

void APPsetup() {
  MOTORsetTargetVelocity(velocityDegPerS);
}

void APPtoggleCANState() {
  CANstate = (CANstate==0? 255 : 0);
}

void APPprintCANmsg(CAN_msg_t &msg) {
  Serial.print("id=");  Serial.println(msg.id);
  Serial.print("data=");  Serial.println(msg.data[8]);
  Serial.print("len=");  Serial.println(msg.len);
  //Serial.print("ch=");  Serial.println(msg.ch);
  Serial.print("format=");  Serial.println(msg.format);
  Serial.print("type=");  Serial.println(msg.type);
}

void APPreadCAN() {
  if(!CANbus.available()) return;

  //Serial.println("available ");
  //Serial.println();

  CAN_msg_t inbound;
  CANbus.receive(&inbound);
  //APPprintCANmsg(inbound);

  uint16_t functionCode = COB_GET_FUNCTION_CODE(inbound.id);
  if(functionCode == COB_SDO_SEND) {
    //Serial.println("SDO Send");
    if(CANbus.CANBusAddress!=0) return;  // only root cares about receiving messages.

    int index = COB_GET_ADDRESS(inbound.id);  // from who?
    if(index<0 || index>=NUM_AXIES) return;

    //Serial.print("address ");
    //Serial.println(index);

    uint8_t i=0;
    uint16_t id = CAN_GET_LONG(inbound,i);
    if( id == CAN_CUSTOM_PARAMETER_READ ) {
      int subIndex = CAN_GET_SHORT(inbound,i);
      if(subIndex == CAN_CUSTOM_PARAMETER_READ_POSITION) {
        APPtoggleCANState();
        lastHeard[index] = CAN_GET_FLOAT(inbound,i) * 360.0;
        //Serial.print("receive one position ");
        //Serial.print(index);
        //Serial.print("=");
        //Serial.println(lastHeard[index]);
      }
    }
  } else if( functionCode == COB_SDO_RECEIVE) {
    //Serial.println("SDO Receive");
    int index = COB_GET_ADDRESS(inbound.id);  // for who?
    if(index != 0x7f && index != CANbus.CANBusAddress) return;  // not for everyone and not for me

    //Serial.println("For me");

    uint8_t i=0;
    int id = CAN_GET_LONG(inbound,i);
    if( id == CAN_CUSTOM_PARAMETER_READ ) {
      APPtoggleCANState();
      int subIndex = CAN_GET_SHORT(inbound,i);
      if(subIndex == CAN_CUSTOM_PARAMETER_READ_POSITION) APPsendSensor();
    } else if( id == CAN_SET ) {
      uint8_t subIndex = CAN_GET_SHORT(inbound,i);
      if(subIndex == CAN_SET_POSITION) {
        float targetPosition = CAN_GET_FLOAT(inbound,i);

        APPtoggleCANState();
        Serial.print("set position ");
        Serial.println(targetPosition*360.0f);

        MOTORsetTargetPosition(targetPosition);
      } else if(subIndex == CAN_SET_VELOCITY) {
        velocityDegPerS = CAN_GET_FLOAT(inbound,i);

        APPtoggleCANState();
        Serial.print("set velocity ");
        Serial.println(velocityDegPerS);

        MOTORsetTargetVelocity(velocityDegPerS);
      }
    }
  }
}


void APPsendSensor() {
  //Serial.println("APPsendSensor");
  CAN_msg_t CAN_TX_msg;
  CAN_TX_msg.id = MAKE_COB_ID(COB_SDO_SEND,CANbus.CANBusAddress);
  CAN_START(CAN_TX_msg);
  CAN_ADD_LONG(CAN_TX_msg,CAN_CUSTOM_PARAMETER_READ);
  CAN_ADD_SHORT(CAN_TX_msg,CAN_CUSTOM_PARAMETER_READ_POSITION);
  CAN_ADD_FLOAT(CAN_TX_msg,sensorAngleUnit);
  CANbus.send(&CAN_TX_msg);
}


void APPrequestOnePosition(uint8_t index) {
  if(index==0 && CANbus.CANBusAddress==0) {
    //Serial.println("APPrequestOnePosition myself");
    lastHeard[index] = sensorAngleUnit * 360.0;
  } else {
    //Serial.print("APPrequestOnePosition ");
    //Serial.println(index);
    CAN_msg_t CAN_TX_msg;
    CAN_TX_msg.id = MAKE_COB_ID(COB_SDO_RECEIVE,index);
    CAN_START(CAN_TX_msg);
    CAN_ADD_LONG(CAN_TX_msg,CAN_CUSTOM_PARAMETER_READ);
    CAN_ADD_SHORT(CAN_TX_msg,CAN_CUSTOM_PARAMETER_READ_POSITION);
    CANbus.send(&CAN_TX_msg);
  }
}

void APPrequestAllPositions2() {
  lastHeard[0] = sensorAngleUnit * 360.0;
  APPrequestOnePosition(ADDRESS_EVERYONE);
}

void APPrequestAllPositions1() {
  float v;
  for(int i=0;i<NUM_AXIES;++i) {
    APPrequestOnePosition(i);
  }
}

void APPrequestAllPositions() {
  APPrequestAllPositions1();
}

/**
 * @param index which joint to move
 * @param v 0...1
 */
void APPsendOnePosition(int index,float v) {
  if(index==0 && CANbus.CANBusAddress==0) {
    //Serial.print("APPsendOnePosition myself ");
    //Serial.println(v);
    MOTORsetTargetPosition(v);
  } else {
    //Serial.print("APPsendOnePosition ");
    //Serial.print(index);
    //Serial.print(' ');
    //Serial.println(v);
    CAN_msg_t CAN_TX_msg;
    CAN_TX_msg.id = MAKE_COB_ID(COB_SDO_RECEIVE,index);
    CAN_START(CAN_TX_msg);
    CAN_ADD_LONG(CAN_TX_msg,CAN_SET);
    CAN_ADD_SHORT(CAN_TX_msg,CAN_SET_POSITION);
    CAN_ADD_FLOAT(CAN_TX_msg,v);
    CANbus.send(&CAN_TX_msg);
  }
}

/**
 * @param index which joint to move
 * @param v 0...1
 */
void APPsendOneVelocity(int index,float v) {
  if(index==0 && CANbus.CANBusAddress==0) {
    //Serial.print("APPsendOnePosition myself ");
    //Serial.println(v);
    velocityDegPerS = v;
    MOTORsetTargetVelocity(velocityDegPerS);
  } else {
    //Serial.print("APPsendOnePosition ");
    //Serial.print(index);
    //Serial.print(' ');
    //Serial.println(v);
    CAN_msg_t CAN_TX_msg;
    CAN_TX_msg.id = MAKE_COB_ID(COB_SDO_RECEIVE,index);
    CAN_START(CAN_TX_msg);
    CAN_ADD_LONG(CAN_TX_msg,CAN_SET);
    CAN_ADD_SHORT(CAN_TX_msg,CAN_SET_VELOCITY);
    CAN_ADD_FLOAT(CAN_TX_msg,v);
    CANbus.send(&CAN_TX_msg);
  }
}


//--------------------------------------------------------------------
//--------------------------------------------------------------------

void APPmeasureBelt() {
  Serial.println("APPmeasureBelt");
  APPreadVelocity();

  int pos = seen('X');
  if(pos>=0) {
    float mm = atof(serialBufferIn+pos);
    Serial.print("Move ");
    Serial.print(mm);
    Serial.println("mm");
    stepThisFar(mm);
  }
}


template <typename T> int signum(T val) {
    return (T(0) < val) - (val < T(0));
}

// motor is 400 steps.
#define MOTOR_STEPS_PER_TURN (400.0)
// Pulley is 20 teeth of GT2, or 40mm pitch.
#define PULLEY_PITCH         (40.0)

#define MM_PER_STEP          (PULLEY_PITCH / MOTOR_STEPS_PER_TURN)

// microsecond delay per step at the first and last step
#define START_STEP_DELAY     (5000)
// microsecond delay at top speed (should be a smaller number)
#define MIN_STEP_DELAY       (800)
// change in delay per step.
#define ACCELERATION         (5)

/**
 * 1 = the belt moves through as the sensor increases
 *-1 = the belt moves through as the sensor decreases
 */
#define SENSOR_TO_BELT       (-1)

void stepThisFar(int mm) {
  mm *= SENSOR_TO_BELT;

  double previousPosition = getCurrentSensorPosition();
  double sum = 0;
  int dir = signum(mm);
  MOTORsetDirection(dir);

  int16_t stepTimeUs = START_STEP_DELAY;

#ifdef DEBUG_BELT
  Serial.print("goal=");
  Serial.println(mm);
  Serial.print("start=");
  Serial.println(previousPosition);
#endif
  // find acceleration trapezoid
  int stepCount=0;
  int stepsToDecel, stepsToAccel;

  double absMM = abs(mm);
  int totalSteps = (int)floor(absMM / MM_PER_STEP);

  int STEP_DIFF = (START_STEP_DELAY-MIN_STEP_DELAY)/ACCELERATION;
  if(totalSteps > STEP_DIFF*2) {
    stepsToDecel = totalSteps - 1 - STEP_DIFF;
    stepsToAccel = STEP_DIFF;
  } else {
    stepsToAccel = 
    stepsToDecel = totalSteps/2;
  }
  Serial.print("Total=");  Serial.println(totalSteps);
  Serial.print("Accel=");  Serial.println(stepsToAccel);
  Serial.print("Decel=");  Serial.println(stepsToDecel);

  // while difference is more than a single step...
  while(abs(mm-sum) >= MM_PER_STEP) {
    MOTORmakeOneStep();
    delayMicroseconds(stepTimeUs);

    // read change in position
    double currentPosition = getCurrentSensorPosition();
    double mmChange = currentPosition - previousPosition;
    previousPosition = currentPosition;

#ifdef DEBUG_BELT
    Serial.print(currentPosition);
    Serial.print('\t');
    Serial.print(mmChange);
    Serial.print('\t');
#endif

    // missed step?
    if(mmChange==0) {
      Serial.println(" missed step?");
      continue;
    }

    // handle sensor rollover
    if(abs(mmChange)>PULLEY_PITCH/2) {
      // probably passing the line from 0 to 359.999.. degrees.
      mmChange += (mmChange>0) ? -PULLEY_PITCH : PULLEY_PITCH;
    }

    // record change
    sum += mmChange;

    // handle acceleration
    if(stepCount < stepsToAccel && stepTimeUs>MIN_STEP_DELAY) stepTimeUs -= ACCELERATION;
    if(stepCount >= stepsToDecel && stepTimeUs<START_STEP_DELAY) stepTimeUs += ACCELERATION;
    stepCount++;

#ifdef DEBUG_BELT
    Serial.print(mmChange);
    Serial.print('\t');
    Serial.print(sum);
    Serial.print('\n');
#endif
  }
  // done!
  Serial.print("done ");
  Serial.println(sum);
}

double getCurrentSensorPosition() {
  SENSORread();
  return sensorAngleUnit * PULLEY_PITCH;
}
//--------------------------------------------------------------------
//--------------------------------------------------------------------




void APPrapidMove() {
  Serial.println("APPrapidMove");
  APPreadVelocity();
  
  // positions
  float v;
  for(int i=0;i<NUM_AXIES;++i) {
    int pos = seen(axies[i]);
    if(pos>=0) {
      v = atof(serialBufferIn+pos);
      Serial.print(' ');
      Serial.print(axies[i]);
      Serial.print(v);
      v /= 360.0;
      nextPos[i] = v;
      APPsendOnePosition(i,v);
    }
  }

  Serial.print(' ');
  Serial.print('F');
  Serial.print(velocityDegPerS);
  Serial.println();
}


void APPreadVelocity() {
  // velocity
  int pos = seen('F');
  if(pos>=0) {
    APPsetVelocity(atof(serialBufferIn+pos));
    // if this delay is too small the other unit doesn't receive the positions that follow.
    // 250 is too small.  500 works.  375 works.
    delay(375);
  }
}


void APPreportAllMotorPositions() {
  Serial.print("M114");
  for(int i=0;i<NUM_AXIES;++i) {
    Serial.print(' ');
    Serial.print(axies[i]);
    Serial.print(lastHeard[i],2);
  }

  Serial.print(' ');
  Serial.print('F');
  Serial.print(velocityDegPerS);
  Serial.println();
}

void APPserverUpdate() {
  // only server at address 0
  if(CANbus.CANBusAddress!=0) return;
  
  // sometimes ask for position updates.
  uint32_t t = millis();
  if(t - lastReq > 1000) {
    lastReq=t;
    APPrequestAllPositions();
  }
}


void APPupdate() {
  APPreadCAN();
  APPserverUpdate();
}

void APPsetVelocity(float newVel) {
  for(int i=0;i<NUM_AXIES;++i) {
    APPsendOneVelocity(i,newVel);
  }
}