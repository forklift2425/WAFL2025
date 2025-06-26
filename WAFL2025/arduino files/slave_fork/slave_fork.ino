#include <SPI.h>

const byte START_BYTE = 0xAA;
const int BUFFER_SIZE = 5;
 long count =0;
// SPI Buffers
volatile byte bufferToSend[BUFFER_SIZE] = {0};
volatile byte spiState = 0;       // 0: waiting for START, 1–2: receiving target, 3–7: sending response
volatile int16_t targetAngleInt = 0;
volatile byte byteIndex = 0;
//Fork Relayes
const byte FORK_UP_PIN = A13;
const byte FORK_DOWN_PIN = A12;
const byte FORK_BOTTOM_LIMIT = A11;
const byte FORK_TOP_LIMIT = A10;

// Fork State Variables
volatile uint8_t forkCommandReceived = 0;  // Command from Mega
uint8_t forkState = 0;  // 0=moving, 1=bottom, 2=top

//Homeing variables
const byte limitSwitchPin = 6;  // Normally closed limit switch
enum SystemState { HOMING, OPERATING };
SystemState systemState = HOMING;
const long PULSES_TO_ZERO = 5764;  // Pulses from limit switch to zero position
volatile long limitPulseCount = 0; // Stores pulse count at limit switch


// Encoder
const byte encoderPinA = 2;
const byte encoderPinB = 3;
volatile long pulseCount = 0;
const int ppr = 996;
const float pulleyToWheel = 3.0 / 47.0;

// Motor
const int motorPWM = 5;
const int motorDIR = 4;
float currentAngleDeg = 0;
float Kp = 3.0;

// ISR: SPI Communication Handler
ISR(SPI_STC_vect) {
  byte incoming = SPDR;

  switch (spiState) {
    case 0:  // Waiting for START_BYTE
      if (incoming == START_BYTE) {
        spiState = 1;
        byteIndex = 0;
      }
      SPDR = 0x00;
      break;

    case 1:  // Receiving LSB of target angle
      targetAngleInt = incoming;
      spiState = 2;
      SPDR = 0x00;
      break;

    case 2:  // Receiving MSB of target angle
      targetAngleInt |= (incoming << 8);
      spiState = 3;
      SPDR = 0x00;
      break;

    // NEW STATE: Receive fork command
    case 3:
      forkCommandReceived = incoming;
      spiState = 4;
      byteIndex = 0;
      SPDR = bufferToSend[0];  // Start sending response
      break;

    case 4: case 5: case 6: case 7:  // Sending feedback buffer
      byteIndex++;
      if (byteIndex < BUFFER_SIZE) {
        SPDR = bufferToSend[byteIndex];
        spiState++;
      } else {
        SPDR = 0x00;
        spiState = 0;
      }
      break;

    default:
      spiState = 0;
      SPDR = 0x00;
      break;
  }
}

void setup() {

  Serial.begin(9600);

  // Limit Switch setup
  pinMode(limitSwitchPin, INPUT_PULLUP);

  // SPI setup
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();

  // Encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);


  // Motor
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);

// Fork Control Setup
pinMode(FORK_UP_PIN, OUTPUT);
pinMode(FORK_DOWN_PIN, OUTPUT);
pinMode(FORK_BOTTOM_LIMIT, INPUT_PULLUP);
pinMode(FORK_TOP_LIMIT, INPUT_PULLUP);
digitalWrite(FORK_UP_PIN, LOW);
digitalWrite(FORK_DOWN_PIN, LOW);


  // Start homing procedure
  digitalWrite(motorDIR, LOW);  // Move clockwise
  analogWrite(motorPWM, 70);

  // Fork Homing (add at end of setup)
  Serial.println("Homing fork...");
  while (digitalRead(FORK_BOTTOM_LIMIT) != HIGH) { // Move down until limit pressed
    digitalWrite(FORK_DOWN_PIN, HIGH);
    delay(10);
}
  digitalWrite(FORK_DOWN_PIN, LOW);
  forkState = 1; // Set to bottom state
  Serial.println("Fork homed.");
 
} 

void loop() {
    if (systemState == HOMING) {
     // SPCR &= ~_BV(SPIE); // Disable SPI interrupts
   Serial.println("System started. Beginning homing procedure...");
    // Stage 1: Move clockwise until limit switch is hit
    if (digitalRead(limitSwitchPin) == HIGH) {  // Switch opens when pressed
      analogWrite(motorPWM, 0);  // Stop motor
       Serial.print(" limit switch pressed");
    
      // Record position and calculate return target
      noInterrupts();
      limitPulseCount = pulseCount;
      long targetPulse = limitPulseCount - PULSES_TO_ZERO;
      
      interrupts();
     

      // Stage 2: Move back to zero position
      digitalWrite(motorDIR, HIGH);  // Reverse direction
      analogWrite(motorPWM, 70);
      
      // Wait until pulses reach target
      while (pulseCount > targetPulse) { 
      Serial.print("targetPulse");
      Serial.println(targetPulse);
      Serial.print("pulscount: ");
      Serial.println(pulseCount);
        delay(10); // Non-blocking wait
      }
      
      // Stop motor and reset encoder
      analogWrite(motorPWM, 0);
      noInterrupts();
      pulseCount = 0;  // Reset to zero position
      interrupts();
      
      systemState = OPERATING;  // Switch to normal operation
      
    }
    delay(100);
   // SPCR |= _BV(SPIE); // Re-enable SPI interrupts
  }
  else{
     // OPERATING MODE (original logic)
     
  // NEW FORK CONTROL LOGIC
    bool bottomPressed = (digitalRead(FORK_BOTTOM_LIMIT) == HIGH);
  // NEW FORK CONTROL LOGIC
    bool bottomPressed = (digitalRead(FORK_BOTTOM_LIMIT) == HIGH);
    bool topPressed = (digitalRead(FORK_TOP_LIMIT) == HIGH);

    // Update fork state based on limits
    if (bottomPressed) forkState = 1;
    else if (topPressed) forkState = 2;
    else forkState = 0;  bool topPressed = (digitalRead(FORK_TOP_LIMIT) == HIGH);

    // Update fork state based on limits
    if (bottomPressed) forkState = 1;
    else if (topPressed) forkState = 2;
    else forkState = 0;
    
   // Execute fork command
    switch (forkCommandReceived) {
      case 1: // Move up
        if (!topPressed) {
          digitalWrite(FORK_UP_PIN, HIGH);
          digitalWrite(FORK_DOWN_PIN, LOW);
        } else {
          digitalWrite(FORK_UP_PIN, LOW);
          digitalWrite(FORK_DOWN_PIN, LOW);
        }
        break;
      case 2: // Move down
        if (!bottomPressed) {
          digitalWrite(FORK_UP_PIN, LOW);
          digitalWrite(FORK_DOWN_PIN, HIGH);
        } else {
          digitalWrite(FORK_UP_PIN, LOW);
          digitalWrite(FORK_DOWN_PIN, LOW);
        }
        break;
      default: // Stop
        digitalWrite(FORK_UP_PIN, LOW);
        digitalWrite(FORK_DOWN_PIN, LOW);
    }

    
    // Calculate current angle (unchanged)
    noInterrupts();
     count = pulseCount;
    interrupts();

    float pulleyTurns = float(count) / ppr;
    float wheelTurns = pulleyTurns * pulleyToWheel;
    currentAngleDeg = wrapTo180(wheelTurns * 360.0);

    // Update buffer to send (unchanged)
    int16_t angleInt = (int16_t)(currentAngleDeg * 100.0);
    noInterrupts();
    bufferToSend[0] = lowByte(angleInt);
    bufferToSend[1] = highByte(angleInt);
    bufferToSend[2] = forkState;
    bufferToSend[3] = 0x66;
    bufferToSend[4] = 0x77;
    interrupts();

    // P Controller (unchanged)
    float targetAngle =targetAngleInt / 100.0;
    float error = targetAngle - currentAngleDeg;
    
    
    if (abs(error) >= 3){
      digitalWrite(motorDIR, error < 0 ? HIGH : LOW);
      analogWrite(motorPWM, 70);
    }
    else{
      analogWrite(motorPWM, 0);
    }
    // Debug output (unchanged)
    Serial.print("Buffer: ");
    for (byte i = 0; i < BUFFER_SIZE; i++) {
      Serial.print("0x");
      if (bufferToSend[i] < 16) Serial.print("0");
      Serial.print(bufferToSend[i], HEX);
      Serial.print(" ");
    }
    Serial.print(" | Target: ");
    Serial.print(targetAngle);
    Serial.print(" | Measured: ");
    Serial.print(currentAngleDeg);
    Serial.print(" | Pulse count ");
    Serial.println(count);

     Serial.print("RECV CMD: ");
    Serial.print(forkCommandReceived);
    Serial.print(" | STATE: ");
    Serial.print(forkState);
    Serial.print(" | ANGLE: ");
    Serial.println(currentAngleDeg);
    
  }
  delay(10);
}

void updateEncoder() {
  bool a = digitalRead(encoderPinA);
  bool b = digitalRead(encoderPinB);
  pulseCount += (a == b) ? 1 : -1;
}

// Utility: Wrap angle to range [-180, 180)
float wrapTo180(float angle) {
  while (angle <= -180.0) angle += 360.0;
  while (angle > 180.0) angle -= 360.0;
  return angle;
}
