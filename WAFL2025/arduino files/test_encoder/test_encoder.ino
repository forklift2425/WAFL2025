// Encoder pins
#define ENCODER_PIN_A 2  // Interrupt pin
#define ENCODER_PIN_B 3  // Regular digital input

// Encoder parameters
const int PPR = 600;           // Pulses Per Revolution
const unsigned long UPDATE_INTERVAL = 100; // Update every 100 ms

// Variables
volatile long pulse_count = 0;
volatile int direction = 1;    // +1 for CW, -1 for CCW

unsigned long last_update_time = 0;
float rpm = 0.0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);

  last_update_time = millis();
}

void loop() {
  unsigned long current_time = millis();

  if (current_time - last_update_time >= UPDATE_INTERVAL) {
    noInterrupts();  // Temporarily disable interrupts to read pulse_count safely
    long pulses = pulse_count;
    pulse_count = 0; // Reset after reading
    int current_direction = direction;
    interrupts();    // Re-enable interrupts

    // Calculate RPM
    float revolutions = (float)pulses / (float)PPR;
    rpm = 80*(revolutions * 600.0) / (float)UPDATE_INTERVAL; // (600 = 60 sec / 0.1 sec)

    // Adjust sign based on direction
    rpm = rpm * current_direction;

    // Print RPM
    Serial.print("RPM: ");
    Serial.println(rpm);

    last_update_time = current_time;
  }
}

// Interrupt Service Routine for Encoder
void encoderISR() {
  bool A = digitalRead(ENCODER_PIN_A);
  bool B = digitalRead(ENCODER_PIN_B);

  if (A == B) {
    direction = 1;  // CW
  } else {
    direction = -1; // CCW
  }

  pulse_count++;
}
