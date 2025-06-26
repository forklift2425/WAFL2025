void setup() {
  Serial1.begin(9600);
  Serial.begin(57600); 
}

void loop() {
 int right_speed_byte = Serial1.read();
 int left_speed_byte = Serial1.read();
      
// Debug prints
Serial.print(left_speed_byte); Serial.print(" ");
Serial.print(right_speed_byte); Serial.println(" ");
delay(20);
}
