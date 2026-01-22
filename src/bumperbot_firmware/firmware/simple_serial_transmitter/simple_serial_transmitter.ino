/**
 * simple_serial_transmitter for wemos d1 mini
 * 
 * sends data over serial to the raspberry pi
 */

int x = 0

void setup() {
  // init serial
  Serial.begin(115200);
}

void loop() {
  Serial.println(x);
  x++;
  delay(1000);
  // will print to serial forever
}
