/**
 * simple_serial_receiver for wemos d1 mini
 * 
 * turns the builtin LED on/off depending on serial input
 */

#define LED_BUILTIN D4                    // wemos d1 led builtin pin is d4

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // init serial
  Serial.begin(115200);
}

void loop() {
  // setup main loop
  if (Serial.available()){
    // take input
    int x = Serial.readString().toInt();    // convert the string input to integer
    if (x == 1){
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  
  }

}
