#include <Arduino.h>

// put function declarations here:


const int SENSOR_AMT=8;
int count=0;
void setup() {
  // put your setup code here, to run once:
  //Consolidate to functions in lib later and declare constants for the pins
  //Mux Ouput Pins Set
  pinMode(32,OUTPUT);
  pinMode(34,OUTPUT);
  pinMode(36,OUTPUT);
  //Enable the analog input
  pinMode(A0, INPUT); 
  //input is pulled up to 5V by PCB, dont need a pull res here
  /*try to go faster and hit the maximum rate of sampling
  9615 * 2 bytes * 8 bits = 153,400
  */
  Serial.begin(115200);
  //Serial.begin(1000000);

}
char buf[40];

void loop() {
  //set outputs to required polling of value 
  //10 bit adc max value 1023
  digitalWrite(32,count>>2);
  digitalWrite(34,count>>1 && 0x1);
  digitalWrite(36,count && 0x1);
  uint16_t sensor_value=analogRead(A0);         // 0-1023
  uint8_t pkt[2];
  pkt[0] = 0x80 | (count << 3) | (sensor_value >> 7);    // marker bit | 4-bit ID | top 3 bits of value
  pkt[1] = sensor_value & 0x7F;                      // low 7 bits
  Serial.write(pkt, 2);
  count++;
  count%=SENSOR_AMT;

}

// put function definitions here:
