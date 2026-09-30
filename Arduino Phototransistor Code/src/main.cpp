#include <Arduino.h>

const int SENSOR_AMT=8;
uint8_t count=0;
const int pinA=A0;
const int pinB=A1;
const int pinC=A2;
const int analog_in=A3;
void setup() {
  // put your setup code here, to run once:
  //Consolidate to functions in lib later and declare constants for the pins
  //Mux Ouput Pins Set
  pinMode(pinA,OUTPUT);
  pinMode(pinB,OUTPUT);
  pinMode(pinC,OUTPUT);
  //Enable the analog input
  pinMode(analog_in, INPUT); 
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
  digitalWrite(pinC,count>>2);
  digitalWrite(pinB,count>>1 && 0x1);
  digitalWrite(pinA,count && 0x1);

  uint16_t sensor_value= analogRead(analog_in);         // 0-1023
  uint8_t pkt[2];
  Serial.println(sensor_value);
  Serial.println(count);
  pkt[0] = 0x80 | (count << 3) | (sensor_value >> 7);    // marker bit | 4-bit ID | top 3 bits of value
  pkt[1] = sensor_value & 0x7F;                      // low 7 bits
  Serial.write(pkt, 2);
  count++;
  count%=SENSOR_AMT;
}

