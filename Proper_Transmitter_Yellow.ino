//code should work ok. when it does, copy it to Proper_Transmitter_Yellow and change 9 to 7, 20 to 50, 30 to 60, and 40 to 70.
#include <SoftwareSerial.h>
SoftwareSerial HC12(3,2); // DEFINITIVELY, HC-12 TX Pin, HC-12 RX Pin
const byte nobytes = 7; // Sending 7 bytes here: 2x pre-amble, 4x data, 1x checksum
byte data[nobytes]; 
byte checksum;
int T1 = 0; int T2 = 0;
byte stage = 0; 
unsigned int preamble = 23893; //Pre-amble for security and to allow byte order to be determined on RX
                               // Make sure that no two consecutive TX bytes are the same as the pre-amble

void setup() {
  data[0] = lowByte(preamble); // Assign pre-amble to the first two bytes, note that numbering starts at [0]
  data[1] = highByte(preamble);
  pinMode(A5,INPUT);
  pinMode(A6,INPUT);
  pinMode(A7,INPUT);
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12

}
void loop() {  
//  Serial.print("Left: ");
//  Serial.print(L);
//  Serial.print("Middle: ");
//  Serial.print(M);
//  Serial.print("Right: ");
//  Serial.println(R);
//  delay(500);

   data[2] = 0; //First data byte
   data[3] = 0;
   data[4] = 0;
   data[5] = 78*((millis() / 500) % 2); // Flash this byte between 78 and 0 at a 1 Hz rate
   checksum = 0; for (int i = 2; i < nobytes-1; i++) {checksum = checksum + data[i];} // form checksum for data integrity
   data[nobytes-1] = checksum;
if(stage==0||stage==1) {
   byte M = analogRead(A6)/4; //middle photodiode voltage. read in every loop. slowest operation.
   T2 = millis();
   if(T2-T1>0 && M<=235) {
    HC12.write(data,sizeof(data));
    T1=T1+6;
    //Serial.println(M);
   }
}
  /*
  //if(stage==0 || stage ==1) { //if stage 0 or 1, provide "coarse" feeback.
    //if any photodiode voltage lower than some some reference (235/255, or 4.61V), then send signal
    if(M<235) {
      T2 = millis();
      if(T2-T1>0) {
        data[2] = 9; //9, a good number
        HC12.write(data, sizeof(data)); 
        //stage = stage+1; //
        T1=T1+6; //6 seconds loop period to avoid buffer overflow. 6bytes*9bits_each/9600bit_rate=5.625ms. 
      }
    } 
  //}
  //above function does the same thing for two stages, 0 and 1, because there are two sweeps performed before the fine-tuning.
  
  if(stage==2) { //if stage 2, provide "fine" feedback.
    byte L = analogRead(A5)/4; //left photodiode voltage. only read if in stage 2 because it's slow.
    byte R = analogRead(A7)/4; //right photodiode voltage. only read if in stage 2 because it's slow.
      
    if(L<220) data[2] = 20; //arbitrary output number 1
    if(M<220) data[3] = 30; //arbitrary output number 2
    if(R<220) data[4] = 40; //arbitrary output number 3
    HC12.write(data,sizeof(data));
    //absolutely no delay here!
  } */
}
