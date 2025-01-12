#include "arduino_secrets.h"
/* 
  float lat;
  float lon;
  int bpm;
  int ecg;
  int oxi;
  CloudLocation destination;
  bool blue;
  bool button;
  bool green;
  bool red;
*/

#include "thingProperties.h"

#include "DFRobot_BloodOxygen_S.h"

#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x57
  DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);
#else

#if defined(ARDUINO_AVR_UNO) || defined(ESP32)
SoftwareSerial mySerial(D2, D3);  // SoftwareSerial mySerial(4, 5);
DFRobot_BloodOxygen_S_SoftWareUart MAX30102(&mySerial, 9600);

#else
DFRobot_BloodOxygen_S_HardWareUart MAX30102(&Serial1, 9600); 
#endif
#endif

const int heartPin = A7;
const int R = 14;
const int G = 15;
const int B = 16;




void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);

  //##############################Initialization###################
  //###############################################################
  while (false == MAX30102.begin())
  {
    Serial.println("init fail!");
    delay(1000);
  }
  Serial.println("init success!");
  Serial.println("start measuring...");
  MAX30102.sensorStartCollect();
  //###############################################################
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  ArduinoCloud.printDebugInfo();
}



//################################################################
//#########################heartrate meter operation##############
void loop() {
  ArduinoCloud.update();
  MAX30102.getHeartbeatSPO2();
  
  bpm = MAX30102._sHeartbeatSPO2.Heartbeat;
  oxi = MAX30102._sHeartbeatSPO2.SPO2;
  
}
unsigned short int bufferheart[300];


void onButtonChange(){
  int i = 0;
  int j = 0;
  while(i < 250){
    bufferheart[i] = analogRead(heartPin);
    i++;
    delay(4);
  }
  while(j < 250){
    
    ecg = bufferheart[j];
    bufferheart[j] = 0;
    j++;
    delay(50);
  }
  
  
}



//#########################Dummy pill drop system###############
void onRedChange()  {
  setColor(255, 0, 0);
}

void onGreenChange()  {
  setColor(0,  255, 0);
}

void onBlueChange()  {
  setColor(0, 0, 255);
}
void setColor(int redValue, int greenValue,  int blueValue) {
  analogWrite(R, redValue);
  analogWrite(G,  greenValue);
  analogWrite(B, blueValue);
}


//#########################Mission control#####################
void onLatChange()  {
  destination = {lat, lon};
}

void onLonChange()  {
  destination = {lat, lon};
}
