// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char DEVICE_LOGIN_NAME[]  = "7b1c38b3-4d70-4c4a-afbc-6e9825f0586b";

const char SSID[]               = SECRET_SSID;    // Network SSID (name)
const char PASS[]               = SECRET_OPTIONAL_PASS;    // Network password (use for WPA, or use as key for WEP)
const char DEVICE_KEY[]  = SECRET_DEVICE_KEY;    // Secret device password

void onLatChange();
void onLonChange();
void onBlueChange();
void onButtonChange();
void onGreenChange();
void onRedChange();

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

void initProperties(){

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(lat, READWRITE, ON_CHANGE, onLatChange);
  ArduinoCloud.addProperty(lon, READWRITE, ON_CHANGE, onLonChange);
  ArduinoCloud.addProperty(bpm, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(ecg, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(oxi, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(destination, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(blue, READWRITE, ON_CHANGE, onBlueChange);
  ArduinoCloud.addProperty(button, READWRITE, ON_CHANGE, onButtonChange);
  ArduinoCloud.addProperty(green, READWRITE, ON_CHANGE, onGreenChange);
  ArduinoCloud.addProperty(red, READWRITE, ON_CHANGE, onRedChange);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
