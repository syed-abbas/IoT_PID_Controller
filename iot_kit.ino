/*
 Basic ESP8266 MQTT example

 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.

 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

*/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <stdlib.h>
#include <string.h>
#include <SimpleDHT.h>
#define MOTOR_TIMER (20*60*1000)

// Update these with values suitable for your network.
const char* ssid = "MBB_ZONG_340D";
const char* password = "alberteinstein1";
//const char* ssid = "93f49c";
//const char* password = "243848772";
const char* mqtt_server = "m11.cloudmqtt.com";
 
WiFiClient espClient;
PubSubClient client(espClient);
void setup_wifi(void);
void callback(char*, byte*, unsigned int);

long lastMsg = 0;
char msg[75];
long value = 0;
int pinDHT22 = 0;
SimpleDHT22 dht22;
uint8_t speed = 15;
unsigned long sw3Timer = 5*60*1000 ;
unsigned long motorOnTime = 0;
unsigned long sw3OnTime = 0;
boolean isMotorOn = false;
boolean isSw3On = false;
double position =0.0;   //current state of the plant
double setPoint = 26.0; //centrigrade
double drive = 0.0;   //that is to drive plant
double m = 0 ;      //slope 
double b = 0;       //y-intercept
double driveMin = 0;
double driveMax = 0; 
boolean isManual = false;
double gTemp = 0;
double gHum = 0;
//Data Structure for PI Control
 struct SPid{
  double iState;     //Integrator state; from -32767 to 32768
  double iMax,iMin ;// maximum and minimum allowable Integrator states
  double pGain;        //Proportional Gain; from 1 to 10 approx
  double iGain;     //Reciprocal of Integral e.g if iGain is .001 then we write it as 1000
};
SPid  pid;         //pid struct
double updatePID(SPid *pid,double error,double position){
   double pTerm, iTerm;
   pTerm = pid->pGain * error;
   pid->iState += error;
   if(pid->iState > pid->iMax){
      pid->iState = pid->iMax;
    }
   else if(pid->iState < pid->iMin){
      pid->iState = pid->iMin;
    }  
   iTerm = pid->iGain*pid->iState;
   if(iTerm>pid->iMax){
      iTerm = pid->iMax;
    }else if(iTerm < pid->iMin){
        iTerm = pid->iMin;
      }
   Serial.print("pGain: ");Serial.print(pid->pGain);Serial.print(", iGain:"); Serial.print(pid->iGain); Serial.print(", iState:"); Serial.println(pid->iState);
   Serial.print("setPoint:"); Serial.print(setPoint);Serial.print(", iMin: ");Serial.print(pid->iMin); Serial.print(",iMax: ");Serial.println(pid->iMax);
   Serial.print("pTerm: ");Serial.println(pTerm);
   Serial.print("iTerm: ");Serial.println(iTerm);
   return pTerm+iTerm;
}
void setMappingParameters(double i_Min,double i_Max,double drive_Min,double drive_Max){
    pid.iMin = i_Min; pid.iMax = i_Max; driveMin = drive_Min; driveMax = drive_Max;
  //  mapper(0,100000,255,0);
    mapper(pid.iMin,pid.iMax,driveMin,driveMax);
}

void setup() {
  pinMode(16, OUTPUT);      // pin 16 as output for Relay1
  pinMode(5, OUTPUT);       // pin 5 as output for Relay2
  pinMode(4, OUTPUT);       // pin 4 as output for Relay3
  pinMode(14, OUTPUT);      // pin 14 as output for Relay4
  pinMode(12, OUTPUT);      // pin 12 as output for Relay5
  Serial1.begin(4800);
  Serial.begin(115200);
   
   setup_wifi();
   client.setServer(mqtt_server, 13207);
   client.setCallback(callback);
  driveMin = 255;
  driveMax = 0;
  pid.iMax = 5000;
  pid.iMin = 0;
  pid.pGain = 200;
  pid.iGain = 1 ;
  setMappingParameters(pid.iMin,pid.iMax,driveMin,driveMax); //iMin,iMax,driveMin,driveMax
}
char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}
void resetMap(){
  setMappingParameters(pid.iMin,pid.iMax,driveMin,driveMax); //iMin,iMax,driveMin,driveMax
  } 
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

double drivePlant(double x){
  double z;
  if(x>pid.iMax){ 
    x = pid.iMax;
  }else if(x<pid.iMin){
    x = pid.iMin;
  }
  z = m*x+b;
  Serial.print("z = ");Serial.println(z);
  uint8_t cmd = (uint8_t)z;
  Serial.print("cmd: ");Serial.println(cmd);  
  return z;
}

void mapper(double fromMin,double fromMax,double toMin, double toMax){
 //   pid.iMin = fromMin; pid.iMax = fromMax; driveMin = toMin; driveMax = toMax;
    m  = (toMax-toMin)/(fromMax-fromMin);
    b = toMin-m*fromMin;
    Serial.print("m=");Serial.print(m);Serial.print(", b=");Serial.println(b);   
}

void setPIDParameters(double p,double i , double d){
  pid.pGain = p; pid.iGain = i;  
}
void readDHT22(){
 // read without samples.
  // @remark We use read2 to get a float data, such as 10.1*C
  //    if user doesn't care about the accurate data, use read to get a byte data, such as 10*C.
  float temperature = 0;
  float humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht22.read2(pinDHT22, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); Serial.println(err);delay(2000);
    return;
  }
  position = (double)temperature;
  
  //drive
 // Serial.print("Sample OK: ");
  Serial.print((float)temperature); Serial.print(" *C, ");
  Serial.print((float)humidity); Serial.println(" RH%");
  gHum = humidity;
  gTemp = temperature;
 // snprintf (msg, 75, "Temperature # %d", (int)temperature);
 // ftoa(msg,(double)temperature,2);
// Serial.println(msg);
//  client.publish("temp",msg);
  ftoa(msg,(double)humidity,2);
//  Serial.println(msg);
//  client.publish("hum",msg);
  // DHT22 sampling rate is 0.5HZ.
//  delay(1000);  
}
 
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  int match = strcmp(topic,"drive");
   char masg[length];
  for (int i = 0; i < length; i++) {
    masg[i]=(char)payload[i];
    Serial.print((char)payload[i]);
  }
  if(match == 0){
      Serial.println("drive selected");
      int valx = atoi(masg);
      Serial.printf("int value received is: %d\n",valx);
      uint8_t spx = valx;
      //speed = valx;
      Serial.printf("uint8_t value received is: %u\n",spx);
      Serial1.write(spx);
    }
  Serial.println();
  // Switch on the LED if an 1 was received as first character
  if (strcmp(topic,"sw1") == 0){
    if((char)payload[0] == '1'){ 
      digitalWrite(16, HIGH);   // Turn on the Relay1
      Serial.println("16 set");
    }else{
      digitalWrite(16,LOW);     // turn off the Relay1  
      Serial.println("16 reset");
    }
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  }else if(strcmp(topic,"sw2") == 0){
     if((char)payload[0] == '1'){ 
      digitalWrite(5, HIGH);   // Turn on the Relay1
      Serial.println("5 set");
    }else{
      digitalWrite(5,LOW);     // turn off the Relay1
      Serial.println("5 reset");  
    } 
  }else if(strcmp(topic,"sw3") == 0){
    if((char)payload[0] == '1'){
        digitalWrite(4,HIGH);
        sw3OnTime = millis();
        isSw3On = true;
        Serial.println("4 set");
      }else{
        digitalWrite(4,LOW);
        isSw3On = false;
        Serial.println("4 reset");
      }
  }else if(strcmp(topic,"sw4") == 0){
    if((char)payload[0] == '1'){
        digitalWrite(14,HIGH);
        Serial.println("14 set");
      }else{
        digitalWrite(14,LOW);
        Serial.println("14 reset");
      }  
  }else if(strcmp(topic,"sw5") == 0){
    if((char)payload[0] == '1'){
        digitalWrite(12,HIGH);
        motorOnTime = millis();
        isMotorOn = true;
        Serial.println("12 set");
      }else{
        digitalWrite(12,LOW);
        isMotorOn = false;
        Serial.println("12 reset");
      }  
  }else if(strcmp(topic,"pGain") == 0){
      Serial.println(masg);
      int valx = atoi(masg);
      pid.pGain = (double)valx;
      Serial.printf("int value received is: %d\n",valx);
      Serial.println(pid.pGain);  
  }else if(strcmp(topic,"iGain") == 0){
      Serial.println(masg);
      float valx = atof(masg);
      pid.iGain = (double)valx;
      Serial.printf("int value received is: %d\n",valx);
      Serial.println(pid.iGain);  
  }else if(strcmp(topic,"iMin") == 0){
       Serial.println(masg);
      int valx = atoi(masg);
      pid.iMin = (double)valx;
      Serial.printf("int value received is: %d\n",valx);
      Serial.println(pid.iMin); 
  }else if(strcmp(topic,"iMax") == 0){
       Serial.println(masg);
      int valx = atoi(masg);
      pid.iMax = (double)valx;
      Serial.printf("int value received is: %d\n",valx);
      Serial.println(pid.iMax);
      resetMap();  
  }else if(strcmp(topic,"dGain") == 0){
      Serial.println(masg);  
  }else if(strcmp(topic,"setPoint") == 0){
       Serial.println(masg);
      int valx = atoi(masg);
      setPoint = (double)valx;
      Serial.printf("int value received is: %d\n",valx);
      Serial.println(setPoint);  
  }
}

  
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("orgo-12345","ali","123")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
//      client.publish("anounce", "online");
      // ... and resubscribe
      
//      client.subscribe("sw1");  //1,0
//      client.subscribe("sw2");
//      client.subscribe("sw3");
//      client.subscribe("sw4");
      client.subscribe("sw5");
      client.subscribe("setPoint");  
      client.subscribe("manual");
      client.subscribe("drive");  //Plant drive from 0 255
//      client.subscribe("iMin");
      client.subscribe("iMax");
//      client.subscribe("driveMin");
//      client.subscribe("driveMax");
      client.subscribe("pGain");
      client.subscribe("iGain");
//      client.subscribe("dGain");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 3 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if (millis() - lastMsg > 30000) {   //millis() - lastMsg > 5000
      lastMsg = millis();
      ++value;
      snprintf (msg, 75, " serial :%ld, at millis():%ld", value , millis());
//    Serial.print("Publish message: ");
      Serial.println(msg);
      ftoa(msg,(double)gTemp,2);
      client.publish("temp", msg);
      ftoa(msg,(double)gHum,2);
      client.publish("hum",msg);
    //Serial1.write(speed);
    }
    /*******
    if(isMotorOn && (millis()-motorOnTime >= MOTOR_TIMER)){
        isMotorOn = false;
        Serial.print("Turn Off motor");
        client.publish("sw5","0");
      }
    if(isSw3On && (millis()-sw3OnTime >= sw3Timer)){
        isSw3On = false;
        Serial.print("Turn off sw3");
        client.publish("sw3","0");
    }
    *******/
   readDHT22();
   Serial.print("setPoint"); Serial.println(setPoint);
   Serial.print("position:"); Serial.println(position);
   Serial.print("error"); Serial.println(setPoint-position);
   drive = updatePID(&pid,setPoint-position,position);
   Serial.print("drive:");Serial.println(drive);
   drivePlant(drive);
   delay(1000);
}
