#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <EEPROM.h> 
#include <ESP_FlexyStepper.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

const char* ssid = "MMV_1";
const char* password = "2Pomad@s";


//MQTT configuration
#define MQTT_HOST IPAddress(192, 168, 1, 100)
#define MQTT_PORT 1883
#define mqtt_user "hass"
#define mqtt_password "mqtt"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;




//JSON to store values to send over MQTT
StaticJsonDocument<256> FeedbackData;
StaticJsonDocument<256> JSONReceived;


// ---------------------- SPECIFIC DEFINITIONS FOR SENSORS IN THIS PROJECT -----------------------------------


//*********Declaring Pins ************************************//

int Dir1 = GPIO_NUM_16;                     //Steppers Pin configuration
int Dir2 = GPIO_NUM_27;
int Dir3 = GPIO_NUM_14;
int Step1 = GPIO_NUM_26;
int Step2 = GPIO_NUM_25;
int Step3 = GPIO_NUM_17;
int EnableStepper = 12;            //enable-low or disable-high stepper drivers


//*********Declaring Variables ************************************//
const char* Action;
int Channel;
float VolumeMl1;
float VolumeMl2;
float VolumeMl3;
float SpeedMlperMin1;
float SpeedMlperMin2;
float SpeedMlperMin3;
const char* Direction1;
const char* Direction2;
const char* Direction3;
long StepsPerMili1;
long StepsPerMili2;
long StepsPerMili3;
long StepsToMove1;
long StepsToMove2;
long StepsToMove3;
long SpeedToMove1;
long SpeedToMove2;
long SpeedToMove3;
long InitialSteps1;
long InitialSteps2;
long InitialSteps3;
long TargetSteps1;
long TargetSteps2;
long TargetSteps3;
int StepperStopped;
byte ChannelStopped;
bool Stepper1Running = false;
bool Stepper2Running = false;
bool Stepper3Running = false;


//EEPROM addresses - Size of uint32_t is 4 positions. All varibles stored in EEPROM are uint32_t type
#define EEPROM_SIZE 512
int EepromStepsPerMili1 = 0;                   //Steps per mililitre in Stepper 1
int EepromStepsPerMili2 = 10;                   //Steps per mililitre in Stepper 2
int EepromStepsPerMili3 = 20;                   //Steps per mililitre in Stepper 3



// Speed settings
const int SPEED_IN_STEPS_PER_SECOND = 2000;
const int ACCELERATION_IN_STEPS_PER_SECOND = 800;
const int DECELERATION_IN_STEPS_PER_SECOND = 800;

// create the stepper motor object
ESP_FlexyStepper stepper1;
ESP_FlexyStepper stepper2;
ESP_FlexyStepper stepper3;

// delays
long ProgressPreviousMillis = 0;                  //variables for delayed broacasting of Progress
long ProgressTime = 5000;                         //     "
long ProgressCurrentMillis;                       //     "

//Core 0
TaskHandle_t C0;

// ------------- FUNCTIONS ----------------



void connectToWifi ()
{
  Serial.println("Booting");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //begin WiFi connection
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}


void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}


void SerialSetup()
{
  // Initialize Serial port

  Serial.begin(115200);


  WiFi.onEvent(WiFiEvent);
  
}




void OTASetup()
{


  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("Peristaltica");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");


  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
    
}



void targetPositionReachedCallback()
{     
    

    //disable steppers if all are not moving
    if (Stepper1Running == false && Stepper2Running == false && Stepper3Running == false)
    {
      digitalWrite(EnableStepper, HIGH);  
    }	
  
    
    char tempJsonStringDone[256];    
    StaticJsonDocument<256> ResponseDone;
    ResponseDone["type"] = "done";
    ResponseDone["action"] = "run";
    ResponseDone["channel"] = ChannelStopped;
    ResponseDone["progress"] = 100;  

    size_t n = serializeJson(ResponseDone, tempJsonStringDone);    
    uint16_t packetIdPubDN = mqttClient.publish("peristaltica/status", 1, true, tempJsonStringDone);

  //  Serial.print("targetPositionReachedCallback: ");
  //  Serial.println(tempJsonStringDone);  

}

void targetPositionReachedCallbackStepper1(long position)
{
  Stepper1Running = false;
  ChannelStopped = 1;
  targetPositionReachedCallback();

}
void targetPositionReachedCallbackStepper2(long position)
{
  Stepper2Running = false;
  ChannelStopped = 2;  
  targetPositionReachedCallback();
 
}
void targetPositionReachedCallbackStepper3(long position)
{
  Stepper3Running = false;
  ChannelStopped = 3;
  targetPositionReachedCallback();
}


void emergencyStopTriggerdCallbackFunction ()
{     
  //disable steppers if all are not moving
  if (Stepper1Running == false && Stepper2Running == false && Stepper3Running == false)
  {
    digitalWrite(EnableStepper, HIGH);  
  }	

  int progressStop;

  if (StepperStopped == 1){
      long CurrentSteps1;
      CurrentSteps1 = stepper1.getCurrentPositionInSteps() ;
      if (Stepper1Running == true){
        progressStop = round(100 * (CurrentSteps1 - InitialSteps1)/(TargetSteps1 - InitialSteps1));
      }
        
  }
  else if (StepperStopped == 2){
      long CurrentSteps2;
      CurrentSteps2 = stepper2.getCurrentPositionInSteps() ;
      if (Stepper2Running == true){
        progressStop = round(100 * (CurrentSteps2 - InitialSteps2)/(TargetSteps2 - InitialSteps2));
      }
  }
  else if (StepperStopped == 3){    
      long CurrentSteps3;
      CurrentSteps3 = stepper3.getCurrentPositionInSteps() ;
      if (Stepper3Running == true){
        progressStop = round(100 * (CurrentSteps3 - InitialSteps3)/(TargetSteps3 - InitialSteps3));
      }
  }


    char tempJsonStringDone[256];    
    StaticJsonDocument<256> ResponseDone;
    ResponseDone["type"] = "done";
    ResponseDone["action"] = "stop";
    ResponseDone["channel"] = StepperStopped;
    ResponseDone["progress"] = progressStop;  

    size_t n = serializeJson(ResponseDone, tempJsonStringDone);    
    uint16_t packetIdPubD = mqttClient.publish("peristaltica/status", 1, true, tempJsonStringDone);

  //  Serial.print("emergencyStopTriggerdCallbackFunction: ");
  //  Serial.println(tempJsonStringDone); 

}


void checkProgress()
{

  ProgressCurrentMillis = millis();
  
  if (ProgressCurrentMillis - ProgressPreviousMillis > ProgressTime)
  {
    ProgressPreviousMillis = ProgressCurrentMillis; 
    //Serial.println("Progress...");
  
    if (Stepper1Running == true)
    {
      
      long CurrentSteps1;
      int Progress1;
      CurrentSteps1 = stepper1.getCurrentPositionInSteps() ;
      Progress1 = round(100 * (CurrentSteps1 - InitialSteps1)/(TargetSteps1 - InitialSteps1));
          
      StaticJsonDocument<256> Response1;
      Response1["type"] = "running";
      Response1["action"] = "run";
      Response1["channel"] = 1;
      Response1["progress"] = Progress1;
      char tempJsonString1[256];
      size_t n1 = serializeJson(Response1, tempJsonString1);    
      uint16_t packetIdPub1 = mqttClient.publish("peristaltica/status", 1, true, tempJsonString1);

    //Serial.print("Progress Stepper 1... ");
    //Serial.println(tempJsonString1);
    }

    if (Stepper2Running == true)
    {
      long CurrentSteps2;
      int Progress2;
      CurrentSteps2 = stepper2.getCurrentPositionInSteps() ;
      Progress2 = round(100 * (CurrentSteps2 - InitialSteps2)/(TargetSteps2 - InitialSteps2));

      StaticJsonDocument<256> Response2;
      Response2["type"] = "running";
      Response2["action"] = "run";
      Response2["channel"] = 2;
      Response2["progress"] = Progress2;
      char tempJsonString2[256];
      size_t n2 = serializeJson(Response2, tempJsonString2);    
      uint16_t packetIdPub2 = mqttClient.publish("peristaltica/status", 1, true, tempJsonString2);

    //Serial.print("Progress Stepper 2... ");
    //Serial.println(tempJsonString2);
    }

    if (Stepper3Running == true)
    {
      long CurrentSteps3;
      int Progress3;
      CurrentSteps3 = stepper3.getCurrentPositionInSteps() ;
      Progress3 = round(100 * (CurrentSteps3 - InitialSteps3)/(TargetSteps3 - InitialSteps3));

      StaticJsonDocument<256> Response3;
      Response3["type"] = "running";
      Response3["action"] = "run";
      Response3["channel"] = 3;
      Response3["progress"] = Progress3;
      char tempJsonString3[256];
      size_t n3 = serializeJson(Response3, tempJsonString3);    
      uint16_t packetIdPub3 = mqttClient.publish("peristaltica/status", 1, true, tempJsonString3);

    //Serial.print("Progress Stepper 3... ");
    //Serial.println(tempJsonString3);
    }

  }

    //disable steppers if all are not moving
    if (Stepper1Running == false && Stepper2Running == false && Stepper3Running == false)
    {
      digitalWrite(EnableStepper, HIGH);  
    }	
 

}


void ParseJSONMessage()
{

  Action = JSONReceived["action"];
  if (String(Action) == "run"){
      Channel = JSONReceived["channel"];
      switch (Channel) {
        case 1:
          VolumeMl1 = JSONReceived["volume"];
          SpeedMlperMin1 = JSONReceived["speed"];
          Direction1 = JSONReceived["direction"];

          break;
        case 2:
          VolumeMl2 = JSONReceived["volume"];
          SpeedMlperMin2 = JSONReceived["speed"];
          Direction2 = JSONReceived["direction"];
          break;
        case 3:
          VolumeMl3 = JSONReceived["volume"];
          SpeedMlperMin3 = JSONReceived["speed"];
          Direction3 = JSONReceived["direction"];
          break;
        default:
          //
          break;
      }
        
  }
  else if (String(Action) == "calibrate"){
      
      Channel = JSONReceived["channel"];
      switch (Channel) {
        case 1:
          StepsPerMili1 = JSONReceived["stepsperml"];
          break;
        case 2:
          StepsPerMili2 = JSONReceived["stepsperml"];
          break;
        case 3:
          StepsPerMili3 = JSONReceived["stepsperml"];
          break;
        default:
          //
          break;
      }
      
      
        
  }
  else if (String(Action) == "stop"){
      Channel = JSONReceived["channel"];              

  }


}


void CallAction ()
{
  if (String(Action) == "run"){ 
    
     
     if (Channel == 1){
  
        digitalWrite(EnableStepper, LOW);
        if (String(Direction1) == "ccw"){
            VolumeMl1 = - VolumeMl1;
        }  
        SpeedToMove1 = round(SpeedMlperMin1 * StepsPerMili1 / 60); //steps per second
        StepsToMove1 =   VolumeMl1 * StepsPerMili1;
        Stepper1Running = true;
        stepper1.setSpeedInStepsPerSecond(SpeedToMove1);
        stepper1.setTargetPositionRelativeInSteps(StepsToMove1);
        InitialSteps1 = stepper1.getCurrentPositionInSteps() ;
        TargetSteps1 = InitialSteps1 + StepsToMove1;
          
      }
      else if (Channel == 2){
        
        digitalWrite(EnableStepper, LOW);
        if (String(Direction2) == "ccw"){
            VolumeMl2 = - VolumeMl2;
        }  
        SpeedToMove2 = round(SpeedMlperMin2 * StepsPerMili2 / 60);
        StepsToMove2 =   VolumeMl2 * StepsPerMili2;
        Stepper2Running = true;        
        stepper2.setSpeedInStepsPerSecond(SpeedToMove2);
        stepper2.setTargetPositionRelativeInSteps(StepsToMove2);
        InitialSteps2 = stepper2.getCurrentPositionInSteps() ;
        TargetSteps2 = InitialSteps2 + StepsToMove2;

      }
      else if (Channel == 3){

        digitalWrite(EnableStepper, LOW);
        if (String(Direction3) == "ccw"){
            VolumeMl3 = - VolumeMl3;
        }  
        SpeedToMove3 = round(SpeedMlperMin3 * StepsPerMili3 / 60);
        StepsToMove3 =   VolumeMl3 * StepsPerMili3;
        Stepper3Running = true;        
        stepper3.setSpeedInStepsPerSecond(SpeedToMove3);
        stepper3.setTargetPositionRelativeInSteps(StepsToMove3);
        InitialSteps3 = stepper3.getCurrentPositionInSteps() ;
        TargetSteps3 = InitialSteps3 + StepsToMove3;

      }



  }
  else if (String(Action) == "calibrate"){

     StaticJsonDocument<256> CalibrateFeedbackData;
     if (Channel == 1){
              
        EEPROM.begin(EEPROM_SIZE);
        EEPROM.write (EepromStepsPerMili1, StepsPerMili1);
        EEPROM.commit();

        CalibrateFeedbackData["channel"] = 1;
            
      }
      else if (Channel == 2){

        EEPROM.begin(EEPROM_SIZE);
        EEPROM.write (EepromStepsPerMili2, StepsPerMili2);
        EEPROM.commit();

        CalibrateFeedbackData["channel"] = 2;

      }
      else if (Channel == 3){
        
        EEPROM.begin(EEPROM_SIZE);
        EEPROM.write (EepromStepsPerMili3, StepsPerMili3);
        EEPROM.commit();

        CalibrateFeedbackData["channel"] = 3;

      }     
         
        CalibrateFeedbackData["action"] = "calibrate";
        CalibrateFeedbackData["type"] = "done";
          
     
        char tempJsonStringC[256];
        size_t nc = serializeJson(CalibrateFeedbackData, tempJsonStringC);    
        uint16_t packetIdPubC = mqttClient.publish("peristaltica/status", 1, true, tempJsonStringC);

//    Serial.print("Calibrate: ");
//    Serial.println(tempJsonStringC);    
  
  }
  else if (String(Action) == "stop"){

      if (Channel == 0){
        StepperStopped = 0;
        Stepper1Running = false;
        Stepper2Running = false;
        Stepper2Running = false;
        stepper1.emergencyStop();
        stepper2.emergencyStop();
        stepper3.emergencyStop();

        digitalWrite(EnableStepper, HIGH);
      
      }
      else if (Channel == 1){
        StepperStopped = 1;
        Stepper1Running = false;
        stepper1.emergencyStop();

            
      }
      else if (Channel == 2){
        StepperStopped = 2;
        Stepper2Running = false;
        stepper2.emergencyStop();
            
      }
      else if (Channel == 3){
        StepperStopped = 3;
        Stepper3Running = false;
        stepper3.emergencyStop();
            
      }

  }
  else if (String(Action) == "params"){

        StaticJsonDocument<256> ResponseFeedbackData;
        ResponseFeedbackData["action"] = "params";
        ResponseFeedbackData["type"] = "done";
        ResponseFeedbackData["StepsPerMili1"] = StepsPerMili1;
        ResponseFeedbackData["StepsPerMili2"] = StepsPerMili2;
        ResponseFeedbackData["StepsPerMili3"] = StepsPerMili3;   

        char tempJsonStringR[256];
        size_t nr = serializeJson(ResponseFeedbackData, tempJsonStringR);    
        uint16_t packetIdPubR = mqttClient.publish("peristaltica/status", 1, true, tempJsonStringR);
  
  }

}



void callback(char* topic, char* payload, unsigned int length) {
 
   JSONReceived="";
   DeserializationError errorDes = deserializeJson(JSONReceived, payload, length);
    if (errorDes) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(errorDes.f_str());
        return;
    }
    else
    {     
      ParseJSONMessage();           //Extract values from incoming JSON to variables
      CallAction();                 //Perform actions on Steppers
    } 
}


void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe("peristaltica/action", 1);
  Serial.print("Subscribing at QoS 1, packetId: ");
  Serial.println(packetIdSub);
  
  uint16_t packetIdPubR = mqttClient.publish("peristaltica/status", 1, true, "Connected to MQTT");
/*
  mqttClient.publish("test/lol", 0, true, "test 1");
  Serial.println("Publishing at QoS 0");
  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);
*/
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}


void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    
  /*
    Serial.println("Publish received.");
    Serial.print("  topic: ");
    Serial.println(topic);
    Serial.print("  payload: ");
    Serial.println(payload);
    Serial.print("  qos: ");
    Serial.println(properties.qos);
    Serial.print("  dup: ");
    Serial.println(properties.dup);
    Serial.print("  retain: ");
    Serial.println(properties.retain);
    Serial.print("  len: ");
    Serial.println(len);
    Serial.print("  index: ");
    Serial.println(index);
    Serial.print("  total: ");
    Serial.println(total);
  */
    callback(topic, payload, len);

}

void onMqttPublish(uint16_t packetId) {
/*
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
*/
}


void MQTTSetup()
{
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(mqtt_user, mqtt_password);

  
}



void EepromRead()            
{
  EEPROM.begin(EEPROM_SIZE);
  
  StepsPerMili1 = EEPROM.read( EepromStepsPerMili1);
  StepsPerMili2 = EEPROM.read( EepromStepsPerMili2);
  StepsPerMili3 = EEPROM.read( EepromStepsPerMili3);
  
  EEPROM.end();    

}

void StepperSetup()
{

  pinMode (Dir1, OUTPUT); pinMode (Step1, OUTPUT);
  pinMode (Dir2, OUTPUT); pinMode (Step2, OUTPUT);
  pinMode (Dir3, OUTPUT); pinMode (Step3, OUTPUT);
  pinMode (EnableStepper, OUTPUT); digitalWrite(EnableStepper, HIGH);
  
  stepper1.connectToPins(Step1, Dir1);
  stepper1.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
  stepper1.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  stepper1.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
  stepper1.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper1);
  stepper1.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

  stepper2.connectToPins(Step2, Dir2);
  stepper2.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
  stepper2.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  stepper2.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
  stepper2.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper2);
  stepper2.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

  stepper3.connectToPins(Step3, Dir3);
  stepper3.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
  stepper3.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  stepper3.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
  stepper3.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper3);
  stepper3.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

  stepper1.startAsService(1);
  stepper2.startAsService(1);
  stepper3.startAsService(1);
}



void core0assignments( void * pvParameters ) { 
for (;;) {
  
  ArduinoOTA.handle();
  
  }
}


void setup() {
  
  disableCore0WDT();            //disable the watchdog time on core 0

  xTaskCreatePinnedToCore(
   core0assignments,        // Function that should be called
   "Core_0",          // Name of the task (for debugging)
   10000,                   // Stack size (bytes)
   NULL,                   // Parameter to pass
   1,                      // Task priority//0
   &C0,                 // Task handle
   0);                     // Core you want to run the task on (0 or 1)
  


  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  SerialSetup();
  MQTTSetup();                 //Configure MQTT broker  
  connectToWifi();                 // Start a Wi-Fi access point, and try to connect 
  OTASetup();                  //Start Over The Air updater service
  StepperSetup();              //Configure Stepper motors
  EepromRead();                //retrieve data from EEPROM
}


void loop() {
  
  checkProgress();

}