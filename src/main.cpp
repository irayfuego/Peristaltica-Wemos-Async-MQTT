#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h> 
#include <ArduinoJson.h>
#include <EEPROM.h> 
#include <ESP_FlexyStepper.h>

const char* ssid = "AMARBA_1";
const char* password = "Travelguide0";


//MQTT configuration
#define mqtt_server "192.168.1.14"
#define mqtt_user "Peristaltica"
#define mqtt_password "mqtt"
String mqtt_client_id="Peristaltica-";  

//MQTT client
WiFiClient espClient;
PubSubClient mqtt_client(espClient);



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
long ProgressTime = 1000;                         //     "
long ProgressCurrentMillis;                       //     "



// ------------- FUNCTIONS ----------------



void mqtt_reconnect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    //Serial.print("Attempting MQTT connection...");

    if (mqtt_client.connect(mqtt_client_id.c_str(), mqtt_user, mqtt_password)) {
      //Serial.println("connected");
      mqtt_client.subscribe("peristaltica/action");

    } else {
      //Serial.print("failed, rc=");
      //Serial.print(mqtt_client.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqtt_reconnect();
}



void WifiSetup ()
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

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("");
      Serial.print("Connected to ");
      Serial.println(ssid);
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      mqtt_reconnect();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      WifiSetup ();
      mqtt_reconnect();
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



void targetPositionReachedCallback(long position)
{     
    
    byte ChannelStopped;
    //Identify which stepper has stopped
    if (stepper1.motionComplete() && Stepper1Running == true)
      {
        Stepper1Running = false;
        ChannelStopped = 1;

      }
    if (stepper2.motionComplete() && Stepper2Running == true)
      {
        Stepper2Running = false;
        ChannelStopped = 2;

      }
    if (stepper3.motionComplete() && Stepper3Running == true)
      {
        Stepper3Running = false;
        ChannelStopped = 3;

      }

    //disable steppers if all are not moving
    if (stepper1.motionComplete())
    {
      if (stepper2.motionComplete())
      {
        if (stepper3.motionComplete())
        {
          digitalWrite(EnableStepper, HIGH);  

        }
      }
    }	
  
    
    char tempJsonStringDone[256];    
    StaticJsonDocument<256> ResponseDone;
    ResponseDone["type"] = "done";
    ResponseDone["action"] = "run";
    ResponseDone["channel"] = ChannelStopped;
    ResponseDone["progress"] = 100;  

    size_t n = serializeJson(ResponseDone, tempJsonStringDone);    
    mqtt_client.publish("peristaltica/status", tempJsonStringDone, n);

}



void emergencyStopTriggerdCallbackFunction ()
{     

    if (Stepper1Running == false)
    {
      if (Stepper2Running == false)
      {
        if (Stepper3Running == false)
        {
          digitalWrite(EnableStepper, HIGH);  //disable steppers if all are not moving

        }
      }
    }	

  int progressStop;

  if (StepperStopped == 1){
      long CurrentSteps1;
      CurrentSteps1 = stepper1.getCurrentPositionInSteps() ;
      progressStop = round(100 * (CurrentSteps1 - InitialSteps1)/(TargetSteps1 - InitialSteps1));

        
  }
  else if (StepperStopped == 2){
      long CurrentSteps2;
      CurrentSteps2 = stepper2.getCurrentPositionInSteps() ;
      progressStop = round(100 * (CurrentSteps2 - InitialSteps2)/(TargetSteps2 - InitialSteps2));

  }
  else if (StepperStopped == 3){    
      long CurrentSteps3;
      CurrentSteps3 = stepper3.getCurrentPositionInSteps() ;
      progressStop = round(100 * (CurrentSteps3 - InitialSteps3)/(TargetSteps3 - InitialSteps3));

  }


    char tempJsonStringDone[256];    
    StaticJsonDocument<256> ResponseDone;
    ResponseDone["type"] = "done";
    ResponseDone["action"] = "stop";
    ResponseDone["channel"] = StepperStopped;
    ResponseDone["progress"] = progressStop;  

    size_t n = serializeJson(ResponseDone, tempJsonStringDone);    
    mqtt_client.publish("peristaltica/status", tempJsonStringDone, n);

}


void checkProgress()
{

  ProgressCurrentMillis = millis();
  
  if (ProgressCurrentMillis - ProgressPreviousMillis > ProgressTime)
  {
    ProgressPreviousMillis = ProgressCurrentMillis; 

  
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
      mqtt_client.publish("peristaltica/status", tempJsonString1, n1);

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
      mqtt_client.publish("peristaltica/status", tempJsonString2, n2);

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
      mqtt_client.publish("peristaltica/status", tempJsonString3, n3);

    }

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
    
      digitalWrite(EnableStepper, LOW);
     
     if (Channel == 1){
  
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
        mqtt_client.publish("peristaltica/status", tempJsonStringC, nc);

      
  
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
        mqtt_client.publish("peristaltica/status", tempJsonStringR, nr);


  }

}



void callback(char* topic, byte* payload, unsigned int length) {
 
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





void MQTTSetup()
{
  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.subscribe("peristaltica/action");
  mqtt_client.setCallback(callback);
  
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
  stepper1.registerTargetPositionReachedCallback(targetPositionReachedCallback);
  stepper1.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

  stepper2.connectToPins(Step2, Dir2);
  stepper2.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
  stepper2.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  stepper2.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
  stepper2.registerTargetPositionReachedCallback(targetPositionReachedCallback);
  stepper2.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

  stepper3.connectToPins(Step3, Dir3);
  stepper3.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
  stepper3.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  stepper3.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
  stepper3.registerTargetPositionReachedCallback(targetPositionReachedCallback);
  stepper3.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

  stepper1.startAsService(0);
  stepper2.startAsService(0);
  stepper3.startAsService(0);
}




void setup() {

  SerialSetup();
  MQTTSetup();                 //Configure MQTT broker  
  WifiSetup();                 // Start a Wi-Fi access point, and try to connect 
  OTASetup();                  //Start Over The Air updater service
  StepperSetup();              //Configure Stepper motors
  EepromRead();                //retrieve data from EEPROM
}


void loop() {
  ArduinoOTA.handle();
  
  if (!mqtt_client.connected()) {
    mqtt_reconnect();
  }
  mqtt_client.loop();

  checkProgress();

}