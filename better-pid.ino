#include <ArduinoQueue.h>

#include <ESPAsyncWebServer.h>

#include <WiFi.h>
#include <WiFiMulti.h>

#include <DallasTemperature.h>

#include <OneWire.h>

#define RELAY 17
#define RELAY2 18
#define TEMP 16

uint8_t sensor1ID[8] = {0x28, 0xFF, 0x64, 0x1F, 0x78, 0x74, 0x68, 0x04};
uint8_t sensor2ID[8] = {0x28, 0xFF, 0x64, 0x1F, 0x78, 0x68, 0xB1, 0x6E};

OneWire oneWire(TEMP);
DallasTemperature sensorLine(&oneWire);



class TemperatureSensor {
private:
  float temperature;
  SemaphoreHandle_t temperatureMutex;

public:
  uint8_t id[8];
  DeviceAddress address;
  DallasTemperature *line;
  SemaphoreHandle_t *lineMutex;
  TemperatureSensor(uint8_t sensorID[8], DallasTemperature *line, SemaphoreHandle_t *lineMutex){
    temperatureMutex = xSemaphoreCreateMutex();
    memcpy(id, sensorID, 8);
    memcpy(address, sensorID, sizeof(address));
    this->line = line;
    this->lineMutex = lineMutex;
  }
  float getTemperature(){
    float ret = NULL;
    if(xSemaphoreTake(temperatureMutex, portMAX_DELAY)){
      ret = temperature;

      xSemaphoreGive(temperatureMutex);
    }

    return ret;
  }
  void setTemperature(float temp) {
    if(xSemaphoreTake(temperatureMutex, portMAX_DELAY)){
      temperature = temp;

      xSemaphoreGive(temperatureMutex);
    }
  }
  float fetchTemperature(DallasTemperature *sensorLine, SemaphoreHandle_t *lineMutex) {
    float recv = -121;
    if(xSemaphoreTake(*lineMutex, portMAX_DELAY)){
      recv = sensorLine->getTempC(address);
      xSemaphoreGive(*lineMutex);
    }
    setTemperature(recv);
    return recv;
  }
};

struct Settings {
  unsigned int min_off_time;
  unsigned int min_on_time;

  float target_temp;

  bool heating; // is the outlet heating or cooling (not heating)?  bool relayState = LOW;
  float Kp;
  float Kd;
  float Ki;
  float Kdheating;
  float Kdcooling;
};

class System {
  public:
  Settings settings;
  float integral;
  float previousError;
  unsigned long previousTime;
  unsigned long lastChangeTime = 0;
  TemperatureSensor *sensor;
  bool relayState;
  int relayPin;
  ArduinoQueue<float> tempQueue;
  ArduinoQueue<unsigned long> timeQueue;

  System(Settings *startSettings, TemperatureSensor *sensor) {
    this->sensor = sensor;
    memcpy(&settings, startSettings, sizeof(Settings));
    ArduinoQueue<float> tempQueue(10);
    ArduinoQueue<unsigned long> timeQueue(10);

    previousTime = 0;
  }

  void changeState(bool newstate){
    // if the new state is different, check if its been long enough since last state change
    if (relayState == newstate) {
      return;
    }
    unsigned int mintime = 0;
    unsigned long curtime = millis();
    if (relayState == LOW) {
      mintime = settings.min_off_time;
    } else {
      mintime = settings.min_on_time;
    }

    if (curtime - lastChangeTime >= mintime) {
      lastChangeTime = curtime;
      relayState = newstate;
    }
  }

  float getTemperature(DallasTemperature *sensorLine, SemaphoreHandle_t *sensorMutex) {
    return sensor->fetchTemperature(sensorLine, sensorMutex);
  }

  float pid(float error, float old_error, unsigned long curtime, unsigned long old_time) {
    float porportional = error;

    float derivative = 0;
    unsigned long dt = curtime - old_time;
    derivative = (error - old_error)/dt;

    float Kd = 0;
    if (derivative > 0) { // error rising, meaning temp is cooling
      Kd = this->settings.Kdcooling;
    } else {
      Kd = this->settings.Kdheating;
    }

    this->integral += error * (previousTime/1000.0);

    //TODO: maybe throw out integral on large errors?
    
    float output = this->settings.Kp * porportional + Kd * derivative + this->settings.Ki * this->integral;
    return output;
  }

  void changeRelays() {
    // the states on my relay are reversed for some reason
    if (relayState == LOW) {
      digitalWrite(relayPin, HIGH);
    } else {
      digitalWrite(relayPin, LOW);
    }
  }

  void processSystem() {
    unsigned long curtime = millis();
    float temp = this->getTemperature(this->sensor->line, this->sensor->lineMutex);
    float old_error = previousError;
    float old_time = previousTime;
    float error = this->settings.target_temp - temp;
    if (tempQueue.isFull()){
      float old_temp = tempQueue.dequeue();
      old_error = settings.target_temp - old_temp;
    }
    if (timeQueue.isFull()) {
      old_time = timeQueue.dequeue();
    }

    tempQueue.enqueue(temp);
    timeQueue.enqueue(curtime);

    float output = this->pid(error, old_error, curtime, old_time);

    bool relayvalue = outputToBin(output);
    this->changeState(relayvalue);
    changeRelays();

    previousError = error;
    previousTime = curtime;
  }

  bool outputToBin(float output) {
    if (output > 0) { // read colder than target temperature
      return settings.heating;
    } else {
      return !settings.heating;
    }
  }
};



WiFiMulti wifiMulti;
const int networks = 1;
const char* ssids[] = {"Vikurbakki18"};
const char* passwords[] = {"Fletturimi35"};


AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
    <head>
        <title>Temperature controller</title>
    </head>
    <body>
        %PAGEPLACEHOLDER%
    </body>
</html>

)rawliteral";


// TODO: add the site processor after all has been fixed


///// NOTE: initializing the sensors and the systems

Settings heatingsettings = {
  10000,
  10000,
  67,
  true,
  1,
  0,
  0,
  0,
  0
};



SemaphoreHandle_t sensorLineMutex;

void setup() {
  Serial.begin(9600);
  Serial.println("started");
  sensorLine.begin();
  sensorLine.setResolution(sensor1ID, 9);

  sensorLineMutex = xSemaphoreCreateMutex();

  TemperatureSensor sensor1(sensor1ID, &sensorLine, &sensorLineMutex);
  System system1(&heatingsettings, &sensor1);
  
  pinMode(RELAY, OUTPUT);

  WiFi.begin(ssids[0], passwords[0]);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("\nWiFi connected.\nIP address: ");
  Serial.println(WiFi.localIP());
}

void temperatureLoop() {
  if(xSemaphoreTake(sensorLineMutex, portMAX_DELAY)) {
    sensorLine.requestTemperatures();

    xSemaphoreGive(sensorLineMutex);
  }

  unsigned long curtime = millis();
}

unsigned long pasttime = 0;
void loop() {
  unsigned long curtime = millis();
  if (curtime - pasttime > 2000) {
    temperatureLoop();
  }
  pasttime = curtime;
}
