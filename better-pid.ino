#include <ArduinoQueue.h>

#include <ESPAsyncWebServer.h>

#include <WiFi.h>
#include <WiFiMulti.h>

#include <DallasTemperature.h>

#include <OneWire.h>

#define APMODE true

#define RELAY 17
#define RELAY2 18
#define TEMP 16

uint8_t sensor1ID[8] = {0x28, 0xFF, 0x64, 0x1F, 0x78, 0x74, 0x68, 0x04};
uint8_t sensor2ID[8] = {0x28, 0xFF, 0x64, 0x1F, 0x78, 0x68, 0xB1, 0x6E};

OneWire oneWire(TEMP);
DallasTemperature sensorLine(&oneWire);

bool debug = true;



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
    if(debug) {
      Serial.printf("porportional: %.4f, derivative: %.4f, integral: %.4f\n", porportional, derivative, integral);
      Serial.printf("Kp*porportional: %.4f + Kd*derivative: %.4f + Ki*integral: %.4f\noutput = %.4f\n", this->settings.Kp * porportional, Kd * derivative, this->settings.Ki * this->integral, output);
    }
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
TemperatureSensor sensor1(sensor1ID, &sensorLine, &sensorLineMutex);
System system1(&heatingsettings, &sensor1);

TemperatureSensor sensor2(sensor2ID, &sensorLine, &sensorLineMutex);
System system2(&heatingsettings, &sensor2);
  
void setup() {
  Serial.begin(9600);
  Serial.println("started");
  sensorLine.begin();
  sensorLine.setResolution(sensor1ID, 9);

  sensorLineMutex = xSemaphoreCreateMutex();


  pinMode(RELAY, OUTPUT);

  if (!APMODE){ // Normal mode, connected to wifi
    WiFi.begin(ssids[0], passwords[0]);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.print("\nWiFi connected.\nIP address: ");
    Serial.println(WiFi.localIP());
  } else { // when no wifi available
    WiFi.softAP("5g-tower", "typpalingur");
    delay(10000);
    Serial.println(WiFi.localIP());
  }

  webServerSetup();
}

String webProcessor(const String& var) {
  float sys1temp = system1.sensor->getTemperature();
  float sys2temp = system2.sensor->getTemperature();


  char site[] = R"rawliteral(
<!DOCTYPE html>
<html>
    <head>
        <title>Temperature controller</title>
    </head>
    <body>
    {{content}}
    </body>
</html>
  )rawliteral";

  String ret(site);
  String content = "";

  if (var == "PAGEPLACEHOLDER") {
    char site[] = R"rawliteral(
<nav>
  <a href="/">Home</a>
  <a href="/settings">Settings</a>
  <a href="/extrasettings">Extra settings</a>
  <a href="/reboot">Reboot</a>
</nav>
<h2>System 1</h2>
<p>Target temperature is %f</p>
<p>Temperature: %f </p>
<p>Relay is currently %d</p>
<br>
<h2>System 2</h2>
<p>Target temperature is %f</p>
<p>Temperature: %f </p>
<p>Relay is currently %d</p>
    )rawliteral";

    char buf[2048]={};

    int n = sprintf(buf, site, system1.settings.target_temp, sys1temp, system1.relayState, system2.settings.target_temp, sys2temp, system2.relayState);

    content += buf;
  } else if (var == "METRICSPLACEHOLDER") {
    char text[] = R"rawliteral(
# HELP sys1_temperature_celcius Current measured temperature in system 1
# TYPE sys1_temperature_celcius gauge
sys1_temperature_celcius %.5f

# HELP sys1_relay_status Current status of relay in system 1
# TYPE sys1_relay_status gauge
sys1_relay_status %s

# HELP sys1_target_temperature Current target temperature in system 1
# TYPE sys1_target_temperature gauge
sys1_target_temperature %.2f

# HELP sys2_temperature_celcius Current measured temperature in system 2
# TYPE sys2_temperature_celcius gauge
sys2_temperature_celcius %.5f

# HELP sys2_relay_status Current status of relay in system 2
# TYPE sys2_relay_status gauge
sys2_relay_status %s

# HELP sys2_target_temperature Current target temperature in system 1
# TYPE sys2_target_temperature gauge
sys2_target_temperature %.2f
    )rawliteral";
    char buf[2048] = {};
    int n = sprintf(buf, text, sys1temp, system1.relayState, system1.settings.target_temp, sys2temp, system2.relayState, system2.settings.target_temp);

    ret += buf;
  } else if (var == "SETVARS") {
    char site[] = R"rawliteral(
       <form action="/set-variables">
        <input type="radio" id="sys1" name="system" value="sys1">
        <label for="sys1">System 1</label><br>
        <input type="radio" id="sys2" name="system" value="sys2">
        <label for="sys2">System 2</label><br>

        <label for="target">Target temperature:</label><br>
        <input type="text" id="target" name="target" value="67"><br>

        <input type="submit" value="Submit">
      </form> 
    )rawliteral";
    char buf[2048] = {};
    int n = sprintf(buf, site);
    ret += buf;
  } else if (var == "EXTRASETVARS") {

 char site[] = R"rawliteral(
       <form action="/set-variables">
        <input type="radio" id="sys1" name="system" value="sys1">
        <label for="sys1">System 1</label><br>
        <input type="radio" id="sys2" name="system" value="sys2">
        <label for="sys2">System 2</label><br>

        <label for="target">Target temperature:</label><br>
        <input type="text" id="target" name="target" value="%d"><br>

        <label for="heating">Heating (0/1):</label><br>
        <input type="text" id="heating" name="heating" value="%d"><br>

        <label for="Kp">Porportional modifier</label><br>
        <input type="text" id="Kp" name="Kp" value="%f"><br><br>

        <label for="Kdheating">Heating derivative modifier</label><br>
        <input type="text" id="Kdheating" name="Kdheating" value="%f"><br><br>

        <label for=Kdcooling">Cooling derivative modifier</label><br>
        <input type="text" id=Kdcooling" name=Kdcooling" value="%f"><br><br>

        <label for="Ki">Integral modifier</label><br>
        <input type="text" id="Ki" name="Ki" value="%f"><br><br>

        <input type="submit" value="Submit">
      </form> 
    )rawliteral";
    char buf[2048] = {};
    int n = sprintf(buf, site, system1.settings.target_temp, system1.settings.heating, system1.settings.Kp, system1.settings.Kdheating, system1.settings.Kdcooling, system1.settings.Ki);
    ret += buf;
  } else if (var == "CONFIRMREBOOT") {
 char site[] = R"rawliteral(
      <form action="/reboot" method="POST" onsubmit="return confirm('Are you sure you want to reboot?');">
      <input type="submit" value="Confirm reboot?">
      </form>
    )rawliteral";
    int n = sprintf(site, site);
    ret += site;
  }

  ret.replace("{{content}}", content);
  return ret;
}

void webServerSetup() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "%PAGEPLACEHOLDER%", webProcessor);
  });


  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/html", "%SETVARS%", webProcessor);
  });

  server.on("/extrasettings", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "%EXTRASETVARS%", webProcessor);
  });

  server.on("/set-variables", HTTP_GET, [](AsyncWebServerRequest *request){
    request->redirect("/");
  });

  server.on("/metrics", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "%METRICSPLACEHOLDER%", webProcessor);
  });

  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "%CONFIRMREBOOT%", webProcessor);
  });

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request){
    ESP.restart();
  });
  server.begin();
}

void temperatureLoop() {
  if(xSemaphoreTake(sensorLineMutex, portMAX_DELAY)) {
    sensorLine.requestTemperatures();

    xSemaphoreGive(sensorLineMutex);
  }

  unsigned long curtime = millis();
  
  system1.processSystem();
}

unsigned long earlierloop = 0;
void loop() {
  unsigned long curtime = millis();
  if (curtime - earlierloop > 2000) {
    temperatureLoop();
    earlierloop = curtime;
  }
}
