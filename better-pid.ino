/**global todo list
//TODO: save values to esp persistent memory (scratchpad or something)
*/


#include <ArduinoQueue.h>

#include <ESPAsyncWebServer.h>

#include <WiFi.h>
#include <WiFiMulti.h>

#include <DallasTemperature.h>

#include <OneWire.h>

#include <Preferences.h>

#define APMODE false

#define RELAY 17
#define RELAY2 18
#define TEMP 16

uint8_t sensor1ID[8] = {0x28, 0xFF, 0x64, 0x1F, 0x78, 0x74, 0x68, 0x04};
uint8_t sensor2ID[8] = {0x28, 0xFF, 0x64, 0x1F, 0x78, 0x68, 0xB1, 0x6E};

OneWire oneWire(TEMP);
DallasTemperature sensorLine(&oneWire);
Preferences savedSettings;

bool debug = true;

typedef struct {
  uint32_t Kp;
  uint32_t Kdheating;
  uint32_t Kdcooling;
  uint32_t Ki;
  uint32_t target_temp;
  uint32_t integral;
  uint64_t minoff;
  uint64_t minon;
  uint8_t heating;
  uint8_t relayState;
  uint8_t relayPin;
  uint8_t systemID;

} storeddata_t;

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
  float Ki;
  float Kdheating;
  float Kdcooling;

  String toString() { // this thing was ai generated, i am ashamed
    String out = "--- Settings ---\n";
    out += "Mode: " + String(heating ? "HEATING" : "COOLING") + "\n";
    out += "Target: " + String(target_temp, 2) + "°C\n";
    out += "Min On/Off: " + String(min_on_time) + "s / " + String(min_off_time) + "s\n";
    out += "PID: P:" + String(Kp, 2) + " I:" + String(Ki, 4) + "\n";
    out += "Special D: H:" + String(Kdheating, 2) + " C:" + String(Kdcooling, 2) + "\n";
    return out;
  }
};

class System {
  public:
  String name;
  uint8_t systemID;
  Settings settings;
  float integral = 0;
  float previousError;
  unsigned long previousTime = 0;
  unsigned long lastChangeTime = 0;
  TemperatureSensor *sensor;
  bool relayState;
  int relayPin;
  ArduinoQueue<float> tempQueue;
  ArduinoQueue<unsigned long> timeQueue;

  System(Settings *startSettings, TemperatureSensor *sensor, String name, int pin) {
    this->sensor = sensor;
    memcpy(&settings, startSettings, sizeof(Settings));
    ArduinoQueue<float> tempQueue(10);
    ArduinoQueue<unsigned long> timeQueue(10);
    this->name = name;
    previousTime = 0;
    this->relayPin = pin;
  } 

  void writeFlash() {
    // write settings and such to flash memory
    storeddata_t latestSettings = {
      (uint32_t)settings.Kp,
      (uint32_t)settings.Kdheating,
      (uint32_t)settings.Kdcooling,
      (uint32_t)settings.Ki,
      (uint32_t)settings.target_temp,
      (uint32_t)integral,
      (uint64_t)settings.min_off_time,
      (uint64_t)settings.min_on_time,
      (uint8_t)settings.heating,
      (uint8_t)relayState,
      (uint8_t)relayPin,
      systemID
    };

    savedSettings.begin(name.c_str());
    savedSettings.putBytes(name.c_str(), &latestSettings, sizeof(latestSettings));
  }

  void getFlash() {
    // initialize concent from flash memory
    savedSettings.begin(name.c_str());
    size_t memlen = savedSettings.getBytesLength(name.c_str());
    if (memlen != sizeof(storeddata_t)) { // check if its the right size
      Serial.println("flash did not have correct size");
      return;
    }

    char buffer[memlen];
    savedSettings.getBytes(name.c_str(), buffer, memlen);

    storeddata_t *newSettings = (storeddata_t *)buffer;

    settings.min_off_time = newSettings->minoff;
    settings.min_on_time = newSettings->minon;

    settings.target_temp = newSettings->target_temp;

    settings.heating = (bool)newSettings->heating;

    settings.Kp = (float)newSettings->Kp;
    settings.Kdheating = (float)newSettings->Kdheating;
    settings.Kdcooling = (float)newSettings->Kdcooling;
    settings.Ki = (float)newSettings->Ki;

    //integral = (float)newSettings->integral;
    relayState = (bool)newSettings->relayState;
    //relayPin = (int)newSettings->relayPin;
  }

  void clearFlash() {
    savedSettings.begin(name.c_str());
    savedSettings.remove(name.c_str());
  }

  String toString() { // this was ai generated for time sake, sorry facts and logic
    String out = "========================\n";
    out += "SYSTEM: " + name + "\n";
    out += "tatus: " + String(relayState ? "RUNNING (ON)" : "IDLE (OFF)") + "\n";
    out += "Relay Pin: " + String(relayPin) + "\n";
    
    // Current state variables
    out += "Integral: " + String(integral, 4) + "\n";
    out += "Prev Error: " + String(previousError, 2) + "\n";
    
    // Calculate time since last change
    unsigned long timeInState = (millis() - lastChangeTime) / 1000;
    out += "Time in current state: " + String(timeInState) + "s\n";

    // Nest the settings string
    out += settings.toString();
    out += "========================\n";
    return out;
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
      Serial.println(relayState);
    }
  }

  float getTemperature(DallasTemperature *sensorLine, SemaphoreHandle_t *sensorMutex) {
    return sensor->fetchTemperature(sensorLine, sensorMutex);
  }

  float pid(float error, float old_error, unsigned long curtime, unsigned long old_time) {
    float porportional = error;

    float derivative = 0;
    unsigned long dt = curtime - old_time;
    if (previousTime == 0) {
      dt = 0;
    }
    derivative = (error - old_error)/(dt/1000.0);

    float Kd = 0;
    if (derivative > 0) { // error rising, meaning temp is cooling
      Kd = this->settings.Kdcooling;
    } else {
      Kd = this->settings.Kdheating;
    }
    
    
    // use last dt and not the same dt as derivative
    if (previousTime != 0) {
      this->integral += error * ((curtime - previousTime)/1000.0);
    }

    // need to prevent integral from going crazy
    //
    //this is to just put a cap on it
    float ilimit = (this->settings.Kp * 10)/(this->settings.Ki==0 ? 1 : this->settings.Ki);
    if (this->integral > ilimit) this->integral = ilimit;
    if (this->integral < -ilimit) this->integral = -ilimit;

    // this is to only use it within a certain range of the target
    // clear it if we are far away
    if (abs(error) > 15.0) {
      this->integral = 0;
    }

    //TODO: maybe throw out integral on large errors?
    
    float output = this->settings.Kp * porportional + Kd * derivative + this->settings.Ki * this->integral;
    if(debug) {
      Serial.printf("porportional: %.4f, derivative: %.4f, integral: %.4f\n", porportional, derivative, this->integral);
      Serial.printf("Kp*porportional: %.4f + Kd*derivative: %.4f + Ki*integral: %.4f\noutput = %.4f\n", this->settings.Kp * porportional, Kd * derivative, this->settings.Ki * this->integral, output);
    }
    return output;
  }

  void changeRelays() {
    // the states on my relay are reversed for some reason
    if (this->relayState == LOW) {
      digitalWrite(this->relayPin, HIGH);
    } else {
      digitalWrite(this->relayPin, LOW);
    }
  }

  void processSystem() {
    unsigned long curtime = millis();
    float temp = this->getTemperature(this->sensor->line, this->sensor->lineMutex);
    Serial.println(name + " temperature: " + String(temp));
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
    this->changeRelays();

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
};




SemaphoreHandle_t sensorLineMutex;
TemperatureSensor sensor1(sensor1ID, &sensorLine, &sensorLineMutex);
System system1(&heatingsettings, &sensor1, "System1", RELAY);

TemperatureSensor sensor2(sensor2ID, &sensorLine, &sensorLineMutex);
System system2(&heatingsettings, &sensor2, "System2", RELAY);

System *systems[] = {&system1, &system2};



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
    String system = "System1";
    String target_temp = "";

    if (request->hasParam("system")) {
      system = request->getParam("system")->value();
    }

    if (request->hasParam("target")) {
      String ans = request->getParam("target")->value();
      float target = ans.toFloat();

      for (int i = 0; i < sizeof(systems)/sizeof(System*); i++) {
        if (system.equals(systems[i]->name)) {
          systems[i]->settings.target_temp = target;
          Serial.println(systems[i]->name);
          Serial.println("setting this temp");
        }
      }
    }

    if (request->hasParam("heating")) {
      String ans = request->getParam("heating")->value();
      bool heat = (ans.toInt());
      for (int i = 0; i < sizeof(systems)/sizeof(System*); i++) {
        if (system.equals(systems[i]->name)) {
          systems[i]->settings.heating = heat;
        }
      }
    }


    if (request->hasParam("Kp")) {
      String ans = request->getParam("Kp")->value();
      float kp = ans.toFloat();
      for (int i = 0; i < sizeof(systems)/sizeof(System*); i++) {
        if (system.equals(systems[i]->name)) {
          systems[i]->settings.Kp = kp;
        }
      }
    }


    if (request->hasParam("Kdheating")) {
      String ans = request->getParam("Kdheating")->value();
      float kdheat = ans.toFloat();
      for (int i = 0; i < sizeof(systems)/sizeof(System*); i++) {
        if (system.equals(systems[i]->name)) {
          systems[i]->settings.Kdheating = kdheat;
        }
      }
    }


    if (request->hasParam("Kdcooling")) {
      String ans = request->getParam("Kdcooling")->value();
      float kdcool = ans.toFloat();
      for (int i = 0; i < sizeof(systems)/sizeof(System*); i++) {
        if (system.equals(systems[i]->name)) {
          systems[i]->settings.Kdcooling = kdcool;
        }
      }
    }


    if (request->hasParam("Ki")) {
      String ans = request->getParam("Ki")->value();
      float ki = ans.toFloat();
      for (int i = 0; i < sizeof(systems)/sizeof(System*); i++) {
        if (system.equals(systems[i]->name)) {
          systems[i]->settings.Ki = ki;
        }
      }
    }

    if (request->hasParam("minon")) {
      String ans = request->getParam("minon")->value();
      float minon = ans.toFloat();
      for (int i = 0; i < sizeof(systems)/sizeof(System*); i++) {
        if (system.equals(systems[i]->name)) {
          systems[i]->settings.min_on_time = minon;
        }
      }
    }


    if (request->hasParam("minoff")) {
      String ans = request->getParam("minoff")->value();
      float minoff = ans.toFloat();
      for (int i = 0; i < sizeof(systems)/sizeof(System*); i++) {
        if (system.equals(systems[i]->name)) {
          systems[i]->settings.min_off_time = minoff;
        }
      }
    }
    for (int i = 0; i < sizeof(systems)/sizeof(System*); i++) {
      if (system.equals(systems[i]->name)) {
        systems[i]->writeFlash();
      }
    }


    request->redirect("/");
  });

  server.on("/metrics", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "%METRICSPLACEHOLDER%", webProcessor);
  });

  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "%CONFIRMREBOOT%", webProcessor);
  });

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request){
    request->redirect("/");
    ESP.restart();
  });

  server.on("/debugon", HTTP_GET, [](AsyncWebServerRequest *request){
    debug = true;
    request->redirect("/");
  });

  server.on("/save", HTTP_GET, [](AsyncWebServerRequest *request){
    system1.writeFlash();
    system2.writeFlash();
    request->redirect("/");
  });

  server.on("/debugoff", HTTP_GET, [](AsyncWebServerRequest *request){
    debug = false;
    request->redirect("/");
  });

  server.on("/clear-rom", HTTP_GET, [](AsyncWebServerRequest *request){
    system1.clearFlash();
    system2.clearFlash();
    request->redirect("/");
  });


  server.begin();
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
  <a href="/clear-rom">Clear eeprom</a>
  {{debugbutton}}
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

    String buffer(buf);
    
    String text = "";
    if(debug) {
      text = "<a href='/debugoff'>Debug mode OFF</a>";
    } else {
      text = "<a href='/debugon'>Debugmode ON</a>";
    }
    buffer.replace("{{debugbutton}}", text);

    content += buffer;
  } else if (var == "METRICSPLACEHOLDER") {
    char text[] = R"rawliteral(
# HELP sys1_temperature_celcius Current measured temperature in system 1
# TYPE sys1_temperature_celcius gauge
sys1_temperature_celcius %.5f

# HELP sys1_relay_status Current status of relay in system 1
# TYPE sys1_relay_status gauge
sys1_relay_status %d

# HELP sys1_target_temperature Current target temperature in system 1
# TYPE sys1_target_temperature gauge
sys1_target_temperature %.2f

# HELP sys2_temperature_celcius Current measured temperature in system 2
# TYPE sys2_temperature_celcius gauge
sys2_temperature_celcius %.5f

# HELP sys2_relay_status Current status of relay in system 2
# TYPE sys2_relay_status gauge
sys2_relay_status %d

# HELP sys2_target_temperature Current target temperature in system 1
# TYPE sys2_target_temperature gauge
sys2_target_temperature %.2f
    )rawliteral";
    char buf[2048] = {};
    int n = sprintf(buf, text, sys1temp, system1.relayState, system1.settings.target_temp, sys2temp, system2.relayState, system2.settings.target_temp);
//return cause we dont want html here
    return String(buf);
  } else if (var == "SETVARS") {
    char site[] = R"rawliteral(
       <form action="/set-variables">
        <input type="radio" id="sys1" name="system" value="System1">
        <label for="sys1">System 1</label><br>
        <input type="radio" id="sys2" name="system" value="System2">
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
        <input type="radio" id="sys1" name="system" value="System1">
        <label for="sys1">System 1</label><br>
        <input type="radio" id="sys2" name="system" value="System2">
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
        <input type="text" id="Kdcooling" name="Kdcooling" value="%f"><br><br>

        <label for="Ki">Integral modifier</label><br>
        <input type="text" id="Ki" name="Ki" value="%f"><br><br>

        <label for="minoff">Min off time</label><br>
        <input type="text" id="minoff" name="minoff" value="%lu"><br><br>
        
        <label for="minon">Min on time</label><br>
        <input type="text" id="minon" name="minon" value="%lu"><br><br>

        <input type="submit" value="Submit">
      </form> 
    )rawliteral";
    
    char buf[2048] = {};
    int target = system1.settings.target_temp;
    int heating = system1.settings.heating;
    float porp = system1.settings.Kp;
    float kdheat = system1.settings.Kdheating;
    float kdcool = system1.settings.Kdcooling;
    float integ = system1.settings.Ki;
    int n = sprintf(buf, site, target, heating, porp, kdheat, kdcool, integ, system1.settings.min_off_time, system1.settings.min_on_time);
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



void temperatureLoop() {
  if(xSemaphoreTake(sensorLineMutex, portMAX_DELAY)) {
    sensorLine.requestTemperatures();

    xSemaphoreGive(sensorLineMutex);
  }

  unsigned long curtime = millis();
  
  system1.processSystem();
  system2.processSystem();
}
  
void setup() {
  Serial.begin(9600);
  Serial.println("started");
  sensorLine.begin();
  sensorLine.setResolution(sensor1ID, 9);

  sensorLineMutex = xSemaphoreCreateMutex();


  pinMode(RELAY, OUTPUT);

  

 /* storeddata_t *storedsettings = (storeddata_t *)buffer;
  if (debug) {
    Serial.println("printing stored settings");
    Serial.println(storedsettings->Kp);
    Serial.println(storedsettings->Kdheating);
    Serial.println(storedsettings->Kdcooling);
    Serial.println(storedsettings->Ki);
    Serial.println(storedsettings->target_temp);
    Serial.println(storedsettings->integral);
    Serial.println(storedsettings->minoff);
    Serial.println(storedsettings->minon);
    Serial.println(storedsettings->heating);
    Serial.println(storedsettings->relayState);
    Serial.println(storedsettings->relayPin);
    Serial.println(storedsettings->systemID);
  }*/
  //TODO: verify this data?

  system1.getFlash();
  system2.getFlash();


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
  }
  webServerSetup();
}


unsigned long earlierloop = 0;
unsigned long savedDataearlier = 0;
unsigned long savedatainterval = (60*60*1000);

void loop() {
  unsigned long curtime = millis();
  if (curtime - earlierloop > 2000) {
    temperatureLoop();
    if (debug) Serial.println(system1.toString());
    earlierloop = curtime;

    if (curtime - savedDataearlier > savedatainterval) {
      system1.writeFlash();
      system2.writeFlash();
      savedDataearlier = curtime;
    }
  }
  delay(100);
}
