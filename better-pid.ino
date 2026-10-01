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
#define RELAY2 27
#define TEMP1 18
#define TEMP2 19
#define INTERNET_CONNECT_LED 2 // led to show internet connectivity, currently on board led

#define WIFI_TIMEOUT_MS 20000 // 20 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 30000 // Wait 30 seconds after a failed connection attempt

bool webServerStarted = false;

uint8_t sensorID[8] = {0};

OneWire sensor1wire(TEMP1);
OneWire sensor2wire(TEMP2);
DallasTemperature sensor1line(&sensor1wire);
DallasTemperature sensor2line(&sensor2wire);
Preferences savedSettings;

WiFiMulti wifiMulti;
const int networks = 1;
const char* ssids[] = {"Vikurbakki18"};
const char* passwords[] = {"Fletturimi35"};

    /*WiFi.begin(ssids[0], passwords[0]);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.print("\nWiFi connected.\nIP address: ");
    Serial.println(WiFi.localIP());*/

// function from https://simplyexplained.com/blog/esp32-keep-wifi-alive-with-freertos-task/
void keepWifiAlive(void *parameter) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(INTERNET_CONNECT_LED, HIGH);
      
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      continue;
    }
    digitalWrite(INTERNET_CONNECT_LED, LOW);
    Serial.println("[WIFI] Connecting");
    WiFi.disconnect();
    WiFi.begin(ssids[0], passwords[0]);

    unsigned long startAttemptTime = millis();

    // Keep looping while we're not connected and haven't reached the timeout
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
      // tiny blinking while connecting
      digitalWrite(INTERNET_CONNECT_LED, HIGH);
      vTaskDelay(10/portTICK_PERIOD_MS);
      digitalWrite(INTERNET_CONNECT_LED, LOW);
      vTaskDelay(500/portTICK_PERIOD_MS);
    }

    // When we couldn't make a WiFi connection (or the timeout expired)
		// sleep for a while and then retry.
    if(WiFi.status() != WL_CONNECTED){
      Serial.println("[WIFI] FAILED");
      vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
			continue;
    }
    digitalWrite(INTERNET_CONNECT_LED, HIGH);
    Serial.print("[WIFI] Connected: ");
    Serial.println(WiFi.localIP());
  }
}

bool debug = false;

typedef struct {
  float Kp;
  float Kdheating;
  float Kdcooling;
  float Ki;
  float target_temp;
  float integral;
  uint64_t minoff;
  uint64_t minon;
  bool heating;
  bool relayState;
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
  float fetchTemperature() {
    float recv = -121;
    if(xSemaphoreTake(*lineMutex, portMAX_DELAY)){
      recv = this->line->getTempCByIndex(0);
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
    out.reserve(1000);
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
  float filteredDerivative = 0;
  float previousError;
  unsigned long previousTime = 0;
  unsigned long lastChangeTime = 0;
  TemperatureSensor *sensor;
  bool relayState;
  int relayPin;
  ArduinoQueue<float> tempQueue;
  ArduinoQueue<unsigned long> timeQueue;
  SemaphoreHandle_t systemMutex;

  System(Settings *startSettings, TemperatureSensor *sensor, String name, int pin) : tempQueue(10), timeQueue(10){
    this->sensor = sensor;
    memcpy(&settings, startSettings, sizeof(Settings));
    //ArduinoQueue<float> tempQueue(10);
    //ArduinoQueue<unsigned long> timeQueue(10);
    this->name = name;
    previousTime = 0;
    this->relayPin = pin;
    systemMutex = xSemaphoreCreateMutex();
  } 

  void writeFlash() {
    // write settings and such to flash memory
    storeddata_t latestSettings = {
      settings.Kp,
      settings.Kdheating,
      settings.Kdcooling,
      settings.Ki,
      settings.target_temp,
      integral,
      settings.min_off_time,
      settings.min_on_time,
      settings.heating,
      relayState,
      (uint8_t)relayPin,
      systemID
    };

    savedSettings.begin(name.c_str());
    //savedSettings.putBytes(name.c_str(), &latestSettings, sizeof(latestSettings));
    savedSettings.putFloat("Kp", settings.Kp);
    savedSettings.putFloat("Kdheating", settings.Kdheating);
    savedSettings.putFloat("Kdcooling", settings.Kdcooling);
    savedSettings.putFloat("Ki", settings.Ki);
    savedSettings.putFloat("target_temp", settings.target_temp);
    savedSettings.putUInt("min_off_time", settings.min_off_time);
    savedSettings.putUInt("min_on_time", settings.min_on_time);
    savedSettings.putBool("heating", settings.heating);
    savedSettings.putUChar("systemID", systemID);
    savedSettings.end();
    
  }

  void getFlash() {
    // initialize concent from flash memory
    savedSettings.begin(name.c_str(), true);
    /*size_t memlen = savedSettings.getBytesLength(name.c_str());
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
    //relayPin = (int)newSettings->relayPin;*/
    
    /*if(savedSettings.isKey("Kp")) {
      settings.Kp = savedSettings.getFloat("Kp");
    }
    if(savedSettings.isKey("Kdheating")) {
      settings.Kdheating = savedSettings.getFloat("Kdheating");
    }
    if(savedSettings.isKey("Kdcooling")) {
      settings.Kdcooling = savedSettings.getFloat("Kdcooling");
    }
    if(savedSettings.isKey("Ki")) {
      settings.Ki = savedSettings.getFloat("Ki");
    }
    if(savedSettings.isKey("target_temp")) {
      settings.target_temp = savedSettings.getFloat("target_temp");
    }
    if(savedSettings.isKey("min_off_time")) {
      settings.min_off_time = savedSettings.getUInt("min_off_time");
    }
    if(savedSettings.isKey("min_on_time")) {
      settings.min_on_time = savedSettings.getUInt("min_on_time");
    }
    if(savedSettings.isKey("heating")) {
      settings.heating = savedSettings.getBool("heating");
    }
    if(savedSettings.isKey("systemID")) {
      systemID = savedSettings.getUChar("systemID");
    }*/
    // i used mr ai to add the else statements for debugging
    if(savedSettings.isKey("Kp")) {
      settings.Kp = savedSettings.getFloat("Kp");
    } else {
      Serial.println("Kp was not stored");
    }

    if(savedSettings.isKey("Kdheating")) {
      settings.Kdheating = savedSettings.getFloat("Kdheating");
    } else {
      Serial.println("Kdheating was not stored");
    }

    if(savedSettings.isKey("Kdcooling")) {
      settings.Kdcooling = savedSettings.getFloat("Kdcooling");
    } else {
      Serial.println("Kdcooling was not stored");
    }

    if(savedSettings.isKey("Ki")) {
      settings.Ki = savedSettings.getFloat("Ki");
    } else {
      Serial.println("Ki was not stored");
    }

    if(savedSettings.isKey("target_temp")) {
      settings.target_temp = savedSettings.getFloat("target_temp");
    } else {
      Serial.println("target_temp was not stored");
    }

    if(savedSettings.isKey("min_off_time")) {
      settings.min_off_time = savedSettings.getUInt("min_off_time");
    } else {
      Serial.println("min_off_time was not stored");
    }

    if(savedSettings.isKey("min_on_time")) {
      settings.min_on_time = savedSettings.getUInt("min_on_time");
    } else {
      Serial.println("min_on_time was not stored");
    }

    if(savedSettings.isKey("heating")) {
      settings.heating = savedSettings.getBool("heating");
    } else {
      Serial.println("heating was not stored");
    }

    if(savedSettings.isKey("systemID")) {
      systemID = savedSettings.getUChar("systemID");
    } else {
      Serial.println("systemID was not stored");
    }


    savedSettings.end();
  }

  void clearFlash() {
    savedSettings.begin(name.c_str());
    savedSettings.remove(name.c_str());
    savedSettings.clear();
    savedSettings.end();
  }

  String toString() { // this was ai generated for time sake, sorry facts and logic
    String out = "========================\n";
    out.reserve(1000);
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

  float getTemperature() {
    return sensor->fetchTemperature();
  }

  float pid(float error, float old_error, unsigned long curtime, unsigned long old_time) {
    float porportional = error;

    float rawDerivative = 0;
    unsigned long dt = curtime - old_time;
    if (previousTime == 0) {
      dt = 0;
    }
    rawDerivative = (error - old_error)/(dt/1000.0);

    float Kd = 0;
    if (rawDerivative > 0) { // error rising, meaning temp is cooling
      Kd = this->settings.Kdcooling;
    } else {
      Kd = this->settings.Kdheating;
    }
    float alpha = 0.1;
    filteredDerivative = alpha * rawDerivative + (1.0 - alpha) * this->filteredDerivative;
    
    
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
    
    float output = this->settings.Kp * porportional + Kd * this->filteredDerivative + this->settings.Ki * this->integral;
    if(debug) {
      Serial.printf("porportional: %.4f, filteredDerivative: %.4f, integral: %.4f\n", porportional, filteredDerivative, this->integral);
      Serial.printf("Kp*porportional: %.4f + Kd*filteredDerivative: %.4f + Ki*integral: %.4f\noutput = %.4f\n", this->settings.Kp * porportional, Kd * filteredDerivative, this->settings.Ki * this->integral, output);
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
    if (xSemaphoreTake(this->systemMutex, pdMS_TO_TICKS(100))){
      unsigned long curtime = millis();
      float temp = this->getTemperature();
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

      xSemaphoreGive(this->systemMutex);
    }
  }

  bool outputToBin(float output) {
    if (abs(output) < 3) { // allow for a small error window
      return this->relayState;
    }
    if (output > 0) { // read colder than target temperature
      return settings.heating;
    } else {
      return !settings.heating;
    }
  }
};






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




SemaphoreHandle_t sensor1lineMutex;
SemaphoreHandle_t sensor2lineMutex;
TemperatureSensor sensor1(sensorID, &sensor1line, &sensor1lineMutex);
System system1(&heatingsettings, &sensor1, "System1", RELAY);

TemperatureSensor sensor2(sensorID, &sensor2line, &sensor2lineMutex);
System system2(&heatingsettings, &sensor2, "System2", RELAY2);

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


  
}

String webProcessor(const String& var) {
  float sys1temp;
  float sys1target;    
  bool sys1relaystate;     
  int sys1heating;
  float sys1porp;
  float sys1kdheat;
  float sys1kdcool;
  float sys1integ;
  unsigned long sys1minoff;
  unsigned long sys1minon;    
  float sys2temp;
  float sys2target;
  bool sys2relaystate;
  int sys2heating;
  float sys2porp;
  float sys2kdheat;
  float sys2kdcool;
  float sys2integ;
  unsigned long sys2minoff;
  unsigned long sys2minon;

  if (xSemaphoreTake(system1.systemMutex, portMAX_DELAY)) {
    sys1temp = system1.sensor->getTemperature();
    sys1target = system1.settings.target_temp;
    sys1relaystate = system1.relayState;
    sys1heating = system1.settings.heating;
    sys1porp = system1.settings.Kp;
    sys1kdheat = system1.settings.Kdheating;
    sys1kdcool = system1.settings.Kdcooling;
    sys1integ = system1.settings.Ki;
    sys1minoff = system1.settings.min_off_time;
    sys1minon = system1.settings.min_on_time;
    
    xSemaphoreGive(system1.systemMutex);
  }
  
  if (xSemaphoreTake(system2.systemMutex, portMAX_DELAY)) {
    sys2temp = system2.sensor->getTemperature();
    sys2target = system2.settings.target_temp;
    sys2relaystate = system2.relayState;
    sys2heating = system2.settings.heating;
    sys2porp = system2.settings.Kp;
    sys2kdheat = system2.settings.Kdheating;
    sys2kdcool = system2.settings.Kdcooling;
    sys2integ = system2.settings.Ki;
    sys2minoff = system2.settings.min_off_time;
    sys2minon = system2.settings.min_on_time;

    xSemaphoreGive(system2.systemMutex);
  }

  char site[] = R"rawliteral(
<!DOCTYPE html>
<html>
    <head>
        <title>Temperature controller</title>
        <meta charset="UTF-8">
    </head>
    <body>
    {{content}}
    </body>
</html>
  )rawliteral";

  String ret(site);
  String content = "";
  content.reserve(5000);
  char buf[2048*4] = {};
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

    buf[0] = '\0';

    int n = snprintf(buf, sizeof(buf), site, sys1target, sys1temp, sys1relaystate, sys2target, sys2temp, sys2relaystate);

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
    buf[0] = '\0';
    int n = snprintf(buf, sizeof(buf), text, sys1temp, sys1relaystate, sys1target, sys2temp, sys2relaystate, sys2target);
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
    ret += site;
  } else if (var == "EXTRASETVARS") {/*

 char site[] = R"rawliteral(
       <form action="/set-variables">
        <input type="radio" id="sys1" name="system" value="System1">
        <label for="sys1">System 1</label><br>
        <input type="radio" id="sys2" name="system" value="System2">
        <label for="sys2">System 2</label><br>

        <label for="target">Target temperature:</label><br>
        <input type="text" id="target" name="target" value="%.2f"><br>

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
    
    char site2[] = R"rawliteral(
    
      <h1>Current values</h1>
      <h2>System 1</h2>


      <h2>System 2</h2>
    )rawliteral";
    buf[0] = '\0';
    int n = snprintf(buf, sizeof(buf), site, sys1target, sys1heating, sys1porp, sys1kdheat, sys1kdcool, sys1integ, sys1minoff, sys1minon);*/
    // 1. The HTML Template
    // Note: %f for floats, %d for booleans/ints, %lu for unsigned long
    char site[] = R"rawliteral(
      <form action="/set-variables">
        <h3>Edit Settings</h3>
        <input type="radio" id="sys1" name="system" value="System1" checked>
        <label for="sys1">System 1</label>
        <input type="radio" id="sys2" name="system" value="System2">
        <label for="sys2">System 2</label><br><br>

        <label for="target">Target temperature:</label><br>
        <input type="text" id="target" name="target" value="%.2f"><br>

        <label for="heating">Heating (1) or Cooling (0):</label><br>
        <input type="text" id="heating" name="heating" value="%d"><br>

        <label for="Kp">Proportional (Kp):</label><br>
        <input type="text" id="Kp" name="Kp" value="%.4f"><br>

        <label for="Kdheating">Heating Derivative (Kd):</label><br>
        <input type="text" id="Kdheating" name="Kdheating" value="%.4f"><br>

        <label for="Kdcooling">Cooling Derivative (Kd):</label><br>
        <input type="text" id="Kdcooling" name="Kdcooling" value="%.4f"><br>

        <label for="Ki">Integral (Ki):</label><br>
        <input type="text" id="Ki" name="Ki" value="%.4f"><br>

        <label for="minoff">Min Off Time (ms):</label><br>
        <input type="text" id="minoff" name="minoff" value="%lu"><br>
        
        <label for="minon">Min On Time (ms):</label><br>
        <input type="text" id="minon" name="minon" value="%lu"><br><br>

        <input type="submit" value="Apply Settings">
      </form> 

      <hr>
      <h2>Current Configuration Summary</h2>
      <table border="1" style="width:100%; text-align:left; border-collapse: collapse;">
        <tr>
          <th>Setting</th>
          <th>System 1</th>
          <th>System 2</th>
        </tr>
        <tr>
          <td>Target Temp</td>
          <td>%.2f°C</td>
          <td>%.2f°C</td>
        </tr>
        <tr>
          <td>Mode</td>
          <td>%s</td>
          <td>%s</td>
        </tr>
        <tr>
          <td>Kp</td>
          <td>%.4f</td>
          <td>%.4f</td>
        </tr>
        <tr>
          <td>Kd (Heat/Cool)</td>
          <td>%.2f / %.2f</td>
          <td>%.2f / %.2f</td>
        </tr>
        <tr>
          <td>Ki</td>
          <td>%.4f</td>
          <td>%.4f</td>
        </tr>
        <tr>
          <td>Min On/Off (s)</td>
          <td>%lu / %lu</td>
          <td>%lu / %lu</td>
        </tr>
      </table>
    )rawliteral";

    buf[0] = '\0';

    // 2. Map variables to the specifiers
    // We pass sys1 values to the form, then both sys1 and sys2 to the table
    snprintf(buf, sizeof(buf), site, 
      // Form values (defaulting to System 1)
      sys1target, sys1heating, sys1porp, sys1kdheat, sys1kdcool, sys1integ, sys1minoff, sys1minon,
      
      // Table values - System 1 vs System 2
      sys1target, sys2target,
      sys1heating ? "HEATING" : "COOLING", sys2heating ? "HEATING" : "COOLING",
      sys1porp, sys2porp,
      sys1kdheat, sys1kdcool, sys2kdheat, sys2kdcool,
      sys1integ, sys2integ,
      sys1minon/1000, sys1minoff/1000, sys2minon/1000, sys2minoff/1000
    );



    ret += buf;
  } else if (var == "CONFIRMREBOOT") {
 char site[] = R"rawliteral(
      <form action="/reboot" method="POST" onsubmit="return confirm('Are you sure you want to reboot?');">
      <input type="submit" value="Confirm reboot?">
      </form>
    )rawliteral";
    buf[0] = '\0';
    int n = snprintf(buf, sizeof(buf), site);
    ret += buf;
  }

  ret.replace("{{content}}", content);
  return ret;
}



void temperatureLoop() {
  if(xSemaphoreTake(sensor1lineMutex, portMAX_DELAY)) {
    sensor1line.requestTemperatures();

    xSemaphoreGive(sensor1lineMutex);
  }

  if(xSemaphoreTake(sensor2lineMutex, portMAX_DELAY)) {
    sensor2line.requestTemperatures();

    xSemaphoreGive(sensor2lineMutex);
  }


  unsigned long curtime = millis();
  
  system1.processSystem();
  system2.processSystem();
}
  
void setup() {
  Serial.begin(9600);
  Serial.println("started");
  sensor1line.begin();
  sensor1line.setResolution(12);
  sensor2line.begin();
  sensor2line.setResolution(12);

  sensor1lineMutex = xSemaphoreCreateMutex();
  sensor2lineMutex = xSemaphoreCreateMutex();


  pinMode(RELAY, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(2, OUTPUT);

  

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
    /*WiFi.begin(ssids[0], passwords[0]);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.print("\nWiFi connected.\nIP address: ");
    Serial.println(WiFi.localIP());*/

    // from https://simplyexplained.com/blog/esp32-keep-wifi-alive-with-freertos-task/
    xTaskCreatePinnedToCore(
      keepWifiAlive,
      "keepWifiAlive",  // Task name
      8192,             // Stack size (bytes)
      NULL,             // Parameter
      1,                // Task priority
      NULL,             // Task handle
      ARDUINO_RUNNING_CORE
    );
  } else { // when no wifi available
    WiFi.softAP("5g-tower", "typpalingur");
  }
  webServerSetup();
}


unsigned long earlierloop = 0;
unsigned long savedDataearlier = 0;
unsigned long savedatainterval = (60*60*1000);

void loop() {
  if(WiFi.status() == WL_CONNECTED && !webServerStarted) {
    server.begin();
    webServerStarted = true;
  }
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
