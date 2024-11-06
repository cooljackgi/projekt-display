/*
 * Hauptskript für das Anzeigen von Daten und QR-Codes auf einem TFT-Display
 * unter Verwendung verschiedener Sensoren und Widgets.
 */

#include <EEPROM.h>
#include "FS.h"
#include "LittleFS.h"
#include <SPI.h>
#include "lvgl.h"
#include "ui.h" // Ersetzen Sie dies durch den Namen Ihrer generierten Datei
#include <stdint.h>
#include <Adafruit_GFX.h>
#include <qrcode.h>
#include <TFT_eSPI.h> // TFT_eSPI-Bibliothek einbinden
#include <TFT_eWidget.h>
#include "C:\Users\becke\Documents\PlatformIO\Projects\projekt display\lib\logo.c"
#include "HX711.h"
#include "C:\Users\becke\Documents\PlatformIO\Projects\projekt display\lib\CST816S\CST816S.h"
#include <XPT2046_Touchscreen.h>
#include <EEPROM.h>
#include <random>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <WebSerial.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

AsyncWebServer server5(80);

bool isSimButtonActive = true; // Flag für den Zustand des Simulations-Buttons
bool isWebServerActive = true; // Flag für den Zustand des Web-Servers

int status = WL_IDLE_STATUS;

uint16_t calData[5];
uint8_t calDataOK = 0;
uint8_t flag = 0;
// Touchscreen-Konfiguration
bool isMenuDisplayed = false;
const int adcPin = -1;              // ADC-Pin
const float referenceVoltage = 3.3; // Referenzspannung des ADC
const int adcResolution = 1023;     // ADC-Auflösung (10 Bit)
// int xMin, xMax, yMin, yMax;
#define BUTTON_PIN 32                // Definieren Sie den Pin, an dem der Knopf angeschlossen ist
unsigned long displayDuration = 100; // Anzeigedauer in Millisekunden (z.B. 5000ms = 5 Sekunden)

String serialData = "";

HX711 scale;
long manualTareOffset = 0; // Globale Variable für manuelle Tara
uint8_t dataPin = 22;
uint8_t clockPin = 21;
// Globale Variablen

TFT_eSPI tft = TFT_eSPI();
#define TCS_PIN 33  // Chip-Select-Pin für den Touchscreen
#define TIRQ_PIN 36 // Touch IRQ Pin (optional)
#define TOUCH_MISO 39
#define TOUCH_MOSI 32

SPIClass spiTouch(HSPI); // Erstellen eines SPI-Objekts für den Touchscreen

XPT2046_Touchscreen ts(33); // CS_PIN mit dem korrekten Pin ersetzen
TS_Point startPoint;
bool isTouching = false;
const int swipeThreshold = 30; // Schwellenwert für Swipe-Erkennung
GraphWidget gr = GraphWidget(&tft);
TraceWidget tr = TraceWidget(&gr);
ButtonWidget button1 = ButtonWidget(&tft);
ButtonWidget button2 = ButtonWidget(&tft);
ButtonWidget button3 = ButtonWidget(&tft);
ButtonWidget button4 = ButtonWidget(&tft);
ButtonWidget button5 = ButtonWidget(&tft);
ButtonWidget buttonIncreaseBrightness = ButtonWidget(&tft);
ButtonWidget buttonDecreaseBrightness = ButtonWidget(&tft);
ButtonWidget buttonResetCounter = ButtonWidget(&tft);
const int menuButtonWidth = 60;
const int menuButtonHeight = 30;
const int menuButtonMargin = 10;
// int menuButtonX, menuButtonY; // Position des Menü-Buttons
int menuButtonX = tft.width() - menuButtonWidth - menuButtonMargin;
int menuButtonY = tft.height() - menuButtonHeight - menuButtonMargin;
const int simButtonWidth = 60;
const int simButtonHeight = 30;
const int simButtonMargin = 40;
// int simButtonX, simButtonY; // Position des Menü-Buttons
int simButtonX = tft.width() - simButtonWidth - simButtonMargin;
int simButtonY = tft.height() - simButtonHeight - simButtonMargin;
bool isSimulationActive = false; // hier ändern
unsigned long startTime = 0;
bool timerRunning = false;
bool reached300 = false;
bool weightOver50 = false;
unsigned long currentTime;
int screenWidth = tft.width(); // Die Breite des Bildschirms
int graphWidth = 140;
int graphHeight = 50;
int graphX = 90;
int graphY = 130;
int graphXValue = 0; // Global deklarieren
unsigned long lastTapTime = 0;
int tapCount = 0;
const unsigned long doubleTapInterval = 550; // Zeitfenster für Doppeltap in Millisekunden
RTC_DATA_ATTR int counter = 0;               // Zähler, der nach dem Aufwachen aus dem Tiefschlafmodus erhalten bleibt
int lastValue = -1;
int lastValueD = -1;
int16_t adc0;
String qrData;
QRCode qrcode;
// #define ANALOG_PIN 28
//  Konstanten-Definitionen
/*#define USE_WIRE
#define SENSOR_SDA  23
#define SENSOR_SCL  24
#define IMU_ADDRESS 0x6B
#define PERFORM_CALIBRATION*/
#define MAX_VALUES 5
int values[MAX_VALUES];
int index5 = 0;
long sum = 0;
float weight = 0;
unsigned long elapsedTime;
// Globale Variablen
float peakValues[5] = {0, 0, 0, 0, 0}; // Array für Spitzenwerte
int peakValuesIndex = 0;               // Aktueller Index im Array
int targetValue;                       // Startwert
volatile unsigned long lastCounterUpdateTime = 0;
const unsigned long shutdownInterval = 60000; // 60 Sekunden in Millisekunden
int brightness = 50;                          // Startwert für die Helligkeit, passen Sie diesen Wert nach Bedarf an
const int brightnessAddress = 10;             // Zum Beispiel Adresse 10

const char *ssid = "BeckerHD_IOT";
const char *password = "daaistmeinwlanhier";

const char *host = "esp32";

#define MOTOR_PIN 25

WebServer server(81);

/*
 * Login page
 */

const char *webPage = R"(
<!DOCTYPE html>
<html>
<body>
    <h1>Serielle Daten:</h1>
    <pre id="serialOutput"></pre>
    <script>
        function fetchSerialData() {
            fetch('/serial-data')
                .then(response => response.text())
                .then(data => {
                    document.getElementById('serialOutput').textContent += data;
                });
        }

        setInterval(fetchSerialData, 1000); // Daten jede Sekunde abfragen
    </script>
</body>
</html>
)";

const char *loginIndex =
    "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
    "<tr>"
    "<td colspan=2>"
    "<center><font size=4><b>ESP32 Login Page</b></font></center>"
    "<br>"
    "</td>"
    "<br>"
    "<br>"
    "</tr>"
    "<tr>"
    "<td>Username:</td>"
    "<td><input type='text' size=25 name='userid'><br></td>"
    "</tr>"
    "<br>"
    "<br>"
    "<tr>"
    "<td>Password:</td>"
    "<td><input type='Password' size=25 name='pwd'><br></td>"
    "<br>"
    "<br>"
    "</tr>"
    "<tr>"
    "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
    "</tr>"
    "</table>"
    "</form>"
    "<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
    "</script>";

/*
 * Server Index Page
 */

const char *serverIndex =
    "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
    "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
    "<input type='file' name='update'>"
    "<input type='submit' value='Update'>"
    "</form>"
    "<div id='prg'>progress: 0%</div>"
    "<script>"
    "$('form').submit(function(e){"
    "e.preventDefault();"
    "var form = $('#upload_form')[0];"
    "var data = new FormData(form);"
    " $.ajax({"
    "url: '/update',"
    "type: 'POST',"
    "data: data,"
    "contentType: false,"
    "processData:false,"
    "xhr: function() {"
    "var xhr = new window.XMLHttpRequest();"
    "xhr.upload.addEventListener('progress', function(evt) {"
    "if (evt.lengthComputable) {"
    "var per = evt.loaded / evt.total;"
    "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
    "}"
    "}, false);"
    "return xhr;"
    "},"
    "success:function(d, s) {"
    "console.log('success!')"
    "},"
    "error: function (a, b, c) {"
    "}"
    "});"
    "});"
    "</script>";

float simulatedWeight = 0.0;
bool increasing = true;
std::random_device rd;
std::mt19937 gen(rd());
static uint8_t lastGesture = 0;
static bool clickHandled = false;     // Neue Variable für die Klick-Verarbeitung
static bool longPressHandled = false; // Neue Variable für die Langzeit-Druck-Verarbeitung
float highestWeight = 0.0;
float saveWeight = 0.0;
unsigned long mytimerStart = 0;
bool timerActive = false;
unsigned long button3PressTime = 0;
bool button3Mode = false;
int counter2 = 0; // Zähler für die Anzahl der gesendeten Nachrichten

unsigned long previousMillis = 0;   // Speichert den letzten Zeitpunkt, zu dem die Aktion ausgeführt wurde
const long interval = 100;          // Intervall, in dem Aktionen ausgeführt werden (100 Millisekunden in diesem Beispiel)
static bool lastButtonState = HIGH; // Speichern Sie den letzten Zustand des Knopfes
bool currentButtonState = digitalRead(BUTTON_PIN);
float startWeightThreshold = 10; // Schwellenwert, ab dem eine Prüfung beginnt
float endWeightThreshold = 10;   // Schwellenwert, unter dem eine Prüfung endet
bool isTesting = false;          // Flag, um den Zustand der Prüfung zu verfolgen
// Definieren eines Schwellenwerts für Wischgesten
const int SWIPE_THRESHOLD = 5; // Sie können diesen Wert anpassen
#define CALIBRATION_FILE "/TouchCalData1"
#define REPEAT_CAL false
unsigned long lastTouchTime = 0;         // Zeitpunkt der letzten Touch-Eingabe
const unsigned long debounceDelay = 200; // Debounce-Zeit in Millisekunden
const int simulationButtonWidth = 60;
const int simulationButtonHeight = 30;
int simulationButtonX = menuButtonX + 40;                          // Gleiche X-Position wie der Menü-Button
int simulationButtonY = menuButtonY - simulationButtonHeight - 10; // Über dem Menü-Buttonb
int sliderX, sliderY, sliderWidth, sliderHeight;
int sliderMinValue = 100;      // Minimale Anzeigezeit in Millisekunden
int sliderMaxValue = 10000;    // Maximale Anzeigezeit in Millisekunden
int sliderCurrentValue = 1000; // Startwert der Anzeigezeit
int screenHeight = tft.height();

struct PeakValueDisplay
{
  int x, y, width, height;
  float value;
};

struct PruefungsDaten
{
  String qrCode;
  // Hier können Sie weitere Daten hinzufügen, falls erforderlich
};

const int maxPruefungen = 6; // Maximale Anzahl der Prüfungen, die gespeichert werden können
PruefungsDaten gespeichertePruefungen[maxPruefungen];
int aktuellePruefungIndex = 0; // Index für die aktuelle Prüfung im Array

PeakValueDisplay peakValueDisplays[5]; // Array für die Anzeige der Spitzenwerte

// Funktionsprototypen
void setup();
void loop();
void drawCenterNumber(int number);
void drawQRCode(const String &data);
void drawEndCaps();

void showCounter();
void drawBar(int value, int maxValue);
void performManualTare();
void performEnhancedTare();
void updateCounter();
void processWeight();
void checkTouch();
void updateGraphAndTrace(unsigned long currentMillis);
void displayTargetValue();
void drawDial(int value, int maxValue); // Behalten Sie diese Deklaration bei
void updateQRData(float newWeight);     // Nur diese Deklaration beibehalten
float getWeight();                      // statt void getWeight();
void saveTargetValue();
void handleWeightChanges();
void displayMenu();
uint16_t lerpColor(uint16_t color1, uint16_t color2, float t);
void processGestures();
void handleSwipeUp();
void handleSwipeDown();
void closeMenu();
float simulateWeightChange();
void handleDoubleTap();
void calibrateTouchscreen();
void loadCalibration(int &xMin, int &xMax, int &yMin, int &yMax);
void saveCalibration(int &xMin, int &xMax, int &yMin, int &yMax);
void drawMenuButton();
void checkMenuButtonTouch();
void handleMenuButtonPress();
TS_Point getCalibratedPoint();
void touch_calibrate();
void saveCalibration(uint16_t calData[]);
void loadCalibration(uint16_t calData[]);
bool isButtonTouched(int touchX, int touchY, int buttonX, int buttonY, int width, int height);
bool isMenuButtonTouched(uint16_t t_x, uint16_t t_y);
void handleButton1Press();
void handleButton2Press();
void handleButton3Press();
void handleButton4Press();
void toggleWeightSimulation();
bool isSimulationButtonTouched(uint16_t t_x, uint16_t t_y);
void handleExtraButtonPress();
bool isExtraButtonTouched(uint16_t t_x, uint16_t t_y);
void setDisplayDuration(unsigned long duration);
void updatePeakValues(float newValue);
void displayPeakValues();
bool isTouched(int touchX, int touchY, int buttonX, int buttonY, int width, int height);
void toggleWiFi();
void sendQRCodeRequest(const String &data);
void toggleWebServerAndWiFi();
void webcounter();
void recvMsg(uint8_t *data, size_t len);
void printGespeichertePruefungen();
void aktualisiereGespeichertePruefungen(const String &neueDaten);
void IRAM_ATTR handleButtonPress();
void goToDeepSleep();
void handleBrightnessUpPress();
void handleBrightnessDownPress();
void saveBrightness(int brightness);
int loadBrightness();
void handleResetCounterPress();

#define CS_PIN (33)
#define DC_PIN (7)
#define RST_PIN (-1)
#define BK_LIGHT_PIN (38)

// touch screen
#define TOUCHSCREEN_SCLK_PIN (1)
#define TOUCHSCREEN_MISO_PIN (4)
#define TOUCHSCREEN_MOSI_PIN (3)
#define TOUCHSCREEN_CS_PIN (2)
#define TOUCHSCREEN_IRQ_PIN (9)

#define PWR_EN_PIN (10)
#define PWR_ON_PIN (14)

void setup()
{

  Serial.begin(9600);
  // DEV_Module_Init();
  // CST816S_init(CST816S_Point_Mode);
  // DEV_KEY_Config(Touch_INT_PIN);
  // attachInterrupt(Touch_INT_PIN, &Touch_INT_callback, RISING);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 0);                                   // Verwenden Sie GPIO 33 als Wake-up-Quelle, Wake-up auf LOW
  pinMode(BUTTON_PIN, INPUT_PULLUP);                                              // Konfigurieren des Pins als Eingang mit Pull-Up
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING); // Interrupt bei FALLING-Edge
  // pinMode(adcPin, INPUT);
  // pinMode(TFT_BL, OUTPUT);
  // digitalWrite(TFT_BL, HIGH);
  // pinMode(PWR_EN_PIN, OUTPUT);
  // digitalWrite(PWR_EN_PIN, HIGH);
  // pinMode(38, OUTPUT);
  // digitalWrite(38, HIGH);
  int r = 0; // Beispielwert, setzen Sie diesen Wert entsprechend Ihrer Anforderung
  int g = 1; // Beispielwert
  int b = 0; // Beispielwert

  tft.init(); // Display initialisieren
  tft.begin();
  // tft.setSwapBytes(true);

  tft.setRotation(1);

  // spiTouch.begin(TOUCH_SCLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);

  ts.begin();
  ts.setRotation(0);

  delay(500);
  Serial.println(targetValue);
  if (!LittleFS.begin())
  {
    Serial.println("Formating file system");
    LittleFS.format();
    LittleFS.begin();
  }
  // EEPROM.get(0, targetValue);  // Lesen des Wertes von Adresse 0
  if (targetValue != 300 && targetValue != 500)
  {
    targetValue = 300; // Setzen eines Standardwerts, falls kein gültiger Wert gespeichert ist
  }
  // Serial.println(targetValue);
  // tft.begin(60000);
  // tft.setRotation(0);

  tft.fillScreen(TFT_BLACK);
  tft.drawBitmap(-20, 0, logo, 270, 239, TFT_WHITE);

  // tft.pushImage(0, 0, 240, 320, (uint16_t *)logo);
  delay(500);
  tft.fillScreen(TFT_BLACK);
  // drawPartialCircles();
  startTime = millis(); // Timer starten
  timerRunning = true;

  gr.createGraph(graphWidth, graphHeight, tft.color565(5, 5, 5));
  gr.setGraphScale(0, 30, 0, targetValue); // Skala entsprechend currentValue anpassen
  gr.setGraphGrid(0, 10.0, 0, 100.0, TFT_WHITE);
  gr.drawGraph(graphX, graphY);
  tr.startTrace(TFT_GREEN);

  menuButtonX = tft.width() - menuButtonWidth - menuButtonMargin;
  menuButtonY = tft.height() - menuButtonHeight - menuButtonMargin;
  drawMenuButton();
  Serial.println("Setup Fertig");
  scale.begin(dataPin, clockPin);
  scale.set_gain(128); // oder ein anderer passender Wert
  Serial.println("Setup Fertig1");
  // Setzen Sie hier die kalibrierten Werte ein
  scale.set_offset(235555);
  scale.set_scale(-1243.5453);
  Serial.println("Setup Fertig2");
  delay(50);
  scale.tare(10);
  Serial.println("Setup Fertig3");
  performManualTare();
  Serial.println("Setup Fertig4");
  // Variable weightOver50 initialisieren
  weightOver50 = false;
  if (flag == 1)
  {
    flag = 0;
    // break;
  }
  Serial.println("Setup Fertig5");
  // loadCalibration(calData);

  // Falls keine gültigen Kalibrierungsdaten vorhanden oder Neukalibrierung gewünscht
  if (REPEAT_CAL)
  {
    touch_calibrate();
    tft.fillScreen(TFT_BLACK);
  }
  touch_calibrate();
  Serial.println("Calibration Data:");
  for (int i = 0; i < 5; i++)
  {
    Serial.print("calData[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(calData[i]);
  }

  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("WiFi connection failed");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  /*use mdns for host name resolution*/
  if (!MDNS.begin(host))
  { // http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  server.on("/serial", HTTP_GET, []()
            {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", webPage); });
  server.on("/serial-data", HTTP_GET, []()
            {
              String combinedData = "Serielle Daten:\n" + serialData + "\nQR-Code Daten:\n" + qrData;
              server.send(200, "text/plain", combinedData);
              sendQRCodeRequest(qrData); // qrData sollte die aktuellen Daten für den QR-Code enthalten

              serialData = ""; // Setzt serialData zurück, nachdem die Daten gesendet wurden
            });
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []()
            {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex); });
  server.on("/serverIndex", HTTP_GET, []()
            {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex); });
  /*handling uploading firmware file */

  /* server.on(
      "/update", HTTP_POST, []()
      {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); },
      []()
      {
        HTTPUpload &upload = server.upload();
        if (upload.status == UPLOAD_FILE_START)
        {
          Serial.printf("Update: %s\n", upload.filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN))
          { // start with max available size
            Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_WRITE)
        {
           flashing firmware to ESP
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
          {
            Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_END)
        {
          if (Update.end(true))
          { // true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
          }
          else
          {
            Update.printError(Serial);
          }
        }
      });
   */
  // server.begin();
  //  WebSerial.begin(&server5);
  /* Attach Message Callback */
  // WebSerial.msgCallback(recvMsg);
  // server5.begin();
  // Serial.println("WebSerial Server Started");

  int brightness = loadBrightness(); // Lade den gespeicherten Helligkeitswert
  if (brightness == -1)
  {                   // Wenn der Wert nicht gefunden wurde
    brightness = 100; // Setze den Standardwert
  }
  analogWrite(TFT_BL, 100); // Stelle die Helligkeit des Displays ein
  pinMode(MOTOR_PIN, OUTPUT); // MOTOR_PIN als Ausgang

}

void loop()
{
  if (millis() - lastCounterUpdateTime > shutdownInterval && lastCounterUpdateTime != 0)
  {
    goToDeepSleep();
  }
  /*String data = Serial.readStringUntil('\n');
  data.trim(); // Entfernt führende und nachfolgende Leerzeichen
  serialData += data + "\n"; // Daten zur globalen Variable hinzufügen*/
  unsigned long currentMillis = millis();
  int graphXValue = 0;       // Initialisieren Sie graphXValue
  uint16_t t_x = 0, t_y = 0; // Variablen für Touch-Koordinaten
  tft.getTouch(&t_x, &t_y, 250);
  if (tft.getTouch(&t_x, &t_y, 250))
  {
    if (millis() - lastTouchTime > debounceDelay)
    { // Debounce-Überprüfung
      if (isMenuDisplayed)
      {
        if (button1.contains(t_x, t_y))
        {
          handleButton1Press();
        }
        else if (button2.contains(t_x, t_y))
        {
          handleButton2Press();
        }
        else if (button3.contains(t_x, t_y))
        {
          handleButton3Press();
        }
        else if (button4.contains(t_x, t_y))
        {
          handleButton4Press();
        }
        // Überprüfung für den Button zur Erhöhung der Helligkeit
        else if (buttonIncreaseBrightness.contains(t_x, t_y))
        {
          handleBrightnessUpPress();
        }
        // Überprüfung für den Button zur Verringerung der Helligkeit
        else if (buttonDecreaseBrightness.contains(t_x, t_y))
        {
          handleBrightnessDownPress();
        }
        else if (buttonResetCounter.contains(t_x, t_y))
        {
          handleResetCounterPress();
        }
        // Prüfen, ob der Slider berührt wurde
        else if (t_y > sliderY - 10 && t_y < sliderY + sliderHeight + 10)
        {
          int newValue = map(t_x, sliderX, sliderX + sliderWidth, sliderMinValue, sliderMaxValue);
          newValue = constrain(newValue, sliderMinValue, sliderMaxValue);
          if (newValue != sliderCurrentValue)
          {
            sliderCurrentValue = newValue;
            setDisplayDuration(sliderCurrentValue); // Funktion zum Aktualisieren der Anzeigedauer
            displayMenu();                          // Menü neu zeichnen, um den Slider zu aktualisieren
          }
        }
      }
      else
      {
        for (int i = 5; i >= 0; i--)
        {
          if (gespeichertePruefungen[i].qrCode != "")
                {
                    // Überprüfen, ob der Touch-Bereich mit dem QR-Code übereinstimmt
                    if (isTouched(t_x, t_y, peakValueDisplays[i].x, peakValueDisplays[i].y, peakValueDisplays[i].width, peakValueDisplays[i].height))
                    {
                        Serial.print("Touch erkannt bei Index: ");
                        Serial.println(i);
                        Serial.print("Angezeigter QR-Code: ");
                        Serial.println(gespeichertePruefungen[i].qrCode);

                        // Zeichnet den QR-Code auf dem Display
                        drawQRCode(gespeichertePruefungen[i].qrCode);
                        break;
                    }
                }
        }
      }
      lastTouchTime = millis(); // Aktualisieren der Zeit der letzten Touch-Eingabe
    }
  }

  // processGestures(); // Aufruf der ausgelagerten Funktion
  //  Serial.println(weight);
  //   Überprüfen, ob das Menü NICHT angezeigt wird

  if (!isMenuDisplayed)
  {
    if (isSimulationActive)
    {
      weight = simulateWeightChange();
    }
    else
    {
      weight = getWeight(); // stimmt das?
      // Hier die normale Gewichtsberechnung oder -abfrage einfügen
      // processWeight(); // oder dass?
    }

    checkMenuButtonTouch();
    processWeight();                    // Verarbeitung des Gewichts
    updateGraphAndTrace(currentMillis); // Aktualisierung des Graphen und Traces
    updateCounter();                    // Aktualisierung des Zählers

    displayPeakValues();
    if(weight > targetValue) {
    digitalWrite(MOTOR_PIN, HIGH); // Motor einschalten, wenn Gewicht über Zielwert
  } else {
    digitalWrite(MOTOR_PIN, LOW); // Motor ausschalten, wenn Gewicht unter oder gleich Zielwert
  }
  }

  showCounter();

  // lastButtonState = currentButtonState; // Aktualisieren Sie den letzten Zustand des Knopfes
  // webcounter ();
  //     if (isWebServerActive) {
  //     server.handleClient();
  // }

  // WebSerial.print(F("IP address: "));
  // WebSerial.println(WiFi.localIP());
  // WebSerial.printf("Millis=%lu\n", millis());
  // WebSerial.printf("Free heap=[%u]\n", ESP.getFreeHeap());
  // printGespeichertePruefungen();
}

void IRAM_ATTR handleButtonPress()
{
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  // Nur akzeptieren, wenn die letzte Unterbrechung vor mehr als 200ms war
  if (interruptTime - lastInterruptTime > 200)
  {
    counter++; // Zähler erhöhen
    lastInterruptTime = interruptTime;
    lastCounterUpdateTime = millis();
  }
}

void goToDeepSleep()
{
  Serial.println("Zähler wurde 10 Sekunden lang nicht erhöht. Gehe in Tiefschlaf...");
  esp_deep_sleep_start(); // Setzt das Gerät in den Tiefschlafmodus
}

void recvMsg(uint8_t *data, size_t len)
{
  WebSerial.println("Received Data...");
  String d = "";
  for (int i = 0; i < len; i++)
  {
    d += char(data[i]);
  }
  WebSerial.println(d);
}

void printGespeichertePruefungen()
{
  Serial.println("Aktueller Inhalt von gespeichertePruefungen:");
  for (int i = 0; i < maxPruefungen; i++)
  {
    Serial.print("gespeichertePruefungen[");
    Serial.print(i);
    Serial.print("].qrCode = ");
    Serial.println(gespeichertePruefungen[i].qrCode);
    // Fügen Sie hier weitere Ausgaben hinzu, falls Sie weitere Attribute in PruefungsDaten haben
  }
}

void aktualisiereGespeichertePruefungen(const String &neueDaten)
{
    // Verschieben aller vorhandenen Daten um eine Position nach vorne
  for (int i = 0; i < maxPruefungen - 1; i++)
  {
        gespeichertePruefungen[i] = gespeichertePruefungen[i + 1];
    }

    // Hinzufügen der neuen Daten am Ende des Arrays
    gespeichertePruefungen[maxPruefungen - 1].qrCode = neueDaten;

    // Debugging: Ausgabe der gespeicherten Prüfungsdaten
    Serial.println("Aktualisierte gespeichertePruefungen:");
    for (int i = 0; i < maxPruefungen; i++) {
        Serial.print("gespeichertePruefungen[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.println(gespeichertePruefungen[i].qrCode);
    }
}

void webcounter()
{
  if (counter2 < 100)
  {
    // Füge den Gewichtseintrag zu den seriellen Daten hinzu
    serialData += "Gewicht: " + String(weight) + "\n";
    counter2++; // Erhöhe den Zähler
  }
  else
  {
    // Wenn 100 Einträge erreicht sind, leere serialData und setze den Zähler zurück
    serialData = ""; // Löscht den Inhalt von serialData
    counter2 = 0;    // Setzt den Zähler zurück
  }
}

bool isTouched(int touchX, int touchY, int buttonX, int buttonY, int width, int height)
{
  return touchX >= buttonX && touchX <= (buttonX + width) &&
         touchY >= buttonY && touchY <= (buttonY + height);
}

void setDisplayDuration(unsigned long duration)
{
  displayDuration = duration;
}

void sendQRCodeRequest(const String &data)
{
  HTTPClient http;
  String serverPath = "https://api.qrserver.com/v1/create-qr-code/?size=150x150&data=" + data;

  http.begin(serverPath);
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0)
  {
    String payload = http.getString();
    // Senden Sie das empfangene Bild an den Webbrowser
    server.send(200, "image/png", payload);
  }
  else
  {
    Serial.print("Error on sending GET Request: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void updatePeakValues(float newValue)
{
  peakValues[peakValuesIndex] = newValue;      // Neuen Wert speichern
  peakValuesIndex = (peakValuesIndex + 1) % 5; // Index aktualisieren
}

void displayPeakValues() {
    int startX = 5;      // Startposition X
    int startY = 130;    // Startposition Y
    int lineHeight = 23; // Höhe jeder Zeile
    int valueWidth = 60; // Breite des Anzeigebereichs für jeden Wert
    int boxPadding = 3;  // Abstand zwischen Text und Kastenrand

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    for (int i = 0; i < 5; i++) {
        int indexToShow = (peakValuesIndex + 4 - i) % 5;
        int valueInt = (int)peakValues[indexToShow];
        String valueStr = String(valueInt) + "N";

        peakValueDisplays[i].x = startX - boxPadding;
        peakValueDisplays[i].y = startY + i * lineHeight - boxPadding;

        // Vergrößere und versetze den Bereich für den ersten Wert
        if (i == 0) {
            peakValueDisplays[i].width = valueWidth + boxPadding * 4;
            peakValueDisplays[i].height = lineHeight + boxPadding * 4;
            peakValueDisplays[i].y += 10; // Optional: leicht versetzen
        } else {
            peakValueDisplays[i].width = valueWidth + boxPadding * 2;
            peakValueDisplays[i].height = lineHeight + boxPadding * 2;
        }

        peakValueDisplays[i].value = peakValues[indexToShow];

        tft.fillRect(peakValueDisplays[i].x, peakValueDisplays[i].y, peakValueDisplays[i].width, peakValueDisplays[i].height, TFT_GREY);
        tft.setCursor(startX, startY + i * lineHeight);
        tft.print(valueStr);
    }

    // Debugging: Ausgabe der aktuellen peakValues
    Serial.println("Aktuelle peakValues:");
    for (int i = 0; i < 5; i++) {
        Serial.print("peakValues[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.println(peakValues[i]);
    }
}


void toggleWeightSimulation()
{
  isSimulationActive = !isSimulationActive;
  isSimButtonActive = !isSimButtonActive; // Umschalten des Button-Zustands
  drawMenuButton();                       // Button neu zeichnen
  if (isSimulationActive)
  {
    Serial.println("Simulation aktiviert");
    serialData += "Simulation aktiviert\n";
  }
  else
  {
    Serial.println("Simulation deaktiviert");
    serialData += "Simulation deaktiviert\n";
  }
}

void handleButton1Press()
{
  // Fügen Sie hier die Logik für Button 1 hinzu
  Serial.println("Button 1 gedrückt");
  serialData += "Button 1 gedrückt\n";
  targetValue = 300;
  saveTargetValue();
  closeMenu();
  
}

void handleButton2Press()
{
  // Fügen Sie hier die Logik für Button 2 hinzu
  Serial.println("Button 2 gedrückt");
  serialData += "Button 2 gedrückt\n";
  targetValue = 500;
  saveTargetValue();
  closeMenu();
  
}

void handleButton3Press()
{
  // Fügen Sie hier die Logik für Button 3 hinzu
  Serial.println("Button 3 gedrückt");
  serialData += "Button 3 gedrückt\n";
  // toggleWiFi(); // WLAN ein- oder ausschalten
  toggleWebServerAndWiFi();
  // drawQRCode(qrData);
  closeMenu();

  
}

void toggleWebServerAndWiFi()
{
  if (!isWebServerActive)
  {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
    }
    server.begin();
    isWebServerActive = true;
  }
  else
  {
    server.stop();
    WiFi.disconnect();
    isWebServerActive = false;
  }
}

void toggleWiFi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.begin(ssid, password);
    Serial.println("WLAN verbunden");
  }
  else
  {
    WiFi.disconnect();
    Serial.println("WLAN getrennt");
  }
}

void handleButton4Press()
{
  Serial.println("Button 4 gedrückt");
  serialData += "Button 4 gedrückt\n";
  performManualTare();
  performEnhancedTare();
  closeMenu();
  // Fügen Sie hier die Logik für Button 4 hinzu
}

void handleBrightnessUpPress()
{
  // Stellen Sie sicher, dass der Helligkeitswert nicht über 255 steigt
  brightness = min(brightness + 25, 255);
  analogWrite(TFT_BL, brightness); // Aktualisiere die Helligkeit des Displays
  saveBrightness(brightness);      // Speichere den neuen Helligkeitswert
  Serial.println("Helligkeit erhöht");
}

void handleBrightnessDownPress()
{
  // Stellen Sie sicher, dass der Helligkeitswert nicht unter 0 fällt
  brightness = max(brightness - 25, 5);
  analogWrite(TFT_BL, brightness); // Aktualisiere die Helligkeit des Displays
  saveBrightness(brightness);      // Speichere den neuen Helligkeitswert
  Serial.println("Helligkeit verringert");
}
void handleMenuButtonPress()
{
  // Code für die Aktion, die ausgeführt werden soll, wenn der Menü-Button gedrückt wird
  Serial.println("Menü-Button gedrückt");
  serialData += "Menü-Button gedrückt\n";
  timerRunning = false;
  delay(200);
  tft.fillScreen(TFT_BLACK);
  displayMenu();
  // Weitere Aktionen, z.B. Menü anzeigen ...
}

void checkMenuButtonTouch()
{
  uint16_t t_x = 0, t_y = 0; // Variablen für Touch-Koordinaten
  if (tft.getTouch(&t_x, &t_y, 250))
  {

    if (isMenuButtonTouched(t_x, t_y))
    {
      // Menüknopf wurde berührt
      handleMenuButtonPress();
    }
    else if (isSimulationButtonTouched(t_x, t_y))
    {
      // Simulationsknopf wurde berührt
      toggleWeightSimulation();
    }
    else if (isExtraButtonTouched(t_x, t_y))
    {
      // QR-Button wurde berührt
      handleExtraButtonPress();
    }
  }
}

void handleExtraButtonPress()
{
  Serial.println("QR-Button gedrückt");
  serialData += "QR-Button gedrückt\n";
  drawQRCode(gespeichertePruefungen[0].qrCode);
  // Fügen Sie hier die gewünschte Logik ein, z.B. das Anzeigen eines QR-Codes
}

bool isExtraButtonTouched(uint16_t t_x, uint16_t t_y)
{
  int screenWidth = tft.width();
  int screenHeight = tft.height();
  int extraButtonWidth = 60;
  int extraButtonHeight = 30;
  int extraButtonMargin = 10;
  int extraButtonX = screenWidth - extraButtonWidth - extraButtonMargin;
  int extraButtonY = screenHeight - extraButtonHeight - extraButtonMargin - 70; // 80 als Beispielabstand
  return t_x >= extraButtonX && t_x <= (extraButtonX + extraButtonWidth) &&
         t_y >= extraButtonY && t_y <= (extraButtonY + extraButtonHeight);
}

bool isSimulationButtonTouched(uint16_t t_x, uint16_t t_y)
{
  // Hier müssen Sie die Koordinaten und Größe des Simulationsbuttons einsetzen
  int simbuttonX = menuButtonX;
  int simbuttonY = menuButtonY - simulationButtonHeight - 10;
  int simbuttonWidth = 60;
  int simbuttonHeight = 30;

  return t_x >= simbuttonX && t_x <= (simbuttonX + simbuttonWidth) &&
         t_y >= simbuttonY && t_y <= (simbuttonY + simbuttonHeight);
}

bool isMenuButtonTouched(uint16_t t_x, uint16_t t_y)
{
  return t_x >= menuButtonX && t_x <= (menuButtonX + menuButtonWidth) &&
         t_y >= menuButtonY && t_y <= (menuButtonY + menuButtonHeight);
}

void touch_calibrate()
{

  // check file system exists
  if (!LittleFS.begin())
  {
    Serial.println("Formating file system");
    LittleFS.format();
    LittleFS.begin();
  }

  // check if calibration file exists and size is correct
  if (LittleFS.exists(CALIBRATION_FILE))
  {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      LittleFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = LittleFS.open(CALIBRATION_FILE, "r");
      if (f)
      {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL)
  {
    // calibration data valid
    tft.setTouch(calData);
  }
  else
  {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();
    Serial.println("Touch corners as indicated");
    serialData += "Touch corners as indicated\n";
    if (REPEAT_CAL)
    {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);
    Serial.println("Calibration Data:");
    serialData += "Calibration Data:\n";
    for (int i = 0; i < 5; i++)
    {
      Serial.print("calData[");
      serialData += "calData[";
      Serial.print(i);
      serialData += i;
      Serial.print("] = ");
      serialData += "] = ";
      Serial.println(calData[i]);
      serialData += calData[i];
    }
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = LittleFS.open(CALIBRATION_FILE, "w");
    if (f)
    {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

void updateCounter()
{
  if (weight > 50 && !weightOver50)
  {
    weightOver50 = true;
    counter++;
    showCounter();
    lastCounterUpdateTime = millis();
  }
  else if (weight < 10 && weightOver50)
  {
    weightOver50 = false;
  }
}

void processGestures()
{
  if (ts.touched())
  {
    if (!isTouching)
    {
      // Startpunkt der Berührung erfassen
      startPoint = ts.getPoint();

      isTouching = true;

      unsigned long currentTime = millis();
      if (currentTime - lastTapTime <= doubleTapInterval)
      {
        tapCount++;
      }
      else
      {
        tapCount = 1;
      }
      lastTapTime = currentTime;

      if (tapCount >= 2)
      {
        // Doppeltap erkannt
        handleDoubleTap();
        tapCount = 0; // Zurücksetzen des Zählers
      }
    }
  }
  else
  {
    if (isTouching)
    {
      // Endpunkt der Berührung erfassen
      TS_Point endPoint = ts.getPoint();
      isTouching = false;

      // Prüfen auf Swipe nach oben
      if (startPoint.y - endPoint.y > swipeThreshold)
      {
        handleSwipeUp();
      }
      // Prüfen auf Swipe nach unten
      else if (endPoint.y - startPoint.y > swipeThreshold)
      {
        handleSwipeDown();
      }
    }
  }
}

void handleDoubleTap()
{
  // Logik für Doppeltap
  Serial.println("Doppeltap erkannt");
  drawQRCode(qrData); // Hier Ihre Aktionen einfügen
}

void handleSwipeUp()
{
  // Hier die Aktionen bei Swipe nach oben definieren
  Serial.println("Swipe nach oben erkannt");
  timerRunning = false;
  tft.fillScreen(TFT_BLACK);
  displayMenu();
}

void handleSwipeDown()
{
  // Hier die Aktionen bei Swipe nach oben definieren
  Serial.println("Swipe nach unten erkannt");
  closeMenu();
}

void saveBrightness(int newBrightness)
{
  int currentBrightness = EEPROM.read(brightnessAddress);
  if (currentBrightness != newBrightness)
  {
    EEPROM.write(brightnessAddress, newBrightness);
  }
}

int loadBrightness()
{
  return EEPROM.read(brightnessAddress);
}

float getRandomStepSize(float min, float max)
{
  std::uniform_real_distribution<> dis(min, max);
  return dis(gen);
}

float simulateWeightChange()
{
  float maxWeight = targetValue;
  float minWeight = 0.0;

  // Definieren Sie den Bereich für zufällige Schrittgrößen
  float minStepSizeUp = 60.0;    // Minimaler Schrittgröße beim Hochzählen
  float maxStepSizeUp = 20.0;    // Maximaler Schrittgröße beim Hochzählen
  float minStepSizeDown = 60.0;  // Minimaler Schrittgröße beim Runterzählen
  float maxStepSizeDown = 200.0; // Maximaler Schrittgröße beim Runterzählen

  if (increasing)
  {
    if (simulatedWeight < maxWeight)
    {
      float stepSizeUp = getRandomStepSize(minStepSizeUp, maxStepSizeUp);
      simulatedWeight += stepSizeUp;
      if (simulatedWeight > maxWeight)
      {
        simulatedWeight = maxWeight;
        increasing = false;
      }
    }
  }
  else
  {
    if (simulatedWeight > minWeight)
    {
      float stepSizeDown = getRandomStepSize(minStepSizeDown, maxStepSizeDown);
      simulatedWeight -= stepSizeDown;
      if (simulatedWeight < minWeight)
      {
        simulatedWeight = minWeight;
        increasing = true;
      }
    }
  }
  delay(100);
  return simulatedWeight;
}

void updateGraphAndTrace(unsigned long currentMillis)
{
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    tr.addPoint(graphXValue, weight); // Gewichtswert zum Trace hinzufügen
    graphXValue++;                    // X-Achsen-Wert erhöhen

    if (graphXValue >= 31)
    {
      graphXValue = 0;
      gr.drawGraph(graphX, graphY); // Neuzeichnen des Graphen
      tr.startTrace(TFT_GREEN);     // Neuen Trace starten
    }

    if (weight != lastValue)
    {
      drawDial(weight, targetValue);
      drawBar(weight, targetValue);
      lastValue = weight;
      displayTargetValue();
    }
  }
}

void drawMenuButton()
{
  int screenWidth = tft.width();
  int screenHeight = tft.height();
  int buttonWidth = 60;  // Breite des Buttons
  int buttonHeight = 30; // Höhe des Buttons
  int buttonMargin = 10; // Abstand vom Rand des Bildschirms

  // Position des Buttons unten rechts
  int buttonX = screenWidth - buttonWidth - buttonMargin;
  int buttonY = screenHeight - buttonHeight - buttonMargin;

  // Zeichnen des Buttons
  tft.fillRect(buttonX, buttonY, buttonWidth, buttonHeight, TFT_BLUE);  // Button-Hintergrund
  tft.drawRect(buttonX, buttonY, buttonWidth, buttonHeight, TFT_WHITE); // Button-Rahmen

  // Text im Button
  String buttonText = "Menu";
  tft.setTextSize(2);
  int textWidth = tft.textWidth(buttonText.c_str());
  int textX = buttonX + (buttonWidth - textWidth) / 2; // Zentrieren des Textes im Button
  int textY = buttonY + (buttonHeight / 4);            // Anpassung der Y-Position für visuelle Zentrierung
  tft.setTextColor(TFT_WHITE, TFT_BLUE);               // Textfarbe und Hintergrundfarbe
  tft.setCursor(textX, textY);
  tft.print(buttonText);

  int simbuttonWidth = 60;  // Breite des Buttons
  int simbuttonHeight = 30; // Höhe des Buttons
  int simbuttonMargin = 10; // Abstand vom Rand des Bildschirms

  int simbuttonX = screenWidth - buttonWidth - buttonMargin;
  int simbuttonY = screenHeight - buttonHeight - buttonMargin - 35;
  int simButtonColor = isSimButtonActive ? TFT_GREEN : TFT_BLUE; // Grün, wenn aktiv, sonst Blau

  // Zeichnen des Buttons
  tft.fillRect(simbuttonX, simbuttonY, simbuttonWidth, simbuttonHeight, simButtonColor); // Button-Hintergrund
  tft.drawRect(simbuttonX, simbuttonY, simbuttonWidth, simbuttonHeight, TFT_WHITE);      // Button-Rahmen

  // Text im Button
  String simbuttonText = "Sim";
  tft.setTextSize(2);
  int simtextWidth = tft.textWidth(simbuttonText.c_str());
  int simtextX = simbuttonX + (simbuttonWidth - simtextWidth) / 2; // Zentrieren des Textes im Button
  int simtextY = simbuttonY + (simbuttonHeight / 4);               // Anpassung der Y-Position für visuelle Zentrierung
  tft.setTextColor(TFT_WHITE, TFT_BLUE);                           // Textfarbe und Hintergrundfarbe
  tft.setCursor(simtextX, simtextY);
  tft.print(simbuttonText);
  // extra button
  int extraButtonWidth = 60;
  int extraButtonHeight = 30;
  int extraButtonMargin = 10;
  int extraButtonX = screenWidth - extraButtonWidth - extraButtonMargin;
  int extraButtonY = screenHeight - extraButtonHeight - extraButtonMargin - 70; // 80 als Beispielabstand

  // Zeichnen des Buttons
  tft.fillRect(extraButtonX, extraButtonY, extraButtonWidth, extraButtonHeight, TFT_BLUE);  // Button-Hintergrund
  tft.drawRect(extraButtonX, extraButtonY, extraButtonWidth, extraButtonHeight, TFT_WHITE); // Button-Rahmen

  // Text im Button
  String extraButtonText = "QR";
  tft.setTextSize(2);
  int extraTextWidth = tft.textWidth(extraButtonText.c_str());
  int extraTextX = extraButtonX + (extraButtonWidth - extraTextWidth) / 2; // Zentrieren des Textes im Button
  int extraTextY = extraButtonY + (extraButtonHeight / 4);                 // Anpassung der Y-Position für visuelle Zentrierung
  tft.setTextColor(TFT_WHITE, TFT_BLUE);                                   // Textfarbe und Hintergrundfarbe
  tft.setCursor(extraTextX, extraTextY);
  tft.print(extraButtonText);
}



void processWeight()
{
    static bool isTesting = false;
    weight = getWeight(); // Aktuelles Gewicht abrufen

    // Start der Prüfung
    if (!isTesting && weight > startWeightThreshold)
    {
        isTesting = true;
        qrData = ""; // Zurücksetzen der QR-Daten für eine neue Prüfung
        Serial.println("Prüfung gestartet");
    }

    // Während der Prüfung Daten sammeln
    if (isTesting)
    {
        updateQRData(weight); // Füge das aktuelle Gewicht zu qrData hinzu

        // Ende der Prüfung
        if (weight < endWeightThreshold)
        {
            isTesting = false;
            aktualisiereGespeichertePruefungen(qrData); // Daten speichern
            Serial.println("Prüfung beendet und Daten gespeichert");
            qrData = ""; // Zurücksetzen der QR-Daten für die nächste Prüfung
        }
    }

    // Zusätzliche Logik zur Verarbeitung des Gewichts
    handleWeightChanges();
}

void handleWeightChanges()
{
    if (weight > 10)
    {
        if (weight > highestWeight)
        {
            highestWeight = weight;
            saveWeight = weight;
        }
    }
    else if (!timerActive && highestWeight > 0)
    {
        updatePeakValues(highestWeight); // Update hier, um den Spitzenwert einmal hinzuzufügen
        drawCenterNumber(weight);
        mytimerStart = millis();
        timerActive = true;
    }

    if (timerActive)
    {
        if (millis() - mytimerStart > displayDuration)
        {
            highestWeight = 0.0;
            timerActive = false;
        }
        else
        {
            drawCenterNumber((int)highestWeight);
        }
    }
    else
    {
        drawCenterNumber((int)weight);
    }
}


void closeMenu()
{
  isMenuDisplayed = false;

  // Code zum Schließen des Menüs und Rückkehr zum normalen Betriebsmodus
  // Beispiel: Bildschirm löschen, Hauptbildschirm neu zeichnen usw.
  tft.fillScreen(TFT_BLACK);
  timerRunning = true;

  drawDial(weight, targetValue); // Zifferblatt zeichnen
  drawCenterNumber(weight);      // Ziffer anzeigen
  drawBar(weight, targetValue);  // Balken zeichnen

  displayTargetValue(); // Zeigt den Zielwert an

  startTime = millis(); // Startet den Timer neu

  // Zeichnen Sie andere Elemente, die benötigt werden, erneut
  gr.drawGraph(graphX, graphY);            // Zeichnen Sie den Graphen erneut
  tr.startTrace(TFT_GREEN);                // Starten Sie den Trace erneut
  gr.setGraphScale(0, 30, 0, targetValue); // Skala entsprechend currentValue anpassen
  drawMenuButton();
  button5.drawButton();
}

void drawCenterNumber(int number)
{
  static int lastDrawnNumber = -1; // Initialisieren Sie mit einem Wert, der sicher nicht gleich dem ersten Wert sein wird

  // Zeichne nur, wenn die neue Zahl verschieden ist
  if (number != lastDrawnNumber)
  {
    int x = tft.width() / 2;
    int y = tft.height() / 2;
    tft.setTextSize(4);

    String numStr = String(number);
    int w = tft.textWidth(numStr.c_str());
    int h = tft.fontHeight();
    int padding = 20; // zusätzlichen Platz hinzufügen

    // Lösche den alten Text
    tft.fillRect(x - w / 2 - padding, y - h / 2 - 7, w + padding * 2, h, TFT_BLACK);

    // Lösche auch den Hintergrund
    tft.fillRect(x - w / 2 - padding, y - h / 2 - 7 - 2, w + padding * 2, h + 2, TFT_BLACK);

    tft.setCursor(x - w / 2, y - h / 2 - 7);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Setzt Textfarbe und Hintergrundfarbe
    tft.print(numStr);

    lastDrawnNumber = number; // Aktualisieren Sie die zuletzt gezeichnete Zahl
  }
}

void drawQRCode(const String &data)
{
  QRCode qrcode;

  uint8_t qrcodeData[qrcode_getBufferSize(3)];
  qrcode_initText(&qrcode, qrcodeData, 3, 0, data.c_str());

  tft.fillScreen(TFT_RED); // Display löschen

  int scale = 6;    // Größe jedes QR-Code-Moduls
  int xOffset = 25; // X-Offset auf dem Display
  int yOffset = 35; // Y-Offset auf dem Display

  for (uint8_t y = 0; y < qrcode.size; y++)
  {
    for (uint8_t x = 0; x < qrcode.size; x++)
    {
      if (qrcode_getModule(&qrcode, x, y))
      {
        tft.fillRect(xOffset + x * scale, yOffset + y * scale, scale, scale, TFT_WHITE);
      }
    }
  }
  delay(7000);
  tft.fillScreen(TFT_BLACK); // Display löschen
  closeMenu();
}

void showCounter()
{
  static int prevCounter = -1; // Statischer Speicher für den vorherigen Wert des Zählers
  int y4 = 65;                 // Y-Position des Zählers, oberhalb des Timers

  if (counter != prevCounter)
  { // Prüfen, ob sich der Wert geändert hat
    tft.setTextSize(2);
    String counterStr = String(counter);
    int strWidth = tft.textWidth(counterStr); // Breite des anzuzeigenden Textes
    int x4 = (tft.width() - strWidth) / 2;    // X-Position zentrieren

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillRect(x4, y4 - 20, strWidth, 20, TFT_BLACK); // Bereich löschen
    tft.setCursor(x4, y4 - 20);
    tft.print(counterStr);

    prevCounter = counter; // Aktualisieren des gespeicherten Wertes
  }
  // Wenn sich der Wert nicht geändert hat, erfolgt kein Neuzeichnen.
}

void drawDial(int value, int maxValue)
{
  int centerX = tft.width() / 2;
  int centerY = tft.height() / 2;
  int outerRadius = min(centerX, centerY) - 10;
  int innerRadius = outerRadius - 18;
  int startAngle = 180;
  int endAngle = 360;
  float angleStep = (float)(endAngle - startAngle) / maxValue;

  uint16_t startColor = TFT_GREEN; // Startfarbe
  uint16_t endColor = TFT_YELLOW;  // Endfarbe

  static unsigned long lastBlinkTime = 0;
  static bool isBlinkOn = false;
  const unsigned long blinkInterval = 500; // 500ms Blinkintervall

  if (value >= targetValue + 10 && millis() - lastBlinkTime > blinkInterval)
  {
    lastBlinkTime = millis();
    isBlinkOn = !isBlinkOn;
  }

  for (int i = 0; i <= maxValue; i++)
  {
    float radianAngle = (startAngle + i * angleStep) * (PI / 180.0);
    float nextRadianAngle = (startAngle + (i + 1) * angleStep) * (PI / 180.0);

    int startX = centerX + innerRadius * cos(radianAngle);
    int startY = centerY + innerRadius * sin(radianAngle);
    int endX = centerX + outerRadius * cos(radianAngle);
    int endY = centerY + outerRadius * sin(radianAngle);
    int nextStartX = centerX + innerRadius * cos(nextRadianAngle);
    int nextStartY = centerY + innerRadius * sin(nextRadianAngle);
    int nextEndX = centerX + outerRadius * cos(nextRadianAngle);
    int nextEndY = centerY + outerRadius * sin(nextRadianAngle);

    uint16_t color = lerpColor(startColor, endColor, (float)i / maxValue);
    if (i > value)
    {
      color = TFT_GREY; // Inaktiver Bereich
    }
    else if (value >= targetValue + 10)
    {
      color = isBlinkOn ? TFT_RED : TFT_BLACK; // Rot blinken
    }

    tft.fillTriangle(startX, startY, endX, endY, nextStartX, nextStartY, color);
    tft.fillTriangle(nextStartX, nextStartY, endX, endY, nextEndX, nextEndY, color);
  }

  lastValueD = value;
}

uint16_t lerpColor(uint16_t color1, uint16_t color2, float weight)
{
  // Extract the red, green, and blue components from color1
  uint8_t r1 = (color1 >> 11) & 0x1F;
  uint8_t g1 = (color1 >> 5) & 0x3F;
  uint8_t b1 = color1 & 0x1F;

  // Extract the red, green, and blue components from color2
  uint8_t r2 = (color2 >> 11) & 0x1F;
  uint8_t g2 = (color2 >> 5) & 0x3F;
  uint8_t b2 = color2 & 0x1F;

  // Perform linear interpolation for each component
  uint8_t r = r1 * (1 - weight) + r2 * weight;
  uint8_t g = g1 * (1 - weight) + g2 * weight;
  uint8_t b = b1 * (1 - weight) + b2 * weight;

  // Combine the interpolated components into a new color
  return ((r << 11) & 0xF800) | ((g << 5) & 0x7E0) | (b & 0x1F);
}

void drawBar(int value, int maxValue)
{
  static int lastFilledWidth = 0;
  int maxBarWidth = 180;                                  // Maximale Breite des Balkens
  int barWidth = min(tft.width() - 90, maxBarWidth);      // Verwenden Sie die kleinere der beiden Breiten
  int barHeight = 10;                                     // Höhe des Balkens
  int x = 60;                                             // Startposition x
  int y = tft.height() - 50;                              // Startposition y, unter der Zahl
  int filledWidth = map(value, 0, maxValue, 0, barWidth); // Berechnet die gefüllte Breite

  if (filledWidth != lastFilledWidth)
  {
    tft.fillRect(x, y, barWidth, barHeight, TFT_GREY); // Löscht den gesamten Balken

    for (int i = 0; i < filledWidth; i++)
    {
      uint16_t color;
      float progress = (float)i / barWidth;

      if (progress <= (float)targetValue / maxValue)
      {
        // Farbverlauf von Grün zu Gelb bis zum targetValue
        color = lerpColor(TFT_YELLOW, TFT_GREEN, progress / ((float)targetValue / maxValue));
      }
      else
      {
        // Farbverlauf von Gelb zu Rot über targetValue
        color = lerpColor(TFT_GREEN, TFT_RED, (progress - (float)targetValue / maxValue) / (1.0 - (float)targetValue / maxValue));
      }

      // Blinken, wenn der Wert 50 Einheiten über dem targetValue liegt
      if (value >= targetValue + 10 && millis() % 500 < 250)
      {
        color = TFT_RED;
      }

      // Zeichnet den gesamten Balken in der berechneten Farbe
      for (int j = 0; j < barHeight; j++)
      {
        tft.drawPixel(x + i, y + j, color);
      }
    }
    lastFilledWidth = filledWidth;
  }
}

float getWeight()
{
  // Berechnen des Gewichts relativ zum manuellen Tara-Wert
  float rawValue = scale.read_average(1); // Durchschnitt aus 10 Messungen
  return (rawValue - manualTareOffset) / scale.get_scale();
}

void updateQRData(float newWeight)
{
    // Runden des Gewichts auf die nächste ganze Zahl
    int roundedWeight = round(newWeight);

    String newWeightStr = String(roundedWeight); // Konvertierung des gerundeten Gewichts in einen String
    if (qrData.length() > 0)
    {
        newWeightStr = "," + newWeightStr; // Fügen Sie ein Komma als Trennzeichen hinzu, wenn bereits Daten vorhanden sind
    }

    // Prüfen, ob das Hinzufügen der neuen Daten das Limit überschreitet
    if (qrData.length() + newWeightStr.length() <= 55)
    {
        qrData += newWeightStr;
    }
    else
    {
        // Entfernen der ältesten Daten bis Platz für die neuen Daten ist
        while (qrData.length() + newWeightStr.length() > 55)
        {
            int firstDelimiter = qrData.indexOf(','); // Das erste Komma finden
            if (firstDelimiter != -1)
            {
                qrData = qrData.substring(firstDelimiter + 1); // Daten ab dem ersten Komma behalten
            }
            else
            {
                qrData = ""; // Falls kein Trennzeichen mehr vorhanden ist, alles zurücksetzen
                break;
            }
        }
        qrData += newWeightStr; // Neues Gewicht hinzufügen
    }
}


void performManualTare()
{
  // Aktuellen Messwert als manuellen Tara-Wert speichern
  manualTareOffset = scale.read_average(10); // Durchschnitt aus 10 Messungen
  Serial.println("Manuelle Tara durchgeführt. Messwert als Nullpunkt gesetzt.");
}

void performEnhancedTare()
{
  // Erfassen des aktuellen Messwerts vor der Tara
  float preTareValue = scale.get_units(10); // Durchschnitt aus 10 Messungen

  scale.tare(); // Standard-Tara-Funktion ausführen
  delay(100);   // Kurze Verzögerung für Sensorstabilisierung

  // Erfassen des Messwerts nach der Tara
  float postTareValue = scale.get_units(10); // Durchschnitt aus 10 Messungen

  // Justieren des Offsets, falls nötig
  if (abs(postTareValue) > 0.05)
  { // Annahme einer Toleranzgrenze
    long currentOffset = scale.get_offset();
    long newOffset = currentOffset + (long)(preTareValue * scale.get_scale());
    scale.set_offset(newOffset);
  }

  Serial.println("Genauere Tara durchgeführt. Messwert sollte nun bei 0 liegen.");
}

void Touch_INT_callback()
{

  if (Touch_CTS816.mode == CST816S_Gesture_Mode)
  {
    uint8_t gesture = CST816S_Get_Gesture();
    if (gesture == CST816S_Gesture_Long_Press)
    {
      flag = 1;
    }
  }
  else
  {
    CST816S_Get_Point();
    flag = 1;
  }
}

void displayMenu()
{
  isMenuDisplayed = true;
  tft.fillScreen(TFT_BLACK);
  char buttonText[] = "300N";
  char buttonText2[] = "500N";
  char buttonText3[] = "WIFI";
  char buttonText4[] = "Tare";
  int buttonWidth = 60;
  int buttonHeight = 60;
  int buttonSpacing = 20;
  int firstRowY = 60;
  int secondRowY = firstRowY + buttonHeight + buttonSpacing;

  sliderWidth = 200;                          // Breite des Sliders
  sliderHeight = 20;                          // Höhe des Sliders
  sliderX = (tft.width() - sliderWidth) / 2;  // Zentrieren des Sliders auf dem Bildschirm
  sliderY = tft.height() - sliderHeight - 10; // Y-Position des Sliders, 10 Pixel über dem unteren Rand

  // Zeichnen des Slider-Hintergrunds
  tft.fillRoundRect(sliderX, sliderY, sliderWidth, sliderHeight, 5, TFT_GREY);

  // Position des Schiebereglers
  int knobX = map(sliderCurrentValue, sliderMinValue, sliderMaxValue, sliderX, sliderX + sliderWidth - 20);
  int knobWidth = 20;

  // Schieberegler (Knob)
  tft.fillRoundRect(knobX, sliderY - 5, knobWidth, sliderHeight + 10, 5, TFT_GREEN);

  // Schatteneffekt für den Knob
  tft.drawRoundRect(knobX, sliderY - 5, knobWidth, sliderHeight + 10, 5, TFT_GREY);

  int textX = sliderX + sliderWidth + 10; // X-Position des Textes, rechts vom Slider
  int textY = sliderY + sliderHeight / 2; // Y-Position des Textes, mittig zur Höhe des Sliders

  String displayText = String(sliderCurrentValue) + " ms"; // Der anzuzeigende Text

  tft.setTextSize(1);                     // Setzen Sie die Größe des Textes
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Setzen Sie die Farbe des Textes (weiß) und den Hintergrund (schwarz)

  // Berechnen der Breite des Textes, um ihn entsprechend auszurichten
  int textWidth = tft.textWidth(displayText.c_str());
  tft.fillRect(textX, textY - tft.fontHeight() / 2, textWidth, tft.fontHeight(), TFT_BLACK); // Bereich löschen

  // Text anzeigen
  tft.setCursor(textX, textY - tft.fontHeight() / 2);
  tft.print(displayText);

  // Button 1 und 2 in der ersten Reihe
  button1.initButtonUL(20, firstRowY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText, 2);
  button1.drawButton();

  button2.initButtonUL(100, firstRowY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText2, 2);
  button2.drawButton();

  // Button 3 und 4 in der zweiten Reihe
  button3.initButtonUL(20, secondRowY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText3, 2);
  button3.drawButton();

  button4.initButtonUL(100, secondRowY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText4, 2);
  button4.drawButton();

  char brightnessUpText[] = "Hel+";
  char brightnessDownText[] = "Hel-";

  // Position und Größe der Helligkeits-Buttons anpassen
  int newButtonsX = 100 + buttonWidth + buttonSpacing; // Annahme, dass 120 die x-Position des letzten Buttons ist

  // Button für Helligkeit erhöhen neben dem ersten Satz von Buttons
  buttonIncreaseBrightness.initButtonUL(newButtonsX, firstRowY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, brightnessUpText, 2);
  buttonIncreaseBrightness.drawButton();

  // Button für Helligkeit verringern neben dem zweiten Satz von Buttons
  buttonDecreaseBrightness.initButtonUL(newButtonsX, secondRowY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, brightnessDownText, 2);
  buttonDecreaseBrightness.drawButton();

  char resetButtonText[] = "Reset";
  int resetButtonX = newButtonsX + buttonWidth + buttonSpacing;       // Position anpassen
  int resetButtonY = firstRowY; // oder secondRowY, abhängig von der gewünschten Position

  // Initialisieren und Zeichnen des Reset-Buttons

  buttonResetCounter.initButtonUL(resetButtonX, resetButtonY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, resetButtonText, 2);
  buttonResetCounter.drawButton();
}

void handleResetCounterPress()
{
  counter = 0; // Setzt den Zähler zurück
  Serial.println("Zähler zurückgesetzt");
  // Optional: Update der Anzeige, um den neuen Zählerstand zu zeigen
}

void displayTargetValue()
{
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Setzen Sie die Textfarbe und den Hintergrund
  tft.setTextSize(2);                     // Größe des Textes
  int textX = 90;                         // X-Position des Textes
  int textY = 215;                        // Y-Position des Textes, unter dem Balken

  tft.setCursor(textX, textY);
  tft.print(targetValue);
  tft.print(" N");
}

void saveTargetValue()
{
  // EEPROM.put(0, targetValue);  // Speichern des Wertes an Adresse 0
  Serial.println(targetValue);
}