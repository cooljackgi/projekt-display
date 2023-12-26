/*
 * Hauptskript für das Anzeigen von Daten und QR-Codes auf einem TFT-Display
 * unter Verwendung verschiedener Sensoren und Widgets.
 */
//  Evtl Handzähler hinzufügen
#include <EEPROM.h>
#include "FS.h"
#include "LittleFS.h"
// #include "hardware/adc.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
// #include <Adafruit_GC9A01A.h>
#include <qrcode.h>
#include <TFT_eSPI.h> // TFT_eSPI-Bibliothek einbinden
#include <TFT_eWidget.h>
#include "C:\Users\becke\Documents\PlatformIO\Projects\projekt display\lib\logo.c"
#include "HX711.h"
#include "C:\Users\becke\Documents\PlatformIO\Projects\projekt display\lib\CST816S\CST816S.h"
#include <XPT2046_Touchscreen.h>
#include <EEPROM.h>
#include <random>

bool isSimButtonActive = false;

uint16_t calData[5];
uint8_t calDataOK = 0;
uint8_t flag = 0;
// Touchscreen-Konfiguration
bool isMenuDisplayed = false;
const int adcPin = -1;              // ADC-Pin
const float referenceVoltage = 3.3; // Referenzspannung des ADC
const int adcResolution = 1023;     // ADC-Auflösung (10 Bit)
// int xMin, xMax, yMin, yMax;
#define BUTTON_PIN -1                 // Definieren Sie den Pin, an dem der Knopf angeschlossen ist
unsigned long displayDuration = 1000; // Anzeigedauer in Millisekunden (z.B. 5000ms = 5 Sekunden)

HX711 scale;
long manualTareOffset = 0; // Globale Variable für manuelle Tara
uint8_t dataPin = 25;
uint8_t clockPin = 26;
// Globale Variablen
TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen ts(TOUCH_CS); // CS_PIN mit dem korrekten Pin ersetzen
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
bool isSimulationActive = false;
unsigned long startTime = 0;
bool timerRunning = false;
bool reached300 = false;
bool weightOver50 = false;
unsigned long currentTime;
int screenWidth = tft.width(); // Die Breite des Bildschirms
int graphWidth = 140;
int graphHeight = 50;
int graphX = (screenWidth - graphWidth) / 2 - 5;
int graphY = 130;
int graphXValue = 0; // Global deklarieren
unsigned long lastTapTime = 0;
int tapCount = 0;
const unsigned long doubleTapInterval = 550; // Zeitfenster für Doppeltap in Millisekunden
int counter = 0;
int lastValue = -1;
int lastValueD = -1;
int16_t adc0;
// Adafruit_ADS1115 ads;
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

int targetValue; // Startwert

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

// Funktionsprototypen
void setup();
void loop();
void drawCenterNumber(int number);
void drawQRCode(const String &data);
void drawEndCaps();
// void showTimer();
void showCounter();
void drawBar(int value, int maxValue);
void performManualTare();
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
void setup()
{

  Serial.begin(9600);
  // DEV_Module_Init();
  // CST816S_init(CST816S_Point_Mode);
  // DEV_KEY_Config(Touch_INT_PIN);
  // attachInterrupt(Touch_INT_PIN, &Touch_INT_callback, RISING);
  // pinMode(BUTTON_PIN, INPUT_PULLUP);  // Setzen Sie den Knopf-Pin als Eingang mit aktiviertem Pull-up-Widerstand
  // pinMode(adcPin, INPUT);
  // pinMode(TFT_BL, OUTPUT);
  // digitalWrite(TFT_BL, HIGH);
  tft.init(); // Display initialisieren
  tft.setRotation(1);
  ts.begin();
  ts.setRotation(1);

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
  delay(500);
  tft.fillScreen(TFT_BLACK);
  // drawPartialCircles();
  startTime = millis();
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

  scale.begin(dataPin, clockPin);
  scale.set_gain(128); // oder ein anderer passender Wert

  // Setzen Sie hier die kalibrierten Werte ein
  scale.set_offset(21576);
  scale.set_scale(-444);
  delay(500);
  scale.tare(10);
  performManualTare();

  // Variable weightOver50 initialisieren
  weightOver50 = false;
  if (flag == 1)
  {
    flag = 0;
    // break;
  }
  Serial.println("Setup Fertig");
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
}

void loop()
{

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
      // Hier die normale Gewichtsberechnung oder -abfrage einfügen
      processWeight();
    }

    checkMenuButtonTouch();
    processWeight();                    // Verarbeitung des Gewichts
    updateGraphAndTrace(currentMillis); // Aktualisierung des Graphen und Traces
    updateCounter();                    // Aktualisierung des Zählers
  }

  // Überprüfung der Touch-Eingaben
  // checkTouch();
  currentButtonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentButtonState == LOW)
  {
    counter++; // Erhöhen Sie den Zähler um 1
    showCounter();
  }

  lastButtonState = currentButtonState; // Aktualisieren Sie den letzten Zustand des Knopfes
}

void setDisplayDuration(unsigned long duration)
{
  displayDuration = duration;
}

void toggleWeightSimulation()
{
  isSimulationActive = !isSimulationActive;
  isSimButtonActive = !isSimButtonActive; // Umschalten des Button-Zustands
  drawMenuButton();                       // Button neu zeichnen
  if (isSimulationActive)
  {
    Serial.println("Simulation aktiviert");
  }
  else
  {
    Serial.println("Simulation deaktiviert");
  }
}

void handleButton1Press()
{
  Serial.println("Button 1 gedrückt");
  targetValue = 300;
  saveTargetValue();
  closeMenu();
  // Fügen Sie hier die Logik für Button 1 hinzu
}

void handleButton2Press()
{
  Serial.println("Button 2 gedrückt");
  targetValue = 500;
  saveTargetValue();
  closeMenu();
  // Fügen Sie hier die Logik für Button 2 hinzu
}

void handleButton3Press()
{
  Serial.println("Button 3 gedrückt");
  drawQRCode(qrData);
  closeMenu();

  // Fügen Sie hier die Logik für Button 3 hinzu
}

void handleButton4Press()
{
  Serial.println("Button 4 gedrückt");
  performManualTare();
  closeMenu();
  // Fügen Sie hier die Logik für Button 4 hinzu
}

void handleMenuButtonPress()
{
  // Code für die Aktion, die ausgeführt werden soll, wenn der Menü-Button gedrückt wird
  Serial.println("Menü-Button gedrückt");
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
    Serial.print("Touch detected at x = ");
    Serial.print(t_x);
    Serial.print(", y = ");
    Serial.println(t_y);
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

    if (REPEAT_CAL)
    {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);
    Serial.println("Calibration Data:");
    for (int i = 0; i < 5; i++)
    {
      Serial.print("calData[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(calData[i]);
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
  Serial.println("speichern fertig");
}

void updateCounter()
{
  if (weight > 50 && !weightOver50)
  {
    weightOver50 = true;
    counter++;
    showCounter();
  }
  else if (weight < 5 && weightOver50)
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
  delay(400);
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
  static float startWeightThreshold = 10; // Schwellenwert für den Beginn der Prüfung
  static float endWeightThreshold = 10;   // Schwellenwert für das Ende der Prüfung

  // float weight = getWeight();
  float weight = simulateWeightChange();
  // Serial.println(weight);
  //  Erkennen des Beginns einer Prüfung
  if (!isTesting && weight > startWeightThreshold)
  {
    isTesting = true;
    qrData = ""; // Zurücksetzen der QR-Daten für die neue Prüfung
  }

  // Verarbeiten des Gewichts während einer Prüfung
  if (isTesting)
  {
    updateQRData(weight);

    // Erkennen des Endes einer Prüfung
    if (weight < endWeightThreshold)
    {
      isTesting = false;
      // drawQRCode(qrData);  // Zeichnen des QR-Codes am Ende der Prüfung
    }
  }

  // Weitere Logik zur Verarbeitung des Gewichts
  if (button3Mode)
  {
    if (millis() - button3PressTime > 3000)
    {
      button3Mode = false;
    }
    else
    {
      drawCenterNumber((int)saveWeight);
      return;
    }
  }

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
}

/*void showTimer() {
  currentTime = millis();
  elapsedTime = (currentTime - startTime) / 1000;  // Verstrichene Zeit in Sekunden
  int minutes = elapsedTime / 60;
  int seconds = elapsedTime % 60;

  // Timer anzeigen
  tft.setTextSize(2);

  String timeStr = String(minutes) + ":" + ((seconds < 10) ? "0" : "") + String(seconds);  // Erstellen des Zeitstrings
  int timeStrWidth = tft.textWidth(timeStr);                                               // Berechnung der Textbreite

  int centerX = tft.width() / 2 - timeStrWidth / 2;  // Berechnung der zentralen x-Position
  int centerY = 70;                                  // Y-Position anpassen

  tft.setCursor(centerX, centerY);
  tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
  tft.print(timeStr);
}*/

void showCounter()
{
  int y4 = 65; // Y-Position des Zählers, oberhalb des Timers

  tft.setTextSize(2);
  String counterStr = String(counter);
  int strWidth = tft.textWidth(counterStr); // Breite des anzuzeigenden Textes
  int x4 = (tft.width() - strWidth) / 2;    // X-Position zentrieren

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillRect(x4, y4 - 20, strWidth, 20, TFT_BLACK); // Bereich löschen
  tft.setCursor(x4, y4 - 20);
  tft.print(counterStr);
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
  // Lineare Interpolation zwischen zwei Farben
  uint8_t r1 = (color1 >> 11) & 0x1F;
  uint8_t g1 = (color1 >> 5) & 0x3F;
  uint8_t b1 = color1 & 0x1F;

  uint8_t r2 = (color2 >> 11) & 0x1F;
  uint8_t g2 = (color2 >> 5) & 0x3F;
  uint8_t b2 = color2 & 0x1F;

  uint8_t r = (r1 * (1 - weight)) + (r2 * weight);
  uint8_t g = (g1 * (1 - weight)) + (g2 * weight);
  uint8_t b = (b1 * (1 - weight)) + (b2 * weight);

  return tft.color565(r << 3, g << 2, b << 3);
}

void drawBar(int value, int maxValue)
{
  static int lastFilledWidth = 0;
  int maxBarWidth = 200;                                  // Maximale Breite des Balkens
  int barWidth = min(tft.width() - 90, maxBarWidth);      // Verwenden Sie die kleinere der beiden Breiten
  int barHeight = 10;                                     // Höhe des Balkens
  int x = 40;                                             // Startposition x
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
  float rawValue = scale.read_average(2); // Durchschnitt aus 10 Messungen
  return (rawValue - manualTareOffset) / scale.get_scale();
}

void updateQRData(float newWeight)
{
  // Runden des Gewichts auf die nächste ganze Zahl
  int roundedWeight = round(newWeight);

  String newWeightStr = String(roundedWeight); // Konvertierung des gerundeten Gewichts in einen String
  if (qrData.length() > 0)
  {
    newWeightStr = "," + newWeightStr; // Fügen Sie ein Trennzeichen hinzu, wenn bereits Daten vorhanden sind
  }

  // Prüfen, ob das Hinzufügen der neuen Daten das Limit überschreitet
  if (qrData.length() + newWeightStr.length() <= 55)
  {
    qrData += newWeightStr;
  }
  else
  {
    // Entfernen der ältesten Daten, um Platz für die neuen Daten zu schaffen
    while (qrData.length() + newWeightStr.length() > 55)
    {
      int firstDelimiter = qrData.indexOf('-');
      if (firstDelimiter != -1)
      {
        qrData = qrData.substring(firstDelimiter + 1);
      }
      else
      {
        qrData = ""; // Wenn kein Trennzeichen vorhanden ist, leeren Sie den gesamten String
        break;       // Beenden Sie die Schleife, wenn es keine Daten mehr gibt
      }
    }
    qrData += newWeightStr; // Fügen Sie das neue Gewicht hinzu
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

/*void displayMenu() {
    isMenuDisplayed = true;
    tft.fillScreen(TFT_BLACK);

    int centerX = 120;
    int buttonY1 = 60, buttonY2 = 120, buttonY3 = 180;
    char buttonText[] = "300N";
    button1.initButtonUL(centerX - 30, buttonY1 - 30, 60, 60, TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText, 2);
    button1.drawButton();
    char buttonText2[] = "500N";
    button2.initButtonUL(centerX - 30, buttonY2 - 30, 60, 60,TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText2, 2);
    button2.drawButton();
    char buttonText3[] = "<<--";
    button3.initButtonUL(centerX - 30, buttonY3 - 30, 60, 60,TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText3, 2);
    button3.drawButton();
}*/

void displayMenu()
{
  isMenuDisplayed = true;
  tft.fillScreen(TFT_BLACK);
  char buttonText[] = "300N";
  char buttonText2[] = "500N";
  char buttonText3[] = "QR";
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
  button1.initButtonUL(40, firstRowY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText, 2);
  button1.drawButton();

  button2.initButtonUL(120, firstRowY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText2, 2);
  button2.drawButton();

  // Button 3 und 4 in der zweiten Reihe
  button3.initButtonUL(40, secondRowY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText3, 2);
  button3.drawButton();

  button4.initButtonUL(120, secondRowY, buttonWidth, buttonHeight, TFT_BLACK, TFT_WHITE, TFT_BLACK, buttonText4, 2);
  button4.drawButton();
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