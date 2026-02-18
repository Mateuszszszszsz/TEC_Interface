/*
Here is code for an arduino nano, which is master element of TEC driver for laser usage.
Project contains 1,5 RGB OLED display, Keyboard, taht will be used as user interfaces and using serial comunication (UART) with TEC driver PCB will send and recieve data form the driver.


Author: mgr. inż. Mateusz Sikorski
December 2025

*/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SPI.h>
#include <string.h>

//PIN Definitions
#define BUTTON_UP_PIN 5
#define BUTTON_OK_PIN 4
#define BUTTON_DOWN_PIN 3
#define UART_TIMEOUT_MS 1000

#define SCLK 13
#define MOSI 11
#define DC   8
#define CS   10
#define RST  9

// ---- OBIEKT OLED ----
Adafruit_SSD1351 display = Adafruit_SSD1351(128, 128, &SPI, CS, DC, RST);

// ----- Software definitions -----
#define ROWS_IN_FIRST_COLUMN 5
#define ALL_DATA_NUMBER 11 
#define NAME_AND_DATA_SPACING 20 //pixels
#define MENU_CHOICE_SPACING 5 //PIXELS

int x [ALL_DATA_NUMBER] = {5,5,5,5,5,5, 69,69,69,69,69};
int y [ALL_DATA_NUMBER] = {30,40,50,60,70,80 , 30,40,50,60,70};
String name[ALL_DATA_NUMBER] = {"ON" , "V", "Ts", "Ta", "H", "C" , "kp", "ki","kd","Ts","lim"};
int firstCycleFlag = 1;

//ENUMS
enum class Button { // BUTTON ENUMERATION
  NONE,
  UP,
  OK,
  DOWN
};


uint8_t editMode = 0 ;// var for change of parameters through keyboard. aim is when goes to to send the data
uint8_t curretDataNumber = 0 ;// var "coursor" inicating which parameter we are changing
float modifyDelta[ALL_DATA_NUMBER] = {0,0,0.1,0,1,1,   0.1,0.1,0.1,0.5,0.1};
void sendPIDparameters(int amount);

//TEC STATUS PARAMETERS TABLES [ACTUAL, PREVIOUS]
bool pidON[2] = {0,1}; //[ACTUAL, PREVIOUS]
float vdc[2] = {-1,1}; //READ ONLY [ACTUAL, PREVIOUS] 
float setTemp[2] = {15,0}; //[ACTUAL, PREVIOUS]
float temp[2] = {-1,1}; //READ ONLY[ACTUAL, PREVIOUS]
uint16_t grz[2] = {0,1}; //[ACTUAL, PREVIOUS]
uint16_t hlo[2] = {0,1}; //[ACTUAL, PREVIOUS]

//TEC PID PARAMETERS TABLES [ACTUAL, PREVIOUS]
float kp[2] = {0.01,0}; //[ACTUAL, PREVIOUS]
float ki[2] = {0.001,0}; //[ACTUAL, PREVIOUS]
float kd[2] = {0.01,0}; //[ACTUAL, PREVIOUS]
float Ts[2] = {0,1}; //[ACTUAL, PREVIOUS]
float li[2] = {0,1}; //[ACTUAL, PREVIOUS]



//functions definitions
int modifyParametrer(int dataNumber, int mode); //mode is 1 or -1 as multiplier for increasing/ decreasing
Button buttonDetection();
void buttonCallback(Button button);
bool uartReadLine(char *buf, size_t maxLen, uint32_t timeout);
bool tecGetStatus();
void updateDisplay(Button button);
void drawValue(int idx, const char *txt);

void setup() {

//PIN Init
pinMode(BUTTON_UP_PIN, INPUT_PULLUP); //Buttons init for pullup LOW = Button PRESSED
pinMode(BUTTON_OK_PIN, INPUT_PULLUP);
pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);

Serial.begin(115200);

tecGetStatus();

//DISPLAY INIT
display.begin();
display.setRotation(3);
display.fillScreen(0x0000);  // czarny
delay(20);
  display.setTextSize(2);
  display.setTextColor(0xFFFF); // biały
  display.setCursor(5, 5);
  display.println("TEC DRIVER");

 for(int i=30 ; i<90 ; i++) //LINE ANIMATION
 { 
  delay(15);
  display.drawLine(64, 25, 64, i, 0x808080);
 }
   
   for(int i  = 0 ; i < ALL_DATA_NUMBER ; i++) // print data NAMES on display 
  {
  display.setTextSize(1);
  display.setCursor(i == 0 ? x[i]+MENU_CHOICE_SPACING : x[i], y[i]);
  display.setTextColor(0xFFFF); // white
  display.print(name[i]);

  }
}

void loop() {
 Button button = buttonDetection();
 buttonCallback(button);

  static uint32_t TEClastTime = 0;
  if(millis() - TEClastTime >5000)
{
  TEClastTime = millis();
  tecGetStatus();
} 
updateDisplay(button);


 //SERIAL DEBUG INFO
if(button != Button::NONE)
  {
  Serial.print(" EM: ");
  Serial.print(editMode);
  Serial.print(" CURR DATA: ");
  Serial.print(curretDataNumber);
  Serial.print(" PID: ");
  Serial.print(pidON[0]);
  Serial.print(" VDC: ");
  Serial.print(vdc[0]);
  Serial.print(" setTemp: ");
  Serial.print(setTemp[0]);
  Serial.print(" temp: ");
  Serial.print(temp[0]);
  Serial.print(" grz: ");
  Serial.print(grz[0]);
  Serial.print(" hlo: ");
  Serial.print(hlo[0]);

  Serial.print("\n");
  }
}


void sendPIDparameters(int amount)
{
  if(amount <=3)
  {
  Serial.print("spp ");
  Serial.print(kp[0],6);  
  Serial.print(" ");      
  Serial.print(ki[0],6);
  Serial.print(" ");      
  Serial.print(kd[0],6);
  Serial.print("\n");
  }
  else if(amount == 4)
  {
  {
  Serial.print("spp ");
  Serial.print(kp[0],6);  
  Serial.print(" ");       
  Serial.print(ki[0],6);
  Serial.print(" ");      
  Serial.print(kd[0],6);
  Serial.print(" ");      
  Serial.print(Ts[0],6);
  Serial.print("\n");
  }
  }
  else if(amount ==5)
  {
  Serial.print("spp ");
  Serial.print(kp[0],6);  
  Serial.print(" ");       
  Serial.print(ki[0],6);
  Serial.print(" ");      
  Serial.print(kd[0],6);
  Serial.print(" ");      
  Serial.print(Ts[0],6);
  Serial.print(" ");      
  Serial.print(li[0],6);
  Serial.print("\n");
  }

}

int modifyParametrer(int dataNumber, int mode)
{
  switch(dataNumber)
  {
    case 0: // PID ON/OFF State
    pidON[0] = !pidON[0];
    if(pidON[0])
    Serial.print("trb 1\n");
    else
    Serial.print("trb 0\n");
    break;
    case 1:
  //read only VDC
    break;
    case 2:// SET TEMPERATURE
    setTemp[0] = setTemp[0]+=((float)mode*modifyDelta[dataNumber]);
    Serial.print("spt ");
    Serial.print(setTemp[0],2); //2 digits after separator
    Serial.print("\n");
    break;
    case 3:
  //read only
    break;
    case 4:
    grz[0] = (grz[0]+=(uint16_t)((float)mode*modifyDelta[dataNumber]+1024))%1024;
    Serial.print("grz ");
    Serial.print(grz[0]);
    Serial.print("\n");
    break;
    case 5:
    hlo[0] = (hlo[0]+=(uint16_t)((float)mode*modifyDelta[dataNumber]+1024))%1024;
    Serial.print("hlo ");
    Serial.print(hlo[0]);
    Serial.print("\n");
    break;


    case 6:
    kp[0] = kp[0]+=((float)mode*modifyDelta[dataNumber]);
    sendPIDparameters(3);
    break;
    case 7:
    ki[0] = ki[0]+=((float)mode*modifyDelta[dataNumber]);
    sendPIDparameters(3);
    break;
    case 8:
    kd[0] = kd[0]+=((float)mode*modifyDelta[dataNumber]);
    sendPIDparameters(3);
    break;
    case 9:
    Ts[0] = Ts[0]+=((float)mode*modifyDelta[dataNumber]);
    sendPIDparameters(4);
    break;
    case 10:
    li[0] = li[0]+=((float)mode*modifyDelta[dataNumber]);
    sendPIDparameters(5);
    break;
  }
  return 1;
}

Button buttonDetection()
{
  static uint32_t lastTime = 0;

  if (millis() - lastTime < 300)
    return Button::NONE;

  if (digitalRead(BUTTON_UP_PIN) == LOW) {
    lastTime = millis();
    return Button::UP;
  }

  if (digitalRead(BUTTON_OK_PIN) == LOW) {
    lastTime = millis();
    return Button::OK;
  }

  if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
    lastTime = millis();
    return Button::DOWN;
  }

  return Button::NONE;
}
void buttonCallback(Button button)
{
  if(button == Button::NONE)
  return;
  if(editMode == 0)
  {
    switch(button)// scrolling through parameters
    {
      case Button::UP:
      curretDataNumber = (curretDataNumber + ALL_DATA_NUMBER - 1) % ALL_DATA_NUMBER;
      break;
      case Button::DOWN:
      curretDataNumber++;
      curretDataNumber= curretDataNumber%ALL_DATA_NUMBER;
      break;
      case Button::OK:
      editMode = 1;
      break;
    }
  }
  else //in parameter edition mode
  {
    switch(button)// scrolling through parameters
    {
      case Button::DOWN:
      //decrease value of parameter data[curretDataNumber]
      modifyParametrer(curretDataNumber, -1); //-1 == decreasing
      break;
      case Button::UP:
      //increase value of parameter data[curretDataNumber]
      modifyParametrer(curretDataNumber, 1); //1 == incereasing
      break;
      case Button::OK:
      editMode = 0;
      break;
    }

  }
}

bool uartReadLine(char *buf, size_t maxLen, uint32_t timeout)
{
    size_t idx = 0;
  uint32_t start = millis();

  while (millis() - start < timeout) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        buf[idx] = 0;
        Serial.print("RX: ");
        Serial.println(buf);
        return true;
      }
      if (idx < maxLen - 1) buf[idx++] = c;
    }
  }
  return false;
}

bool tecGetStatus()
{ 
  char line[128];

  Serial.print("gs?\n");

  if (!uartReadLine(line, sizeof(line), UART_TIMEOUT_MS))
    return false;

  char *p;

  // T
  p = strstr(line, "T");
  if (!p) return false;
  pidON[0] = atoi(p + 1);

  // VDC
  p = strstr(line, "VDC");
  if (!p) return false;
  vdc[0] = atof(p + 3);
  // Voltage range CLAMP //uart sometimes gives sh1t 
if(vdc[0] < 0) vdc[0] = 0;
if(vdc[0] > 60) vdc[0] = 60;

  // NAST
  p = strstr(line, "NAST");
  if (!p) return false;
  if(editMode == 0)
    setTemp[0] = atof(p + 4);

  // TEMP
  p = strstr(line, "TEMP");
  if (!p) return false;
  temp[0] = atof(p + 4);

  // GRZ
  p = strstr(line, "GRZ");
  if (!p) return false;
  if(editMode == 0)
  grz[0] = (uint16_t)atoi(p + 3);

  // HLO
  p = strstr(line, "HLO");
  if (!p) return false;
  if(editMode == 0)
  hlo[0] = (uint16_t)atoi(p + 3);

  return true;
}

void drawValue(int idx, const char *txt) { //drawing a value deleting previous one
  // wyczyść tylko obszar wartości
  display.fillRect(
    x[idx] + NAME_AND_DATA_SPACING,
    y[idx],
    29,
    8,
    0x0000
  );
  display.setTextColor(0xffff);
  display.setCursor(x[idx]+NAME_AND_DATA_SPACING, y[idx]);
  display.print(txt);
}

void updateDisplay(Button button)
{
  char buf [12];
 //update changed data
 if (pidON[0] != pidON[1]) {
    sprintf(buf, "%d", pidON[0]);
    drawValue(0, buf);
    pidON[1]=pidON[0];
  }

  if (vdc[0] != vdc[1]) {
    dtostrf(vdc[0], 5, 2, buf);
    drawValue(1, buf);
    vdc[1] = vdc[0];
  }

  if (setTemp[0] != setTemp[1]) {
    dtostrf(setTemp[0], 5, 2, buf);
    drawValue(2, buf);
    setTemp[1] = setTemp[0];
  }

  if (temp[0] != temp[1]) {
    dtostrf(temp[0], 5, 2, buf);
    drawValue(3, buf);
    temp[1] = temp[0];
  }

  if (grz[0] != grz[1]) {
    sprintf(buf, "%d", grz[0]);
    drawValue(4, buf);
    grz[1] = grz[0];
  }

  if (hlo[0] != hlo[1]) {
    sprintf(buf, "%d", hlo[0]);
    drawValue(5, buf);
    hlo[1] = hlo[0];
  }

  // ---- PID ----

  if (kp[0] != kp[1]) {
    dtostrf(kp[0], 6, 3, buf);
    drawValue(6, buf);
    kp[1] = kp[0];
  }

  if (ki[0] != ki[1]) {
    dtostrf(ki[0], 6, 3, buf);
    drawValue(7, buf);
    ki[1] = ki[0];
  }

  if (kd[0] != kd[1]) {
    dtostrf(kd[0], 6, 3, buf);
    drawValue(8, buf);
    kd[1] = kd[0];
  }

  if (Ts[0] != Ts[1]) {
    dtostrf(Ts[0], 5, 2, buf);
    drawValue(9, buf);
    Ts[1] = Ts[0];
  }

  if (li[0] != li[1]) {
    dtostrf(li[0], 6, 3, buf);
    drawValue(10, buf);
    li[1] = li[0];
  }

  if( button == Button::DOWN && editMode == 0)
  {
    display.fillRect(x[(curretDataNumber+ALL_DATA_NUMBER-1)%ALL_DATA_NUMBER], y[(curretDataNumber+ALL_DATA_NUMBER-1)%ALL_DATA_NUMBER], NAME_AND_DATA_SPACING, 10  , 0x0000);
    display.fillRect(x[curretDataNumber], y[curretDataNumber], NAME_AND_DATA_SPACING, 10  , 0x0000);
    display.setCursor(x[(curretDataNumber+ALL_DATA_NUMBER-1)%ALL_DATA_NUMBER], y[(curretDataNumber+ALL_DATA_NUMBER-1)%ALL_DATA_NUMBER]);
    display.println(name[(curretDataNumber+ALL_DATA_NUMBER-1)%ALL_DATA_NUMBER]);
    
    display.setCursor(x[curretDataNumber]+MENU_CHOICE_SPACING, y[curretDataNumber]);
    display.println(name[curretDataNumber]);
  }
  else if (button == Button::UP && editMode == 0)
  {
    display.fillRect(x[(curretDataNumber+ALL_DATA_NUMBER+1)%ALL_DATA_NUMBER], y[(curretDataNumber+ALL_DATA_NUMBER+1)%ALL_DATA_NUMBER], NAME_AND_DATA_SPACING, 10  , 0x0000);
    display.fillRect(x[curretDataNumber], y[curretDataNumber], NAME_AND_DATA_SPACING, 10  , 0x0000);
    display.setCursor(x[(curretDataNumber+ALL_DATA_NUMBER+1)%ALL_DATA_NUMBER], y[(curretDataNumber+ALL_DATA_NUMBER+1)%ALL_DATA_NUMBER]);
    display.println(name[(curretDataNumber+ALL_DATA_NUMBER+1)%ALL_DATA_NUMBER]);
    
    display.setCursor(x[curretDataNumber]+MENU_CHOICE_SPACING, y[curretDataNumber]);
    display.println(name[curretDataNumber]);
  }
  else if (button == Button::OK)
  {
    display.fillRect(x[curretDataNumber], y[curretDataNumber], NAME_AND_DATA_SPACING, 10  , 0x0000);
    if(editMode == 1)
    display.setTextColor(0xF000);
    if(editMode == 0)
    display.setTextColor(0xFFFF);

    display.setCursor(x[curretDataNumber]+MENU_CHOICE_SPACING, y[curretDataNumber]);
    display.println(name[curretDataNumber]);
  }

}
