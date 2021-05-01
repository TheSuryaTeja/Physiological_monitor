#include <TaskScheduler.h>
#include <Wire.h>

//RTC init
#include <DS3231.h>
DS3231 clk;
RTCDateTime dt; // date-time type variable


//OLED init
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET    -1  // Reset pin # (or -1 if sharing reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int x, y = 0;


// Temp init
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//ECG init
const int ecg_switch = 14; 
const int ANALOG_INPUT_PIN = 34;
const int MIN_ANALOG_INPUT = 1400;
const int MAX_ANALOG_INPUT = 2200;
int _circularBuffer[SCREEN_WIDTH]; //fast way to store values (rather than an ArrayList with remove calls)
int _curWriteIndex = 0; // tracks where we are in the circular buffer
int depth = 20;
int _graphHeight = 40;
int analogVal = 0;
int yPos;
volatile bool flag = 0;

//heart rate init
#include "MAX30105.h"
#include "spo2_algorithm.h"
MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
byte pp = 0;

// Callback methods prototypes
void t1Callback();
void t2Callback();
void t3Callback();
void t4Callback();

//Tasks
Task t1(800, TASK_FOREVER, &t1Callback); //rtc
Task t2(50, TASK_FOREVER, &t2Callback);   //temp
Task t3(100, TASK_FOREVER, &t3Callback);   //ECG
Task t4(100, TASK_FOREVER, &t4Callback);   //SPo2

Scheduler runner;


void t1Callback() {
  
for (y=0; y<=18; y++){
       for (x=0; x<1000; x++){
        display.drawPixel(x, y, BLACK); 
       }
      }
    dt = clk.getDateTime();

    display.setTextSize(2);
    display.setCursor(15,0);
    display.println(dt.hour, DEC);
    //display.display(); 
    
    display.setTextSize(2);
    display.setCursor(40,0);
    display.println(":");
    //display.display(); 

    display.setTextSize(2);
    display.setCursor(55,0);
    display.println(dt.minute, DEC);
    //display.display(); 

    display.setTextSize(2);
    display.setCursor(80,0);
    display.println(":");
    //display.display(); 
    
    display.setTextSize(2);
    display.setCursor(90,0);
    display.println(dt.second, DEC);
  
  display.display(); 
}

void t2Callback() {
    for (y=20; y<=37; y++){
       for (x=15; x<512; x++){
        display.drawPixel(x, y, BLACK); 
       }
      }
  display.setTextSize(2);
  display.setCursor(15,20);
  display.println("Temp : ");
    
  display.setTextSize(2);
  display.setCursor(90,20);
  display.print(String(int(mlx.readObjectTempC())));

    display.display();
  }
  
void t3Callback() {
      
      int analogVal = analogRead(ANALOG_INPUT_PIN);
      Serial.println(analogVal);
      _circularBuffer[_curWriteIndex++] = analogVal;
      if(_curWriteIndex >= display.width()){
        _curWriteIndex = 0;
      }
      int xPos = 0; 
      for (int i = _curWriteIndex; i < display.width(); i++){
        int analogVal = _circularBuffer[i];
        analogVal = _circularBuffer[i];
        yPos = map(analogVal, MIN_ANALOG_INPUT, MAX_ANALOG_INPUT, depth, _graphHeight+depth);    
        display.drawPixel(xPos, yPos, WHITE);
        xPos++;
      }
      for(int i = 0; i < _curWriteIndex; i++){
        analogVal = _circularBuffer[i];
        yPos = map(analogVal, MIN_ANALOG_INPUT, MAX_ANALOG_INPUT, depth, _graphHeight+depth);
        display.drawPixel(xPos, yPos, WHITE);
        xPos++;;
      }
      
      display.display();
       for (y=20; y<=60; y++){
           for (x=0; x<1024; x++){
            display.drawPixel(x, y, BLACK); 
           }
          }
        } 

void t4Callback() {
  
    bufferLength = 50; //buffer length of 100 stores 4 seconds of samples running at 25sps
        
          //read the first 100 samples, and determine the signal range
          for (byte i = 0 ; i < bufferLength ; i++)
          {
            while (particleSensor.available() == false) //do we have new data?
              particleSensor.check(); //Check the sensor for new data
        
            redBuffer[i] = particleSensor.getRed();
            irBuffer[i] = particleSensor.getIR();
            particleSensor.nextSample(); //We're finished with this sample so move to next sample
        

          }
        
          //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
          maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
        
          //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
          for (pp = 0; pp<2; pp++) {
            //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
            for (byte i = 25; i < 100; i++)
            {
              redBuffer[i - 25] = redBuffer[i];
              irBuffer[i - 25] = irBuffer[i];
            }
        
            //take 25 sets of samples before calculating the heart rate.
            for (byte i = 75; i < 100; i++)
            {
              for (y=36; y<=63; y++){
               for (x=0; x<100; x++){
                display.drawPixel(x, y, BLACK); 
               }
              }
              while (particleSensor.available() == false) //do we have new data?
                particleSensor.check(); //Check the sensor for new data
        
              digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
        
              redBuffer[i] = particleSensor.getRed();
              irBuffer[i] = particleSensor.getIR();
              particleSensor.nextSample(); //We're finished with this sample so move to next sample
        
              display.setTextSize(2);
              display.setCursor(0,48);
              display.println("H:");
              display.setTextSize(2);
              display.setCursor(25,48);
              if (heartRate/2 < 150 && heartRate/2 > 0)
              display.println(heartRate/2, DEC);
              else
              display.println("-");
              display.display();
        
              display.setTextSize(2);
              display.setCursor(65,48);
              display.println("S:");
              display.setTextSize(2);
              display.setCursor(85,48);
              if (spo2 < 101 && spo2 > 0)
              display.println(spo2, DEC);
              else
              display.println("-");
              display.display();
            
            }
            
            //After gathering 25 new samples recalculate HR and SP02
            maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
          }
}

void IRAM_ATTR isr() {
  if (!flag)//set flag to low if flag is true
    flag = 1;//set flag
}

void setup () {

  pinMode(ecg_switch, INPUT_PULLUP);
  attachInterrupt(ecg_switch, isr, FALLING);
  
  //oled check 
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  { 
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
  }

  //RTC check
    if (! clk.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  
clk.setDateTime(__DATE__, __TIME__);

// Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.display();
  
  t1Callback();
  t2Callback();
  
  // start
  runner.init();
  runner.addTask(t1);
  runner.addTask(t2);
  runner.addTask(t3);
  runner.addTask(t4);
}


void loop () {
  runner.execute();
    if (!digitalRead(ecg_switch)) {
    flag = !flag;
    display.clearDisplay();
  }
  
    if (flag) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(8,2);
    display.println("ECG:");
    t3.enable();
    t2.disable();
    t1.disable();
    t4.disable();
  }
  else {  
           
            t3.disable();
            t2.enable();
            t1.enable();
            t4.enable();
}
}
