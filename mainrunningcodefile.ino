/*THIS IS THE GIC MICROGRID COMPETITION PROJECT CREATED BY TEAM MICROLINK PRESENTING THE FLOWGRID PRODUCT TO AID IN ACHIEVING SDG 7*/
//coded by Marwan Khaled

#include <LiquidCrystal.h>
#include <Adafruit_INA219.h>
#include <Wire.h>
#include <avr/wdt.h>



/*===== 1. VARIABLES AND CONSTANTS =====*/
#define mosfet1 4
#define mosfet2 5
#define mosfet3 6
#define mosfet4 7
#define mosfet5 8
#define isolationRelay 43

#define lcdRS 22
#define lcdE 25
#define lcdD4 26
#define lcdD5 29
#define lcdD6 30
#define lcdD7 33

#define buttonNormal 37
#define buttonConserving 36
#define buttonDisaster 38
#define buttonNextPage 39
#define buttonPrevPage 41

#define ledNormal 48
#define ledConserving 47
#define ledDisaster 51

#define buzzer 44

#define INA219_ADDR1 0x45
#define INA219_ADDR2 0x44

#define disasterTimeThreshold 20000
#define maxDetectedVoltage 24
int pageNumber;
int totalPageNumber = 1;
int danger = 0;

unsigned long disasterEnterTime = 0;
unsigned long lastSwitchTime = 0;
#define recoveryStableTime 15000
#define relayMinSwitchTime 10000



/*===== 2. GLOBAL STATES =====*/

LiquidCrystal lcd1(lcdRS,lcdE,lcdD4,lcdD5,lcdD6,lcdD7);

Adafruit_INA219 ina219_1(INA219_ADDR1);
Adafruit_INA219 ina219_2(INA219_ADDR2);

enum SYSTEMSTATE {
  NORMAL, CONSERVING, DISASTER,
};

static SYSTEMSTATE systemState;

enum LOADPRIORITY {
  CRITICAL, IMPORTANT, OPTIONAL,
};

enum FAULTTYPE {
  NO_FAULT = 0, SENSOR_FAULT, UNDERVOLTAGE, OVERCURRENT, OVERVOLTAGE, USER_INPUT,
};

FAULTTYPE Fault;

enum SENSORTYPE {
  BUSSENSOR, LOADSENSOR,
};
SENSORTYPE sensorType;


struct LOADGROUP {
  char GroupName[20];
  LOADPRIORITY LoadPriority;
  bool AcceptThrottling;
  bool DCload;
  uint8_t mosfetPin;
  bool isInductive;
  int PWM;   
};

struct SENSORDATA {
  char SensorName[20];
  Adafruit_INA219*sensors;
  float voltage;
  float current;
  float power;
  bool valid;
  float maxVoltage;
  float minVoltage;
  float maxCurrent;
  bool present;
  FAULTTYPE Fault;
  SENSORTYPE sensorType;

};


/*===== 3. Load group control =====*/

LOADGROUP loadgroups[] = {
  {"Group1-LIGHTING", IMPORTANT, true, true, mosfet1, false, 100},
  {"Group2-HVAC", IMPORTANT, false, true, mosfet2, true, 100},
  {"Group3-EMERLIGHTS", CRITICAL, false, true, mosfet3, false, 100},
  {"Group4-LUXURY", OPTIONAL, false, true, mosfet4, true, 100}, //| Group name | Load Priority | Accept throttling | DC load | Mosfet Pin | isInductive | PWM |
};

const int loadAmount = sizeof(loadgroups)/sizeof(loadgroups[0]);

SENSORDATA sensorData[] = {
  {"GRID",&ina219_1, 0, 0, 0, false, 14.0, 11.5, 3.0, false, NO_FAULT, BUSSENSOR},
  {"UPS",&ina219_2, 0, 0, 0, false, 14.0, 10.9, 3.0, false, NO_FAULT, BUSSENSOR}, //| sensor | voltage | current | power | valid | max voltage | min voltage | max current | present | Fault type | Sensor Type
};

const int NUMOFSENSORS = sizeof(sensorData)/sizeof(sensorData[0]);



/*===== 4. MEASUREMENT AND VALIDATION =====*/

void SensorSetup() {
  for(int i = 0; i < NUMOFSENSORS; i++){
    if(!sensorData[i].sensors->begin()){
      Serial.print("SENSOR "); Serial.print(i+1); Serial.println(" NOT FOUND");
      sensorData[i].valid = false;
      sensorData[i].present = false;
    }
    else{
      sensorData[i].valid = true;
      sensorData[i].present = true;
      Serial.print("Sensor "); Serial.print(i+1); Serial.println(" initialised");
    }
  } 
  Serial.println("Beginning sensory operation");
  
}

void retryMissingSensors() {
  static unsigned long lastRetry = 0;
  if(millis()-lastRetry < 1000) return;
  lastRetry = millis();
  
  for(int i = 0; i < NUMOFSENSORS; i++){
    if(sensorData[i].present==false){
      if(sensorData[i].sensors->begin()){
        Serial.print("Sensor "); Serial.print(i+1); Serial.println(" initialised after not being found");
        sensorData[i].present=true;
      }
    }
  }
}

void measureAndValidate() {
  
  static unsigned long lastTime = 0;
  static unsigned long lastReading = 0;

  if(millis()-lastTime < 200) return;
  lastTime = millis();
  lastReading = millis();
  

  for(int i = 0; i < NUMOFSENSORS; i++){
    
   if(!sensorData[i].present){
     sensorData[i].valid = false;
     continue;
    }

    float v = sensorData[i].sensors->getBusVoltage_V();
    float c = sensorData[i].sensors->getCurrent_mA();
    
    if(v < -0.1 || v > maxDetectedVoltage || c < -0.1 || c > sensorData[i].maxCurrent*1000){ //*1000 to convert from A to mA
      if(millis() - lastReading > 2000){
        sensorData[i].voltage = sensorData[i].current = sensorData[i].power = 0;
        sensorData[i].valid = false;
        continue;
      }      
    }
    else{
      sensorData[i].valid = true;
      sensorData[i].voltage = v;
      sensorData[i].current = c;
      sensorData[i].power = v * (c / 1000.0); //current divided by 1000 in order to convert to W
    }
  }
}


void printSensorData(int score) {
  
  static unsigned long lastTime = 0;
  if(millis()-lastTime <  2000) return;
  lastTime = millis();

  Serial.print("DANGER SCORE IS "); Serial.println(score);
    
  for(int i = 0; i < NUMOFSENSORS; i++){
    if(sensorData[i].valid == true){
      Serial.println("---------");   
      Serial.print("V"); Serial.print(i+1); Serial.print(": "); Serial.print(sensorData[i].voltage); Serial.println(" V");
      Serial.print("A"); Serial.print(i+1); Serial.print(": "); Serial.print(sensorData[i].current); Serial.println(" mA");
      Serial.print("P"); Serial.print(i+1); Serial.print(": "); Serial.print(sensorData[i].power); Serial.println(" W");
      Serial.println("---------");      
    }
    else{
      Serial.println("---------------------------------------");    
      Serial.print("WARNING SENSOR "); Serial.print(i+1); Serial.println(" READING IS UNRELIABLE!");
      Serial.println("---------------------------------------");      
    }
  }
  Serial.println("");
}



/*===== 5.CALCULATE DANGER SCORE =====*/

int computeDangerScore(){
  
  static SYSTEMSTATE lastState = NORMAL;
  static int score = 0;
  FAULTTYPE worstfault = NO_FAULT;
  bool GRIDBAD = false;

  if(digitalRead(buttonDisaster) == HIGH){
    lastState = DISASTER;
    worstfault = USER_INPUT;
    Fault = USER_INPUT;    
  }  //When disaster button is pressed (override)

  if(digitalRead(buttonConserving) == HIGH){
    lastState = CONSERVING;
    worstfault = USER_INPUT;
    Fault = USER_INPUT;
  }

  if(digitalRead(buttonNormal) == HIGH){
    lastState = NORMAL;
    worstfault = NO_FAULT;
    Fault = NO_FAULT;
  }
  
  if(lastState == DISASTER && Fault == USER_INPUT){
    switch(lastState){
      case DISASTER: systemState = lastState; return 100; 
      case CONSERVING: systemState = lastState; return 50;
      case NORMAL: systemState = lastState; return 0; break;
    }
  }
  
  for(int i=0; i < NUMOFSENSORS; i++){
    if(sensorData[i].sensorType == BUSSENSOR){
    if(!sensorData[i].present){
      worstfault = max(worstfault, SENSOR_FAULT);
      continue;
    }
    if(!sensorData[i].valid){
      worstfault = max(worstfault, SENSOR_FAULT);
      continue;
    } //For sensor faults
    }
    
    float v = sensorData[i].voltage;
    float c = sensorData[i].current;
    
    if(sensorData[i].sensorType == BUSSENSOR){
      if(v > sensorData[i].maxVoltage){
       worstfault = max(worstfault, OVERVOLTAGE);
      }
      if(v < sensorData[i].minVoltage){
        if(sensorData[i].sensors == &ina219_1){
          GRIDBAD = true;
        }        
        if(GRIDBAD == true){
          worstfault = max(worstfault, UNDERVOLTAGE);
        }
      }
    }

    float currentRatio = c / (sensorData[i].maxCurrent*1000); //max current is divided by 1000 in order to convert from A to mA
    if(currentRatio > 0.95){
      worstfault = max(worstfault, OVERCURRENT);
    }
    sensorData[i].Fault = worstfault;
  } 

  Fault = worstfault;
  
  static unsigned long faultStartTime = 0;
  static FAULTTYPE lastFault = NO_FAULT;

  if(worstfault != NO_FAULT){    
    if(worstfault != lastFault){
      if(millis()-faultStartTime > 2000){
        faultStartTime = millis();
        lastFault = worstfault;    
        switch (worstfault) {
          case UNDERVOLTAGE:
            score +=10;
            break;
          case SENSOR_FAULT:
            score += 2;
            break;
          case OVERCURRENT:
            score += 50;
            break;
          case OVERVOLTAGE:
            score += 50;
            break;
          default:
            break;
        }
      }

    }
    

    if(millis()-faultStartTime > disasterTimeThreshold){
      faultStartTime = millis();
      switch (worstfault) {
        case UNDERVOLTAGE: 
        score += 5;
        break;
       
        case SENSOR_FAULT: 
        score += 1;
        break;
        
        case OVERCURRENT: 
        score += 50; 
        break;
       
        case OVERVOLTAGE: 
        score += 50; 
        break;
        
        default:  
        break;         
      } 
    }
  }
  else{
    lastFault = NO_FAULT;
    if(score > 0 && millis()-faultStartTime > 1000){
      faultStartTime = millis();
      score -= 1;      
    }
  }  

  score = constrain(score, 0, 100);   
  return score;
}


/*===== 6. HARDWARE CONTROL =====*/

void hardwareControl(int score){
  static bool justBooted = true;
  bool relayCanSwitch = (millis() - lastSwitchTime > relayMinSwitchTime);

  if(millis() < 3000 && justBooted == true){
    digitalWrite(isolationRelay, LOW);
    return;
  }
  else{
    justBooted = false;    
  }

  static unsigned long throttleStartTime[loadAmount] = {0};
  static unsigned long shedStartTime[loadAmount] = {0};
  static unsigned long buzzerTime = 0;

  if(Fault != USER_INPUT){
    if(score >= 70){
      systemState = DISASTER;
    }
    else if(score >= 30){
      systemState = CONSERVING;
    }
    else{
      systemState = NORMAL;
    }
  }

  
  if(systemState == DISASTER){    
    digitalWrite(ledDisaster, HIGH);
    digitalWrite(ledNormal, LOW);
    digitalWrite(ledConserving, LOW);
  }
  else if(systemState == CONSERVING){
    digitalWrite(ledDisaster, LOW);
    digitalWrite(ledNormal, LOW);
    digitalWrite(ledConserving, HIGH);
  }
  else{
    digitalWrite(ledDisaster, LOW);
    digitalWrite(ledNormal, HIGH);
    digitalWrite(ledConserving, LOW);
  }

  if(systemState == DISASTER && relayCanSwitch){
    digitalWrite(isolationRelay, LOW);
    lastSwitchTime = millis();
  }
  else if(systemState != DISASTER && relayCanSwitch){
    digitalWrite(isolationRelay, HIGH);
    lastSwitchTime = millis();
  }



  if(Fault == OVERVOLTAGE || Fault == UNDERVOLTAGE){
    if(Fault == UNDERVOLTAGE){
      digitalWrite(isolationRelay, LOW);
    }
    if(systemState == DISASTER){
      for(int i = 0; i < loadAmount; i++){
        if(loadgroups[i].LoadPriority == CRITICAL){
          digitalWrite(loadgroups[i].mosfetPin, HIGH);
          loadgroups[i].PWM = 100;
        }
        else{
          digitalWrite(loadgroups[i].mosfetPin, LOW);
          loadgroups[i].PWM = 0;
        }
       
      }
    }

    else if(systemState == CONSERVING){
      for(int i = 0; i < loadAmount; i++){
        if(loadgroups[i].LoadPriority == OPTIONAL){
          if(loadgroups[i].AcceptThrottling == true){ 
            unsigned long dt = millis() - throttleStartTime[i];           
            if(dt < 10000){
              analogWrite(loadgroups[i].mosfetPin, 255*0.75); 
              loadgroups[i].PWM = 75;                       
            }
            else if(dt < 20000){
              analogWrite(loadgroups[i].mosfetPin, 255*0.5);
              loadgroups[i].PWM = 50;
            }
            else if(dt < 30000){
              analogWrite(loadgroups[i].mosfetPin, 255*0.25);
              loadgroups[i].PWM = 25;
            }
            else{
              digitalWrite(loadgroups[i].mosfetPin, LOW);
              loadgroups[i].PWM = 0;
            }

          }
          else{
            if(millis() - shedStartTime[i] < 30000){
              digitalWrite(loadgroups[i].mosfetPin, LOW);
              loadgroups[i].PWM = 0;
            }
          }  
        }  

        else if(loadgroups[i].LoadPriority == IMPORTANT){          
          if(loadgroups[i].AcceptThrottling == true){
            unsigned long dt = millis() - throttleStartTime[i];

            if(dt < 10000){
              analogWrite(loadgroups[i].mosfetPin, 255);
              loadgroups[i].PWM = 100;                       
            }
            else if(dt < 20000){
              analogWrite(loadgroups[i].mosfetPin, 255*0.75);
              loadgroups[i].PWM = 75;
            }
            else if(dt < 30000){
              analogWrite(loadgroups[i].mosfetPin, 255*0.5);
              loadgroups[i].PWM = 50;
            }
            else if(dt < 40000){
              analogWrite(loadgroups[i].mosfetPin, 255*0.25);
              loadgroups[i].PWM = 25;
            }
            else{
              digitalWrite(loadgroups[i].mosfetPin, LOW);
              loadgroups[i].PWM = 0;
            }

          }
          else{            
            if(millis() - shedStartTime[i] < 40000){
              digitalWrite(loadgroups[i].mosfetPin, LOW);
              loadgroups[i].PWM = 0;
            }
          }  
        }

        else if(loadgroups[i].LoadPriority == CRITICAL){
          digitalWrite(loadgroups[i].mosfetPin, HIGH);
          loadgroups[i].PWM = 100;
        }
      }
    }
    else{
      for(int i = 0; i < loadAmount; i++){
        digitalWrite(loadgroups[i].mosfetPin, HIGH);
        loadgroups[i].PWM = 100;
        throttleStartTime[i] = millis();
        shedStartTime [i] = millis();
      }      
    }
   
  }
  else if(Fault == OVERCURRENT){
    for(int i = 0; i < loadAmount; i++){
      if(systemState == DISASTER){        
        if(loadgroups[i].LoadPriority == CRITICAL){
          digitalWrite(loadgroups[i].mosfetPin, HIGH);
          loadgroups[i].PWM = 100;
        }
        else if(loadgroups[i].LoadPriority == IMPORTANT){
          if(loadgroups[i].isInductive == true){
            digitalWrite(loadgroups[i].mosfetPin, LOW);
            loadgroups[i].PWM = 0;
          }
          else{
            if(millis()-shedStartTime[i] < 60000){
              digitalWrite(loadgroups[i].mosfetPin, HIGH);
              loadgroups[i].PWM = 100;
            }
            else{
              digitalWrite(loadgroups[i].mosfetPin, LOW);
              loadgroups[i].PWM = 0;
            }
          } 
        }
        else{
          if(loadgroups[i].isInductive == true){
            digitalWrite(loadgroups[i].mosfetPin, LOW);
            loadgroups[i].PWM = 0;
          }
          else{
            if(millis()-shedStartTime[i] < 30000){
              digitalWrite(loadgroups[i].mosfetPin, HIGH);
              loadgroups[i].PWM = 100;
            }
          }
        }
      }
      else if(systemState == CONSERVING){
        if(loadgroups[i].LoadPriority == CRITICAL){
          digitalWrite(loadgroups[i].mosfetPin, HIGH);
          loadgroups[i].PWM = 100;
        }
        else if (loadgroups[i].LoadPriority == IMPORTANT){
          if(loadgroups[i].isInductive){
            if(loadgroups[i].AcceptThrottling == true){
              if(millis()-throttleStartTime[i] < 30000){
                analogWrite(loadgroups[i].mosfetPin, 255*0.5);
                loadgroups[i].PWM = 50;
              }
              else if(millis()-throttleStartTime[i] < 50000){
                analogWrite(loadgroups[i].mosfetPin, 255*0.25);
                loadgroups[i].PWM = 25;
              }
              else{
                digitalWrite(loadgroups[i].mosfetPin, LOW);
                loadgroups[i].PWM = 0;
              }
            }
            else{
              if(millis()-shedStartTime[i] < 50000){
                digitalWrite(loadgroups[i].mosfetPin, HIGH);
                loadgroups[i].PWM = 100;
              }
              else{
                digitalWrite(loadgroups[i].mosfetPin, LOW);
                loadgroups[i].PWM = 0;
              }
            }
          }
          else{
            if(loadgroups[i].AcceptThrottling == true){
              if(millis()-throttleStartTime[i] < 60000){
                analogWrite(loadgroups[i].mosfetPin, 255*0.5);
                loadgroups[i].PWM = 50;
              }
              else if(millis()-throttleStartTime[i] < 80000){
                analogWrite(loadgroups[i].mosfetPin, 255*0.25);
                loadgroups[i].PWM = 25;
              }
              else{
                digitalWrite(loadgroups[i].mosfetPin, LOW);
                loadgroups[i].PWM = 0;
              }
            }
            else{
              if(millis()-shedStartTime[i] < 80000){
                digitalWrite(loadgroups[i].mosfetPin, HIGH);
                loadgroups[i].PWM = 100;
              }
              else{
                digitalWrite(loadgroups[i].mosfetPin, LOW);
                loadgroups[i].PWM = 0;
              }
            }
          }    
        }
        else{
          if(loadgroups[i].isInductive){
            if(loadgroups[i].AcceptThrottling == true){
              if(millis()-throttleStartTime[i] < 10000){
                analogWrite(loadgroups[i].mosfetPin, 255*0.5);
                loadgroups[i].PWM = 50;
              }
              else if(millis()-throttleStartTime[i] < 30000){
                analogWrite(loadgroups[i].mosfetPin, 255*0.25);
                loadgroups[i].PWM = 25;
              }
              else{
                digitalWrite(loadgroups[i].mosfetPin, LOW);
                loadgroups[i].PWM = 0;
              }
            }
            else{
              if(millis()-shedStartTime[i] < 30000){
                digitalWrite(loadgroups[i].mosfetPin, HIGH);
                loadgroups[i].PWM = 100;
              }
              else{
                digitalWrite(loadgroups[i].mosfetPin, LOW);
                loadgroups[i].PWM = 0;
              }
            }
          }
          else{
            if(loadgroups[i].AcceptThrottling == true){
              if(millis()-throttleStartTime[i] < 30000){
                analogWrite(loadgroups[i].mosfetPin, 255*0.5);
                loadgroups[i].PWM = 50;
              }
              else if(millis()-throttleStartTime[i] < 70000){
                analogWrite(loadgroups[i].mosfetPin, 255*0.25);
                loadgroups[i].PWM = 25;
              }
              else{
                digitalWrite(loadgroups[i].mosfetPin, LOW);
                loadgroups[i].PWM = 0;
              }
            }
            else{
              if(millis()-shedStartTime[i] < 70000){
                digitalWrite(loadgroups[i].mosfetPin, HIGH);
                loadgroups[i].PWM = 100;
              }
              else{
                digitalWrite(loadgroups[i].mosfetPin, LOW);
                loadgroups[i].PWM = 0;
              }
            }
          }    
        }          
      }
      else{
        digitalWrite(loadgroups[i].mosfetPin, HIGH);
        loadgroups[i].PWM = 100;
        shedStartTime[i] = millis();
        throttleStartTime[i] = millis();
      }  
    }
    
  }
  else if(Fault == USER_INPUT){
    for(int i = 0; i < loadAmount; i++){
      if(systemState == DISASTER){       
        if(loadgroups[i].LoadPriority == CRITICAL){
          digitalWrite(loadgroups[i].mosfetPin, HIGH);
          loadgroups[i].PWM = 100;
        }
        else{
          digitalWrite(loadgroups[i].mosfetPin, LOW);
          loadgroups[i].PWM = 0;
        }
      }
      else if(systemState == CONSERVING){
        if(loadgroups[i].LoadPriority == CRITICAL){
          digitalWrite(loadgroups[i].mosfetPin, HIGH); 
          loadgroups[i].PWM = 100;         
        }
        else if(loadgroups[i].LoadPriority == IMPORTANT){
          if(loadgroups[i].AcceptThrottling == true){
            analogWrite(loadgroups[i].mosfetPin, 255*0.5);
            loadgroups[i].PWM = 50;
          }
          else{
            digitalWrite(loadgroups[i].mosfetPin, HIGH);
            loadgroups[i].PWM = 100;
          }
        }
        else{
          if(millis()-shedStartTime[i] < 30000){
            if(loadgroups[i].AcceptThrottling == true){
              analogWrite(loadgroups[i].mosfetPin, 255*0.5);
              loadgroups[i].PWM = 50;
            }
            else{
              digitalWrite(loadgroups[i].mosfetPin, HIGH);
              loadgroups[i].PWM = 100;
            }
          }
          else{
            digitalWrite(loadgroups[i].mosfetPin, LOW);
            loadgroups[i].PWM = 0;
          }
        }
      }
      else{
        digitalWrite(loadgroups[i].mosfetPin, HIGH);
        loadgroups[i].PWM = 100;
        throttleStartTime[i] = millis();
        shedStartTime[i] = millis();
      }
    }
    
  }  
  else if(Fault == SENSOR_FAULT){
    static unsigned long buzzerEndTime = 0;
    static int buzzerState = LOW;
    for(int i = 0; i < loadAmount; i++){
      loadgroups[i].PWM = 100;
    if(systemState == DISASTER){
      digitalWrite(buzzer, HIGH);
    }
    else if(systemState == CONSERVING){
      if(millis()-buzzerEndTime > 2000){
        buzzerEndTime = millis();
        if(buzzerState == LOW){
          buzzerState = HIGH;
        }
        else{
          buzzerState = LOW;
        }
      }
      digitalWrite(buzzer, buzzerState);
    }
    else{
      digitalWrite(buzzer, LOW);      
    }
    digitalWrite(loadgroups[i].mosfetPin,HIGH);
    }
    
  }
  else{
    for(int i = 0; i < loadAmount; i++){
      if(systemState == DISASTER){
        if(loadgroups[i].LoadPriority == CRITICAL){
          digitalWrite(loadgroups[i].mosfetPin, HIGH);
          loadgroups[i].PWM = 100;
        }
        else{
          digitalWrite(loadgroups[i].mosfetPin, LOW);
          loadgroups[i].PWM = 0;
        }
      }
      else if(systemState == CONSERVING){
        if(loadgroups[i].LoadPriority == CRITICAL){
          digitalWrite(loadgroups[i].mosfetPin, HIGH); 
          loadgroups[i].PWM = 100;         
        }
        else if(loadgroups[i].LoadPriority == IMPORTANT){
          if(loadgroups[i].AcceptThrottling == true){
            analogWrite(loadgroups[i].mosfetPin, 255*0.5);
            loadgroups[i].PWM = 50;
          }
          else{
            digitalWrite(loadgroups[i].mosfetPin, HIGH);
            loadgroups[i].PWM = 100;
          }
        }
        else{
          if(millis()-shedStartTime[i] < 30000){
            if(loadgroups[i].AcceptThrottling == true){
              analogWrite(loadgroups[i].mosfetPin, 255*0.5);
              loadgroups[i].PWM = 50;
            }
            else{
              digitalWrite(loadgroups[i].mosfetPin, HIGH);
              loadgroups[i].PWM = 100;
            }
          }
          else{
            digitalWrite(loadgroups[i].mosfetPin, LOW);
            loadgroups[i].PWM = 0;
          }
        }
      }
      else{
        digitalWrite(loadgroups[i].mosfetPin, HIGH);
        loadgroups[i].PWM = 100;
        throttleStartTime[i] = millis();
        shedStartTime[i] = millis();
      }
    }
    
  }
}


/*===== 7. lcd display =====*/
void configureLcd(int currentPage){
  static int lastPage = -1;
  static unsigned long lastUpdateTime = 0;
  if(currentPage != lastPage || millis()-lastUpdateTime > 500){
    lcd1.clear();
    lastPage = currentPage;
    lastUpdateTime = millis();
  }

  if(currentPage == 1){    
    lcd1.setCursor(0,0);
    lcd1.print("STATE:");
    switch (systemState){
      case NORMAL: lcd1.print("NORMAL"); break;
      case DISASTER: lcd1.print("DISASTER"); break;
      case CONSERVING: lcd1.print("CONSERVING"); break;
    }
    lcd1.setCursor(0,1);
    lcd1.print("DANGER: ");
    lcd1.print(danger);
    lcd1.print("%");
    return;
  }
  const int sensorStartPage = 2;
  const int sensorsPerPage = 2;
  int totalSensorPages = (NUMOFSENSORS + sensorsPerPage - 1)/sensorsPerPage;
  if(currentPage >= sensorStartPage && currentPage <= sensorStartPage + totalSensorPages - 1){
    int sensorPageNumber = currentPage - sensorStartPage;
    int firstSensor = sensorPageNumber*sensorsPerPage;
    for(int i = 0; i < sensorsPerPage; i++){
      int sensorNumber = firstSensor + i;
      if(sensorNumber >= NUMOFSENSORS) break;
      lcd1.setCursor(0,i);
      lcd1.print("V"); lcd1.print(sensorNumber + 1); lcd1.print(":"); lcd1.print(sensorData[sensorNumber].voltage,1); lcd1.print("V");
      lcd1.print(" I"); lcd1.print(sensorNumber + 1); lcd1.print(":"); lcd1.print(sensorData[sensorNumber].current/1000,2); lcd1.print("A");
    }
    return;
  }  
  int loadStartPage = sensorStartPage + totalSensorPages;
  int loadPerPage = 2;
  int totalLoadPages = (loadAmount + loadPerPage - 1)/loadPerPage;
  if(currentPage >= loadStartPage && currentPage <= loadStartPage + totalLoadPages - 1){
    int loadPageNumber = currentPage - loadStartPage;
    int firstLoad = loadPageNumber * loadPerPage;
    for(int i = 0; i < loadPerPage; i++){
      int loadNumber = firstLoad + i;
      if(loadNumber >= loadAmount) break;
      lcd1.setCursor(0,i);
      lcd1.print("LG"); lcd1.print(loadNumber + 1); lcd1.print(": ");
      switch (loadgroups[loadNumber].LoadPriority){
        case CRITICAL: lcd1.print("! "); break;
        case IMPORTANT: lcd1.print("* "); break;
        case OPTIONAL: lcd1.print("- "); break;
      }
      lcd1.print("PWM:"); lcd1.print(loadgroups[loadNumber].PWM); lcd1.print("%  ");      
    }
    return;
  }
  
  int faultPage = loadStartPage + totalLoadPages;
  if(currentPage == faultPage){
    lcd1.setCursor(0,0);
    lcd1.print("F:");
    switch(Fault){
      case NO_FAULT: lcd1.print("NO FAULT"); break;
      case SENSOR_FAULT: lcd1.print("SENSOR FAULT"); break;
      case UNDERVOLTAGE: lcd1.print("UNDERVOLTAGE"); break;
      case OVERVOLTAGE: lcd1.print("OVERVOLTAGE"); break;
      case OVERCURRENT: lcd1.print("OVERCURRENT"); break;
      case USER_INPUT: lcd1.print("USER INPUT"); break;
    }
    lcd1.setCursor(0,1);
    lcd1.print("FLOWGRID V1.0");
  }
  totalPageNumber = faultPage;
}



/*===== 8. lcd buttons =====*/
int lcdButtons(){
  static unsigned long lastButtonTime = 0;
  const unsigned long debounce = 200;
  static int currentPageNumber = 1;

  if (millis() - lastButtonTime < debounce) return currentPageNumber;
  
  if(digitalRead(buttonNextPage) == HIGH){
    currentPageNumber += 1; 
    lastButtonTime = millis();   
  }
  if(digitalRead(buttonPrevPage) == HIGH){
    currentPageNumber -= 1;
    lastButtonTime = millis();
  }
  if(currentPageNumber > totalPageNumber) currentPageNumber = 1;
  
  if(currentPageNumber < 1) currentPageNumber = totalPageNumber;
  

  if(digitalRead(buttonNormal) == HIGH||digitalRead(buttonDisaster) == HIGH||digitalRead(buttonConserving) == HIGH){
    currentPageNumber = 1;
    lastButtonTime = millis();
  }
  return currentPageNumber;
}



/*===== ARDUINO EXECUUTION =====*/
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);
  SensorSetup();
  ina219_1.setCalibration_16V_400mA();
  ina219_2.setCalibration_16V_400mA();

  pinMode(isolationRelay, OUTPUT);
  for(int i = 0; i < loadAmount; i++){
    pinMode(loadgroups[i].mosfetPin, OUTPUT);
  }
  pinMode(ledNormal, OUTPUT);
  pinMode(ledConserving, OUTPUT);
  pinMode(ledDisaster, OUTPUT);
  pinMode(buttonNormal, INPUT);
  pinMode(buttonConserving, INPUT);
  pinMode(buttonDisaster, INPUT);
  pinMode(buttonNextPage, INPUT);
  pinMode(buttonPrevPage, INPUT);
  pinMode(buzzer, OUTPUT);
  lcd1.begin(16, 2);
  lcd1.clear();
  wdt_enable(WDTO_2S);  
}

void loop() {
  wdt_reset();
  retryMissingSensors();
  measureAndValidate();
  danger = computeDangerScore();
  hardwareControl(danger);
  printSensorData(danger);
  pageNumber = lcdButtons();
  configureLcd(pageNumber);
}

/*Marwan Khaled*/
