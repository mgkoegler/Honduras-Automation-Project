#include <OneWire.h>

#define TempSense1 22 
#define TempSense2 23

/******** Definitions for EC Sensor 1 *********/

#define StartConvert1 0
#define ReadTemperature1 1
#define TempSense3 24
#define CondTempSense1 A1

/******** Definitions for EC Sensor 2 *********/

#define StartConvert2 0
#define ReadTemperature2 1
#define TempSense4 25
#define CondTempSense2 A2

/******** Definitions for EC Sensor 3 *********/

#define StartConvert3 0
#define ReadTemperature3 1
#define TempSense5 26
#define CondTempSense3 A3

/******** Definitions for pH Sensor 1 *********/

#define pHSense1 A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset1 0.00            //deviation compensate
#define LED1 13
#define samplingInterval1 20
#define prInterval1 800
#define ArrayLenth1  40    //times of collection

//Declaration of Temperature Sensor 1 Object
OneWire ds1(TempSense1);
//Declaration of Temperature Sensor 2 Object
OneWire ds2(TempSense2);
//Declaration of Temperature Sensor 3 Object
OneWire ds3(TempSense3);  // on digital pin 2
//Declaration of Temperature Sensor 4 Object
OneWire ds4(TempSense4);  // on digital pin 2
//Declaration of Temperature Sensor 4 Object
OneWire ds5(TempSense5);  // on digital pin 2

/******** Pre-Setup Code for EC Sensor 1 *********/

const byte numReadings1 = 20;     //the number of sample times
unsigned int AnalogSampleInterval1=25,printInterval1=700,tempSampleInterval1=850;  //analog sample interval;serial print interval;temperature sample interval
unsigned int readings1[numReadings1];      // the readings from the analog input
byte index1 = 0;                  // the index of the current reading
unsigned long AnalogValueTotal1 = 0;                  // the running total
unsigned int AnalogAverage1 = 0,averageVoltage1=0;                // the average
unsigned long AnalogSampleTime1,printTime1,tempSampleTime1;
float temperature3,ECcurrent1; 

/******** Pre-Setup Code for EC Sensor 2 *********/

const byte numReadings2 = 20;     //the number of sample times
unsigned int AnalogSampleInterval2=25,printInterval2=700,tempSampleInterval2=850;  //analog sample interval;serial print interval;temperature sample interval
unsigned int readings2[numReadings2];      // the readings from the analog input
byte index2 = 0;                  // the index of the current reading
unsigned long AnalogValueTotal2 = 0;                  // the running total
unsigned int AnalogAverage2 = 0,averageVoltage2=0;                // the average
unsigned long AnalogSampleTime2,printTime2,tempSampleTime2;
float temperature4,ECcurrent2; 

/******** Pre-Setup Code for EC Sensor 3 *********/

const byte numReadings3 = 20;     //the number of sample times
unsigned int AnalogSampleInterval3=25,printInterval3=700,tempSampleInterval3=850;  //analog sample interval;serial print interval;temperature sample interval
unsigned int readings3[numReadings3];      // the readings from the analog input
byte index3 = 0;                  // the index of the current reading
unsigned long AnalogValueTotal3 = 0;                  // the running total
unsigned int AnalogAverage3 = 0,averageVoltage3=0;                // the average
unsigned long AnalogSampleTime3,printTime3,tempSampleTime3;
float temperature5,ECcurrent3;                                                                                                                                              

/******** Pre-Setup for pH Sensor 1 *********/

int pHArray1[ArrayLenth1];   //Store the average value of the sensor feedback
int pHArrayIndex1=0;    

void setup(void) {
  
Serial.begin(115200);

/******** Setup Code for EC Sensor 1 *********/

  // initialize all the readings to 0:
  for (byte thisReading1 = 0; thisReading1 < numReadings1; thisReading1++)
    readings1[thisReading1] = 0;
  TempProcess1(StartConvert1);   //let the DS18B20 start the convert
  AnalogSampleTime1=millis();
  printTime1=millis();
  tempSampleTime1=millis();

/******** Setup Code for EC Sensor 2 *********/

  // initialize all the readings to 0:
  for (byte thisReading2 = 0; thisReading2 < numReadings2; thisReading2++)
    readings2[thisReading2] = 0;
  TempProcess2(StartConvert2);   //let the DS18B20 start the convert
  AnalogSampleTime2=millis();
  printTime2=millis();
  tempSampleTime2=millis();

/******** Setup Code for EC Sensor 3 *********/

  // initialize all the readings to 0:
  for (byte thisReading3 = 0; thisReading3 < numReadings3; thisReading3++)
    readings3[thisReading3] = 0;
  TempProcess3(StartConvert3);   //let the DS18B20 start the convert
  AnalogSampleTime3=millis();
  printTime3=millis();
  tempSampleTime3=millis();

/******** Setup Code for pH Sensor 1 *********/

  pinMode(LED1,OUTPUT);  
  Serial.println("pH meter experiment!");    //Test the serial monitor
  
}

void loop(void) {
  
/******** Loop Code for Temperature Sensor 1 *********/

  float temperature1 = getTemp1();
  Serial.print("Temp Sense 1: ");
  Serial.print(temperature1);

  Serial.println("");

/******** Loop Code for Temperature Sensor 2 *********/

  float temperature2 = getTemp2();
  Serial.print("Temp Sense 2: ");
  Serial.print(temperature2);
  
  Serial.println("");
  
  delay(1000); //just here to slow down the output so it is easier to read

/******** Loop Code for EC Sensor 1 *********/
  
  /*
   Every once in a while,sample the analog value and calculate the average.
  */
  if(millis()-AnalogSampleTime1>=AnalogSampleInterval1)  
  {
    AnalogSampleTime1=millis();
     // subtract the last reading:
    AnalogValueTotal1 = AnalogValueTotal1 - readings1[index1];
    // read from the sensor:
    readings1[index1] = analogRead(CondTempSense1);
    // add the reading to the total:
    AnalogValueTotal1 = AnalogValueTotal1 + readings1[index1];
    // advance to the next position in the array:
    index1 = index1 + 1;
    // if we're at the end of the array...
    if (index1 >= numReadings1)
    // ...wrap around to the beginning:
    index1 = 0;
    // calculate the average:
    AnalogAverage1 = AnalogValueTotal1 / numReadings1;
  }
  /*
   Every once in a while,MCU read the temperature from the DS18B20 and then let the DS18B20 start the convert.
   Attention:The interval between start the convert and read the temperature should be greater than 750 millisecond,or the temperature is not accurate!
  */
   if(millis()-tempSampleTime1>=tempSampleInterval1) 
  {
    tempSampleTime1=millis();
    temperature3 = TempProcess1(ReadTemperature1);  // read the current temperature from the  DS18B20
    TempProcess1(StartConvert1);                   //after the reading,start the convert for next reading
  }
   /*
   Every once in a while,print the information on the serial monitor.
  */
  if(millis()-printTime1>=printInterval1)
  {
    printTime1=millis();
    averageVoltage1=AnalogAverage1*(float)5000/1024;
    Serial.print("EC1 - ");
    Serial.print("Analog value:");
    Serial.print(AnalogAverage1);   //analog average,from 0 to 1023
    Serial.print("    Voltage:");
    Serial.print(averageVoltage1);  //millivolt average,from 0mv to 4995mV
    Serial.print("mV    ");
    Serial.print("temp:");
    Serial.print(temperature3);    //current temperature
    Serial.print("^C     EC:");
    
    float TempCoefficient1=1.0+0.0185*(temperature3-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVolatge1=(float)averageVoltage1/TempCoefficient1;   
    if(CoefficientVolatge1<150)Serial.println("No solution!");   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
    else if(CoefficientVolatge1>3300)Serial.println("Out of the range!");  //>20ms/cm,out of the range
    else
    { 
      if(CoefficientVolatge1<=448)ECcurrent1=6.84*CoefficientVolatge1-64.32;   //1ms/cm<EC<=3ms/cm
      else if(CoefficientVolatge1<=1457)ECcurrent1=6.98*CoefficientVolatge1-127;  //3ms/cm<EC<=10ms/cm
      else ECcurrent1=5.3*CoefficientVolatge1+2278;                           //10ms/cm<EC<20ms/cm
      ECcurrent1/=1000;    //convert us/cm to ms/cm
      Serial.print(ECcurrent1,2);  //two decimal
      Serial.println("ms/cm");
    }
  }

/******** Loop Code for EC Sensor 2 *********/

  /*
   Every once in a while,sample the analog value and calculate the average.
  */
  if(millis()-AnalogSampleTime2>=AnalogSampleInterval2)  
  {
    AnalogSampleTime2=millis();
     // subtract the last reading:
    AnalogValueTotal2 = AnalogValueTotal2 - readings2[index2];
    // read from the sensor:
    readings2[index2] = analogRead(CondTempSense2);
    // add the reading to the total:
    AnalogValueTotal2 = AnalogValueTotal2 + readings2[index2];
    // advance to the next position in the array:
    index2 = index2 + 1;
    // if we're at the end of the array...
    if (index2 >= numReadings2)
    // ...wrap around to the beginning:
    index2 = 0;
    // calculate the average:
    AnalogAverage2 = AnalogValueTotal2 / numReadings2;
  }
  /*
   Every once in a while,MCU read the temperature from the DS18B20 and then let the DS18B20 start the convert.
   Attention:The interval between start the convert and read the temperature should be greater than 750 millisecond,or the temperature is not accurate!
  */
   if(millis()-tempSampleTime2>=tempSampleInterval2) 
  {
    tempSampleTime2=millis();
    temperature4 = TempProcess2(ReadTemperature2);  // read the current temperature from the  DS18B20
    TempProcess2(StartConvert2);                   //after the reading,start the convert for next reading
  }
   /*
   Every once in a while,print the information on the serial monitor.
  */
  if(millis()-printTime2>=printInterval2)
  {
    printTime2=millis();
    averageVoltage2=AnalogAverage2*(float)5000/1024;
    Serial.print("EC2 - ");
    Serial.print("Analog value:");
    Serial.print(AnalogAverage2);   //analog average,from 0 to 1023
    Serial.print("    Voltage:");
    Serial.print(averageVoltage2);  //millivolt average,from 0mv to 4995mV
    Serial.print("mV    ");
    Serial.print("temp:");
    Serial.print(temperature4);    //current temperature
    Serial.print("^C     EC:");
    
    float TempCoefficient2=1.0+0.0185*(temperature4-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVolatge2=(float)averageVoltage2/TempCoefficient2;   
    if(CoefficientVolatge2<150)Serial.println("No solution!");   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
    else if(CoefficientVolatge2>3300)Serial.println("Out of the range!");  //>20ms/cm,out of the range
    else
    { 
      if(CoefficientVolatge2<=448)ECcurrent2=6.84*CoefficientVolatge2-64.32;   //1ms/cm<EC<=3ms/cm
      else if(CoefficientVolatge2<=1457)ECcurrent2=6.98*CoefficientVolatge2-127;  //3ms/cm<EC<=10ms/cm
      else ECcurrent2=5.3*CoefficientVolatge2+2278;                           //10ms/cm<EC<20ms/cm
      ECcurrent2/=1000;    //convert us/cm to ms/cm
      Serial.print(ECcurrent2,2);  //two decimal
      Serial.println("ms/cm");
    }
  }


/******** Loop Code for EC Sensor 3 *********/

  /*
   Every once in a while,sample the analog value and calculate the average.
  */
  if(millis()-AnalogSampleTime3>=AnalogSampleInterval3)  
  {
    AnalogSampleTime3=millis();
     // subtract the last reading:
    AnalogValueTotal3 = AnalogValueTotal3 - readings3[index3];
    // read from the sensor:
    readings3[index3] = analogRead(CondTempSense3);
    // add the reading to the total:
    AnalogValueTotal3 = AnalogValueTotal3 + readings3[index3];
    // advance to the next position in the array:
    index3 = index3 + 1;
    // if we're at the end of the array...
    if (index3 >= numReadings3)
    // ...wrap around to the beginning:
    index3 = 0;
    // calculate the average:
    AnalogAverage3 = AnalogValueTotal3 / numReadings3;
  }
  /*
   Every once in a while,MCU read the temperature from the DS18B20 and then let the DS18B20 start the convert.
   Attention:The interval between start the convert and read the temperature should be greater than 750 millisecond,or the temperature is not accurate!
  */
   if(millis()-tempSampleTime3>=tempSampleInterval3) 
  {
    tempSampleTime3=millis();
    temperature5 = TempProcess3(ReadTemperature3);  // read the current temperature from the  DS18B20
    TempProcess3(StartConvert3);                   //after the reading,start the convert for next reading
  }
   /*
   Every once in a while,print the information on the serial monitor.
  */
  if(millis()-printTime3>=printInterval3)
  {
    printTime3=millis();
    averageVoltage3=AnalogAverage3*(float)5000/1024;
    Serial.print("EC3 - ");
    Serial.print("Analog value:");
    Serial.print(AnalogAverage3);   //analog average,from 0 to 1023
    Serial.print("    Voltage:");
    Serial.print(averageVoltage3);  //millivolt average,from 0mv to 4995mV
    Serial.print("mV    ");
    Serial.print("temp:");
    Serial.print(temperature5);    //current temperature
    Serial.print("^C     EC:");
    
    float TempCoefficient3=1.0+0.0185*(temperature5-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVolatge3=(float)averageVoltage3/TempCoefficient3;   
    if(CoefficientVolatge3<150)Serial.println("No solution!");   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
    else if(CoefficientVolatge3>3300)Serial.println("Out of the range!");  //>20ms/cm,out of the range
    else
    { 
      if(CoefficientVolatge3<=448)ECcurrent3=6.84*CoefficientVolatge3-64.32;   //1ms/cm<EC<=3ms/cm
      else if(CoefficientVolatge3<=1457)ECcurrent3=6.98*CoefficientVolatge3-127;  //3ms/cm<EC<=10ms/cm
      else ECcurrent3=5.3*CoefficientVolatge3+2278;                           //10ms/cm<EC<20ms/cm
      ECcurrent3/=1000;    //convert us/cm to ms/cm
      Serial.print(ECcurrent3,2);  //two decimal
      Serial.println("ms/cm");
    }
  }

/******** Loop Code for pH Sensor 1 *********/

  static unsigned long samplingTime1 = millis();
  static unsigned long printTime1 = millis();
  static float pHValue1,voltage1;
  if(millis()-samplingTime1 > samplingInterval1)
  {
      pHArray1[pHArrayIndex1++]=analogRead(pHSense1);
      if(pHArrayIndex1==ArrayLenth1)pHArrayIndex1=0;
      voltage1 = avergearray1(pHArray1, ArrayLenth1)*5.0/1024;
      pHValue1 = 3.5*voltage1+Offset1;
      samplingTime1=millis();
  }
  if(millis() - printTime1 > prInterval1)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
  Serial.print("Voltage:");
        Serial.print(voltage1,2);
        Serial.print("    pH value: ");
  Serial.println(pHValue1,2);
        digitalWrite(LED1,digitalRead(LED1)^1);
        printTime1=millis();
  }
  
}

/******** Function for Temperature Sensor 1 *********/

float getTemp1() {
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds1.search(addr)) {
    //no more sensors on chain, reset search
    ds1.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds1.reset();
  ds1.select(addr);
  ds1.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds1.reset();
  ds1.select(addr);
  ds1.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds1.read();
  }

  ds1.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}

/******** Function for Temperature Sensor 2 *********/

float getTemp2() {
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds2.search(addr)) {
    //no more sensors on chain, reset search
    ds2.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds2.reset();
  ds2.select(addr);
  ds2.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds2.reset();
  ds2.select(addr);
  ds2.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds2.read();
  }

  ds2.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}

/******** Thermometer Function Code for EC Sensor 1 *********/

/*
ch=0,let the DS18B20 start the convert;ch=1,MCU read the current temperature from the DS18B20.
*/
float TempProcess1(bool ch)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if(!ch){
          if ( !ds3.search(addr)) {
              Serial.println("no more sensors on chain, reset search!");
              ds3.reset_search();
              return 0;
          }      
          if ( OneWire::crc8( addr, 7) != addr[7]) {
              Serial.println("CRC is not valid!");
              return 0;
          }        
          if ( addr[0] != 0x10 && addr[0] != 0x28) {
              Serial.print("Device is not recognized!");
              return 0;
          }      
          ds3.reset();
          ds3.select(addr);
          ds3.write(0x44,1); // start conversion, with parasite power on at the end
  }
  else{  
          byte present = ds3.reset();
          ds3.select(addr);    
          ds3.write(0xBE); // Read Scratchpad            
          for (int i = 0; i < 9; i++) { // we need 9 bytes
            data[i] = ds3.read();
          }         
          ds3.reset_search();           
          byte MSB = data[1];
          byte LSB = data[0];        
          float tempRead = ((MSB << 8) | LSB); //using two's compliment
          TemperatureSum = tempRead / 16;
    }
          return TemperatureSum;  
}

/******** Thermometer Function Code for EC Sensor 2 *********/

/*
ch=0,let the DS18B20 start the convert;ch=1,MCU read the current temperature from the DS18B20.
*/
float TempProcess2(bool ch)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if(!ch){
          if ( !ds4.search(addr)) {
              Serial.println("no more sensors on chain, reset search!");
              ds4.reset_search();
              return 0;
          }      
          if ( OneWire::crc8( addr, 7) != addr[7]) {
              Serial.println("CRC is not valid!");
              return 0;
          }        
          if ( addr[0] != 0x10 && addr[0] != 0x28) {
              Serial.print("Device is not recognized!");
              return 0;
          }      
          ds4.reset();
          ds4.select(addr);
          ds4.write(0x44,1); // start conversion, with parasite power on at the end
  }
  else{  
          byte present = ds4.reset();
          ds4.select(addr);    
          ds4.write(0xBE); // Read Scratchpad            
          for (int i = 0; i < 9; i++) { // we need 9 bytes
            data[i] = ds4.read();
          }         
          ds4.reset_search();           
          byte MSB = data[1];
          byte LSB = data[0];        
          float tempRead = ((MSB << 8) | LSB); //using two's compliment
          TemperatureSum = tempRead / 16;
    }
          return TemperatureSum;  
}

/******** Thermometer Function Code for EC Sensor 3 *********/

/*
ch=0,let the DS18B20 start the convert;ch=1,MCU read the current temperature from the DS18B20.
*/
float TempProcess3(bool ch)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if(!ch){
          if ( !ds5.search(addr)) {
              Serial.println("no more sensors on chain, reset search!");
              ds5.reset_search();
              return 0;
          }      
          if ( OneWire::crc8( addr, 7) != addr[7]) {
              Serial.println("CRC is not valid!");
              return 0;
          }        
          if ( addr[0] != 0x10 && addr[0] != 0x28) {
              Serial.print("Device is not recognized!");
              return 0;
          }      
          ds5.reset();
          ds5.select(addr);
          ds5.write(0x44,1); // start conversion, with parasite power on at the end
  }
  else{  
          byte present = ds5.reset();
          ds5.select(addr);    
          ds5.write(0xBE); // Read Scratchpad            
          for (int i = 0; i < 9; i++) { // we need 9 bytes
            data[i] = ds5.read();
          }         
          ds5.reset_search();           
          byte MSB = data[1];
          byte LSB = data[0];        
          float tempRead = ((MSB << 8) | LSB); //using two's compliment
          TemperatureSum = tempRead / 16;
    }
          return TemperatureSum;  
}

/******** Function for pH Sensor 1 *********/

double avergearray1(int* arr1, int number1){
  int i;
  int max,min;
  double avg1;
  long amount1=0;
  if(number1<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number1<5){   //less than 5, calculated directly statistics
    for(i=0;i<number1;i++){
      amount1+=arr1[i];
    }
    avg1 = amount1/number1;
    return avg1;
  }else{
    if(arr1[0]<arr1[1]){
      min = arr1[0];max=arr1[1];
    }
    else{
      min=arr1[1];max=arr1[0];
    }
    for(i=2;i<number1;i++){
      if(arr1[i]<min){
        amount1+=min;        //arr<min
        min=arr1[i];
      }else {
        if(arr1[i]>max){
          amount1+=max;    //arr>max
          max=arr1[i];
        }else{
          amount1+=arr1[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg1 = (double)amount1/(number1-2);
  }//if
  return avg1;
}
