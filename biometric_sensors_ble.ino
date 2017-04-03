//BLE init
#include <SoftwareSerial.h>
#define BT_SERIAL_TX 10 //For BLE modules's TXD pin
#define BT_SERIAL_RX 11 //For BLE modules's RXD pin
SoftwareSerial BluetoothSerial(BT_SERIAL_TX, BT_SERIAL_RX);


//Accelerometer init
#include <Wire.h>
#include "MMA7660.h"
MMA7660 accelemeter;


//GSR init
const int GSR=A2;
int threshold=0;
int sensorValue;


//HR init
unsigned int max_time_len = 31;
unsigned int max_hr_len = 31;
unsigned long temp_time_intervals[31];
unsigned long sub;
unsigned long heart_rate_arr[31];
unsigned char counter;
unsigned char hr_counter;
unsigned int heart_rate;//the output heart rate value
const int max_heartpulse_duty = 2000; //Error caused if duty exceeds 2s
bool beat_detect=true; 



void setup()
{ 
  //Every sensor
  Serial.begin(9600);
  BluetoothSerial.begin(9600);


  //BLE setup
  //OK
  BluetoothSerial.print("AT");
  waitForResponse();
  
  //version
  BluetoothSerial.print("AT+VERSION");
  waitForResponse();
  
  //Pin = 1234
  BluetoothSerial.print("AT+PIN1234");
  waitForResponse();
  
  //name = BLU
  BluetoothSerial.print("AT+NAMEBLU");
  waitForResponse();
  
  //AT+BAUD4 for baudrate 9600, this is 57600
  BluetoothSerial.print("AT+BAUD7"); 
  waitForResponse();


  
  //Accelerometer setup
  accelemeter.init();  



  //GSR setup
  long gsr_sum=0;
  delay(2000); //delay to give time before sensor gets baseline

  for(int i=0;i<500;i++)
  {
    sensorValue=analogRead(GSR);
    Serial.println("GSR SENSOR VAL = " + sensorValue);
  }




   //HR setup
   Serial.println("Get ready for HR monitoring");
   delay(5000);
   arrayInit();
   arrayInit2();
   Serial.println("Heart rate test begin. Allow time for values to normalise");
   attachInterrupt(0, interrupt, RISING);//set interrupt 0,digital port 2



   Serial.println("**************************************");
}




void loop()
{

  //Every sensor
  delay(2000);
  Serial.print('#'); //Start char
  Serial.print(millis()); //Timestamp
  Serial.print(", ");

  
  //Accelerometer loop
  int8_t x,y,z;
  float ax,ay,az;
  
  accelemeter.getXYZ(&x,&y,&z);
  accelemeter.getAcceleration(&ax,&ay,&az);

  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.print(", ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.print(az);
  Serial.print(", ");


  //GSR loop
  sensorValue=analogRead(GSR);
  Serial.print(sensorValue);
  Serial.print(", ");

  
 


   //HR loop

    long hr_sum = 0L ;  
    for (int i = 0 ; i < max_hr_len ; i++)
    {  
      hr_sum += heart_rate_arr [i] ;
    }

    heart_rate = ((float) hr_sum) / max_hr_len;
    
    Serial.print(heart_rate);
    Serial.print(", ");

    Serial.println('~'); //End char

//x, y, z, ax, ay, az, sensorValue, heart_rate

  
}






//Interrupt for heart beats
void interrupt()
{
    
    temp_time_intervals[counter]=millis();

    switch(counter)
    {
        case 0:
            sub=temp_time_intervals[counter]-temp_time_intervals[max_time_len - 1];
            break;
        default:
            sub=temp_time_intervals[counter]-temp_time_intervals[counter-1];
            break;
    }
    if(sub>max_heartpulse_duty) //If sub > 2s
    {
        beat_detect=0;//sign bit
        counter=0;
        Serial.println("Heart rate measure error!" );
//        arrayInit();
    }

    
    if (counter==max_time_len - 1&&beat_detect)
    {
        sum();
        counter=0;
    }
    else if(counter!=max_time_len - 1&&beat_detect)
    {
      sum();
      counter++;
    }
    
    else 
    {
        counter=0;
        beat_detect=1;
    }

}


// Get HR
void sum()
    {
     if(beat_detect && counter==max_time_len - 1)
        {
          heart_rate_arr[counter] = (60000*(max_time_len-1))/(temp_time_intervals[max_time_len - 1]-temp_time_intervals[0]);
        }
      if(beat_detect && counter!=max_time_len - 1)
        {
          heart_rate_arr[counter] =(60000*(max_time_len-1))/(temp_time_intervals[counter]-temp_time_intervals[counter+1]);
        }
       beat_detect=1;//sign bit
    }

    
void arrayInit()
{
    for(unsigned char i=0;i < max_time_len - 1;i ++)
    {
        temp_time_intervals[i]=0;
    }
    temp_time_intervals[max_time_len - 1]=millis();
}

void arrayInit2()
{
    for(unsigned char i=0;i < max_hr_len - 1;i ++)
    {
        heart_rate_arr[i]=0;
    }
    heart_rate_arr[max_hr_len - 1]=0;
}


// For Bluetooth response
void waitForResponse() {
  delay(1000);
  while (BluetoothSerial.available()) {
    Serial.write(BluetoothSerial.read());
  }
  Serial.write("\n");
}

