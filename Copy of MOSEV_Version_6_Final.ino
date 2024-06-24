#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "ADS1X15.h"

//ADC (the library can be uniformed for two sensors, use either of the two)
Adafruit_ADS1115 ads; //for pressure sensor
ADS1115 ADS(0x48); //for encoder

//For DWIN configuration (the fifth term is the address)
unsigned char Buffer[9];
unsigned char displayHOLDTIME[8] = {0x5a, 0xa5, 0x05, 0x82, 0x50, 0x00, 0x00, 0x00};
unsigned char display_I[8] = {0x5a, 0xa5, 0x05, 0x82, 0x56, 0x00, 0x00, 0x00};
unsigned char displayBreathsPerMinute[8] = {0x5a, 0xa5, 0x05, 0x82, 0x54, 0x00, 0x00, 0x00};
unsigned char displaySensitivity[8] = {0x5a, 0xa5, 0x05, 0x82, 0x58, 0x00, 0x00, 0x00};
unsigned char displayVolume[8] = {0x5a, 0xa5, 0x05, 0x82, 0x75, 0x00, 0x00, 0x00};
unsigned char displayVolumeDefault[8] = {0x5a, 0xa5, 0x05, 0x82, 0x75, 0x00, 0x00, 0x00};
unsigned char displayPEEP[8] = {0x5a, 0xa5, 0x05, 0x82, 0x64, 0x00, 0x00, 0x00};
unsigned char displayPIP[8] = {0x5a, 0xa5, 0x05, 0x82, 0x65, 0x00, 0x00, 0x00};
unsigned char displayPlateau[8] = {0x5a, 0xa5, 0x05, 0x82, 0x66, 0x00, 0x00, 0x00};

//time initialization
unsigned long lastBreathTime = 0;  // Time when the last breath cycle started
float Initial_Inhale_Duration = 0;  // Time it actually took to reach the target pressureValue
int start_time_for_hold = 0;
float relaxTime = 0;
float start_time_exhale = 0;
float start_time_inhale = 0;
float end_time_exhale = 0;
float Total_Exhale_Duration = 0;
float Total_Inhale_Duration = 0;
float exhaleDuration = 0;
int Set_holdTime = 0; 
float elapsed_time = 0;
int actual_holdTime = 0;
int start_time_marker = 0;
int stop_time_marker = 0;
unsigned long lastStateChangeTime = 0;// Track the last time a state change was detected
unsigned long debouncePeriod = 10;// Define a "debounce" period (in milliseconds)
int lastSetHoldTime = 500;
float time_turned_off = 0;
float time_off_duration = 0;

//For flags
bool IMVswitchRead = false;
bool assistSwitchRead = false;
bool IEControlSwitchRead = false;
bool holdTimeSwitchRead = false;
bool firstRun = true; //to track if it's the first run
bool adjustTime = false; // Flag to indicate if time needs to be adjusted

// Forward declaration
void displaySNTVTYFunc(int sensitivity_slider = 0, bool useDefault = false);

//Variable initialization
int initialSpeed = 110;
int lastSpeed = 0;
float scalingFactor = 1.0;  // Initialize scalingFactor to 1.0
float setpointPressure = 0;
int BPMslider = 0;
float cycleDuration = 0;
int sensitivity_slider = 0;
float sensitivityValue = 1; // let's put the value of -1 cmH2O as default
int Slider_HoldTime = 0;
int pressureSlider = 0;
int mappedPressure = 0; 
int pressureMin = 0;    //set the minimum pressureValue here 
int pressureMax = 26;   //set the maximum pressureValue here 
int volume = 0; 
int finalVolume = 0;
float m;
float b;
float angle = 0;
float encoderAngle = 0;
int BPMvalue = 5; //ensuring the scalingFactor will not become zero after the firt run
int inHome = 0; // if inHome == 0 that is the phase for going to home position, while inHome == 1 is the phase of targeting the setpoint pressureValue
float setpointAngle = 0;
int I = 1;
float E = 0;
int assist = 0;
int PEEP2 = 0;
int count = 0;

//For pressureValue sensor
int sensorPin = A7;  //pin for pressureValue sensor
int sensorValue = 0;
int sensorValue1 = 0;
int sensorValue2 = 0;
float diffPressure = 0;
float pressureKPA = 0;
float pressure_cmH2O = 0;
float signal1 = 0;
float Vout = 0;
float unconvertedPressure = 0;
float pressureValue = 0;
int peak_Pressure = 0.0;
int PEEP = 0;
float calibratedCmH2O = 0;
float previousLocal = 0;
int plateau = 0.0;
int i = 0;

//For PID
float last_error = 0;
float error = 0;
float changeError = 0;
float totalError = 0;
int pidTerm = 0;
int pidTerm_scaled = 0;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|
double Kp = 5.2;  
double Ki = 0.0; 
double Kd = 3.5;  
double Kp_angle = 5.2;
double Ki_angle = 0.0;
double Kd_angle = 3.5;

// For motor driver
double pwm1 = 3;// this is the PWM pin for the motor for how much we move it to correct for its error  
const byte dir1 = A0;//these pins are to control the direction of the motor (clockwise/counter-clockwise)
 
//for moving average
float sum1;
float READINGS[5];
float averaged = 0;
float pressure = 0;
int indexx = 0;
int windowsize= 5;

void setup() 
{
  Serial.begin (115200);                                   
  Serial2.begin(115200);  // for RX and TX of DWIN
  Wire.begin();
  ADS.begin();
  ads.begin();
  digitalWrite (pwm1, LOW);
  digitalWrite (pwm1, 0);
  pinMode(dir1, OUTPUT);
  pinMode(36, OUTPUT);// connected to S terminal of Relay
  digitalWrite(36, LOW);// turn relay OFF

 if (!ADS.begin()) 
  {
   Serial.println("Failed to initialize ADS.");
  }

  // Calculate slope and intercept based on motor angle (for equivalent volume reading)
  float x1 = 150, y1 = 200; // x is the angle and y is the corresponding volume (mL)
  float x2 = 210, y2 = 600;
  m = (y2 - y1) / (x2 - x1);
  b = y1 - m * x1;

  displaySNTVTYFunc(0, true); //ensuring it starts at the default value.
}

void loop() 
{
  
  if (IMVswitchRead && adjustTime) // this block and the next block purpose is to account for time when IMV is off to avoid large inhaleDuration and relaxTime after turning on the IMV switch
  {
    // Only enter this block if IMV is turned ON and time needs to be adjusted
    
    // Calculate the time duration for which IMV was off
    unsigned long time_off_duration = millis() - time_turned_off;

    // Adjust the start times by subtracting the off-time
    start_time_inhale = millis() - time_off_duration;
    start_time_exhale = millis() - time_off_duration;

    // Reset the flag so that this adjustment only happens once
    adjustTime = false;
  }

  if (!IMVswitchRead) 
  {
    // Capture the time when IMV was turned off
    time_turned_off = millis();

    // Set the flag to adjust time when turning back ON
    adjustTime = true;
  }

  static unsigned long displayUpdateTime = 0;
  if (millis() - displayUpdateTime > 1000) // let's say, every 1 second
  {
    updateDisplayValues();
    
    if (!IMVswitchRead)
     {
       if (!IEControlSwitchRead)    
              {
                displayExhaleTime_for_IE_Default();  //displays the default E to "2"
              } 
        else if (IEControlSwitchRead)  
              {
                displayExhaleTime_for_IE();  //displays E based on cycle duration selection
              }
      displayVolume_Default();    //displays the volume to zero as default
      displayPIPvalueIMVOFF();   //displays the PIP to zero when IMV is off
        if (!holdTimeSwitchRead) 
          {
           // Default value
           Set_holdTime = lastSetHoldTime;  //set the hold time to 500 ms as default
           displayHoldTimerDefIMVoff(); // this displays the 500 ms
          } 
        else if (holdTimeSwitchRead) 
          {
           Set_holdTime = map (Slider_HoldTime, 0, 180, 0, 1000);
           Set_holdTime = constrain (Set_holdTime, 0 , 1000);
           lastSetHoldTime = Set_holdTime;  //this will hold the value if you select hold time aside from default
           displayHoldTimerIMVOff(); // this displays the hold time you selected
          }
     }
    displayUpdateTime = millis(); 
  }

  readSliders();
  delay(5); //this is needed in order to perform the sliders
  checkSwitchState();
  angle = encoder();


  if (!IMVswitchRead) //when the IMV switch is off
      {
        readSliders();
        checkSwitchState();
        digitalWrite(36, HIGH);// turn relay OFF
        PEEP = 0;
        plateau = 0;
      }

  if (IMVswitchRead) //IMV switch is on
    {
      digitalWrite(36, LOW);// turn relay ON     
      checkSwitchState();     

      // Initialize first-run variables
     if(firstRun)
     {
      lastSpeed = initialSpeed; // give an initial speed for motor during the first start to capture the the firt inhale duration. This is important for scalingFactor.
      lastBreathTime = millis(); // Initialize time tracking
      // Set initial motor speed or scaling factor here
      scalingFactor = 1.0; // Initialize with 1.0 or another value as appropriate
      firstRun = false;      // Set to false after initializing
     }

    // Calculate the total cycle time (in milliseconds) based on desired BPM
    float totalCycleTime = 60000 / BPMvalue;
  
    // Calculate the ideal inhale time based on I:E of 1:2
    float idealInhaleTime = totalCycleTime/3 ;

  if (millis() - lastBreathTime >= cycleDuration) // this serves as a check to update the scalingFactor
     {
      lastBreathTime = millis();
      // Calculate a scaling factor based on the difference between ideal and actual inhale times
      if (Initial_Inhale_Duration != 0) //when Initial_Inhale_Duration is no longer zero (usually during the first run)
        {          
          if (idealInhaleTime < (Initial_Inhale_Duration + Set_holdTime)) 
            {
              scalingFactor = (Initial_Inhale_Duration + Set_holdTime) /idealInhaleTime;
            }
          else 
            {
              scalingFactor = (Initial_Inhale_Duration + Set_holdTime )/idealInhaleTime;
            }
              int motorSpeed = lastSpeed * scalingFactor;  // Updates the motor speed until scaling factor equals 1 after several cycles
              lastSpeed = motorSpeed; // lastSpeed will be used for the motor speed
              lastSpeed = constrain(lastSpeed, 25, 250); //constrain to minimum and maximum speed of motor to avoid damage
        }   
      else // usually happens during first run
        {
           scalingFactor = 1.0; // Reset to 1 if actualInhaleTime has not yet been measured
        }
 
      // Reset the time taken to reach the target pressureValue
      Initial_Inhale_Duration = 0;
    }

    if (inHome == 0)
      {
        readSliders();
        checkSwitchState();
        setpointAngle = 90;  //always set the angle to 90 degrees as a default home position
        volume = 0;
      }

    peak_Pressure = 0; // ensuring it is zero before the new reading

    if (inHome == 1)
       {
         readSliders();
         checkSwitchState();
         pressureValue = pressureSensor();
          if ( pressureValue > peak_Pressure )     //finding the peak pressureValue (PIP)
            {
             peak_Pressure = pressureValue;
            }         
        }

      if (inHome == 0) //ensuring that assist control feature is available throughout the duration of inHome == 0 / exhale phase
      {
        bool assistSwitchState = assistSwitchRead;

        if (assistSwitchState == true) 
        {
         
         float previousPressureValue = pressureValue; // store the previous value
         previousPressureValue = movingaverage(previousPressureValue);
         pressureValue = pressureSensor(); // read the new current value
         pressureValue = movingaverage(pressureValue);
         int invertedSensitivity = 5 - sensitivityValue; // Here sensitivityValue is what you read from your slider
                                                         // Lower slider value -> higher invertedSensitivity -> less aggressive.
                                                         // Higher slider value -> lower invertedSensitivity -> more aggressive.
          if (pressureValue <= previousPressureValue - 5 + invertedSensitivity)
              {
               Serial.println("PATIENT BREATH TRIGGERED");
               inHome = 1;
               start_time_inhale = millis();
               return;  
              }
        } 
      }
//=================================================================================================
// this section is for motor control
    angle = encoder();
    
   if (inHome == 1)
    {
      PIDcalculation();
      readSliders();
      checkSwitchState();
    }

   if (inHome == 0)
      {
       PIDcalculationR();
       readSliders();
       checkSwitchState();
      }

    if (error < 0)
    {
      digitalWrite(dir1, LOW);
      analogWrite(pwm1, pidTerm_scaled);
    }
     else
    {
      digitalWrite(dir1, HIGH);
      analogWrite(pwm1, lastSpeed);
    }

//============================================================================================
//this section is the the switching of motor state

    if ((inHome == 1 && (pressureValue >= setpointPressure)) || (inHome == 1 && angle >= 195)) //this block will execute when it hits the target pressureValue or reaches the hard limit angle
      {
        readSliders();
        checkSwitchState();

        finalVolume = m * angle + b; // calculate volume at the peak
        displayVolumeFunc(finalVolume); // display volume at the peak
        analogWrite (pwm1, 0); // stopping the motor for the duration of the execution of this block which depends on how much is the desired hold time
        inHome = 0; // flipping the inHome state to zero, signaling that it should go home after executing this if-statement
          
        float end_time_inhale = millis();  //this is the time marker at the time it hits the condition above (setpoint)
        Initial_Inhale_Duration = end_time_inhale - start_time_inhale; 

        start_time_for_hold = millis(); //a time marker for hold time
        actual_holdTime = 0; // making sure it is zero
           if (!holdTimeSwitchRead) //this switch sets the default setting for hold time
               {
                Set_holdTime = lastSetHoldTime; // Default value
                displayHoldTimerDef();
               } 
           else if (holdTimeSwitchRead) // this swtich makes an adjustment to hold time (0-1000 ms)
               {
                Set_holdTime = map (Slider_HoldTime, 0, 180, 0, 1000);
                Set_holdTime = constrain (Set_holdTime, 0 , 1000);
                lastSetHoldTime = Set_holdTime;
                displayHoldTimer();
              }
                 
        plateau = 0.0;
        count = 0;
        while ( actual_holdTime < Set_holdTime) //hold period, measuring plateau pressureValue
            {
              readSliders();
              checkSwitchState();
              pressureValue = pressureSensor();
              plateau += pressureValue;              // plateau pressureValue diagnostic
              actual_holdTime = millis();
              actual_holdTime = actual_holdTime - start_time_for_hold;
              count += 1;
            }

          Total_Inhale_Duration = Initial_Inhale_Duration + actual_holdTime;   //this is the total inhale duration that includes that hold time
          plateau = (plateau / count); //calculating the final plateau pressureValue
          start_time_exhale = millis()- time_off_duration; // this marks the time as a starting point in going to home position since we set the inHome = 0 above
      }

      else if (inHome == 0 && (encoderAngle >= 0.95 * setpointAngle && encoderAngle <= 1.05 * setpointAngle)) // it will be executed after it hits the default 90 degree home position
      {
        readSliders();
        checkSwitchState();
        analogWrite (pwm1, 0); // stopping the motor for the duration of the execution of this block which depends on how much is the remaining relax time
        inHome = 1;  // flipping the inHome state to 1, signaling that it should go to the setpoint after executing this if-statement
        end_time_exhale = millis(); //this is the time marker at the time it hits the condition above (setpoint)
        Total_Exhale_Duration = (end_time_exhale - start_time_exhale); 

        if (!IEControlSwitchRead) //this switch sets the default setting for I:E control
          {
           // Maintain a 1:2 ratio
           cycleDuration = 3 * Total_Inhale_Duration;
          } 
        else if (IEControlSwitchRead)  // this swtich makes an adjustment to the desired cycle duration 
          {
           // Use IEslider to set cycleDuration
           int IEslider;
           cycleDuration = map(IEslider, 0, 180, 2000, 11900);        
           cycleDuration = (int) constrain(cycleDuration, 2400, 12000);
          }

        relaxTime = cycleDuration - Total_Inhale_Duration - Total_Exhale_Duration; 

          if (relaxTime < 0)
            {
              relaxTime = 100;    // this time is set for times when total cycle time exceeds the set time.
              //this usually happens at the start up.
            }
            
          start_time_marker = millis(); //this is a time marker for relax time
          elapsed_time = 0;
          PEEP = 0;
          PEEP2 = 0;
          count = 0;

         bool assistSwitchState = assistSwitchRead;
              
           while (elapsed_time < (relaxTime * 0.25))  // measuring the PEEP during the 25% of the relax time & still ensuring that assist control feature is accesible
              {
               float previousPressureValue = pressureValue; // store the previous value
               pressureValue = pressureSensor();
               int invertedSensitivity = 5 - sensitivityValue; // Here sensitivityValue is what you read from your slider
                                                               // Lower slider value -> higher invertedSensitivity -> less aggressive.
                                                               // Higher slider value -> lower invertedSensitivity -> more aggressive.
               stop_time_marker = millis();
               elapsed_time = stop_time_marker - start_time_marker;
               count += 1;
               PEEP2 += pressureValue;
               readSliders();
               checkSwitchState();    
               if (assistSwitchState == true) // if the assist switch is turned on
                 {
                  if (pressureValue <= previousPressureValue - 5 + invertedSensitivity)
                    {
                      Serial.println("PATIENT BREATH TRIGGEREED");
                      start_time_inhale = millis();
                      break;  // if true, it will cut the remaining relax time and ending the execution of this block and proceed to inHome == 1
                    }      
                 }             
              }

          PEEP = (PEEP2 / count); //calculate the final PEEP

            if (assistSwitchState == true) // if the assist switch is turned on
              {
                readSliders();
                checkSwitchState();

                while (elapsed_time <= relaxTime) // the relax time is also allotted to wait for patient's effort
                    {                 
                      float previousPressureValue = pressureValue; // store the previous value
                      pressureValue = pressureSensor(); // read the new current value
                      int invertedSensitivity = 5 - sensitivityValue; // Here sensitivityValue is what you read from your slider
                                                                       // Lower slider value -> higher invertedSensitivity -> less aggressive.
                                                                      // Higher slider value -> lower invertedSensitivity -> more aggressive.
                      stop_time_marker = millis();
                      elapsed_time = stop_time_marker - start_time_marker;
  
                      if (pressureValue <= previousPressureValue - 5 + invertedSensitivity)
                        {
                          Serial.println("PATIENT BREATH TRIGGEREED");
                          start_time_inhale = millis();
                          break;  // if true, it will cut the remaining relax time and ending the execution of this block and proceed to inHome == 1
                        }
                    }
              }

            else if (assistSwitchState == false) // if the assist switch is turned off
              {
                readSliders();
                checkSwitchState(); 

                while (elapsed_time < (relaxTime * 0.75))   //  full 75% expiratory time wait 
                  {
                    pressureValue = pressureSensor();
                    stop_time_marker = millis();
                    elapsed_time = stop_time_marker - start_time_marker;
                    readSliders();
                    checkSwitchState();
                  }  
              }
              readSliders();
              checkSwitchState();
              start_time_marker = millis();
              elapsed_time = 0;
              start_time_inhale = millis() - time_off_duration;
            }

    else
      {
        readSliders();
        checkSwitchState();
      }

    } // End of IMV Switch ON

   else 
    {
    plateau = 0;
    PEEP =0;
    analogWrite (pwm1, 0);
    } 
} // End of void loop

void PIDcalculationR()
{
  error = setpointAngle - encoderAngle;
  changeError = error - last_error;                                       // derivative term
  totalError += error;                                                   //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);      //total gain
  pidTerm = (int) constrain(pidTerm, -230, 230);                              //constraining to appropriate value
  if (pidTerm > 0)
  {
    pidTerm_scaled = pidTerm;
  }
  if (pidTerm < 0) {                                         //make sure it's a positive value
    pidTerm_scaled = -1 * pidTerm;
  }
    last_error = error;
}

void PIDcalculation()
{

  error = setpointPressure - pressureValue;
  //Turned off since the speed of motor now is determined outside in the scaling factor block
  /*changeError = error - last_error;                                  // derivative term
  totalError += error;                                               //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);   //total gain
  pidTerm = (int) constrain(pidTerm, -limitSpeed, limitSpeed);       //constraining to appropriate value
  if (pidTerm > 0)
  {
    pidTerm_scaled = pidTerm;
  }
  if (pidTerm < 0) {                                         //make sure it's a positive value
    pidTerm_scaled = -1 * pidTerm;
    last_error = error;
  }*/

}

int encoder()
{
  int16_t dataOut;  // 16-bit variable to hold the ADC reading        
  dataOut = ADS.readADC(3);  //analog pin for encoder angle is connected at A3 on ADC
  encoderAngle = map(dataOut, 0, 26000, 0, 359);  // Remap the 16-bit reading to 0-359
  return encoderAngle;
}

float pressureSensor()
{
  int16_t adc0 = ads.readADC_SingleEnded(0); //analog pin for pressure sensor is connected at A0 on ADC
  diffPressure = adc0;
  float voltage = (diffPressure / 26000) * 5.0;
  pressureKPA = ((voltage / 5) - 0.5) / 0.2;  // in kPa
  signal1 = pressureKPA*1000; // to Pa
  signal1 = signal1 - 56; //observes offset and implement
  signal1 = signal1/1000; //back to kPa
  pressure_cmH2O = signal1 * 10.1972; //converts to cmH2O

  return pressure_cmH2O;
}

// for moving average
float movingaverage(float input)
{
    float accel;
    accel =  input;
    sum1 = sum1 - READINGS[indexx];
    READINGS[indexx]= accel;
    sum1 = sum1 + accel;
    indexx = (indexx+1) % windowsize;
    averaged = sum1 / windowsize;
  return(averaged);
}

void checkSwitchState() 
{
  if (Serial2.available()) 
  {
    for (int i = 0; i <= 8; i++) 
    {
      Buffer[i] = Serial2.read();
    }

    if (Buffer[0] == 0x5A) 
    {
      switch (Buffer[4]) 
      {
        case 0x61:  // IMV switch
          // Only process this state change if enough time has passed since the last one
          if (millis() - lastStateChangeTime >= debouncePeriod) 
           {
            if (Buffer[8] == 0)
             {
              IMVswitchRead = false; // Update switch state to off
              lastStateChangeTime = millis();
             } 
            else if (Buffer[8] == 1)
             {
              IMVswitchRead = true; // Update switch state to on
              lastStateChangeTime = millis();
             }
            // Update the last state change time
           }
        break;

        case 0x62:  // Assist switch
          // Only process this state change if enough time has passed since the last one
          if (millis() - lastStateChangeTime >= debouncePeriod) 
           {
            if (Buffer[8] == 0)
             {
              assistSwitchRead = false; // Update switch state to off
              lastStateChangeTime = millis();
             } 
            else if (Buffer[8] == 1)
             {
              assistSwitchRead = true; // Update switch state to on
              lastStateChangeTime = millis();
             }
            // Update the last state change time
           }
        break;

        case 0x73:  // IE control switch
          // Only process this state change if enough time has passed since the last one
          if (millis() - lastStateChangeTime >= debouncePeriod) 
           {
            if (Buffer[8] == 0)
             {
              IEControlSwitchRead = false; // Update switch state to off
              lastStateChangeTime = millis();
             } 
            else if (Buffer[8] == 1)
             {
              IEControlSwitchRead = true; // Update switch state to on
              lastStateChangeTime = millis();
             }
            // Update the last state change time
           }
        break;

         case 0x74:  // Hold Time switch
          // Only process this state change if enough time has passed since the last one
          if (millis() - lastStateChangeTime >= debouncePeriod) 
           {
            if (Buffer[8] == 0) 
             {
              holdTimeSwitchRead = false; // Update switch state to off
              lastStateChangeTime = millis();
             } 
            else if (Buffer[8] == 1) 
             {
              holdTimeSwitchRead = true; // Update switch state to on
              lastStateChangeTime = millis();
             }
          }
        break;
      }
    }
  }
}

void readSliders() 
{
  if (Buffer[0] == 0x5A) 
    {
      switch (Buffer[4]) 
        {
          case 0x51:  // pressureValue slider
          pressureSlider = Buffer[8];        
          setpointPressure = map(pressureSlider, 0, 180, pressureMin, pressureMax);
          setpointPressure = (int) constrain(setpointPressure, 5, 25);
          displayPressureFunc();
          break;
          
          case 0x53:  // RR slider
          BPMslider = Buffer[8];      
          displayRespiratoryRate();
          break;

          case 0x55:  // IE slider
          int IEslider;
          IEslider = Buffer[8];            
          cycleDuration = map(IEslider, 0, 180, 2000, 11900);        
          cycleDuration = (int) constrain(cycleDuration, 2400, 12000);
          displayInhaleTime_for_IE();
          displayExhaleTime_for_IE();
          break;

          case 0x57:  // AS slider       
          int sensitivity_slider;
          sensitivity_slider = Buffer[8];       
          sensitivityValue = map(sensitivity_slider, 0, 180, -0.8, 5.2);    
          sensitivityValue = constrain(sensitivityValue, 0, 5);
          displaySNTVTYFunc(sensitivity_slider);
          break;

          case 0x59:  // HOLD slider
          Slider_HoldTime = Buffer[8];   
          displayHoldTimer();
          break;
        } 
    }
}

void displayPressureFunc()
{
  float  mappedPressure;
  mappedPressure = map (pressureSlider, 0, 180, pressureMin, pressureMax);
  mappedPressure = (float) constrain(mappedPressure, 5, 25);

  Serial2.write(0x5A);  // Header
  Serial2.write(0xA5);  // Header
  Serial2.write(0x07);  // Length: VP address + write command + length of the int (2 bytes)
  Serial2.write(0x82);  // Write command
  Serial2.write(0x52);  // Write address
  Serial2.write((byte) 0x00); // Write address

  byte hex[4] = {0};
  FloatToHexPressure(mappedPressure, hex);

  Serial2.write(hex[3]);
  Serial2.write(hex[2]);
  Serial2.write(hex[1]);
  Serial2.write(hex[0]);     
   
}
void FloatToHexPressure(float f, byte* hex)
{
  byte* f_byte = reinterpret_cast<byte*>(&f);
  memcpy(hex, f_byte, 4);
}

void displayVolumeFunc(int volume)
{
  volume = constrain(volume, 0, 600); // Ensuring volume stays within 200-600ml range from original MOSEV system (volume based)

  displayVolume[6] = highByte(volume);
  displayVolume[7] = lowByte(volume);
  Serial2.write(displayVolume, 8);
}

void displayVolume_Default()
{
  static unsigned long previousMillis = 0;
  unsigned long interval = 200; // set the interval at which the display is updated
  unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;
      int volume;
      volume = 0;

      displayVolume[6] = highByte(volume);
      displayVolume[7] = lowByte(volume);
      Serial2.write(displayVolumeDefault, 8);
    }
}

void displayRespiratoryRate()
{
  int desiredTotalCycleDuration;
  int displayBPM;
  desiredTotalCycleDuration = map(BPMslider, 0, 180, 12000, 2400);  
  desiredTotalCycleDuration = (int) constrain(desiredTotalCycleDuration, 2400, 12000);  
  float unscaledBPM = 60000 / desiredTotalCycleDuration;

  BPMvalue =  (int) unscaledBPM - 2; //the "2" here is an offset based on actual observation/stopwatch.
  displayBPM = (int) unscaledBPM;  // we display still the desired BPM 

  displayBreathsPerMinute[6] = highByte(displayBPM);
  displayBreathsPerMinute[7] = lowByte(displayBPM);
  Serial2.write(displayBreathsPerMinute, 8);
}

void displaySNTVTYFunc(int sensitivity_slider, bool useDefault)
{
  int mappedSNTVTY;

  if (useDefault) 
  {
    // Here, set mappedSNTVTY to whatever value corresponds to a sensitivity of 1 on your display
    mappedSNTVTY = 1;  
  } 
  else 
  {
    mappedSNTVTY = map(sensitivity_slider, 0, 180, 0, 5);
    mappedSNTVTY = constrain(mappedSNTVTY, 0, 5);
  }

  displaySensitivity[6] = highByte(mappedSNTVTY);
  displaySensitivity[7] = lowByte(mappedSNTVTY);
  Serial2.write(displaySensitivity, 8);
}

void displayHoldTimer()
{
   int mappedHOLDTIME;
   mappedHOLDTIME = map(Slider_HoldTime, 0, 180, 0, 1000);
    
   displayHOLDTIME[6] = highByte(mappedHOLDTIME);
   displayHOLDTIME[7] = lowByte(mappedHOLDTIME);
   Serial2.write(displayHOLDTIME, 8);
}

void displayHoldTimerIMVOff()
{
   static unsigned long previousMillis = 0;
   unsigned long interval = 200; // set the interval at which the display is updated
   unsigned long currentMillis = millis();
 
    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;
      int mappedHOLDTIME;
      mappedHOLDTIME = map(Slider_HoldTime, 0, 180, 0, 1000);
        
      displayHOLDTIME[6] = highByte(mappedHOLDTIME);
      displayHOLDTIME[7] = lowByte(mappedHOLDTIME);
      Serial2.write(displayHOLDTIME, 8);
    }
}

void displayHoldTimerDef()
{
  int mappedHOLDTIME;
  mappedHOLDTIME = 500;
  
  displayHOLDTIME[6] = highByte(mappedHOLDTIME);
  displayHOLDTIME[7] = lowByte(mappedHOLDTIME);
  Serial2.write(displayHOLDTIME, 8);
}

void displayHoldTimerDefIMVoff()
{
  static unsigned long previousMillis = 0;
  unsigned long interval = 200; // set the interval at which the display is updated
  unsigned long currentMillis = millis();
 
    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;
      int mappedHOLDTIME;
      mappedHOLDTIME = 500;
  
      displayHOLDTIME[6] = highByte(mappedHOLDTIME);
      displayHOLDTIME[7] = lowByte(mappedHOLDTIME);
      Serial2.write(displayHOLDTIME, 8);
    }
}

void displayInhaleTime_for_IE()
{
  int I =1;
  display_I[6] = highByte(I);
  display_I[7] = lowByte(I);
  Serial2.write(display_I, 8);
}

void displayExhaleTime_for_IE()
{
  float E;
  E = (cycleDuration - Total_Inhale_Duration) / Total_Inhale_Duration;

  Serial2.write(0x5A);  // Header
  Serial2.write(0xA5);  // Header
  Serial2.write(0x07);  // Length: VP address + write command + length of the int (2 bytes)
  Serial2.write(0x82);  // Write command
  Serial2.write(0x63);  // Write address
  Serial2.write((byte) 0x00); // Write address

  byte hex[4] = {0};
  FloatToHexE(E, hex);

  Serial2.write(hex[3]);
  Serial2.write(hex[2]);
  Serial2.write(hex[1]);
  Serial2.write(hex[0]);        
}

void FloatToHexE(float f, byte* hex)
{
  byte* f_byte = reinterpret_cast<byte*>(&f);
  memcpy(hex, f_byte, 4);
}

void displayExhaleTime_for_IE_Default()
{
  static unsigned long previousMillis = 0;
  unsigned long interval = 200; // set the interval at which the display is updated
  unsigned long currentMillis = millis();
 
    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;
      float E;
      E = 2.0;

      Serial2.write(0x5A);  // Header
      Serial2.write(0xA5);  // Header
      Serial2.write(0x07);  // Length: VP address + write command + length of the int (2 bytes)
      Serial2.write(0x82);  // Write command
      Serial2.write(0x63);  // Write address
      Serial2.write((byte) 0x00); // Write address

      byte hex[4] = {0};
      FloatToHexEDef(E, hex);

      Serial2.write(hex[3]);
      Serial2.write(hex[2]);
      Serial2.write(hex[1]);
      Serial2.write(hex[0]);  

    }      
}

void FloatToHexEDef(float f, byte* hex)
{
  byte* f_byte = reinterpret_cast<byte*>(&f);
  memcpy(hex, f_byte, 4);
}

void displayPEEPvalue(int PEEP)
{
  displayPEEP[6] = highByte(PEEP);
  displayPEEP[7] = lowByte(PEEP);
  Serial2.write(displayPEEP, 8); 
}
 
void displayPIPvalue(int peak_Pressure)
{
  displayPIP[6] = highByte(peak_Pressure);
  displayPIP[7] = lowByte(peak_Pressure);
  Serial2.write(displayPIP, 8);  
}

void displayPIPvalueIMVOFF()
{
  static unsigned long previousMillis = 0;
  unsigned long interval = 200; // set the interval at which the display is updated
  unsigned long currentMillis = millis();
 
    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;
      peak_Pressure = 0;
  
      displayPIP[6] = highByte(peak_Pressure);
      displayPIP[7] = lowByte(peak_Pressure);
      Serial2.write(displayPIP, 8);  
    }
}

void displayPlateauValue(int plateau)
{
  displayPlateau[6] = highByte(plateau);
  displayPlateau[7] = lowByte(plateau);
  Serial2.write(displayPlateau, 8);   
}

void updateDisplayValues() 
{
    static unsigned long previousMillis = 0;
    unsigned long interval = 200; // set the interval at which the display is updated
    unsigned long currentMillis = millis();
 
    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;

      // update display values
      displayPEEPvalue(PEEP);
      displayPlateauValue(plateau);
      displayInhaleTime_for_IE();
      displayExhaleTime_for_IE();
    }
}