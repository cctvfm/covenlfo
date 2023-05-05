//V1.1
//March 30 2023
//Implemented changes suggested by ryokell
//Out of order divs
//Non-1 first div

#include <FlashAsEEPROM_SAMD.h>
#include <avr/pgmspace.h>
#include "antilog.h"
#include <Arduino.h>

#define TRIANGLE 1
#define SAW 2
#define SQUARE 3
#define RANDOM 4 

#define FREQ 0
#define POT 1


#define HZPHASOR 91183 //phasor value for 1 hz.


long unsigned int accumulator1 = 0;
long unsigned int accumulator2 = 0;
long unsigned int accumulator3 = 0;
long unsigned int accumulator4 = 0;
long unsigned int phasor1;
long unsigned int phasor2;
long unsigned int phasor3;
long unsigned int phasor4;

char randNum[4];

FlashStorage(div_storage, int);
FlashStorage(wave_storage, int);
FlashStorage(init_storage, char);

////////////////////////////////////////////////////////////////////////////////////
//       DIVIDE DOWN ARRAYS                                                       //
//       Add more if you want!                                                    //
////////////////////////////////////////////////////////////////////////////////////

#define DIVSIZE 3 // if you add more divide down arrays, increase this number from 3 (the default number of arrays) to how many arrays you have total  
char divs[DIVSIZE][4]={{1,3,7,11},
                      {1,2,4,8},
                      {1,4,8,16}}; // You could add more divide down arrays here 

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

char debounceState = 0;
unsigned long int debounceTime = 0; 
int waveSelect = 1;
int divSelect = 1;
unsigned long lastSettingsSave = 0;

bool Mode = 0; // 0 = POT and 1 = SYNC 
float sweepValue;
long unsigned int Time1 = 0;
long unsigned int Time2 = 0; 
long unsigned int Periud = 0; // Period (arduino didn't allow use of word "period")
float syncFrequency = 0; 
void timerIsr();
void setupTimers();
void TCC0_Handler();



// +++++++++++++++++++++++++++++++++++ SETUP ++++++++++++++++++++++++++++++++++++++++
void setup() {
  pinMode(1, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT); // using pin 13 to check interupt on timer 1
  pinMode(6, INPUT_PULLUP); // pin 7 pushbutton to select waveform
  pinMode(A10, INPUT); //used for analogReading sync (cv2) 
  readSettings();
  //Serial.begin(9600);
  setupTimers(); //  **this may not be the right location
  randomSeed(analogRead(A8));
  
   
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++ MAIN LOOP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void loop() {
  
  static int modeCounter = 0;
  bool lastButtonState = 0;
  static bool buttonState = 0;
  static int syncState = 0; // used to find find the leading edge to calculate period (static so it isn't updated to zero each loop)
  static int syncCounter = 0;
  int Sync; 

////////////////////////////////////////////////////////////////////////////////////
//       "EEPROM" Save                                                            //
//       Store the settings after 60 seconds                                      //
////////////////////////////////////////////////////////////////////////////////////
  if((millis() - (lastSettingsSave)) > (60000))
  {
    saveSettings();
    lastSettingsSave = millis();
  }
  
////////////////////////////////////////////////////////////////////////////////////
//        Sync mode                                                               //
//       sync the 1:1 output with an incoming signal                              //
////////////////////////////////////////////////////////////////////////////////////
     Sync = analogRead(A10); // CV2 pin A10 on XIAO 
  
    if((Sync < 370) && (syncState == 0)){ //if analog read CV2 is less than 370 RISING EDGE
      Time1 = micros(); // Capture the time 
      syncState = 1;  // set syncState to 1 
      //This only enters on the first pulse received,
      //following pulses alternate between syncstate 2 and 3
      
    }

    else if((Sync > 370) && (syncState == 1)){ //FALLING
     
      syncState = 2; 
    
     }

    else if((Sync < 370) && (syncState == 2)){ //NEXT RISING EDGE
      syncState = 3;
      Time2 = micros();
      Mode = 1;
      Periud = (Time2 - Time1); // Find the period by subtracting Time2 from Time1 
      syncFrequency = 1000000;
      syncFrequency = syncFrequency/Periud; // NEW FREQUENCY HERE 
      
      if(divs[divSelect-1][0] == 1)
      {
        //if the first divider is 1, clear acc 1 every rising edge to keep synced
        accumulator1=0;
      }
      if(divs[divSelect-1][1] == 1)
      {
        //if the second divider is 1, clear acc 2
        accumulator2=0;
      }
      if(divs[divSelect-1][2] == 1)
      {
        //if the third divider is 1, clear acc 3
        accumulator3=0;
      }
      if(divs[divSelect-1][3] == 1)
      {
        //if the fourth divider is 1, clear acc 4
        accumulator4=0;
      }
      
      Time1 = Time2;
      
      
      if(syncCounter == 0)  //first time calculation, clear all accs so waveform starts at 0 and begins synced
      {
        accumulator1=0;
        accumulator2=0;  
        accumulator3=0;
        accumulator4=0;
      }

      if(syncCounter%divs[divSelect-1][0] == 0){ // div 1
        accumulator1 = 0;
      }

      if(syncCounter%divs[divSelect-1][1] == 0){ // div 2
        accumulator2 = 0;
      }

      if(syncCounter%divs[divSelect-1][2] == 0){ // div 3
        accumulator3 = 0;
      }

      if(syncCounter%divs[divSelect-1][3] ==0){ // div 4
        accumulator4 = 0;
      }
    

      if((syncCounter%divs[divSelect-1][0] == 0) && (syncCounter%divs[divSelect-1][1] == 0) && (syncCounter%divs[divSelect-1][2] == 0) && (syncCounter%divs[divSelect-1][3] == 0)){
        syncCounter = 0;
        
      }
      syncCounter++;

    }
    
    else if((Sync > 370) && (syncState == 3)){ // last falling edge
     syncState = 2;
     
     }   

   ////////////////////////////////////////////////////////////
   //                END OF SYNC STUFF                       //  
   ////////////////////////////////////////////////////////////
     
   if(digitalRead(6)==LOW && debounceState ==0){ //button has been pressed and hasn't been previously pressed                           
      debounceState = 1;   //real button press?
      debounceTime = millis();
   }

   else if(debounceState == 1){
    if((millis() - debounceTime) > 80){
        if(digitalRead(6) == LOW){
          debounceState = 2;
        }
        else{
          debounceState = 0;
        }
      }
  }

   else if(debounceState == 2){
      if(digitalRead(6) == HIGH){
          debounceState = 0;
          if((millis() - debounceTime) < 3000){ //short press to change waveform
           waveSelect++;  //move to the next waveform
              if(waveSelect>4){ //if we're trying to select more than 4 waveforms, cycle back to the 1st
                 waveSelect=1;
              }
          }
      }
        else if((millis() - debounceTime) > 3000)  //long press to change divisions
        {
          debounceState = 3;
          debounceTime=millis();
          accumulator1=0; 
          accumulator2=0; 
          accumulator3=0; 
          accumulator4=0;
          phasor1 = 0;
          phasor2 = 0;
          phasor3 = 0;
          phasor4 = 0;
          delay(2000);
          
          divSelect++;
            if(divSelect>DIVSIZE){
               divSelect=1;
            }

        }
      
   }
  else if(debounceState == 3){  //holding only
    if((millis() - debounceTime) > 3000)  //long press
        {
          debounceState = 3;
          debounceTime=millis();
          divSelect++;
            if(divSelect>3){
               divSelect=1;
            }

        }
      else if(digitalRead(6) == HIGH)
      {
          debounceState = 0;
      }
  }
  
   
   
  float tempphasor;
  int cv1Value; // to store value of cv1 (Frequency)
  static int potValue; // to store the value of the potentiometer
  
  static int oldpotValue; 
  filterPut(POT,analogRead(A0));
  potValue = filterGet(POT);
  //Serial.println(potValue);

  //This section pops us out of sync mode
  modeCounter++;
  if(modeCounter>100) //do this only every 100 samples
  {
    if((potValue - oldpotValue)> 20 || (oldpotValue - potValue) > 20){
    Mode = 0; 
    }

    oldpotValue = potValue;

    modeCounter = 0;
  }

  filterPut(FREQ,analogRead(A4));
  cv1Value = filterGet(FREQ);; // at this stage, a -12V CV corresponds to +3.3V (1023 as an analog read) on the XIAO (Because of the inverting op-amp)
  
  cv1Value = 1023-cv1Value; // so we want to invert it (making -12V correspond to 0V on the XIAO)
  cv1Value = cv1Value - 565;  //at this point cv1Value contains between -512 and +511 (this line used to center values around zero)
  
  if(Mode == 1){ //better way to update sweepvalue with sync frequency
  sweepValue = syncFrequency;
  }
  else
  {
      if( (Mode == 0) && ( (cv1Value > 20) || (cv1Value < -20) )  ){// Don't want the frequency to be considered if the cv input is close to 0 (+/- 20) 
        int totalcv = potValue+cv1Value;
        if(totalcv <0)
          totalcv = 0;
        else if(totalcv>1023)
          totalcv = 1023;
          
        sweepValue=pgm_read_float_near(hzcurve + totalcv);
      }
      else
        sweepValue=pgm_read_float_near(hzcurve + potValue);

  }




  tempphasor=sweepValue*HZPHASOR;


 
  phasor1=(unsigned long int)tempphasor/divs[divSelect-1][0];
  phasor2=(unsigned long int)tempphasor/divs[divSelect-1][1]; // dividing down for the slower outputs 
  phasor3=(unsigned long int)tempphasor/divs[divSelect-1][2];
  phasor4=(unsigned long int)tempphasor/divs[divSelect-1][3];

}

// +++++++++++++++++++++++++++++++++++++++++ FUNCTION DEFINITIONS ++++++++++++++++++++++++++++++++++

unsigned long int previous_acc[4];

unsigned char generator(unsigned long int acc, char waveshape, char channel) {

unsigned char shifted_acc = acc>>24;


  if (waveshape == SQUARE) {
    if ((shifted_acc)>127) {
      return 255;
    }
    else
    {
      return 0;
    }
  }

  if (waveshape == SAW) {
   return(shifted_acc);
  }

  if (waveshape == TRIANGLE){
    if ((shifted_acc)<127)
      {
        return ((shifted_acc)<<1);
      }
    else if (shifted_acc == 127){
      return 255;  
    }
    else //greater than 127 (128-255)
    {
      return ((255-shifted_acc)<<1);
    }
  }
  // WANT TO IMPLEMENT RANDOM SQUARE WAVESHAPE HERE 

   if(waveshape == RANDOM) {
    
      if(shifted_acc < previous_acc[channel] ) // LESS THAN PREVIOUS NOT SURE HOW TO IMPLEMENT THIS 
      {
        randNum[channel] = random(255);
        
      }
    
      previous_acc[channel] = shifted_acc;  
      return randNum[channel]; 
 
   }

 
     
}

void setupTimers() // used to set up fast PWM on pins 1,9,2,3
{
 
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(2) |          //// Divide the 48MHz clock source by divisor N=1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            //// Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          //// Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  //enable our 4 pins to be PWM outputs
  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;

  //assign the 4 outputs to the PWM registers on PMUX
  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg = PORT_PMUX_PMUXE_E;  // D3 is on PA11 = odd      
  PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E; // D11 is on PA08 = even 
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;  // D3 is on PA11 = odd
  PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F; // D11 is on PA08 = even

 

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK4 //0 only works for interrup?
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |           // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH |
                    TCC_WAVE_WAVEGEN_NFRQ;     // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: Freq = 48Mhz/(2*N*PER)
  REG_TCC0_PER = 0xFF;                           // Set the FreqTcc and period of the PWM on TCC1
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization
 
  REG_TCC0_CC1 = 10;                             // TCC1 CC1 - on D3  50% pin 9
  while (TCC0->SYNCBUSY.bit.CC1);                   // Wait for synchronization
  REG_TCC0_CC0 = 50;                             // TCC1 CC0 - on D11 50% pin 1
  while (TCC0->SYNCBUSY.bit.CC0);                   // Wait for synchronization
    REG_TCC0_CC2 = 200;                             // TCC1 CC1 - on D3  50% pin 2
  while (TCC0->SYNCBUSY.bit.CC2);                   // Wait for synchronization
  REG_TCC0_CC3 = 254;                             // TCC1 CC0 - on D11 50% pin 3
  while (TCC0->SYNCBUSY.bit.CC3);                   // Wait for synchronization
 
  // Divide the GCLOCK signal by 1 giving  in this case 48MHz (20.83ns) TCC1 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV2 |    // Divide GCLK4 by 1 ****************************************************************************
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  TCC0->INTENSET.reg = 0;
  TCC0->INTENSET.bit.CNT = 1;  //*****************************************************
  TCC0->INTENSET.bit.MC0 = 0;

  NVIC_EnableIRQ(TCC0_IRQn);
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE ;
  
}

void TCC0_Handler() 
{
  if (TCC0->INTFLAG.bit.CNT == 1) { //*************************************************
   accumulator1 = accumulator1 + phasor1;
   accumulator2 = accumulator2 + phasor2;
   accumulator3 = accumulator3 + phasor3;
   accumulator4 = accumulator4 + phasor4;
   delayMicroseconds(6);
   REG_TCC0_CC0 = generator(accumulator1, waveSelect,3); // pin 9 //#4
   REG_TCC0_CC1 = generator(accumulator4, waveSelect,0); // pin 2 //#1
   REG_TCC0_CC2 = generator(accumulator2, waveSelect,1); // pin 1 //#2  
   REG_TCC0_CC3 = generator(accumulator3, waveSelect,2); // pin 3 //#3

   TCC0->INTFLAG.bit.CNT = 1; //*******************************************************
  }
}


void readSettings (void)
{
  char x;
  char c = 'S';
  int y = 1;
  init_storage.read(x);
  if(x == 'S')  //S means eeprom has been initialized.
  {
    wave_storage.read(waveSelect);
    div_storage.read(divSelect);
  }
  else
  {
    //we initialize, no 'S' found
    init_storage.write(c);  //use variables because this library hates constants
    wave_storage.write(y);
    div_storage.write(y);
    divSelect = 1;
    waveSelect = 1;
  }

}

void saveSettings (void)
{
  int x;
  wave_storage.read(x);
  
  if(x != waveSelect)
    wave_storage.write(waveSelect);  // <-- save the waveSelect 

   div_storage.read(x);
  
  if(x != divSelect)
    div_storage.write(divSelect);  // <-- save the waveSelect  
  

}

#define NUMREADINGS 50
unsigned int pot[NUMREADINGS];
unsigned int freq[NUMREADINGS];

void filterPut (char input, unsigned int newreading)
{
  static unsigned char potptr=0;
  static unsigned char freqptr = 0;

  if(input == POT)
  {
    pot[potptr] = newreading;
    potptr++;
    if(potptr >= NUMREADINGS)
      potptr=0;
  }

  else if(input == FREQ)
  {
    freq[freqptr] = newreading;
    freqptr++;
    if(freqptr >= NUMREADINGS)
      freqptr = 0;
  }
  
}

unsigned int filterGet (bool input)
{
  unsigned long int x;
  float z;
  unsigned char y;

  x = 0;
  if(input == POT)
  {
    for (y=0;y<NUMREADINGS;y++)
    {
      x = x + pot[y];
    }
  }
  else if(input == FREQ)
  {
    for (y=0;y<NUMREADINGS;y++)
    {
      x = x + freq[y];
    }
  }

  z = x;
  z = z/NUMREADINGS;
  z = z + 0.5;
  return (unsigned int)z;
}
