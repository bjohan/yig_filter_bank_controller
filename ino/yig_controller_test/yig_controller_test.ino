
#define SAMPLE_RATE 8
#define ADCS 0 
#define MUTEMIC 1 // disable MIC input
#define INSEL 0 // disable MIC input
#define MICBOOST 0 // enable MICBOOST
#define BYPASS 1 // Bypass LINEIN
#define DACSEL 1 // Select DAC for audio loopback
#define SIDETONE 0 // Deselect SIDETONE
#define SIDEATT 0
#include <Wire.h>
#include <SPI.h>
#include <AudioCodec.h>
#include <SerialCommands.h>
#include <EEPROM.h>
//#include "wm_8731.hpp"


struct yigParam{
  float slope[8];
  float offset[8];
  float lowLim[8];
  float highLim[8];
};

struct yigParams{
  struct yigParam a;
  struct yigParam b;
};

struct yigParams yp;



int relayDataPin=A3;
int relayClockPin=9;
int relayOutputPin=4;
int relayLatchPin=3;

int yigAMux0Pin=8;
int yigAMux1Pin=7;
int yigAMux2Pin=6;

int yigBMux0Pin=A2;
int yigBMux1Pin=A1;
int yigBMux2Pin=A0;



uint8_t relayAState=0;
uint8_t relayBState=0;

uint8_t yigAPort=6;
uint8_t yigBPort=7;
int yigAControl=0;
int yigBControl=0;

char primChan='A';

//Wm8731 *codec;

char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\n", " ");


void updateRelays(){
  digitalWrite(relayLatchPin, LOW);
  shiftOut(relayDataPin, relayClockPin, LSBFIRST, 1<<relayBState);
  shiftOut(relayDataPin, relayClockPin, LSBFIRST, 1<<relayAState);
  digitalWrite(relayOutputPin,LOW);
  digitalWrite(relayLatchPin, HIGH);
}


void updateYigs(){
  digitalWrite(yigAMux0Pin, yigAPort&0x1);
  digitalWrite(yigAMux1Pin, yigAPort&0x2);
  digitalWrite(yigAMux2Pin, yigAPort&0x4);

  digitalWrite(yigBMux0Pin, yigBPort&0x1);
  digitalWrite(yigBMux1Pin, yigBPort&0x2);
  digitalWrite(yigBMux2Pin, yigBPort&0x4);

  int left_in, right_in;
  AudioCodec_data(&left_in, &right_in, yigAControl, yigBControl);
  AudioCodec_data(&left_in, &right_in, yigAControl, yigBControl);  
 
}

int calculateTuningWordA(int filtNum, float freq){
    return yp.a.slope[filtNum]*freq+yp.a.offset[filtNum];
}

int calculateTuningWordB(int filtNum, float freq){
    return yp.b.slope[filtNum]*freq+yp.a.offset[filtNum];
}


void cmd_status(SerialCommands* sender, const char* cmd){
  sender->GetSerial()->print(F("Relay A: "));
  sender->GetSerial()->print(relayAState);
  sender->GetSerial()->println("");
  
  sender->GetSerial()->print(F("Relay B: "));
  sender->GetSerial()->print(relayAState);
  sender->GetSerial()->println("");

  sender->GetSerial()->print(F("YIG driver A channel: "));
  sender->GetSerial()->print(yigAPort);
  sender->GetSerial()->print(F(" value: "));
  sender->GetSerial()->print(yigAControl);
  sender->GetSerial()->println("");

  sender->GetSerial()->print(F("YIG driver B channel: "));
  sender->GetSerial()->print(yigBPort);
  sender->GetSerial()->print(F(" value: "));
  sender->GetSerial()->print(yigBControl);
  sender->GetSerial()->println("");
  
}

void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

void cmd_switch(SerialCommands* sender){


  char *which_rel=sender->Next();
  if(which_rel == NULL) {
    sender->GetSerial()->println(F("No relay specified, first argument must be 'A' or 'B'"));
    return;
  }
  char *which_port=sender->Next();
  if(which_port == NULL){
    sender->GetSerial()->println(F("No port specified, second argument must be [0..6]"));
    return;
  }
  int portNum=atoi(which_port);
  if(portNum < 0 || portNum > 6){
    sender->GetSerial()->println(F("Wrong port specified, second argument must be [0..6]"));
    return;
  }
  if(which_rel[0]=='A' ||which_rel[0]=='a'){
    relayAState=portNum;
    updateRelays();
  } else if(which_rel[0]=='B' ||which_rel[0]=='b'){
    relayBState=portNum;
    updateRelays();
  } else {
    sender->GetSerial()->println(F("Wrong relay specified, first argument must be 'A' or 'B'"));
    return;
  }
}

void cmd_yig(SerialCommands* sender){
  char *which_driver=sender->Next();

  if(which_driver == NULL) {
    sender->GetSerial()->println(F("No driver specified, first argument must be 'A' or 'B'"));
    return;
  }
  char *which_yig=sender->Next();
  if(which_yig == NULL){
    sender->GetSerial()->println(F("No port specified, second argument must be [0..7]"));
    return;
  }
  int yigNum=atoi(which_yig);
  if(yigNum < 0 || yigNum > 7){
    sender->GetSerial()->println(F("Wrong port specified, second argument must be [0..7]"));
    return;
  }


  char *yig_ctrl_str=sender->Next();
  if(yig_ctrl_str == NULL){
    sender->GetSerial()->println(F("No control word specified, third argument must be [-32768..32767]"));
    return;
  }
  int yigControl=atoi(yig_ctrl_str);
  
  if(which_driver[0]=='A' ||which_driver[0]=='a'){
    if(yigBPort==yigNum){
      sender->GetSerial()->println(F("Driver B already controls commanded port"));
      return;
    }
    yigAPort=yigNum;
    yigAControl=yigControl;
    updateYigs();
  } else if(which_driver[0]=='B' ||which_driver[0]=='b'){
    if(yigAPort==yigNum){
      sender->GetSerial()->println(F("Driver A already controls commanded port"));
      return;
    }
    yigBPort=yigNum;
    yigBControl=yigControl;
    updateYigs();
  } else {
    sender->GetSerial()->println(F("Wrong driver specified, first argument must be 'A' or 'B'"));
    return;
  }
}

void cmd_coeff(SerialCommands *sender){
  const char usage[] PROGMEM = "COEF A|B S|O|L|H 0..7 float";
  char *channel = sender->Next();
  if(channel == NULL){
    sender->GetSerial()->println(usage);
    return;
  }

  char *type= sender->Next();
  if(type == NULL){
    sender->GetSerial()->println(usage);  
    return;
  }

  char *yigNum = sender->Next();
  if(yigNum == NULL){
    sender->GetSerial()->println(usage);
    return;
  }

  char *theValue=sender->Next();
  if(theValue == NULL){
    sender->GetSerial()->println(usage);
    return;
  }

  int yign = atoi(yigNum);
  if(yign < 0 || yign > 7){
    sender->GetSerial()->println(usage);
    return;
  }

  float tv = atof(theValue);
  
  if(channel[0] == 'A'){
    if(type[0] == 'S') yp.a.slope[yign] = tv;
    else if(type[0] == 'O') yp.a.offset[yign] = tv;
    else if(type[0] == 'L') yp.a.lowLim[yign] = tv;
    else if(type[0] == 'H') yp.a.highLim[yign] = tv;
    else{
      sender->GetSerial()->println(usage);
      return;
    }
  } else if(channel[0] == 'B'){
    if(type[0] == 'S') yp.b.slope[yign] = tv;
    else if(type[0] == 'O') yp.b.offset[yign] = tv;
    else if(type[0] == 'L') yp.b.lowLim[yign] = tv;
    else if(type[0] == 'H') yp.b.highLim[yign] = tv;
    else{
      sender->GetSerial()->println(usage);
      return;
    }
  } else {
    sender->GetSerial()->println(usage);
  }
  sender->GetSerial()->println("OK");
}


void cmd_print_coeff(SerialCommands *sender){
  for(int i = 0 ; i < 8 ; i++){
    sender->GetSerial()->print(F("Channel A "));
    sender->GetSerial()->print(i);
    sender->GetSerial()->print(F(" Slope: "));
    sender->GetSerial()->print(yp.a.slope[i]*1e-6);
    sender->GetSerial()->print(F(" [MHz/LSB] Offset: "));
    sender->GetSerial()->print(yp.a.offset[i]);
    sender->GetSerial()->print(F(" [LSB] FrequencyLow: "));
    sender->GetSerial()->print(yp.a.lowLim[i]*1e-6);
    sender->GetSerial()->print(F(" [MHz] FrequencyHigh: "));
    sender->GetSerial()->print(yp.a.highLim[i]*1e-6);
    sender->GetSerial()->println(F(" [MHz]"));
  }
    for(int i = 0 ; i < 8 ; i++){
    sender->GetSerial()->print(F("Channel B "));
    sender->GetSerial()->print(i);
    sender->GetSerial()->print(F(" Slope: "));
    sender->GetSerial()->print(yp.b.slope[i]*1e-6);
    sender->GetSerial()->print(F(" [MHz/LSB] Offset: "));
    sender->GetSerial()->print(yp.b.offset[i]);
    sender->GetSerial()->print(F(" [LSB] FrequencyLow: "));
    sender->GetSerial()->print(yp.b.lowLim[i]*1e-6);
    sender->GetSerial()->print(F(" [MHz] FrequencyHigh: "));
    sender->GetSerial()->print(yp.b.highLim[i]*1e-6);
    sender->GetSerial()->println(F(" [MHz]"));
  }
}

int findFilterIndex(struct yigParam *a, float f){
  for(int i = 0 ; i < 8 ; i++){
    if(a->lowLim[i] <= f &&  f <= a->highLim[i]) return i;
  }
  return -1;
}

void cmd_tune(SerialCommands *sender){
 const char usage[] PROGMEM = " A|B freqHzAsFloat"; 
  char *chan=sender->Next();
  if(chan == NULL){
    sender->GetSerial()->println(usage);
    return;
  }

  char *freqStr = sender->Next();
  if(freqStr == NULL){
    sender->GetSerial()->println(usage);
    return;
  }
  
  float freq = atof(freqStr);
  if(chan[0] == 'A'){
    int i = findFilterIndex(&yp.a, freq);
    if(i < 0) return;
    if(yigBPort==i) return;
    yigAPort = i;
    sender->GetSerial()->println(i);
    yigAControl = (freq-yp.a.offset[i])/yp.a.slope[i];
    //sender->GetSerial()->println((freq-yp.a.offset)/yp.a.slope[i]);
    if(primChan == 'A'){
      relayAState=i+1;
      updateRelays();
    }
    updateYigs();
  }

  if(chan[0] == 'B'){
    int i = findFilterIndex(&yp.b, freq);
    if(i < 0) return;
    if(yigAPort==i) return;
    yigBPort = i;
    yigBControl = freq*yp.b.slope[i]+yp.b.offset[i];
    if(primChan == 'B'){
      relayAState=i+1;
      updateRelays();
    }
    updateYigs();
  }
  sender->GetSerial()->println("OK");
}

void cmd_prim(SerialCommands *sender){
  char *c=sender->Next();
  const static char usage[] PROGMEM = "A or B for primary channel";
  if(c == 'A' or c == 'a')
    primChan='A';
  else if (c == 'B' or c == 'b')
    primChan='B';
  else
    sender->GetSerial()->println(usage);
}

void saveCoeff(){
  EEPROM.put(0, yp);
}

void loadCoeff(){
  EEPROM.get(0, yp);
}

void cmd_mem(SerialCommands *sender){
  char *c = sender->Next();
  const static char usage[] PROGMEM = "S Save or L Load";
  if(c==NULL){
    sender->GetSerial()->println(usage);
    return;
  }
  if(c[0] == 'S'){
    saveCoeff();
    sender->GetSerial()->println(F("saved"));
  } else if(c[0] == 'L'){
    loadCoeff();
    sender->GetSerial()->println(F("loaded"));
  } else{
    sender->GetSerial()->println(usage);
  }
}

SerialCommand cmd_switch_("SWITCH", cmd_switch);
SerialCommand cmd_sw_("SW", cmd_switch);
SerialCommand cmd_yig_("YIG", cmd_yig);
SerialCommand cmd_status_("STATUS", cmd_status);
SerialCommand cmd_coeff_("COEF", cmd_coeff);
SerialCommand cmd_print_coeff_("PC", cmd_print_coeff);
SerialCommand cmd_s_("S", cmd_status);
SerialCommand cmd_tune_to_("T", cmd_tune);
SerialCommand cmd_mem_("M", cmd_mem);
SerialCommand cmd_prim_("P", cmd_prim);

void setup() {
  // call this last if you are setting up other things
  AudioCodec_init(); // setup codec and microcontroller registers
  Serial.begin(115200); // Begin serial communications for radio tansmitter
  
  Serial.println("INIT STARTED");
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(yigAMux0Pin, OUTPUT);
  pinMode(yigAMux1Pin, OUTPUT);
  pinMode(yigAMux2Pin, OUTPUT);
  pinMode(yigBMux0Pin, OUTPUT);
  pinMode(yigBMux1Pin, OUTPUT);
  pinMode(yigBMux2Pin, OUTPUT);

  pinMode(relayDataPin, OUTPUT);
  pinMode(relayClockPin, OUTPUT);
  pinMode(relayOutputPin, OUTPUT);
  pinMode(relayLatchPin, OUTPUT);
  loadCoeff();
  updateYigs();  
  updateRelays();

  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&cmd_switch_);
  serial_commands_.AddCommand(&cmd_sw_);
  serial_commands_.AddCommand(&cmd_yig_);
  serial_commands_.AddCommand(&cmd_status_);
  serial_commands_.AddCommand(&cmd_s_);
  serial_commands_.AddCommand(&cmd_coeff_);
  serial_commands_.AddCommand(&cmd_print_coeff_);
  serial_commands_.AddCommand(&cmd_mem_);
  serial_commands_.AddCommand(&cmd_tune_to_);
  serial_commands_.AddCommand(&cmd_prim_);
  //AudioCodec_init();
  //codec= new Wm8731(10);
  Serial.println("INIT DONE");
}


void loop() {
  serial_commands_.ReadSerial();
}
