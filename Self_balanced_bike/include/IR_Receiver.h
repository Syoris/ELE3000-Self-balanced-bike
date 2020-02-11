#ifndef IR_RECEIVER_H
#define IR_RECEIVER_H

#include "IRremote.h" //IR Receiver

const int RECV_PIN = 6;


//Prototypes
void IR_Setup();
void IR_decode();
unsigned long IR_receive();
void IR_print_cmd();

//Associations de la télécommande
#define OFF 0x7522
#define RADIO_BAND 0x7B26
#define AUDIO_IN 0x6122
#define UP 0x2426
#define DOWN 0x6426
#define SLEEP 0x322
#define MENU 0x4426
#define ENTER 0x426
#define PLAY 0x6626
#define BACK 0x6126
#define NEXT 0x1126
#define SOUND 0x622
#define VOL_UP 0x2422
#define VOL_DOWN 0x6422
#define TUNE_UP 0x6726
#define TUNE_DOWN 0x1726

#endif