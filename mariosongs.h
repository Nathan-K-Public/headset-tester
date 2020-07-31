#ifndef MarioSongs_h
#define MarioSongs_h
/*
  Arduino Mario Bros Tunes
  With Piezo Buzzer and PWM

  Connect the positive side of the Buzzer to pin 3,
  then the negative side to a 1k ohm resistor. Connect
  the other side of the 1 k ohm resistor to
  ground(GND) pin on the Arduino.

  by: Dipto Pratyaksa
  last updated: 31/3/13
*/

#include "notes.h"

//Mario main theme melody
int world1theme[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0
};
//Mario main them tempo
int world1tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12
};
//Underworld melody
int underworld_melody[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
 
};
//Underwolrd tempo
int underworld_tempo[] = {
  12, 12, 12, 12,
  12, 12, 6,
  3
};

//Underworld melody
int underworldfull_melody[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4,
  NOTE_DS4, NOTE_GS3,
  NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3,
  NOTE_AS3, NOTE_A3, NOTE_GS3,
  0, 0, 0
};
//Underwolrd tempo
int underworldfull_tempo[] = {
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  6, 18, 18, 18,
  6, 6,
  6, 6,
  6, 6,
  18, 18, 18, 18, 18, 18,
  10, 10, 10,
  10, 10, 10,
  3, 3, 3
};



int theme[] = {11,                                                  // Array for Theme song
  NOTE_E4, 8, NOTE_E4, 8, NOTE_H, 8, NOTE_E4, 8, NOTE_H, 8, NOTE_C4, 8, NOTE_E4, 8, NOTE_H, 8, NOTE_G4, 8, NOTE_H, 3, NOTE_G3, 8};
int life[] = {6,                                                    // Array for 1-up sound effect
  NOTE_E5, 10, NOTE_G5, 10, NOTE_E6, 10, NOTE_C6, 10, NOTE_D6, 10, NOTE_G6, 10};
int flagpole[] = {27,                                               // Array for Flag pole sound effect & song
  NOTE_G2, 10, NOTE_C3, 10, NOTE_E3, 10, NOTE_G3, 10, NOTE_C4, 10, NOTE_E4, 10, NOTE_G4, 3, NOTE_E4, 3, NOTE_GS2, 10, NOTE_C3, 10, 
  NOTE_DS3, 10, NOTE_GS3, 10, NOTE_C4, 10, NOTE_DS4, 10, NOTE_GS4, 3, NOTE_DS4, 3, NOTE_AS2, 10, NOTE_D3, 10, NOTE_F3, 10, 
  NOTE_AS3, 10, NOTE_D4, 10, NOTE_F4, 10, NOTE_AS4, 3, NOTE_B4, 10, NOTE_B4, 10, NOTE_B4, 10, NOTE_C5, 2};
int death[] = {17,                                                  // Array for Death sound effect & song
  NOTE_C4, 32, NOTE_CS4, 32, NOTE_D4, 16, NOTE_H, 4, NOTE_H, 2, NOTE_B3, 8, NOTE_F4, 8, NOTE_H, 8, NOTE_F4, 8, NOTE_F4, 6, 
  NOTE_E4, 6, NOTE_D4, 6, NOTE_C4, 8, NOTE_E3, 8, NOTE_H, 8, NOTE_E3, 8, NOTE_C3, 8};
int gameover[] = {15,                                               // Array for Game over song
  NOTE_C4, 8, NOTE_H, 8, NOTE_H, 8, NOTE_G3, 8, NOTE_H, 4, NOTE_E3, 4, NOTE_A3, 6, NOTE_B3, 6, NOTE_A3, 6, NOTE_GS3, 6, NOTE_AS3, 6, 
NOTE_GS3, 6, NOTE_G3, 8, NOTE_F3, 8, NOTE_G3, 4};


void sing(int song, int melodyPin) {
  // iterate over the notes of the melody:

  int* songToPlay=NULL;
  int* songTempo=NULL;
  int songLength=0;
  int songIncr=1;
  int songSpeed=1000;

  switch(song){
    case 1:
      Serial.println(" 'World1 Theme'");
      songToPlay=world1theme;
      songTempo=world1tempo;
      songIncr=1;
      songLength = sizeof(world1theme)/ sizeof(int);
      songSpeed=1500;
    break;
    case 2:
      Serial.println(" 'Underworld Theme'");
      songToPlay=underworld_melody;
      songTempo=underworld_tempo;
      songIncr=1;
      songLength = sizeof(underworld_melody)/ sizeof(int);
      songSpeed=1500;
    break;
    case 3:
       Serial.println(" 'Death Theme'");
       songToPlay=&(death[1]);
       songTempo=&(death[2]);
       songIncr=2;
       //songLength=sizeof(death)/ (sizeof(int) * 2);
       songLength=death[0];
       songSpeed=1000;
    break;
    case 4:
       Serial.println(" 'Game Over Theme'");
       songToPlay=&(gameover[1]);
       songTempo=&(gameover[2]);
       songIncr=2;
       songLength=gameover[0];
       songSpeed=1000;
    break;
    case 5:
       Serial.println(" 'Life Theme'");
       songToPlay=&(life[1]);
       songTempo=&(life[2]);
       songIncr=2;
       songLength=life[0];
       songSpeed=1000;
    break;    
    case 6:
       Serial.println(" 'Flagpole Theme'");
       songToPlay=&(flagpole[1]);
       songTempo=&(flagpole[2]);
       songIncr=2;
       songLength=flagpole[0];
       songSpeed=1000;
    break; 
    case 7:
       Serial.println(" 'Main Theme'");
       songToPlay=&(theme[1]);
       songTempo=&(theme[2]);
       songIncr=2;
       songLength=theme[0];
       songSpeed=1000;
    break;
    case 8:
      Serial.println(" 'Underworld Full Theme'");
      songToPlay=underworldfull_melody;
      songTempo=underworldfull_tempo;
      songIncr=1;
      songLength = sizeof(underworldfull_melody)/ sizeof(int);
      songSpeed=1500;
    break; 
    case 9:
      Serial.println(" 'Error Theme'");
      tone(melodyPin, NOTE_DS5);
      delay(300);
      tone(melodyPin, NOTE_D5);
      delay(300);
      tone(melodyPin, NOTE_CS5);
      delay(300);
      for (int i = 0; i < 200; i++) {
        tone(melodyPin, NOTE_C5 + (i % 20 - 10));
        delay(5);
      }
      noTone(melodyPin);
    break;
    default:
      Serial.println(" Invalid song selected.");
      return;
  }

 
  for (int thisNote = 0; thisNote < songLength; thisNote++) {
      
      // to calculate the note duration, take one second
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = songSpeed / songTempo[thisNote*songIncr];
      
      tone(melodyPin, songToPlay[thisNote*songIncr], noteDuration);

      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);

      // stop the tone playing:
      noTone(melodyPin);
  }  

  //delay(500);
return;
}

#endif

