/* *************************************************************
   Encoder driver function definitions - by James Nugen
   Update to add Arduino Mega support - by Nathaniel Gallinger
   ************************************************************ */


#ifdef ARDUINO_UNO_ENC_COUNTER
  //below can be changed, but should be PORTD pins;
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2 Uno
  #define LEFT_ENC_PIN_B PD3  //pin 3 Uno

  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4 Uno
  #define RIGHT_ENC_PIN_B PC5  //pin A5 Uno
#endif


#ifdef ARDUINO_MEGA_ENC_COUNTER
  //below can be changed, but should be PORTK pins;
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PK7  //pin A15 Mega 1280/2560
  #define LEFT_ENC_PIN_B PK6  //pin A14 Mega 1280/2560

  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PB0  //pin 53 Mega 1280/2560
  #define RIGHT_ENC_PIN_B PB1  //pin 52 Mega 1280/2560
#endif


long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

