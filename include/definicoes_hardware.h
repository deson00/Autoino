// #define Autoino   //Descomente essa linha caso utilizar autoino em um Arduino Mega, UNO ou Nano etc
#define Speeduino  //Descomente essa linha caso utilizar Autoino em uma Speeduino

#ifdef Autoino
#define pino_sensor_roda_fonica 2
#define pino_sensor_fase 3
#define pino_sensor_map A0
#define pino_sensor_tps A1
#define pino_sensor_clt A2
#define pino_sensor_iat A3
#define pino_sensor_o2  A4
#define pino_sensor_brv A5
byte ign1 = 4;
byte ign2 = 5;
byte ign3 = 6;
byte ign4 = 7;
byte inj1 = 8;
byte inj2 = 9;
byte inj3 = 12;
byte inj4 = 13;
#endif
#ifdef Speeduino
#define pino_sensor_roda_fonica 19
#define pino_sensor_fase 18
#define pino_sensor_clt A1
#define pino_sensor_tps A2
#define pino_sensor_map A3
#define pino_sensor_iat A0
#define pino_sensor_o2  A8
#define pino_sensor_brv A4
byte ign1 = 40;
byte ign2 = 38;
byte ign3 = 52;
byte ign4 = 50;
byte inj1 = 8;
byte inj2 = 9;
byte inj3 = 10;
byte inj4 = 11;
#endif

// #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) // Uno e Nano
// void setPinHigh(uint8_t pin) {
//   if (pin >= 0 && pin <= 7) {
//     PORTD |= _BV(pin);       // Pinos 0 a 7 -> PORTD
//   } else if (pin >= 8 && pin <= 13) {
//     PORTB |= _BV(pin - 8);   // Pinos 8 a 13 -> PORTB
//   } else if (pin >= 14 && pin <= 19) {
//     PORTC |= _BV(pin - 14);  // Pinos 14 a 19 -> PORTC (A0 a A5)
//   }
// }

// void setPinLow(uint8_t pin) {
//   if (pin >= 0 && pin <= 7) {
//     PORTD &= ~_BV(pin);      // Pinos 0 a 7 -> PORTD
//   } else if (pin >= 8 && pin <= 13) {
//     PORTB &= ~_BV(pin - 8);  // Pinos 8 a 13 -> PORTB
//   } else if (pin >= 14 && pin <= 19) {
//     PORTC &= ~_BV(pin - 14); // Pinos 14 a 19 -> PORTC (A0 a A5)
//   }
// }
// #elif defined(__AVR_ATmega2560__) // Mega 2560
// void setPinHigh(uint8_t pin) {
//   if (pin >= 22 && pin <= 29) {
//     PORTA |= _BV(pin - 22);  // Pinos 22 a 29 -> PORTA
//   } else if (pin >= 10 && pin <= 13) {
//     PORTB |= _BV(pin - 10);  // Pinos 10 a 13 -> PORTB
//   } else if (pin >= 50 && pin <= 53) {
//     PORTB |= _BV(pin - 50 + 4); // Pinos 50 a 53 -> PORTB (4 a 7)
//   } else if (pin >= 30 && pin <= 37) {
//     PORTC |= _BV(pin - 30);  // Pinos 30 a 37 -> PORTC
//   } else if (pin >= 18 && pin <= 21) {
//     PORTD |= _BV(pin - 18);  // Pinos 18 a 21 -> PORTD
//   } else if (pin >= 0 && pin <= 7) {
//     PORTE |= _BV(pin);       // Pinos 0 a 7 -> PORTE
//   } else if (pin >= 8 && pin <= 9) {
//     PORTH |= _BV(pin - 8);   // Pinos 8 a 9 -> PORTH
//   } else if (pin >= 14 && pin <= 15) {
//     PORTJ |= _BV(pin - 14);  // Pinos 14 a 15 -> PORTJ
//   } else if (pin >= 16 && pin <= 17) {
//     PORTH |= _BV(pin - 16 + 1); // Pinos 16 a 17 -> PORTH (1 a 2)
//   } else if (pin >= 38 && pin <= 41) {
//     PORTG |= _BV(pin - 38);  // Pinos 38 a 41 -> PORTG
//   } else if (pin >= 42 && pin <= 49) {
//     PORTL |= _BV(pin - 42);  // Pinos 42 a 49 -> PORTL
//   }
// }

// void setPinLow(uint8_t pin) {
//   if (pin >= 22 && pin <= 29) {
//     PORTA &= ~_BV(pin - 22);  // Pinos 22 a 29 -> PORTA
//   } else if (pin >= 10 && pin <= 13) {
//     PORTB &= ~_BV(pin - 10);  // Pinos 10 a 13 -> PORTB
//   } else if (pin >= 50 && pin <= 53) {
//     PORTB &= ~_BV(pin - 50 + 4); // Pinos 50 a 53 -> PORTB (4 a 7)
//   } else if (pin >= 30 && pin <= 37) {
//     PORTC &= ~_BV(pin - 30);  // Pinos 30 a 37 -> PORTC
//   } else if (pin >= 18 && pin <= 21) {
//     PORTD &= ~_BV(pin - 18);  // Pinos 18 a 21 -> PORTD
//   } else if (pin >= 0 && pin <= 7) {
//     PORTE &= ~_BV(pin);       // Pinos 0 a 7 -> PORTE
//   } else if (pin >= 8 && pin <= 9) {
//     PORTH &= ~_BV(pin - 8);   // Pinos 8 a 9 -> PORTH
//   } else if (pin >= 14 && pin <= 15) {
//     PORTJ &= ~_BV(pin - 14);  // Pinos 14 a 15 -> PORTJ
//   } else if (pin >= 16 && pin <= 17) {
//     PORTH &= ~_BV(pin - 16 + 1); // Pinos 16 a 17 -> PORTH (1 a 2)
//   } else if (pin >= 38 && pin <= 41) {
//     PORTG &= ~_BV(pin - 38);  // Pinos 38 a 41 -> PORTG
//   } else if (pin >= 42 && pin <= 49) {
//     PORTL &= ~_BV(pin - 42);  // Pinos 42 a 49 -> PORTL
//   }
// }
// #endif
