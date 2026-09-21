// Perfil de hardware (mapa de pinos). Quem escolhe e o build_flags do
// platformio.ini, um -D por ambiente: os ambientes de Nano/Uno passam
// -D Autoino e o de Mega passa -D Speeduino.
//
// Antes havia um "#define Autoino" fixo aqui e trocar de placa exigia editar
// este arquivo na mao, comentando uma linha e descomentando outra. O trabalho
// era o de menos: o risco real era commitar o arquivo trocado sem perceber e
// o build seguinte sair com a pinagem errada - bobina e bico em pino que nao
// e o deles.
//
// O fallback por MCU abaixo continua valendo para quem compila fora do
// PlatformIO (Arduino IDE, por exemplo), onde nao ha build_flags.
#if !defined(Autoino) && !defined(Speeduino)
	#if defined(__AVR_ATmega2560__)
		#define Speeduino
	#else
		#define Autoino
	#endif
#endif

#if defined(Autoino) && defined(Speeduino)
	#error "Autoino e Speeduino definidos ao mesmo tempo - escolha um perfil de hardware"
#endif

#ifdef Autoino
	#pragma message("perfil de hardware: Autoino")
#else
	#pragma message("perfil de hardware: Speeduino")
#endif

// Timer do PWM da marcha lenta. Depende do perfil E do MCU, porque o que
// importa e qual pino FISICO a saida de comparacao do timer alcanca:
//
//   Autoino no 328P/168 -> Timer2/OC2A cai no pino 11, que e pino_marcha_lenta
//   Speeduino no 2560   -> Timer3/OC3A cai no pino 5, que e pino_marcha_lenta
//
// Autoino no ATmega2560 nao tem combinacao valida. La o pino 11 e OC1A, do
// Timer1, que pertence ao agendador de ignicao - mexer no TCCR1A para gerar
// PWM quebraria o agendamento. E OC2A no Mega e o pino 10, ou seja usar o
// Timer2 ali colocaria o PWM num pino onde a fiacao nao esta, silenciosamente.
//
// Entao nessa variante a marcha lenta por PWM fica indisponivel, de forma
// declarada. Os modos liga/desliga e motor de passo continuam funcionando:
// os dois usam digitalWrite e nao dependem de timer.
#if defined(Autoino) && (defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__))
	#define MARCHA_LENTA_PWM_TIMER2
#elif defined(Speeduino) && defined(__AVR_ATmega2560__)
	#define MARCHA_LENTA_PWM_TIMER3
#else
	#define MARCHA_LENTA_PWM_INDISPONIVEL
	#pragma message("marcha lenta por PWM indisponivel nesta combinacao de perfil e MCU (liga/desliga e passo seguem funcionando)")
#endif

#ifdef Autoino
#define pino_sensor_roda_fonica 2
#define pino_sensor_fase 3
#define pino_sensor_map A0
#define pino_sensor_tps A1
#define pino_sensor_clt A2
#define pino_sensor_iat A3
#define pino_sensor_o2  A4
#define pino_sensor_brv A5
#define pino_sensor_flex A6
#define pino_sensor_pressao_oleo A7
#define pino_marcha_lenta 11
#define pino_passo_marcha_lenta 11
// DIR trocou de lugar com o inj4 (era D10, o inj4 era D13).
//
// O D13 e o SCK e o LED do bootloader: ele e dirigido a cada reset e a
// cada gravacao, por volta de 100 ms por pisco. Com bico ali, isso
// despejava 20 a 50 vezes uma injetada normal (2 a 5 ms) com o motor
// parado. Pulldown nao resolve - o bootloader DIRIGE o pino, nao o
// deixa solto -, entao a saida era tirar o atuador de la.
//
// DIR e o sinal mais tolerante a isso em todo o sistema: ele so diz o
// lado, e o motor de passo so anda quando o STEP pulsa. O bootloader
// mexendo na direcao de um motor que nao esta recebendo passo nao faz
// nada - e o LED da placa passa a acompanhar a direcao, de brinde.
#define pino_direcao_marcha_lenta 13
constexpr byte ign1 = 4;
constexpr byte ign2 = 5;
constexpr byte ign3 = 6;
constexpr byte ign4 = 7;
constexpr byte inj1 = 8;
constexpr byte inj2 = 9;
constexpr byte inj3 = 12;
constexpr byte inj4 = 10;
#endif
#ifdef Speeduino
#define pino_sensor_roda_fonica 19
#define pino_sensor_fase 18
#define pino_sensor_iat A0
#define pino_sensor_clt A1
#define pino_sensor_tps A2
#define pino_sensor_map A3
#define pino_sensor_brv A4
#define pino_sensor_flex A6
#define pino_sensor_pressao_oleo A7
#define pino_sensor_o2  A8
#define pino_marcha_lenta 5 // Saida Idle 1 / IDLE-OUT padrao da Speeduino v0.4.
#define pino_passo_marcha_lenta 17 // STEP padrao do soquete stepper Speeduino v0.4.
#define pino_direcao_marcha_lenta 16 // DIR padrao do soquete stepper Speeduino v0.4.
constexpr byte ign1 = 40;
constexpr byte ign2 = 38;
constexpr byte ign3 = 52;
constexpr byte ign4 = 50;
constexpr byte inj1 = 8;
constexpr byte inj2 = 9;
constexpr byte inj3 = 10;
constexpr byte inj4 = 11;
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

// Guarda de colisao de pinos, em tempo de compilacao.
//
// O motivo de existir: ao mover o inj4 do D13 para o D10, o D10 ja era o DIR do
// motor de passo. Sem perceber, os dois codigos dirigiriam o mesmo pino - o
// bico abriria a cada troca de direcao e a direcao inverteria a cada injetada.
// Colisao de pino nao aparece em teste de bancada com uma funcao de cada vez;
// aparece com fiacao ligada e motor no lugar.
//
// Nao custa flash: e tudo avaliado pelo compilador.
//
// pino_marcha_lenta e pino_passo_marcha_lenta compartilham pino de proposito -
// PWM ou passo, nunca os dois -, entao esse par fica fora da conta.
#define AUTOINO_PINOS_DIFEREM(a, b) static_assert((a) != (b),   "colisao de pino: " #a " e " #b " estao no mesmo pino")

AUTOINO_PINOS_DIFEREM(ign1, ign2); AUTOINO_PINOS_DIFEREM(ign1, ign3);
AUTOINO_PINOS_DIFEREM(ign1, ign4); AUTOINO_PINOS_DIFEREM(ign2, ign3);
AUTOINO_PINOS_DIFEREM(ign2, ign4); AUTOINO_PINOS_DIFEREM(ign3, ign4);

AUTOINO_PINOS_DIFEREM(inj1, inj2); AUTOINO_PINOS_DIFEREM(inj1, inj3);
AUTOINO_PINOS_DIFEREM(inj1, inj4); AUTOINO_PINOS_DIFEREM(inj2, inj3);
AUTOINO_PINOS_DIFEREM(inj2, inj4); AUTOINO_PINOS_DIFEREM(inj3, inj4);

AUTOINO_PINOS_DIFEREM(ign1, inj1); AUTOINO_PINOS_DIFEREM(ign1, inj2);
AUTOINO_PINOS_DIFEREM(ign1, inj3); AUTOINO_PINOS_DIFEREM(ign1, inj4);
AUTOINO_PINOS_DIFEREM(ign2, inj1); AUTOINO_PINOS_DIFEREM(ign2, inj2);
AUTOINO_PINOS_DIFEREM(ign2, inj3); AUTOINO_PINOS_DIFEREM(ign2, inj4);
AUTOINO_PINOS_DIFEREM(ign3, inj1); AUTOINO_PINOS_DIFEREM(ign3, inj2);
AUTOINO_PINOS_DIFEREM(ign3, inj3); AUTOINO_PINOS_DIFEREM(ign3, inj4);
AUTOINO_PINOS_DIFEREM(ign4, inj1); AUTOINO_PINOS_DIFEREM(ign4, inj2);
AUTOINO_PINOS_DIFEREM(ign4, inj3); AUTOINO_PINOS_DIFEREM(ign4, inj4);

AUTOINO_PINOS_DIFEREM(pino_marcha_lenta, ign1);  AUTOINO_PINOS_DIFEREM(pino_marcha_lenta, ign2);
AUTOINO_PINOS_DIFEREM(pino_marcha_lenta, ign3);  AUTOINO_PINOS_DIFEREM(pino_marcha_lenta, ign4);
AUTOINO_PINOS_DIFEREM(pino_marcha_lenta, inj1);  AUTOINO_PINOS_DIFEREM(pino_marcha_lenta, inj2);
AUTOINO_PINOS_DIFEREM(pino_marcha_lenta, inj3);  AUTOINO_PINOS_DIFEREM(pino_marcha_lenta, inj4);

AUTOINO_PINOS_DIFEREM(pino_direcao_marcha_lenta, ign1); AUTOINO_PINOS_DIFEREM(pino_direcao_marcha_lenta, ign2);
AUTOINO_PINOS_DIFEREM(pino_direcao_marcha_lenta, ign3); AUTOINO_PINOS_DIFEREM(pino_direcao_marcha_lenta, ign4);
AUTOINO_PINOS_DIFEREM(pino_direcao_marcha_lenta, inj1); AUTOINO_PINOS_DIFEREM(pino_direcao_marcha_lenta, inj2);
AUTOINO_PINOS_DIFEREM(pino_direcao_marcha_lenta, inj3); AUTOINO_PINOS_DIFEREM(pino_direcao_marcha_lenta, inj4);

AUTOINO_PINOS_DIFEREM(pino_sensor_roda_fonica, pino_sensor_fase);
