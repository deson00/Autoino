
void timerCallback2();
void initializeTimerTwo(unsigned long microseconds) {
  TCCR2A = 0; // Limpa o registrador de controle A
  TCCR2B = 0; // Limpa o registrador de controle B
  TCNT2 = 0;  // Inicializa o contador

  // Calcula o número de ciclos
  unsigned long cycles = (F_CPU / 2000000) * microseconds;

  // Define o divisor do clock
  unsigned char clockSelectBits;
  if (cycles < 256) {
    clockSelectBits = _BV(CS20); // Sem prescaler
  } else if ((cycles >>= 3) < 256) {
    clockSelectBits = _BV(CS21); // Prescaler de 8
  } else if ((cycles >>= 2) < 256) {
    clockSelectBits = _BV(CS21) | _BV(CS20); // Prescaler de 32
  } else if ((cycles >>= 1) < 256) {
    clockSelectBits = _BV(CS22); // Prescaler de 64
  } else if ((cycles >>= 1) < 256) {
    clockSelectBits = _BV(CS22) | _BV(CS20); // Prescaler de 128
  } else if ((cycles >>= 1) < 256) {
    clockSelectBits = _BV(CS22) | _BV(CS21); // Prescaler de 256
  } else if ((cycles >>= 1) < 256) {
    clockSelectBits = _BV(CS22) | _BV(CS21) | _BV(CS20); // Prescaler de 1024
  } else {
    cycles = 255;
    clockSelectBits = _BV(CS22) | _BV(CS21) | _BV(CS20); // Valor máximo
  }

  OCR2A = cycles; // Define o valor de comparação
  TCCR2A = _BV(WGM21); // Configura o Timer2 para CTC (Clear Timer on Compare Match)
  TCCR2B = clockSelectBits; // Define o prescaler
  TIMSK2 = _BV(OCIE2A); // Habilita a interrupção no compare match
}

// ISR para o Timer2
ISR(TIMER2_COMPA_vect) {
  timerCallback2();
}

void timerCallback2() {
//funçoes de controle aqui
}