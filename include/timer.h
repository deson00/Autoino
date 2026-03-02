void timerCallback();
void initializeTimerOne(unsigned long microseconds) {
  TCCR1A = 0; // clear control register A
  TCCR1B = _BV(WGM13); // set mode 8: phase and frequency correct pwm, stop the timer
  unsigned long cycles = (F_CPU / 2000000) * microseconds;
  unsigned char clockSelectBits;
  
  if (cycles < 65536) {
    clockSelectBits = _BV(CS10); // no prescale, full xtal
  } else if ((cycles >>= 3) < 65536) {
    clockSelectBits = _BV(CS11); // prescale by /8
  } else if ((cycles >>= 3) < 65536) {
    clockSelectBits = _BV(CS11) | _BV(CS10); // prescale by /64
  } else if ((cycles >>= 2) < 65536) {
    clockSelectBits = _BV(CS12); // prescale by /256
  } else if ((cycles >>= 2) < 65536) {
    clockSelectBits = _BV(CS12) | _BV(CS10); // prescale by /1024
  } else {
    cycles = 65535;
    clockSelectBits = _BV(CS12) | _BV(CS10); // request was out of bounds, set as maximum
  }

  // unsigned char oldSREG = SREG;
  // cli();
  // ICR1 = cycles;
  // SREG = oldSREG;
  // TCCR1B = _BV(WGM13) | clockSelectBits;
  // TIMSK1 = _BV(TOIE1); // sets the timer overflow interrupt enable bit
  // Desativa apenas a interrupção do Timer 1
  TIMSK1 &= ~_BV(TOIE1); // clear the timer overflow interrupt enable bit
  ICR1 = cycles; // Set the top value for the timer
  TCCR1B = _BV(WGM13) | clockSelectBits;
  // Reativa a interrupção do Timer 1
  TIMSK1 |= _BV(TOIE1); // sets the timer overflow interrupt enable bit
}

ISR(TIMER1_OVF_vect) {
  timerCallback();
}

void timerCallback() {
tempo_atual = micros();
// tempo_inicial_codigo = micros(); // Registra o tempo inicial      
if(local_rodafonica == 1 && tipo_ignicao_sequencial == 0){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0
  
if(loop_timer >= qtd_cilindro){
  loop_timer = 0;
}
int i = loop_timer; //provisorio para teste
// if(grau_pms > 180 && rpm < rpm_partida){
//   grau_pms = grau_pms - 180; //ajuste temporario ate outra solução para baixos rpm
// }

  if (grau_pms <= 120 && rpm > rpm_partida) {
    if (grau_pms < 60) {
        ajuste_pms = 180;
    }else{
      ajuste_pms = 0;
    } 
  }else{
    ajuste_pms = 0;
  }
  tempo_atual += i+1 * 100;// ajuste do timer para cada chamada 
  
calcula_dwell_comando(i);
iniciar_dwell_comando(i);
desligar_dwell_comando(i);
calcula_grau_injetor_comando(i);
ligar_injetor_comando(i);
desligar_injetor_comando(); 
  loop_timer++;
  }

if(local_rodafonica == 2 && tipo_ignicao_sequencial == 0 ){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0

int i = loop_timer; //provisorio para teste 
if(++loop_timer >= qtd_cilindro){
  loop_timer = 0;
}
  if (grau_pms < 90 && rpm > rpm_partida) {
    ajuste_pms = 180;
  }else{
    ajuste_pms = 0;
  }
calcula_grau_injetor(i);
calcula_grau_ignicao(i);
iniciar_dwell(i);
desligar_dwell(i);
ligar_injetor(i);
tempo_check = micros();
  for (int j = 0; j < qtd_cilindro; j++)
  {
    desligar_injetor(j); 
  }
}

}
