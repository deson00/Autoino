#include <Arduino.h>

#define qtd_dente 60
#define qtd_dente_faltante 2
#define pino_sensor_roda_fonica 19
#define pino_sensor_fase 18

volatile int pulsos;
volatile unsigned long ti = 0;
volatile unsigned long tf;
volatile unsigned int rpm;
int inicia = 1;
int ign1 = 40;
int ign2 = 38;
int ign3 = 52;
int ign4 = 50;
volatile unsigned int dwell_bobina = 4;
volatile unsigned int local_rodafonica = 2; // 2 para virabrequinho e 1 para comando
volatile unsigned int qtd_cilindro = 6 / local_rodafonica;
volatile unsigned int qtd_voltas = 0;
volatile unsigned int grau_cada_dente = 360 / qtd_dente;
volatile unsigned int grau_avanco = 0;
volatile unsigned int grau_pms = 54 + grau_avanco;
volatile unsigned int grau_entre_cada_cilindro = 360 / qtd_cilindro;
volatile unsigned int posicao_atual_sensor = 0;
volatile unsigned int leitura = 0;
volatile unsigned int qtd_leitura = 0;
volatile unsigned int rpm_anterior = 0;
volatile unsigned long tempo_anterior;
volatile unsigned long tempo_dente_anterior[2];
volatile unsigned long tempo_dente_anterior_pms;
volatile unsigned long tempo_inicio_volta_completa;
volatile unsigned long tempo_final_volta_completa;
volatile unsigned long tempo_total_volta_completa;
volatile unsigned long tempo_cada_grau = 0;
volatile unsigned long tempo_proxima_ignicao = 0;
volatile unsigned long tempo_atual;
volatile unsigned long intervalo_tempo_entre_dente = 0;
volatile unsigned long verifica_falha = 0;
volatile int angle;
volatile int pms = 0;
volatile long falha = 0;
volatile int cilindro = 0;
int dente = 0;
volatile int tipo_ignicao_sequencial = 0;

int rpm_adiciona_ponto(int rpm) {
    int ponto_ignicao = 0;
    if (rpm >= 600) {
        ponto_ignicao = rpm / 600;
    } 
    return ponto_ignicao * 6;
}

void leitor_sensor_roda_fonica()
{
  noInterrupts();
  qtd_leitura++;
  pulsos++;
  tf = micros();
  long delta = tf - ti;
  rpm = (60) / (float(delta) / 1000000 * 60);
  ti = tf;
  // calculo de mediana
  /*if(n % 2 == 0){
    md = n/2;
    mediana = (v[md] + v[md-1])/2;
  }
  else{
    md = n/2;
    mediana = v[md];
  })*/

  tempo_atual = micros();
  intervalo_tempo_entre_dente = (tempo_atual - tempo_anterior);
  verifica_falha = (tempo_dente_anterior[leitura] / 2) + tempo_dente_anterior[leitura];

  if (inicia)
  {
    tempo_anterior = micros();
    tempo_dente_anterior[0] = tempo_anterior;
    inicia = 0;
  }

  if (leitura == 0)
  {
    leitura = 1;
    tempo_dente_anterior[0] = (tempo_atual - tempo_anterior);
  }
  else
  {
    leitura = 0;
    tempo_dente_anterior[1] = (tempo_atual - tempo_anterior);
  }

  //Serial.print("|");
  if (verifica_falha < intervalo_tempo_entre_dente)
  {
    if (qtd_voltas == 1)
    {
      tempo_final_volta_completa = micros();
      tempo_total_volta_completa = (tempo_final_volta_completa - tempo_inicio_volta_completa);
      qtd_voltas = 0;
    }
    if (qtd_voltas == 0)
    {
      tempo_inicio_volta_completa = micros();
      qtd_voltas = 1;
    }

    //Serial.println("");
    //Serial.print("__");
    tempo_cada_grau = tempo_total_volta_completa / 360;

    posicao_atual_sensor = grau_cada_dente * qtd_dente_faltante;
    qtd_leitura = qtd_dente_faltante;
    falha++;
    
    
  }
  posicao_atual_sensor = posicao_atual_sensor + grau_cada_dente;
  if (posicao_atual_sensor == grau_pms  - rpm_adiciona_ponto(rpm_anterior))
  {
    //Serial.print(posicao_atual_sensor);
    tempo_proxima_ignicao = micros();
    pms = 1;
    cilindro = 0;
  }

  tempo_anterior = tempo_atual;
  interrupts();
}


void setup()
{
  pinMode(ign1, OUTPUT);
  pinMode(ign2, OUTPUT);
  pinMode(ign3, OUTPUT);
  pinMode(ign4, OUTPUT);
  pinMode(pino_sensor_roda_fonica, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pino_sensor_roda_fonica), leitor_sensor_roda_fonica, RISING);
  Serial.begin(9600);
}

void loop()
{ 
  long tempo_atual_ignicao = micros();
  if ((tempo_atual_ignicao + (dwell_bobina * 1000) >= tempo_proxima_ignicao) && (falha > 3) && (pms == 1) && (rpm >= 100))
  {
    rpm_anterior = rpm;
    cilindro++;
    tempo_proxima_ignicao = tempo_atual_ignicao + (tempo_cada_grau * grau_entre_cada_cilindro);
    if(cilindro <= qtd_cilindro){
      Serial.print("ign: ");
      Serial.print(cilindro);
      Serial.print(" RPM: ");
      Serial.println(rpm_anterior);
    }
    rpm = 0;
    if ((tipo_ignicao_sequencial == 1) && (qtd_cilindro <= 2) && (cilindro <= qtd_cilindro))
    {
      if (cilindro == 1)
      {
        digitalWrite(ign1, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign1, LOW);
      }
      if (cilindro == 2)
      {
        digitalWrite(ign2, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign2, LOW);
      }
      if (cilindro == 3)
      {
        digitalWrite(ign3, HIGH);
        delay(dwell_bobina / 1000);
        digitalWrite(ign3, LOW);
      }
      if (cilindro == 4)
      {
        digitalWrite(ign4, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign4, LOW);
      }
    }
    if ((tipo_ignicao_sequencial == 0) && (qtd_cilindro <= 2) && (cilindro <= qtd_cilindro))
    {
      if (cilindro == 1)
      {
        digitalWrite(ign1, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign1, LOW);
      }
      if (cilindro == 2)
      {
        digitalWrite(ign2, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign2, LOW);
      }
      if (cilindro == 3)
      {
        digitalWrite(ign1, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign1, LOW);
      }
      if (cilindro == 4)
      {
        digitalWrite(ign2, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign2, LOW);
      }
    }
    if ((tipo_ignicao_sequencial == 0) && (qtd_cilindro > 2) && (qtd_cilindro < 4) && (cilindro <= qtd_cilindro))
    {
      if (cilindro == 1)
      {
        digitalWrite(ign1, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign1, LOW);
      }
      if (cilindro == 2)
      {
        digitalWrite(ign2, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign2, LOW);
      }
      if (cilindro == 3)
      {
        digitalWrite(ign3, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign3, LOW);
      }
      if (cilindro == 4)
      {
        digitalWrite(ign1, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign1, LOW);
      }
      if (cilindro == 5)
      {
        digitalWrite(ign2, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign2, LOW);
      }
      if (cilindro == 6)
      {
        digitalWrite(ign3, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign3, LOW);
      }
    }
    if ((tipo_ignicao_sequencial == 0) && (qtd_cilindro > 3) && (cilindro <= qtd_cilindro))
    {
      if (cilindro == 1)
      {
        digitalWrite(ign1, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign1, LOW);
      }
      if (cilindro == 2)
      {
        digitalWrite(ign2, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign2, LOW);
      }
      if (cilindro == 3)
      {
        digitalWrite(ign3, HIGH);
        delay(dwell_bobina / 1000);
        digitalWrite(ign3, LOW);
      }
      if (cilindro == 4)
      {
        digitalWrite(ign4, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign4, LOW);
      }
      if (cilindro == 5)
      {
        digitalWrite(ign1, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign1, LOW);
      }
      if (cilindro == 6)
      {
        digitalWrite(ign2, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign2, LOW);
      }
      if (cilindro == 7)
      {
        digitalWrite(ign3, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign3, LOW);
      }
      if (cilindro == 8)
      {
        digitalWrite(ign4, HIGH);
        delay(dwell_bobina);
        digitalWrite(ign4, LOW);
      }
    }
    if ((qtd_cilindro * local_rodafonica) == cilindro)
    {
      cilindro = 0;
    }
  }
}
