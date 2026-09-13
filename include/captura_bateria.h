#ifndef CAPTURA_BATERIA_H
#define CAPTURA_BATERIA_H

/*
 * Captura crua da tensao de bateria em arranque - modo "osciloscopio".
 *
 * POR QUE ELE EXISTE
 *
 * O teste de compressao por roda fonica (teste_compressao.h) mede a frenagem
 * de cada cilindro pelo tempo do dente. Isso exige dente fino: uma 60-2 no
 * virabrequim da 19 dentes por compressao num 6 cilindros, e o vale aparece
 * inteiro. Uma 12-1 no comando da 1,8 - dois pontos por evento, e ainda por
 * cima o gap sozinho cobre 120 graus, uma compressao inteira. Nessa roda o
 * metodo do dente nao tem como funcionar, e a UI recusa o teste.
 *
 * A tensao de bateria resolve porque as duas medidas pedem coisas diferentes
 * da roda fonica. O metodo do dente usa a roda como relogio E como sensor, por
 * isso precisa de dente fino. Aqui a roda so diz QUAL cilindro - 60 graus
 * identificam um entre seis com folga - e quem mede a forca da compressao e o
 * ADC, que a 500 Hz da uns 55 pontos por evento a 180 rpm.
 *
 * O QUE FOI MEDIDO ANTES DE ESCREVER ISTO
 *
 * Divisor do BRV no shield Speeduino v0.4: 3,9k em cima, 1k embaixo (fundo de
 * escala 24,5 V, que e o que a UI usa). A impedancia vista pelo capacitor de
 * filtro e 796 ohms, entao seria preciso um capacitor de 20 uF para atenuar os
 * 9 Hz de um 6 cilindros a 180 rpm. Entrada de ADC leva 100 nF, que poe o corte
 * em 2 kHz - 200 vezes acima do sinal.
 *
 * Em 20 logs reais de partida, o passo entre amostras vizinhas foi:
 *
 *   motor parado, chave ligada   1 contagem  (24 mV)  <- piso de ruido
 *   arranque estavel             4 contagens (96 mV)
 *   marcha lenta estavel         7 contagens (168 mV)
 *
 * Ou seja: a cadeia esta limpa, e a lenta e MAIS ruidosa que o arranque, por
 * causa do chaveamento de bobina e bico e do ripple do alternador. Nada disso
 * existe durante a captura, porque ignicao e injecao ficam suprimidas - a
 * medida roda na condicao eletrica mais silenciosa que o carro tem.
 *
 * O que aqueles logs NAO provam, e por isso este modo e captura crua e nao
 * laudo: eles vem a 5 Hz, abaixo de Nyquist para um evento de 9 Hz. Eles
 * mostram que o sinal chega ao pino e que o piso e 24 mV, mas nao medem a
 * diferenca entre um cilindro bom e um fraco. So amostragem rapida responde -
 * e ate olhar a forma de onda de um motor real nao ha por que escolher
 * algoritmo de comparacao nenhum.
 *
 * POR QUE CRU, E NAO MEDIA PRONTA
 *
 * A licao do teste de compressao por dente: num arranque real de 4 voltas os
 * tres eventos deram 100/71/54, 100/76/73, 100/67/73 e 77/75/100 - ate QUEM
 * era o pior mudava de volta para volta. Media sem dispersao teria mandado
 * abrir o motor errado. Guardar a forma de onda inteira deixa esse tipo de
 * coisa visivel na tela antes de virar veredito.
 *
 * Por isso tambem os 500 Hz em vez de 1 kHz: com a mesma RAM, meio da taxa
 * cobre o dobro do tempo. 4 segundos sao ~6 ciclos completos a 180 rpm, e
 * consistencia ciclo a ciclo importa mais aqui que resolucao dentro do vale.
 */

#if TESTE_COMPRESSAO

// 500 Hz. A 180 rpm num 6 cilindros a compressao acontece a ~9 Hz, entao sao
// uns 55 pontos por evento - de sobra para desenhar o vale.
#define CAPTURA_BATERIA_INTERVALO_US 2000UL

// 2000 amostras = 4,0 s = ~6 ciclos completos a 180 rpm.
#define CAPTURA_BATERIA_MAX_AMOSTRAS 2000

// Marcas de dente, para a UI mapear amostra -> angulo. Numa 60-2 a 180 rpm sao
// 174 dentes por segundo e as 256 marcas cobrem 1,5 s; numa 12-1 no comando
// sao 16,5 por segundo e elas cobrem a captura inteira. O que acabar primeiro
// para de gravar, e o despejo diz quantas de cada coisa entraram.
#define CAPTURA_BATERIA_MAX_MARCAS 256

// Mesmo desarme automatico do teste de compressao: se a UI travar ou o cabo
// cair, o motor volta a funcionar sozinho.
#define CAPTURA_BATERIA_TIMEOUT_MS 20000UL

// Faixa de arranque. Abaixo do minimo o motor nem esta girando; acima do maximo
// ele pegou, e combustao mascara a compressao.
#define CAPTURA_BATERIA_RPM_MIN 30
#define CAPTURA_BATERIA_RPM_MAX 600

// Acima disso o despejo e recusado: sao milhares de bytes a 9600 baud, e o
// Serial.write trava o laco quando o buffer de 64 enche. Com o motor girando
// isso viraria centelha atrasada. Parado nao custa nada.
#define CAPTURA_BATERIA_RPM_DESPEJO 50

#define CAPTURA_BATERIA_PARADA 0
#define CAPTURA_BATERIA_ARMADA 1
#define CAPTURA_BATERIA_GRAVANDO 2
#define CAPTURA_BATERIA_CHEIA 3

volatile bool captura_bateria_ativa = false;
volatile uint8_t captura_bateria_estado = CAPTURA_BATERIA_PARADA;

// As amostras vao como desvio de 8 bits em torno de uma referencia tomada no
// inicio da gravacao, nao como valor absoluto de 10 bits. Guardar os 10 bits
// custaria o dobro da RAM para cobrir o mesmo tempo, e o que interessa aqui e
// a ONDULACAO sobre o patamar de arranque, nao o patamar. A janela de +-127
// contagens vale 3,04 V, muito mais que a variacao dentro de uma captura - e
// se ainda assim estourar, o contador de clipes avisa em vez de mentir.
uint8_t captura_bateria_amostras[CAPTURA_BATERIA_MAX_AMOSTRAS];

// Cada marca e o indice da amostra em que o dente passou, com o bit 15 ligado
// quando aquele dente e o primeiro da volta da roda. Como o indice vai no
// maximo a 2000, o bit 15 sobra: a UI ganha o alinhamento de volta de graca,
// sem um segundo vetor.
volatile uint16_t captura_bateria_marcas[CAPTURA_BATERIA_MAX_MARCAS];

volatile uint16_t captura_bateria_n = 0;
volatile uint16_t captura_bateria_m = 0;
uint16_t captura_bateria_base = 0;
uint16_t captura_bateria_clipes = 0;
unsigned long captura_bateria_inicio_ms = 0;
unsigned long captura_bateria_proxima_us = 0;

static void captura_bateria_iniciar() {
  captura_bateria_n = 0;
  captura_bateria_m = 0;
  captura_bateria_base = 0;
  captura_bateria_clipes = 0;
  captura_bateria_inicio_ms = millis();
  captura_bateria_estado = CAPTURA_BATERIA_ARMADA;
  captura_bateria_ativa = true;
}

static void captura_bateria_parar() {
  captura_bateria_ativa = false;
  if (captura_bateria_estado == CAPTURA_BATERIA_ARMADA ||
      captura_bateria_estado == CAPTURA_BATERIA_GRAVANDO) {
    captura_bateria_estado = (captura_bateria_n > 0) ? CAPTURA_BATERIA_CHEIA
                                                     : CAPTURA_BATERIA_PARADA;
  }
}

// Chamada de dentro da ISR do dente, ao lado do registro do teste de
// compressao. Custa um teste de bandeira no caminho normal.
static inline void captura_bateria_marcar_dente() {
  if (captura_bateria_estado != CAPTURA_BATERIA_GRAVANDO) {
    return;
  }
  if (captura_bateria_m >= CAPTURA_BATERIA_MAX_MARCAS) {
    return;
  }
  uint16_t indice = captura_bateria_n;
  if (indice > 0x7FFF) {
    return;
  }
  if (qtd_leitura == 1) {
    indice |= 0x8000;
  }
  captura_bateria_marcas[captura_bateria_m] = indice;
  captura_bateria_m++;
}

// Chamada do laco principal. A 1835 passagens por segundo o laco visita esta
// funcao a cada 545 us, entao a amostra de 2000 us sai com no maximo meia
// passagem de atraso - 0,5% do vale de 111 ms que se quer desenhar.
static void captura_bateria_processar() {
  if (!captura_bateria_ativa) {
    return;
  }
  if ((millis() - captura_bateria_inicio_ms) >= CAPTURA_BATERIA_TIMEOUT_MS) {
    captura_bateria_parar();
    return;
  }

  if (captura_bateria_estado == CAPTURA_BATERIA_ARMADA) {
    // So comeca com o decoder sincronizado: sem sincronismo as marcas de dente
    // nao tem posicao conhecida e o vetor de amostras perde a serventia.
    if (revolucoes_sincronizada >= 1 &&
        rpm >= CAPTURA_BATERIA_RPM_MIN && rpm <= CAPTURA_BATERIA_RPM_MAX) {
      captura_bateria_base = (uint16_t)analogRead(pino_sensor_brv);
      captura_bateria_proxima_us = micros();
      captura_bateria_estado = CAPTURA_BATERIA_GRAVANDO;
    }
    return;
  }

  if (captura_bateria_estado != CAPTURA_BATERIA_GRAVANDO) {
    return;
  }

  const unsigned long agora = micros();
  if ((long)(agora - captura_bateria_proxima_us) < 0) {
    return;
  }
  // Somar o passo em vez de reancorar em micros() evita deriva. Mas se o laco
  // atrasar muito (uma passada longa qualquer), reancora em vez de disparar
  // uma rajada para "recuperar" o atraso, que deformaria a forma de onda.
  captura_bateria_proxima_us += CAPTURA_BATERIA_INTERVALO_US;
  if ((long)(agora - captura_bateria_proxima_us) > (long)(4UL * CAPTURA_BATERIA_INTERVALO_US)) {
    captura_bateria_proxima_us = agora + CAPTURA_BATERIA_INTERVALO_US;
  }

  int valor = analogRead(pino_sensor_brv);
  int desvio = valor - (int)captura_bateria_base;
  if (desvio > 127) {
    desvio = 127;
    captura_bateria_clipes++;
  } else if (desvio < -127) {
    desvio = -127;
    captura_bateria_clipes++;
  }

  uint16_t indice = captura_bateria_n;
  captura_bateria_amostras[indice] = (uint8_t)(desvio + 128);
  indice++;
  // A ISR do dente le este contador para carimbar a marca. Escrita de 16 bits
  // no AVR sao duas instrucoes: sem a trava, um dente que caisse no meio dela
  // leria meio indice velho e meio novo e a marca iria parar longe do lugar.
  const uint8_t sreg = SREG;
  cli();
  captura_bateria_n = indice;
  SREG = sreg;

  if (indice >= CAPTURA_BATERIA_MAX_AMOSTRAS ||
      captura_bateria_m >= CAPTURA_BATERIA_MAX_MARCAS) {
    captura_bateria_estado = CAPTURA_BATERIA_CHEIA;
  }
}

static inline void captura_bateria_hex8(uint8_t v) {
  static const char digitos[] = "0123456789ABCDEF";
  Serial.write(digitos[(v >> 4) & 0x0F]);
  Serial.write(digitos[v & 0x0F]);
}

// Resposta:
//   ;G,<estado>,<base_adc>,<n_amostras>,<intervalo_us>,<clipes>,<n_marcas>,
//      M<4 digitos hex por marca>,A<2 digitos hex por amostra>;
//
// Hex em vez de decimal separado por virgula porque o bloco e grande: 2000
// amostras em decimal passariam de 8000 caracteres, em hex sao 4000. A 9600
// baud isso e a diferenca entre 8 e 4 segundos de despejo.
static void captura_bateria_enviar() {
  Serial.write(';');
  Serial.write('G');
  Serial.write(',');

  // Com o motor girando o despejo travaria o laco por segundos e viraria
  // centelha atrasada. Devolve estado 255 para a UI dizer o motivo.
  if (rpm > CAPTURA_BATERIA_RPM_DESPEJO) {
    sendSerialInt(255);
    Serial.write(',');
    sendSerialInt((int)rpm);
    Serial.write(';');
    return;
  }

  sendSerialInt((int)captura_bateria_estado);
  Serial.write(',');
  sendSerialInt((int)captura_bateria_base);
  Serial.write(',');
  sendSerialInt((int)captura_bateria_n);
  Serial.write(',');
  sendSerialInt((int)(CAPTURA_BATERIA_INTERVALO_US));
  Serial.write(',');
  sendSerialInt((int)captura_bateria_clipes);
  Serial.write(',');
  sendSerialInt((int)captura_bateria_m);
  Serial.write(',');

  Serial.write('M');
  for (uint16_t i = 0; i < captura_bateria_m; i++) {
    const uint16_t marca = captura_bateria_marcas[i];
    captura_bateria_hex8((uint8_t)(marca >> 8));
    captura_bateria_hex8((uint8_t)(marca & 0xFF));
  }

  Serial.write(',');
  Serial.write('A');
  for (uint16_t i = 0; i < captura_bateria_n; i++) {
    captura_bateria_hex8(captura_bateria_amostras[i]);
  }

  Serial.write(';');
}

#define CAPTURA_BATERIA_SUPRIME (captura_bateria_ativa)

#else // TESTE_COMPRESSAO

#define CAPTURA_BATERIA_SUPRIME (false)
#define captura_bateria_marcar_dente() ((void)0)
#define captura_bateria_processar() ((void)0)

#endif // TESTE_COMPRESSAO

#endif // CAPTURA_BATERIA_H
