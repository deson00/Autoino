#ifndef TESTE_COMPRESSAO_H
#define TESTE_COMPRESSAO_H

/*
 * Teste de compressao relativa pela roda fonica.
 *
 * O QUE ELE MEDE
 *
 * Em arranque, cada compressao freia o virabrequim e a expansao seguinte o
 * solta. Cilindro com compressao baixa freia menos. Como a ECU ja cronometra
 * todo dente, basta acumular o tempo de cada dente ao longo de varias voltas e
 * comparar a profundidade das frenagens: nao precisa de sensor novo, nem de
 * leitura rapida de bateria, nem de tocar em hardware.
 *
 * Medido em motor real (60-2 no virabrequim, 6 cilindros, arranque a 200 rpm):
 * a velocidade do virabrequim varia +-15% ao longo da volta, com tres vales
 * claros a 120 graus um do outro. O sinal e enorme - nao e um efeito sutil que
 * precise de filtro esperto para aparecer.
 *
 * POR QUE O FIRMWARE SO ACUMULA
 *
 * Achar os vales, descontar o gap, comparar os cilindros e montar o laudo sao
 * contas que cabem folgadas na UI, onde nao ha 2 KB de RAM nem disputa com a
 * ISR do dente. Aqui fica so o que SO pode ser feito aqui: somar o tempo de
 * cada dente na hora em que ele passa. O firmware devolve um vetor de medias
 * por posicao de dente e a UI faz o resto.
 *
 * Essa divisao tambem e o que mantem o custo em flash baixo o bastante para um
 * dia caber na Nano - hoje ela tem 136 bytes livres, entao a funcao nasce so
 * na Mega do shield Speeduino, atras da flag TESTE_COMPRESSAO.
 *
 * SEGURANCA
 *
 * Enquanto o teste roda, ignicao e injecao ficam suprimidas: o motor nao pode
 * pegar no meio da medida, e combustao arruinaria a leitura. Por isso existe um
 * tempo limite - se a UI travar ou o cabo cair, o modo se desarma sozinho e o
 * motor volta a funcionar normalmente.
 */

#if TESTE_COMPRESSAO

// Maior roda suportada. O indice vem de qtd_leitura, que comeca em 1, entao
// 60 posicoes cobrem qualquer roda com gap ate 59 dentes efetivos - a 60-2 usa
// 58. Roda sem gap nenhum de 60 dentes perderia o ultimo, mas o decoder precisa
// de gap para sincronizar, entao esse caso nao existe.
#define TESTE_COMPRESSAO_MAX_DENTES 60

// Dieta de RAM, para um dia caber na Uno e na Nano.
//
// O que os vetores custavam antes: soma em uint32 (256 B), soma dos quadrados
// em uint32 (256 B) e contagem em uint16 (128 B), num total de 645 bytes com a
// flag e o relogio. A Uno e a Nano tem 405 livres - faltavam 240.
//
// Tres cortes e uma troca:
//
//   soma      uint32 -> uint16, guardando o intervalo dividido por 32
//   contagem  uint16 -> uint8
//   quadrados guardados a cada 4 dentes, nao em todos
//
// O ultimo e o unico que custa alguma coisa: perde-se dizer "ESTE dente tem
// leitura instavel". O nivel de ruido continua medido, porque ele e do conjunto
// sensor mais fiacao e nao de um dente especifico - e e o nivel de ruido que
// trava o veredito falso, que era a parte que importava.
#define TESTE_COMPRESSAO_ESCALA 5
#define TESTE_COMPRESSAO_PASSO_VARIANCIA 4
#define TESTE_COMPRESSAO_SLOTS_VARIANCIA (TESTE_COMPRESSAO_MAX_DENTES / TESTE_COMPRESSAO_PASSO_VARIANCIA)

// Teto de voltas por posicao. Nao e limite de uso - a 200 rpm numa 60-2 o
// teste inteiro de 20 s da umas 66 voltas.
#define TESTE_COMPRESSAO_VOLTAS_MAX 250

// Desarme automatico. Vinte segundos e muito mais que os ~5 s de arranque que
// o teste pede, e curto o bastante para ninguem ficar sem ignicao sem entender
// por que.
#define TESTE_COMPRESSAO_TIMEOUT_MS 20000UL

// Acima disso o motor pegou (ou o teste foi pedido com o motor ligado) e a
// amostra nao serve: combustao mascara a compressao.
#define TESTE_COMPRESSAO_RPM_MAX 600

volatile bool teste_compressao_ativo = false;
volatile uint16_t teste_compressao_soma[TESTE_COMPRESSAO_MAX_DENTES];
// Soma dos quadrados, para a UI poder calcular o espalhamento volta a volta.
//
// Sem isso o laudo mostra so a media, e media sem barra de erro engana: no log
// real de um arranque de 4 voltas os tres eventos deram 100/71/54, 100/76/73,
// 100/67/73 e 77/75/100 - ou seja, ate QUEM era o pior mudava de volta para
// volta. Quem olhasse so a media iria trocar junta de um cilindro que estava
// bom. A dispersao e o que impede esse erro.
//
// Guardo o intervalo dividido por 32 antes de elevar ao quadrado: a 5000 us um
// dente vira 156, o quadrado 24 mil, e duzentas voltas cabem folgado em 32
// bits. A resolucao perdida e de 32 us em 5000, 0,6% - irrelevante diante de
// um sinal de 15%.
//
// Guardado a cada 4 dentes: quinze pontos descrevem o nivel de ruido do
// conjunto, que e o que a UI precisa para calcular a incerteza.
volatile uint32_t teste_compressao_soma_q[TESTE_COMPRESSAO_SLOTS_VARIANCIA];
volatile uint8_t teste_compressao_amostras[TESTE_COMPRESSAO_MAX_DENTES];
unsigned long teste_compressao_inicio_ms = 0;

static void teste_compressao_zerar() {
  for (byte i = 0; i < TESTE_COMPRESSAO_MAX_DENTES; i++) {
    teste_compressao_soma[i] = 0;
    teste_compressao_amostras[i] = 0;
  }
  for (byte i = 0; i < TESTE_COMPRESSAO_SLOTS_VARIANCIA; i++) {
    teste_compressao_soma_q[i] = 0;
  }
}

static void teste_compressao_iniciar() {
  teste_compressao_zerar();
  teste_compressao_inicio_ms = millis();
  teste_compressao_ativo = true;
}

static void teste_compressao_parar() {
  teste_compressao_ativo = false;
}

// Chamada de dentro da ISR do dente. Precisa ser barata: um indice, uma soma
// de 32 bits e um incremento de 16. A 200 rpm numa 60-2 sao 193 dentes por
// segundo, entao o custo e desprezivel - mas o teste de bandeira vem primeiro
// justamente para que o caminho normal, com o teste desligado, custe so uma
// comparacao.
static inline void teste_compressao_registrar(uint16_t indice, unsigned long intervalo_us) {
  if (!teste_compressao_ativo) {
    return;
  }
  if (indice >= TESTE_COMPRESSAO_MAX_DENTES) {
    return;
  }
  // Intervalo absurdo (primeiro dente depois de parar, ou ISR muito atrasada)
  // so sujaria a media.
  if (intervalo_us == 0 || intervalo_us > 200000UL) {
    return;
  }
  const uint16_t reduzido = (uint16_t)(intervalo_us >> TESTE_COMPRESSAO_ESCALA);
  if (reduzido == 0) {
    return;
  }
  if (teste_compressao_amostras[indice] >= TESTE_COMPRESSAO_VOLTAS_MAX) {
    return;
  }
  // Para de somar antes de estourar em vez de depois. Com arranque muito lento
  // o dente fica longo e a soma sobe rapido; melhor perder as ultimas voltas do
  // que virar a conta e devolver um numero sem sentido.
  if (teste_compressao_soma[indice] > (uint16_t)(65535U - reduzido)) {
    return;
  }
  teste_compressao_soma[indice] += reduzido;
  if ((indice % TESTE_COMPRESSAO_PASSO_VARIANCIA) == 0) {
    const byte slot = (byte)(indice / TESTE_COMPRESSAO_PASSO_VARIANCIA);
    if (slot < TESTE_COMPRESSAO_SLOTS_VARIANCIA) {
      teste_compressao_soma_q[slot] += (uint32_t)reduzido * reduzido;
    }
  }
  teste_compressao_amostras[indice]++;
}

// Desarme por tempo, chamado do laco principal.
static void teste_compressao_verificar_tempo() {
  if (!teste_compressao_ativo) {
    return;
  }
  if ((millis() - teste_compressao_inicio_ms) >= TESTE_COMPRESSAO_TIMEOUT_MS) {
    teste_compressao_parar();
  }
}

// Resposta: ;E,<qtd_dente>,<qtd_dente_faltante>,<local_rodafonica>,<ativo>,
//            <voltas>,<media_us_0>,<desvio_us_0>,<media_us_1>,<desvio_us_1>,...;
//
// Media em microssegundos por posicao de dente, nao a soma: a 200 rpm numa
// 60-2 o dente leva ~5200 us, entao a media cabe num inteiro e o pacote fica
// pequeno. A UI conhece a roda e sabe qual posicao e o gap.
static void teste_compressao_enviar() {
  Serial.write(';');
  Serial.write('E');
  Serial.write(',');
  sendSerialInt((int)qtd_dente);
  Serial.write(',');
  sendSerialInt((int)qtd_dente_faltante);
  Serial.write(',');
  sendSerialInt((int)local_rodafonica);
  Serial.write(',');
  sendSerialInt(teste_compressao_ativo ? 1 : 0);
  Serial.write(',');

  // Quantas voltas entraram na conta: a maior contagem entre as posicoes.
  uint8_t voltas = 0;
  for (byte i = 0; i < TESTE_COMPRESSAO_MAX_DENTES; i++) {
    if (teste_compressao_amostras[i] > voltas) {
      voltas = teste_compressao_amostras[i];
    }
  }
  sendSerialInt((int)voltas);

  byte posicoes = (byte)qtd_dente;
  if (posicoes > TESTE_COMPRESSAO_MAX_DENTES) {
    posicoes = TESTE_COMPRESSAO_MAX_DENTES;
  }
  for (byte i = 0; i < posicoes; i++) {
    const uint8_t n = teste_compressao_amostras[i];
    uint16_t media = 0;
    uint16_t desvio = 0;
    uint32_t mr = 0;
    if (n > 0) {
      mr = (uint32_t)teste_compressao_soma[i] / n;     // media na escala reduzida
      const uint32_t m = mr << TESTE_COMPRESSAO_ESCALA;
      media = (m > 65535UL) ? 65535U : (uint16_t)m;
    }
    // Desvio so nas posicoes que guardam quadrados; nas outras vai zero, e a UI
    // ja trata zero como "nao medido" em vez de "sem variacao".
    if (n > 1 && (i % TESTE_COMPRESSAO_PASSO_VARIANCIA) == 0) {
      const byte slot = (byte)(i / TESTE_COMPRESSAO_PASSO_VARIANCIA);
      if (slot < TESTE_COMPRESSAO_SLOTS_VARIANCIA) {
        const uint32_t mq = teste_compressao_soma_q[slot] / n;
        const uint32_t var = (mq > mr * mr) ? (mq - mr * mr) : 0UL;
        uint32_t d = 0;
        // Raiz inteira por bisseccao: sem ponto flutuante, que este firmware
        // nao carrega mais.
        for (uint32_t bit = 1UL << 15; bit; bit >>= 1) {
          const uint32_t t = d | bit;
          if (t * t <= var) d = t;
        }
        d <<= TESTE_COMPRESSAO_ESCALA;
        desvio = (d > 65535UL) ? 65535U : (uint16_t)d;
      }
    }
    Serial.write(',');
    sendSerialInt((int)media);
    Serial.write(',');
    sendSerialInt((int)desvio);
  }
  Serial.write(';');
}

#define TESTE_COMPRESSAO_SUPRIME (teste_compressao_ativo || CAPTURA_BATERIA_SUPRIME)

#else  // TESTE_COMPRESSAO

// Com a funcao desligada tudo vira zero em tempo de compilacao: os binarios da
// Nano, da Uno e da Mega Autoino ficam identicos aos de antes.
#define TESTE_COMPRESSAO_SUPRIME (false)
#define teste_compressao_registrar(indice, intervalo) ((void)0)
#define teste_compressao_verificar_tempo() ((void)0)

#endif // TESTE_COMPRESSAO

#endif // TESTE_COMPRESSAO_H
