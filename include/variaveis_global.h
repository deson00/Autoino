byte tipo_ignicao = 1;//1 roda fonica e 2 distribuidor
byte qtd_dente = 12; //60 
byte qtd_dente_faltante = 1; //2
byte local_rodafonica = 2; // 2 para virabrequinho e 1 para comando
byte qtd_cilindro = 6 / local_rodafonica;
int grau_pms = 60;
volatile unsigned long dwell_bobina = 3;
int dwell_partida = 4;
int dwell_funcionamento = 3;

byte tipo_ignicao_sequencial = 0;// sequencial 1 semi-sequencial 0
volatile unsigned int qtd_voltas = 0;
byte grau_cada_dente = 360 / qtd_dente;
byte grau_avanco = 0;
byte grau_avanco_partida = 1; // avanço definido apenas na partida
byte grau_entre_cada_cilindro = 360 / qtd_cilindro;
int posicao_atual_sensor = 0;
volatile unsigned int leitura = 0;
volatile unsigned int qtd_leitura_media = 0;
byte qtd_leitura = 0;
byte referencia_leitura_ignicao = 1;//map 1 e tps 2
byte referencia_leitura_injecao = 1;//map 1 e tps 2
byte modo_ignicao = 1; // 1 para centelha perdida e 2 para centelha unica  
byte avanco_fixo = 0; // avanço fixo 0 desligado e 1 ligado
byte grau_avanco_fixo = 0; // grau de avanço fixo de 0 a 360 mais usado para calibrar o pms
byte tipo_sinal_bobina = 1 ; // 1 alto e 0 baixo tipo de sinal enviado para bobina ente alto ou baixo conforme modelo da bobina

volatile unsigned long tempo_anterior = 0;
volatile unsigned long tempo_dente_anterior[2] = {0,0};
volatile unsigned long tempo_inicio_volta_completa = 0;
volatile unsigned long tempo_final_volta_completa = 0;
volatile unsigned long tempo_total_volta_completa = 0;
volatile unsigned long tempo_cada_grau = 0;
volatile unsigned long tempo_proxima_ignicao[8];
volatile unsigned long tempo_proxima_injecao[8];
volatile unsigned long tempo_atual = 0;
volatile unsigned long tempo_atual_proxima_ignicao[8];
volatile unsigned long tempo_atual_proxima_injecao[8];
volatile unsigned long intervalo_tempo_entre_dente = 0;
volatile unsigned long verifica_falha = 0;
unsigned long tempo_check = 0;
byte inicia_tempo_sensor_roda_fonica = 1;
volatile long revolucoes_sincronizada = 0;
int qtd_revolucoes = 0;
byte qtd_perda_sincronia = 0;
int qtd_loop = 0;
int loop_timer = 0; //variavel para ser incrementada a cada chamada da função timer
int loop_timer2 = 0; 
int verifica_posicao_sensor = 0;
byte intervalo_execucao = 200; // intervalo em milissegundos
unsigned long ultima_execucao = 0;       // variável para armazenar o tempo da última execução
unsigned long tempo_inicial_rpm; // Variáveis para registrar o tempo inicial do rpm
unsigned long tempo_final_rpm;  // Variáveis para registrar o tempo final do rpm
volatile unsigned int rpm = 0;
volatile int rpm_anterior = 0;
unsigned int rpm_partida = 400;
byte ignicao_pins[] = {ign1, ign2, ign3, ign4, ign1, ign2, ign3, ign4}; // Array com os pinos de ignição
byte injecao_pins[] = {inj1, inj2, inj3, inj4, inj1, inj2, inj3, inj4}; // Array com os pinos de injecao
// Declare as variáveis para controlar o estado do pino de saída
volatile bool captura_dwell[8] = {false, false, false, false, false, false, false, false};
volatile bool ign_acionado[8] = {false, false, false, false, false, false, false, false};
volatile bool captura_req_fuel[8] = {false, false, false, false, false, false, false, false};
volatile bool inj_acionado[8] = {false, false, false, false, false, false, false, false};
volatile unsigned long tempo_percorrido[8];
volatile unsigned long tempo_percorrido_inj[8];
//volatile bool flag_interrupcao = false;
unsigned long tempo_inicial_codigo, tempo_final_codigo, tempo_decorrido_codigo;
// variaveis reverente a entrada de dados pela serial
const int maximo_valores_recebido = 30; // tamanho máximo de dados recebido do vetor ou matriz
int values[maximo_valores_recebido];     // vetor para armazenar os valores recebidos
int matriz_ve_count = 0;
int matriz_avanco_count = 0;
byte matriz_avanco[16][16];
byte matriz_ve[16][16];
byte matriz_lambda[16][16];
int vetor_map_tps[16];
int vetor_rpm[16];
int vetor_map_tps_ve[16];
int vetor_rpm_ve[16];
byte vetor_avanco_temperatura[5];
byte vetor_temperatura[5];
int indice_vetor_entrada_dados_serial = 0;
int indice_matrix_entrada_dados_seriala = 0; // Índice usado para rastrear a linha atual na matriz
int indice_matrix_entrada_dados_serialb = 0; // Índice usado para rastrear a coluna atual na matriz
int index = 0;   // índice atual do vetor
int indice_envio = 0;   // índice atual do vetor de envio
char buffer[6]; // buffer temporário para armazenar caracteres recebidos
byte tipo_vetor_map_tps_avanco = 0;
byte tipo_vetor_rpm_avanco = 0;
byte tipo_vetor_matriz_avanco = 0;
byte tipo_vetor_map_tps_ve = 0;
byte tipo_vetor_rpm_ve = 0;
byte tipo_vetor_matriz_ve = 0;
byte tipo_vetor_configuracao_inicial = 0;
byte tipo_vetor_configuracao_faisca = 0;
byte tipo_vetor_configuracao_dwell = 0;
byte tipo_vetor_configuracao_clt = 0;
byte tipo_vetor_configuracao_injecao = 0;
byte tipo_vetor_protecao = 0;
byte tipo_vetor_enriquecimento_aceleracao = 0;
byte tipo_vetor_configuracao_tps = 0;
byte tipo_vetor_configuracao_map = 0;
bool status_dados_tempo_real = false;
//int leituras_map[10]={0};
//int leituras_tps[10]={0};
//int contador_leitura = 0;
int valor_map = 0;
byte valor_map_tipo;
int valor_map_minimo = 10;
int valor_map_maximo = 100;
int valor_tps = 0;
int valor_tps_adc = 0;
int valor_tps_minimo = 0;
int valor_tps_maximo = 1023;
int valor_referencia_busca_avanco = 0;
int valor_referencia_busca_tempo_injecao = 0;
int ajuste_pms =  0;
int busca_avanco_linear = true;
int referencia_temperatura_clt1 = 20;
int referencia_resistencia_clt1 = 2500;
int referencia_temperatura_clt2 = 100;
int referencia_resistencia_clt2 = 187;
byte temperatura_motor = 0;
int grau_fechamento_injetor = 0; //este grau tem referencia ao apms
int deslocamento_motor = 250;//polegadas cubicas
int numero_cilindro_injecao = 4;
int numero_injetor = 4;//qtd de injetores total
int numero_esguicho = 4;
int tamanho_injetor = 32;// lbs/hora por injetor
byte tipo_acionamento_injetor = 1;// 1 - simultaneo 2 alternado
int tipo_combustivel = 14700; // 14700 - Gasolina
byte tipo_motor = 4;// 4 - motor 4 tempo, 2 - motor 2 tempo
byte modo_injecao = 1; // 1 - pareado, 2 semi-sequencial, 3 - sequencial
byte emparelhar_injetor = 1; // 1 - para 1 e 4 | 2 e 3, 2 - para 1 e 3 | 2 e 4
byte limite_injetor = 100; // 90% valor em porcentagem
int tempo_abertura_injetor = 0;// Dead time, tempo que o injetor leva para abrir
byte acrescimo_injecao_partida = 50;// valor de acrecimo da ve na partida em porcentagem 
byte acrescimo_injecao_funcionamento = 0;// valor em porcentagem acrecimo da ve
int REQ_FUEL = 10000; //em ms
int dreq_fuel = 10000;//em ms
int VE = 0;
int GammaE = 100;
unsigned long tempo_injecao = 0;
byte temperatura_trabalho = 70;
byte correcao_maxima_temperatura = 50; // % de enriquecimento a 0 graus
int tipo_protecao = 1; // 0 desligado, 1 apenas ignição, 2 apenas injeção e 3 ignição e injeção
int rpm_pre_corte = 6000;
int avanco_corte = 20; //graus
int tempo_corte = 2;//em segundos 
int rpm_maximo_corte = 6500;
int numero_base_corte = 10;
int qtd_corte = 3;
int status_corte = 0;
int tps_anterior = 0;   // Variável para armazenar o valor anterior do sensor de TPS
byte tipo_verificacao_aceleracao_rapida = 0; //0 para TPS e  1 para MAP 
int intervalo_tempo_aceleracao = 100; // valor em ms
int duracao_enriquecimento = 5; // valor em ms
unsigned long tempo_anterior_aceleracao = 0;  // Variável para armazenar o tempo anterior de leitura do sensor
unsigned long tempo_ultima_mudanca = 0; // Tempo da última mudança significativa no TPS
float tps_dot_porcentagem_aceleracao = 0;
float tps_dot_porcentagem_desaceleracao = 0;
float tps_mudanca_minima = 5; // Threshold para detectar mudança significativa no TPS
byte enriquecimento_aceleracao[5] = {50, 100, 150, 200, 250}; // Quantidade de enriquecimento de combustível em porcentagem
int tps_dot_escala[5] = {200, 400, 600, 800, 1000}; //escala de velocidade no acionamento do tps 
int rpm_minimo_enriquecimento = 1000;
int rpm_maximo_enriquecimento = 5000;
int enriquecimento_desaceleracao = 0; // Quantidade de redução de combustível em porcentagem
int valor_o2 = 0;
int sonda_o2 = 0;
bool tipo_sonda_o2 = 1; // 0 para narrow band e 1 para wide band