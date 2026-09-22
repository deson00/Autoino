# Autoino ECU

Firmware de ignição e injeção eletrônica com configuração pela interface AutoinoUI.

> Projeto em desenvolvimento. Antes de utilizar em um motor, valide sensores,
> sincronismo, limites de proteção e acionamentos em bancada.

## Compilação

O projeto utiliza PlatformIO. O ambiente padrão é `nanoatmega328_old`:

```bash
platformio run
```

Para gravar na placa configurada em `platformio.ini`:

```bash
platformio run --target upload
```

## Pinagem

Os valores abaixo saem de `include/definicoes_hardware.h`, que é a única fonte
de verdade. Se esta tabela divergir do arquivo, o arquivo está certo.

| função | Autoino (Nano/Uno) | Speeduino (Mega) |
|---|---|---|
| sensor da roda fônica | `D2` | `D19` |
| sensor de fase — declarado, ainda sem uso | `D3` | `D18` |
| MAP | `A0` | `A3` |
| TPS | `A1` | `A2` |
| CLT | `A2` | `A1` |
| IAT | `A3` | `A0` |
| sonda O2 | `A4` | `A8` |
| tensão de bateria | `A6` | `A4` |
| flex | `A5` | `A6` |
| pressão de óleo | `A7` | `A7` |
| IGN 1 | `D4` | `D40` |
| IGN 2 | `D5` | `D38` |
| IGN 3 | `D6` | `D52` |
| IGN 4 | `D7` | `D50` |
| INJ 1 | `D8` | `D8` |
| INJ 2 | `D9` | `D9` |
| INJ 3 | `D12` | `D10` |
| INJ 4 | `D10` | `D11` |
| marcha lenta: PWM ou liga-desliga | `D11` | `D5` |
| marcha lenta: STEP | `D11` | `D17` |
| marcha lenta: DIR | `D13` | `D16` |

### Perfil de hardware

Quem escolhe é o `build_flags` do `platformio.ini`, um `-D` por ambiente: os
ambientes de Nano e Uno passam `-D Autoino`, o de Mega com shield Speeduino
passa `-D Speeduino`. Fora do PlatformIO há um fallback por MCU, para quem
compila pela IDE do Arduino.

### Revisão de placa

`PLACA_REVISAO` escolhe a pinagem, não é rótulo:

| | INJ 4 | marcha lenta DIR | flex | bateria |
|---|---|---|---|---|
| v1 | `D13` | `D10` | `A6` | `A5` |
| **v2** (padrão) | `D10` | `D13` | `A5` | `A6` |

Compilar sem flag nenhuma dá a revisão atual. Para gerar a anterior, acrescente
`-D PLACA_REVISAO=1` ao `build_flags` de um ambiente próprio.

A ECU informa para qual revisão foi compilada na seção `V` da resposta do
comando `h;` — veja *Dados recebidos da ECU*. Ela diz o que o **firmware**
espera encontrar, e não o que a **placa** é: numa v1 rodando firmware v2 o
campo continua dizendo v2, corretamente.

**Placa v1 com firmware v2** funciona, desde que o INJ 4 não seja usado — é o
caso das placas de desenvolvimento. A exceção é a marcha lenta por motor de
passo (`modo_marcha_lenta = 3`): como o firmware dirige o DIR no `D13` e a
placa v1 tem o DIR no `D10`, o motor só anda para um lado. Marcha lenta por
PWM e liga/desliga não passam por esses pinos e não são afetadas. O conserto
na placa v1 é trocar dois fios, `D13` e `D10`.

### Por que o INJ 4 saiu do D13

No Nano e no Uno o `D13` é o `SCK` e o LED do bootloader: ele é **dirigido** a
cada reset e a cada gravação, cerca de 100 ms por pisco. Com bico ali, isso
despeja 20 a 50 vezes uma injetada normal (2 a 5 ms) com o motor parado.

Resistor de pulldown não resolve — o bootloader dirige o pino com o driver de
20 mA do próprio AVR, e um resistor em paralelo com isso é detalhe de consumo.
A única saída era tirar o atuador de lá.

O `D10` foi escolhido porque o `SS` não é tocado nem pelo bootloader nem por
ISP, e porque injetor não precisa de PWM: precisa de liga/desliga com instante
preciso, que o agendador faz por `digitalWrite`. O DIR do motor de passo foi
para o `D13` no lugar, por ser o sinal mais tolerante a ruído de boot de todo
o sistema — ele só diz o lado, e o motor só anda quando o STEP pulsa.

### Por que o flex e a bateria trocaram de pino

No 328P o `A6` e o `A7` são **só ADC**: não têm entrada digital nem interrupção
de mudança de pino. Um sensor de flex de **frequência** precisa das duas coisas,
então não podia ficar no `A6`. O `A5` tem `PCINT13` e serve. A tensão de bateria
é analógica pura e não perde nada num pino só de ADC, então assumiu o `A6`.

A placa v2 ganhou o **JP4 (PU FLEX)**, que liga um pull-up de 1 kΩ para 5 V:
fechado para sensor de frequência, aberto para sensor analógico. O pull-up é
externo, e o firmware não deve ligar o interno — e não liga: não há `pinMode`
para essa linha, e o padrão no reset já é entrada sem pull-up.

O filtro dessa linha passou para 1 nF e 10 nF, para não arredondar a onda
quadrada. Com sensor **analógico** isso deixa a leitura mais crua: **faça a média
no firmware**, porque não há mais capacitor segurando.

O pull-up de 2,4 kΩ da linha de óleo passou para trás do **JP5 (PU OLEO)**:
fechado para sensor resistivo ou pressostato, aberto para transdutor de 0–5 V de
três fios.

### Ressalvas

**Os analógicos trocam de lugar entre os perfis.** MAP é `A0` no Autoino e `A3`
no Speeduino; IAT e CLT invertem. Quem trocar de placa reaproveitando o chicote
lê temperatura no lugar de pressão.

**No perfil Autoino, o PWM da marcha lenta e o STEP do motor de passo são o
mesmo pino, `D11`.** É de propósito: usa-se um ou outro, nunca os dois. O
driver do atuador PWM está nesse pino.

**`A6` e `A7` não existem no Uno.** Valem no Nano, que é TQFP e tem ADC6 e
ADC7; o Uno é DIP e não tem. Compila, mas `analogRead` devolve lixo — a
telemetria de flex e de pressão de óleo não significa nada no ambiente `uno`.

**A marcha lenta por PWM não existe no ambiente `megaatmega2560_autoino`.** No
Mega o `D11` é `OC1A`, do Timer1, que pertence ao agendador de ignição, e usar
o Timer2 poria o PWM num pino onde a fiação não está. Liga/desliga e motor de
passo continuam funcionando, porque não dependem de timer.

**Não sobra I2C no perfil Autoino.** No 328P o `SDA` e o `SCL` são o `A4` e o
`A5`, ocupados pela sonda O2 e pelo flex. Qualquer periférico I2C exigiria
realocar um dos dois.

**A roda fônica precisa de pino de interrupção.** No 328P são só o `D2` e o
`D3`; no Mega, o `D18` e o `D19`. O `pino_sensor_fase` está declarado mas ainda
não é usado em lugar nenhum do código.

### Estado das saídas na partida

No reset todo pino do AVR volta a ser entrada de alta impedância. A primeira
coisa que o `setup()` faz é pôr as oito saídas em `LOW` e `OUTPUT`, antes de
ler a EEPROM — `HIGH` liga bobina e bico, então `LOW` é o estado seguro dos
dois.

Isso **não cobre a janela do bootloader**, que roda antes do `setup()` existir.
Essa parte é de hardware: toda saída de atuador deveria ter pulldown no gate do
driver, para o caso de o microcontrolador estar em reset, travado ou sem
alimentação com os drivers energizados.

### Guarda de colisão

`definicoes_hardware.h` termina com um `static_assert` para cada par de saída.
Duas funções no mesmo pino param a compilação com o nome das duas:

```text
error: static assertion failed: colisao de pino:
       pino_direcao_marcha_lenta e inj4 estao no mesmo pino
```

Colisão de pino não aparece em bancada testando uma função de cada vez; aparece
com a fiação ligada e o motor no lugar. `pino_marcha_lenta` e
`pino_passo_marcha_lenta` ficam fora da conta, porque compartilham pino por
projeto.

## Comunicação serial

- Velocidade: **9600 baud**.
- Configurações e tabelas usam texto ASCII.
- O monitoramento em tempo real usa o protocolo V2 com quadros binários fixos de 40 bytes.
- Os números são inteiros decimais; valores negativos são aceitos pelo parser.

### Formato dos comandos de configuração

```text
comando,valor1,valor2,...,valorN,;
```

Exemplo:

```text
k,4,3,;
```

Existe uma vírgula depois do último valor, antes de `;`. Ela é necessária para
que o parser atual armazene o último número antes de processar o comando.

Cada valor deve possuir no máximo cinco caracteres, incluindo o sinal, devido
ao tamanho atual do buffer numérico do firmware.

## Comandos enviados para a ECU

### Tabelas de ignição e injeção

| Comando | Conteúdo | Quantidade |
|---|---|---:|
| `a` | Eixo MAP/TPS da tabela de ignição | 16 |
| `b` | Eixo RPM da tabela de ignição | 16 |
| `c` | Matriz de avanço, linha por linha | 256 |
| `d` | Eixo MAP/TPS da tabela VE | 16 |
| `e` | Eixo RPM da tabela VE | 16 |
| `f` | Matriz VE, linha por linha | 256 |

Exemplos:

```text
a,1,16,20,26,30,36,40,46,50,56,66,74,80,88,96,100,;
b,500,700,1000,1500,2000,2500,3000,3500,4000,4500,5000,5500,6000,6500,7000,7500,;
```

As tabelas ficam na memória de trabalho. Envie `z;` para gravar as tabelas de
ignição e VE na EEPROM.

### Sensor sem dente de falha (`tipo_ignicao = 2`)

Modo para sensores em que **cada pulso já é uma referência angular**, sem gap
para procurar. Cobre dois casos que na prática são o mesmo:

- **Distribuidor com sensor hall** (ex.: Opala): um pulso por cilindro, sensor
  girando na rotação do comando.
- **Volante com um único dente** (ex.: Titan e motos em geral): um pulso por
  volta do virabrequim.

Como configurar:

| Campo | Distribuidor 6 cil | Volante 1 dente |
|---|---|---|
| `tipo_ignicao` | `2` | `2` |
| `qtd_dentes` | `6` (pulsos por volta do sensor) | `1` |
| `qtd_dentes_faltantes` | `0` | `0` |
| `local_roda` | `1` (comando) | `2` (virabrequim) |
| `grau_pms` | graus **do sensor** entre o pulso e o PMS | idem |

O `grau_pms` é medido em graus do **sensor**, não do virabrequim. Num
distribuidor, que gira na metade da rotação do motor, um captador a 60° APMS de
virabrequim corresponde a `grau_pms = 30`. O avanço configurado continua sendo
em graus de virabrequim — o firmware converte sozinho quando `local_roda = 1`.

Neste modo a ignição usa **uma única saída** (`ign1`): no distribuidor o
roteamento é mecânico, e na moto há uma bobina só, eventualmente dupla em
paralelo para motores gêmeos. Centelha perdida cobre os dois casos.

Limitações inerentes ao sensor, não ao firmware:

- **Não há como validar sincronismo.** Sem gap, não existe contagem de dentes
  para conferir; um pulso espúrio vira um evento espúrio, e a única defesa é o
  filtro de ruído.
- **Precisão angular menor.** Com 58 dentes o firmware corrige o ângulo dezenas
  de vezes por volta; com 1 dente ele extrapola a volta inteira a partir do
  intervalo anterior. Em partida e aceleração forte o erro de avanço é
  visivelmente maior.

### `g` — Configuração inicial

```text
g,tipo_ignicao,qtd_dentes,local_roda,qtd_dentes_faltantes,grau_pms,qtd_cilindros,;
```

| Campo | Valores/unidade |
|---|---|
| `tipo_ignicao` | `1` roda fônica com dente de falha, `2` sensor sem falha (ver abaixo) |
| `qtd_dentes` | Quantidade total de dentes |
| `local_roda` | `1` comando, `2` virabrequim |
| `qtd_dentes_faltantes` | Quantidade de dentes ausentes |
| `grau_pms` | Graus de referência do PMS |
| `qtd_cilindros` | Número de cilindros |

### `j` — Configuração da faísca

```text
j,referencia,modo,avanco_partida,avanco_fixo,grau_avanco_fixo,tipo_sinal,;
```

- `referencia`: `1` MAP, `2` TPS.
- `modo`: `1` centelha perdida, `2` centelha única.
- `avanco_fixo`: `0` desativado, `1` ativado.
- `tipo_sinal`: `0` nível baixo, `1` nível alto.

### `k` — Dwell

```text
k,dwell_partida,dwell_funcionamento,;
```

Os valores são informados em milissegundos.

### `l` — Calibração do sensor CLT

```text
l,temperatura1,resistencia1,temperatura2,resistencia2,;
```

- Temperaturas em graus Celsius.
- Resistências em ohms.
- No perfil Autoino, o CLT utiliza `A2`.

### `u` — Calibração do sensor IAT

```text
u,temperatura1,resistencia1,temperatura2,resistencia2,;
```

- Temperaturas em graus Celsius.
- Resistências em ohms.
- No perfil Autoino, o IAT utiliza `A3`.

### `m` — Configuração da injeção

```text
m,referencia,tipo_motor,modo,emparelhamento,deslocamento,cilindros,injetores,esguichos,fluxo,acionamento,combustivel,req_fuel,dreq_fuel,tipo_sonda_o2,;
```

| Índice | Campo | Valores/unidade |
|---:|---|---|
| 0 | `referencia` | `1` MAP, `2` TPS |
| 1 | `tipo_motor` | `4` quatro tempos, `2` dois tempos |
| 2 | `modo` | `0` injeção desligada (carburador / só ignição), `1` pareado, `2` semissequencial, `3` sequencial |
| 3 | `emparelhamento` | Código do emparelhamento |
| 4 | `deslocamento` | Polegadas cúbicas |
| 5 | `cilindros` | Quantidade de cilindros |
| 6 | `injetores` | Quantidade de injetores |
| 7 | `esguichos` | Esguichos por ciclo |
| 8 | `fluxo` | lb/h por injetor |
| 9 | `acionamento` | `1` simultâneo, `2` alternado |
| 10 | `combustivel` | AFR multiplicado por 1000 |
| 11 | `req_fuel` | Milissegundos multiplicados por 1000 |
| 12 | `dreq_fuel` | Milissegundos multiplicados por 1000 |
| 13 | `tipo_sonda_o2` | `0` narrowband, `1` wideband |

### `t` — Parâmetros do injetor

```text
t,limite,dead_time_us,angulo_fechamento,acrescimo_partida,acrescimo_funcionamento,;
```

| Campo | Faixa/unidade |
|---|---|
| `limite` | `0` a `100` % |
| `dead_time_us` | `0` a `5000` microssegundos |
| `angulo_fechamento` | `0` a `360` graus APMS |
| `acrescimo_partida` | `0` a `100` % |
| `acrescimo_funcionamento` | `0` a `100` % |

A UI mostra o dead time em milissegundos e o converte para microssegundos antes
de enviar.

### `n` — Proteção e limites

```text
n,tipo_protecao,rpm_pre_corte,avanco_corte,tempo_corte,rpm_corte,base_corte,qtd_corte,;
```

`tipo_protecao`: `0` desligado, `1` ignição, `2` injeção, `3` ignição e injeção.

### `o` — Enriquecimento de aceleração

```text
o,enr1,enr2,enr3,enr4,enr5,tpsdot1,tpsdot2,tpsdot3,tpsdot4,tpsdot5,tipo_verificacao,mudanca_minima,intervalo,duracao,rpm_min,rpm_max,desaceleracao,;
```

O comando contém 17 valores, na ordem apresentada acima.

### `p` — Calibração do TPS

```text
p,adc_minimo,adc_maximo,;
```

### `q` — Calibração do MAP

```text
q,tipo_map,map_minimo,map_maximo,;
```

### `r` — Injeção por temperatura

```text
r,temp1,temp2,temp3,temp4,temp5,enr1,enr2,enr3,enr4,enr5,habilitado,;
```

- Temperaturas: `0` a `150` °C, em ordem crescente.
- Enriquecimentos: percentual total de `0` a `250`.
- `100` significa sem correção; `120` significa 20% a mais.
- `habilitado`: `0` desativa, `1` ativa.
- O firmware interpola linearmente entre os pontos.

Exemplo:

```text
r,0,20,40,60,80,140,130,120,110,100,1,;
```

### `s` — Avanço por temperatura

```text
s,temp1,temp2,temp3,temp4,temp5,avanco1,avanco2,avanco3,avanco4,avanco5,habilitado,;
```

- Temperaturas: `0` a `150` °C, em ordem crescente.
- Avanços adicionais: `0` a `10` graus.
- `habilitado`: `0` desativa, `1` ativa.
- O firmware interpola linearmente entre os pontos.

### `v` — Configuração de partida

```text
v,rpm_max_partida,limpeza_afogamento,atraso_injecao_ms,reducao_enriquecimento_s,;
```

| Campo | Faixa/unidade |
|---|---|
| `rpm_max_partida` | `100` a `2000` RPM |
| `limpeza_afogamento` | `0` a `100` % de TPS |
| `atraso_injecao_ms` | `0` a `10000` milissegundos |
| `reducao_enriquecimento_s` | `0` a `600` segundos |

Durante a partida, TPS igual ou superior ao nível de limpeza bloqueia somente a
injeção. Depois que a rotação supera `rpm_max_partida`, o acréscimo de partida é
reduzido linearmente até zero no tempo configurado. O atraso afeta apenas o pulso
inicial simultâneo; se o motor começar a girar antes dele, esse pulso é cancelado.

### `w` — Controle de marcha lenta

```text
w,modo,temp_desligamento,histerese,abertura_frio,abertura_quente,rpm_alvo,max_passos,inverter_direcao,;
```

| Campo | Faixa/unidade |
|---|---|
| `modo` | `0` desligado, `1` abre/fecha, `2` PWM, `3` motor de passo |
| `temp_desligamento` | `0` a `120` °C |
| `histerese` | `0` a `20` °C |
| `abertura_frio` | `0` a `100` % a 0 °C |
| `abertura_quente` | `0` a `100` % na temperatura de desligamento |
| `rpm_alvo` | `500` a `2500` RPM |
| `max_passos` | `0` a `250` passos entre fechado e aberto |
| `inverter_direcao` | `0` normal, `1` invertida |

No modo abre/fecha, a saída desliga ao atingir `temp_desligamento` e volta a
ligar abaixo desse valor menos a histerese. Nos modos PWM e motor de passo, a
abertura base é interpolada conforme a temperatura. Com o motor funcionando e
TPS até 5%, a abertura sobe ou desce um ponto por ciclo para manter `rpm_alvo`
dentro de uma banda fixa de 50 RPM, sem usar PID.

No perfil Autoino com ATmega328P, o controle usa o D11/OC2A. No perfil Speeduino
v0.4 com ATmega2560, usa a saída padrão Idle 1 no D5/OC3A; o D6 reservado para a
segunda via de válvulas de três fios ainda não é utilizado. Os dois perfis geram
PWM de hardware a aproximadamente 61 Hz. A saída deve acionar um driver de
potência adequado; não conecte a válvula diretamente ao microcontrolador.

No modo motor de passo, Autoino usa D11 para `STEP` e D10 para `DIR`; Speeduino
usa os pinos padrão D17 para `STEP` e D16 para `DIR`. Ao inicializar, o firmware
move `max_passos + 10` no sentido de fechamento para referenciar a posição zero.
Os pulsos são espaçados em 1000 microssegundos e possuem largura de 2 microssegundos.
O A4988 deve ter `RESET` e `SLEEP` em nível alto, `ENABLE` habilitado e o limite
de corrente ajustado para o motor. Configure o modo de passo com o motor parado.

## Comandos de controle

| Comando | Ação |
|---|---|
| `h;` | Solicita todas as configurações e tabelas da ECU |
| `i;` | Alterna o envio dos dados binários em tempo real |
| `z;` | Grava as tabelas de ignição e VE na EEPROM |

Os comandos de configuração (`g` a `w`, conforme aplicável) gravam seus próprios
blocos na EEPROM imediatamente. O comando `z` é específico para as duas tabelas.

## Dados recebidos da ECU

### Resposta do comando `h;`

A ECU retorna uma sequência ASCII de blocos separados por `;`. A resposta começa
com `;` e cada bloco também possui vírgula depois do último valor:

```text
;a,...,;b,...,;c,...,;g,...,;j,...,;k,...,;l,...,;u,...,;d,...,;e,...,;f,...,;m,...,;t,...,;n,...,;o,...,;p,...,;q,...,;r,...,;s,...,;v,...,;w,...,;z,...,;V,...,;
```

Ordem atual dos blocos:

1. `a`: 16 valores do eixo MAP/TPS de ignição.
2. `b`: 16 valores do eixo RPM de ignição.
3. `c`: 256 valores da matriz de avanço.
4. `g`: 6 valores da configuração inicial.
5. `j`: 6 valores da faísca.
6. `k`: 2 valores de dwell.
7. `l`: 4 valores da calibração CLT.
8. `u`: 4 valores da calibração IAT.
9. `d`: 16 valores do eixo MAP/TPS VE.
10. `e`: 16 valores do eixo RPM VE.
11. `f`: 256 valores da matriz VE.
12. `m`: 14 valores da configuração da injeção.
13. `t`: 5 parâmetros do injetor.
14. `n`: 7 valores de proteção.
15. `o`: 17 valores de enriquecimento de aceleração.
16. `p`: 2 valores da calibração TPS.
17. `q`: 3 valores da calibração MAP.
18. `r`: 5 temperaturas, 5 enriquecimentos e a flag de ativação.
19. `s`: 5 temperaturas, 5 avanços e a flag de ativação.
20. `v`: RPM de partida, TPS de limpeza, atraso da injeção e tempo de redução.
21. `w`: modo, abertura, alvos de RPM e parâmetros do motor de passo.
22. `z`: flag da tabela de offset por cilindro e os 8 ângulos.
23. `V`: identidade — perfil de hardware e revisão de placa.

O bloco `V` é o último de propósito. Firmware anterior a ele simplesmente não o
envia, e quem lê encontra a ausência em vez de encontrar o resto deslocado.
A UI trata "a ECU não disse" como desconhecido, nunca como um valor padrão.

```text
;V,<perfil>,<revisão>,;      perfil 1 = Autoino, 2 = Speeduino
```

A revisão é a da placa para a qual o firmware foi **compilado** — ver *Pinagem*.

Exemplo parcial:

```text
;l,20,2490,100,200,;u,20,2500,100,187,;t,90,1000,5,50,0,;r,0,20,40,60,80,140,130,120,110,100,1,;
```

### Monitoramento binário em tempo real — protocolo V2

O comando `i;` alterna o monitoramento entre ligado e desligado. Enquanto estiver
ligado, o firmware envia um quadro binário fixo de **40 bytes** aproximadamente a
cada **200 ms (5 Hz)**. O comando não possui resposta de confirmação e atua como
*toggle*: a UI deve manter o estado da conexão para não inverter o estado por
engano.

O protocolo V2 possui cabeçalho, versão, tamanho e CRC-8. A UI deve procurar o
cabeçalho, validar o tamanho e descartar qualquer quadro cujo CRC seja inválido.
Isso permite recuperar o sincronismo automaticamente quando um byte é perdido.

#### Estrutura do quadro V2

| Bytes absolutos | Tamanho | Conteúdo |
|---:|---:|---|
| 0–1 | 2 | Cabeçalho fixo `0xA5 0x5A` |
| 2 | 1 | Versão do protocolo: `2` |
| 3 | 1 | Tamanho total do quadro: `40` |
| 4–28 | 25 | Payload atual |
| 29–38 | 10 | Reserva para dados futuros, preenchida com zero |
| 39 | 1 | CRC-8 dos bytes 0 a 38 |

O CRC utiliza o polinômio `0x07`, valor inicial `0x00`, sem reflexão e sem
`xorout`. O byte do CRC não participa do próprio cálculo. O quadro é transmitido
campo por campo e não existe um vetor de 40 bytes reservado na RAM.

Todos os campos de dois bytes usam `uint16` em little-endian (byte menos
significativo primeiro). Mesmo quando a variável interna tem 32 bits, somente os
16 bits menos significativos são enviados.

#### Payload atual

Os offsets abaixo são relativos ao início do payload, no byte absoluto 4. Para
obter a posição absoluta no quadro, some 4 ao offset.

| Offset no payload | Tipo bruto | Chave sugerida na UI | Dado enviado | Unidade/conversão |
|---:|---|---|---|---|
| 0–1 | `uint16 LE` | `rpm` | Rotação atual | RPM |
| 2–3 | `uint16 LE` | `map` | MAP convertido pela calibração | normalmente kPa |
| 4 | `uint8` | `cltC` | Temperatura do motor | °C |
| 5 | `uint8` | `ignitionAdvanceDeg` | Avanço aplicado | graus |
| 6–7 | `uint16 LE` | `loopRateHz` | `qtd_loop * 5` | loops/s |
| 8 | `uint8` | `syncLossCount` | Perdas de sincronismo acumuladas | ocorrências |
| 9 | `uint8` | `vePercent` | Célula VE atual | % |
| 10–11 | `uint16 LE` | `injectionTimeUs` | Largura final do pulso | µs |
| 12 | `uint8` | `tpsPercent` | TPS calibrado | % |
| 13 | `uint8` | `cutActive` | Proteção/pré-corte | `0` ou `1` |
| 14–15 | `uint16 LE` | `accelEnrichmentUs` | Enriquecimento de aceleração | µs |
| 16–17 | `uint16 LE` | `o2Adc` | Sonda O2 bruta | ADC, `0` a `1023` |
| 18–19 | `uint16 LE` | `tpsAdc` | TPS bruto | ADC, `0` a `1023` |
| 20 | `uint8` | `flagsEstado` | Estados agrupados | máscara de bits |
| 21–22 | `uint16 LE` | `flexAdc` | Sensor flex em A6 | ADC, `0` a `1023` |
| 23–24 | `uint16 LE` | `oilPressureAdc` | Pressão do óleo em A7 | ADC, `0` a `1023` |

O payload contém **16 dados**. Os sensores flex e de pressão do óleo são lidos
diretamente no momento do envio, sem variáveis globais adicionais. A conversão
nominal para tensão é `volts = adc * 5 / 1023`; porcentagem de etanol e pressão
em bar dependem da calibração dos sensores instalados.

#### Bits de `flagsEstado`

| Bit | Máscara | Estado ativo quando |
|---:|---:|---|
| 0 | `0x01` | Motor sincronizado |
| 1 | `0x02` | Motor em partida |
| 2 | `0x04` | Motor funcionando |
| 3 | `0x08` | Proteção/pré-corte ativo |
| 4 | `0x10` | Enriquecimento de aceleração ativo |
| 5 | `0x20` | Limpeza de afogamento ativa |
| 6 | `0x40` | Saída de marcha lenta ativa |
| 7 | `0x80` | Enriquecimento de partida ativo |

#### Validação e decodificação na UI

```ts
function crc8(bytes: Uint8Array) {
  let crc = 0;
  for (const value of bytes) {
    crc ^= value;
    for (let bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) !== 0 ? ((crc << 1) ^ 0x07) & 0xff : (crc << 1) & 0xff;
    }
  }
  return crc;
}

function decodeRealtimeFrame(frame: Uint8Array) {
  if (frame.length !== 40) throw new Error("Quadro deve ter 40 bytes");
  if (frame[0] !== 0xa5 || frame[1] !== 0x5a) throw new Error("Cabeçalho inválido");
  if (frame[2] !== 2) throw new Error("Versão de telemetria incompatível");
  if (frame[3] !== 40) throw new Error("Tamanho declarado inválido");
  if (crc8(frame.subarray(0, 39)) !== frame[39]) throw new Error("CRC inválido");

  const payload = frame.subarray(4, 29);
  const u16 = (offset: number) => payload[offset] | (payload[offset + 1] << 8);
  const flagsEstado = payload[20];

  return {
    rpm: u16(0),
    map: u16(2),
    cltC: payload[4],
    ignitionAdvanceDeg: payload[5],
    loopRateHz: u16(6),
    syncLossCount: payload[8],
    vePercent: payload[9],
    injectionTimeUs: u16(10),
    tpsPercent: payload[12],
    cutActive: payload[13] !== 0,
    accelEnrichmentUs: u16(14),
    o2Adc: u16(16),
    tpsAdc: u16(18),
    flagsEstado,
    flexAdc: u16(21),
    oilPressureAdc: u16(23),
  };
}
```

A UI não deve simplesmente consumir blocos consecutivos de 40 bytes. Ela deve
procurar `0xA5 0x5A`, verificar versão e tamanho, aguardar o quadro completo e
validar o CRC. Se a validação falhar, descarte apenas o primeiro byte candidato e
continue procurando o próximo cabeçalho.

Respostas ASCII de configuração compartilham a mesma porta. Evite solicitar
`h;` enquanto a telemetria estiver ligada.
## EEPROM

Principais blocos utilizados atualmente:

| Endereços | Conteúdo |
|---|---|
| 10–41 | Eixo RPM da ignição |
| 50–65 | Eixo MAP/TPS da ignição |
| 100–355 | Matriz de avanço |
| 360–371 | Configuração inicial |
| 380–391 | Configuração da faísca |
| 400–403 | Dwell |
| 410–417 | Calibração CLT |
| 420–428 | Calibração IAT e marcador |
| 500–531 | Eixo RPM VE |
| 532–547 | Eixo MAP/TPS VE |
| 548–803 | Matriz VE |
| 820–827 | Configuração de partida e marcador |
| 830–839 | Configuração de marcha lenta e marcador |
| 900–918 | Configuração da injeção |
| 920–927 | Parâmetros do injetor e marcador |
| 950–958 | Proteção |
| 970–995 | Enriquecimento de aceleração |
| 1000–1003 | Calibração TPS |
| 1010–1014 | Calibração MAP |
| 1015–1016 | Flags das correções por temperatura |
| 1020–1029 | Tabela de injeção por temperatura |
| 1030–1039 | Tabela de avanço por temperatura |

> Atenção: ATmega328P possui 1024 bytes de EEPROM, com endereços válidos de 0 a
> 1023. O layout atual ultrapassa esse limite nas tabelas localizadas após 1023.
> Esses blocos precisam ser realocados antes de depender da persistência completa
> em placas Nano/Uno.

## Arquivos relacionados ao protocolo

- `include/leitura_entrada_dados_serial.h`: recebe e interpreta comandos.
- `include/ler_dados_memoria.h`: monta a resposta ASCII do comando `h`.
- `include/envia_dados_tempo_real.h`: monta o quadro V2 fixo de 40 bytes.
- `include/grava_dados_eeprom.h`: grava configurações na EEPROM.
- `include/ler_dados_eeprom.h`: restaura configurações da EEPROM.
- `include/util.h`: conversão serial e funções auxiliares de EEPROM.
