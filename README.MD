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
| 2 | `modo` | `1` pareado, `2` semissequencial, `3` sequencial |
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
;a,...,;b,...,;c,...,;g,...,;j,...,;k,...,;l,...,;u,...,;d,...,;e,...,;f,...,;m,...,;t,...,;n,...,;o,...,;p,...,;q,...,;r,...,;s,...,;v,...,;w,...,;
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
