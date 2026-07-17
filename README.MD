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
- O monitoramento em tempo real usa quadros binários de 20 bytes.
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

### `g` — Configuração inicial

```text
g,tipo_ignicao,qtd_dentes,local_roda,qtd_dentes_faltantes,grau_pms,qtd_cilindros,;
```

| Campo | Valores/unidade |
|---|---|
| `tipo_ignicao` | Tipo de ignição configurado pela UI |
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

### Monitoramento binário em tempo real

O comando `i;` ativa ou desativa quadros binários de **20 bytes**. Valores de
dois bytes são enviados em little-endian: byte menos significativo primeiro.

| Offset | Tamanho | Campo |
|---:|---:|---|
| 0 | 2 | RPM |
| 2 | 2 | MAP |
| 4 | 1 | Temperatura CLT |
| 5 | 1 | Avanço atual |
| 6 | 2 | Contador/carga do loop |
| 8 | 1 | Perdas de sincronismo |
| 9 | 1 | VE atual |
| 10 | 2 | Tempo de injeção |
| 12 | 1 | TPS em porcentagem |
| 13 | 1 | Estado do corte |
| 14 | 2 | Incremento de aceleração |
| 16 | 2 | Lambda multiplicado por 1000 |
| 18 | 2 | ADC bruto do TPS |

Para reconstruir um valor de dois bytes:

```text
valor = byte_baixo | (byte_alto << 8)
```

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
- `include/envia_dados_tempo_real.h`: monta o quadro binário de 20 bytes.
- `include/grava_dados_eeprom.h`: grava configurações na EEPROM.
- `include/ler_dados_eeprom.h`: restaura configurações da EEPROM.
- `include/util.h`: conversão serial e funções auxiliares de EEPROM.
