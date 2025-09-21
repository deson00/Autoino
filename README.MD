Este projeto ainda esta em desenvolvimento todo cuidado e atenção no seu uso.
┌─────────────────────┐
│   RODA FÔNICA       │ ← Pulsos do sensor Hall
│ (decoder_padrao.h)  │
└──────────┬──────────┘
           │ Atualiza: rpm, posicao_atual_sensor, 
           │          revolucoes_sincronizada, tempo_cada_grau
           ▼
┌─────────────────────┐
│      TIMER1         │ ← Executa a cada 100µs
│    (timer.h)        │
└─────────────────────┘
           │ Usa as variáveis da roda fônica para:
           │ - Calcular timing de ignição/injeção
           │ - Acionar bobinas/injetores
           ▼
┌─────────────────────┐
│  IGNIÇÃO/INJEÇÃO    │
│   (Hardware)        │
└─────────────────────┘


# 🏎️ Sistema de Comunicação Serial para ECU

Sistema de comunicação serial para configuração de ECU (Electronic Control Unit) para motores de combustão interna.

## 📋 Índice

- [Sobre](#sobre)
- [Características](#características)
- [Instalação](#instalação)
- [Protocolo de Comunicação](#protocolo-de-comunicação)
- [Comandos Disponíveis](#comandos-disponíveis)
- [Exemplos de Uso](#exemplos-de-uso)
- [API Reference](#api-reference)
- [Contribuição](#contribuição)
- [Licença](#licença)

## 🎯 Sobre

Este projeto implementa um sistema de comunicação serial robusto para configuração de ECUs automotivas, permitindo:

- ✅ Configuração de mapas de ignição e injeção
- ✅ Calibração de sensores (TPS, MAP, CLT)
- ✅ Sistema de proteção do motor
- ✅ Monitoramento em tempo real
- ✅ Armazenamento permanente (EEPROM)

## 🚀 Características

### Sistema de Ignição
- Suporte a centelha perdida e única
- Mapas 3D de avanço (16x16)
- Configuração de dwell
- Proteção por RPM

### Sistema de Injeção
- Modos: Pareado, Semi-sequencial, Sequencial
- Cálculo automático de REQ_FUEL
- Enriquecimento na aceleração
- Suporte a diferentes combustíveis

### Sensores Suportados
- **TPS** (Throttle Position Sensor)
- **MAP** (Manifold Absolute Pressure)
- **CLT** (Coolant Temperature)

## 📦 Instalação

1. Clone o repositório:
```bash
git clone https://github.com/deson00/Autoino.git
```

2. Abra o arquivo no Arduino IDE

3. Configure a porta serial e faça o upload

## 📡 Protocolo de Comunicação

### Formato Geral
```
COMANDO,valor1,valor2,...,valorN;
```

- **COMANDO**: Letra identificadora (a-z)
- **Valores**: Números separados por vírgula
- **Terminador**: Ponto e vírgula (`;`)

### Exemplo
```
a,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170;
```

## 🔧 Comandos Disponíveis

### 📊 Tabelas de Calibração

| Comando | Função | Formato |
|---------|--------|---------|
| `a` | MAP/TPS Ignição | `a,val1,val2,...,val16;` |
| `b` | RPM Ignição | `b,500,1000,1500,...;` |
| `c` | Matriz Avanço | `c,val1,val2,...,val256;` |
| `d` | MAP/TPS VE | `d,val1,val2,...,val16;` |
| `e` | RPM VE | `e,500,1000,1500,...;` |
| `f` | Matriz VE | `f,val1,val2,...,val256;` |

### ⚙️ Configurações do Sistema

<details>
<summary><b>Configuração Inicial (g)</b></summary>

```
g,tipo_ignição,qtd_dentes,local_roda_fonica,qtd_dentes_faltantes,grau_pms,qtd_cilindros;
```

**Parâmetros:**
- `tipo_ignição`: Tipo do sistema
- `qtd_dentes`: Dentes da roda fônica
- `local_roda_fonica`: 2=virabrequim, 1=comando
- `qtd_dentes_faltantes`: Dentes ausentes
- `grau_pms`: Graus do PMS
- `qtd_cilindros`: Número de cilindros

</details>

<details>
<summary><b>Configuração Ignição (j)</b></summary>

```
j,referência,modo,avanço_partida,avanço_fixo,grau_fixo,tipo_sinal;
```

**Parâmetros:**
- `referência`: 1=MAP, 2=TPS
- `modo`: 1=centelha perdida, 2=única
- `avanço_partida`: Graus na partida
- `avanço_fixo`: 0=off, 1=on
- `grau_fixo`: Valor fixo (0-360°)
- `tipo_sinal`: 1=alto, 0=baixo

</details>

<details>
<summary><b>Outros Comandos</b></summary>

| Comando | Função | Parâmetros |
|---------|--------|------------|
| `k` | Dwell | partida,funcionamento |
| `l` | Sensor CLT | temp1,res1,temp2,res2 |
| `m` | Injeção | 13 parâmetros |
| `n` | Proteção | 7 parâmetros |
| `o` | Enriquecimento | 17 parâmetros |
| `p` | TPS | min,max |
| `q` | MAP | tipo,min,max |

</details>

### 🎮 Comandos de Controle

| Comando | Função |
|---------|--------|
| `h` | 📤 Lê dados da ECU |
| `i` | 📊 Toggle dados tempo real |
| `z` | 💾 Salva na EEPROM |

## 📝 Exemplos de Uso

### Configuração Básica

```bash
# 1. Motor 4 cilindros, roda fônica 60-2
g,1,60,2,2,10,4;

# 2. Ignição por MAP, centelha perdida
j,1,1,10,0,0,1;

# 3. Configuração TPS (0-1023)
p,0,1023;

# 4. Salvar configurações
z
```

### Carregamento de Mapa

```bash
# RPM da tabela de ignição
b,800,1000,1500,2000,2500,3000,3500,4000,4500,5000,5500,6000,6500,7000,7500,8000;

# MAP/TPS da tabela de ignição  
a,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170;

# Matriz de avanço (exemplo com 16 valores, na prática são 256)
c,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40;

# Salvar
z
```

### Monitoramento

```bash
# Ativar dados em tempo real
i

# Ler dados salvos
h
```

## 📖 API Reference

### Função Principal

```cpp
void leitura_entrada_dados_serial()
```

Processa dados recebidos via Serial e executa comandos correspondentes.

### Variáveis Globais Importantes

```cpp
// Tabelas de calibração
int vetor_rpm[16];              // RPM para ignição
int vetor_map_tps[16];          // MAP/TPS para ignição
int matriz_avanco[16][16];      // Matriz de avanço

// Configurações
int tipo_ignicao;               // Tipo de ignição
int qtd_cilindro;              // Quantidade de cilindros
bool status_dados_tempo_real;   // Status monitoramento
```

## 🛠️ Desenvolvimento

### Estrutura do Projeto
ENDEREÇOS EEPROM - LAYOUT REORGANIZADO:
0-9      : Reservados
10-41    : vetor_rpm ignição (16 x 2 bytes = 32 bytes)
50-65    : vetor_map_tps ignição (16 x 1 byte = 16 bytes) 
100-355  : matriz_avanco (16x16 x 1 byte = 256 bytes)
360-371  : configuracao_inicial (12 bytes)
380-391  : configuracao_faisca (12 bytes)
400-403  : configuracao_dwell (4 bytes)
410-417  : configuracao_clt (8 bytes)

500-531  : vetor_rpm_ve (16 x 2 bytes = 32 bytes)
540-555  : vetor_map_tps_ve (16 x 1 byte = 16 bytes)
600-855  : matriz_ve (16x16 x 1 byte = 256 bytes)

900-917  : configuracao_injecao (18 bytes) - MOVIDO PARA EVITAR CONFLITO
950-958  : configuracao_protecao (9 bytes)
970-991  : enriquecimento_aceleracao (22 bytes)
1000-1003: configuracao_tps (4 bytes)
1010-1014: configuracao_map (5 bytes)
```
ecu-serial-communication/
├── src/
│   ├── main.cpp
│   ├── serial_handler.cpp
│   └── eeprom_manager.cpp
├── include/
│   └── config.h
├── examples/
│   └── basic_setup.ino
└── README.md
```

### Compilação

Requisitos:
- Arduino IDE 1.8.0+
- Placa compatível (Arduino Uno, Mega, etc.)

## 🤝 Contribuição

1. Faça um Fork do projeto
2. Crie sua Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit suas mudanças (`git commit -m 'Add some AmazingFeature'`)
4. Push para a Branch (`git push origin feature/AmazingFeature`)
5. Abra um Pull Request

### 📋 Diretrizes

- Siga o padrão de código existente
- Adicione testes para novas funcionalidades  
- Documente mudanças no CHANGELOG.md
- Use commits semânticos

## 🐛 Problemas Conhecidos

- [ ] Buffer overflow em strings muito longas
- [ ] Validação limitada de entrada
- [ ] Sem checksum nos dados

## 📈 Roadmap

- [ ] Interface gráfica para configuração
- [ ] Suporte a CAN Bus
- [ ] Datalog integrado  
- [ ] Simulador de ECU
- [ ] Testes automatizados

## 📄 Licença

Este projeto está licenciado sob a Licença MIT - veja o arquivo [LICENSE](LICENSE) para detalhes.

## 👥 Autores

- **Seu Nome** - *Trabalho inicial* - [SeuGitHub](https://github.com/seu-usuario)

## 🙏 Agradecimentos

- Comunidade Arduino
- Desenvolvedores de ECUs open source
- Contribuidores do projeto

---

<div align="center">

**[⬆ Voltar ao topo](#sistema-de-comunicação-serial-para-ecu)**

Made with ❤️ for the automotive community

</div>