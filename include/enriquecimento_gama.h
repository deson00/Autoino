// Aplica as correcoes percentuais ao tempo base de injecao.
// Valores acima de 100 adicionam combustivel, abaixo de 100 retiram.
//
// Aritmetica inteira: cada correcao e aplicada como (valor * correcao) / 100,
// uma de cada vez. Dividir a cada passo (em vez de multiplicar as quatro e
// dividir por 100^4 no fim) mantem o resultado dentro de 32 bits com folga -
// tempo_base fica na casa de milhares de microssegundos e as correcoes em
// torno de 100.
unsigned long enriquecimento_gama(unsigned long tempo_base_injecao,
                                  int correcao_aquecimento,
                                  int correcao_O2,
                                  int correcao_temperatura_ar,
                                  int correcao_barometrica) {
  unsigned long valor = tempo_base_injecao;

  if (correcao_aquecimento != 100) valor = (valor * (unsigned long)correcao_aquecimento) / 100UL;
  if (correcao_O2 != 100) valor = (valor * (unsigned long)correcao_O2) / 100UL;
  if (correcao_temperatura_ar != 100) valor = (valor * (unsigned long)correcao_temperatura_ar) / 100UL;
  if (correcao_barometrica != 100) valor = (valor * (unsigned long)correcao_barometrica) / 100UL;

  return valor;
}
