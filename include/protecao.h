
void protege_ignicao_injecao(){
  static bool protecao_baixa_rotacao_ativa = false;
  bool rotacao_baixa = (rpm < 20);

  // So executa na borda de entrada (rotacao acabou de cair abaixo de 20 rpm).
  // Antes isso rodava a cada ciclo enquanto a rotacao ficasse baixa, o que
  // zerava revolucoes_sincronizada continuamente e nunca deixava o decoder
  // reconquistar sincronismo mesmo depois do sinal do sensor voltar ao normal.
  if (rotacao_baixa && !protecao_baixa_rotacao_ativa) {
    digitalWrite(ignicao_pins[0],0);
    digitalWrite(ignicao_pins[1],0);
    digitalWrite(ignicao_pins[2],0);
    digitalWrite(ignicao_pins[3],0);
    digitalWrite(injecao_pins[0],0);
    digitalWrite(injecao_pins[1],0);
    digitalWrite(injecao_pins[2],0);
    digitalWrite(injecao_pins[3],0);
    revolucoes_sincronizada = 0;
    falhas_sync_consecutivas = 0;
  }

  protecao_baixa_rotacao_ativa = rotacao_baixa;
}

void protege_limite_ignicao(){
    digitalWrite(ignicao_pins[0],0);
    digitalWrite(ignicao_pins[1],0);
    digitalWrite(ignicao_pins[2],0);
    digitalWrite(ignicao_pins[3],0);
}

void protege_limite_injecao(){
    if (modo_injecao == 0) return; // injecao desligada: pinos nao sao saida de bico
    digitalWrite(injecao_pins[0],0);
    digitalWrite(injecao_pins[1],0);
    digitalWrite(injecao_pins[2],0);
    digitalWrite(injecao_pins[3],0);
}