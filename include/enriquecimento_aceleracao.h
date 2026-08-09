void calcula_enriquecimento_aceleracao(float tempo_pulso) {
 // Calcula a taxa de mudança do TPS (TPSDot)
          if (micros() - tempo_anterior_aceleracao >= (unsigned long)intervalo_tempo_aceleracao * 1000) {

              // So atua dentro da faixa de RPM configurada na tela. Fora
              // dela, zera qualquer enriquecimento pendente em vez de deixar
              // um valor de quando ainda estava dentro da faixa - por
              // exemplo, evita ficar enriquecendo perto da marcha lenta,
              // onde uma variacao pequena do TPS pode brigar com o controle
              // de marcha lenta.
              if (rpm < rpm_minimo_enriquecimento || rpm > rpm_maximo_enriquecimento) {
                  incremento_aceleracao = 0;
                  decremento_desaceleracao = 0;
                  tps_anterior = valor_tps;
                  tempo_anterior_aceleracao = micros();
                  return;
              }

              // Converte o intervalo para segundos
              float tps_dot = (valor_tps - tps_anterior) / (intervalo_tempo_aceleracao / 1000.0);

              // Verifica se está ocorrendo uma aceleração ou desaceleração
              if (tps_dot > tps_mudanca_minima) {
                  // Interpolação linear para o enriquecimento de aceleração
                  bool ponto_encontrado = false;
                  for (int i = 0; i < 4; i++) {
                      if (tps_dot <= tps_dot_escala[i+1]) {
                          // Calcula a interpolação linear
                          float tps_dot_range = tps_dot_escala[i+1] - tps_dot_escala[i];
                          float enrichment_range = enriquecimento_aceleracao[i+1] - enriquecimento_aceleracao[i];
                          float proportion = (tps_dot - tps_dot_escala[i]) / tps_dot_range;
                          tps_dot_porcentagem_aceleracao = enriquecimento_aceleracao[i] + (proportion * enrichment_range);
                          ponto_encontrado = true;
                          break;
                      }
                  }
                  if (!ponto_encontrado) {
                      // tps_dot passou do ultimo ponto da curva (ex.: um
                      // chute de acelerador mais brusco que o topo
                      // configurado) - usa o valor maximo da curva em vez de
                      // ler uma posicao fora dos limites do vetor.
                      tps_dot_porcentagem_aceleracao = enriquecimento_aceleracao[4];
                  }
                  incremento_aceleracao = round(tempo_pulso * (tps_dot_porcentagem_aceleracao / 100.0));
                  decremento_desaceleracao = 0;
                  tps_dot_porcentagem_desaceleracao = 0;
                  tempo_ultima_mudanca = micros();
              } else if (tps_dot < -tps_mudanca_minima) {
                  incremento_aceleracao = 0;
                  // Usa o valor configurado na tela ("Enriquecimento na
                  // desaceleracao (%)") em vez da variavel que nunca era
                  // preenchida - antes o corte na desaceleracao nunca
                  // acontecia de verdade, independente do que a tela dizia.
                  decremento_desaceleracao = round(tempo_pulso * (enriquecimento_desaceleracao / 100.0));
                  tempo_ultima_mudanca = micros();
              }

              // Reseta os valores após a duração do enriquecimento
              if (micros() - tempo_ultima_mudanca >= (unsigned long)duracao_enriquecimento * 1000) {
                  incremento_aceleracao = 0;
                  decremento_desaceleracao = 0;
              }

              // Atualiza o valor anterior do TPS e o tempo de leitura
              tps_anterior = valor_tps;
              tempo_anterior_aceleracao = micros();
          }
}