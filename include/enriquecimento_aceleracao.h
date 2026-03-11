void calcula_enriquecimento_aceleracao(float tempo_pulso) {
 // Calcula a taxa de mudança do TPS (TPSDot)
          if (micros() - tempo_anterior_aceleracao >= (unsigned long)intervalo_tempo_aceleracao * 1000) {
              // Converte o intervalo para segundos
              float tps_dot = (valor_tps - tps_anterior) / (intervalo_tempo_aceleracao / 1000.0);

              // Verifica se está ocorrendo uma aceleração ou desaceleração
              if (tps_dot > tps_mudanca_minima) {
                  // Interpolação linear para o enriquecimento de aceleração
                  for (int i = 0; i < 5; i++) {
                      if (tps_dot <= tps_dot_escala[i+1]) {
                          // Calcula a interpolação linear
                          float tps_dot_range = tps_dot_escala[i+1] - tps_dot_escala[i];
                          float enrichment_range = enriquecimento_aceleracao[i+1] - enriquecimento_aceleracao[i];
                          float proportion = (tps_dot - tps_dot_escala[i]) / tps_dot_range;
                          tps_dot_porcentagem_aceleracao = enriquecimento_aceleracao[i] + (proportion * enrichment_range);
                          incremento_aceleracao = round(tempo_pulso * (tps_dot_porcentagem_aceleracao / 100.0));
                          decremento_desaceleracao = 0;
                          break;
                      }
                  }
                  tps_dot_porcentagem_desaceleracao = 0;
                  tempo_ultima_mudanca = micros();
              } else if (tps_dot < -tps_mudanca_minima) {
                  incremento_aceleracao = 0;  
                  decremento_desaceleracao = round(tempo_pulso * (tps_dot_porcentagem_desaceleracao / 100.0));
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