void calcula_enriquecimento_aceleracao(){
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
                          break;
                      }
                  }
                  tps_dot_porcentagem_desaceleracao = 0;
                  tempo_ultima_mudanca = micros();
              } else if (tps_dot < -tps_mudanca_minima) {
                  tps_dot_porcentagem_desaceleracao = 5; // Pode ajustar conforme necessário
                  tps_dot_porcentagem_aceleracao = 0;
                  tempo_ultima_mudanca = micros();
              }

              // Reseta os valores após a duração do enriquecimento
              if (micros() - tempo_ultima_mudanca >= (unsigned long)duracao_enriquecimento * 1000) {
                  tps_dot_porcentagem_aceleracao = 0;
                  tps_dot_porcentagem_desaceleracao = 0;
              }

              // Atualiza o valor anterior do TPS e o tempo de leitura
              tps_anterior = valor_tps;
              tempo_anterior_aceleracao = micros();
          }       
}