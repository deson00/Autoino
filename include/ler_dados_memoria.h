void ler_dados_memoria(){
Serial.print(";a,");
      // vetor map ou tps
      for (int  i = 0; i < 16; i++){
        Serial.print(vetor_map_tps[i]);
        Serial.print(",");
      }
      Serial.print(";");
      Serial.print("b,");
      // vetor rpm
      for (int  i = 0; i < 16; i++){
        Serial.print(vetor_rpm[i]);
        Serial.print(",");
      }
      Serial.print(";");
      // transforma matriz em vetor
      Serial.print("c,");
        int k = 0;
        for (int i = 0; i < 16; i++){
          for (int j = 0; j < 16; j++){
            Serial.print(matriz_avanco[i][j]);
            Serial.print(",");
            k++;
          }
        }
        Serial.print(";");
      
      // dados configuração inicial
      Serial.print("g,");
      Serial.print(tipo_ignicao);
      Serial.print(",");
      Serial.print(qtd_dente);
      Serial.print(",");
      Serial.print(local_rodafonica);
      Serial.print(",");
      Serial.print(qtd_dente_faltante);
      Serial.print(",");
      Serial.print(grau_pms);
      Serial.print(",");
      Serial.print(qtd_cilindro * local_rodafonica);
      Serial.print(",");
      Serial.print(";");

      // dados configuração faisca
      Serial.print("j,");
      Serial.print(referencia_leitura_ignicao);
      Serial.print(",");
      Serial.print(modo_ignicao);
      Serial.print(",");
      Serial.print(grau_avanco_partida);
      Serial.print(",");
      Serial.print(avanco_fixo);
      Serial.print(",");
      Serial.print(grau_avanco_fixo);
      Serial.print(",");
      Serial.print(tipo_sinal_bobina);
      Serial.print(",");
      Serial.print(";");

      // dados configuração dwell
      Serial.print("k,");
      Serial.print(dwell_partida);
      Serial.print(",");
      Serial.print(dwell_funcionamento);
      Serial.print(",");
      Serial.print(";");

      // dados configuração calibrate temperature sensor ctl
      Serial.print("l,");//letra L minusculo
      Serial.print(referencia_temperatura_clt1);
      Serial.print(",");
      Serial.print(referencia_resistencia_clt1);
      Serial.print(",");
      Serial.print(referencia_temperatura_clt2);
      Serial.print(",");
      Serial.print(referencia_resistencia_clt2);
      Serial.print(",");
      Serial.print(";");

      //---------ve-------------//
      Serial.print("d,");
      // vetor map ou tps da ve
      for (int  i = 0; i < 16; i++){
        Serial.print(vetor_map_tps_ve[i]);
        Serial.print(",");
      }
      Serial.print(";");
      Serial.print("e,");
      // vetor rpm da tabela ve
      for (int  i = 0; i < 16; i++){
        Serial.print(vetor_rpm_ve[i]);
        Serial.print(",");
      }
      Serial.print(";");
      // transforma matriz em vetor
      Serial.print("f,");
        k = 0;
        for (int i = 0; i < 16; i++){
          for (int j = 0; j < 16; j++){
            Serial.print(matriz_ve[i][j]);
            Serial.print(",");
            k++;
          }
        }
        Serial.print(";");
      //---------ve------------//  
      // dados configuração injeção 
Serial.print("m,");
Serial.print(referencia_leitura_injecao);
Serial.print(",");
Serial.print(tipo_motor);
Serial.print(",");
Serial.print(modo_injecao);
Serial.print(",");
Serial.print(emparelhar_injetor);
Serial.print(",");
Serial.print(deslocamento_motor);
Serial.print(",");
Serial.print(numero_cilindro_injecao);
Serial.print(",");
Serial.print(numero_injetor);
Serial.print(",");
Serial.print(numero_esguicho);
Serial.print(",");
Serial.print(tamanho_injetor);
Serial.print(",");
Serial.print(tipo_acionamento_injetor);
Serial.print(",");
Serial.print(tipo_combustivel);
Serial.print(",");
Serial.print(REQ_FUEL);
Serial.print(",");
Serial.print(dreq_fuel);
Serial.print(",");
Serial.print(";");
// dados configuração proteção e limites 
Serial.print("n,");
Serial.print(tipo_protecao);
Serial.print(",");
Serial.print(rpm_pre_corte);
Serial.print(",");
Serial.print(avanco_corte);
Serial.print(",");
Serial.print(tempo_corte);
Serial.print(",");
Serial.print(rpm_maximo_corte);
Serial.print(",");
Serial.print(numero_base_corte);
Serial.print(",");
Serial.print(qtd_corte);
Serial.print(",");
Serial.print(";");
}