
void protege_ignicao(){
  if(rpm_anterior < 20){
    digitalWrite(ignicao_pins[0],0);
    digitalWrite(ignicao_pins[1],0);
    digitalWrite(ignicao_pins[2],0);
    digitalWrite(ignicao_pins[3],0);
  }
}