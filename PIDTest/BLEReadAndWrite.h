bool success;

//read a float characteristic and convert the byte array to float value
float readFloatCharacteristic(BLEFloatCharacteristic characteristicFlt){
  byte bufferB[4];
  byte temp[4];
  int j=3;
  characteristicFlt.readValue(bufferB,4);
  for(int i=0;i<4;i++){
    temp[j]=bufferB[i];
    j--;
  }
  float valueFlt = *(float *)&temp;
  return valueFlt;
}

//read an int characteristic and convert the byte array to an int value
int readIntCharacteristic(BLEIntCharacteristic characteristicInt){
  byte bufferB[2];
  characteristicInt.readValue(bufferB,2);
  int valueInt = 0;
  Serial.println(bufferB[0]);
  Serial.println(bufferB[1]);
  valueInt = bufferB[1] + (bufferB[0] << 8);
  return valueInt;
}

//try to write a float value to central
void writeFloatValue(BLEFloatCharacteristic characteristicFloat, float valueFlt) {
  success = characteristicFloat.writeValue(valueFlt);
  while(!success) {
    success = characteristicFloat.writeValue(valueFlt);
  }
}
