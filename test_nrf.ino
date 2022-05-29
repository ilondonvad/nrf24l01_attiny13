#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <avr/sleep.h>


RF24 radio(9, 53);                            // CE CSN
byte address[] = {0xe7,0xe7,0xe7,0xe7,0xe7};  // адрес модуля для приема
byte recieved_data[1];                        // массив полученых данных

void setup() {
  
  pinMode(13,1);
  Serial.begin(9600);
  radio.begin();                      // активировать модуль
  //radio.maskIRQ(1,1,1);             // маска прерываний по полученым данным
  radio.setAutoAck(true);             // режим подтверждения приёма, 1 вкл 0 выкл
  //radio.setRetries(0, 15);          // (время между попыткой достучаться, число попыток)
  //radio.enableAckPayload();         // разрешить отсылку данных в ответ на входящий сигнал
  //radio.setPayloadSize(32);         // размер пакета, байт
  radio.openReadingPipe(0,address);   // хотим слушать трубу 0
  //radio.openWritingPipe(address[0]);
  radio.setChannel(0x64);             // выбираем канал (в котором нет шумов!) такой же как на передатчике
  radio.setPALevel(RF24_PA_MAX);      // На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate(RF24_1MBPS);      // скорость передачи данных должна быть такая же как на передатчике
  //radio.setCRCLength(2);            // контрольная сумма, по умолчанию 2 байта          
  radio.powerUp();                    // начать работу
  radio.startListening();             // начинаем слушать эфир, мы приёмный модуль
  


}
void loop() {

   while (radio.available()) {  
    radio.read( &recieved_data, sizeof(recieved_data)); // чиатем входящий сигнал
    
  }
   
  if(recieved_data[0]>0){
  digitalWrite(13,1);
  Serial.print("==");
  Serial.println(recieved_data[0]);
  Serial.println("----------------");
  recieved_data[0]=0;}
  else {digitalWrite(13,0);recieved_data[0]=0;radio.stopListening();radio.startListening();}
  delay(10);
}
