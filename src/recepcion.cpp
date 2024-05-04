#include <SPI.h>  
#include <RH_NRF24.h> 

RH_NRF24 nrf24(21, 16);   // CE en pin 21, CSN en pin 16

float TEMPERATURA;  
int HUMEDAD;    

void setup() {
  Serial.begin(115200);   
  if (!nrf24.init())    
    Serial.println("Fallo de inicialización");
  if (!nrf24.setChannel(2)) 
    Serial.println("Fallo en establecer canal");
  if (!nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("Fallo en opciones RF");          
}

void loop() {
  TEMPERATURA = 20.0;  
  HUMEDAD = 10;   
  
  uint8_t temperaturaBytes[sizeof(float)];
  memcpy(temperaturaBytes, &TEMPERATURA, sizeof(float)); // Copia el valor de TEMPERATURA a un arreglo de bytes
  
  nrf24.send(temperaturaBytes, sizeof(float));  // Envía los bytes de la temperatura
  nrf24.waitPacketSent();       
  delay(1000);          
}
