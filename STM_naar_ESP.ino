#include <SoftwareSerial.h>

SoftwareSerial mySerial(5,15);
String sensorData = ""; // Voor sensorwaarden ontvangen via UART

void setup() {
  mySerial.begin(115200);   // UART met STM32 // Seriële monitor van Arduino IDE
  Serial.begin(9600);      

  Serial.println("UART Communication Start");
}

void loop() {
  Read_Uart(); // Lees inkomende gegevens van STM32
  delay(100); 
}

void Read_Uart() {
  
  while (mySerial.available()) {
    char inChar = (char)mySerial.read();
    if (inChar == '\n') { // Einde van een bericht
      Serial.print("Ontvangen sensorgegevens: ");
      Serial.println(sensorData);
      sensorData = ""; // Reset de buffer voor het volgende bericht
    } else {
      sensorData += inChar; // Voeg de ontvangen tekens toe aan de buffer
    }
  }
}