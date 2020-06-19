
/*
  Модуль SD карты подключен в SPI по схеме:
  MOSI - пин 11
 MISO - пин12
  CLK - пин 13
 CS - pin 4 
 */
#include <SPI.h>
#include <SD.h>
const int PIN_CHIP_SELECT = 4;
void setup() {
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
 
  pinMode(10, OUTPUT);
  // инициализировать модуль
  if (!SD.begin(PIN_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    // Если что-то пошло не так, завершает работу:
    return;
  }
  Serial.println("card initialized.");
}
void loop() {
  // данные, которые мы поместим в файл:
  String logStringData = "";
  // Считываем данные с портов и записываем в строку
  for (int i = 0; i < 5; i++) {
    int sensor = analogRead(i);
    logStringData += String(sensor);
    if (i < 4) {
      logStringData += ",";
    }
  }
  // Открываем файл, но помним, что одновременно можно работать только с одним файлом.
  // Если файла с таким именем не будет, ардуино создаст его.
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  // Если все хорошо, то записываем строку:
  if (dataFile) {
    dataFile.println(logStringData);
    dataFile.close();
    // Публикуем в мониторе 
    Serial.println(logStringData);
  }
  else {
  // Сообщаем об ошибке, если все плохо
    Serial.println("error opening datalog.csv");
  }
}
