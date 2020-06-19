/*
 * SVET
 */
 /*
  * фоторезисторный датчик освещенности а. и ц.
  * analogov
  */
  void setup() {
  pinMode(A1, INPUT);
  analogWrite(A1, LOW);
  Serial.begin(9600);   // подключаем монитор порта
}

void loop() {
  // считываем данные с датчика и выводим на монитор порта
  int light = analogRead(A1);
  Serial.print("Light = ");
  Serial.println(light);

  // рассчитываем напряжение и выводим на монитор порта
  float u = light * 0.48 / 100;
  Serial.print("U = ");
  Serial.println(u);

  // ставим паузу и делаем перенос строки
  delay(500);
  Serial.println("");
}
/*
 * cifrovoi
 */
 void setup() {
  pinMode(13, OUTPUT);
  pinMode(A1, INPUT);
}

void loop() {
   // считываем данные с датчика и выводим на монитор порта
   if (digitalRead(A1) == HIGH) {
      digitalWrite (13, LOW);
  }
   if (digitalRead(A1) == LOW) {
      digitalWrite (13, HIGH);
  }
}
/*
 * фоторещистор (отдельно)
 */
 int potpin=0;// initialize analog pin 0, connected with photovaristor
int ledpin=11;// initialize digital pin 11, output regulating the brightness of LED
int val=0;// initialize variable va
void setup()
{
pinMode(ledpin,OUTPUT);// set digital pin 11 as “output”
Serial.begin(9600);// set baud rate at “9600”
}
void loop()
{
val=analogRead(potpin);// read the analog value of the sensor and assign it to val
Serial.println(val);// display the value of val
analogWrite(ledpin,val/4);// turn on the LED and set up brightness（maximum output value 255）
delay(10);// wait for 0.01
}
/*
 * ИК датчик движения
 */
 byte sensorPin = 3;
byte indicator = 13;
void setup()
{
  pinMode(sensorPin,INPUT);
  pinMode(indicator,OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  byte state = digitalRead(sensorPin);
  digitalWrite(indicator,state);
  if(state == 1)Serial.println("Somebody is in this area!");
  else if(state == 0)Serial.println("No one!");
  delay(500);
}
/*релешка
 * 
 */
  int Relay = 8;
  void setup()
{
  pinMode(13, OUTPUT);         //Set Pin13 as output
  digitalWrite(13, HIGH);     //Set Pin13 High
  pinMode(Relay, OUTPUT);     //Set Pin3 as output
}
void loop()
{
          digitalWrite(Relay, HIGH);   //Turn off relay
          delay(2000);
          digitalWrite(Relay, LOW);    //Turn on relay
          delay(2000);
}
