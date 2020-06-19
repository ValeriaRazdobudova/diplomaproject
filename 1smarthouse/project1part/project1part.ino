
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include "timer-api.h"

#include  "Modem.h"
#include "SmsMaker.h"

#include "Constants.h"
#include "Util.h"


/*  -- THEMPERATURE библ  --  */

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);
DeviceAddress sensorAddr;
/*
 * пины подкл on
 */
SoftwareSerial mySerial(8,9);
String voice;
int sensor=A1;
float temp_read,Temp_alert_val,Temp_shut_val,temp_ac;
int sms_count=0,Fire_Set;
int sensorPin=A0; //Select the input pin for the LDR
int sensorValue=0;//variable to store the input value
int trigPin=13; //Trigger Pin
int echoPin=12;
long duration;
int distance;
int led1=2; //Connect Led1 to Pin#2
int led2=3; //Connect Led2 to Pin#3
int led3=4; //Connect Led3 to Pin#4
int led4=5; //Connect Led4 to Pin#5
int led5=6; //Connect Led5 to Pin#6
int led6=7; //Connect Led6 to Pin#7

// flags
volatile uint8_t flags;
#define CHECK_FLAG          0       // flag on check sensors values
#define RESET_FLAG          1       //  reset system flag
#define     Free_flag_2         2
#define     Free_flag_3         3
#define     Free_flag_4         4
#define MOTION_SEND_FLAG    5       //set after send  sms on motion in house
#define COOL_SEND_FLAG      6       //set after send  sms on very small firing themperature
#define VOLTAGE_SEND_FLAG   7       //set after send  sms on drop external power
volatile uint8_t warningFlags;
// if flag set need send Warning sms
#define COOL_SMS            0
#define MOTION_SMS          1
#define VOLTAGE_SMS         2

uint8_t volatile motionCounter = 0;
unsigned volatile int dropMotionTimer = 0;
uint8_t volatile delayEspired = 0;

// temperature
uint8_t coolThemperature = MIN_FIRING_THEMPERATURE;

enum modes { DUCT = 0, TEST = 1, WORK = 2};
uint8_t mode = WORK;


void setup() {
  wdt_reset();
  wdt_disable();
  // start up serial port`s
  Serial.begin(9600);
  Serial.println(F("\n\tStart_Smart_Home_Apps v_07\n")); 
  // SIM
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, SIM_RST);
  delay(9000);
  // led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  //motion sensor
  pinMode(MOTION_SENSOR_PIN, INPUT);
  digitalWrite(MOTION_SENSOR_PIN, LOW);

  //  start up the motion sensor handler - external intr
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motion_sensor_INTR_handler, RISING);

  // Start up the Dallas library
  sensors.begin();
  // get address for connected sensors
  if (!sensors.getAddress(sensorAddr, 0)) Serial.println(F("\nDallas Lib -Unable to find address for Device 0"));
  /*
    if (!sensors.getAddress(sensorAddr, 0)) Serial.println(F("Unable to find address for Device 0"));
    else {
    Serial.print("\n\tDevice address = ");
    printAddress(sensorAddr);
    }
  */
 
    // --------------- warnings SMS ---------------------
    void SendTextMessage(){
    
    mySerial.println("AT+CMGF=1");//To send SMS in text Mode
    delay(2000);
    mySerial.println("AT+CMGS=\"+Enter the number here\"r");//Enter the phone number that you 
    //using and enter the country code also
    delay(2000);
    mySerial.println("Fire in New Room");//the message content to be displayed for SMS
    delay(2000);
    mySerial.println((char)26);//the stopping character
    delay(5000);
    mySerial.println("AT+CMGS=\"+Enter the number here\"r");//Enter the phone number that you 
    //using and enter the country code also
    delay(2000);
    mySerial.println("Fire in New Room");//the message content to be displayed for SMS
    delay(2000);
    mySerial.println((char)26);//the stopping character
    {delay(5000);
    sms_count++;
    }}
    

    // ---------------  check themperature  -----------------
    if (bitRead(flags, CHECK_FLAG) == 1)executeScheduledTask();


/*  ====================================================
    ====================================================
*/
/*  DS18B20 ADDRESS  */
/*
  // function to print a device address
  void printAddress(DeviceAddress deviceAddress)
  {
  Serial.print("{ ");
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print(deviceAddress[i]);
    Serial.print(":");
  }
  Serial.print(" }\n");
  }
*/

/*
   on click Button change mode and
   reflect current mode
   DUCT - arduino work as pipe beetwen modem amd computer
   TEST - no sms send mode( device check sensors and execute all actions
            as normally work exclude sms send). Also in this mode groowe
            debug messages
*/
void buttonHandle() {
  delay(20);
  int pressCounter = 0;
  for (uint8_t i = 0; i < 6; i++) {
    if (digitalRead(BUTTON_PIN) == LOW)pressCounter++;
    else pressCounter--;
    delay(20);
  }
  if (pressCounter > 2 ) {
    Serial.print("\n ___ mode : ");
    switch (mode) {
      case DUCT: mode = TEST; Serial.println("TEST");
        break;
      case TEST: mode = WORK ; Serial.println("WORK");
        break;
      case WORK: mode = DUCT; Serial.println("DUCT");
      default:
        Serial.println(" ___");
    }
    modem.changeShowCommand(mode);
  }
  while (digitalRead(BUTTON_PIN) == LOW) {}
}



/* ==============================================
                    motion sensor signals
   ==============================================
*/

void motion_sensor_INTR_handler(void) {
  dropMotionTimer = DROP_MOTION_TIME;   // start wait for drop motion counter
  if (bitRead(flags, MOTION_SEND_FLAG) == 1 )bitSet(warningFlags, MOTION_SMS);
  if (motionCounter < 6) motionCounter++;
}

/*
    timer handler
    execute some shedule tasks
*/
void timer_handle_interrupts(int timer) {
  //   inner variables
  static uint16_t timerCounter;
  static uint8_t loopCounter;

  //  measure time for check themperature
  static uint16_t checkSensors;
  //  for set flags leaves send messages
  static uint16_t coolEspired;
  static uint16_t motionEspired;
  static uint16_t voltageEspired;

  // delayEspired
  // wait delay for answer from modem
  if (delayEspired > 0 ) {
    delayEspired--;
  }

  //  check sensors values
  checkSensors++;
  if (checkSensors == CHECK_SENSOR_TICK) {
    bitSet(flags, CHECK_FLAG);
    checkSensors = 0;
  }

  //  motion espired
  //  run motion sensor set espired time > 0
  // if espired time is 1 drop motion counter
  if (dropMotionTimer > 0) dropMotionTimer--;
  else motionCounter = 0;

  //    ==========  SEND SMS  ================
  //  work with delay time beetwen send cool sms
  if (bitRead(flags, COOL_SEND_FLAG) == 0) {
    coolEspired ++;
    if (coolEspired == DELAY_FOR_COOL) {
      bitSet(flags, COOL_SEND_FLAG);
      coolEspired = 0;
    }
  }
  //  work with delay time beetwen send motion sms
  if (bitRead(flags, MOTION_SEND_FLAG) == 0) {
    motionEspired ++;
    if (motionEspired == DELAY_FOR_MOTION) {
      bitSet(flags, MOTION_SEND_FLAG);
      motionEspired = 0;
    }
  }
  //  work with delay time beetwen send drop voltage sms
  if (bitRead(flags, VOLTAGE_SEND_FLAG) == 0) {
    voltageEspired ++;
    if (voltageEspired == DELAY_FOR_VOLTAGE) {
      bitSet(flags, VOLTAGE_SEND_FLAG);
      voltageEspired = 0;
    }
  }

  // blink on DUCT
  if (digitalRead(DUCT_PIN) == LOW) digitalWrite( LED_PIN, !digitalRead(LED_PIN));

  //  WDT soft reset after 4 timerCounter loop
  //  device make restart after ~ 2 days work
  timerCounter ++;
#if DEBUG == 0
  if (timerCounter == 43200) {
#else
  if (timerCounter == 300) {
#endif
    loopCounter++;
    timerCounter = 0;
    if (loopCounter == 4) {
      digitalWrite(RST_PIN, LOW);
      delay(9000);
      //  REBOOT the device after 2 days work
      wdt_enable (WDTO_8S);
      Serial.println(F("\n___________________________\n\tAFTER 8 seconds to be RESET\n___________________________"));
      delay(9000);

    }
  }
/*
 * TEPERATURE
 */
 /* DHT library 

MIT license
written by Adafruit Industries
*/

#include "DHT.h"

DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
  _pin = pin;
  _type = type;
  _count = count;
  firstreading = true;
}

void DHT::begin(void) {
  // set up the pins!
  pinMode(_pin, INPUT);
  digitalWrite(_pin, HIGH);
  _lastreadtime = 0;
}

//boolean S == Scale.  True == Farenheit; False == Celcius
float DHT::readTemperature(bool S) {
  float f;

  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if(S)
       f = convertCtoF(f);
        
      return f;
    case DHT22:
    case DHT21:
      f = data[2] & 0x7F;
      f *= 256;
      f += data[3];
      f /= 10;
      if (data[2] & 0x80)
  f *= -1;
      if(S)
  f = convertCtoF(f);

      return f;
    }
  }
  Serial.print("Read fail");
  return NAN;
}

float DHT::convertCtoF(float c) {
  return c * 9 / 5 + 32;
}

float DHT::readHumidity(void) {
  float f;
  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[0];
      return f;
    case DHT22:
    case DHT21:
      f = data[0];
      f *= 256;
      f += data[1];
      f /= 10;
      return f;
    }
  }
  Serial.print("Read fail");
  return NAN;
}


boolean DHT::read(void) {
  uint8_t laststate = HIGH;
  uint8_t counter = 0;
  uint8_t j = 0, i;
  unsigned long currenttime;

  // pull the pin high and wait 250 milliseconds
  digitalWrite(_pin, HIGH);
  delay(250);

  currenttime = millis();
  if (currenttime < _lastreadtime) {
    // ie there was a rollover
    _lastreadtime = 0;
  }
  if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
    return true; // return last correct measurement
    //delay(2000 - (currenttime - _lastreadtime));
  }
  firstreading = false;
  /*
    Serial.print("Currtime: "); Serial.print(currenttime);
    Serial.print(" Lasttime: "); Serial.print(_lastreadtime);
  */
  _lastreadtime = millis();

  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  
  // now pull it low for ~20 milliseconds
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);
  cli();
  digitalWrite(_pin, HIGH);
  delayMicroseconds(40);
  pinMode(_pin, INPUT);

  // read in timings
  for ( i=0; i< MAXTIMINGS; i++) {
    counter = 0;
    while (digitalRead(_pin) == laststate) {
      counter++;
      delayMicroseconds(1);
      if (counter == 255) {
        break;
      }
    }
    laststate = digitalRead(_pin);

    if (counter == 255) break;

    // ignore first 3 transitions
    if ((i >= 4) && (i%2 == 0)) {
      // shove each bit into the storage bytes
      data[j/8] <<= 1;
      if (counter > _count)
        data[j/8] |= 1;
      j++;
    }

  }

  sei();
  
  /*
  Serial.println(j, DEC);
  Serial.print(data[0], HEX); Serial.print(", ");
  Serial.print(data[1], HEX); Serial.print(", ");
  Serial.print(data[2], HEX); Serial.print(", ");
  Serial.print(data[3], HEX); Serial.print(", ");
  Serial.print(data[4], HEX); Serial.print(" =? ");
  Serial.println(data[0] + data[1] + data[2] + data[3], HEX);
  */

  // check we read 40 bits and that the checksum matches
  if ((j >= 40) && 
      (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {
    return true;
  }
  

  return false;

}
/*
 * 
 */
