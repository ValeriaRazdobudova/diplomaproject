//---------------Перевірка на займання у приміщенні------------//

void CheckFire() {
	Temp_alert_val = CheckTemp();
	Serial.print(Temp_alert_val);
	if (Temp_alert_val > 45) {
		Fire_Set = 1;
		while (sms_count < 3)//Кількість повідомлень для відправки 
		{
			SmsMaker();

		}
	}
}
#include "SmsMaker.h"

extern volatile uint8_t delayEspired;

SmsMaker::SmsMaker(Modem *m) {
	modem = m;
}

/*
   Відправка повідомлення
*/
bool SmsMaker::sendWarningSms(uint8_t clientOrder, int messageType) {
	if (digitalRead(DUCT_PIN) == LOW) {
		Serial.print(F(" спроба send_WarningSms відправки повідомлення користувачу"));
		Serial.print(String(clientOrder) + " msgTYPE=" + String(messageType));
	}
	char mess[MAX_MESS_LEN];
	char phone[49];
	if (modem->fillUCS2PhoneNumber(clientOrder, phone) == false) {
#if DEBUG
		Serial.println(F(" помилка! такий номер не можливий"));
#endif
		return false;
	}
	phone[48] = '\0';
	strcpy_P(mess, (char *)pgm_read_word(&(messages[messageType])));
#if DEBUG
	/*
	  uint8_t i = 0;
	  Serial.print(F(" text msg: "));
	  while (mess[i] != '\0') {
		Serial.write(mess[i]);
		i++;
	  }
	  Serial.println(F(" :"));
	*/
#endif
	return sendSms("UCS2\0", phone, mess);
}
