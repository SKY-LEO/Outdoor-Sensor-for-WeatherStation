#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <RH_ASK.h>
#include <TinyWireM.h>
#include <forcedClimate.h>
#define rx 4
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC
RH_ASK driver(2000, NO_PIN, rx); // D3
struct datastruct
{
  int32_t temp;
  int32_t pres;
  int32_t hum;
  int32_t vcc;
} mydata;
byte tx_buf[sizeof(mydata)] = {0};
volatile byte watchdog_counter;
ForcedClimate climateSensor = ForcedClimate(TinyWireM, 0x76);

void setup()
{
  driver.init();
  setup_watchdog(WDTO_8S);
  sei();
}

void loop()
{
  ATtiny85_sleep();
  if (watchdog_counter >= 1)
  {
    watchdog_counter = 0;
    adc_enable();
    pinMode(1, OUTPUT);
    digitalWrite(1, HIGH);
    delay(200);
    TinyWireM.begin();
    climateSensor.begin();
    mydata.vcc = readVcc();
    delay(100);
    mydata.temp = climateSensor.getTemperatureCelcius(true);
    mydata.pres = climateSensor.getPressure(true);
    mydata.hum = climateSensor.getRelativeHumidity(true);
    delay(100);
    memcpy(tx_buf, &mydata, sizeof(mydata));
    byte zize = sizeof(mydata);
    driver.send((uint8_t*)&tx_buf, zize);
    driver.waitPacketSent();
    delay(200);
    digitalWrite(1, LOW);
    all_pins_input();
  }
}

void ATtiny85_sleep()
{
  adc_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void setup_watchdog(byte sleep_time)
{
  wdt_enable(sleep_time);
  WDTCR |= _BV(WDIE);     /* enable interrupts instead of MCU reset when watchdog is timed out
                         used for wake-up MCU from power-down */
}

ISR(WDT_vect)
{
  watchdog_counter++;
}

void all_pins_input()
{
  for (byte ATtiny_pin; ATtiny_pin < 5; ATtiny_pin++)
  {
    pinMode(ATtiny_pin, INPUT);
  }
}

uint32_t readVcc()
{
  // Установка референса в Vcc и измерение внутреннего Vbg == 1.1V
  ADMUX = _BV(MUX3) | _BV(MUX2);
  delay(2); // требуется минимум 1ms для стабилизации
  ADCSRA |= _BV(ADSC); // ADC Start conversion - начать опрос
  while (bit_is_set(ADCSRA, ADSC)); // ADSC == 1 пока опрос активен

  // Чтение ADCL блокирует ADCH, чтение ADCH разблокирует оба регистра => ADCL читать первым
  uint8_t low  = ADCL; // младший регистр данных ADC
  uint8_t high = ADCH; // старший регистр данных ADC

  long result = (high << 8) | low;

  result = 1098888L / result; // 1125300 = 1.1*1023*1000
  return result; // Vcc в милливольтах
}
