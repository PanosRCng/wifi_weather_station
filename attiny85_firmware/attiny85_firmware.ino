#include "Arduino.h"
#include "Dht11.h"
#include <SoftwareSerial.h>

#include <avr/io.h>
#include <avr/sleep.h>



#define DHT_DATA_PIN 1
#define WIFI_WAKEUP_PIN 0
#define RX_PIN 3
#define TX_PIN 4

#define SERIAL_BAUD 9600
#define RESAMPLE_NUMBER 0
#define DELAY_BETWEEN_SAMPLES 1000
#define WIFI_WAKEUP_DELAY 1000

#define SAMPLE 0
#define SEND_MSG 1
#define SLEEP 2

#define LOW_VATTERY_THRESHOLD 3



SoftwareSerial mySerial(RX_PIN, TX_PIN);

static Dht11 sensor(DHT_DATA_PIN);

volatile int watchdog_counter;
volatile int state;
volatile int halfhour_counter;
String msg;
int resample_number;
int t_sample;
int h_sample;
double battery_voltage;
bool battery_low;


void wakeupWifi()
{
  DDRB |= (1 << WIFI_WAKEUP_PIN);
  PORTB &= ~(1 << WIFI_WAKEUP_PIN);
  delay(100);
  DDRB &= ~(1 << WIFI_WAKEUP_PIN);
}


void system_sleep()
{          
  // SLEEP_MODE_PWR_DOWN
  MCUCR |= (1 << SM1);
  MCUCR &= ~(1 << SM0);
    
  // sleep_enable
  MCUCR |= (1 << SE);
    
  sleep_mode();
    
  // sleep_disable
  MCUCR &= ~(1 << SE);
}


void initADC()
{
  ADMUX =
            (1 << ADLAR) |     // left shift result
            (1 << REFS2) |     // Sets ref. voltage to 2.5v internal, bit 1
            (1 << REFS1) |     // Sets ref. voltage to 2.5v internal, bit 1
            (0 << REFS0) |     // Sets ref. voltage to 2.5v internal, bit 0
            (0 << MUX3)  |     // use ADC1 for input (PB2), MUX bit 3
            (0 << MUX2)  |     // use ADC1 for input (PB2), MUX bit 2
            (0 << MUX1)  |     // use ADC1 for input (PB2), MUX bit 1
            (1 << MUX0);       // use ADC1 for input (PB2), MUX bit 0

  ADCSRA = 
            (1 << ADEN)  |     // Enable ADC 
            (1 << ADPS2) |     // set prescaler to 64, bit 2 
            (1 << ADPS1) |     // set prescaler to 64, bit 1 
            (0 << ADPS0);      // set prescaler to 64, bit 0  
}


void setup_watchdog(int ii)
{
  byte bb;
  int ww;

  if(ii > 9)
  {
    ii = 9;
  }

  bb = ii & 7;

  if(ii > 7)
  {
    bb |= (1<<5);
  }

  bb |= (1<<WDCE);

  ww == bb;

  MCUSR &= ~(1<<WDRF);

  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);

  // set new watchdog timeout value
  WDTCR = bb;

  WDTCR |= _BV(WDIE);
}


void sample_battery()
{
  ADCSRA |= (1 << ADSC);
  
  while(ADCSRA & (1 << ADSC));
  
  battery_voltage = ADCH;

  battery_voltage = battery_voltage * 0.02;

  if(battery_voltage <= LOW_VATTERY_THRESHOLD)
  {
    battery_low = true;
  }
  else
  {
    battery_low = false;
  }
}


void sample_temp_humi()
{
  switch( sensor.read() )
  {
      
    case Dht11::OK:
      t_sample = sensor.getTemperature();
      h_sample = sensor.getHumidity();
      resample_number++;  
      break;

    /*
    case Dht11::ERROR_CHECKSUM:
      mySerial.print("Checksum error");
      mySerial.println("#");
      break;

    case Dht11::ERROR_TIMEOUT:
      mySerial.print("Timeout error");
      mySerial.println("#");
      break;
    default:
      mySerial.print("Unknown error");
      mySerial.println("#");
      break;
    */
    default:
      t_sample = -1;
      h_sample = -1;
      break;  
  }
}


void sampling()
{
  sample_battery();

  sample_temp_humi();
}


void getMsg()
{
  msg = "{";
  msg += "\"type\": \"info\",";
  msg += "\"data\": ";
  msg += "{";
  msg += "\"Temperature\": ";
  msg += t_sample;
  msg += ", ";
  msg += "\"Humidity\": ";
  msg += h_sample;
  msg += ", ";
  msg += "\"Light\": ";
  msg += battery_voltage;
  msg += "}";
  msg += "}";
  msg += "#";
}


void move2FirstState()
{  
  t_sample = 0;
  h_sample = 0;
  battery_voltage = 0;
  resample_number = 0;
  msg = "";
  state = SAMPLE;
}


void sample()
{  
  if(resample_number >= (RESAMPLE_NUMBER + 1))
  {
    getMsg();
    
    state = SEND_MSG;
    return;
  }

  sampling();

  if(battery_low)
  {
    state = SLEEP;
    return;
  }

  delay(DELAY_BETWEEN_SAMPLES);
}


void sendMsg()
{
  wakeupWifi();
  delay(WIFI_WAKEUP_DELAY);
  
  mySerial.print(msg);

  halfhour_counter = 0;
  watchdog_counter = 0;
  state = SLEEP;
}


void sleep()
{
  //~ 30 min
  if(watchdog_counter > 180)
  {
    watchdog_counter = 0;
    halfhour_counter++;

    if(halfhour_counter >= 4)
    {
      move2FirstState();
      return;
    }
  }

  system_sleep();
}


ISR(WDT_vect)
{  
  watchdog_counter++;
}





void setup()
{
  initADC();
  
  mySerial.begin(SERIAL_BAUD);

  // ~ 10s
  setup_watchdog(9);

  battery_low = false;

  move2FirstState();
}



void loop()
{
  switch(state)
  {
    case SAMPLE:
      sample();
      break;

    case SEND_MSG:
      sendMsg();
      break;

    case SLEEP:
      sleep();
      break;
  }
}
