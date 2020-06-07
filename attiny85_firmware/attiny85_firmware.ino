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
#define RESAMPLE_NUMBER 3
#define DELAY_BETWEEN_SAMPLES 1000
#define WIFI_WAKEUP_DELAY 5000
#define SEND_DELAY 1000

#define SLEEP 0
#define SAMPLE 1
#define SEND_MSG 2




SoftwareSerial mySerial(RX_PIN, TX_PIN);

static Dht11 sensor(DHT_DATA_PIN);

volatile int watchdog_counter;
volatile int state;
String msg;
int resample_number;
int t_sample;
int h_sample;



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


void sampling()
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


void getMsg()
{
  msg = "{";
  msg += "\"Temperature\": ";
  msg += t_sample;
  msg += ", ";
  msg += "\"Humidity\": ";
  msg += h_sample;
  msg += "}";
  msg += "#";
}


void move2FirstState()
{
  t_sample = 0;
  h_sample = 0;
  resample_number = 0;
  msg = "";
  watchdog_counter = 0;
  state = SLEEP;
}


void sleep()
{
  //~ 15 min
  if(watchdog_counter > 90)
  {
    state = SAMPLE;
    return;
  }

  system_sleep();
}


void sample()
{
  if(resample_number >= RESAMPLE_NUMBER)
  {
    getMsg();
    
    state = SEND_MSG;
    return;
  }

  sampling();

  delay(DELAY_BETWEEN_SAMPLES);
}


void sendMsg()
{
  wakeupWifi();
  delay(WIFI_WAKEUP_DELAY);
  
  mySerial.print(msg);

  delay(SEND_DELAY);

  move2FirstState();
}


ISR(WDT_vect)
{  
  watchdog_counter++;
}





void setup()
{
  mySerial.begin(SERIAL_BAUD);
  
  move2FirstState();

  // ~ 10s
  setup_watchdog(9);
}



void loop()
{
  switch(state)
  {
    case SLEEP:
      sleep();
      break;

    case SAMPLE:
      sample();
      break;

    case SEND_MSG:
      sendMsg();
      break;
  }
}
