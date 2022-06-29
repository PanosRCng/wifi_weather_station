#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>


#define AP_SSID "XXXXXXXXXX"
#define AP_PASSWD  "XXXXXXXXXXXXXXX"
 
#define HTTP_SERVER_IP "XXX.XXX.XXX.XXX"
#define HTTP_SERVER_ENDPOINT "XXXXXXXXXXXXXXX"


#define SERIAL_BAUD 9600

#define CONNECT_RETRY 3
#define CONNECT_RETRY_DELAY 5000
#define SEND_RETRY 3
#define SEND_RETRY_DELAY 2000


#define WAIT_MSG 0
#define AP_CONNECT 1
#define SEND_MSG 2


IPAddress ip(XXX.XXX.XXX.XXX);
IPAddress gateway(XXX.XXX.XXX.XXX);
IPAddress subnet(XXX.XXX.XXX.XXX);


int state;
String msg;
int connect_retry;
int send_retry;




void getMsg()
{
  if(Serial.available())
  {    
    msg = Serial.readStringUntil('#');

    if(msg.length() < 5)
    {
      sleep();
      return;
    }
    
    state = AP_CONNECT;
  }
}


void connect2AP()
{  
  if(connect_retry++ >= (CONNECT_RETRY + 1))
  {
    sleep();
    return;
  }
  
  if(connect_retry > 1)
  {
    delay(CONNECT_RETRY_DELAY);
  }
  
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(AP_SSID, AP_PASSWD); 

  if(WiFi.status() == WL_CONNECTED)
  {     
    state = SEND_MSG;
  }
}


void sendMsg()
{
  if(send_retry++ >= (SEND_RETRY + 1))
  {
    sleep();
    return;
  }

  if(send_retry > 1)
  {
    delay(SEND_RETRY_DELAY);
  }

  String url = "http://";
  url += HTTP_SERVER_IP;
  url += HTTP_SERVER_ENDPOINT;

  String data = "msg=";
  data += msg;

  HTTPClient http;

  http.begin(url);
  http.addHeader("content-type", "application/x-www-form-urlencoded");

  int httpCode = http.POST(data);

  if(httpCode == HTTP_CODE_OK)
  {
    sleep();
    return;
  } 
  
  http.end();
}


void sleep()
{
  msg = "";
  connect_retry = 0;
  send_retry = 0;
  state = WAIT_MSG;

  ESP.deepSleep(0);
}



void setup()
{
  Serial.begin(SERIAL_BAUD);

  msg = "";
  connect_retry = 0;
  send_retry = 0;
  state = WAIT_MSG; 
}


void loop()
{  
  switch(state)
  {
    case WAIT_MSG:
      getMsg();
      break;

    case AP_CONNECT:
      connect2AP();
      break;

    case SEND_MSG:
      sendMsg();
      break;
  }
}
