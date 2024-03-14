#include <Arduino.h>
#define RxPin 16
#define TxPin 17
#define BAUDRATE 115200
HardwareSerial SerialAT(2);

#define sw 13
#define wake_up 15
#define led 14
int flag = 1;
int flag1 = 0;
int state=0;
String Publish = "henrik/test"; //Publish Topic
String Subscribe = "henrik/test"; //Subscribe Topic
String Clientid = "elementz/123"; //Client ID - change this for each client as this need to be unique
String Brokeradr = "tcp://iot.traceme.dk:1883,90,0";

bool wait_for_char(char ch, uint16_t timeout);
bool wait_for_respons(String respons, uint16_t timeout);
String gsm_send_serial(String command, int delay);
void publish_message(String topic, String msg);
void led_blink();
void handle_MQTT();
void setup_esp();
void connect_MQTT();
void disconnect_MQTT();
void setup_7022();
void setup_PSM();


void setup() 
{
  setup_esp();
  setup_7022();
  connect_MQTT();
}

void loop() 
{
  static unsigned long next_time = millis() + 2000;  

  led_blink();

  if (millis() > next_time)
  {
    next_time += 60000;
    handle_MQTT();
  }
}

bool wait_for_respons(String respons, uint16_t timeout)
{
  
}

bool wait_for_char(char ch, uint16_t timeout)
{
  bool done = false;
  bool ok = false;
  unsigned long exit_time = millis() + timeout;

  while((done == false) && (millis() < exit_time))
  { 
    if(SerialAT.available() > 0)
    {
      // read the incoming byte:
      byte buffer[5];
      SerialAT.readBytes(buffer, 1);
      Serial.println(buffer[0], HEX);
      if (buffer[0] == ch)
      {
        done = true;
        ok = true;
      }
    }
  }
  return ok;
}

void handle_MQTT()
{
  digitalWrite(wake_up, HIGH);
  delay(500);
  digitalWrite(wake_up, LOW);
  // put in wakeup
  //PUBLISH MESSAGE
  static uint8_t n=0;
  publish_message(Publish, String(n++));
  //publish_message("morten", "henrik");  
  delay(1000);
  // put in sleep
}

void setup_PSM()
{
  SerialAT.println("AT+QCPMUCFG=0");
  delay(1000);
  SerialAT.println("AT+QCPSMR=0"); 
  delay(1000);
  //SerialAT.println("AT+CPSMS=1,,,\"01000101\",\"00100001\");
  SerialAT.println("AT+CPSMS=0,,,\"01000101\",\"00001111\"");
  delay(10000);
  SerialAT.println("AT+QCPMUCFG=1,1");
  delay(100);
  SerialAT.println("AT+QCPMUCFG=0");
  delay(100);
  SerialAT.println("AT+CPSMS=0");
  delay(5000); 
}

void connect_MQTT()
{
  //digitalWrite(PA9, HIGH);
  Serial.println("Connect MQTT");
  SerialAT.println("AT+CMQTTCONNECT?");  // test connect
  delay(100);  
  // wait_for_char('\r',100); //wait for CR
  delay(100);
  SerialAT.println("AT+CMQTTACCQ?");  // test Client ID
  delay(100);
  SerialAT.println("AT+CMQTTSTART=?");  // test connect
  delay(100);
  SerialAT.println("AT+CMQTTSTART"); //Establishing MQTT Connection
  delay(1000); 
  // SerialAT.println("AT+CMQTTACCQ=0,\"elementz/123\""); //Client ID - change this for each client as this need to be unique
  SerialAT.println("AT+CMQTTACCQ=0," + Clientid); //Client ID - change this for each client as this need to be unique
  delay(2000);
  SerialAT.println("AT+CMQTTCONNECT=0," + Brokeradr); //MQTT Server Name for connecting this client (org ,1)
  //SerialAT.println("AT+CMQTTCONNECT=0,\"tcp://iot.traceme.dk:1883\",90,0"); //MQTT Server Name for connecting this client (org ,1)
  //SerialAT.println("AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",90,1"); //MQTT Server Name for connecting this client
  delay(1000);
  // digitalWrite(PA9, LOW);
}

void disconnect_MQTT()
{
  SerialAT.println("AT+CMQTTDISC=0,60"); //Establishing MQTT Connection
  delay(1000);
  SerialAT.println("AT+CMQTTREL=0"); //Establishing MQTT Connection
  delay(1000);
  SerialAT.println("AT+CMQTTSTOP"); //Establishing MQTT Connection
  delay(1000);
}

void setup_7022()
{
  SerialAT.println("AT+VIOSET=2");
  delay(1000);
  SerialAT.println("AT+CPSMS=0,,,\"01000101\",\"00001111\"");
  // Serial.println("Connecting To Server........");  
  delay(1000);
  //SerialAT.println("AT+QCDNSCFG=\"8.8.8.8\",\"4.4.4.4\""); // setup name server
  //delay(1000);  
  SerialAT.println("AT+QCDNSCFG?");  
  delay(1000);
  SerialAT.println("AT+CGMM");  
  delay(1000);
  SerialAT.println("ATI");  
  delay(1000);
  SerialAT.println("AT+CGSN=2");  // Read IMEI
  delay(1000);
  SerialAT.println("AT+COPS=0"); // Auto connect (tdc= AT+COPS=1,2,"28301",9)
  delay(2000);
  SerialAT.println("AT+CGCONTRDP"); // Auto connect !! vent på IP adr
  delay(5000);
  SerialAT.println("AT+COPS?"); //
  delay(1000); 
  SerialAT.println("AT+CGPADDR"); // Request the assigned IP address
  delay(1000);
  SerialAT.println("AT+CREG?");  // Auto connect (tdc= AT+COPS=1,2,"28301",9)
  delay(1000);
}
String gsm_send_serial(String command, int delay)
{
  String buff_resp = "";
  Serial.println("Send ->: " + command);
  SerialAT.println(command);
  long wtimer = millis();
  while (wtimer + delay > millis())
  {
      while (SerialAT.available())
      {
          buff_resp = SerialAT.readString();
          Serial2.println(buff_resp);
      }
  }
  Serial.println();
  return buff_resp;
}

void publish_message(String topic, String msg)
{
  while(SerialAT.available() > 0)
  {
    char incomingByte = SerialAT.read();
  }

  String cmd = "AT+CMQTTTOPIC=0," + String(topic.length());
  SerialAT.println(cmd);  
  wait_for_char('>',100);
  SerialAT.println(topic);  
  delay(1000);

  String payload = "AT+CMQTTPAYLOAD=0," + String(msg.length());
  SerialAT.println(payload);  
  wait_for_char('>',100);
  SerialAT.println(msg);  
  delay(1000);

  SerialAT.println("AT+CMQTTPUB=0,1,60");
  delay(1000);
}

void led_blink()
{
  static unsigned long next_time = millis() + 1000;  
  static bool on = false;
  if (millis() > next_time)
  {
    next_time += 1000;
    if (on)
    {
      digitalWrite(led, LOW);
      on = false;
    }
    else
    {
      digitalWrite(led, HIGH);
      on = true;
    }
  }
}

void setup_esp()
{
  Serial.begin(115200);
  SerialAT.begin(BAUDRATE, SERIAL_8N1, RxPin, TxPin);;
  pinMode(sw, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  pinMode(wake_up, OUTPUT);
  digitalWrite(wake_up, LOW);
}

//###################################################################################################################

// #include <Arduino.h>
// #define MODEM_TX PB6
// #define MODEM_RX PB7
// //HardwareSerial Serial2(PA3, PA2);
// HardwareSerial SerialAT(PA10, PA9);

// #define sw PA0
// #define wake_up PB9
// #define led PC13
// int flag = 1;
// int flag1 = 0;
// int state=0;
// String Publish = "henrik/test"; //Publish Topic
// String Subscribe = "henrik/test"; //Subscribe Topic

// bool wait_for_char(char ch, uint16_t timeout)
// {
//   bool done = false;
//   bool ok = false;
//   unsigned long exit_time = millis() + timeout;

//   while((done == false) && (millis() < exit_time))
//   { 
//     if(SerialAT.available() > 0)
//     {
//       // read the incoming byte:
//       byte buffer[5];
//       SerialAT.readBytes(buffer, 1);
//       Serial.println(buffer[0], HEX);
//       if (buffer[0] == ch)
//       {
//         done = true;
//         ok = true;
//       }
//     }
//   }
//   return ok;
// }

// String gsm_send_serial(String command, int delay)
// {
//   String buff_resp = "";
//   Serial.println("Send ->: " + command);
//   SerialAT.println(command);
//   long wtimer = millis();
//   while (wtimer + delay > millis())
//   {
//       while (SerialAT.available())
//       {
//           buff_resp = SerialAT.readString();
//           Serial2.println(buff_resp);
//       }
//   }
//   Serial.println();
//   return buff_resp;
// }

// void publish_message(String topic, String msg)
// {
//   while(SerialAT.available() > 0)
//   {
//     char incomingByte = SerialAT.read();
//   }

//   String cmd = "AT+CMQTTTOPIC=0," + String(topic.length());
//   SerialAT.println(cmd);  
//   wait_for_char('>',100);
//   SerialAT.println(topic);  
//   delay(1000);

//   String payload = "AT+CMQTTPAYLOAD=0," + String(msg.length());
//   SerialAT.println(payload);  
//   wait_for_char('>',100);
//   SerialAT.println(msg);  
//   delay(1000);

//   SerialAT.println("AT+CMQTTPUB=0,1,60");
//   delay(1000);
// }


// void setup() 
// {
//   Serial.begin(115200);
//   SerialAT.begin(115200);
//   pinMode(sw, INPUT_PULLUP);
//   pinMode(led, OUTPUT);
//   digitalWrite(led, LOW);
//   pinMode(wake_up, OUTPUT);
//   digitalWrite(wake_up, LOW);
//   //SerialAT.println("modem");
//   //AT Commands for setting up the client id and Server
//   //Need to be executed once -- Open serial terminal doe seeing the debug messages
//   SerialAT.println("AT+VIOSET=2");
//   delay(1000);
//   SerialAT.println("AT+CPSMS=0,,,\"01000101\",\"00001111\"");
//   // Serial.println("Connecting To Server........");  
//   delay(1000);
//   //SerialAT.println("AT+QCDNSCFG=\"8.8.8.8\",\"4.4.4.4\"");
//   //delay(1000);  
//   SerialAT.println("AT+QCDNSCFG?");  
//   delay(1000);
//   SerialAT.println("AT+CGMM");  
//   delay(1000);
//   SerialAT.println("ATI");  
//   delay(1000);
//   SerialAT.println("AT+CGSN=2");  
//   delay(1000);

//   // SerialAT.println("AT+CEREG=1"); // Let the modem display network connection status
//   // delay(100);
//   //SerialAT.println("AT+CFUN=0"); // Switch off the radio module
//   //delay(100);
//   // SerialAT.println("AT+QCBAND=0,20"); // Set the frequency band to 20 for TDC (think can read
//   // delay(100);
//   // SerialAT.println("AT+CFUN=1,1"); // Switch on the radio again
//   // delay(100);
//   // SerialAT.println("AT+CGDCONT=0,,\"internet\""); // Set APN AT+CGDCONT= 0 -> 10
//   // delay(100);
//   // SerialAT.println("AT+CGDCONT=1,,\"nbiot.tdc.dk\""); // Set APN AT+CGDCONT= 0 -> 10
//   // delay(100);
//   SerialAT.println("AT+COPS=0"); // Auto connect (tdc= AT+COPS=1,2,"28301",9)
//   delay(2000);
//   SerialAT.println("AT+CGCONTRDP"); // Auto connect !! vent på IP adr
//   delay(5000);
//   SerialAT.println("AT+COPS?"); //
//   delay(1000); 
//   SerialAT.println("AT+CGPADDR"); // Request the assigned IP address
//   delay(1000);
//   SerialAT.println("AT+CREG?");  // Auto connect (tdc= AT+COPS=1,2,"28301",9)
//   delay(1000);
//   delay(1000);
//   // SerialAT.println("AT+CPSMS=0");  
//   // delay(1000);
//   // SerialAT.println("AT+QCPSMR=0");  
//   // delay(1000);
//   // SerialAT.println("AT+CGDCONT?");
//   // delay(1000);
//   // //SerialAT.println("AT+CPSMS=1");
//   // delay(5000);
//   // //SerialAT.println("AT+QCPMUCFG=0,0");
//   // delay(100);

//   //SUBSCRIBE MESSAGE
//   //Need to be executed once
//   // SerialAT.println("AT+CMQTTSUBTOPIC=0,9,1"); //AT Command for Setting up the Subscribe Topic Name 
//   // delay(1000);
//   // SerialAT.println(Subscribe); //Topic Name
//   // delay(1000);
//   // SerialAT.println("AT+CMQTTSUB=0,4,1,1"); //Length of message
//   // delay(1000);
//   // SerialAT.println("HAII"); //message
//   // delay(1000);
//   // Serial2.println("Done");
//   // SerialAT.println(""); //MQTT Server Name for connecting this client
//   // delay(1000);
// }

// void led_blink()
// {
//   static unsigned long next_time = millis() + 1000;  
//   static bool on = false;
//   if (millis() > next_time)
//   {
//     next_time += 1000;
//     if (on)
//     {
//       digitalWrite(led, LOW);
//       on = false;
//     }
//     else
//     {
//       digitalWrite(led, HIGH);
//       on = true;
//     }
//   }
// }

// void handle_MQTT()
// {
//   //digitalWrite(PA9, HIGH);
//   static uint8_t n=0;
//   Serial.println("Connect MQTT");
//   digitalWrite(wake_up, HIGH);
//   delay(500);
//   digitalWrite(wake_up, LOW);
//   SerialAT.println("AT");
//   delay(100);
//   SerialAT.println("AT+QCPMUCFG=0");
//   delay(100);
//   SerialAT.println("AT+CPSMS=0");
//   delay(100);
//   SerialAT.println("AT+CGCONTRDP"); // Auto connect !! vent på IP adr
//   delay(5000);  
//   SerialAT.println("AT+CMQTTSTART"); //Establishing MQTT Connection
//   delay(1000); 
//   SerialAT.println("AT+CMQTTACCQ=0,\"elementz/123\""); //Client ID - change this for each client as this need to be unique
//   delay(2000);
//   SerialAT.println("AT+CMQTTCONNECT=0,\"tcp://iot.traceme.dk:1883\",90,0"); //MQTT Server Name for connecting this client (org ,1)
//   //SerialAT.println("AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",90,1"); //MQTT Server Name for connecting this client
//   delay(1000);
//   //PUBLISH MESSAGE
//   publish_message(Publish, String(n++));
//   publish_message("morten", "henrik");  
//   SerialAT.println("AT+CMQTTDISC=0,60"); //Establishing MQTT Connection
//   delay(1000);
//   SerialAT.println("AT+CMQTTREL=0"); //Establishing MQTT Connection
//   delay(1000);
//   SerialAT.println("AT+CMQTTSTOP"); //Establishing MQTT Connection
//   delay(1000);
//   //
//   //SerialAT.println("AT+QCPSMR=?");
//   //delay(1000);
//   SerialAT.println("AT+QCPMUCFG=0");
//   delay(1000);
//   SerialAT.println("AT+QCPSMR=0"); 
//   delay(1000);
//   //SerialAT.println("AT+CPSMS=1,,,\"01000101\",\"00100001\");
//   SerialAT.println("AT+CPSMS=0,,,\"01000101\",\"00001111\"");
//   delay(10000);
//   SerialAT.println("AT+QCPMUCFG=1,1");
//  // digitalWrite(PA9, LOW);
//   delay(100);
// }

// void loop() 
// {
//   static unsigned long next_time = millis() + 2000;  

//   led_blink();

//   if (millis() > next_time)
//   {
//     next_time += 50000;
//     handle_MQTT();
//   }

// }



//   // String a;
//   // if(state==0)
//   // {
//   //   if(digitalRead(sw) == 0 && flag1 == 0)
//   //   {
//   //     flag1 = 1;
//   //     digitalWrite(led, HIGH);
//   //     Serial.println("Publishing Message: LED ON");
//   //     //PUBLISH MESSAGE
//   //     publish_message(Publish, "a");
//   //   }
//   //   else if(digitalRead(sw) == 0 && flag1 == 1)
//   //   {
//   //     flag1 = 0;
//   //     digitalWrite(led, LOW); 
//   //     Serial.println("Publishing Message: LED OFF");
//   //     //PUBLISH MESSAGE
//   //     publish_message(Publish, "b");
//   //   }
//   // }
//   // if(state==1)
//   // {
//   //   if(digitalRead(sw) == 0 && flag1 == 0)
//   //   {
//   //     //PUBLISH MESSAGE
//   //     flag1 = 1;
//   //     digitalWrite(led, LOW);
//   //     Serial.println("Publishing Message: LED OFF");
//   //     //PUBLISH MESSAGE
//   //     publish_message(Publish, "b");
//   //   }
//   //   else if(digitalRead(sw) == 0 && flag1 == 1)
//   //   {
//   //     flag1 = 0;
//   //     digitalWrite(led,HIGH); 
//   //     Serial.println("Publishing Message: LED ON");
//   //     //PUBLISH MESSAGE
//   //     publish_message(Publish, "b");
//   //   }

//  //Receiving MODEM Response
// //   while(SerialAT.available()>0)
// //   {
// //     delay(10);
// //     a = SerialAT.readString();
// //     if(flag==0)
// //     {
// //       //Serial2.println(a);
// //     flag = 1;
// //     }
// //     //Serial2.println(b);
// //     if(a.indexOf("PAYLOAD") != -1)
// //     {
// //        flag = 0;
// //        int new1 = a.indexOf("PAYLOAD");
// //        String neww = a.substring(new1);
// //        int new2 = neww.indexOf('\n');
// //        String new3 = neww.substring(new2+1);
// //        int new4 = new3.indexOf('\n');
// //        String new5 = new3.substring(0,new4);
       
// //        Serial.println("Topic: led/subscribe");
// //        Serial.print("Message is: ");
// //        Serial.println(new5);
// //        new5.remove(new5.length()-1);
// //        if(new5 == "a")
// //        {
// //         state=1;
// //         Serial.println("LED ON");
// //         digitalWrite(led, HIGH);
// //        }
// //        else if(new5 == "b")
// //        {
// //         state=0;
// //         flag1=0;
// //         Serial.println("LED OFF");
// //         digitalWrite(led, LOW);
// //        }
// //     }      
// //   }
// // }