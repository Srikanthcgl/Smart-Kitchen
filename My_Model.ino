#define BLYNK_PRINT Serial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
//#define BLYNK_TEMPLATE_ID   "YourTemplateID"


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <DHT.h>
#include <Servo.h>


char auth[] = "_EM_emVAKyjShp5m4Q4eUwtNc_0JznCn";
char ssid[] = "OnePlus";
char pass[] = "12345678";

// An IR detector/demodulator is connected to GPIO pin 5 (D1 on a NodeMCU
// board).
// Note: GPIO 16 won't work on the ESP8266 as it does not have interrupts.
const uint16_t kRecvPin = 5;
unsigned long key_value = 0;
// Control LEDs with the 
const int greenPin = 4; 
const int yellowPin = 2;

DHT dht(D3, DHT11); //(sensor pin,sensor type)
BlynkTimer timer;

Servo servo;

#define MQ2 A0
#define Buzzer D0

IRrecv irrecv(kRecvPin);

decode_results results;

void setup()
{
  // Debug console
  Serial.begin(115200);
  irrecv.enableIRIn();  // Start the receiver
  while (!Serial)  // Wait for the serial connection to be establised.
  delay(50);
  Serial.println();
  Serial.print("IRrecvDemo is now running and waiting for IR message on Pin ");
  Serial.println(kRecvPin);
  pinMode(greenPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(D8,OUTPUT);
  dht.begin();

  timer.setInterval(100L, gassensor);
  timer.setInterval(100L, DHT11sensor);

  servo.attach(15);
  servo.write(0);
  delay(2000);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
}

//Get the MQ2 sensor values
void gassensor() {
  int value = analogRead(MQ2);
  Serial.println(value);
  value = map(value, 0, 1024, 0, 100);
  if (value <= 55) {
    digitalWrite(Buzzer, LOW);
  } else if (value > 55) {
    Blynk.notify("Warning! Gas leak detected");
    digitalWrite(Buzzer, HIGH);
  }
 Blynk.virtualWrite(V1, value);
}

//Get the DHT11 sensor values
void DHT11sensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Blynk.virtualWrite(V2, t);
  Blynk.virtualWrite(V3, h);
}

void loop() {
  if (irrecv.decode(&results)) {
    // print() & println() can't handle printing long longs. (uint64_t)
    serialPrintUint64(results.value, HEX);
    Serial.println("");

    switch(results.value){
          case 0xFFA25D:
          Serial.println("1");
          // green LED on for 2 seconds
          digitalWrite(greenPin, HIGH);
          Blynk.notify("led 1 is on");
          delay(2000);
          digitalWrite(greenPin, LOW);
          break;
          case 0xFF629D:
          Serial.println("2");
           // yellow LED on for 2 seconds
          digitalWrite(yellowPin, HIGH);
          Blynk.notify("led 2 is on");
          delay(2000);
          digitalWrite(yellowPin, LOW);
          break;
          case 0xFFE21D:
          Serial.println("3");
          break;
          case 0xFF22DD:
          Serial.println("4");
          break;
          case 0xFF02FD:
          Serial.println("5");
          break ;  
          case 0xFFC23D:
          Serial.println("6");
          break ;               
          case 0xFFE01F:
          Serial.println("7");
          break ;  
          case 0xFFA857:
          Serial.println("8");
          break ;  
          case 0xFF906F:
          Serial.println("9");
          break ;  
          case 0xFF6897:
          Serial.println("100+");
          break ;  
          case 0xFF9867:
          Serial.println("0");
          break ;
          case 0xFFB04F:
          Serial.println("200+");
          break ;
          case 0xFF18E7:
          Serial.println("CH+");
          servo.write(180);
          delay(500);
          Blynk.notify("Door Opened");
          break ;
          case 0xFF10EF:
          Serial.println("-");
          break ;
          case 0xFF38C7:
          Serial.println("EQ");
          break ;
          case 0xFF5AA5:
          Serial.println("+");
          break ;
          case 0xFF4AB5:
          Serial.println("CH-");
          servo.write(0);
          delay(500);
          Blynk.notify("Door Closed");
          break ;     
        }
        key_value = results.value;
        
    irrecv.resume();  // Receive the next value
  }
  delay(100);
  Blynk.run();
  timer.run();
}
