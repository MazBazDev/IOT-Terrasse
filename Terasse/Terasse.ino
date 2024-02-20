#include <Wire.h>
#include <BH1750.h>
#include "DHT.h"
#include <Servo.h>

BH1750 LIGTH_METER;

#define TEMP_PIN 3
DHT dht(TEMP_PIN, DHT11);

#define POTENTIOMETER_PIN A3
#define PIR_PIN 2

#define MOTOR_PWM 11
#define MOTOR_HIGH 9
#define MOTOR_LOW 8

#define LED_BLUE 5
#define LED_GREEN 6

#define BUTTON_BLIND_DOWN 12
#define BUTTON_BLIND_UP 13
#define BUTTON_MANUAL 7

#define servo_pin 10

Servo SERVO;

bool light_switch = true;
int selected_led = 0;
int temperatureThreshold_1 = 30;
int temperatureThreshold_2 = 25;
int temperatureThreshold_3 = 26;
int motionDetected = 0;
int lightIntensityThreshold = 500;

boolean b1_value = false;
boolean b1_last_state = HIGH;

boolean b2_value = false;
boolean b2_last_state = HIGH;

boolean is_manual = false;
boolean b3_last_state = HIGH;

bool blind_opened = false;
bool light_on = false;


void setup() {
  Serial.begin(9600);
  LIGTH_METER.begin();
  dht.begin();

  pinMode(PIR_PIN, INPUT); 

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_HIGH, OUTPUT);
  pinMode(MOTOR_LOW, OUTPUT);

  SERVO.attach(servo_pin);

  digitalWrite(MOTOR_HIGH, HIGH);
  digitalWrite(MOTOR_LOW, LOW);

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  pinMode(BUTTON_BLIND_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_BLIND_UP, INPUT_PULLUP);
  pinMode(BUTTON_MANUAL, INPUT_PULLUP);
  
  Serial.println("Init complete");
}

void loop() {
  logs();

  int temp = dht.readTemperature();
  int lightLevel = LIGTH_METER.readLightLevel();

  boolean b1 = digitalRead(BUTTON_BLIND_DOWN);
  if(b1_last_state == HIGH && b1==LOW)
  {
    if(is_manual) {
      b1_value = !b1_value;
      interactBlind();
    }
  }
  b1_last_state = b1;

  boolean b2 = digitalRead(BUTTON_BLIND_UP);
  if(b2_last_state == HIGH && b2==LOW)
  {
    if(is_manual) {
      b2_value = !b2_value;
      interactLight();
    }

  }
  b2_last_state = b2;

  boolean b3 = digitalRead(BUTTON_MANUAL);
  if(b3_last_state == HIGH && b3==LOW)
  {
    is_manual = !is_manual;
  }
  b3_last_state = b3;

  if (analogRead(POTENTIOMETER_PIN) < 511) {
    selected_led = 0;
  } else {
    selected_led = 1;
  }
  
  motionDetected = digitalRead(PIR_PIN);
  if (is_manual == 0) {  
    if (lightLevel > lightIntensityThreshold) {
      closeBlind();
    } else {
      openBlind();
    }
  }
  
  if (temp < temperatureThreshold_1) {
    fan(0);
  } else if (temp < temperatureThreshold_2) {
    fan(1);
  } else if (temp < temperatureThreshold_3) {
    fan(2);
  } else {
    fan(3); 
  }

  if (light_switch && is_manual == 0) {
    if (lightLevel > lightIntensityThreshold) {
      lightDown();
    } else if (motionDetected) {
      lightUp();
    } else {
      lightLow(); 
    }
  }

  if (is_manual == 1){
    delay(500);
  }
}

void lightUp() {
  if(selected_led == 0) {
    analogWrite(LED_BLUE, 0);
    analogWrite(LED_GREEN, 255);
  } else if (selected_led == 1) {
    analogWrite(LED_GREEN, 0);
    analogWrite(LED_BLUE, 255);
  }
}

void lightDown() {
  analogWrite(LED_GREEN, 255);
  analogWrite(LED_BLUE, 255);
}

void lightLow() {
  if(selected_led == 0) {
    analogWrite(LED_BLUE, 128);
    analogWrite(LED_GREEN, 255);
  } else if (selected_led == 1) {
    analogWrite(LED_GREEN, 128);
    analogWrite(LED_BLUE, 255);
  }
}

void fan(int speed) {
  if (speed == 0) {
    analogWrite(MOTOR_PWM, 0);
  } else if (speed == 1) {
    analogWrite(MOTOR_PWM, 190);
    delay(100);
    analogWrite(MOTOR_PWM, 100);
  } else if (speed == 2) {
    analogWrite(MOTOR_PWM, 150);
  } else if (speed == 3) {
    analogWrite(MOTOR_PWM, 255);
  }
}

void interactBlind() {

  if (blind_opened) {
    closeBlind();
  } else {
    openBlind();
  }
}

void interactLight() {
  if (b2_value == 1) {
    lightUp();
  } else {
    lightDown();
  }
}

void openBlind() {
  blind_opened = true;

  SERVO.write(90); 
  delay(500);
}

void closeBlind() {
  blind_opened = false;

  SERVO.write(0);
  delay(500);
}




void logs() {
  Serial.print("Luminosité : ");
  Serial.print(LIGTH_METER.readLightLevel());
  Serial.println(" lux");
  Serial.print("Temperature = ");
  Serial.print(dht.readTemperature());
  Serial.println(" °C");
  Serial.print("Humidite = ");
  Serial.print(dht.readHumidity());
  Serial.println(" %");
  Serial.print("Mouvement ? : ");
  Serial.println(digitalRead(PIR_PIN));
  Serial.print("Potentio : ");
  Serial.println(analogRead(POTENTIOMETER_PIN));


  Serial.print("Btn 1 : ");
  Serial.print(b1_value);

   Serial.print("Btn 2 : ");
  Serial.print(b2_value);

   Serial.print("Btn 3 : ");
  Serial.print(is_manual);
  Serial.println();

}
