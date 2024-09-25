#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

const int mq135Pin = A1; // Menggunakan pin A1 untuk sensor MQ135
const int phPin    = A2; // Menggunakan pin A2 untuk sensor pH
const int temperaturePin  = A0; // Menggunakan pin A3 untuk sensor Temperature NTC Thermistor 10 K

unsigned long previousMillis = 0;
const long interval = 5000; // interval pembacaan sensor setiap 5 detik

void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    float temperature = getTemperature(temperaturePin, 'C');
    float ammonia = bacaAmmonia(mq135Pin);
    float phValue = bacaPh(phPin);

    Serial.print("T");
    Serial.println(temperature);
    Serial.print("A");
    Serial.println(ammonia);
    Serial.print("P");
    Serial.println(phValue);

    updateLCD(temperature, ammonia, phValue);
  }
}

float getTemperature(uint8_t pin, char unit) {
  const float R1 = 10000;  
  const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  int adc = analogRead(pin);
  float R2 = R1 * (1023.0 / (float)adc - 1.0);
  float logR2 = log(R2);
  float T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));  
  float Tc = T - 273.15;                                            
  float Tf = (Tc * 9.0) / 5.0 + 32.0;                                
  if (unit == 'K') return T;
  else if (unit == 'F') return Tf;
  else return Tc;
}

float bacaAmmonia(int pin) {
  float VRL = analogRead(pin) * (3.3 / 10230);
  float RS = (3.3 / VRL - 1) * 10;
  float Ro = 28; 
  float ratio = RS / Ro;
  float ppm = pow(10, ((log10(ratio) - 0.858) / -0.417));
  return ppm;
}

float bacaPh(int pin) {
  int adcPH = analogRead(pin);
  float voltage = adcPH * (5.0 / 1023.0);
  float pHValue = 7.0 - (voltage - 2.5); 
  pHValue = constrain(pHValue, 0.0, 14.0);
  return pHValue;
}

void updateLCD(float temperature, float ammonia, float phValue) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  BULE App - UNIDA  ");

  lcd.setCursor(0, 1);
  lcd.print("Temp    : ");
  lcd.print(temperature);
  lcd.setCursor(15, 1);
  lcd.print(" *C");

  lcd.setCursor(0, 2);
  lcd.print("Ammonia : ");
  lcd.print(ammonia);
  lcd.setCursor(15, 2);
  lcd.print(" mg/L");

  lcd.setCursor(0, 3);
  lcd.print("pH      : ");
  lcd.print(phValue);
  lcd.setCursor(15, 3);
  lcd.print(" pH");
}
