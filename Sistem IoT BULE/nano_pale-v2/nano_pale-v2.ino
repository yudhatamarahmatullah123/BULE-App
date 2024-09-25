#include <Wire.h>
// #include <LiquidCrystal_I2C.h>
#include <LiquidCrystal_PCF8574.h>

// Inisialisasi LCD dengan alamat I2C dan ukuran layar
LiquidCrystal_PCF8574 lcd(0x27);

// Definisi pin untuk sensor
const int mq135Pin = A1; // Pin untuk sensor MQ135
const int phPin = A2;    // Pin untuk sensor pH
const int temperaturePin = A0; // Pin untuk thermistor NTC 10k ohm

// Interval pembacaan sensor
unsigned long previousMillis = 0;
const long interval = 5000; // interval pembacaan sensor setiap 5 detik

// Parameter untuk thermistor
const float R1 = 10000.0;  // Nilai resistor pull-up 10k ohm
const float R25 = 10000.0; // Nilai thermistor pada 25°C 10k ohm
const float BETA = 3950.0; // Koefisien BETA untuk thermistor 10k ohm
const float VREF = 5.0;    // Tegangan referensi (biasanya 5V pada Arduino)
const float calibrationOffset = 7.5; // Offset kalibrasi suhu

void setup() {
  Serial.begin(115200);
  lcd.begin(20, 4);  // Inisialisasi LCD ukuran 20x4
  lcd.setBacklight(255);  // Menyalakan backlight
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Membaca nilai dari sensor
    float temperature = getTemperature(temperaturePin, 'C');
    float ammonia = bacaAmmonia(mq135Pin);
    float phValue = bacaPh(phPin);

    // Menampilkan hasil ke Serial Monitor
    Serial.print("T");
    Serial.println(temperature);
    Serial.print("A");
    Serial.println(ammonia);
    Serial.print("P");
    Serial.println(phValue);

    // Menampilkan hasil ke LCD
    updateLCD(temperature, ammonia, phValue);
  }
}

float getTemperature(uint8_t pin, char unit) {
  // Membaca nilai ADC
  int adc = analogRead(pin);
  
  // Menghitung tegangan dari ADC
  float voltage = adc * (VREF / 1023.0);
  
  // Menghitung resistansi thermistor
  float R2 = R1 * ((VREF / voltage) - 1.0);
  
  // Menangani kasus di mana R2 = 0 untuk menghindari perhitungan log yang tidak valid
  if (R2 <= 0) {
    return -999.0; // Nilai error jika resistansi thermistor tidak valid
  }
  
  // Menggunakan rumus BETA untuk perhitungan suhu
  float logR2 = log(R2 / R25);
  float T_kelvin = 1.0 / (1.0 / 298.15 + logR2 / BETA); // Kelvin
  float T_celsius = T_kelvin - 273.15; // Celsius
  
  // Mengaplikasikan kalibrasi
  T_celsius += calibrationOffset;
  
  // Menghitung suhu dalam Fahrenheit dan Kelvin
  float T_fahrenheit = (T_celsius * 9.0 / 5.0) + 32.0;

  if (unit == 'K') return T_kelvin;
  else if (unit == 'F') return T_fahrenheit;
  else return T_celsius;
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
  delay(100); // Tambahkan delay

  lcd.setCursor(0, 1);
  lcd.print("Temp    : ");
  lcd.print(temperature);
  lcd.setCursor(15, 1);
  lcd.print(" *C");
  delay(100); // Tambahkan delay

  lcd.setCursor(0, 2);
  lcd.print("Ammonia : ");
  lcd.print(ammonia);
  lcd.setCursor(15, 2);
  lcd.print(" mg/L");
  delay(100); // Tambahkan delay

  lcd.setCursor(0, 3);
  lcd.print("pH      : ");
  lcd.print(phValue);
  lcd.setCursor(15, 3);
  lcd.print(" pH");
  delay(100); // Tambahkan delay
}