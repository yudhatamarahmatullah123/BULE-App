#include <Wire.h>
// #include <LiquidCrystal_I2C.h>
#include <LiquidCrystal_PCF8574.h>

// Inisialisasi LCD dengan alamat I2C dan ukuran layar
LiquidCrystal_PCF8574 lcd(0x27);

// Definisi pin untuk sensor
const int temperaturePin = A0; // Pin untuk thermistor NTC 10k ohm
const int mq135Pin = A1; // Pin untuk sensor MQ135
const int phPin = A2;    // Pin untuk sensor pH

// Interval pembacaan sensor
unsigned long previousMillis = 0;
const long interval = 5000; // interval pembacaan sensor setiap 5 detik

// Parameter untuk thermistor
const float R1 = 10000.0;  // Nilai resistor pull-up 10k ohm
const float R25 = 10000.0; // Nilai thermistor pada 25°C 10k ohm
const float BETA = 3950.0; // Koefisien BETA untuk thermistor 10k ohm
const float VREF = 5.0;    // Tegangan referensi (biasanya 5V pada Arduino)
const float calibrationOffset = 7.5; // Offset kalibrasi suhu

// Variabel kalibrasi pH
float calibration_offset = 0.0;
float calibration_slope = -5.70;

// Parameter untuk MQ135
// Nilai resistansi dasar Ro, sesuaikan dengan hasil kalibrasi
float Ro = 28.0;
// Faktor konversi ppm ke mg/L [re:https://pakguru.co.id/konversi-ppm-ke-mg-l/]
const float faktor_konversi = 0.001; // 1 ppm = 1 mg/L
// Persamaan logaritmik untuk mendeteksi NH3
float calculateNH3(float RS_Ro_ratio) {
  return pow(10, ((log10(RS_Ro_ratio) - 0.587) / -0.23)); // Rumus log untuk NH3
}

// Custom karakter untuk simbol derajat
byte degreeChar[8] = {
  0B00110,
  0B01001,
  0B01001,
  0B00110,
  0B00000,
  0B00000,
  0B00000
};

void setup() {
  Serial.begin(115200);
  lcd.begin(20, 4);  // Inisialisasi LCD ukuran 20x4
  lcd.setBacklight(255);  // Menyalakan backlight

  // Buat karakter custom
  lcd.createChar(0, degreeChar); // Menyimpan karakter custom di memori LCD
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
  delay(100); 
  int adc = analogRead(pin);
  
  // Jika nilai ADC sangat rendah atau tinggi, sensor tidak terhubung
  if (adc < 10 || adc > 1013) {
    // Serial.println("Sensor Suhu tidak terhubung!");
    return 0.0; // Nilai default jika sensor tidak terhubung
  }
  
  float voltage = adc * (VREF / 1023.0);
  float R2 = R1 * ((VREF / voltage) - 1.0);
  
  if (R2 <= 0) {
    return 0.0; // Nilai default jika perhitungan tidak valid
  }
  
  float logR2 = log(R2 / R25);
  float T_kelvin = 1.0 / (1.0 / 298.15 + logR2 / BETA);
  float T_celsius = T_kelvin - 273.15;
  T_celsius += calibrationOffset;
  
  if (unit == 'F') {
    return (T_celsius * 9.0 / 5.0) + 32.0;
  } else {
    return T_celsius;
  }
}

float bacaPh(int pin) {
  int adcPH = analogRead(pin);
  
  // Cek apakah sensor pH terhubung
  if (adcPH < 10 || adcPH > 1013) {
    // Serial.println("Sensor pH tidak terhubung!");
    return 7.0; // Nilai default pH
  }

  float voltage = adcPH * (5.0 / 1023.0);
  float pHValue = 7.0 - (voltage - 2.5); 
  pHValue = constrain(pHValue, 0.0, 14.0);
  
  return pHValue;
}

// float bacaAmmonia(int pin) {
//   int adc = analogRead(pin); // Membaca nilai ADC dari sensor MQ135
//   float VRL = adc * (3.3 / 10230); // Menghitung tegangan VRL (output dari sensor)
  
//   // Jika VRL di luar rentang normal, berarti sensor tidak terhubung
//   if (VRL < 0.1 || VRL > 4.9) {
//     Serial.println("Sensor Ammonia tidak terhubung!");
//     return -999.0; // Nilai default untuk ammonia
//   }
  
//   float RS = (3.3 / VRL - 1.0) * 10.0; // Menghitung resistansi RS
//   float Ro = 28.0; // Ro yang didapat dari kalibrasi
//   float ratio = RS / Ro; // Menghitung rasio RS/Ro
  
//   // Mengonversi rasio ke ppm (gunakan persamaan sensor yang tepat)
//   float ppm = pow(10, ((log10(ratio) - 0.858) / -0.417));

//   // Menyesuaikan skala dari ppm ke mg/L jika diperlukan
//   // Sesuaikan faktor ini dengan hasil kalibrasi yang akurat
//   float ammonia_mgL = ppm * 0.5; // Misalnya, faktor konversi ppm ke mg/L
  
//   return ammonia_mgL;
// }
float bacaAmmonia(int pin) {
  int adc = analogRead(pin); // Membaca nilai ADC dari sensor MQ135
  float VRL = adc * (VREF / 1023.0); // Menghitung tegangan output VRL dari sensor

  // Jika nilai tegangan di luar rentang normal, sensor mungkin tidak terhubung
  if (VRL < 0.1 || VRL > 4.9) {
    // Serial.println("Sensor Ammonia tidak terhubung!");
    return -999.0; // Nilai default untuk ammonia jika sensor tidak terhubung
  }
  
  // Menghitung resistansi sensor (RS) dalam kohm
  float RS = (VREF / VRL - 1) * 10.0; // Nilai resistor beban RL = 10k ohm
  
  // Menghitung rasio RS/Ro
  float RS_Ro_ratio = RS / Ro;
  
  // Menghitung konsentrasi ammonia menggunakan persamaan logaritmik
  float ammonia_ppm = calculateNH3(RS_Ro_ratio);
  
  // Jika diperlukan, sesuaikan atau konversikan ppm ke mg/L
  float ammonia_mgL = ammonia_ppm * faktor_konversi;
  
  return ammonia_mgL;
}

void updateLCD(float temperature, float ammonia, float phValue) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  BULE App - UNIDA  ");

  lcd.setCursor(0, 1);
  if (temperature == 0.0) {
    lcd.print("Temp    : ERR");
  } else {
    lcd.print("Temp    : ");
    lcd.print(temperature);
    lcd.print(" ");
    lcd.write(0); // Menampilkan simbol derajat
    lcd.print("C");
  }

  lcd.setCursor(0, 2);
  if (ammonia == 0.0) {
    lcd.print("Ammonia : ERR");
  } else {
    lcd.print("Ammonia : ");
    lcd.print(ammonia);
    lcd.print(" ml/L");
  }

  lcd.setCursor(0, 3);
  if (phValue == 7.0) {
    lcd.print("pH      : ERR");
  } else {
    lcd.print("pH      : ");
    lcd.print(phValue);
    lcd.print(" pH");
  }
}

float readVoltage(int pin) {
  int buffer[10];
  unsigned long int avg = 0;
  
  // Ambil 10 bacaan untuk mendapatkan rata-rata
  for (int i = 0; i < 10; i++) {
    buffer[i] = analogRead(pin);
    delay(10);
  }
  
  for (int i = 0; i < 10; i++) {
    avg += buffer[i];
  }
  
  avg /= 10;
  return avg * (5.0 / 1023.0);
}