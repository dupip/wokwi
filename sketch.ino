#include "LiquidCrystal_I2C.h"
#include "RTClib.h" // Bisa untuk RTC PCF8563
#include "DHT.h"
#include "Wire.h"
#include "MQ135.h"
#include "EEPROM.h"

#define DHTPIN 7
#define DHT2PIN 8
#define DHTTYPE DHT22

RTC_DS1307 rtc; //RTC_DS1307 ganti dengan RTC_PCF8563
LiquidCrystal_I2C lcd(0x27, 20, 4);
DHT dht(DHTPIN, DHTTYPE);
DHT dht2(DHT2PIN, DHTTYPE);

const int tomSetPin = A3;
const int threshold = 100;
const int mqPin = A1; // MQ135 dihubungkan ke pin A1
const int mq2Pin = A2; // MQ135 dihubungkan ke pin A2
MQ135 gasSensor = MQ135(mqPin);
MQ135 gas2Sensor = MQ135(mq2Pin);

const int relay1Pin = 6;
const int relay2Pin = 5;
const int relay3Pin = 4;
const int relay4Pin = 3;
const int relay5Pin = 2;

const uint8_t tomPin[] = { 13, 12, 11, 10, 9};
const int jlTom = 5;
const float BETA = 3950; // should match the Beta Coefficient of the thermistor

float targetTemp = 37.5;          // Suhu mesin dijaga pada suhu ini
float targetHum = 52.5;           // Kelembaban mesin dijaga disini
byte tomSw = 0;
byte setTimer=0;
bool relay3Active = false;
unsigned long waitTime = 300000;  // Waktu tunggu pengecekan kelembaban setelah suhu naik 2 derajat
                                  // (5 menit) 1 menit = 60.000 milidetik

int tmp;
bool mtrNyala = 0;
bool mtrAktif = 0;
long targetTime=10800; // Motor nyala berulang setiap targetTime (10800)
long lamaNyala = 90;  // Lama nyala motor (90)
long targetTimeRem;
long lamaNyalaRem;
int remainingTime = 0;
unsigned long previousMillis = 0;
unsigned long previousMillis1 = 0;

bool pemanasOn = false;
float suhuMin2 = 0.0;
bool tunggu5Mnt = false;
unsigned long tunggu5Mnt1 = 0;
unsigned long currentMillis = 0;
bool suhuNaik2 = false;
byte suhuNaik = 2;
unsigned long tunggu5MntDur = 300; // Dalam satuan detik

float deltaPPM = 75;

bool fullOtomatis = 0;
float suhuH0H3 = 38;
float suhuH4H17 = 37.8;
float suhuH18 = 37.4;
float suhuH19 = 37;
float suhuH20 = 36.6;
float suhuH21 = 36.2;
float rhH0H3 = 53;
float rhH4H17 = 55;
float rhH18H21 = 60;
float rhH22 = 50;
bool mtrH0H3 = 0;
bool mtrH4H17 = 1;
bool mtrH18H22 = 0;
float deltaSuhu = 0.5;
float deltaRH = 1;

byte hari0Day = 22;
byte hari0Month = 9;
int hari0Year = 2023;
int hariKe = 0; // Diupdate setiap hari jam 06:00
bool updateParHariKe=0;

int  setThn1;
int setBln1;
int setHr1;
int setJam1;
int setMnt1;
int setDet1;

bool kedip=0;
bool ambilDtnow=0;
bool upDateTgljam=0;
bool upDateEEPROM=0;

float temp;
float hum;
float temp2;
float hum2;
float suhuSekarang;
float ppm;
float ppm2;
char buffer1at10kar[10];
char buffer2at10kar[10];
char buffer1at7kar[7];
char buffer2at7kar[7];
bool simpanMulaiPenetasan = 0;
DateTime now;

void setup() {
  //Serial.begin(9600);
  for (uint8_t i = 0; i < jlTom; i++) {
    pinMode(tomPin[i], INPUT_PULLUP);
  }
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);
  pinMode(relay4Pin, OUTPUT);
  pinMode(relay5Pin, OUTPUT);

  pinMode(tomSetPin, INPUT_PULLUP);

  lcd.begin(20, 4);
  lcd.backlight();
  rtc.begin();
  dht.begin();
  dht2.begin();
  int storedValue = EEPROM.read(0); 
  if (storedValue==72) {getFromEEPROM();}else{saveToEEPROM();}
}

void loop() {
  unsigned long currentMillis;
  currentMillis = millis();
  if (currentMillis - previousMillis >= 250) {
    previousMillis = currentMillis;
    now = rtc.now();
    temp = dht.readTemperature(); // Baca suhu dari sensor DHT22
    hum = dht.readHumidity();       // Baca kelembaban dari sensor DHT22
    temp2 = dht2.readTemperature(); // Baca suhu dari sensor DHT22
    hum2 = dht2.readHumidity();       // Baca kelembaban dari sensor DHT22
    suhuSekarang = bacaSuhuAir();
    ppm = gasSensor.getPPM();
    ppm2 = gas2Sensor.getPPM();

    if (ambilDtnow) {
      setThn1 = now.year();
      setBln1 = now.month();
      setHr1  = now.day();
      setJam1 = now.hour();
      setMnt1 = now.minute();
      setDet1 = now.second();
      ambilDtnow=0;
    }

    if (upDateEEPROM) {
      upDateEEPROM=0;
      saveToEEPROM();
    }

    if (simpanMulaiPenetasan){
      simpanMulaiPenetasan=0;
      EEPROM.put(70, hari0Day);
      EEPROM.put(71, hari0Month);
      EEPROM.put(72, hari0Year);
    }

    if (upDateTgljam){
      rtc.adjust(DateTime(setThn1, setBln1, setHr1, setJam1, setMnt1, setDet1));
      upDateTgljam=0;
    }

    int tmpHarike = (now - DateTime(hari0Year, hari0Month, hari0Day, 6, 0, 0)).days(); // Hitung hari ke berapa
    if (hariKe!=tmpHarike){
      updateParHariKe=1;
      hariKe = tmpHarike;
      targetTimeRem = targetTime + millis() / 1000;
    } else{
      updateParHariKe=0;
    }

    if (fullOtomatis && updateParHariKe){
      if (hariKe < 4) {targetTemp = suhuH4H17;targetHum = rhH4H17;mtrAktif = 0;
      } else if (hariKe < 18) {targetTemp = suhuH4H17;targetHum = rhH4H17;mtrAktif = 1; targetTimeRem = 1 + millis() / 1000;;
      } else if (hariKe < 19) {targetTemp = suhuH18;targetHum = rhH18H21;mtrAktif = 0;
      } else if (hariKe < 20) {targetTemp = suhuH19;
      } else if (hariKe < 21) {targetTemp = suhuH20;
      } else if (hariKe < 22) {targetTemp = suhuH21;targetHum = rhH22;}
    } 
    //lcd.clear();
    lcd.setCursor(0, 0);
    
    tmp = now.day();   if (tmp<10) {lcd.print("0");} lcd.print(tmp, DEC); lcd.print('-');
    tmp = now.month(); if (tmp<10) {lcd.print("0");} lcd.print(tmp, DEC); lcd.print('-');
    tmp = now.year();  lcd.print(tmp, DEC);  lcd.print("  ");
    tmp = now.hour();  if (tmp<10) {lcd.print("0");} lcd.print(tmp, DEC); lcd.print(':');
    tmp = now.minute();if (tmp<10) {lcd.print("0");} lcd.print(tmp, DEC); lcd.print(":");
    tmp = now.second();if (tmp<10) {lcd.print("0");} lcd.print(tmp, DEC);
    
    lcd.setCursor(0, 1); lcd.print(temp); lcd.write((uint8_t)223); lcd.print("C ");

    if ((currentMillis - previousMillis1 >= 1000) && fullOtomatis) {
      previousMillis1 = currentMillis;
      lcd.print("   ");
    } else {
      String hariKeSt = String(hariKe);
      if (hariKe<0) {hariKeSt = "00";}
      else if (hariKe<10){hariKeSt="0"+hariKeSt;}
      else if (hariKe>99){hariKeSt="99";}
      lcd.print(hariKeSt+" ");
    }

    lcd.print("%RH:"); lcd.print(hum);

    // Relay 5 On jika selisih ppm  > deltaPPM
    float bedaPpm = abs(ppm-ppm2);
    if (bedaPpm > deltaPPM) {
      digitalWrite(relay5Pin, HIGH); // Menyalakan relay
    } else {
      digitalWrite(relay5Pin, LOW); // Mematikan relay
    }

    // Relay 4 On jika beda suhu >= 0.5
    // Lakukan perataan suhu mesin tetas
    float bedaSuhu = abs(temp-temp2);
    float bedaRH = abs(hum-hum2);
    if (bedaSuhu >= deltaSuhu || bedaRH > deltaRH) {
      digitalWrite(relay4Pin, HIGH);
    } else {
      digitalWrite(relay4Pin, LOW);
    }

    // Relay 3 On jika %RH < #RH Target dengan metode:
    // 1. Naikkan suhu air x derajat kemudian tunggu 5 menit
    // 2. Jika kelembaban %RH < #RH Target ulangi langkah 1
    if (suhuSekarang > (suhuMin2 + suhuNaik) && pemanasOn) {
      pemanasOn = false;
      digitalWrite(relay3Pin, LOW);
      tunggu5Mnt = true;
      tunggu5Mnt1 = currentMillis/1000;
      suhuNaik2 = false;
    }
    if (hum < targetHum && !tunggu5Mnt) {
      if (!pemanasOn) {
        pemanasOn = true;
        digitalWrite(relay3Pin, HIGH);
        suhuMin2 = suhuSekarang;
        suhuNaik2 = true;
      }
    }
    if (tunggu5Mnt) {
      if (((currentMillis/1000) - tunggu5Mnt1) >= tunggu5MntDur) {
        tunggu5Mnt = false;
      }
    }

    // Relay 2 On jika suhu mesin <= suhu target
    if (temp>targetTemp){
      digitalWrite(relay2Pin, LOW);
    } else {
      digitalWrite(relay2Pin, HIGH);
    }
    lcd.setCursor(0, 2); lcd.print(targetTemp);

    if (mtrAktif) {
      if (mtrNyala) {
        remainingTime = lamaNyalaRem - millis() / 1000;
        int hours = remainingTime / 3600;
        int minutes = (remainingTime % 3600) / 60;
        int seconds = (remainingTime % 3600) % 60;
        String jam = String(hours);
        if (hours < 10) {
          jam = "0" + jam;
        }
        String menit = String(minutes);
        if (minutes < 10) {
          menit = "0" + menit;
        }
        String detik = String(seconds);
        if (seconds < 10) {
          detik = "0" + detik;
        }
        lcd.print(" " + jam + ":" + menit + ":" + detik + " ");
        if (remainingTime==0){
          mtrNyala=false;
          targetTimeRem = targetTime + millis()/1000;
          // Matikan Relay 1: Motor Utama
          digitalWrite(relay1Pin, LOW);
        }
      } else {
        remainingTime = targetTimeRem - millis() / 1000;
        int hours = remainingTime / 3600;
        int minutes = (remainingTime % 3600) / 60;
        int seconds = (remainingTime % 3600) % 60;
        String jam = String(hours);
        if (hours < 10) {
          jam = "0" + jam;
        }
        String menit = String(minutes);
        if (minutes < 10) {
          menit = "0" + menit;
        }
        String detik = String(seconds);
        if (seconds < 10) {
          detik = "0" + detik;
        }
        lcd.print(" " + jam + ":" + menit + ":" + detik + " ");
        if (remainingTime==0){
          mtrNyala=true;
          lamaNyalaRem = lamaNyala + millis()/1000;
          // Nyalakan Relay 1: Motor Utama
          digitalWrite(relay1Pin, HIGH);
        }
      }
    } else {
      lcd.print(" MTR OFF  ");
    }
    lcd.print(targetHum);

    lcd.setCursor(0, 3);

    tampilkanMenu();   
    bacaTombol(); 
    //delay(25);
  }
}

void tampilkanMenu() {
  String buffer;
  
  switch (tomSw) {
    case 0: lcd.print("(STDBY)             "); break;
    case 1: lcd.print("(SUHU)              "); break;
    case 2: lcd.print("(%RH)               "); break;
    case 3: 
      lcd.print("MOTOR (ON/OFF)");
      lcd.print(mtrAktif ? "  <ON>" : " <OFF>");
      break;
    case 4:
      tampilkanJadwal("MOTOR (JADWAL)      ", targetTime);
      break;
    case 5:
      tampilkanJadwal("MOTOR (Durasi)      ", lamaNyala);
      break;
    case 6:
      tampilkanJadwal("%RH (Tunggu)        ", tunggu5MntDur);
      break;
    case 7:
      tampilkanStepSuhuAir();
      break;
    case 8:
      tampilkanPPM();
      break;
    case 9:
      lcd.print(fullOtomatis ? "FULL OTOMATIS:    ON" : "FULL OTOMATIS:   OFF");
      break;
    case 10:
      tampilkanSetVarFullOtomatis();
      break;
    case 11:
      tampilkanSettingTglJam();
      break;
    case 12:
      tampilkanSensorBawah();
      break;
    case 13:
      tampilkanSuhuAir();
      break;
    case 14:
      tampilkanStatFullOtomatis();
      break;
    case 15:
      tampilkanMulaiPenetasan();
      break;
    default:
      lcd.print("                    ");
  }
}

void tampilkanJadwal(String label, int waktu) {
  int jamJadwal = waktu / 3600;
  int mntJadwal = (waktu % 3600) / 60;
  int detJadwal = (waktu % 3600) % 60;

  String jamJdSt = (jamJadwal < 10) ? "0" + String(jamJadwal) : String(jamJadwal);
  String mntJdSt = (mntJadwal < 10) ? "0" + String(mntJadwal) : String(mntJadwal);
  String detSt = (detJadwal < 10) ? "0" + String(detJadwal) : String(detJadwal);

  // lcd.print(label);
  switch (setTimer) {
    case 0: lcd.print(label); break;
    case 1: lcd.print("JADWAL(Jam) " + jamJdSt + ":" + mntJdSt + ":" + detSt); break;
    case 2: lcd.print("JADWAL(Mnt) " + jamJdSt + ":" + mntJdSt + ":" + detSt); break;
    case 3: lcd.print("JADWAL(Det) " + jamJdSt + ":" + mntJdSt + ":" + detSt); break;
  }
}

void tampilkanStepSuhuAir() {
  String suhuNaikSt = String(suhuNaik);
  if (suhuNaik<10) suhuNaikSt = " "+suhuNaikSt;
  if (setTimer==0) lcd.print("STEP SUHU AIR (Set) ");
  if (setTimer==1) lcd.print("SUHU AIR (Naik):  " + String(suhuNaik));
}

void tampilkanPPM() {
  String buffer;
  //lcd.print("PPM: ");
  if (setTimer == 0) {
    if (ppm > 9999.99) {
      dtostrf(9999.99, 7, 2, buffer1at10kar);
    } else {
      dtostrf(ppm, 7, 2, buffer1at10kar);
    }

    if (ppm2 > 9999.99) {
      dtostrf(9999.99, 7, 2, buffer2at10kar);
    } else {
      dtostrf(ppm2, 7, 2, buffer2at10kar);
    }

    lcd.print("PPM: " + String(buffer1at10kar) + ";" +  String(buffer2at10kar));
  } else if (setTimer == 1) {
    dtostrf(deltaPPM, 7, 2, buffer1at10kar);
    lcd.print("Delta PPM:   " + String(buffer1at10kar));
  } else if (setTimer == 2) {
    dtostrf(deltaPPM, 7, 2, buffer1at10kar);
    lcd.print("Delta (Set): " + String(buffer1at10kar));
  }
}

void tampilkanStatFullOtomatis(){
  lcd.print(setTimer == 1 ? "PERUBAHAN   DISIMPAN" : "SAVE PERUBAHAN  DATA");
}
void tampilkanSetVarFullOtomatis() {
  String buffer;
  switch (setTimer) {
    case 0: lcd.print("SETVAR FULL OTOMATIS"); break;
    case 1: buffer = String(suhuH0H3, 2); lcd.print("Suhu Hr:00-03: "+buffer); break;
    case 2: buffer = String(suhuH4H17,2); lcd.print("Suhu Hr:04-17: "+buffer); break;
    case 3: buffer = String(suhuH18, 2);  lcd.print("Suhu Hari-18 : "+buffer); break;
    case 4: buffer = String(suhuH19, 2);  lcd.print("Suhu Hari-19 : "+buffer); break;
    case 5: buffer = String(suhuH20, 2);  lcd.print("Suhu Hari-20 : "+buffer); break;
    case 6: buffer = String(suhuH21, 2);  lcd.print("Suhu Hari-21 : "+buffer); break;
    case 7: buffer = String(rhH0H3, 2);   lcd.print("RH Hari:00-03: "+buffer); break;
    case 8: buffer = String(rhH4H17,2);   lcd.print("RH Hari:04-17: "+buffer); break;
    case 9: buffer = String(rhH18H21, 2); lcd.print("RH Hari:18-21: "+buffer); break;
    case 10: buffer = String(rhH22, 2);   lcd.print("RH Hari ke-22: "+buffer); break;
    case 11: lcd.print("MTR H0:0 H4:1 H18:0 "); break;
  }
}

void tampilkanSettingTglJam() {
  String buffer;
  switch (setTimer) {
    case 0: lcd.print("SETTING TGL & JAM   "); break;
    case 1: lcd.print("Ambil TglJam  UpDawn"); break;
    case 2: buffer = String(setThn1); if (setThn1 < 10) { buffer = "0" + buffer; } lcd.print("SET TGL (Tahun):" + buffer); break;
    case 3: buffer = String(setBln1); if (setBln1 < 10) { buffer = "0" + buffer; } lcd.print("SET TGL (Bulan):  " + buffer); break;
    case 4: buffer = String(setHr1);  if (setHr1 < 10) { buffer = "0" + buffer; }  lcd.print("SET TGL (Hari) :  " + buffer); break;
    case 5: buffer = String(setJam1); if (setJam1 < 10) { buffer = "0" + buffer; } lcd.print("SET TGL (Jam)  :  " + buffer); break;
    case 6: buffer = String(setMnt1); if (setMnt1 < 10) { buffer = "0" + buffer; } lcd.print("SET TGL (Menit):  " + buffer); break;
    case 7: buffer = String(setDet1); if (setDet1 < 10) { buffer = "0" + buffer; } lcd.print("SET TGL (Detik):  " + buffer); break;
    case 8: lcd.print("SAVE TGL&JAM  UpDawn"); break;
  }
}

void tampilkanSensorBawah() {
  dtostrf(temp2, 6, 2, buffer1at7kar);
  dtostrf(hum2, 6, 2, buffer2at7kar);
  lcd.print("BAWAH:" + String(buffer1at7kar) + "C" + String(buffer2at7kar) + "%");
}

void tampilkanSuhuAir() {
  dtostrf(suhuSekarang, 6, 2, buffer1at7kar);
  lcd.print("SUHU AIR:     " + String(buffer1at7kar));
}

void tampilkanMulaiPenetasan() {
  //lcd.print("                    ");
  switch (setTimer) {
    case 0: lcd.print("MULAI   (Hijau=Skrg)"); break;
    case 1: lcd.print("MULAI: " + String(hari0Day) + "-" + String(hari0Month) + "-" + String(hari0Year)+"  "); break;
    case 2: lcd.print("SIMPAN (TEkan HIJAU)"); break;
  }
}

void bacaTombol() {
  byte pitch = 5;
  
  for (uint8_t i = 0; i < jlTom; i++) {
    if (digitalRead(tomPin[i]) == LOW) {
      pitch = i;
    }  
  }

  if (pitch == 0) {
    if (tomSw == 1) targetTemp++;
    else if (tomSw == 2) targetHum++;
    else if (tomSw == 4) {
      if (setTimer == 1) targetTime += 10800;
      else if (setTimer == 2) targetTime += 600;
      else if (setTimer == 3) targetTime += 10;
    }
    else if (tomSw == 5) {
      if (setTimer == 1) lamaNyala += 10800;
      else if (setTimer == 2) lamaNyala += 600;
      else if (setTimer == 3) lamaNyala += 10;
    }
    else if (tomSw == 6) {
      if (setTimer == 1) tunggu5MntDur += 3600;
      else if (setTimer == 2) tunggu5MntDur += 600;
      else if (setTimer == 3) tunggu5MntDur += 10;
    }
    else if (tomSw == 8 && setTimer == 2) deltaPPM += 10;
    else if (tomSw == 10) {
      switch (setTimer) {
        case 1: suhuH0H3++; break;
        case 2: suhuH4H17++; break;
        case 3: suhuH18++; break;
        case 4: suhuH19++; break;
        case 5: suhuH20++; break;
        case 6: suhuH21++; break;
        case 7: rhH0H3++; break;
        case 8: rhH4H17++; break;
        case 9: rhH18H21++; break;
        case 10: rhH22++; break;
      }
    }
    else if (tomSw == 11) {
      switch (setTimer) {
        case 1: ambilDtnow = 1; break;
        case 2: setThn1 += 100; break;
        case 3: setBln1++; if (setBln1 > 12) setBln1 = 1; break;
        case 4: setHr1 += 10; if (setHr1 > 31) setHr1 = setHr1 - 31; break;
        case 5: setJam1 += 10; if (setJam1 > 24) setJam1 = setJam1 - 24; break;
        case 6: setMnt1 += 10; if (setMnt1 > 60) setMnt1 = setMnt1 - 60; break;
        case 7: setDet1 += 10; if (setDet1 > 60) setDet1 = setDet1 - 60; break;
        case 8: upDateTgljam = 1; break;
      }
    }
    else if (tomSw == 15) {
      if (setTimer==1){
        hari0Month++;
        if (hari0Month>12){
          hari0Month=1;
          hari0Year++;
        }
      }
    }    
  }
  else if (pitch == 1) {
    tomSw++;
    if (tomSw == 16) tomSw = 0;
    setTimer = 0;
  }
  else if (pitch == 2) {
    if (tomSw == 1) targetTemp--;
    else if (tomSw == 2) targetHum--;
    else if (tomSw == 4) {
      if (setTimer == 1) targetTime -= 10800;
      else if (setTimer == 2) targetTime -= 600;
      else if (setTimer == 3) targetTime -= 10;
    }
    else if (tomSw == 5) {
      if (setTimer == 1) lamaNyala -= 10800;
      else if (setTimer == 2) lamaNyala -= 600;
      else if (setTimer == 3) lamaNyala -= 10;
    }
    else if (tomSw == 6) {
      if (setTimer == 1) tunggu5MntDur -= 3600;
      else if (setTimer == 2) tunggu5MntDur -= 600;
      else if (setTimer == 3) tunggu5MntDur -= 10;
    }
    else if (tomSw == 8 && setTimer == 2) deltaPPM -= 10;
    else if (tomSw == 10) {
      switch (setTimer) {
        case 1: suhuH0H3--; break;
        case 2: suhuH4H17--; break;
        case 3: suhuH18--; break;
        case 4: suhuH19--; break;
        case 5: suhuH20--; break;
        case 6: suhuH21--; break;
        case 7: rhH0H3--; break;
        case 8: rhH4H17--; break;
        case 9: rhH18H21--; break;
        case 10: rhH22--; break;
      }
    }
    else if (tomSw == 11) {
      switch (setTimer) {
        case 1: ambilDtnow = 1; break;
        case 2: setThn1-=100; break;
        case 3: setBln1--; if (setBln1 < 1) setBln1 = 12; break;
        case 4: setHr1 -= 10; if (setHr1 < 1) setHr1 = 31 + setHr1; break;
        case 5: setJam1 -= 10; if (setJam1 < 0) setJam1 = 60 + setJam1; break;
        case 6: setMnt1 -= 10; if (setMnt1 < 0) setMnt1 = 60 + setMnt1; break;
        case 7: setDet1 -= 10; if (setDet1 < 1) setDet1 = 1 - setDet1; break;
        case 8: upDateTgljam = 1; break;
      }
    }
    else if (tomSw == 15) {
      if (setTimer==1){
        hari0Month--;
        if (hari0Month<1){
          hari0Month=12;
          hari0Year--;
        }
      }
    }        
  }
    else if (pitch == 3) {
    if (tomSw == 1) targetTemp -= 0.1;
    else if (tomSw == 2) targetHum -= 0.1;
    else if (tomSw == 4) {
      if (setTimer == 1) targetTime -= 3600;
      else if (setTimer == 2) targetTime -= 60;
      else if (setTimer == 3) targetTime--;
    }
    else if (tomSw == 5) {
      if (setTimer == 1) lamaNyala -= 3600;
      else if (setTimer == 2) lamaNyala -= 60;
      else if (setTimer == 3) lamaNyala--;
    }
    else if (tomSw == 6) {
      if (setTimer == 1) tunggu5MntDur -= 3600;
      else if (setTimer == 2) tunggu5MntDur -= 60;
      else if (setTimer == 3) tunggu5MntDur--;
    }
    else if (tomSw == 7 && setTimer == 1) suhuNaik--;
    else if (tomSw == 8 && setTimer == 2) deltaPPM--;
    else if (tomSw == 10) {
      switch (setTimer) {
        case 1: suhuH0H3 -= 0.1; break;
        case 2: suhuH4H17 -= 0.1; break;
        case 3: suhuH18 -= 0.1; break;
        case 4: suhuH19 -= 0.1; break;
        case 5: suhuH20 -= 0.1; break;
        case 6: suhuH21 -= 0.1; break;
        case 7: rhH0H3 -= 0.1; break;
        case 8: rhH4H17 -= 0.1; break;
        case 9: rhH18H21 -= 0.1; break;
        case 10: rhH22 -= 0.1; break;
      }
    }
    else if (tomSw == 11) {
      switch (setTimer) {
        case 1: ambilDtnow = 1; break;
        case 2: setThn1--; break;
        case 3: setBln1--; if (setBln1 < 1) setBln1 = 12; break;
        case 4: setHr1--; if (setHr1 < 1) setHr1 = 31; break;
        case 5: setJam1--; if (setJam1 < 1) setJam1 = 24; break;
        case 6: setMnt1--; if (setMnt1 < 0) setMnt1 = 60; break;
        case 7: setDet1--; if (setDet1 < 0) setDet1 = 60; break;
        case 8: upDateTgljam = 1; break;
      }
    }
    else if (tomSw == 15) {
      if (setTimer==0){
	      hari0Day  = now.day();  
        hari0Month = now.month();
        hari0Year  = now.year();         
      }
      else if (setTimer==1){
        hari0Day--;
        if (hari0Day<1){
          hari0Day=31;
          hari0Month++;
          if (hari0Month<1){
            hari0Month=12;
            hari0Year--;
          }
        }
      }
      else if (setTimer==2){
        simpanMulaiPenetasan=1;
      } 
    }
  }
  else if (pitch == 4) {
    if (tomSw == 1) targetTemp += 0.1;
    else if (tomSw == 2) targetHum += 0.1;
    else if (tomSw == 4) {
      if (setTimer == 1) targetTime += 3600;
      else if (setTimer == 2) targetTime += 60;
      else if (setTimer == 3) targetTime++;
    }
    else if (tomSw == 5) {
      if (setTimer == 1) lamaNyala += 3600;
      else if (setTimer == 2) lamaNyala += 60;
      else if (setTimer == 3) lamaNyala++;
    }
    else if (tomSw == 6) {
      if (setTimer == 1) tunggu5MntDur += 3600;
      else if (setTimer == 2) tunggu5MntDur += 60;
      else if (setTimer == 3) tunggu5MntDur++;
    }
    else if (tomSw == 7 && setTimer == 1) suhuNaik++;
    else if (tomSw == 8 && setTimer == 2) deltaPPM++;
    else if (tomSw == 10) {
      switch (setTimer) {
        case 1: suhuH0H3 += 0.1; break;
        case 2: suhuH4H17 += 0.1; break;
        case 3: suhuH18 += 0.1; break;
        case 4: suhuH19 += 0.1; break;
        case 5: suhuH20 += 0.1; break;
        case 6: suhuH21 += 0.1; break;
        case 7: rhH0H3 += 0.1; break;
        case 8: rhH4H17 += 0.1; break;
        case 9: rhH18H21 += 0.1; break;
        case 10: rhH22 += 0.1; break;
      }
    }
    else if (tomSw == 11) {
      switch (setTimer) {
        case 1: ambilDtnow = 1; break;
        case 2: setThn1++; break;
        case 3: setBln1++; if (setBln1 > 12) setBln1 = 1; break;
        case 4: setHr1++; if (setHr1 > 31) setHr1 = 1; break;
        case 5: setJam1++; if (setJam1 > 24) setJam1 = 1; break;
        case 6: setMnt1++; if (setMnt1 > 60) setMnt1 = 1; break;
        case 7: setDet1++; if (setDet1 > 60) setDet1 = 1; break;
        case 8: upDateTgljam = 1; break;
      }
    }
    else if (tomSw == 15) {
      if (setTimer==0){
	      hari0Day  = now.day();  
        hari0Month = now.month();
        hari0Year  = now.year();         
      }
      else if (setTimer==1){
        hari0Day++;
        if (hari0Day>31){
          hari0Day=1;
          hari0Month++;
          if (hari0Month>12){
            hari0Month=1;
            hari0Year++;
          }
        }
      }
      else if (setTimer==2){
        simpanMulaiPenetasan=1;
      } 
    }    
  }

  int tomSetPinVal = analogRead(tomSetPin);
  if (tomSetPinVal < threshold) {
    if (tomSw == 3) {
      mtrAktif = !mtrAktif;
      if (mtrAktif) targetTimeRem = 1 + millis() / 1000;
    }
    else if (tomSw >= 4 && tomSw < 9) {
      setTimer++;
      if (setTimer == 4) setTimer = 0;
    }
    else if (tomSw == 9) fullOtomatis = !fullOtomatis;
    else if (tomSw == 10) {
      setTimer++;
      if (setTimer == 12) setTimer = 0;
    }
    else if (tomSw == 11) {
      setTimer++;
      if (setTimer == 9) setTimer = 0;
    }
    else if (tomSw == 14 && setTimer == 0) {
      upDateEEPROM = 1;
      setTimer = 1;
    }
    else if (tomSw == 15) {
      setTimer++;
      if (setTimer == 3) setTimer = 0;
    }
  } 
//  Serial.print("tomSw: ");
//  Serial.print(tomSw);
//  Serial.print(" pitch: ");
//  Serial.print(pitch);
//  Serial.print(" setTimer: ");
//  Serial.print(setTimer);
//  Serial.print(" hariKe: ");
//  Serial.print(hariKe);
//  Serial.println();
}

float bacaSuhuAir(){
  int analogValue = analogRead(A0);
  float celsius = 1 / (log(1 / (1023. / analogValue - 1)) / BETA + 1.0 / 298.15) - 273.15;
  return celsius;
}

void saveToEEPROM() {
  EEPROM.put(0, 72);
  EEPROM.put(2, targetTemp);
  EEPROM.put(6, targetHum);
  EEPROM.put(10, targetTime);
  EEPROM.put(14, lamaNyala);
  EEPROM.put(18, tunggu5MntDur);
  EEPROM.put(22, suhuNaik);
  EEPROM.put(23, deltaPPM);
  EEPROM.put(27, suhuH0H3);
  EEPROM.put(31, suhuH4H17);
  EEPROM.put(35, suhuH18);
  EEPROM.put(39, suhuH19);
  EEPROM.put(43, suhuH20);
  EEPROM.put(47, suhuH21);
  EEPROM.put(51, rhH0H3);
  EEPROM.put(55, rhH4H17);
  EEPROM.put(59, rhH18H21);
  EEPROM.put(63, rhH22);
  EEPROM.put(67, mtrH0H3);
  EEPROM.put(68, mtrH4H17);
  EEPROM.put(69, mtrH18H22);
  EEPROM.put(70, hari0Day);
  EEPROM.put(71, hari0Month);
  EEPROM.put(72, hari0Year);
  EEPROM.put(74, fullOtomatis);
}

void getFromEEPROM() {
  EEPROM.get(2, targetTemp);
  EEPROM.get(6, targetHum);
  EEPROM.get(10, targetTime);
  EEPROM.get(14, lamaNyala);
  EEPROM.get(18, tunggu5MntDur);
  EEPROM.get(22, suhuNaik);
  EEPROM.get(23, deltaPPM);
  EEPROM.get(27, suhuH0H3);
  EEPROM.get(31, suhuH4H17);
  EEPROM.get(35, suhuH18);
  EEPROM.get(39, suhuH19);
  EEPROM.get(43, suhuH20);
  EEPROM.get(47, suhuH21);
  EEPROM.get(51, rhH0H3);
  EEPROM.get(55, rhH4H17);
  EEPROM.get(59, rhH18H21);
  EEPROM.get(63, rhH22);
  EEPROM.get(67, mtrH0H3);
  EEPROM.get(68, mtrH4H17);
  EEPROM.get(69, mtrH18H22);
  EEPROM.get(70, hari0Day );
  EEPROM.get(71, hari0Month );
  EEPROM.get(72, hari0Year );
  EEPROM.get(74, fullOtomatis);
}