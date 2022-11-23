
// Library yang digunakan
#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>
#include "WiFi.h"
#include "HCSR04.h"
#include "Servo.h"
#include "FirebaseESP32.h"


// Definisi firebase dan wifi yang digunakan
#define FIREBASE_HOST "https://strollerpintar3-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "wUcsBUZxSrRj0aGNEtEdGxELKT2RhZ46stk5SwAh"
#define WIFI_SSID "Stroller_Pintar"
#define WIFI_PASSWORD "87654321"

UltraSonicDistanceSensor distanceSensor(22, 21);  //initialisation class HCSR04 (trig pin , echo pin)

// mendeklarasikan objek data dari FirebaseESP8266
FirebaseData firebaseData;

String path = "/Hasil Pembacaan";
int oldDistance;
int newDistance;


//untuk mengonversi dari dBm ke jarak
int dbm;
float ratadbm;
float JPengguna;

//untuk mengendalikan servo
Servo myservo;
int oldPos;
int newPos;
String dataServo;

//untuk pembacaan DFPlayer
String dataDF;

//indikator pemutar musik
int LED1 = 27;  //PLAY LAMPU HIJAU MENYALA
int LED2 = 12;  //NEXT LAMPU KUNING MENYALA
int LED3 = 13;  //PREVIOUS LAMPU BIRU MENYALA
int LED4 = 14;  //PAUSE LAMPU MERAH MENYALA


DFRobotDFPlayerMini mp3;

void initWifi() {
  // Koneksi ke Wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  Firebase.reconnectWiFi(true);
  Firebase.setMaxRetry(firebaseData, 3);
}

void JarakPengguna() {
  for (int i = 0; i <= 4; i++) {
    Firebase.setInt(firebaseData, path + "/RSSI", WiFi.RSSI());
    dbm = dbm + WiFi.RSSI();
    delay(100);
  }
  ratadbm = dbm / 5;       //rata-rata dbm dari 5 data yang didapat
  if (ratadbm >= -62.5) {  //data yang nilai dbmnya kurang dari -62.5 akan di jadikan -62.5
    ratadbm = -62.5;
  }
  Serial.println(ratadbm);
  dbm = 0;

  JPengguna = ((4.5 * ratadbm) + 296.42) / 19.16;  //persamaan linier antara jarak dengan dbm
  Serial.println(JPengguna);
  Firebase.setFloat(firebaseData, path + "/JarakPengguna", JPengguna);


  if (JPengguna <= 1) {
    Serial.println("< 1 Meter");
  } else if (JPengguna <= 2) {
    Serial.println("< 2 Meter");
  } else if (JPengguna <= 3) {
    Serial.println("< 3 Meter");
  } else if (JPengguna > 3) {
    Serial.println("Anda Terlalu Jauh dari Stroller");
  }
}

void setup() {

  Serial.begin(115200);
  initWifi();
  oldDistance = distanceSensor.measureDistanceCm();


  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  myservo.attach(2);

  Serial2.begin(9600);
  delay(100);
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  if (!mp3.begin(Serial2)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true)
      ;
  }
  Serial.println(F("DFPlayer Mini online."));
  mp3.setTimeOut(500);  //Set serial communictaion time out 500ms
  mp3.volume(30);       //Set volume value (0~30).
  //  delay(1000);
  //  mp3.play(98);  //Play the first mp3
  delay(1000);
}

void loop() {
  delay(500);

  //ambil nilai dari sensor ultrasonic dan kirim ke firebase
  newDistance = distanceSensor.measureDistanceCm();
  
  if (newDistance != oldDistance) {
    Firebase.setInt(firebaseData, path + "/Jarak", newDistance);
    oldDistance = newDistance;
  }
  Serial.println(newDistance);
  Firebase.setInt(firebaseData, path + "/RSSI", WiFi.RSSI());

  //baca data dari firebase untuk penutup stroller
  if (Firebase.getString(firebaseData, "/PenutupStroller/Derajat")) {
    if (firebaseData.dataTypeEnum() == fb_esp_rtdb_data_type_string) {
      String dataServo = firebaseData.to<String>();
      //Serial.print(dataServo);
      if (dataServo == "80") {  //jika penutup stroller nyala maka servo akan bergerak ke posisi 80 derajat
        myservo.write(80);
        delay(20);
      }

      else if (dataServo == "45") {  //jika penutup stroller nyala maka servo akan bergerak ke posisi 45 derajat
        myservo.write(45);
        delay(20);
      }

      else if (dataServo == "10") {  //jika penutup stroller mati maka servo akan bergerak ke posisi 10 derajat
        myservo.write(0);
        delay(20);
      }
    }
  }

  if (Firebase.getString(firebaseData, "/Musik/Status")) {
    if (firebaseData.dataTypeEnum() == fb_esp_rtdb_data_type_string) {
      String dataDF = firebaseData.to<String>();

      if (dataDF == "1") {
        mp3.play();
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        digitalWrite(LED4, LOW);
        delay(5000);
      }

      else if (dataDF == "2") {
        mp3.next();
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, LOW);
        digitalWrite(LED4, LOW);
        delay(1000);
      }

      else if (dataDF == "3") {
        mp3.previous();
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, HIGH);
        digitalWrite(LED4, LOW);
        delay(1000);
      }

      else if (dataDF == "4") {
        mp3.pause();
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        digitalWrite(LED4, HIGH);
        delay(1000);
      }
    }
  }
}
