#include <MQ135.h>//khai báo thư viện
#include <DHT.h>
#define PIN_MQ135 A1 // khai báo các chân của MQ135 và DHT11
#define DHTPIN A0 // 
#define DHTTYPE DHT11 // 

MQ135 mq135_sensor(PIN_MQ135);
DHT dht(DHTPIN, DHTTYPE);

float temperature, humidity; 

void setup() {
  Serial.begin(9600);//bắt đầu cổng kết nối

  dht.begin();
}

void loop() {
  humidity = dht.readHumidity();// đọc giá trị độ ẩm từ DHT11
  temperature = dht.readTemperature();//đọc giá trị nhiệt độ của DHT11
  if (isnan(humidity) || isnan(temperature)) {   //kiểm tra lỗi của chương trình
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  //lấy dữ liệu từ cảm biến mq135
  float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity); //cảm biến MQ135 sẽ lấy dữ liệu từ DHT11 để tăng độ chính xác
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);

  Serial.print("MQ135 RZero: ");//in ra giá trị của MQ135
  Serial.print(rzero);
  Serial.print("\t Corrected RZero: ");
  Serial.print(correctedRZero);
  Serial.print("\t Resistance: ");// giá trị điện trở
  Serial.print(resistance);
  Serial.print("\t PPM: ");
  Serial.print(ppm);
  Serial.print("ppm");
  Serial.print("\t Corrected PPM: ");
  Serial.print(correctedPPM);
  Serial.println("ppm");

  delay(300);//cập nhật giá trị sau mỗi 300 miligiây
}
