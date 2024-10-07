#define SENSOR_PIR 7
#define LED 13

bool movimento_detectado = false;

void setup() {
  pinMode(SENSOR_PIR, INPUT);
  pinMode(LED, OUTPUT);

  // Inicializa a comunicação serial com o ESP8266 a 19200 bps
  Serial.begin(9600);  
  Serial.println("Arduino pronto para detectar movimento.");
}

void loop() {
  bool estado_sensor = digitalRead(SENSOR_PIR);

  if (estado_sensor && !movimento_detectado) {
    digitalWrite(LED, LOW);  // Acende o LED
    Serial.println("MOVIMENTO_DETECTADO");  // Envia para o ESP8266
    movimento_detectado = true;
  } 
  else if (!estado_sensor && movimento_detectado) {
    digitalWrite(LED, HIGH);  // Apaga o LED
    Serial.println("SEM_MOVIMENTO");  // Envia para o ESP8266
    movimento_detectado = false;
  }

  delay(100);  // Pequeno atraso para evitar sobrecarga
}
