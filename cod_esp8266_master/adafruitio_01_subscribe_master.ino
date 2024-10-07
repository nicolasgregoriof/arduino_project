#include "config.h"  // Inclui o arquivo config.h com as credenciais da Adafruit IO
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

// Defina as credenciais do WiFi
#define WIFI_SSID "nick_office"
#define WIFI_PASS "00870086"

// Criação da instância para a comunicação com a Adafruit IO
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, "io.adafruit.com", 1883, IO_USERNAME, IO_KEY);

// Criação do feed na Adafruit IO para enviar os códigos
Adafruit_MQTT_Publish codigoFeed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/teste");  // Mudado para o feed "teste"

String inputString = ""; // String recebida do Arduino

// Variável para controle do LED
unsigned long previousMillis = 0;
const long interval = 2000;  // Intervalo de 2 segundos para o piscar do LED

void setup() {
  // Inicializa a comunicação serial com o Arduino através das portas RX e TX padrão
  Serial.begin(9600);  
  Serial.println("ESP8266 configurado como Master com Adafruit IO.");

  // Configura o LED como OUTPUT
  pinMode(LED_BUILTIN, OUTPUT);

  // Conectando-se ao WiFi (LED ligado durante a conexão)
  connectToWiFi();

  // Conectando-se ao Adafruit IO
  connectToAdafruitIO();
}

void loop() {
  // Mantém a conexão com o Adafruit IO
  MQTT_connect();

  // Pisca o LED a cada 2 segundos se estiver conectado ao WiFi
  blinkLED();

  if (Serial.available()) {
    char c = Serial.read();  // Lê o caractere recebido
    inputString += c;

    // Verifica se a mensagem completa foi recebida
    if (c == '\n') {
      inputString.trim();  // Remove espaços ou quebras de linha extras

      if (inputString == "MOVIMENTO_DETECTADO") {
        enviarCodigo(3);  // Envia o código 3 para a Adafruit IO
      } else if (inputString == "SEM_MOVIMENTO") {
        enviarCodigo(1);  // Envia o código 1 para a Adafruit IO
      }

      inputString = "";  // Limpa a string para a próxima mensagem
    }
  }
}

void enviarCodigo(int codigo) {
  // Publica o código no feed "teste" da Adafruit IO
  if (!codigoFeed.publish(codigo)) {
    Serial.println("Falha ao enviar o código para Adafruit IO");
  } else {
    Serial.print("Código enviado: ");
    Serial.println(codigo);
  }
}

void connectToWiFi() {
  Serial.print("Conectando-se ao WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Liga o LED enquanto o WiFi está conectando
  digitalWrite(LED_BUILTIN, LOW);  // LED aceso (LED_BUILTIN é ativo em LOW)

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Conectado!");

  // Desliga o LED após conectar ao WiFi
  digitalWrite(LED_BUILTIN, HIGH);  // LED apagado (LED_BUILTIN é ativo em LOW)
}

void connectToAdafruitIO() {
  Serial.print("Conectando-se ao Adafruit IO...");
  while (mqtt.connect() != 0) { // Aguarda conexão
    Serial.println(mqtt.connectErrorString(mqtt.connect()));
    delay(5000);
  }
  Serial.println(" Conectado ao Adafruit IO!");
}

void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Conectando-se ao MQTT... ");
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println(" Tentando reconectar em 5 segundos...");
    delay(5000);
  }
  Serial.println("Conectado ao MQTT!");
}

// Função para piscar o LED a cada 2 segundos
void blinkLED() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Alterna o estado do LED
    if (digitalRead(LED_BUILTIN) == HIGH) {
      digitalWrite(LED_BUILTIN, LOW);  // Liga o LED (LED_BUILTIN é ativo em LOW)
    } else {
      digitalWrite(LED_BUILTIN, HIGH);  // Desliga o LED
    }
  }
}
