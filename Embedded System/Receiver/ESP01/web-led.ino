#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>

#define WIFI_SSID "messi123"
#define WIFI_PASS "messili9nel"
#define SERVER_URL "https://sistema-anti-furto.vercel.app/api/equipament/alert-out-of-range"

void sendUID(String uid);

WiFiClientSecure client;

void setup() {
  Serial.begin(9600);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi conectado com sucesso!");

  // Ignorar verificação de certificado (opcional, mas necessário na maioria dos casos com ESP8266)
  client.setInsecure();  // ⚠️ Usar apenas em testes. Para produção, use certificado confiável!
}

void sendUID(String uid) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient https;
    https.begin(client, SERVER_URL);
    https.addHeader("Content-Type", "application/json");

    String json = "{\"rfid\":\"" + uid + "\"}";

    int httpResponseCode = https.POST(json);

    Serial.print("Código de resposta HTTP: ");
    Serial.println(httpResponseCode);

    if (httpResponseCode > 0) {
      String payload = https.getString();
      Serial.println("Resposta do servidor:");
      Serial.println(payload);
    } else {
      Serial.println("Erro ao enviar UID.");
    }

    https.end();
  } else {
    Serial.println("Wi-Fi desconectado.");
  }
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data.startsWith("UID:")) {
      String uid = data.substring(4);
      sendUID(uid);
      delay(2000);
    }
  }
}
