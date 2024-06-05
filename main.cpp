#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "defines.h"
#include "velocidade.h"

void testaLinha();

void leituraSensoresDistancia();

void leituraSensoresLinha();

std::string comando = "p";

#define bleRobo "WALLY"

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool bleBool = false;

BLEServer *pServer = NULL;

BLECharacteristic bleComCharacteristics("9295411b-0868-4655-9b84-ec3a4c74e258", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bleComDescriptor(BLEUUID((uint16_t)0x2902));

BLECharacteristic bleLinhaCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bleLinhaDescriptor(BLEUUID((uint16_t)0x2902));

BLECharacteristic bleDistanciaCharacteristics("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bleDistanciaDescriptor(BLEUUID((uint16_t)0x2902));

// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("CONECTADO");
  };
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    pServer->startAdvertising();
    Serial.println("DESCONECTADO");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    comando = value;
    Serial.println(value.c_str());

    if (value.substr(0, 3) == "vel")
    {
      velmotor = std::stoi(value.substr(3, 7));
    }

    if (value.substr(0, 3) == "par")
    {
      parametro = std::stoi(value.substr(3, 7));
    }
  }
};

void envia_sensores_distancia()
{

  leituraSensoresDistancia();

  std::string sensores = "esq:";
  sensores.append(std::to_string(distancia_lateral_esquerdo));
  sensores.append(" front_esq:");
  sensores.append(std::to_string(distancia_frontal_esquerdo));
  sensores.append(" dir:");
  sensores.append(std::to_string(distancia_lateral_direita));
  sensores.append(" front_dir:");
  sensores.append(std::to_string(distancia_frontal_direita));
  bleDistanciaCharacteristics.setValue(sensores);
  bleDistanciaCharacteristics.notify();
  delay(100);
}

void envia_sensores_linha()
{

  leituraSensoresLinha();

  std::string sensores = "esq:";
  sensores.append(std::to_string(linha_esquerda));
  sensores.append(" dir:");
  sensores.append(std::to_string(linha_direita));
  bleLinhaCharacteristics.setValue(sensores);
  bleLinhaCharacteristics.notify();
  delay(100);
}

/**************************************************************************/

void setup()
{

  pinMode(SENSOR_LAT_ESQ, INPUT_PULLDOWN);   // sensor de distancia A
  pinMode(SENSOR_FRONT_ESQ, INPUT_PULLDOWN); // sensor de distancia B
  pinMode(SENSOR_FRONT_DIR, INPUT_PULLDOWN); // sensor de distancia C
  pinMode(SENSOR_LAT_DIR, INPUT_PULLDOWN);   // sensor de distancia D
  pinMode(SENSOR_LINHA_A, INPUT);            // sensor de linha A
  pinMode(SENSOR_LINHA_B, INPUT);            // sensor de linha B
  pinMode(PWMA, OUTPUT);                     // PWMA
  pinMode(PWMB, OUTPUT);                     // PWMB
  pinMode(MODULO_START, INPUT_PULLDOWN);     // modulo start
  pinMode(IN1_A, OUTPUT);                    // entrada motor A
  pinMode(IN2_A, OUTPUT);                    // entrada motor A
  pinMode(IN3_B, OUTPUT);                    // entrada motor B
  pinMode(IN4_B, OUTPUT);                    // entrada motor B
  pinMode(LEDPIN, OUTPUT);                   // led
  ledcSetup(0, 5000, 10);
  ledcAttachPin(PWMA, 0);
  ledcSetup(1, 5000, 10);
  ledcAttachPin(PWMB, 1);

  Serial.begin(9600);

  BLEDevice::init(bleRobo);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  pService->addCharacteristic(&bleDistanciaCharacteristics);
  bleDistanciaDescriptor.setValue("bleDist");
  bleDistanciaCharacteristics.addDescriptor(&bleDistanciaDescriptor);

  pService->addCharacteristic(&bleLinhaCharacteristics);
  bleLinhaDescriptor.setValue("Linha");
  bleLinhaCharacteristics.addDescriptor(&bleLinhaDescriptor);

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("ENVIAR CMD");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  pAdvertising->start();
}
// Atualizar leitura dos sensores de linha e distancia

void leituraSensoresLinha()
{
  linha_direita = analogRead(SENSOR_LINHA_A);
  linha_esquerda = analogRead(SENSOR_LINHA_B);
}
void leituraSensoresDistancia()
{
  distancia_lateral_esquerdo = digitalRead(SENSOR_LAT_ESQ);
  distancia_frontal_esquerdo = digitalRead(SENSOR_FRONT_ESQ);
  distancia_frontal_direita = digitalRead(SENSOR_FRONT_DIR);
  distancia_lateral_direita = digitalRead(SENSOR_LAT_DIR);
}

#include "pwm.h"

void ataque()
{
  if (digitalRead(SENSOR_FRONT_ESQ) == 1 && digitalRead(SENSOR_FRONT_DIR) == 1 || digitalRead(SENSOR_FRONT_ESQ) == 1 && digitalRead(SENSOR_FRONT_DIR) == 0 || digitalRead(SENSOR_FRONT_ESQ) == 0 && digitalRead(SENSOR_FRONT_DIR) == 1)
  {
    mover_motor('e', 'f', 400);
    mover_motor('d', 'f', 400);
  }
}

void piscar_led()
{
  digitalWrite(LEDPIN, HIGH);
  delay(500);
  digitalWrite(LEDPIN, LOW);
  delay(500);
}

void busca()
{
  Serial.println("Entrou busca");
  leituraSensoresLinha();
  testaLinha();
  leituraSensoresDistancia();

  if ((SENSOR_LAT_ESQ == 0) && (SENSOR_FRONT_ESQ == 0) && (SENSOR_FRONT_DIR == 1) && (SENSOR_LAT_DIR == 1))
  {
    mover_motor('d', 'f', 250);
    mover_motor('e', 't', 200);
    delay(100);
  }
  if ((SENSOR_LAT_ESQ == 0) && (SENSOR_FRONT_ESQ == 0) && (SENSOR_FRONT_DIR == 0) && (SENSOR_LAT_DIR == 1))
  {
    mover_motor('d', 'f', 250);
    mover_motor('e', 't', 200);
    delay(100);
  }
  else if ((SENSOR_LAT_ESQ == 1) && (SENSOR_FRONT_ESQ == 1) && (SENSOR_FRONT_DIR == 0) && (SENSOR_LAT_DIR == 0))
  {
    mover_motor('d', 'f', 250);
    mover_motor('e', 't', 200);
    delay(100);
  }
  else if ((SENSOR_LAT_ESQ == 1) && (SENSOR_FRONT_ESQ == 0) && (SENSOR_FRONT_DIR == 0) && (SENSOR_LAT_DIR == 0))
  {
    mover_motor('d', 'f', 250);
    mover_motor('e', 't', 200);
    delay(100);
  }
  else if ((SENSOR_LAT_ESQ == 0) && (SENSOR_FRONT_ESQ == 0) && (SENSOR_FRONT_DIR == 0) && (SENSOR_LAT_DIR == 0))
  {
    girar_eixo();
  }
}

void missil()
{
  mover_motor('e', 'f', 1023);
  mover_motor('d', 'f', 1023);
}

void evitaLinha()
{
  {
    if ((analogRead(SENSOR_LINHA_A) < 900) && (analogRead(SENSOR_LINHA_B) > 900))
    {
      mover_motor('e', 't', 90);
      mover_motor('d', 't', 100);
      delay(50);
    }
    else if ((analogRead(SENSOR_LINHA_A) > 900) && (analogRead(SENSOR_LINHA_B) < 900))
    {
      mover_motor('e', 't', 90);
      mover_motor('d', 't', 100);
      delay(50);
    }

    else if ((analogRead(SENSOR_LINHA_A) < 900) && (analogRead(SENSOR_LINHA_B) < 900))
    {
      mover_motor('e', 't', 100);
      mover_motor('d', 't', 100);
      delay(50);
    }
  }
}

void testaLinha()
{
  // 0 branco 1 preto
  // aumentar delay da linha branca
  if ((SENSOR_LINHA_A > 1100) && (SENSOR_LINHA_B < 1100))
  {
    mover_motor('e', 't', velmotor * 0.7);
    mover_motor('d', 't', velmotor * 0.7);
    delay(3000);
    mover_motor('e', 'f', velmotor * 0.5);
    mover_motor('d', 't', velmotor * 0.5);
    delay(3000);
  }
  else if ((SENSOR_LINHA_A < 1100) && (SENSOR_LINHA_B > 1100))
  {
    mover_motor('d', 't', velmotor * 0.7);
    mover_motor('e', 't', velmotor * 0.7);
    delay(3000);
    mover_motor('d', 'f', velmotor * 0.5);
    mover_motor('e', 't', velmotor * 0.5);
    delay(3000);
  }
  else if ((SENSOR_LINHA_A < 1100) && (SENSOR_LINHA_B < 1100))
  {
    mover_motor('e', 'f', velmotor * 0.5);
    mover_motor('d', 'f', velmotor * 0.5);
  }
  else if ((SENSOR_LINHA_A > 1100) && (SENSOR_LINHA_B > 1100))
  {
    mover_motor('e', 't', velmotor * 0.8);
    mover_motor('d', 't', velmotor * 0.8);
    delay(3000);
    mover_motor('e', 't', velmotor * 0.7);
    mover_motor('d', 'f', velmotor * 0.8);
    delay(3000);
  }
}

/*********************************************************************************************/

void loop()
{

  Serial.println(digitalRead(MODULO_START));

  ledcWrite(0, 400);
  ledcWrite(1, 400);

  if (deviceConnected)
  {
    envia_sensores_distancia();
    envia_sensores_linha();
    delay(5);
  }

  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }

  while (digitalRead(MODULO_START) == 0)
  {
    parar();
    Serial.println(digitalRead(MODULO_START));
  }

  if (comando == "led")
  {
    piscar_led();
  }
  if (comando == "p")
  {
    parar();
  }
  if (comando == "f")
  {
    frente();
  }

  if (comando == "t")
  {
    tras();
  }

  if (comando == "eixo")
  {
    girar_eixo();
  }

  if (comando == "circulo")
  {
    girar_circulo();
  }

  if (comando == "180")
  {
    girar_180();
  }

  if (comando == "ataque")
  {
    ataque();
  }

  if (comando == "pwm")
  {
    if (digitalRead(MODULO_START))
    {
      mover_motor('e', 'f', 100);
      mover_motor('d', 'f', 100);
    }
    else
    {
      parar();
    }
  }

  if (comando == "vel")
  {
    if (digitalRead(MODULO_START))
    {
      int velocidadePWM = 100;
      for (int i = 0; i < 14; i++)
      {
        velocidadePWM = velocidadePWM + 50;
        mover_motor('d', 'f', velocidadePWM);
        mover_motor('e', 'f', velocidadePWM);
        Serial.println(velocidadePWM);
        delay(2000);
      }
    }
    else
    {
      parar();
    }
  }

  // ESTRATÉGIAS

  // STRAT 1
  if (comando == "m")
  {
    missil();
  }

  // STRAT 2
  if (comando == "e")
  {
    if (analogRead(SENSOR_LINHA_A) > 2500 && analogRead(SENSOR_LINHA_B < 2500))
    {
      tras();
      delay(200);
      girar_180();
      delay(300);
    }
    if (analogRead(SENSOR_LINHA_A) < 2500 && analogRead(SENSOR_LINHA_B > 2500))
    {
      tras();
      delay(200);
      girar_180();
      delay(300);
    }
    if (analogRead(SENSOR_LINHA_A) < 2500 && analogRead(SENSOR_LINHA_B < 2500))
    {
      tras();
      delay(200);
      girar_180();
      delay(300);
    }
    else
    {
      frente();
    }
  }
  // STRAT 3
  if (comando == "x")
  {
    if (digitalRead(SENSOR_FRONT_ESQ) == 1 && digitalRead(SENSOR_FRONT_DIR == 1))
    {
      ataque();
    }
    if (digitalRead(SENSOR_FRONT_ESQ) == 1 && digitalRead(SENSOR_FRONT_DIR == 0))
    {
      ataque();
    }
    if (digitalRead(SENSOR_FRONT_ESQ) == 0 && digitalRead(SENSOR_FRONT_DIR == 1))
    {
      ataque();
    }
    if (digitalRead(SENSOR_FRONT_ESQ) == 0 && digitalRead(SENSOR_FRONT_DIR == 0))
    {
      girar_eixo();
      delay(3000);
      parar();
    }
  }
}
