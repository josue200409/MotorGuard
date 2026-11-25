/*
 * MotorGuard - Detector de desbalance y fallas en motores (TinyML / mantenimiento predictivo)
 * Autor : Josue Alexander Mazariegos Lopez
 * Curso : Proyecto Integrador Final IoT & Web 4.0 - Modulo 4
 *
 * Flujo:  MPU6050 (100 Hz) -> ventana 2 s -> caracteristicas espectrales -> modelo TinyML
 *         -> maquina de estados (NORMAL / ADVERTENCIA / PARO) -> rele + LED -> MQTT (HiveMQ)
 * La señal cruda de vibracion NUNCA sale del ESP32: solo se publica el diagnostico.
 *
 * Dos motores de inferencia (elige con USE_EDGE_IMPULSE):
 *   0 = modelo local de respaldo (centroides + K-means, modelo_local.h). Funciona en Wokwi web.
 *   1 = modelo exportado desde Edge Impulse (libreria Arduino). Ver README.
 */
#define USE_EDGE_IMPULSE    0      // 0 = modelo local | 1 = Edge Impulse
#define SIMULAR_VIBRACION   1      // 1 = Wokwi (suma firma sintetica) | 0 = motor real

#if USE_EDGE_IMPULSE
  #include <motorguard_inferencing.h>   // <-- cambia por el nombre de TU libreria: <NOMBRE_inferencing.h>
#endif
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include "motorguard_dsp.h"
#include "motorguard_sim.h"

// ============================== Configuracion ==============================
#define WIFI_SSID   "Wokwi-GUEST"
#define WIFI_PASS   ""
#define MQTT_HOST   "broker.hivemq.com"
#define MQTT_PORT   1883
#define ID_ALUMNO   "josue_mazariegos"                       // topic unico para no cruzarse con otros
#define TOPIC_BASE  "curso_iot_2026/" ID_ALUMNO "/motorguard"
#define TOPIC_TELEMETRIA TOPIC_BASE "/telemetria"            // JSON de diagnostico cada 2 s
#define TOPIC_ALERTA     TOPIC_BASE "/alerta"                // eventos de cambio de estado
#define TOPIC_ESTADO     TOPIC_BASE "/estado"                // online/offline (LWT retenido)
#define TOPIC_COMANDO    TOPIC_BASE "/comando"               // rearmar | paro | sim:<modo>

// Pines (ESP32 DevKit C v4)
const uint8_t PIN_SDA = 21, PIN_SCL = 22;     // I2C -> MPU6050
const uint8_t PIN_POT = 34;                   // potenciometro = temperatura de carcasa
const uint8_t PIN_RELE = 26;                  // modulo rele (alimenta el motor)
const uint8_t PIN_LED = 27;                   // LED rojo de advertencia
const uint8_t PIN_BTN_SIM = 4;                // boton: cambia la falla simulada
const uint8_t PIN_BTN_RST = 5;                // boton: rearme tras un paro
#define RELE_NIVEL_ACTIVO HIGH                // HIGH = bobina energizada = motor alimentado

// Umbrales de decision
const float    TEMP_ADV  = 80.0f;             // C -> advertencia
const float    TEMP_PARO = 95.0f;             // C -> paro de emergencia
const float    CONF_MIN  = 0.80f;             // confianza minima para aceptar una clase de falla
#if USE_EDGE_IMPULSE
const float    UMBRAL_ANOM = 0.30f;           // ajusta con Model testing de Edge Impulse
#else
const float    UMBRAL_ANOM = 1.00f;           // puntaje ya normalizado por MG_ANOM_THR
#endif
const int      VENTANAS_PARO = 3;             // ventanas fallidas consecutivas (~3 s) para disparar el paro
const uint32_t HOP_MUESTRAS = 100;            // un diagnostico por segundo (traslape 50 %)
const uint32_t PERIODO_TELEMETRIA_MS = 2000;

// ============================== Estado global ==============================
enum Estado { E_NORMAL, E_ADVERTENCIA, E_PARO };
static const char* const ESTADO_TXT[] = { "NORMAL", "ADVERTENCIA", "PARO" };

struct Diag { char etiqueta[20]; float conf, anom, rms; uint32_t dspUs; };
struct Boton { uint8_t pin; bool ultimo; uint32_t t; };   // definido arriba: Arduino genera prototipos antes de la 1a funcion

static Estado   estado = E_NORMAL;
static Diag     diag = { "normal", 1.0f, 0.0f, 0.0f, 0 };
static char     causa[24] = "";
static bool     releOn = false;
static int      fallaRacha = 0;
static float    tempC = 25.0f;
static float    fsMedida = 0;
static uint32_t i2cErrPrev = 0;
static uint32_t gHoldoff = MG_WIN;            // no inferir hasta tener una ventana 100 % nueva
static uint32_t gUltInfer = 0;

static volatile int modoSim = MODO_NORMAL;

// Ring buffer de muestreo (escribe tareaMuestreo, lee loop)
static float    gRing[MG_WIN][MG_AXES];
static float    gWin[MG_WIN][MG_AXES];
static volatile uint32_t gCount = 0;          // muestras totales adquiridas
static volatile uint32_t gJitterMaxUs = 0;    // peor desviacion del periodo de 10 ms
static volatile uint32_t gI2cErr = 0;
static volatile float    gGyro = 0;
static portMUX_TYPE      gMux = portMUX_INITIALIZER_UNLOCKED;

static MGSim  gSim;
static MGDsp  gDsp;
static WiFiClient   net;
static PubSubClient mqtt(net);

#if USE_EDGE_IMPULSE
static float eiBuf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static_assert(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE == MG_WIN * MG_AXES,
              "Edge Impulse: usa ventana de 2000 ms, 100 Hz y 3 ejes (accX, accY, accZ)");
static int eiGetData(size_t offset, size_t length, float* out) {
  memcpy(out, eiBuf + offset, length * sizeof(float));
  return 0;
}
#endif

// ============================== MPU6050 (I2C, registros directos) ==============================
static const uint8_t MPU_ADDR = 0x68;

static bool mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00);          // salir de sleep
  if (Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); // filtro 44 Hz (anti-aliasing para fs = 100 Hz)
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); // rango +-2 g
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(); // giroscopio +-250 dps
  return true;
}

static bool mpuLeer(float a[3], float* gyroDps) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read(); uint8_t lo = Wire.read();
    r[i] = (int16_t)((hi << 8) | lo);
  }
  a[0] = r[0] / 16384.0f; a[1] = r[1] / 16384.0f; a[2] = r[2] / 16384.0f;     // g
  float gx = r[4] / 131.0f, gy = r[5] / 131.0f, gz = r[6] / 131.0f;           // dps
  *gyroDps = sqrtf(gx * gx + gy * gy + gz * gz);
  return true;
}

// Tarea de muestreo: 100 Hz constantes con vTaskDelayUntil (core 1, prioridad alta)
static void tareaMuestreo(void*) {
  TickType_t t0 = xTaskGetTickCount();
  uint32_t prev = 0;
  for (;;) {
    vTaskDelayUntil(&t0, pdMS_TO_TICKS(1000 / MG_FS_HZ));
    uint32_t ahora = micros();
    if (prev) {                                              // medir jitter del periodo
      int32_t dev = (int32_t)(ahora - prev) - (int32_t)(1000000 / MG_FS_HZ);
      if (dev < 0) dev = -dev;
      if ((uint32_t)dev > gJitterMaxUs) gJitterMaxUs = (uint32_t)dev;
    }
    prev = ahora;

    float a[3], g;
    if (!mpuLeer(a, &g)) { gI2cErr++; continue; }
    gGyro = g;
#if SIMULAR_VIBRACION
    float v[3]; gSim.step(modoSim, v);
    a[0] += v[0]; a[1] += v[1]; a[2] += v[2];
#endif
    portENTER_CRITICAL(&gMux);
    uint32_t i = gCount % MG_WIN;
    gRing[i][0] = a[0]; gRing[i][1] = a[1]; gRing[i][2] = a[2];
    gCount++;
    portEXIT_CRITICAL(&gMux);
  }
}

static void copiarVentana() {                                // ventana ordenada: mas antigua -> mas reciente
  portENTER_CRITICAL(&gMux);
  uint32_t ini = gCount % MG_WIN;
  memcpy(gWin[0], gRing[ini], (MG_WIN - ini) * sizeof(gRing[0]));
  if (ini) memcpy(gWin[MG_WIN - ini], gRing[0], ini * sizeof(gRing[0]));
  portEXIT_CRITICAL(&gMux);
}

// ============================== Inferencia TinyML ==============================
static bool inferir(Diag& d) {
  copiarVentana();
  uint32_t t0 = micros();
#if USE_EDGE_IMPULSE
  float mean[MG_AXES] = {0, 0, 0}; double acc = 0;
  for (int n = 0; n < MG_WIN; n++) for (int ax = 0; ax < MG_AXES; ax++) mean[ax] += gWin[n][ax];
  for (int ax = 0; ax < MG_AXES; ax++) mean[ax] /= MG_WIN;
  for (int n = 0; n < MG_WIN; n++) for (int ax = 0; ax < MG_AXES; ax++) {
    float v = gWin[n][ax] - mean[ax];                        // sin gravedad (igual que el dataset)
    eiBuf[n * MG_AXES + ax] = v; acc += (double)v * v;
  }
  signal_t signal; signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; signal.get_data = &eiGetData;
  ei_impulse_result_t res = {0};
  if (run_classifier(&signal, &res, false) != EI_IMPULSE_OK) return false;
  int best = 0;
  for (size_t i = 1; i < EI_CLASSIFIER_LABEL_COUNT; i++)
    if (res.classification[i].value > res.classification[best].value) best = (int)i;
  strncpy(d.etiqueta, res.classification[best].label, sizeof(d.etiqueta) - 1);
  d.etiqueta[sizeof(d.etiqueta) - 1] = 0;
  d.conf = res.classification[best].value;
  #if EI_CLASSIFIER_HAS_ANOMALY
  d.anom = res.anomaly;
  #else
  d.anom = 0;
  #endif
  d.rms = (float)sqrt(acc / MG_WIN);
#else
  float f[MG_NFEAT], rms;
  gDsp.extract(gWin, f, &rms);
  MGResult r = gDsp.classify(f, rms);
  strncpy(d.etiqueta, MG_CLASS_NAMES[r.clase], sizeof(d.etiqueta) - 1);
  d.etiqueta[sizeof(d.etiqueta) - 1] = 0;
  d.conf = r.conf; d.anom = r.anomalia; d.rms = r.rms;
#endif
  d.dspUs = micros() - t0;
  return true;
}

static bool esFalla(const Diag& d) {
  bool claseFalla = strcmp(d.etiqueta, "normal") != 0 && d.conf >= CONF_MIN;
  return claseFalla || d.anom >= UMBRAL_ANOM;
}
static const char* causaDeDiag(const Diag& d) {
  return strcmp(d.etiqueta, "normal") != 0 ? d.etiqueta : "anomalia_desconocida";
}

// ============================== Actuadores ==============================
static void setRele(bool on) {
  releOn = on;
  digitalWrite(PIN_RELE, on ? RELE_NIVEL_ACTIVO : (RELE_NIVEL_ACTIVO == HIGH ? LOW : HIGH));
  gSim.setMotor(on);                                         // en simulacion, motor detenido = sin vibracion
}

static void actualizarLed() {
  bool on = (estado == E_PARO) || (estado == E_ADVERTENCIA && ((millis() / 250) % 2 == 0));
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

// ============================== MQTT ==============================
static void publicarAlerta(const char* tipo, const char* motivo) {
  if (!mqtt.connected()) return;
  char buf[200];
  snprintf(buf, sizeof(buf),
           "{\"up_s\":%lu,\"tipo\":\"%s\",\"causa\":\"%s\",\"conf\":%.2f,\"anom\":%.2f,\"temp_c\":%.1f}",
           (unsigned long)(millis() / 1000), tipo, motivo, diag.conf, diag.anom, tempC);
  mqtt.publish(TOPIC_ALERTA, buf);
  Serial.printf("[TX] %s %s\n", TOPIC_ALERTA, buf);
}

static void publicarTelemetria() {
  if (!mqtt.connected()) return;
  char buf[360];
  snprintf(buf, sizeof(buf),
           "{\"up_s\":%lu,\"estado\":\"%s\",\"clase\":\"%s\",\"conf\":%.2f,\"anom\":%.2f,\"umbral\":%.2f,"
           "\"rms_g\":%.3f,\"temp_c\":%.1f,\"rele\":%d,\"sim\":\"%s\",\"fs_hz\":%.1f,"
           "\"jitter_us\":%lu,\"gyro_dps\":%.1f,\"dsp_us\":%lu,\"rssi\":%d}",
           (unsigned long)(millis() / 1000), ESTADO_TXT[estado], diag.etiqueta, diag.conf, diag.anom,
           UMBRAL_ANOM, diag.rms, tempC, releOn ? 1 : 0, MG_MODO_NOMBRE[modoSim], fsMedida,
           (unsigned long)gJitterMaxUs, (float)gGyro, (unsigned long)diag.dspUs, WiFi.RSSI());
  mqtt.publish(TOPIC_TELEMETRIA, buf);
  Serial.printf("[TX] %s %s\n", TOPIC_TELEMETRIA, buf);
}

// ============================== Maquina de estados ==============================
static void cambiarEstado(Estado nuevo, const char* motivo) {
  if (nuevo == estado) return;
  estado = nuevo;
  strncpy(causa, motivo, sizeof(causa) - 1); causa[sizeof(causa) - 1] = 0;
  if (nuevo == E_PARO) setRele(false);                       // PARO DE EMERGENCIA: corta la energia
  Serial.printf("[ESTADO] -> %s (%s)\n", ESTADO_TXT[estado], causa);
  publicarAlerta(ESTADO_TXT[estado], causa);
}

static void forzarParo(const char* motivo) { cambiarEstado(E_PARO, motivo); }

static void rearmar(const char* origen) {
  if (estado != E_PARO) return;
  if (tempC >= TEMP_ADV) {
    Serial.printf("[REARME] rechazado (%s): temperatura %.1f C aun alta\n", origen, tempC);
    publicarAlerta("REARME_RECHAZADO", "temperatura_alta");
    return;
  }
  fallaRacha = 0;
  estado = E_NORMAL; causa[0] = 0;
  strcpy(diag.etiqueta, "normal"); diag.conf = 1; diag.anom = 0;
  setRele(true);
  gHoldoff = gCount + MG_WIN; gUltInfer = gHoldoff;        // la ventana previa tenia el motor detenido
  Serial.printf("[REARME] motor energizado (%s)\n", origen);
  publicarAlerta("REARME", origen);
}

static void evaluar(const Diag& d) {
  bool falla = esFalla(d);
  fallaRacha = falla ? fallaRacha + 1 : 0;
  if (tempC >= TEMP_PARO)                forzarParo("sobretemperatura");
  else if (fallaRacha >= VENTANAS_PARO)  forzarParo(causaDeDiag(d));
  else if (fallaRacha >= 1)              cambiarEstado(E_ADVERTENCIA, causaDeDiag(d));
  else if (tempC >= TEMP_ADV)            cambiarEstado(E_ADVERTENCIA, "temperatura_alta");
  else                                   cambiarEstado(E_NORMAL, "recuperado");
}

// ============================== Comandos MQTT ==============================
static void alRecibir(char*, byte* payload, unsigned int len) {
  char cmd[32];
  unsigned n = len < sizeof(cmd) - 1 ? len : sizeof(cmd) - 1;
  memcpy(cmd, payload, n); cmd[n] = 0;
  Serial.printf("[RX] comando: %s\n", cmd);
  if (!strcmp(cmd, "rearmar")) rearmar("mqtt");
  else if (!strcmp(cmd, "paro")) forzarParo("paro_manual");
  else if (!strncmp(cmd, "sim:", 4)) {
    for (int i = 0; i < MODO_N; i++) if (!strcmp(cmd + 4, MG_MODO_NOMBRE[i])) modoSim = i;
  }
}

static void conectarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);                       // canal 6 = conexion rapida en Wokwi
  Serial.print("[WiFi] conectando");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); Serial.print('.'); }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n[WiFi] conectado | IP %s | RSSI %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  else
    Serial.println("\n[WiFi] sin conexion (el paro local sigue funcionando)");
}

static void mqttReconectar() {
  static uint32_t ult = 0;
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;
  if (millis() - ult < 5000 && ult) return;
  ult = millis();
  char id[32]; snprintf(id, sizeof(id), "motorguard-%04lx", (unsigned long)random(0xFFFF));
  if (mqtt.connect(id, nullptr, nullptr, TOPIC_ESTADO, 1, true, "offline")) {
    mqtt.publish(TOPIC_ESTADO, "online", true);
    mqtt.subscribe(TOPIC_COMANDO);
    Serial.printf("[MQTT] conectado a %s:%d | base %s\n", MQTT_HOST, MQTT_PORT, TOPIC_BASE);
  } else {
    Serial.printf("[MQTT] fallo rc=%d, reintento en 5 s\n", mqtt.state());
  }
}

// ============================== Entradas ==============================
static Boton btnSim = { PIN_BTN_SIM, HIGH, 0 }, btnRst = { PIN_BTN_RST, HIGH, 0 };
static bool pulsado(Boton& b) {                              // flanco de bajada con antirrebote
  bool v = digitalRead(b.pin);
  if (v != b.ultimo && millis() - b.t > 40) {
    b.t = millis(); b.ultimo = v;
    return v == LOW;
  }
  return false;
}

static void leerTemperatura() {                              // potenciometro 0-3.3 V -> 20-120 C (filtro EMA)
  float t = 20.0f + (analogRead(PIN_POT) / 4095.0f) * 100.0f;
  tempC += 0.2f * (t - tempC);
}

// ============================== setup / loop ==============================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== MotorGuard fw 1.0 ===");
  Serial.printf("[BOOT] motor IA: %s | muestreo %d Hz | ventana %d muestras | vibracion %s\n",
                USE_EDGE_IMPULSE ? "Edge Impulse" : "modelo local", MG_FS_HZ, MG_WIN,
                SIMULAR_VIBRACION ? "simulada" : "real");

  pinMode(PIN_RELE, OUTPUT); pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BTN_SIM, INPUT_PULLUP); pinMode(PIN_BTN_RST, INPUT_PULLUP);
  analogReadResolution(12);
  randomSeed(micros());
  setRele(false);                                            // seguro por defecto: sin energia

  bool mpuOk = mpuInit();
  Serial.printf("[MPU6050] %s\n", mpuOk ? "detectado en 0x68" : "NO RESPONDE");
  xTaskCreatePinnedToCore(tareaMuestreo, "muestreo", 4096, nullptr, 3, nullptr, 1);

  conectarWiFi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(2);                                  // un broker caido no debe congelar el lazo de seguridad
  mqtt.setCallback(alRecibir);

  if (mpuOk) {                                               // el motor solo arranca si el monitoreo esta vivo
    setRele(true);
    gHoldoff = gCount + MG_WIN; gUltInfer = gHoldoff;
    Serial.println("[REL] motor energizado");
  } else {
    estado = E_PARO; strcpy(causa, "mpu6050_no_detectado");
  }
}

void loop() {
  static uint32_t tTemp = 0, tTel = 0, tWifi = 0, usPrev = 0, cntPrev = 0;
  uint32_t ahora = millis();

  if (WiFi.status() != WL_CONNECTED && ahora - tWifi > 10000) { tWifi = ahora; WiFi.reconnect(); }
  mqttReconectar();
  mqtt.loop();

  if (pulsado(btnSim)) {
    modoSim = (modoSim + 1) % MODO_N;
    Serial.printf("[SIM] falla simulada: %s\n", MG_MODO_NOMBRE[modoSim]);
  }
  if (pulsado(btnRst)) rearmar("boton");

  if (ahora - tTemp >= 100) { tTemp = ahora; leerTemperatura(); }

  // Diagnostico: una inferencia por segundo sobre la ultima ventana de 2 s
  if (estado != E_PARO && gCount >= gHoldoff && gCount - gUltInfer >= HOP_MUESTRAS) {
    gUltInfer = gCount;
    uint32_t us = micros(), cnt = gCount;
    if (usPrev) fsMedida = (cnt - cntPrev) * 1e6f / (float)(us - usPrev);
    usPrev = us; cntPrev = cnt;

    if (gI2cErr - i2cErrPrev > 20) {                         // sensor mudo -> no operar a ciegas
      i2cErrPrev = gI2cErr; forzarParo("falla_sensor");
    } else if (inferir(diag)) {
      i2cErrPrev = gI2cErr;
      evaluar(diag);
      Serial.printf("[INF] clase=%s conf=%.2f anom=%.2f rms=%.3fg temp=%.1fC estado=%s | dsp=%luus fs=%.1fHz jitter=%luus\n",
                    diag.etiqueta, diag.conf, diag.anom, diag.rms, tempC, ESTADO_TXT[estado],
                    (unsigned long)diag.dspUs, fsMedida, (unsigned long)gJitterMaxUs);
    }
  }

  // Temperatura critica tambien dispara el paro aunque el motor ya este detenido por otra causa
  if (estado != E_PARO && tempC >= TEMP_PARO) forzarParo("sobretemperatura");

  if (ahora - tTel >= PERIODO_TELEMETRIA_MS) { tTel = ahora; publicarTelemetria(); }

  actualizarLed();
  delay(2);
}
