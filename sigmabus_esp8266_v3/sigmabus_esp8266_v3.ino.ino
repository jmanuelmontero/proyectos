#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266Ping.h>
#include <Wire.h>
#include <time.h>
#include <LittleFS.h>
#include <WiFiManager.h>       // Librería "WiFiManager" de tzapu — instálala desde el Gestor de Librerías del IDE de Arduino
#include <ArduinoJson.h>       // Librería "ArduinoJson", VERSION 6.x (no la 7) — mismo Gestor de Librerías
#include "Adafruit_SGP30.h"

// Versión del firmware (se imprime al encender, para saber qué equipo la tiene)
const char* VERSION_FIRMWARE = "v3 | WiFi estable: red protegida + envio pausado + diagnostico";

//=================== DHT22 ===================
#define DHTPIN D5
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ======================================
// LECTURA CONFIABLE DEL DHT22
//
// Antes, si el sensor fallaba (NaN), el código ponía temperatura = 0 y
// humedad = 0 y ENVIABA esos ceros como si fueran reales (por eso
// aparecían puntos de 0 °C en las gráficas). Ahora:
//   - se reintenta hasta DHT_INTENTOS veces (el DHT22 necesita >= 2 s
//     entre lecturas);
//   - una lectura fuera del rango de abajo también se considera error;
//   - si ningún intento es válido, esa lectura se OMITE (no se envía ni
//     se guarda): es mejor un hueco de 3 minutos que un dato falso.
//
// Si en tu zona quieres descartar temperaturas imposibles dentro de un
// bus, ajusta TEMP_MIN_VALIDA / TEMP_MAX_VALIDA.
// ======================================
const int   DHT_INTENTOS    = 3;
const float TEMP_MIN_VALIDA = 2.0;    // °C
const float TEMP_MAX_VALIDA = 70.0;   // °C
const float HUM_MIN_VALIDA  = 1.0;    // %
const float HUM_MAX_VALIDA  = 100.0;  // %

bool leerDHTConfiable(float &temp, float &hum) {

  for (int intento = 1; intento <= DHT_INTENTOS; intento++) {

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    bool valida = !isnan(h) && !isnan(t) &&
                  t >= TEMP_MIN_VALIDA && t <= TEMP_MAX_VALIDA &&
                  h >= HUM_MIN_VALIDA && h <= HUM_MAX_VALIDA;

    if (valida) {
      temp = t;
      hum = h;
      return true;
    }

    Serial.print("DHT22: lectura invalida (intento ");
    Serial.print(intento);
    Serial.print(" de ");
    Serial.print(DHT_INTENTOS);
    Serial.println(")");

    if (intento < DHT_INTENTOS) {
      delay(2100);   // el DHT22 no admite lecturas con menos de 2 s de diferencia
    }
  }

  return false;
}

//=================== SGP30 ===================
Adafruit_SGP30 sgp;

int respuesta_wifi = -1;

//=================== GPS =====================
static const int RXPin = D7;   // GPS TX -> D7
static const int TXPin = D6;   // No conectar físicamente

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

//=================== SERVIDOR =====================
const String server =
"http://jhomon86.freehostia.com/php/enviar_datos.php";

//=================== ALMACENAMIENTO OFFLINE =====================
const char* ARCHIVO_PENDIENTES = "/pendientes.csv";
const size_t TAMANO_MAXIMO_ARCHIVO = 300000; // ~300 KB tope, para no llenar la flash
const int MAX_REENVIOS_POR_CICLO = 5;        // no vaciar todo el backlog de golpe

// Al encender con una WiFi ya guardada, se espera como máximo este tiempo a que
// conecte y se SIGUE con las lecturas (antes se quedaba hasta 5 minutos en el
// portal de configuración si el router del bus todavía no estaba listo).
const unsigned long ESPERA_WIFI_ARRANQUE_MS = 15000;

// ======================================
// AJUSTES DE ESTABILIDAD WIFI (v3)
// ======================================
const unsigned long CICLO_MEDICION_MS = 180000;                     // cada cuanto se mide (3 min)
const unsigned long INTERVALO_VIGILANCIA_MS = 20000;                // cada cuanto se revisa la WiFi entre mediciones
const unsigned long PAUSA_ENTRE_ENVIOS_MS = 2000;                   // pausa entre un pendiente y el siguiente
const unsigned long MAX_SIN_WIFI_ANTES_DE_REINICIAR_MS = 900000UL;  // 15 min sin WiFi -> reinicia el equipo
const char* HOST_SERVIDOR = "jhomon86.freehostia.com";              // solo para el diagnostico de red

// Ajustes OPCIONALES para probar si el diagnostico apunta a un problema de radio
// (motivo 200 = BEACON_TIMEOUT). Estan apagados porque no se ha comprobado que hagan falta.
const bool USAR_WIFI_SIN_AHORRO_ENERGIA = false;   // true = la radio nunca "duerme" (mas consumo)
const bool FORZAR_WIFI_11G = false;                // true = solo 802.11g (menos picos de corriente)

// ======================================
// CONFIGURACION POR PORTAL WEB (placa, interno, token)
//
// Antes estos tres valores estaban escritos directo en el código
// (había que recompilar y volver a subir el firmware para cada bus
// distinto). Ahora se guardan en la memoria flash del propio equipo
// (LittleFS, archivo /config.json) y se cargan al encender — el
// MISMO archivo .ino, compilado UNA sola vez, sirve para los 300
// vehículos. Cada equipo se configura después, por WiFi, con el
// portal que abre WiFiManager.
// ======================================

char placaGuardada[15] = "";
char internoGuardado[10] = "";
char apiTokenGuardado[70] = "";

// Respaldo de la red WiFi (ver "PROTEGER LA RED WIFI GUARDADA" mas abajo)
char ssidRespaldo[33] = "";
char pskRespaldo[65] = "";
bool credencialesCambiaron = false;

const char* ARCHIVO_CONFIG = "/config.json";

bool debeGuardarConfig = false;

void callbackGuardarConfig() {
  debeGuardarConfig = true;
}

void cargarConfig() {

  if (!LittleFS.exists(ARCHIVO_CONFIG)) {
    return;
  }

  File archivo = LittleFS.open(ARCHIVO_CONFIG, "r");

  if (!archivo) {
    return;
  }

  StaticJsonDocument<512> doc;

  DeserializationError error = deserializeJson(doc, archivo);

  if (!error) {
    strlcpy(placaGuardada, doc["placa"] | "", sizeof(placaGuardada));
    strlcpy(internoGuardado, doc["interno"] | "", sizeof(internoGuardado));
    strlcpy(apiTokenGuardado, doc["token"] | "", sizeof(apiTokenGuardado));
    strlcpy(ssidRespaldo, doc["ssid"] | "", sizeof(ssidRespaldo));
    strlcpy(pskRespaldo, doc["psk"] | "", sizeof(pskRespaldo));
  } else {
    Serial.println("No se pudo leer config.json (puede que aun no exista)");
  }

  archivo.close();
}

void guardarConfig() {

  StaticJsonDocument<512> doc;

  doc["placa"] = placaGuardada;
  doc["interno"] = internoGuardado;
  doc["token"] = apiTokenGuardado;
  doc["ssid"] = ssidRespaldo;
  doc["psk"] = pskRespaldo;

  File archivo = LittleFS.open(ARCHIVO_CONFIG, "w");

  if (archivo) {
    serializeJson(doc, archivo);
    archivo.close();
    Serial.println("config.json guardado.");
  } else {
    Serial.println("No se pudo guardar config.json");
  }
}

// ======================================
// ABRIR EL PORTAL SIN NECESITAR UN BOTON FISICO
//
// Para volver a configurar un equipo ya instalado (por ejemplo, se
// cambia de bus o de empresa) sin tener que destaparlo ni conectarlo
// a una computadora: apágalo y préndelo 2 veces seguidas, rápido
// (antes de que pasen ~20 segundos de cada encendido). El equipo lo
// detecta solo y abre el portal de configuración.
// ======================================

const char* ARCHIVO_BANDERA_REINICIO = "/boot_flag.txt";
const unsigned long VENTANA_CONFIRMACION_MS = 20000; // 20s "andando bien" antes de olvidar el conteo

bool banderaLimpiada = false;
unsigned long inicioSistema = 0;

bool detectarReinicioRapido() {

  int contador = 0;

  if (LittleFS.exists(ARCHIVO_BANDERA_REINICIO)) {

    File f = LittleFS.open(ARCHIVO_BANDERA_REINICIO, "r");

    if (f) {
      contador = f.parseInt();
      f.close();
    }
  }

  contador++;

  File f = LittleFS.open(ARCHIVO_BANDERA_REINICIO, "w");

  if (f) {
    f.print(contador);
    f.close();
  }

  Serial.print("Encendidos rapidos seguidos detectados: ");
  Serial.println(contador);

  return contador >= 2;
}

//=================================================
// PROTEGER LA RED WIFI GUARDADA
//
// En el ESP8266, WiFi.disconnect() BORRA de la memoria la red guardada
// (cuando WiFi.persistent esta en true, que es el valor por defecto). Un
// reintento de conexion que llame a WiFi.disconnect() deja al equipo sin red
// para siempre y, al reiniciar, abre el portal ("No wifi saved").
//
// Por eso:
//  1) la red (SSID y clave) tambien se guarda en /config.json como respaldo;
//  2) para reconectar se usa SIEMPRE ese respaldo con WiFi.begin(ssid, clave);
//  3) mientras se reconecta, persistent se apaga (no se escribe la flash en
//     cada intento) y se vuelve a encender al terminar.
//=================================================

volatile bool desconexionPendiente = false;
volatile uint8_t motivoDesconexion = 0;
WiFiEventHandler manejadorDesconexion;

unsigned long inicioSinWiFi = 0;
int intentosReconexion = 0;
bool redSospechosa = false;   // el ultimo envio fallo aunque el estado diga "conectado"

const char* nombreMotivoDesconexion(uint8_t m) {

  switch (m) {
    case 1:   return "UNSPECIFIED";
    case 2:   return "AUTH_EXPIRE (la autenticacion expiro)";
    case 3:   return "AUTH_LEAVE (el router nos desconecto)";
    case 4:   return "ASSOC_EXPIRE";
    case 5:   return "ASSOC_TOOMANY (el router tiene demasiados equipos)";
    case 6:   return "NOT_AUTHED";
    case 7:   return "NOT_ASSOCED";
    case 8:   return "ASSOC_LEAVE (el router nos saco)";
    case 9:   return "ASSOC_NOT_AUTHED";
    case 15:  return "4WAY_HANDSHAKE_TIMEOUT";
    case 200: return "BEACON_TIMEOUT (se perdio la senal del router)";
    case 201: return "NO_AP_FOUND (no se ve la red)";
    case 202: return "AUTH_FAIL (clave rechazada)";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    default:  return "otro";
  }
}

void configurarRadioWiFi() {

  WiFi.setAutoReconnect(true);

  if (USAR_WIFI_SIN_AHORRO_ENERGIA) {
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
  }

  if (FORZAR_WIFI_11G) {
    WiFi.setPhyMode(WIFI_PHY_MODE_11G);
  }
}

// Copia la red a la que se esta conectado al respaldo (si cambio)
void capturarCredencialesWiFi() {

  String ssid = WiFi.SSID();
  String psk = WiFi.psk();

  if (ssid.length() == 0 || ssid.length() >= sizeof(ssidRespaldo) || psk.length() >= sizeof(pskRespaldo)) {
    return;
  }

  if (strcmp(ssidRespaldo, ssid.c_str()) != 0 || strcmp(pskRespaldo, psk.c_str()) != 0) {

    strlcpy(ssidRespaldo, ssid.c_str(), sizeof(ssidRespaldo));
    strlcpy(pskRespaldo, psk.c_str(), sizeof(pskRespaldo));

    credencialesCambiaron = true;
  }
}

void informarDesconexionSiHubo() {

  if (!desconexionPendiente) {
    return;
  }

  desconexionPendiente = false;

  Serial.print("WiFi: la conexion se corto. Motivo ");
  Serial.print(motivoDesconexion);
  Serial.print(" - ");
  Serial.println(nombreMotivoDesconexion(motivoDesconexion));
  Serial.print("Heap libre: ");
  Serial.println(ESP.getFreeHeap());
}

// Cuando un envio falla: que es lo que realmente esta fallando?
void diagnosticarFalloDeRed() {

  Serial.println("--- Diagnostico de red ---");

  Serial.print("Estado WiFi (3 = conectado): ");
  Serial.println(WiFi.status());

  Serial.print("Senal (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

  Serial.print("Heap libre: ");
  Serial.println(ESP.getFreeHeap());

  IPAddress ipServidor;
  unsigned long t0 = millis();
  bool dnsOk = WiFi.hostByName(HOST_SERVIDOR, ipServidor, 2000);

  Serial.print("DNS del servidor: ");
  Serial.print(dnsOk ? "OK " : "FALLA ");
  Serial.print(millis() - t0);
  Serial.println(" ms");

  bool routerOk = Ping.ping(WiFi.gatewayIP(), 1);

  Serial.print("El router responde: ");
  Serial.println(routerOk ? "si" : "NO");

  Serial.println("--------------------------");
}

//=================================================
// RECONEXION WIFI (reintento normal durante el funcionamiento —
// distinto de la configuracion inicial, que la maneja WiFiManager)
//
// forzar = true: reconecta aunque WiFi.status() diga "conectado"
// (enlace "zombi": el estado dice conectado pero nada pasa).
//=================================================
bool reconectarWiFi(bool forzar) {

  informarDesconexionSiHubo();

  if (!forzar && WiFi.status() == WL_CONNECTED) {
    inicioSinWiFi = 0;
    intentosReconexion = 0;
    return true;
  }

  if (inicioSinWiFi == 0) {
    inicioSinWiFi = millis();
  }

  intentosReconexion++;

  Serial.println("WiFi desconectado, reintentando con la red guardada...");

  WiFi.persistent(false);   // NO tocar la flash ni borrar la red guardada
  WiFi.disconnect();
  delay(200);

  // Cada 3 intentos seguidos se reinicia tambien la radio por completo
  if (intentosReconexion % 3 == 0) {

    WiFi.mode(WIFI_OFF);
    delay(300);
    WiFi.mode(WIFI_STA);
    configurarRadioWiFi();

    Serial.println("(radio WiFi reiniciada)");
  }

  if (strlen(ssidRespaldo) > 0) {
    WiFi.begin(ssidRespaldo, pskRespaldo);
  } else {
    WiFi.begin();   // sin respaldo todavia: usa lo que tenga guardado el ESP
  }

  unsigned long inicio = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - inicio < 10000) {
    delay(500);
    Serial.print(".");
  }

  WiFi.persistent(true);

  if (WiFi.status() == WL_CONNECTED) {

    Serial.println("\nReconectado.");

    inicioSinWiFi = 0;
    intentosReconexion = 0;
    redSospechosa = false;

    capturarCredencialesWiFi();

    if (credencialesCambiaron) {
      guardarConfig();
      credencialesCambiaron = false;
    }

    return true;
  }

  Serial.println("\nSigue sin conexion.");

  return false;
}

bool asegurarWiFi() {
  return reconectarWiFi(false);
}

// Ultimo recurso: si lleva mucho rato sin WiFi, reiniciar el equipo
void controlarTiempoSinWiFi() {

  if (inicioSinWiFi != 0 && millis() - inicioSinWiFi > MAX_SIN_WIFI_ANTES_DE_REINICIAR_MS) {

    Serial.println("Mas de 15 minutos sin WiFi - se reinicia el equipo para recuperar la conexion.");
    delay(200);
    ESP.restart();
  }
}

//=================================================
// HORA REAL TOMADA DEL GPS (no depende de internet)
//=================================================
String obtenerFechaBogota() {

  if (!gps.date.isValid() || !gps.time.isValid()) {
    return "";
  }

  struct tm tGPS = {0};
  tGPS.tm_year = gps.date.year() - 1900;
  tGPS.tm_mon  = gps.date.month() - 1;
  tGPS.tm_mday = gps.date.day();
  tGPS.tm_hour = gps.time.hour();
  tGPS.tm_min  = gps.time.minute();
  tGPS.tm_sec  = gps.time.second();

  time_t epochUTC = mktime(&tGPS);

  const long OFFSET_BOGOTA_SEGUNDOS = -5L * 3600L; // Bogotá = UTC-5

  time_t epochBogota = epochUTC + OFFSET_BOGOTA_SEGUNDOS;

  struct tm tBogota;
  gmtime_r(&epochBogota, &tBogota);

  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tBogota);

  return String(buffer);
}

//=================================================
// ENVIAR UNA LECTURA AL SERVIDOR
//=================================================
bool enviarLectura(
  String placaEnv, String internoEnv,
  float temp, float hum, float vel,
  int latenciaMs, int eco2v, int tvocv,
  double latitud, double longitud,
  String fecha
) {

  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  WiFiClient client;
  HTTPClient http;

  String url = server +
               "?token=" + String(apiTokenGuardado) +
               "&temperatura=" + String(temp, 1) +
               "&humedad=" + String(hum, 1) +
               "&placa=" + placaEnv +
               "&interno=" + internoEnv +
               "&longitud=" + String(longitud, 6) +
               "&latitud=" + String(latitud, 6) +
               "&latencia=" + String(latenciaMs) +
               "&velocidad=" + String(vel, 1) +
               "&eco2=" + String(eco2v) +
               "&tvoc=" + String(tvocv);

  if (fecha.length() > 0) {

    String fechaCodificada = fecha;
    fechaCodificada.replace(" ", "%20");

    url += "&fecha=" + fechaCodificada;
  }

  Serial.println("Enviando:");
  Serial.println(url);

  http.begin(client, url);

  // El valor por defecto (5 s) se queda corto con una conexion lenta + el
  // hosting gratuito: el servidor GUARDA el dato pero la respuesta llega
  // tarde, el equipo cree que fallo y lo vuelve a enviar (duplicado).
  http.setTimeout(15000);

  int httpCode = http.GET();

  bool exito = false;

  if (httpCode == 200) {

    String payload = http.getString();
    Serial.println(payload);

    if (payload.indexOf("\"estado\":\"ok\"") >= 0) {
      exito = true;
      redSospechosa = false;
    }

  } else {

    Serial.print("Error HTTP: ");
    Serial.print(httpCode);

    if (httpCode < 0) {
      Serial.print(" (");
      Serial.print(HTTPClient::errorToString(httpCode));
      Serial.print(")");
    }

    Serial.println();

    // Fallo de transporte (no una respuesta del servidor): el enlace es sospechoso
    if (httpCode < 0) {
      redSospechosa = true;
    }
  }

  http.end();

  if (redSospechosa && !exito) {
    diagnosticarFalloDeRed();
  }

  return exito;
}

//=================================================
// GUARDAR UNA LECTURA COMO PENDIENTE
//=================================================
void guardarPendiente(String linea) {

  File verificar = LittleFS.open(ARCHIVO_PENDIENTES, "r");
  size_t tamanoActual = verificar ? verificar.size() : 0;
  if (verificar) verificar.close();

  if (tamanoActual >= TAMANO_MAXIMO_ARCHIVO) {
    Serial.println("Archivo de pendientes lleno (limite de flash) — se descarta esta lectura.");
    return;
  }

  File archivo = LittleFS.open(ARCHIVO_PENDIENTES, "a");

  if (!archivo) {
    Serial.println("No se pudo abrir el archivo de pendientes para escribir.");
    return;
  }

  archivo.println(linea);
  archivo.close();

  Serial.println("Lectura guardada como pendiente (sin conexión).");
}

//=================================================
// REENVIAR LO QUE QUEDO PENDIENTE
//=================================================
void procesarPendientes(int maxEnvios) {

  if (!LittleFS.exists(ARCHIVO_PENDIENTES)) {
    return;
  }

  File origen = LittleFS.open(ARCHIVO_PENDIENTES, "r");

  if (!origen) {
    return;
  }

  File temporal = LittleFS.open("/pendientes_tmp.csv", "w");

  int enviados = 0;
  bool detener = false;   // tras el primer fallo NO se intenta con el resto

  while (origen.available()) {

    String linea = origen.readStringUntil('\n');
    linea.trim();

    if (linea.length() == 0) {
      continue;
    }

    bool yaEnviado = false;

    if (!detener && enviados < maxEnvios && WiFi.status() == WL_CONNECTED) {

      String valores[11];
      int campo = 0;
      int inicio = 0;

      for (int i = 0; i <= (int)linea.length(); i++) {

        if (i == (int)linea.length() || linea[i] == ',') {

          if (campo < 11) {
            valores[campo] = linea.substring(inicio, i);
          }

          campo++;
          inicio = i + 1;
        }
      }

      if (campo == 11) {

        bool ok = enviarLectura(
          valores[0], valores[1],
          valores[2].toFloat(), valores[3].toFloat(),
          valores[4].toFloat(), valores[5].toInt(),
          valores[6].toInt(), valores[7].toInt(),
          atof(valores[8].c_str()), atof(valores[9].c_str()),
          valores[10]
        );

        if (ok) {
          yaEnviado = true;
          enviados++;
          delay(PAUSA_ENTRE_ENVIOS_MS);   // sin rafagas: una peticion cada 2 s
        } else {
          detener = true;                 // la conexion esta mala: el resto queda para despues
          Serial.println("Envio de pendientes detenido en el primer fallo.");
        }
      }
    }

    if (!yaEnviado) {
      temporal.println(linea);
    }
  }

  origen.close();
  temporal.close();

  LittleFS.remove(ARCHIVO_PENDIENTES);
  LittleFS.rename("/pendientes_tmp.csv", ARCHIVO_PENDIENTES);

  if (enviados > 0) {
    Serial.print("Pendientes reenviados en este ciclo: ");
    Serial.println(enviados);
  }
}

//=================================================
// VIGILAR LA CONEXION ENTRE MEDICIONES
//
// En vez de dormir 3 minutos seguidos, se revisa la WiFi cada 20 s:
//  - si se cayo, se reconecta (sin borrar la red guardada);
//  - si el ultimo envio fallo pero el estado dice "conectado", se comprueba
//    si el router de verdad responde y, si no, se fuerza la reconexion;
//  - cuando la conexion vuelve, se vacia la cola de pendientes (con calma);
//  - si pasan 15 minutos sin WiFi, se reinicia el equipo.
//=================================================
void vigilarConexion() {

  bool estabaConectado = (WiFi.status() == WL_CONNECTED);

  informarDesconexionSiHubo();

  if (!estabaConectado) {

    if (reconectarWiFi(false)) {
      delay(3000);   // dejar que la conexion se asiente antes de enviar
      procesarPendientes(MAX_REENVIOS_POR_CICLO);
    }

  } else if (redSospechosa) {

    bool routerOk = Ping.ping(WiFi.gatewayIP(), 1);

    redSospechosa = false;

    if (routerOk) {

      Serial.println("El router responde: el fallo fue del servidor o de internet, no de la WiFi.");

    } else {

      Serial.println("El router NO responde aunque el estado decia conectado - se fuerza la reconexion.");

      if (reconectarWiFi(true)) {
        delay(3000);
        procesarPendientes(MAX_REENVIOS_POR_CICLO);
      }
    }
  }

  controlarTiempoSinWiFi();
}

void esperarProximoCiclo() {

  unsigned long inicio = millis();
  unsigned long ultimaVigilancia = millis();

  while (millis() - inicio < CICLO_MEDICION_MS) {

    delay(250);

    if (millis() - ultimaVigilancia >= INTERVALO_VIGILANCIA_MS) {

      ultimaVigilancia = millis();

      vigilarConexion();
    }
  }
}

void setup() {

  Serial.begin(115200);

  Serial.println();
  Serial.println("Iniciando sistema...");
  Serial.print("Firmware: ");
  Serial.println(VERSION_FIRMWARE);

  inicioSistema = millis();

  // MEMORIA FLASH
  if (!LittleFS.begin()) {
    Serial.println("LittleFS no se pudo montar, formateando...");
    LittleFS.format();
    LittleFS.begin();
  }

  bool forzarPortal = detectarReinicioRapido();

  cargarConfig();

  // DHT22
  dht.begin();

  // GPS
  gpsSerial.begin(9600);

  // SGP30
  Wire.begin(D2, D1);   // SDA, SCL

  if (!sgp.begin()) {
    Serial.println("No se encontro el SGP30");
  } else {
    Serial.println("SGP30 listo");
  }

  // ======================================
  // PORTAL DE CONFIGURACION (WiFiManager)
  //
  // Campos personalizados: placa, interno y token — además de los
  // campos de red WiFi que WiFiManager ya trae por defecto.
  // ======================================

  WiFiManagerParameter custom_placa("placa", "Placa del vehiculo (ej. WEO463)", placaGuardada, 14);
  WiFiManagerParameter custom_interno("interno", "Numero interno (ej. 7910)", internoGuardado, 9);
  WiFiManagerParameter custom_token("token", "Token de la empresa (api_key)", apiTokenGuardado, 69);

  WiFiManager wifiManager;

  wifiManager.setSaveConfigCallback(callbackGuardarConfig);
  wifiManager.addParameter(&custom_placa);
  wifiManager.addParameter(&custom_interno);
  wifiManager.addParameter(&custom_token);

  // Si nadie configura nada en 5 minutos, deja de insistir con el
  // portal y sigue con el ciclo normal (así el equipo no se queda
  // "colgado" esperando para siempre si lo instalaron y se fueron).
  wifiManager.setConfigPortalTimeout(300);

  bool conectado;

  // ¿Hay una red WiFi guardada en el equipo? (hay que poner el modo STA
  // primero para poder leerla)
  WiFi.mode(WIFI_STA);
  configurarRadioWiFi();

  // Para saber POR QUE se corta la WiFi (se imprime en el loop, no aqui)
  manejadorDesconexion = WiFi.onStationModeDisconnected(
    [](const WiFiEventStationModeDisconnected& evento) {
      motivoDesconexion = evento.reason;
      desconexionPendiente = true;
    });

  bool hayRedGuardada = WiFi.SSID().length() > 0;

  // Equipo que ya traia su red en la memoria del ESP pero todavia no tiene
  // respaldo en config.json: se crea el respaldo.
  if (strlen(ssidRespaldo) == 0 && hayRedGuardada &&
      WiFi.SSID().length() < sizeof(ssidRespaldo) && WiFi.psk().length() < sizeof(pskRespaldo)) {

    strlcpy(ssidRespaldo, WiFi.SSID().c_str(), sizeof(ssidRespaldo));
    strlcpy(pskRespaldo, WiFi.psk().c_str(), sizeof(pskRespaldo));

    credencialesCambiaron = true;
  }

  // La memoria del ESP perdio la red pero el respaldo la conserva: se restaura
  // (asi no se abre el portal por error).
  if (!hayRedGuardada && strlen(ssidRespaldo) > 0 && !forzarPortal) {

    Serial.println("La red WiFi guardada se habia borrado - se restaura desde el respaldo.");

    hayRedGuardada = true;
  }

  if (forzarPortal) {

    Serial.println("Apagado/encendido rapido detectado - abriendo portal de configuracion...");

    // AbrE el portal aunque ya haya una red WiFi guardada y
    // funcionando — es la forma de "reconfigurar" un equipo que ya
    // estaba en uso.
    conectado = wifiManager.startConfigPortal("SIGMABUS-Config", "sigmabus123");

  } else if (!hayRedGuardada) {

    // Equipo nuevo (recién flasheado, sin red guardada): aquí SÍ hace falta
    // el portal, así que se abre automáticamente.
    conectado = wifiManager.autoConnect("SIGMABUS-Config", "sigmabus123");

  } else {

    // Equipo ya configurado: NO se abre el portal ni se bloquea el arranque.
    // Se intenta conectar unos segundos y se sigue con las lecturas aunque el
    // router del bus todavía no esté listo; lo que no se pueda enviar se guarda
    // y se reenvía luego, y asegurarWiFi() sigue reintentando la conexión.
    Serial.println("Red WiFi guardada - conectando sin abrir el portal...");

    if (strlen(ssidRespaldo) > 0) {
      WiFi.begin(ssidRespaldo, pskRespaldo);
    } else {
      WiFi.begin();
    }

    unsigned long inicioEspera = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - inicioEspera < ESPERA_WIFI_ARRANQUE_MS) {
      delay(250);
    }

    conectado = (WiFi.status() == WL_CONNECTED);
  }

  if (conectado) {
    Serial.println("WiFi conectado.");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    capturarCredencialesWiFi();   // respaldo de la red a la que se conecto
  } else {
    Serial.println("Sin WiFi por ahora - se empieza a medir y guardar igual; se seguira intentando conectar.");
  }

  if (debeGuardarConfig) {

    strlcpy(placaGuardada, custom_placa.getValue(), sizeof(placaGuardada));
    strlcpy(internoGuardado, custom_interno.getValue(), sizeof(internoGuardado));
    strlcpy(apiTokenGuardado, custom_token.getValue(), sizeof(apiTokenGuardado));

    guardarConfig();
    credencialesCambiaron = false;

    Serial.println("Configuracion nueva guardada (placa/interno/token).");

  } else if (credencialesCambiaron) {

    guardarConfig();   // solo el respaldo de la red WiFi
    credencialesCambiaron = false;
  }

  Serial.print("Placa configurada: ");
  Serial.println(placaGuardada);
  Serial.print("Interno configurado: ");
  Serial.println(internoGuardado);
}

void loop() {

  // Pasado el "periodo de gracia" sin reinicios, se olvida el
  // conteo de encendidos rápidos — para que un apagado accidental
  // suelto, horas después, no cuente para el doble-reinicio.
  if (!banderaLimpiada && millis() - inicioSistema > VENTANA_CONFIRMACION_MS) {
    LittleFS.remove(ARCHIVO_BANDERA_REINICIO);
    banderaLimpiada = true;
    Serial.println("Arranque estable confirmado.");
  }

  //=================================================
  // RECONEXION WIFI SI SE PERDIO
  //=================================================
  asegurarWiFi();

  //=================================================
  // MEDIR INTERNET
  //=================================================
  IPAddress host(8, 8, 8, 8);

  if (Ping.ping(host, 5)) {

    respuesta_wifi = Ping.averageTime();

    Serial.print("Latencia: ");
    Serial.print(respuesta_wifi);
    Serial.println(" ms");

  } else {

    respuesta_wifi = -1;

    Serial.println("Error en Ping");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Senal WiFi: ");
    Serial.print(WiFi.RSSI());
    Serial.print(" dBm | Heap libre: ");
    Serial.println(ESP.getFreeHeap());
  }

  //=================================================
  // LEER DHT22
  //=================================================
  float humedad = 0;
  float temperatura = 0;

  bool lecturaDHTvalida = leerDHTConfiable(temperatura, humedad);

  if (lecturaDHTvalida) {

    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.println(" °C");

    Serial.print("Humedad: ");
    Serial.print(humedad);
    Serial.println(" %");
  }

  //=================================================
  // LEER SGP30
  //=================================================
  uint16_t eco2 = 0;
  uint16_t tvoc = 0;

  if (sgp.IAQmeasure()) {

    eco2 = sgp.eCO2;
    tvoc = sgp.TVOC;

    Serial.print("eCO2: ");
    Serial.print(eco2);
    Serial.println(" ppm");

    Serial.print("TVOC: ");
    Serial.print(tvoc);
    Serial.println(" ppb");

  } else {

    Serial.println("Error leyendo SGP30");
  }

  //=================================================
  // LEER GPS
  //=================================================
  unsigned long start = millis();

  while (millis() - start < 2000) {

    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
  }

  bool envioFallidoEnEsteCiclo = false;

  if (!lecturaDHTvalida) {

    // No se envían ni se guardan ceros inventados: esta lectura se omite.
    Serial.println("DHT22 sin lectura valida - se omite esta lectura (no se envian datos falsos).");

  } else if (gps.location.isValid()) {

    double latitud = gps.location.lat();
    double longitud = gps.location.lng();
    double velocidad = gps.speed.kmph();

    String fecha = obtenerFechaBogota();

    Serial.println("--------------------------");
    Serial.print("Latitud: ");
    Serial.println(latitud, 6);
    Serial.print("Longitud: ");
    Serial.println(longitud, 6);
    Serial.print("Velocidad: ");
    Serial.print(velocidad);
    Serial.println(" km/h");
    Serial.print("Fecha (GPS, Bogota): ");
    Serial.println(fecha.length() > 0 ? fecha : "(aun sin hora valida)");
    Serial.println("--------------------------");

    bool enviado = false;

    if (WiFi.status() == WL_CONNECTED) {

      enviado = enviarLectura(
        placaGuardada, internoGuardado,
        temperatura, humedad, velocidad,
        respuesta_wifi, eco2, tvoc,
        latitud, longitud, fecha
      );
    }

    if (!enviado) {

      envioFallidoEnEsteCiclo = true;

      if (fecha.length() > 0) {

        String linea = String(placaGuardada) + "," + String(internoGuardado) + "," +
                        String(temperatura, 1) + "," + String(humedad, 1) + "," +
                        String(velocidad, 1) + "," + String(respuesta_wifi) + "," +
                        String(eco2) + "," + String(tvoc) + "," +
                        String(latitud, 6) + "," + String(longitud, 6) + "," +
                        fecha;

        guardarPendiente(linea);

      } else {

        Serial.println("Sin hora de GPS todavia - esta lectura no se guarda ni se envia.");
      }
    }

  } else {

    Serial.println("Esperando señal GPS...");
  }

  //=================================================
  // SI HAY CONEXION, APROVECHAR PARA VACIAR EL BACKLOG
  //=================================================
  // Si el envio de ESTA vuelta acaba de fallar, la conexion esta mala: no se
  // reintenta segundos despues (era justo lo que generaba duplicados). Se
  // espera al siguiente ciclo.
  if (WiFi.status() == WL_CONNECTED && !envioFallidoEnEsteCiclo && !redSospechosa) {
    procesarPendientes(MAX_REENVIOS_POR_CICLO);
  }

  //=================================================
  // ESPERAR 3 MINUTOS (vigilando la WiFi cada 20 s)
  //=================================================
  esperarProximoCiclo();
}
