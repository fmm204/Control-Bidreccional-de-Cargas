#include <WiFi.h>             // Biblioteca WiFi para ESP32
#include <HTTPClient.h>       // Para peticiones HTTP
#include <WiFiClient.h>       // Cliente WiFi
#include <ArduinoJson.h>      // Para manejar JSON
#include <WebServer.h>        // Servidor web para ESP32
#include <NTPClient.h>        // Cliente NTP
#include <WiFiUdp.h>          // UDP para NTP
#include <time.h>             // Funciones de tiempo
#include <Arduino.h>
#include <driver/gpio.h>
#include "driver/ledc.h"


// WiFi credentials
const char* ssid = "Lowi55D0";
const char* password = "InternetOK";

// Web server instance
WebServer server(80);  // Port 80

// API details
const char* loginUrl = "http://192.168.0.254/logIn";
const char* statusUrl = "http://192.168.0.254/getStatus";

// Authentication
String sessionCookie = "";
unsigned long lastStatusRequestTime = 0;
const unsigned long statusRequestInterval = 60000; // 1 minute

// Power monitoring
bool isOverPowerLimit = false;
bool previousPowerLimitState = false;
float POWER_LIMIT_WATTS = 1200.0;
String powerStatusMessage = "No power data available yet";
unsigned int powerLimitExceededCount = 0;

// Counter for successful sent packets
unsigned long successfulLoopCount = 0;

// NTP Client setup
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
String formattedDate;
String dayStamp;
String timeStamp;

// Data structure to store different types of measurements
struct DeviceData {
  bool valid;
  String value;
  String unit;
  String timestamp;
  String arrayValues[50];
  int arraySize;
};

// Data containers with basic initialization
DeviceData powerData = {false, "", "W", "", {}, 0};
DeviceData frequencyData = {false, "", "Hz", "", {}, 0};
DeviceData ApparentPowerData = {false, "", "VA", "", {}, 0};
DeviceData voltageData = {false, "", "V", "", {}, 0};
DeviceData currentData = {false, "", "A", "", {}, 0};
DeviceData harmonicsData = {false, "", "V", "", {}, 0};


// Variables para almacenar los datos extraídos
struct SlaveData {
    uint8_t slaveID;       // ID del esclavo (1-16)
    bool slaveState;       // Estado (true=encendido, false=apagado)
    bool isValid;          // Indica si los datos son válidos
    uint32_t timestamp;    // Marca de tiempo cuando se recibió
};

// Buffer para almacenar últimos 5 comandos recibidos
#define MAX_STORED_COMMANDS 5
SlaveData commandHistory[MAX_STORED_COMMANDS];
int commandHistoryIndex = 0;
int totalCommandsReceived = 0;

// Último comando recibido
SlaveData lastValidCommand = {0, false, false, 0};


// Variables para detección de secuencia y almacenamiento de datos
#define SEQUENCE_LENGTH 6
#define CAPTURE_LENGTH 8
const uint8_t targetSequence[SEQUENCE_LENGTH] = {1, 1, 1, 0, 0, 0}; // Secuencia a detectar: 111000
uint8_t capturedBits[CAPTURE_LENGTH];  // Para almacenar los 12 bits después de la secuencia
bool sequenceFound = false; // Indicador si se encontró la secuencia

// Configuración del pin digital
const int inputPin = 34;      // GPIO34 para entrada digital
const int maxSamples = 8000;  // Cantidad de muestras a capturar por ciclo
const int displaySamples = 80; // Número de muestras a mostrar en pantalla

// Variables para flancos
int risingFlankIndex = -1;   // Índice del flanco ascendente
int fallingFlankIndex = -1;  // Índice del flanco descendente
int bitsInSignal = 0;        // Número de bits calculados según el tiempo

// Parámetros para decodificación de bits
#define FLANK_THRESHOLD 1     // Umbral para detectar un cambio en señal digital
#define BIT_THRESHOLD 1       // Umbral para determinar si un bit es 1 o 0 (en digital)
#define US_PER_BIT 100        // Microsegundos por bit (100us)
#define MAX_BITS 100          // Máximo número de bits a decodificar

// Estructura para almacenar información de los flancos detectados
#define MAX_FLANKS 50  // Número máximo de flancos a detectar

typedef struct {
    int index;         // Posición del flanco en el array de muestras
    bool isRising;     // true si es ascendente, false si es descendente
} FlankInfo;

// Variables para flancos múltiples
FlankInfo detectedFlanks[MAX_FLANKS];
int flankCount = 0;            // Contador de flancos detectados

// Variables para almacenar datos
uint8_t samples[maxSamples];   // Almacena sólo 0 o 1 (digital)
unsigned long captureInterval = 500; // Milisegundo entre capturas
unsigned long lastCaptureTime = 0;
int captureCount = 0;        // Contador de capturas
bool showFullArray = false;  // Control para mostrar array completo

// Variables para análisis
unsigned long startTime;
unsigned long endTime;
float samplingRate;

// Variables para la decodificación de bits
uint8_t decodedBits[MAX_BITS];    // Array para almacenar los bits decodificados
int bitCount = 0;                // Contador de bits decodificados



// Definiciones para el sistema de 16 esclavos
#define NUM_SLAVES 16
#define TIMEOUT_MS 90000  // 90 segundos de timeout

// Estados posibles para los esclavos
enum SlaveState {
    DISCONNECTED,   // Nunca se ha recibido información de este esclavo
    CONNECTED,      // Esclavo funcionando correctamente (se ha recibido recientemente)
    TIMEOUT         // Se recibió anteriormente pero no se ha actualizado en el tiempo establecido
};

// Estructura para almacenar la información de cada esclavo
typedef struct {
    uint8_t slaveID;                 // ID del esclavo (1-16)
    bool state;                      // Estado actual (ON/OFF)
    SlaveState connectionState;      // Estado de conexión
    unsigned long lastUpdateTime;    // Última vez que se recibió información
    uint32_t receiveCount;           // Contador de recepción de datos
} SlaveInfo;

// Array para almacenar información de todos los esclavos
SlaveInfo slaves[NUM_SLAVES];

// Variable para el último ID de esclavo recibido
uint8_t lastReceivedSlaveID = 0;

// Definiciones para la transmisión ASK según el estado del límite de potencia
const int PIN_TX_POWER_ALERT = 33;  // Pin para transmisión ASK

// Secuencias para los distintos estados
const uint8_t SECUENCIA_ALERTA_EXCESO[] = {1, 0, 0, 0, 0, 0, 0, 1};  // Secuencia 10000001 para límite excedido
const uint8_t SECUENCIA_ALERTA_NORMAL[] = {1, 0, 0, 0, 0, 1, 0, 1};  // Secuencia 10000101 para nivel normal

const int LONGITUD_SECUENCIA = 8;      // Ambas secuencias tienen la misma longitud
const int DURACION_BIT = 95;           // microsegundos
const int PAUSA_ENVIOS = 1000;         // microsegundos
const int DURACION_SYNC = 300;         // microsegundos para señal de sincronización
const int PAUSA_SYNC = 260;            // microsegundos de espera después de sincronización
const int INTERVALO_TRANSMISION = 50;  // microsegundos entre transmisiones
const int NUM_TRANSMISIONES = 800;       // Exactamente 100 transmisiones para cada secuencia


// Variables para rastrear el estado del transmisor
bool canalActivo = false;
unsigned long ultimaTransmisionAlerta = 0;
bool alertaTransmisionActiva = false;
bool estadoAnteriorLimite = false;
int contadorTransmisiones = 0;
unsigned long tiempoInicioExceso = 0;
bool esperandoTiempoMinimo = false;
bool transmitiendoSecuenciaExceso = false;
bool transmitiendoSecuenciaNormal = false;



// Variables for tracking sequence transmission and expected responses
unsigned long lastSequenceTransmissionTime = 0; 
bool expectingSlaveResponses = false;
const unsigned long SLAVE_RESPONSE_TIMEOUT = 90000; // 90 seconds timeout

// Variables para controlar respuestas de esclavos
bool slaveResponseReceived[NUM_SLAVES] = {false};
int totalResponsesThisCycle = 0;
int uniqueResponsesThisCycle = 0;

typedef struct {
  bool sequenceFound;
  uint8_t capturedBits[CAPTURE_LENGTH];
  uint8_t decodedBits[MAX_BITS];
  int bitCount;
  FlankInfo detectedFlanks[MAX_FLANKS];
  int flankCount;
} DetectionState;


void setup() {
  // Inicializar comunicación Serial
  Serial.begin(115200);
  delay(3000);
  
  // Inicializar arrays
  for (int i = 0; i < 50; i++) {
    powerData.arrayValues[i] = "";
    voltageData.arrayValues[i] = "";
    currentData.arrayValues[i] = "";
    harmonicsData.arrayValues[i] = "";
    frequencyData.arrayValues[i] = "";
    //askSignalData.arrayValues[i] = "";
  }
  
  Serial.println();
  Serial.println("ESP32 Monitor de Señal Digital ASK con Servidor Web y Cliente HTTP");
  Serial.println("------------------------------------------------------------");
  
  // Configurar GPIO para entrada digital
  pinMode(inputPin, INPUT);
  
  // Conectar a WiFi
  connectToWiFi();
  
  // Inicializar cliente NTP
  timeClient.begin();
  timeClient.setTimeOffset(7200); // Ajustar para tu zona horaria (2 horas = 7200 segundos)
  updateNTPTime(); // Tiempo inicial
  
  // Configurar rutas del servidor web
  setupWebServer();
  
  // Realizar login en el dispositivo externo
  performLogin();
  
  if (sessionCookie != "") {
    // Si el login fue exitoso, hacer la primera petición de estado inmediatamente
    getDeviceStatus();
    lastStatusRequestTime = millis();
  }

  
  Serial.println("Capturando datos automáticamente cada segundo...");
  lastCaptureTime = millis();

  initPowerAlertTransmitter();
  initSlaveSystem();
}

void loop() {
  // Manejar solicitudes de clientes web (siempre activo)
  server.handleClient();
  
  unsigned long currentTime = millis();
  
  // FASE 1: Lectura de datos web cada minuto exacto
  if (currentTime - lastStatusRequestTime >= statusRequestInterval) {
    Serial.println("\n======================================================");
    Serial.println("===== FASE 1: LECTURA DE DATOS WEB (CADA MINUTO) =====");
    Serial.println("======================================================");
    
    // Actualizar tiempo NTP
    updateNTPTime();
    
    // Consultar datos del dispositivo
    if (sessionCookie != "") {
      getDeviceStatus();
    } else {
      Serial.println("No hay cookie de sesión válida. Intentando login...");
      performLogin();
      if (sessionCookie != "") {
        getDeviceStatus();
      }
    }
    
    lastStatusRequestTime = currentTime;
    
    // FASE 2: Transmitir secuencia basada en el nivel de potencia
    Serial.println("\n=======================================================");
    Serial.println("===== FASE 2: TRANSMISIÓN DE SECUENCIA (10 VECES) =====");
    Serial.println("=======================================================");
    
    // Transmitir secuencia según el estado de potencia actual
    transmitirSecuenciaActual();
    
    
    // FASE 3: Cambiar a modo escucha y decodificación
    Serial.println("\n===================================================");
    Serial.println("===== FASE 3: ESCUCHA Y DECODIFICACIÓN DE ASK =====");
    Serial.println("===================================================");
    
    // Realizar capturas continuas hasta el próximo ciclo
    int numCapturasIniciales = 5; // Capturas iniciales más frecuentes
    
    // Primeras capturas más frecuentes para no perder respuestas inmediatas
    for (int i = 0; i < numCapturasIniciales; i++) {
      Serial.print("\nCaptura rápida #");
      Serial.print(i+1);
      Serial.print(" de ");
      Serial.println(numCapturasIniciales);
      
      // Capturar y analizar datos
      captureAndDisplayData();
      updateSlaveConnectionStates();
      
      // Espera breve entre capturas rápidas
      delay(200);
    }
    
    // Mostrar resumen del estado actual de los esclavos
    displaySlaveStatus();
  } else {
    // Durante el resto del tiempo, seguimos escuchando pero con menor frecuencia
    // Solo hacemos capturas cada 500ms para reducir carga de CPU
    static unsigned long lastCaptureTime = 0;
    
    if (currentTime - lastCaptureTime >= captureInterval) {
      captureAndDisplayData();
      updateSlaveConnectionStates();
      lastCaptureTime = currentTime;
    }
  }
  
  // Verificar entrada del usuario para alternar visualización
  if (Serial.available() > 0) {
    char input = Serial.read();
    
    if (input == 'a' || input == 'A') {
      showFullArray = !showFullArray;
      captureAndDisplayData();
    }
    else if (input == 's' || input == 'S') {
      // Mostrar estado de los esclavos
      displaySlaveStatus();
    }
  }
  
  yield(); // Permitir que ESP32 maneje tareas en segundo plano
}




// Inicializar la información de los esclavos
void initSlaveSystem() {
    for (int i = 0; i < NUM_SLAVES; i++) {
        slaves[i].slaveID = i + 1;           // IDs del 1 al 16
        slaves[i].state = false;             // Todos apagados inicialmente
        slaves[i].connectionState = DISCONNECTED;  // Inicialmente desconectados
        slaves[i].lastUpdateTime = 0;        // Nunca actualizados
        slaves[i].receiveCount = 0;          // Contador de recepciones a cero
    }
    Serial.println("Sistema de monitoreo de 16 esclavos inicializado");
}

void updateNTPTime() {
  if (WiFi.status() == WL_CONNECTED) {
    // Update NTP client
    timeClient.update();
    
    // Get raw epoch time (seconds since Jan 1, 1970)
    unsigned long epochTime = timeClient.getEpochTime();
    
    // Convert to time_t
    time_t rawTime = (time_t)epochTime;
    
    // Extract date and time components
    struct tm *timeInfo = localtime(&rawTime);
    
    // Format date string: YYYY-MM-DD
    char dateStr[11];
    sprintf(dateStr, "%04d-%02d-%02d", 
            timeInfo->tm_year + 1900, 
            timeInfo->tm_mon + 1, 
            timeInfo->tm_mday);
    dayStamp = String(dateStr);
    
    // Format time string: HH:MM (without seconds)
    char timeStr[6];
    sprintf(timeStr, "%02d:%02d", 
            timeInfo->tm_hour, 
            timeInfo->tm_min);
    timeStamp = String(timeStr);
    
    Serial.println("Current date: " + dayStamp);
    Serial.println("Current time: " + timeStamp);
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected successfully!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupWebServer() {
  // Define the routes
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/time", handleTime);
  
  // Start the server
  server.begin();
  Serial.println("Web server started");
}

void handleRoot() {
  String html = "<html><head>";
  html += "<title>ESP32 Power & Slave Monitor</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  // Estilos base mejorados
  html += "* { box-sizing: border-box; }";
  html += "body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 0; background-color: #f5f7fa; color: #333; }";
  html += ".wrapper { max-width: 1400px; margin: 0 auto; padding: 20px; }";
  html += "h1 { color: #2c3e50; margin-bottom: 25px; text-align: center; font-size: 28px; position: relative; padding-bottom: 10px; }";
  html += "h1:after { content: ''; position: absolute; bottom: 0; left: 50%; transform: translateX(-50%); width: 80px; height: 3px; background-color: #3498db; }";
  
  // Nuevo contenedor principal con diseño para lado izquierdo y derecho
  html += ".main-container { display: flex; gap: 30px; margin-bottom: 20px; }";
  html += ".left-panel { flex: 0 0 400px; display: flex; flex-direction: column; gap: 20px; }";
  html += ".right-panel { flex: 1; }";
  
  // Barra de última actualización que ocupa todo el ancho
  html += ".full-width-update { width: 100%; }";
  
  // Tarjetas mejoradas
  html += ".card { background-color: #ffffff; border-radius: 12px; padding: 20px; box-shadow: 0 5px 15px rgba(0,0,0,0.08); transition: transform 0.3s, box-shadow 0.3s; }";
  html += ".card:hover { transform: translateY(-5px); box-shadow: 0 8px 20px rgba(0,0,0,0.12); }";
  
  // Estilos para la tarjeta de datos de energía
  html += ".power-data-card { }";
  html += ".power-data-title { color: #2c3e50; font-size: 20px; margin-bottom: 20px; border-bottom: 2px solid #f0f0f0; padding-bottom: 10px; }";
  html += ".power-metric { display: flex; justify-content: space-between; padding: 15px 0; border-bottom: 1px solid #eee; }";
  html += ".power-metric:last-child { border-bottom: none; }";
  html += ".metric-label { font-weight: 600; color: #7f8c8d; }";
  html += ".metric-value { font-weight: 500; color: #2c3e50; display: flex; align-items: center; }";
  html += ".metric-value span:first-child { margin-right: 5px; }";
  html += ".metric-unit { color: #95a5a6; font-size: 14px; }";
  html += ".invalid-data { color: #e74c3c; font-style: italic; }";
  
  // Estilos para alertas
  html += ".warning { color: #e74c3c; font-weight: bold; padding: 12px; background-color: #fdecea; border-radius: 8px; border-left: 4px solid #e74c3c; }";
  html += ".normal { color: #27ae60; font-weight: bold; padding: 12px; background-color: #e8f5e9; border-radius: 8px; border-left: 4px solid #27ae60; }";
  html += ".info-blue { color: #3498db; font-weight: bold; padding: 12px; background-color: #e3f2fd; border-radius: 8px; border-left: 4px solid #3498db; }";
  
  // Tarjeta de tiempo mejorada
  html += ".time-card { }";
  html += ".time-card .icon { font-size: 24px; color: #3498db; margin-bottom: 10px; text-align: center; }";
  html += ".time-data { display: flex; justify-content: space-between; padding: 12px 0; border-bottom: 1px solid #eee; }";
  html += ".time-data:last-child { border-bottom: none; }";
  html += ".data-label { font-weight: 600; color: #7f8c8d; }";
  html += ".data-value { font-weight: 500; color: #2c3e50; }";
  
  // Estilos para la tarjeta de última actualización - ahora con diseño horizontal
  html += ".update-card { text-align: center; display: flex; align-items: center; justify-content: space-between; }";
  html += ".update-card .icon { font-size: 24px; color: #3498db; margin-right: 15px; }";
  html += ".update-main { display: flex; align-items: center; }";
  html += "#last-update { margin: 0; font-weight: 600; color: #34495e; margin-right: 20px; }";
  html += ".update-info { color: #7f8c8d; font-size: 14px; margin: 0 10px; }";
  html += ".update-details { display: flex; gap: 20px; }";
  
  // Cuadrícula de esclavos mejorada - 3 filas x 5 columnas con altura reducida
  html += ".slaves-section h2 { margin-bottom: 20px; color: #2c3e50; text-align: center; }";
  html += ".slave-grid { display: grid; grid-template-columns: repeat(5, 1fr); grid-template-rows: repeat(3, 1fr); gap: 20px; }";
  html += ".slave-box { background-color: #ffffff; padding: 16px; border-radius: 12px; box-shadow: 0 5px 15px rgba(0,0,0,0.08); transition: transform 0.3s; min-height: 170px; }"; // Reducido de 200px a 170px
  html += ".slave-box:hover { transform: translateY(-5px); box-shadow: 0 8px 20px rgba(0,0,0,0.12); }";
  html += ".slave-box h3 { margin-top: 0; margin-bottom: 12px; color: #2c3e50; font-size: 18px; border-bottom: 2px solid #f0f0f0; padding-bottom: 8px; text-align: center; }"; // Reducido margin-bottom y padding-bottom
  html += ".slave-data { margin-bottom: 10px; display: flex; justify-content: space-between; font-size: 15px; }"; // Reducido margin-bottom
  html += ".slave-data-label { font-weight: 600; color: #7f8c8d; }";
  html += ".slave-data-value { font-weight: 500; color: #2c3e50; }";
  html += ".slave-status { padding: 8px; text-align: center; border-radius: 6px; margin-top: 10px; font-weight: 600; font-size: 14px; }"; // Reducido padding y margin-top
  html += ".connected { background-color: #e8f8f5; color: #27ae60; }";
  html += ".disconnected { background-color: #f7f9fb; color: #95a5a6; }";
  html += ".timeout { background-color: #fdecea; color: #e74c3c; }";
  
  // Responsive design
  html += "@media (max-width: 1200px) {";
  html += "  .main-container { flex-direction: column; }";
  html += "  .left-panel { flex: none; max-width: none; }";
  html += "  .slave-grid { grid-template-columns: repeat(3, 1fr); grid-template-rows: repeat(5, 1fr); }";
  html += "  .update-card { flex-direction: column; text-align: center; }";
  html += "  .update-main { margin-bottom: 10px; }";
  html += "  .update-details { justify-content: center; }";
  html += "}";
  html += "@media (max-width: 768px) {";
  html += "  .slave-grid { grid-template-columns: repeat(2, 1fr); }";
  html += "  .slave-box { min-height: 150px; }"; // Aún más pequeño en móviles
  html += "}";
  html += "@media (max-width: 480px) {";
  html += "  .slave-grid { grid-template-columns: 1fr; }";
  html += "  .update-details { flex-direction: column; gap: 5px; }";
  html += "}";
  
  html += "</style>";
  
  // JavaScript para actualizaciones
  html += "<script>";
  
  // Función para actualizar la hora usando XMLHttpRequest
  html += "function updateTime() {";
  html += "  var xhr = new XMLHttpRequest();";
  html += "  xhr.open('GET', '/time', true);";
  html += "  xhr.onreadystatechange = function() {";
  html += "    if (xhr.readyState == 4 && xhr.status == 200) {";
  html += "      var data = JSON.parse(xhr.responseText);";
  html += "      document.getElementById('dateValue').textContent = data.date;";
  html += "      document.getElementById('timeValue').textContent = data.time;";
  html += "    }";
  html += "  };";
  html += "  xhr.send();";
  html += "}";
  
  // Función para actualizar estado y datos de potencia
  html += "function updateStatus() {";
  html += "  var xhr = new XMLHttpRequest();";
  html += "  xhr.open('GET', '/status', true);";
  html += "  xhr.onreadystatechange = function() {";
  html += "    if (xhr.readyState == 4 && xhr.status == 200) {";
  html += "      var data = JSON.parse(xhr.responseText);";
  html += "      var statusElement = document.getElementById('powerStatus');";
  html += "      statusElement.textContent = data.powerStatus.message;";
  html += "      statusElement.className = data.powerStatus.isOverLimit ? 'warning' : 'normal';";
  
  // Actualizar datos de potencia
  html += "      updatePowerMetric('powerValue', 'powerUnit', data.power);";
  html += "      updatePowerMetric('voltageValue', 'voltageUnit', data.voltage);";
  html += "      updatePowerMetric('currentValue', 'currentUnit', data.current);";
  html += "      document.getElementById('loopCount').textContent = data.loopCount;";

  // Actualizar datos de esclavos - solo del 1 al 15
  for (int i = 1; i <= 15; i++) {
    String slaveID = String(i);
    html += "      updateSlaveData('slave_" + slaveID + "', data.slave_" + slaveID + ");";
  }
  
  html += "      var now = new Date();";
  html += "      var hours = ('0' + now.getHours()).slice(-2);";
  html += "      var mins = ('0' + now.getMinutes()).slice(-2);";
  html += "      var secs = ('0' + now.getSeconds()).slice(-2);";
  html += "      var timeString = hours + ':' + mins + ':' + secs;";
  html += "      document.getElementById('last-update').textContent = 'Last update: ' + timeString;";
  html += "    }";
  html += "  };";
  html += "  xhr.send();";
  html += "}";
  
  // Función auxiliar para actualizar métricas de potencia
  html += "function updatePowerMetric(valueId, unitId, data) {";
  html += "  var valueElement = document.getElementById(valueId);";
  html += "  var unitElement = document.getElementById(unitId);";
  html += "  if (data.valid) {";
  html += "    valueElement.textContent = data.value;";
  html += "    valueElement.className = '';";
  html += "    unitElement.textContent = data.unit;";
  html += "  } else {";
  html += "    valueElement.textContent = 'N/A';";
  html += "    valueElement.className = 'invalid-data';";
  html += "    unitElement.textContent = '';";
  html += "  }";
  html += "}";
  
  // Nueva función para actualizar datos de esclavos
  html += "function updateSlaveData(slaveKey, data) {";
  html += "  if (!data) return;";
  
  html += "  var stateElement = document.getElementById(slaveKey + '_state');";
  html += "  var updateElement = document.getElementById(slaveKey + '_update');";
  html += "  var countElement = document.getElementById(slaveKey + '_count');";
  html += "  var statusElement = document.getElementById(slaveKey + '_status');";
  
  html += "  if (stateElement) stateElement.textContent = data.state;";
  html += "  if (updateElement) updateElement.textContent = data.update;";
  html += "  if (countElement) countElement.textContent = data.count;";
  
  html += "  if (statusElement) {";
  html += "    statusElement.textContent = data.status;";
  html += "    statusElement.className = 'slave-status';";
  html += "    if (data.status === 'Connected') {";
  html += "      statusElement.classList.add('connected');";
  html += "    } else if (data.status === 'Disconnected') {";
  html += "      statusElement.classList.add('disconnected');";
  html += "    } else if (data.status === 'Timeout') {";
  html += "      statusElement.classList.add('timeout');";
  html += "    }";
  html += "  }";
  html += "}";
  
  // Inicialización
  html += "window.onload = function() {";
  html += "  updateTime();";
  html += "  updateStatus();";
  html += "  setInterval(updateTime, 15000);"; // 15 segundos
  html += "  setInterval(updateStatus, 5000);"; // 5 segundos para actualización rápida
  html += "};";
  html += "</script>";
  html += "</head><body>";
  
  html += "<div class='wrapper'>";
  html += "<h1>ESP32 Power & Slave Monitor</h1>";
  
  // Contenedor principal con división izquierda/derecha
  html += "<div class='main-container'>";
  
  // PANEL IZQUIERDO - Datos de potencia, alerta y fecha/hora (SIN última actualización)
  html += "<div class='left-panel'>";
  
  // Tarjeta de datos de potencia
  html += "<div class='card power-data-card'>";
  html += "<h2 class='power-data-title'>Power Metrics</h2>";
  
  // Potencia
  html += "<div class='power-metric'>";
  html += "<span class='metric-label'>Power:</span>";
  html += "<div class='metric-value'>";
  html += "<span id='powerValue'>" + (powerData.valid ? powerData.value : "N/A") + "</span>";
  html += "<span id='powerUnit' class='metric-unit'>" + (powerData.valid ? powerData.unit : "") + "</span>";
  html += "</div>";
  html += "</div>";
  
  // Voltaje
  html += "<div class='power-metric'>";
  html += "<span class='metric-label'>Voltage:</span>";
  html += "<div class='metric-value'>";
  html += "<span id='voltageValue'>" + (voltageData.valid ? voltageData.value : "N/A") + "</span>";
  html += "<span id='voltageUnit' class='metric-unit'>" + (voltageData.valid ? voltageData.unit : "") + "</span>";
  html += "</div>";
  html += "</div>";
  
  // Corriente
  html += "<div class='power-metric'>";
  html += "<span class='metric-label'>Current:</span>";
  html += "<div class='metric-value'>";
  html += "<span id='currentValue'>" + (currentData.valid ? currentData.value : "N/A") + "</span>";
  html += "<span id='currentUnit' class='metric-unit'>" + (currentData.valid ? currentData.unit : "") + "</span>";
  html += "</div>";
  html += "</div>";
  
  
  // Violaciones de límite
  html += "<div class='power-metric'>";
  html += "<span class='metric-label'>Violations count:</span>";
  html += "<span class='metric-value'>" + String(powerLimitExceededCount) + "</span>";
  html += "</div>";
  
  html += "</div>"; // Fin de la tarjeta de datos de potencia
  
  // Tarjeta de alerta
  html += "<div class='card'>";
  String statusClass = isOverPowerLimit ? "warning" : "normal";
  html += "<div id='powerStatus' class='" + statusClass + "'>";
  html += powerStatusMessage;
  html += "</div>";
  html += "</div>";

  // Estadísticas del sistema
  html += "<div class='card'>";
  html += "<div class='info-blue'>";
  html += "Sent requests: <span id='loopCount'>" + String(successfulLoopCount) + "</span>";
  html += "</div>";
  html += "</div>";
  
  // Tarjeta de tiempo
  html += "<div class='card time-card'>";
  html += "<div class='icon'>&#128337;</div>"; // Icono de reloj
  html += "<div class='time-data'>";
  html += "<span class='data-label'>Date:</span>";
  html += "<span id='dateValue' class='data-value'>" + dayStamp + "</span>";
  html += "</div>";
  html += "<div class='time-data'>";
  html += "<span class='data-label'>Time:</span>";
  html += "<span id='timeValue' class='data-value'>" + timeStamp + "</span>";
  html += "</div>";
  html += "</div>";
    
  html += "</div>"; // Fin del panel izquierdo
  
  // PANEL DERECHO - Cuadrícula de esclavos 3x5
  html += "<div class='right-panel'>";
  html += "<div class='slaves-section'>";
  html += "<h2>Slave Monitoring System</h2>";
  html += "<div class='slave-grid'>";
  
  // Generar 15 cuadros de esclavo en cuadrícula 3x5
  for (int i = 0; i < 15; i++) {
    int slaveID = i + 1;
    String slaveKey = "slave_" + String(slaveID);
    
    // Determinar estado de conexión
    String connectionStateClass = "disconnected";
    String connectionStateText = "Disconnected";
    
    if (slaves[i].connectionState == CONNECTED) {
      connectionStateClass = "connected";
      connectionStateText = "Connected";
    } else if (slaves[i].connectionState == TIMEOUT) {
      connectionStateClass = "timeout";
      connectionStateText = "Timeout";
    }
    
    // Calcular tiempo desde la última actualización
    String lastUpdateText = "Never";
    if (slaves[i].lastUpdateTime > 0) {
      unsigned long elapsed = millis() - slaves[i].lastUpdateTime;
      if (elapsed < 60000) {
        lastUpdateText = String(elapsed / 1000) + "s";
      } else if (elapsed < 3600000) {
        lastUpdateText = String(elapsed / 60000) + "m";
      } else {
        lastUpdateText = String(elapsed / 3600000) + "h";
      }
    }
    
    html += "<div class='slave-box'>";
    html += "<h3>Slave " + String(slaveID) + "</h3>";
    
    html += "<div class='slave-data'>";
    html += "<span class='slave-data-label'>State:</span>";
    html += "<span class='slave-data-value' id='" + slaveKey + "_state'>" + String(slaves[i].state ? "ON" : "OFF") + "</span>";
    html += "</div>";
    
    html += "<div class='slave-data'>";
    html += "<span class='slave-data-label'>Update:</span>";
    html += "<span class='slave-data-value' id='" + slaveKey + "_update'>" + lastUpdateText + "</span>";
    html += "</div>";
    
    html += "<div class='slave-data'>";
    html += "<span class='slave-data-label'>Count:</span>";
    html += "<span class='slave-data-value' id='" + slaveKey + "_count'>" + String(slaves[i].receiveCount) + "</span>";
    html += "</div>";
    
    html += "<div class='slave-status " + connectionStateClass + "' id='" + slaveKey + "_status'>" + connectionStateText + "</div>";
    
    html += "</div>";
  }
  
  html += "</div>"; // Fin de slave-grid
  html += "</div>"; // Fin de slaves-section
  html += "</div>"; // Fin del panel derecho
  
  html += "</div>"; // Fin del contenedor principal
  
  // BARRA DE ÚLTIMA ACTUALIZACIÓN - Ahora ocupa todo el ancho
  html += "<div class='card update-card full-width-update'>";
  html += "<div class='update-main'>";
  html += "<div class='icon'>&#128472;</div>"; // Icono de actualización
  html += "<p id='last-update'>Last update: " + getTimeString() + "</p>";
  html += "</div>";
  html += "<div class='update-details'>";
  html += "<p class='update-info'>Data updates every 5s</p>";
  html += "<p class='update-info'>Time updates every 15s</p>";
  html += "</div>";
  html += "</div>";
  
  html += "</div>"; // Fin del wrapper
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}



void handleTime() {
  // Actualizar el tiempo NTP
  updateNTPTime();
  
  String json = "{";
  json += "\"date\": \"" + dayStamp + "\",";
  json += "\"time\": \"" + timeStamp + "\"";
  json += "}";
  
  server.send(200, "application/json", json);
}


void handleStatus() {
  String json = "{";
  json += "\"power\": {\"value\": \"" + (powerData.valid ? powerData.value : "N/A") + "\", \"unit\": \"" + powerData.unit + "\", \"valid\": " + (powerData.valid ? "true" : "false") + "},";
  json += "\"voltage\": {\"value\": \"" + (voltageData.valid ? voltageData.value : "N/A") + "\", \"unit\": \"" + voltageData.unit + "\", \"valid\": " + (voltageData.valid ? "true" : "false") + "},";
  json += "\"current\": {\"value\": \"" + (currentData.valid ? currentData.value : "N/A") + "\", \"unit\": \"" + currentData.unit + "\", \"valid\": " + (currentData.valid ? "true" : "false") + "},";
  json += "\"frequency\": {\"value\": \"" + (frequencyData.valid ? frequencyData.value : "N/A") + "\", \"unit\": \"" + frequencyData.unit + "\", \"valid\": " + (frequencyData.valid ? "true" : "false") + "},";
  json += "\"powerStatus\": {\"message\": \"" + powerStatusMessage + "\", \"isOverLimit\": " + (isOverPowerLimit ? "true" : "false") + ", \"violationCount\": " + String(powerLimitExceededCount) + "}";
  json += ",\"loopCount\": " + String(successfulLoopCount);

  // Añadir datos de los esclavos manteniendo el mismo formato que los datos existentes
  for (int i = 0; i < NUM_SLAVES; i++) {
    String slaveID = String(i + 1);
    
    // Convertir estado de conexión a texto en inglés
    String connectionStateText = "Disconnected";
    if (slaves[i].connectionState == CONNECTED) {
      connectionStateText = "Connected";
    } else if (slaves[i].connectionState == TIMEOUT) {
      connectionStateText = "Timeout";
    }
    
    // Calcular tiempo desde la última actualización
    String lastUpdateText = "Never";
    if (slaves[i].lastUpdateTime > 0) {
      unsigned long elapsed = millis() - slaves[i].lastUpdateTime;
      if (elapsed < 60000) {
        lastUpdateText = String(elapsed / 1000) + " sec";
      } else if (elapsed < 3600000) {
        lastUpdateText = String(elapsed / 60000) + " min";
      } else {
        lastUpdateText = String(elapsed / 3600000) + " hrs";
      }
    }
    
    json += ",\"slave_" + slaveID + "\": {\"id\": " + slaveID + ", \"state\": \"" + String(slaves[i].state ? "ON" : "OFF") + 
            "\", \"status\": \"" + connectionStateText + 
            "\", \"update\": \"" + lastUpdateText + 
            "\", \"count\": \"" + String(slaves[i].receiveCount) + 
            "\", \"valid\": true}";
  }

  json += ",\"date\": \"" + dayStamp + "\",";
  json += "\"time\": \"" + timeStamp + "\"";
  json += "}";
  
  server.send(200, "application/json", json);
}

String getTimeString() {
  unsigned long currentMillis = millis();
  unsigned long seconds = currentMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  seconds %= 60;
  minutes %= 60;
  hours %= 24;
  
  String timeString = "";
  if (hours < 10) timeString += "0";
  timeString += String(hours) + ":";
  if (minutes < 10) timeString += "0";
  timeString += String(minutes) + ":";
  if (seconds < 10) timeString += "0";
  timeString += String(seconds);
  
  return timeString;
}

void performLogin() {
  // Check WiFi connection status
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;
    
    Serial.println("Making login request...");
    
    // Define the headers we want to collect
    const size_t headerCount = 1;
    const char* headers[headerCount] = {"Set-Cookie"};
    http.collectHeaders(headers, headerCount);
    
    // Begin HTTP connection
    http.begin(client, loginUrl);
    
    // Add headers
    http.addHeader("Accept", "application/json");
    http.addHeader("Content-Type", "application/json");
    
    // Prepare JSON payload - USING ArduinoJson v5
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& jsonDoc = jsonBuffer.createObject();
    jsonDoc["Password"] = "admin";
    jsonDoc["UpdateLastLogin"] = true;
    jsonDoc["User"] = "admin";
    
    // Serialize JSON to string
    String jsonString;
    jsonDoc.printTo(jsonString);
    
    // Print request details
    Serial.println("Login Request URL: " + String(loginUrl));
    Serial.println("Request Body: " + jsonString);
    
    // Send the POST request
    int httpResponseCode = http.POST(jsonString);
    
    // Check response
    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      
      // Look specifically for the Set-Cookie header
      if (http.header("Set-Cookie") != "") {
        String cookieHeader = http.header("Set-Cookie");
        Serial.println("Found Set-Cookie header: " + cookieHeader);
        
        // Look for SESSION cookie
        int sessionStart = cookieHeader.indexOf("SESSION=");
        if (sessionStart >= 0) {
          int sessionEnd = cookieHeader.indexOf(";", sessionStart);
          
          if (sessionEnd == -1) {
            // No semicolon found, take the rest of the string
            sessionCookie = cookieHeader.substring(sessionStart);
          } else {
            sessionCookie = cookieHeader.substring(sessionStart, sessionEnd);
          }
          
          Serial.println("Extracted session cookie: " + sessionCookie);
        } else {
          Serial.println("SESSION cookie not found in Set-Cookie header");
        }
      } else {
        // Get the response body and print it
        String response = http.getString();
        Serial.println("Login Response Body:");
        Serial.println(response);
        
        Serial.println("No Set-Cookie header found!");
      }
    } else {
      Serial.print("Error on login request. Error code: ");
      Serial.println(httpResponseCode);
    }
    
    // Free resources
    http.end();
  } else {
    Serial.println("WiFi Disconnected. Reconnecting...");
    connectToWiFi();
  }
}

void getDeviceStatus() {
  // Check WiFi connection status
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;
    updateNTPTime();
    Serial.println("\nMaking status request...");
    
    // Begin HTTP connection
    http.begin(client, statusUrl);
    
    // Add headers
    http.addHeader("Accept", "application/json");
    http.addHeader("Content-Type", "application/json");
    
    // If we have a session cookie
    if (sessionCookie != "") {
      Serial.println("Using cookie: " + sessionCookie);
      http.addHeader("Cookie", sessionCookie);
    }
    
    // Use ArduinoJson v5
    StaticJsonBuffer<512> jsonBuffer;
    JsonObject& jsonDoc = jsonBuffer.createObject();
    jsonDoc["Analyzer"] = "F100A1009034E42413032302_CH1_1P";
    
    // Add Series - Add all possible measurement types
    JsonArray& series = jsonDoc.createNestedArray("Series");
    series.add("Active_Power");
    series.add("Reactive_Power");
    series.add("Apparent_Power");
    series.add("Voltage");
    series.add("Current");
    series.add("Voltage_Harmonics");
    
    // Create JSON string
    String jsonString;
    jsonDoc.printTo(jsonString);
    
    // Debug info
    Serial.println("Status Request URL: " + String(statusUrl));
    Serial.println("Request Body: " + jsonString);
    
    // Send request
    int httpResponseCode = http.POST(jsonString);
    
    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      
      String response = http.getString();
      
      // Parse to extract key values
      parseStatusResponse(response);
    } else {
      Serial.print("Error on status request. Error code: ");
      Serial.println(httpResponseCode);
      
      if (httpResponseCode == -1 || httpResponseCode == 401) {
        Serial.println("Session may have expired. Will login again.");
        sessionCookie = ""; // Force a new login
        performLogin();
      }
    }
    
    http.end();
  } else {
    Serial.println("WiFi Disconnected. Reconnecting...");
    connectToWiFi();
  }
}

void parseStatusResponse(String response) {
  // Reset all data validity flags
  powerData.valid = false;
  frequencyData.valid = false;
  ApparentPowerData.valid = false;
  voltageData.valid = false;
  currentData.valid = false;
  harmonicsData.valid = false;
  
  // Parse the JSON response - using v5 syntax
  DynamicJsonBuffer jsonBuffer(8192); // Increased size for complex data
  
  JsonObject& doc = jsonBuffer.parseObject(response);
  
  if (!doc.success()) {
    Serial.println("JSON parsing failed!");
    
    // Print part of the response to see what we're dealing with
    Serial.println("First 200 chars of response:");
    Serial.println(response.substring(0, 200));
    return;
  }
  
  Serial.println("\n======= DEVICE STATUS DATA =======");
  
  // Track what we found for diagnostic purposes
  bool foundPower = false;
  bool foundVoltage = false;
  bool foundHarmonics = false;

  
  if (doc.containsKey("Active_Power")) {
    foundPower = true;
    Serial.println("\n--- Active Power ---");
    
    // Store directly as string without conversion
    powerData.value = doc["Active_Power"].as<String>();
    powerData.unit = "W";
    powerData.valid = true;
    
    Serial.print("Value: ");
    Serial.print(powerData.value);
    Serial.print(" ");
    Serial.println(powerData.unit);
  }
  
  if (doc.containsKey("Voltage")) {
    foundVoltage = true;
    Serial.println("\n--- Voltage ---");
    
    // Store directly as string without conversion
    voltageData.value = doc["Voltage"].as<String>();
    voltageData.unit = "V";
    voltageData.valid = true;
    
    Serial.print("Value_1: ");
    Serial.print(voltageData.value);
    Serial.print(" ");
    Serial.println(voltageData.unit);
  }
  
  if (doc.containsKey("Current")) {
    Serial.println("\n--- Current ---");
    
    // Store directly as string without conversion
    currentData.value = doc["Current"].as<String>();
    currentData.unit = "A";
    currentData.valid = true;
    
    Serial.print("Value_1: ");
    Serial.print(currentData.value);
    Serial.print(" ");
    Serial.println(currentData.unit);
  }
  
  if (doc.containsKey("Frequency")) {
    Serial.println("\n--- Frequency ---");
    
    // Store directly as string without conversion
    frequencyData.value = doc["Frequency"].as<String>();
    frequencyData.unit = "Hz";
    frequencyData.valid = true;
    
    Serial.print("Value_1: ");
    Serial.print(frequencyData.value);
    Serial.print(" ");
    Serial.println(frequencyData.unit);
  }
  
  // Process Voltage Harmonics if present
  if (doc.containsKey("Voltage_Harmonics")) {
    foundHarmonics = true;
    Serial.println("\n--- VOLTAGE HARMONICS ---");
    
    // Check if it's an array
    if (doc["Voltage_Harmonics"].is<JsonArray>()) {
      JsonArray& harmonicsArray = doc["Voltage_Harmonics"].as<JsonArray>();
      
      Serial.print("Number of harmonics 1: ");
      Serial.println(harmonicsArray.size());
      
      // Store harmonics values (up to 50)
      harmonicsData.arraySize = min((int)harmonicsArray.size(), 50);
      harmonicsData.valid = (harmonicsData.arraySize > 0);
      harmonicsData.unit = "%";
      
      int count = 0;
      for (JsonArray::iterator it = harmonicsArray.begin(); it != harmonicsArray.end(); ++it) {
        if (count < 50) {
          // Store as string directly
          harmonicsData.arrayValues[count] = it->as<String>();
          
          if (count < 10) { // Only print first 10 for brevity
            Serial.print("Harmonic_1");
            Serial.print(count);
            Serial.print(": ");
            Serial.println(harmonicsData.arrayValues[count]);
          }
        }
        count++;
      }
    }
    // If it's not an array, attempt to extract any useful data
    else {
      Serial.println("Harmonics not in expected array format:");
      String harmonicsStr = "";
      doc["Voltage_Harmonics"].printTo(harmonicsStr);
      Serial.println(harmonicsStr);
    }
  }

  Serial.println(" ");
  
  float floatVoltage = stringToFloat(voltageData.value);
  voltageData.value = floatVoltage;
  Serial.println("Extracted voltage: " + String(floatVoltage));

  float floatCurrent = stringToFloat(currentData.value);
  currentData.value = floatCurrent;
  Serial.println("Extracted voltage: " + String(floatVoltage));

  float floatPower = stringToFloat(powerData.value);
  powerData.value = floatPower;
  Serial.println("Extracted power: " + String(floatPower));
  
  float floatFrequency = stringToFloat(frequencyData.value);
  frequencyData.value = floatFrequency;
  
  Serial.println("Extracted frequency: " + String(floatFrequency));
  
  Serial.println("==============================");

  // Call checkPowerLimit correctly
  if (powerData.valid) {
    checkPowerLimit(floatPower);
  }
}

float stringToFloat(String stringValue) {
  bool isNegative = false;
  bool decimalPointFound = false;
  float result = 0.0;
  float decimalMultiplier = 0.1;
  
  // Check if empty string
  if (stringValue.length() == 0) return 0.0;
  
  // Process each character
  for (unsigned int i = 0; i < stringValue.length(); i++) {
    char c = stringValue.charAt(i);
    
    // Handle negative sign at the beginning
    if (i == 0 && c == '-') {
      isNegative = true;
      continue;
    }
    
    // Handle decimal point
    if (c == '.') {
      decimalPointFound = true;
      continue;
    }
    
    // Handle digits
    if (c >= '0' && c <= '9') {
      int digit = c - '0';
      
      if (decimalPointFound) {
        // Add fractional part
        result = result + digit * decimalMultiplier;
        decimalMultiplier *= 0.1;
      } else {
        // Add to integer part
        result = result * 10.0 + digit;
      }
    } else {
      // Non-numeric character (besides decimal point or initial minus)
      // In a strict parser, you might want to return an error here
      continue;
    }
  }
  
  if (isNegative) {
    result = -result;
  }
  
  return result;
}

void checkPowerLimit(float powerValue) {
  previousPowerLimitState = isOverPowerLimit;
  
  if (powerValue > POWER_LIMIT_WATTS) {
    isOverPowerLimit = true;
    if (!previousPowerLimitState && isOverPowerLimit) {
      powerLimitExceededCount++;
      Serial.print("Power limit exceeded count: ");
      Serial.println(powerLimitExceededCount);
    }

    powerStatusMessage = "ALERT: Power limit exceeded! Current power: " + 
                        String(powerValue) + " W exceeds limit of " + 
                        String(POWER_LIMIT_WATTS) + " W";
    
    Serial.print("ALERT: Power limit exceeded! Current power: ");
    Serial.print(powerValue);
    Serial.print(" W exceeds limit of ");
    Serial.print(POWER_LIMIT_WATTS);
    Serial.println(" W");
  } else {
    isOverPowerLimit = false;
    powerStatusMessage = "Normal: Power consumption normal at " + 
                        String(powerValue) + " W (Limit: " + 
                        String(POWER_LIMIT_WATTS) + " W)";
    
    Serial.print("Normal: Power consumption normal at ");
    Serial.print(powerValue);
    Serial.print(" W (Limit: ");
    Serial.print(POWER_LIMIT_WATTS);
    Serial.println(" W)");
  }
}

// Función mejorada de decodificación de bits siguiendo las reglas de ASK
void decodeBitsFromSamples(DetectionState* state) {
    // Reiniciar contadores en el estado local
    state->bitCount = 0;
    state->flankCount = 0;
    
    // Calcular muestras por bit usando la frecuencia real medida
    float samplesPerBitFloat = samplingRate * 0.0001; // Muestras en 100μs con la tasa real
    int samplesPerBit = (int)(samplesPerBitFloat + 0.5); // Redondeo al entero más cercano
    
    Serial.println("\n--- DETECCIÓN DE FLANCOS ---");
    Serial.print("Muestras por bit (100μs): ");
    Serial.println(samplesPerBit);
    
    // Paso 1: Detectar todos los flancos ascendentes y descendentes en el vector
    for (int i = 15; i < maxSamples - 15; i++) {
        // Detección de flanco ascendente (de 0 a 1)
        if (i > 0 && samples[i-1] == 0 && samples[i] == 1) {
            // Verificar que las 15 muestras anteriores sean todas cero
            bool allPreviousZero = true;
            for (int j = i-15; j < i; j++) {
                if (samples[j] != 0) {
                    allPreviousZero = false;
                    break;
                }
            }
            
            // Si encontramos 15 ceros consecutivos antes del flanco, es válido
            if (allPreviousZero && state->flankCount < MAX_FLANKS) {
                state->detectedFlanks[state->flankCount].index = i;
                state->detectedFlanks[state->flankCount].isRising = true;
                state->flankCount++;
            }
        }
        
        // Detección de flanco descendente (de 1 a 0)
        if (i > 0 && samples[i-1] == 1 && samples[i] == 0) {
            // Verificar que las 15 muestras siguientes sean todas cero
            bool allFollowingZero = true;
            for (int j = i; j < i + 15 && j < maxSamples; j++) {
                if (samples[j] != 0) {
                    allFollowingZero = false;
                    break;
                }
            }
            
            // Si hay 15 ceros consecutivos después del flanco es considerado válido
            if (allFollowingZero && state->flankCount < MAX_FLANKS) {
                state->detectedFlanks[state->flankCount].index = i;
                state->detectedFlanks[state->flankCount].isRising = false;
                state->flankCount++;
            }
        }
    }
    
    // Ordenar los flancos por posición
    for (int i = 1; i < state->flankCount; i++) {
        FlankInfo key = state->detectedFlanks[i];
        int j = i - 1;
        while (j >= 0 && state->detectedFlanks[j].index > key.index) {
            state->detectedFlanks[j + 1] = state->detectedFlanks[j];
            j--;
        }
        state->detectedFlanks[j + 1] = key;
    }
    
    // Mostrar información sobre los flancos detectados
    if (state->flankCount > 0) {
        Serial.print("Flancos detectados: ");
        Serial.println(state->flankCount);
        
        for (int i = 0; i < state->flankCount; i++) {
            Serial.print("  Flanco #");
            Serial.print(i+1);
            Serial.print(": ");
            Serial.print(state->detectedFlanks[i].isRising ? "Ascendente" : "Descendente");
            Serial.print(" en muestra ");
            Serial.println(state->detectedFlanks[i].index);
        }
        
        // Verificar patrón correcto de flancos según modulación ASK
        bool validPattern = true;
        for (int i = 0; i < state->flankCount - 1; i++) {
            if ((state->detectedFlanks[i].isRising && state->detectedFlanks[i+1].isRising) || 
                (!state->detectedFlanks[i].isRising && !state->detectedFlanks[i+1].isRising)) {
                Serial.print("⚠️ Error en patrón de flancos: dos flancos consecutivos ");
                Serial.print(state->detectedFlanks[i].isRising ? "ascendentes" : "descendentes");
                Serial.println(" (no cumple con modulación ASK)");
                validPattern = false;
            }
        }
        
        if (validPattern) {
            Serial.println("✓ Secuencia de flancos alternados (ascendente-descendente) correcta");
        }
        
    } else {
        Serial.println("No se detectaron flancos válidos");
        return;
    }
    
    // Paso 2: Decodificar bits entre flancos siguiendo reglas de modulación ASK
    for (int f = 0; f < state->flankCount - 1; f++) {
        int startIndex = state->detectedFlanks[f].index;
        int endIndex = state->detectedFlanks[f+1].index;
        bool isRisingToFalling = state->detectedFlanks[f].isRising && !state->detectedFlanks[f+1].isRising;
        bool isFallingToRising = !state->detectedFlanks[f].isRising && state->detectedFlanks[f+1].isRising;
        
        // Calcular número teórico de bits entre los flancos (redondeado al entero más cercano)
        float bitsFloating = (float)(endIndex - startIndex) / samplesPerBit;
        int bitsInSegment = round(bitsFloating);
        
        Serial.print("\nEntre flanco ");
        Serial.print(state->detectedFlanks[f].isRising ? "ascendente" : "descendente");
        Serial.print(" y flanco ");
        Serial.print(state->detectedFlanks[f+1].isRising ? "ascendente" : "descendente");
        Serial.print(": ");
        Serial.print(endIndex - startIndex);
        Serial.print(" muestras (");
        Serial.print(bitsFloating, 2);
        Serial.print(" bits teóricos, redondeado a ");
        Serial.print(bitsInSegment);
        Serial.println(" bits)");
        
        // Verificar si cumple con las reglas de modulación ASK
        if (isRisingToFalling) {
            // Flanco ascendente y descendente, todos deben ser '1'
            Serial.println("  En modulación ASK: Todos estos bits deben ser '1'");
            
            // Agregar 'bitsInSegment' bits con valor '1'
            for (int b = 0; b < bitsInSegment && state->bitCount < MAX_BITS; b++) {
                state->decodedBits[state->bitCount++] = 1;
            }
        } 
        else if (isFallingToRising) {
            // Flanco descendente y ascendente, todos deben ser '0'
            Serial.println("  En modulación ASK: Todos estos bits deben ser '0'");
            
            // Agregar 'bitsInSegment' bits con valor '0'
            for (int b = 0; b < bitsInSegment && state->bitCount < MAX_BITS; b++) {
                state->decodedBits[state->bitCount++] = 0;
            }
        }
        else {
            // Este caso no debería ocurrir si verificamos el patrón correctamente
            Serial.println("⚠️ Patrón de flancos inconsistente, no se decodifican bits");
        }
    }
    
    // Mostrar información resumida de la decodificación
    Serial.print("\nTotal bits decodificados según reglas ASK: ");
    Serial.println(state->bitCount);
}


void captureAndDisplayData() {
  // Create local detection state (automatically initialized to zeros)
  DetectionState detState = {false, {0}, {0}, 0, {0}, 0};
  
  // Captura tiempo inicial
  startTime = micros();
  
  // Capturar datos directo del GPIO a alta velocidad
  for (int i = 0; i < maxSamples; i++) {
    // Leer el pin digital y guardar como 0 o 1
    samples[i] = digitalRead(inputPin);
    
    // Pequeña demora para controlar tasa de muestreo
    delayMicroseconds(1);
  }
  
  // Captura tiempo final
  endTime = micros();
  
  // Calcula tasa de muestreo real
  samplingRate = (float)maxSamples * 1000000.0 / (endTime - startTime);
  
  // Detectar flancos y decodificar bits con estado local
  decodeBitsFromSamples(&detState);

  if (detState.bitCount > 0) {
    // Buscar la secuencia 111000 y capturar los bits siguientes
    findSequenceAndCaptureBits(&detState);
  }
  
  // Limpiar la pantalla
  Serial.write(27);      Serial.print("[2J");   // Limpiar pantalla
  Serial.write(27);      Serial.print("[H");    // Mover a posición inicial
  
  // Mostrar encabezado e información de muestreo
  Serial.println("=== MONITOR DE SEÑAL ESP32 DIGITAL CON DECODIFICADOR ASK ===");
  Serial.print("Captura #: ");
  Serial.println(++captureCount);
  Serial.print("Tiempo: ");
  Serial.print(millis() / 1000.0, 1);
  Serial.println(" segundos");
  
  Serial.print("Tasa de muestreo: ");
  Serial.print(samplingRate / 1000.0, 2);
  Serial.println(" kHz");
  
  Serial.print("Duración de la captura: ");
  Serial.print((endTime - startTime) / 1000.0, 2);
  Serial.println(" ms");
  
  // Información de la señal
  Serial.println("\n--- ANÁLISIS DE LA SEÑAL ---");
  
  // Contar unos y ceros
  int zeroCount = 0, oneCount = 0;
  for (int i = 0; i < maxSamples; i++) {
    if (samples[i] == 0) zeroCount++;
    else oneCount++;
  }
  
  Serial.print("Ceros: ");
  Serial.print(zeroCount);
  Serial.print(" (");
  Serial.print((float)zeroCount / maxSamples * 100.0, 1);
  Serial.print("%)  Unos: ");
  Serial.print(oneCount);
  Serial.print(" (");
  Serial.print((float)oneCount / maxSamples * 100.0, 1);
  Serial.println("%)");
  
  // Mostrar bits decodificados
  displayDecodedBits(&detState);
  
  // Mostrar bits capturados y extraer información si se encontró la secuencia
  if (detState.sequenceFound) {
    displayCapturedBits(&detState);
    extractSequenceInfo(&detState);
  } else {
    Serial.println("\n--- DETECCIÓN DE SECUENCIA 111000 ---");
    Serial.println("❌ Secuencia 111000 no encontrada en los datos");
  }
  
  displayCommandHistory();
  updateSlaveConnectionStates();
  
  // Muestra datos numéricos (muestra reducida o completa)
  if (!showFullArray) {
    // Mostrar muestra reducida
    Serial.println("\n--- MUESTRAS SELECCIONADAS ---");
    int step = maxSamples / displaySamples;
    if (step < 1) step = 1;
    
    for (int i = 0; i < maxSamples; i += step) {
      Serial.print(i);
      Serial.print(": ");
      Serial.println(samples[i]);
    }
    
    Serial.println("\nPresiona 'a' para mostrar TODAS las muestras");
  } 
  else {
    // Mostrar array completo en formato [valor1, valor2, valor3, ...]
    Serial.println("\n--- ARRAY COMPLETO DE DATOS ---");
    
    // Abrimos el corchete inicial
    Serial.print("[");
    
    // Imprimimos todos los valores separados por comas
    for (int i = 0; i < maxSamples; i++) {
      Serial.print(samples[i]);
      
      // Si no es el último, añadir coma
      if (i < maxSamples - 1) {
        Serial.print(", ");
      }
      
      // Añadir un salto de línea cada 15 valores para mejor legibilidad
      if ((i + 1) % 15 == 0 && i < maxSamples - 1) {
        Serial.println();
      }
    }
    
    // Cerramos el corchete
    Serial.println("]");
    
    Serial.println("\nPresiona 'a' para mostrar vista resumida");
  }
}

void findSequenceAndCaptureBits(DetectionState* state) {
    state->sequenceFound = false;
    
    // Verificamos si tenemos suficientes bits para buscar la secuencia y capturar datos
    if (state->bitCount < SEQUENCE_LENGTH + CAPTURE_LENGTH) {
        Serial.println("\n--- DETECCIÓN DE SECUENCIA 111000 ---");
        Serial.println("⚠️ No hay suficientes bits para buscar la secuencia y capturar datos");
        return;
    }
    
    // Validación adicional: Verificar que hay suficientes bits no nulos
    int nonZeroBits = 0;
    for (int i = 0; i < state->bitCount; i++) {
        if (state->decodedBits[i] != 0) nonZeroBits++;
    }
    
    // Requiere un mínimo de bits significativos
    if (nonZeroBits < 3) {
        Serial.println("⚠️ Señal demasiado débil o sin datos significativos");
        return;
    }
    
    // Buscamos la secuencia 111000 en los bits decodificados
    for (int i = 0; i <= state->bitCount - SEQUENCE_LENGTH - CAPTURE_LENGTH; i++) {
        bool matchFound = true;
        
        // Comparamos cada bit de la secuencia objetivo
        for (int j = 0; j < SEQUENCE_LENGTH; j++) {
            if (state->decodedBits[i + j] != targetSequence[j]) {
                matchFound = false;
                break;
            }
        }
        
        // Si encontramos la secuencia, capturamos los 8 bits siguientes
        if (matchFound) {
            state->sequenceFound = true;
            
            // Almacenamos los 8 bits siguientes
            for (int k = 0; k < CAPTURE_LENGTH; k++) {
                state->capturedBits[k] = state->decodedBits[i + SEQUENCE_LENGTH + k];
            }
            
            // Salimos del bucle después de encontrar la primera ocurrencia
            break;
        }
    }
}



// Función para extraer y analizar la información de la secuencia capturada
void extractSequenceInfo(DetectionState* state) {
    Serial.println("\n--- ANÁLISIS DE DATOS CAPTURADOS ---");
    
    if (!state->sequenceFound) {
        Serial.println("❌ No hay datos válidos para analizar");
        return;
    }
    
    // Declarar todas las variables necesarias
    uint8_t startBit = state->capturedBits[0];
    uint8_t slaveID = 0;
    bool validID = false;
    uint8_t stateBit = 0;
    uint8_t parityBit = 0;
    uint8_t stopBit = 0;
    uint8_t calculatedParity = 0;
    int sumaBits = 0;
    
    // Extraer el ID del esclavo (4 bits)
    for (int i = 0; i < 4; i++) {
        slaveID |= (state->capturedBits[i+1] << (3-i));
    }
    
    // Comprobar si el ID es válido (1-16)
    validID = (slaveID >= 1 && slaveID <= NUM_SLAVES);
    
    // Extraer el bit de estado
    stateBit = state->capturedBits[5];
    
    // Extraer el bit de paridad
    parityBit = state->capturedBits[6];
    
    // Extraer el bit de parada
    stopBit = state->capturedBits[7];
    
    // Calcular la paridad (paridad par) de los bits de ID y estado
    for (int i = 0; i < 5; i++) {
      sumaBits += state->capturedBits[i];
    }
    sumaBits = sumaBits + state->capturedBits[7];
    calculatedParity = (sumaBits % 2 == 0) ? 0 : 1; // Paridad par
    
    // Mostrar la información extraída
    Serial.println("Estructura de trama: [Inicio(1)][ID-Esclavo(4)][Estado(1)][Paridad(1)][Parada(1)]");
    
    Serial.print("Bit de inicio: ");
    Serial.println(startBit ? "1 ✓" : "0 ❌");
    
    Serial.print("ID del esclavo: ");
    Serial.print(slaveID);
    
    if (validID) {
        Serial.println(" ✓ (ID válido)");
    } else {
        Serial.println(" ❌ (fuera del rango válido 1-16)");
    }
    
    Serial.print("Bit de estado: ");
    Serial.print(stateBit);
    Serial.println(stateBit ? " (ENCENDIDO)" : " (APAGADO)");
    
    Serial.print("Bit de paridad: ");
    Serial.print(parityBit);
    if (parityBit == 0) {
        Serial.println(" ✓ (paridad correcta)");
    } else {
        Serial.print(" ❌ (incorrecto, debería ser ");
        Serial.print(calculatedParity);
        Serial.println(")");
    }
    
    Serial.print("Bit de parada: ");
    Serial.println(stopBit ? "1 ✓" : "0 ❌");
    
    // Validación general de la trama
    bool validFrame = (startBit == 1) && 
                      validID &&
                      (parityBit == calculatedParity) &&
                      (stopBit == 1);
    
    Serial.println();
    if (!validFrame) {
        Serial.println("❌ TRAMA INVÁLIDA - Errores detectados");
        return; // No procesar más si la trama es inválida
    }

    Serial.println("✅ TRAMA VÁLIDA");
    Serial.print("Comando: Esclavo #");
    Serial.print(slaveID);
    Serial.print(" - ");
    Serial.println(stateBit ? "ENCENDER" : "APAGAR");
    
    // Actualizar la información del esclavo
    int index = slaveID - 1;  // Convertir ID (1-16) a índice (0-15)
    
    // Aumentar contador total sin importar si es duplicado
    totalResponsesThisCycle++;
    
    if (index >= 0 && index < NUM_SLAVES) {
        unsigned long currentTime = millis();
        
        // Verificar si es la primera respuesta en este ciclo
        bool firstResponseInCycle = !slaveResponseReceived[index];
        
        // SOLO incrementamos el contador si es la primera respuesta en este ciclo
        if (firstResponseInCycle) {
            // Incrementar contador de recepción SOLO UNA VEZ por ciclo
            slaves[index].receiveCount++;
            
            // Actualizar timestamp
            slaves[index].lastUpdateTime = currentTime;
            
            // Actualizar estado ON/OFF
            slaves[index].state = (stateBit == 1);
            
            // Marcar como respondido en este ciclo
            slaveResponseReceived[index] = true;
            
            if (expectingSlaveResponses) {
                uniqueResponsesThisCycle++;
                Serial.print("✓ Esclavo #");
                Serial.print(slaveID);
                Serial.println(" respondió a la secuencia (primera vez en este ciclo)");
            }
            
            // Si es la primera vez que se recibe, cambiar estado a CONNECTED
            if (slaves[index].connectionState == DISCONNECTED) {
                slaves[index].connectionState = CONNECTED;
                Serial.print("Esclavo #");
                Serial.print(slaveID);
                Serial.println(" conectado por primera vez");
            } 
            // Si estaba en timeout, volver a CONNECTED
            else if (slaves[index].connectionState == TIMEOUT) {
                slaves[index].connectionState = CONNECTED;
                Serial.print("Esclavo #");
                Serial.print(slaveID);
                Serial.println(" reconectado después de timeout");
            }
            
            // Actualizamos historial solo con la primera respuesta del ciclo
            SlaveData newCommand;
            newCommand.slaveID = slaveID;
            newCommand.slaveState = (stateBit == 1);
            newCommand.isValid = true;
            newCommand.timestamp = currentTime;
            
            // Guardar en el historial circular
            commandHistory[commandHistoryIndex] = newCommand;
            commandHistoryIndex = (commandHistoryIndex + 1) % MAX_STORED_COMMANDS;
            totalCommandsReceived++;
            
            // Actualizar último comando válido
            lastValidCommand = newCommand;
        } else {
            // Mensaje duplicado: NO actualizamos contador ni estado
            Serial.print("ℹ️ Mensaje duplicado del esclavo #");
            Serial.print(slaveID);
            Serial.println(" - Ya fue contabilizado en este ciclo");
        }
        
        lastReceivedSlaveID = slaveID;
        
        // Comprobar si todos los esclavos conectados han respondido
        if (expectingSlaveResponses) {
            bool allSlavesResponded = true;
            for (int i = 0; i < NUM_SLAVES; i++) {
                if (slaves[i].connectionState == CONNECTED && !slaveResponseReceived[i]) {
                    allSlavesResponded = false;
                    break;
                }
            }
            
            // Si todos los esclavos han respondido, podemos mostrar estadísticas
            if (allSlavesResponded) {
                expectingSlaveResponses = false;
                Serial.println("✓ Todos los esclavos conectados han respondido a la secuencia");
                
                // Mostrar estadísticas de respuestas
                displayResponseStats();
            }
        }
    }
    else {
        Serial.print("ℹ️ ID de esclavo fuera de rango: #");
        Serial.println(slaveID);
    }
}


// ==================== Representación en pantalla de los datos extraidos ===========================

// Visualizar la función displayDecodedBits para mostrar los bits según ASK
void displayDecodedBits(DetectionState* state) {
    Serial.println("\n--- BITS DECODIFICADOS (MODULACIÓN ASK) ---");
    
    if (state->bitCount > 0) {
        // Mostrar bits como array binario
        Serial.print("Bits: [");
        for (int i = 0; i < state->bitCount; i++) {
            Serial.print(state->decodedBits[i]);
            if (i < state->bitCount - 1) Serial.print(", ");
            
            // Nueva línea cada 20 bits para legibilidad
            if ((i + 1) % 20 == 0 && i < state->bitCount - 1) {
                Serial.println();
                Serial.print("       ");
            }
        }
        Serial.println("]");
        
        // Mostrar bits como valor hexadecimal para facilitar interpretación
        Serial.print("Hex: ");
        for (int i = 0; i < state->bitCount; i += 8) {
            uint8_t byteVal = 0;
            // Convertir grupos de 8 bits a bytes (si hay suficientes)
            for (int b = 0; b < 8 && (i + b) < state->bitCount; b++) {
                byteVal |= (state->decodedBits[i + b] << (7 - b));
            }
            Serial.print("0x");
            if (byteVal < 16) Serial.print("0"); // Asegurar dos dígitos
            Serial.print(byteVal, HEX);
            Serial.print(" ");
        }
        Serial.println();
    } else {
        Serial.println("No se decodificaron bits");
    }
    
    Serial.println();
}


void displayResponseStats() {
    Serial.println("\n--- ESTADÍSTICAS DE RESPUESTAS DEL CICLO ACTUAL ---");
    Serial.print("Respuestas totales recibidas: ");
    Serial.println(totalResponsesThisCycle);
    Serial.print("Esclavos únicos que respondieron: ");
    Serial.println(uniqueResponsesThisCycle);
    Serial.print("Respuestas duplicadas ignoradas: ");
    Serial.println(totalResponsesThisCycle - uniqueResponsesThisCycle);
    
    Serial.println("\nEstado de respuestas por esclavo:");
    int respondedCount = 0;
    for (int i = 0; i < NUM_SLAVES; i++) {
        if (slaves[i].connectionState == CONNECTED || slaves[i].connectionState == TIMEOUT) {
            Serial.print("Esclavo #");
            Serial.print(i + 1);
            Serial.print(": ");
            if (slaveResponseReceived[i]) {
                Serial.println("✓ Respondió y actualizado en web");
                respondedCount++;
            } else {
                Serial.println("⚠️ No respondió en este ciclo");
            }
        }
    }
    
    Serial.print("\n→ ");
    Serial.print(respondedCount);
    Serial.print(" de ");
    Serial.print(uniqueResponsesThisCycle);
    Serial.println(" esclavos conectados respondieron");
}

// Mostrar el estado de todos los esclavos
void displaySlaveStatus() {
    Serial.println("\n--- ESTADO DE LOS 16 ESCLAVOS ---");
    Serial.println("ID | Estado | Conexión | Última actualización | Recibidos");
    Serial.println("---------------------------------------------------------");
    
    unsigned long currentTime = millis();
    
    for (int i = 0; i < NUM_SLAVES; i++) {
        // ID del esclavo (formato de 2 dígitos)
        Serial.print(i + 1);
        if (i < 9) Serial.print(" ");
        Serial.print(" | ");
        
        // Estado ON/OFF
        Serial.print(slaves[i].state ? "ON " : "OFF");
        Serial.print(" | ");
        
        // Estado de conexión
        switch (slaves[i].connectionState) {
            case DISCONNECTED:
                Serial.print("Desconectado ");
                break;
            case CONNECTED:
                Serial.print("Correcto    ");
                break;
            case TIMEOUT:
                Serial.print("Timeout     ");
                break;
        }
        Serial.print("| ");
        
        // Tiempo desde la última actualización
        if (slaves[i].lastUpdateTime > 0) {
            unsigned long elapsed = currentTime - slaves[i].lastUpdateTime;
            if (elapsed < 1000) {
                Serial.print(elapsed);
                Serial.print(" ms        ");
            } else if (elapsed < 60000) {
                Serial.print(elapsed / 1000);
                Serial.print(" seg       ");
            } else if (elapsed < 3600000) {
                Serial.print(elapsed / 60000);
                Serial.print(" min       ");
            } else {
                Serial.print(elapsed / 3600000);
                Serial.print(" horas     ");
            }
        } else {
            Serial.print("Nunca           ");
        }
        Serial.print("| ");
        
        // Contador de recepciones
        Serial.print(slaves[i].receiveCount);
        
        Serial.println();
    }
    
    if (lastReceivedSlaveID > 0 && lastReceivedSlaveID <= NUM_SLAVES) {
        Serial.print("\nÚltimo esclavo actualizado: #");
        Serial.println(lastReceivedSlaveID);
    }
}

// Función para mostrar el historial de comandos recibidos
void displayCommandHistory() {
    Serial.println("\n--- HISTORIAL DE COMANDOS ---");
    
    if (totalCommandsReceived == 0) {
        Serial.println("No se han recibido comandos");
        return;
    }
    
    int displayCount = min(totalCommandsReceived, MAX_STORED_COMMANDS);
    Serial.print("Mostrando los últimos ");
    Serial.print(displayCount);
    Serial.println(" comandos:");
    
    // Determinar el índice de inicio para mostrar desde el comando más antiguo al más reciente
    int startIndex;
    if (totalCommandsReceived <= MAX_STORED_COMMANDS) {
        startIndex = 0;
    } else {
        startIndex = commandHistoryIndex;
    }
    
    for (int i = 0; i < displayCount; i++) {
        int idx = (startIndex + i) % MAX_STORED_COMMANDS;
        
        Serial.print(i + 1);
        Serial.print(". Esclavo: ");
        Serial.print(commandHistory[idx].slaveID);
        Serial.print(" | Estado: ");
        Serial.print(commandHistory[idx].slaveState ? "ENCENDIDO" : "APAGADO");
        Serial.print(" | Válido: ");
        Serial.print(commandHistory[idx].isValid ? "Sí" : "No");
        Serial.print(" | Tiempo: ");
        
        // Mostrar tiempo relativo
        unsigned long elapsed = millis() - commandHistory[idx].timestamp;
        if (elapsed < 1000) {
            Serial.print(elapsed);
            Serial.println(" ms atrás");
        } else if (elapsed < 60000) {
            Serial.print(elapsed / 1000);
            Serial.println(" s atrás");
        } else {
            Serial.print(elapsed / 60000);
            Serial.print(" min ");
            Serial.print((elapsed % 60000) / 1000);
            Serial.println(" s atrás");
        }
    }
    
    Serial.println("\n--- ÚLTIMO COMANDO VÁLIDO ---");
    if (lastValidCommand.isValid) {
        Serial.print("Esclavo #");
        Serial.print(lastValidCommand.slaveID);
        Serial.print(" - ");
        Serial.println(lastValidCommand.slaveState ? "ENCENDIDO" : "APAGADO");
        
        // Tiempo desde el último comando válido
        unsigned long elapsed = millis() - lastValidCommand.timestamp;
        Serial.print("Recibido hace: ");
        if (elapsed < 1000) {
            Serial.print(elapsed);
            Serial.println(" ms");
        } else if (elapsed < 60000) {
            Serial.print(elapsed / 1000);
            Serial.println(" segundos");
        } else {
            Serial.print(elapsed / 60000);
            Serial.print(" minutos ");
            Serial.print((elapsed % 60000) / 1000);
            Serial.println(" segundos");
        }
    } else {
        Serial.println("No se ha recibido ningún comando válido todavía");
    }
}

void displayCapturedBits(DetectionState* state) {
    Serial.println("\n--- DETECCIÓN DE SECUENCIA 111000 ---");
    
    if (!state->sequenceFound) {
        Serial.println("❌ Secuencia 111000 no encontrada en los datos");
        return;
    }
    
    // Mostramos los bits capturados
    Serial.println("✅ Secuencia 111000 encontrada!");
    Serial.print("8 bits capturados: [");
    for (int i = 0; i < CAPTURE_LENGTH; i++) {
        Serial.print(state->capturedBits[i]);
        if (i < CAPTURE_LENGTH - 1) Serial.print(", ");
    }
    Serial.println("]");
    
    // Convertimos los bits a un valor decimal
    uint16_t decimalValue = 0;
    for (int i = 0; i < CAPTURE_LENGTH; i++) {
        decimalValue |= (state->capturedBits[i] << (CAPTURE_LENGTH - 1 - i));
    }
    
    // Mostramos el valor en diferentes formatos
    Serial.print("Valor decimal: ");
    Serial.println(decimalValue);
    
    Serial.print("Valor hexadecimal: 0x");
    if (decimalValue < 0x10) Serial.print("0");
    if (decimalValue < 0x100) Serial.print("0");
    Serial.println(decimalValue, HEX);
}


//======================== Actualizar el estado de conexión de los esclavos =========================
void updateSlaveConnectionStates() {
    unsigned long currentTime = millis();
    
    // Si hemos enviado una secuencia y estamos esperando respuestas
    if (expectingSlaveResponses) {
        // Comprobar si han pasado 90 segundos desde el envío de la secuencia
        if (currentTime - lastSequenceTransmissionTime > SLAVE_RESPONSE_TIMEOUT) {
            // Mostrar estadísticas finales del ciclo
            Serial.println("\n--- FIN DEL CICLO DE RESPUESTAS (TIMEOUT) ---");
            displayResponseStats();
            
            // Recorrer cada esclavo
            for (int i = 0; i < NUM_SLAVES; i++) {
                // Si un esclavo estaba conectado pero no ha respondido desde que enviamos la secuencia
                if (slaves[i].connectionState == CONNECTED && !slaveResponseReceived[i]) {
                    // Establecer en timeout
                    slaves[i].connectionState = TIMEOUT;
                    Serial.print("Esclavo #");
                    Serial.print(i + 1);
                    Serial.println(" en TIMEOUT por falta de respuesta");
                }
            }
            
            // Resetear la bandera y contadores
            expectingSlaveResponses = false;
            totalResponsesThisCycle = 0;
            uniqueResponsesThisCycle = 0;
        }
    }
    
    // Comprobación de timeout original (se puede mantener como respaldo)
    for (int i = 0; i < NUM_SLAVES; i++) {
        // Si el esclavo está conectado pero no se ha actualizado durante TIMEOUT_MS
        if (slaves[i].connectionState == CONNECTED && 
            (currentTime - slaves[i].lastUpdateTime) > TIMEOUT_MS) {
            
            slaves[i].connectionState = TIMEOUT;
            Serial.print("Esclavo #");
            Serial.print(i + 1);
            Serial.println(" en TIMEOUT por falta de actualización general");
        }
    }
}


// =================== Función para inicializar el transmisor ASK =======================
void initPowerAlertTransmitter() {
  pinMode(PIN_TX_POWER_ALERT, OUTPUT);
  digitalWrite(PIN_TX_POWER_ALERT, LOW);

  // Configuración del temporizador LEDC para 100kHz
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_1_BIT,  // 1 bit para obtener 50% duty cycle
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 100000,  // 100 kHz exactamente
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
    .gpio_num = PIN_TX_POWER_ALERT,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 1,  // 50% duty cycle con resolución de 1 bit
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel);

  // Detener el canal completamente en lugar de solo pausar
  ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
  
  Serial.println("Transmisor de alerta de potencia inicializado en el pin " + String(PIN_TX_POWER_ALERT));
}

void activarPortadora() {
  if (!canalActivo) {
    // Configuramos y arrancamos el canal nuevamente
    ledc_channel_config_t ledc_channel = {
      .gpio_num = PIN_TX_POWER_ALERT,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .timer_sel = LEDC_TIMER_0,
      .duty = 1,  // 50% duty cycle con resolución de 1 bit
      .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
    canalActivo = true;
  } else {
    // Si ya está activo, solo resumimos el timer
    ledc_timer_resume(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);
  }
}

void desactivarPortadora() {
  // Detener completamente el canal y forzar nivel bajo
  ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
  canalActivo = false;
}

// Función para enviar la señal de sincronización
void enviarSincronizacion() {
  // Activamos la portadora durante DURACION_SYNC microsegundos
  activarPortadora();
  delayMicroseconds(DURACION_SYNC);
  
  // Desactivamos la portadora durante PAUSA_SYNC microsegundos
  desactivarPortadora();
  delayMicroseconds(PAUSA_SYNC);
}

// Función para enviar una secuencia específica
void enviarSecuencia(const uint8_t* secuencia) {
  // Enviar señal de sincronización inicial
  enviarSincronizacion();
  
  // Transmitir la secuencia de datos
  for (int i = 0; i < LONGITUD_SECUENCIA; i++) {
    if (secuencia[i] == 1) {
      activarPortadora();
      delayMicroseconds(DURACION_BIT);
    } else {
      desactivarPortadora();
      delayMicroseconds(DURACION_BIT);
    }
  }

  // Asegurar que el pin esté en LOW después de cada transmisión
  desactivarPortadora();
  pinMode(PIN_TX_POWER_ALERT, OUTPUT);
  digitalWrite(PIN_TX_POWER_ALERT, LOW);

  // Pausa entre transmisiones
  delayMicroseconds(PAUSA_ENVIOS);

  // Record that we sent a sequence and are expecting responses
  lastSequenceTransmissionTime = millis();
  expectingSlaveResponses = true;  
}


void monitorearYTransmitirAlertaPotencia() {
  if (isOverPowerLimit != estadoAnteriorLimite) {
    if (isOverPowerLimit) {
      Serial.println("⚠️ ALERTA: Límite de potencia excedido");
      powerLimitExceededCount++;
    } else {
      Serial.println("✓ Potencia volvió a nivel normal");
    }
    
    // No actualizar estadoAnteriorLimite aquí
    // Eso lo hace la función transmitirSecuenciaActual()
  }
}



void transmitirSecuenciaActual() {
  // Resetear contadores de estadísticas
  totalResponsesThisCycle = 0;
  uniqueResponsesThisCycle = 0;
  
  // Resetear el array de respuestas recibidas al inicio de un nuevo ciclo
  for (int i = 0; i < NUM_SLAVES; i++) {
    slaveResponseReceived[i] = false;
  }
  
  if (isOverPowerLimit) {
    Serial.println("⚠️ EXCESO DE POTENCIA - Transmitiendo secuencia 10000001 (desconexión)");
    
    for (int i = 0; i < NUM_TRANSMISIONES; i++) {
      enviarSecuencia(SECUENCIA_ALERTA_EXCESO);
      delayMicroseconds(INTERVALO_TRANSMISION);
      
      if (i % 2 == 0) {
        Serial.print(".");
      }
    }
    
    Serial.println("\n✓ Transmisión de desconexión completada");
  } else {
    Serial.println("✓ POTENCIA NORMAL - Transmitiendo secuencia 10000101 (conexión)");
    
    for (int i = 0; i < NUM_TRANSMISIONES; i++) {
      enviarSecuencia(SECUENCIA_ALERTA_NORMAL);
      delayMicroseconds(INTERVALO_TRANSMISION);
      
      if (i % 2 == 0) {
        Serial.print(".");
      }
    }
    
    Serial.println("\n✓ Transmisión de conexión completada");
  }
  
  lastSequenceTransmissionTime = millis();
  expectingSlaveResponses = true;
  estadoAnteriorLimite = isOverPowerLimit;
  successfulLoopCount++;
}
