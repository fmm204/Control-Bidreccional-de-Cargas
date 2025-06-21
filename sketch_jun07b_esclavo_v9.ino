#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include "driver/i2s.h"
#include "driver/adc.h"
#include "soc/i2s_reg.h"
#include "driver/ledc.h"

// ==================== PIN DEFINITIONS ====================
#define AP_TRIGGER_PIN 27
#define OUTPUT_PIN 25    // Pin que controlará la salida basada en la secuencia detectada
#define PIN_TX 33        // Pin para transmisión ASK

// ==================== TRANSMITTER CONFIGURATION ====================
const int DURACION_BIT = 95;       // microsegundos
const int PAUSA_ENVIOS = 100;     // microsegundos
const int DURACION_SYNC = 300;     // microsegundos para señal de sincronización
const int PAUSA_SYNC = 260;        // microsegundos de espera después de sincronización
const int NUM_REPETICIONES = 800;    // Número de veces a transmitir la secuencia
bool canalActivo = false;          // Estado del canal LEDC
uint8_t secuenciaTransmision[8];  // Array para los 12 bits a transmitir

// ==================== SEQUENCE CONTROL ====================
#define TARGET_SEQUENCE_LOW 0x81   // Hexadecimal para "1000 0000 0001" - Pone el pin a LOW
#define TARGET_SEQUENCE_HIGH 0x85  // Hexadecimal para "1000 0000 0101" - Pone el pin a HIGH
uint16_t lastDetectedValue = 0;
bool manualPinControl = false;

// ==================== NETWORK CONFIGURATION ====================
const char* ap_ssid = "ESP32_Config";
const char* ap_password = "12345678";
IPAddress ap_local_ip(192, 168, 254, 254);
IPAddress ap_gateway(192, 168, 254, 254);
IPAddress ap_subnet(255, 255, 255, 0);

// ==================== WEB SERVER ====================
WebServer server(80);

// ==================== AUTHENTICATION ====================
bool isAuthenticated = false;
const char* valid_username = "admin";
const char* valid_password = "admin";

// ==================== EEPROM CONFIGURATION ====================
#define EEPROM_SIZE 10
#define SLAVE_ADDRESS_ADDR 0

// ==================== SLAVE NUMBER CONSTRAINTS ====================
#define MIN_SLAVE_VALUE 1
#define MAX_SLAVE_VALUE 15

// ==================== STATE VARIABLES ====================
uint8_t slaveNumber = 1;
bool apStarted = false;
int lastPinState = LOW;

// ==================== SIGNAL DETECTION VARIABLES ====================

// Variables for sequence detection and data storage
#define SEQUENCE_LENGTH 6
#define CAPTURE_LENGTH 8
const uint8_t targetSequence[SEQUENCE_LENGTH] = {1, 1, 1, 0, 0, 0}; // Sequence to detect: 111000
uint8_t capturedBits[CAPTURE_LENGTH];  // To store 12 bits after the sequence
bool sequenceFound = false; // Indicator if sequence was found

// ADC pin and samples configuration
const int maxSamples = 8000;   // Number of samples to capture per cycle
const int displaySamples = 80; // Number of samples to display on screen

// Variables for flanks
int risingFlankIndex = -1;   // Index of rising flank
int fallingFlankIndex = -1;  // Index of falling flank
int bitsInSignal = 0;        // Number of bits calculated based on time

// I2S ADC configuration
#define I2S_NUM           I2S_NUM_0   // I2S controller to use (0 or 1)
#define I2S_SAMPLE_RATE   1000000     // 1MHz - sample rate
#define DMA_BUF_COUNT     8           // Number of DMA buffers
#define DMA_BUF_LEN       1024        // Size of each DMA buffer

// Parameters for bit decoding
#define US_PER_BIT 100           // Microseconds per bit (100us)
#define MAX_BITS 100             // Maximum number of bits to decode

// Data processing
bool useFilteredData = true;     // Control to use filtered or raw data

// Structure to store information about detected flanks
#define MAX_FLANKS 50  // Maximum number of flanks to detect

typedef struct {
    int index;         // Position of flank in samples array
    bool isRising;     // True if rising, false if falling
} FlankInfo;

// Variables for multiple flanks
FlankInfo detectedFlanks[MAX_FLANKS];
int flankCount = 0;            // Counter for detected flanks

// Variables for storing data
unsigned long lastCaptureTime = 0;
int captureCount = 0;        // Capture counter
bool showFullArray = false;  // Control to show complete array

// Variables for analysis
unsigned long startTime;
unsigned long endTime;
float samplingRate;

// Variables for bit decoding
uint8_t decodedBits[MAX_BITS];    // To store decoded bits
int bitCount = 0;                // Counter for decoded bits



// Add these state machine variables to the global variables section

// Sequence detection and response state machine
enum ResponseState {
  STATE_MONITORING,           // Default state: monitoring for sequences with ADC active
  STATE_WAITING_TO_TRANSMIT,  // Waiting for the slave-based delay (ADC disabled)
  STATE_TRANSMITTING          // Actively transmitting the response (ADC disabled)
};

ResponseState currentState = STATE_MONITORING;
unsigned long detectionTime = 0;        // When the sequence was detected
unsigned long transmissionDelay = 0;    // How long to wait before transmitting
uint8_t detectedSequenceType = 0;       // 1 = LOW sequence (0x81), 2 = HIGH sequence (0x85)



typedef struct {
  bool sequenceFound;
  uint8_t capturedBits[CAPTURE_LENGTH];
  uint8_t decodedBits[MAX_BITS];
  int bitCount;
  FlankInfo detectedFlanks[MAX_FLANKS];
  int flankCount;
} DetectionState;


// Replace I2S and ADC-related definitions
#define INPUT_PIN 34        // GPIO34 for digital input instead of ADC

// Variables for storing digital data (replace analog versions)
uint8_t samples[maxSamples];   // Store only 0 or 1 (digital)
unsigned long captureInterval = 500; // Milliseconds between captures

// Update detection thresholds for digital signals
#define FLANK_THRESHOLD 1     // Threshold for detecting a change in digital signal
#define BIT_THRESHOLD 1       // Threshold for determining if a bit is 1 or 0 (in digital)



// ==================== SETUP FUNCTION ====================
void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(AP_TRIGGER_PIN, INPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, HIGH);  // Initially HIGH
  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, LOW);
  
  // Set input pin as digital input (replace I2S initialization)
  pinMode(INPUT_PIN, INPUT);
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load saved slave number
  slaveNumber = EEPROM.read(SLAVE_ADDRESS_ADDR);
  if (slaveNumber < MIN_SLAVE_VALUE || slaveNumber > MAX_SLAVE_VALUE) {
    slaveNumber = MIN_SLAVE_VALUE;  // Default value if not valid
    EEPROM.write(SLAVE_ADDRESS_ADDR, slaveNumber);
    EEPROM.commit();
  }
  
  Serial.println("ESP32 initialized");
  Serial.print("Current Slave Number (Decimal): ");
  Serial.println(slaveNumber);
  Serial.print("Current Slave Number (Binary): ");
  printBinary(slaveNumber);
  
  Serial.println("Set pin 27 HIGH to enable configuration mode");
  Serial.println("Pin 25 control sequences:");
  Serial.println("- 1 0000 001 (0x81): Sets pin 25 to LOW");
  Serial.println("- 1 0000 101 (0x85): Sets pin 25 to HIGH");

  // Configure ASK transmitter
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_1_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 100000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
    .gpio_num = PIN_TX,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 1,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel);

  // Detener el canal completamente
  ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);

  Serial.println("\n\nESP32 - Power Monitor & Signal Analyzer");
  Serial.println("----------------------------------------");
  Serial.println("Set pin 27 to HIGH to enter configuration mode");
  Serial.println("In signal monitor mode:");
  Serial.println("Press 'a' to show/hide complete array");
  Serial.println("Pin 33: ASK transmitter output");
  
  delay(1000);
  lastCaptureTime = millis();
  
  // Generate initial transmission sequence
  //generarSecuenciaTransmision();
}

// ==================== MAIN LOOP ====================
void loop() {
  // Verify configuration mode changes (AP)
  int currentPinState = digitalRead(AP_TRIGGER_PIN);
  
  if (currentPinState != lastPinState) {
    if (currentPinState == HIGH && !apStarted) {
      startAP();
      apStarted = true;
      Serial.println("Configuration mode activated (Pin 27 HIGH)");
    } else if (currentPinState == LOW && apStarted) {
      stopAP();
      apStarted = false;
      Serial.println("Configuration mode deactivated (Pin 27 LOW)");
      generarSecuenciaTransmision();
    }
    lastPinState = currentPinState;
    delay(50); // Debounce
  }
  
  // Manage web server if in configuration mode
  if (apStarted) {
    server.handleClient();
    return; // Skip the rest of the loop in configuration mode
  }
  
  // NORMAL MODE: Operation cycle
  unsigned long currentTime = millis();
  
  // Only capture data if enough time has passed
  if (currentTime - lastCaptureTime >= captureInterval) {
    captureAndDisplayData();
    lastCaptureTime = currentTime;
    captureCount++;
  }
  
  // If a valid sequence was detected, respond
  if (detectedSequenceType > 0) {
    // Calculate delay based on slave number
    unsigned long transmissionDelay = 1000 + slaveNumber * 500;
    
    Serial.print("✓ Sequence detected. Waiting ");
    Serial.print(transmissionDelay);
    Serial.println(" ms before transmitting...");
    
    // Wait calculated time
    delay(transmissionDelay);
    
    // Generate and send response
    Serial.println("⏱️ Transmitting response...");
    generarSecuenciaTransmision();
    
    Serial.print("Transmitting: [");
    for (int i = 0; i < 8; i++) {
      Serial.print(secuenciaTransmision[i]);
      if (i < 7) Serial.print("");
    }
    Serial.print("] (Slave ");
    Serial.print(slaveNumber);
    Serial.print(", Pin=");
    Serial.print(digitalRead(OUTPUT_PIN) ? "HIGH" : "LOW");
    Serial.println(")");
    
    // Transmit sequence multiple times
    for (int i = 0; i < NUM_REPETICIONES; i++) {
      transmitirSecuencia();
      delayMicroseconds(500);
    }
    
    Serial.println("✓ Transmission completed");
    
    // Reset detection and status variables
    detectedSequenceType = 0;
    
    Serial.println("\n▶️ Returning to listen mode...");
    delay(100); // Allow time to stabilize before next capture
    lastCaptureTime = millis(); // Reset capture timer
  }
  
  // Handle user input
  if (Serial.available() > 0) {
    char input = Serial.read();
    
    if (input == 'a' || input == 'A') {
      showFullArray = !showFullArray;
      captureAndDisplayData();
      lastCaptureTime = millis(); // Reset capture timer
    }
    else if (input == 't' || input == 'T') {
      Serial.println("\nStarting manual transmission...");
      transmitirSecuencia();
      Serial.println("Manual transmission completed.");
    }
  }
  
  yield(); // Allow ESP32 to handle background tasks
}



// ==================== TRANSMISSION FUNCTIONS ====================
void generarSecuenciaTransmision() {
  // Inicializar secuencia
  for (int i = 0; i < 8; i++) {
    secuenciaTransmision[i] = 0;
  }
  
  // 1. Bit de inicio (siempre 1)
  secuenciaTransmision[0] = 1;
  
  // 2. 4 bits del número de esclavo
  for (int i = 0; i < 4; i++) {
    secuenciaTransmision[i+1] = (slaveNumber >> (3-i)) & 0x01;
  }
  
  // 3. Bit de estado del pin 25 (0 si LOW, 1 si HIGH)
  secuenciaTransmision[5] = (digitalRead(OUTPUT_PIN) == HIGH) ? 1 : 0;
  
  // 4. Calcular bit de paridad par
  int sumaBits = 0;
  for (int i = 0; i < 7; i++) {
    sumaBits += secuenciaTransmision[i];
  }
  secuenciaTransmision[6] = (sumaBits % 2 == 0) ? 1 : 0; // Paridad par

  secuenciaTransmision[6] = 0;
  
  // 5. Bit de fin (siempre 1)
  secuenciaTransmision[7] = 1;
  
  // Imprimir la secuencia generada
  Serial.println("\n--- SECUENCIA DE TRANSMISIÓN GENERADA ---");
  Serial.print("Bits: [");
  for (int i = 0; i < 8; i++) {
    Serial.print(secuenciaTransmision[i]);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("]");
  Serial.println("Formato: [Inicio(1), Esclavo(4 bits), Estado Pin(1), Paridad(1), Fin(1)]");
}

void activarPortadora() {
  if (!canalActivo) {
    // Configuramos y arrancamos el canal nuevamente
    ledc_channel_config_t ledc_channel = {
      .gpio_num = PIN_TX,
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

void transmitirSecuencia() {
  // Regenerar la secuencia para asegurar que refleja el estado actual
  generarSecuenciaTransmision();
  
  // Enviar señal de sincronización inicial
  enviarSincronizacion();
  
  // Transmitir la secuencia de datos
  for (int i = 0; i < 8; i++) {
    if (secuenciaTransmision[i] == 1) {
      activarPortadora();
      delayMicroseconds(DURACION_BIT);
    } else {
      desactivarPortadora();
      delayMicroseconds(DURACION_BIT);
    }
  }

  // Asegurar que el pin esté en LOW después de cada transmisión
  desactivarPortadora();
  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, LOW);

  // Pausa entre transmisiones
  delayMicroseconds(PAUSA_ENVIOS);
}

// ==================== SEQUENCE TRIGGER FUNCTION ====================
void checkForTargetSequence(uint16_t value) {
  bool estadoPrevio = digitalRead(OUTPUT_PIN);
  
  if (value == TARGET_SEQUENCE_LOW) {
    // Set pin 25 to LOW
    digitalWrite(OUTPUT_PIN, LOW);
    manualPinControl = true;
    
    Serial.println("■ SECUENCIA DETECTADA: 10000001 → Pin 25 = LOW");
    
    // Activar mecanismo de respuesta
    detectedSequenceType = 1;
  }
  else if (value == TARGET_SEQUENCE_HIGH) {
    // Set pin 25 to HIGH
    digitalWrite(OUTPUT_PIN, HIGH);
    manualPinControl = true;
    
    Serial.println("■ SECUENCIA DETECTADA: 10000101 → Pin 25 = HIGH");
    
    // Activar mecanismo de respuesta
    detectedSequenceType = 2;
  }
}

// ==================== BINARY PRINTING ====================
void printBinary(uint8_t value) {
  for (int i = 7; i >= 0; i--) {
    Serial.print((value >> i) & 1);
  }
  Serial.println();
}

void print16BitBinary(uint16_t value) {
  for (int i = 7; i >= 0; i--) { // Solo mostramos 12 bits (el formato que nos interesa)
    Serial.print((value >> i) & 1);
    if (i % 4 == 0) Serial.print(" "); // Espacio cada 4 bits para mejor lectura
  }
}

// ==================== ACCESS POINT FUNCTIONS ====================
void startAP() {
  Serial.println("Starting Access Point...");
  
  // Configure AP
  WiFi.softAP(ap_ssid, ap_password);
  WiFi.softAPConfig(ap_local_ip, ap_gateway, ap_subnet);
  
  // Start web server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/style.css", HTTP_GET, handleCSS);
  server.on("/logout", HTTP_GET, handleLogout);
  server.onNotFound(handleNotFound);
  
  server.begin();
  
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  Serial.println("Web server started");
}

void stopAP() {
  Serial.println("Stopping Access Point...");
  
  // Stop the server first
  server.close();
  server.stop();
  
  // Disconnect and turn off the AP
  WiFi.softAPdisconnect(true);
  
  Serial.println("Access Point stopped");
}

// ==================== WEB SERVER HANDLER FUNCTIONS ====================
void handleRoot() {
  if (isAuthenticated) {
    server.sendHeader("Location", "/config");
    server.send(303);
  } else {
    String html = R"(
      <!DOCTYPE html>
      <html>
      <head>
        <title>ESP32 Power Monitor - Login</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <link rel="stylesheet" href="style.css">
      </head>
      <body>
        <div class="container">
          <h1>ESP32 Power Monitor</h1>
          <div class="card">
            <h2>Login</h2>
            <form action="/login" method="POST">
              <div class="input-group">
                <label for="username">Username:</label>
                <input type="text" id="username" name="username" required>
              </div>
              <div class="input-group">
                <label for="password">Password:</label>
                <input type="password" id="password" name="password" required>
              </div>
              <button type="submit" class="btn">Login</button>
            </form>
          </div>
        </div>
      </body>
      </html>
    )";
    server.send(200, "text/html", html);
  }
}

void handleLogin() {
  String username = server.arg("username");
  String password = server.arg("password");
  
  if (username == valid_username && password == valid_password) {
    isAuthenticated = true;
    server.sendHeader("Location", "/config");
    server.send(303);
  } else {
    String html = R"(
      <!DOCTYPE html>
      <html>
      <head>
        <title>ESP32 Power Monitor - Error</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <link rel="stylesheet" href="style.css">
        <meta http-equiv="refresh" content="2;url=/" />
      </head>
      <body>
        <div class="container">
          <h1>Login Failed</h1>
          <p>Invalid username or password. Redirecting...</p>
        </div>
      </body>
      </html>
    )";
    server.send(401, "text/html", html);
  }
}

void handleConfig() {
  if (!isAuthenticated) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }
  
  String html = R"(
    <!DOCTYPE html>
    <html>
    <head>
      <title>ESP32 Power Monitor - Configuration</title>
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <link rel="stylesheet" href="style.css">
    </head>
    <body>
      <div class="container">
        <h1>ESP32 Power Monitor</h1>
        <div class="card">
          <h2>Slave Monitor Configuration</h2>
          <form action="/save" method="POST">
            <div class="input-group">
              <label for="slaveNumber">Number of slave (1-247):</label>
              <input type="number" id="slaveNumber" name="slaveNumber" min="1" max="247" value=")";
  
  html += String(slaveNumber);
  
  html += R"(" required>
            </div>
            <button type="submit" class="btn">Save</button>
          </form>
          <div class="logout-container">
            <a href="/logout" class="btn logout-btn">Logout</a>
          </div>
        </div>
      </div>
    </body>
    </html>
  )";
  
  server.send(200, "text/html", html);
}

void handleSave() {
  if (!isAuthenticated) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }
  
  String slaveNumberStr = server.arg("slaveNumber");
  int newSlaveNumber = slaveNumberStr.toInt();
  
  if (newSlaveNumber >= MIN_SLAVE_VALUE && newSlaveNumber <= MAX_SLAVE_VALUE) {
    slaveNumber = (uint8_t)newSlaveNumber;
    EEPROM.write(SLAVE_ADDRESS_ADDR, slaveNumber);
    EEPROM.commit();
    
    // Regenerar la secuencia de transmisión con el nuevo número de esclavo
    generarSecuenciaTransmision();
    
    // Print the slave number to serial monitor
    Serial.println("--------------------------------------");
    Serial.println("Slave configuration updated:");
    Serial.print("Slave Number (Decimal): ");
    Serial.println(slaveNumber);
    Serial.print("Slave Number (Binary): ");
    printBinary(slaveNumber);
    Serial.println("--------------------------------------");
    
    String html = R"(
      <!DOCTYPE html>
      <html>
      <head>
        <title>ESP32 Power Monitor - Saved</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <link rel="stylesheet" href="style.css">
        <meta http-equiv="refresh" content="2;url=/config" />
      </head>
      <body>
        <div class="container">
          <h1>Settings Saved</h1>
          <p>Slave number updated successfully.</p>
          <p>Redirecting back to configuration...</p>
        </div>
      </body>
      </html>
    )";
    server.send(200, "text/html", html);
  } else {
    String html = R"(
      <!DOCTYPE html>
      <html>
      <head>
        <title>ESP32 Power Monitor - Error</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <link rel="stylesheet" href="style.css">
        <meta http-equiv="refresh" content="3;url=/config" />
      </head>
      <body>
        <div class="container">
          <h1>Error</h1>
          <p>Invalid slave number. Please enter a number between 1 and 247.</p>
          <p>Redirecting back to configuration...</p>
        </div>
      </body>
      </html>
    )";
    server.send(400, "text/html", html);
  }
}

void handleCSS() {
  String css = R"(
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: Arial, sans-serif;
    }
    
    body {
      background-color: #f5f5f5;
      padding: 20px;
    }
    
    .container {
      max-width: 800px;
      margin: 0 auto;
    }
    
    h1 {
      color: #2c3e50;
      text-align: center;
      margin-bottom: 20px;
      padding-bottom: 10px;
      border-bottom: 2px solid #3498db;
    }
    
    .card {
      background-color: white;
      border-radius: 8px;
      box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
      padding: 20px;
      margin-bottom: 20px;
    }
    
    h2 {
      color: #3498db;
      margin-bottom: 20px;
    }
    
    .input-group {
      margin-bottom: 15px;
    }
    
    label {
      display: block;
      margin-bottom: 5px;
      font-weight: bold;
    }
    
    input {
      width: 100%;
      padding: 10px;
      border: 1px solid #ddd;
      border-radius: 4px;
      font-size: 16px;
    }
    
    .btn {
      background-color: #3498db;
      color: white;
      border: none;
      border-radius: 4px;
      padding: 10px 15px;
      font-size: 16px;
      cursor: pointer;
      text-decoration: none;
      display: inline-block;
    }
    
    .btn:hover {
      background-color: #2980b9;
    }
    
    .status-normal {
      background-color: #d4edda;
      color: #155724;
      padding: 15px;
      border-radius: 4px;
      margin-bottom: 20px;
    }
    
    .status-warning {
      background-color: #fff3cd;
      color: #856404;
      padding: 15px;
      border-radius: 4px;
      margin-bottom: 20px;
    }
    
    .status-danger {
      background-color: #f8d7da;
      color: #721c24;
      padding: 15px;
      border-radius: 4px;
      margin-bottom: 20px;
    }
    
    .logout-container {
      text-align: right;
      margin-top: 20px;
    }
    
    .logout-btn {
      background-color: #e74c3c;
    }
    
    .logout-btn:hover {
      background-color: #c0392b;
    }
  )";
  
  server.send(200, "text/css", css);
}

void handleLogout() {
  isAuthenticated = false;
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleNotFound() {
  String html = R"(
    <!DOCTYPE html>
    <html>
    <head>
      <title>ESP32 Power Monitor - Not Found</title>
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <link rel="stylesheet" href="style.css">
      <meta http-equiv="refresh" content="3;url=/" />
    </head>
    <body>
      <div class="container">
        <h1>404 Not Found</h1>
        <p>The page you requested was not found.</p>
        <p>Redirecting to the homepage...</p>
      </div>
    </body>
    </html>
  )";
  server.send(404, "text/html", html);
}

// ==================== SIGNAL ANALYSIS FUNCTIONS ====================

// Improved function to decode bits from samples following ASK rules
void decodeBitsFromSamples(DetectionState* state) {
    // Reset counters in local state
    state->bitCount = 0;
    state->flankCount = 0;
    
    // Calculate samples per bit using the actual measured frequency
    float samplesPerBitFloat = samplingRate * 0.0001; // Samples in 100μs with real rate
    int samplesPerBit = (int)(samplesPerBitFloat + 0.5); // Round to nearest integer
    
    Serial.println("\n--- FLANK DETECTION ---");
    Serial.print("Samples per bit (100μs): ");
    Serial.println(samplesPerBit);
    
    // Step 1: Detect all rising and falling edges in the vector
    for (int i = 15; i < maxSamples - 15; i++) {
        // Rising edge detection (from 0 to 1)
        if (i > 0 && samples[i-1] == 0 && samples[i] == 1) {
            // Check that previous 15 samples are all zero
            bool allPreviousZero = true;
            for (int j = i-15; j < i; j++) {
                if (samples[j] != 0) {
                    allPreviousZero = false;
                    break;
                }
            }
            
            // If we found 15 consecutive zeros before the edge, it's valid
            if (allPreviousZero && state->flankCount < MAX_FLANKS) {
                state->detectedFlanks[state->flankCount].index = i;
                state->detectedFlanks[state->flankCount].isRising = true;
                state->flankCount++;
            }
        }
        
        // Falling edge detection (from 1 to 0)
        if (i > 0 && samples[i-1] == 1 && samples[i] == 0) {
            // Check that next 15 samples are all zero
            bool allFollowingZero = true;
            for (int j = i; j < i + 15 && j < maxSamples; j++) {
                if (samples[j] != 0) {
                    allFollowingZero = false;
                    break;
                }
            }
            
            // If there are 15 consecutive zeros after the edge, it's considered valid
            if (allFollowingZero && state->flankCount < MAX_FLANKS) {
                state->detectedFlanks[state->flankCount].index = i;
                state->detectedFlanks[state->flankCount].isRising = false;
                state->flankCount++;
            }
        }
    }
    
    // Sort flanks by position
    for (int i = 1; i < state->flankCount; i++) {
        FlankInfo key = state->detectedFlanks[i];
        int j = i - 1;
        while (j >= 0 && state->detectedFlanks[j].index > key.index) {
            state->detectedFlanks[j + 1] = state->detectedFlanks[j];
            j--;
        }
        state->detectedFlanks[j + 1] = key;
    }
    
    // Display information about detected flanks
    if (state->flankCount > 0) {
        Serial.print("Detected flanks: ");
        Serial.println(state->flankCount);
        
        for (int i = 0; i < state->flankCount; i++) {
            Serial.print("  Flank #");
            Serial.print(i+1);
            Serial.print(": ");
            Serial.print(state->detectedFlanks[i].isRising ? "Rising" : "Falling");
            Serial.print(" at sample ");
            Serial.println(state->detectedFlanks[i].index);
        }
        
        // Check for correct pattern of flanks according to ASK modulation
        bool validPattern = true;
        for (int i = 0; i < state->flankCount - 1; i++) {
            if ((state->detectedFlanks[i].isRising && state->detectedFlanks[i+1].isRising) || 
                (!state->detectedFlanks[i].isRising && !state->detectedFlanks[i+1].isRising)) {
                Serial.print("⚠️ Error in flank pattern: two consecutive ");
                Serial.print(state->detectedFlanks[i].isRising ? "rising" : "falling");
                Serial.println(" flanks (does not comply with ASK modulation)");
                validPattern = false;
            }
        }
        
        if (validPattern) {
            Serial.println("✓ Correct alternating flank sequence (rising-falling)");
        }
        
    } else {
        Serial.println("No valid flanks detected");
        return;
    }
    
    // Step 2: Decode bits between flanks following ASK modulation rules
    for (int f = 0; f < state->flankCount - 1; f++) {
        int startIndex = state->detectedFlanks[f].index;
        int endIndex = state->detectedFlanks[f+1].index;
        bool isRisingToFalling = state->detectedFlanks[f].isRising && !state->detectedFlanks[f+1].isRising;
        bool isFallingToRising = !state->detectedFlanks[f].isRising && state->detectedFlanks[f+1].isRising;
        
        // Calculate theoretical number of bits between flanks (rounded to nearest integer)
        float bitsFloating = (float)(endIndex - startIndex) / samplesPerBit;
        int bitsInSegment = round(bitsFloating);
        
        Serial.print("\nBetween flank ");
        Serial.print(state->detectedFlanks[f].isRising ? "rising" : "falling");
        Serial.print(" and flank ");
        Serial.print(state->detectedFlanks[f+1].isRising ? "rising" : "falling");
        Serial.print(": ");
        Serial.print(endIndex - startIndex);
        Serial.print(" samples (");
        Serial.print(bitsFloating, 2);
        Serial.print(" theoretical bits, rounded to ");
        Serial.print(bitsInSegment);
        Serial.println(" bits)");
        
        // Check if it complies with ASK modulation rules
        if (isRisingToFalling) {
            // Rising to falling edge means all bits are '1'
            Serial.println("  In ASK modulation: All these bits should be '1'");
            
            // Add 'bitsInSegment' bits with value '1'
            for (int b = 0; b < bitsInSegment && state->bitCount < MAX_BITS; b++) {
                state->decodedBits[state->bitCount++] = 1;
            }
        } 
        else if (isFallingToRising) {
            // Falling to rising edge means all bits are '0'
            Serial.println("  In ASK modulation: All these bits should be '0'");
            
            // Add 'bitsInSegment' bits with value '0'
            for (int b = 0; b < bitsInSegment && state->bitCount < MAX_BITS; b++) {
                state->decodedBits[state->bitCount++] = 0;
            }
        }
        else {
            // This case shouldn't happen if we verify pattern correctly
            Serial.println("⚠️ Inconsistent flank pattern, bits not decoded");
        }
    }
    
    // Show summary info about decoding
    Serial.print("\nTotal bits decoded according to ASK rules: ");
    Serial.println(state->bitCount);
}


void displayDecodedBits(DetectionState* state) {
    Serial.println("\n--- DECODED BITS (ASK MODULATION) ---");
    
    if (state->bitCount > 0) {
        // Show bits as binary array
        Serial.print("Bits: [");
        for (int i = 0; i < state->bitCount; i++) {
            Serial.print(state->decodedBits[i]);
            if (i < state->bitCount - 1) Serial.print(", ");
            
            // Line break every 20 bits for readability
            if ((i + 1) % 20 == 0 && i < state->bitCount - 1) {
                Serial.println();
                Serial.print("       ");
            }
        }
        Serial.println("]");
        
        // Display bits as hexadecimal value for easier interpretation
        Serial.print("Hex: ");
        for (int i = 0; i < state->bitCount; i += 8) {
            uint8_t byteVal = 0;
            // Convert 8-bit groups to bytes (if there are enough)
            for (int b = 0; b < 8 && (i + b) < state->bitCount; b++) {
                byteVal |= (state->decodedBits[i + b] << (7 - b));
            }
            Serial.print("0x");
            if (byteVal < 16) Serial.print("0"); // Show two digits
            Serial.print(byteVal, HEX);
            Serial.print(" ");
        }
        Serial.println();
    } else {
        Serial.println("No bits decoded");
    }
    
    Serial.println();
}



void captureAndDisplayData() {
  
  DetectionState detState = {false, {0}, {0}, 0, {0}, 0};
  
  // Capture start time
  startTime = micros();
  
  // Capture data directly from GPIO at high speed
  for (int i = 0; i < maxSamples; i++) {
    // Read digital pin and save as 0 or 1
    samples[i] = digitalRead(INPUT_PIN);
    
    // Small delay to control sampling rate
    delayMicroseconds(1);
  }
  
  // Capture end time
  endTime = micros();
  
  // Calculate actual sampling rate
  samplingRate = (float)maxSamples * 1000000.0 / (endTime - startTime);
  
  // Detect flanks and decode bits with local state
  decodeBitsFromSamples(&detState);

  if (detState.bitCount > 0) {
    // Look for 111000 sequence and capture the following bits
    findSequenceAndCaptureBits(&detState);
  }
  
  // Clear the screen
  Serial.write(27);      Serial.print("[2J");   // Clear screen
  Serial.write(27);      Serial.print("[H");    // Move to home position
  
  // Show header and sampling information
  Serial.println("=== ESP32 DIGITAL SIGNAL MONITOR WITH ASK DECODER ===");
  Serial.print("Capture #: ");
  Serial.println(captureCount);
  Serial.print("Time: ");
  Serial.print(millis() / 1000.0, 1);
  Serial.println(" seconds");
  
  Serial.print("Sampling rate: ");
  Serial.print(samplingRate / 1000.0, 2);
  Serial.println(" kHz");
  
  Serial.print("Capture duration: ");
  Serial.print((endTime - startTime) / 1000.0, 2);
  Serial.println(" ms");

  // Display pin status and transmitter information
  Serial.println("\n--- OUTPUT PIN & TRANSMITTER STATUS ---");
  Serial.print("Pin 25 is currently: ");
  Serial.println(digitalRead(OUTPUT_PIN) == HIGH ? "HIGH" : "LOW");
  Serial.print("Slave Number: ");
  Serial.print(slaveNumber);
  Serial.print(" (");
  for (int i = 7; i >= 0; i--) {
    Serial.print((slaveNumber >> i) & 1);
  }
  Serial.println(")");
  Serial.println("Pin 33: ASK transmitter output");
  Serial.println("Press 't' to manually transmit the current sequence");
  
  // Show current transmission sequence
  Serial.print("Transmitter sequence: [");
  for (int i = 0; i < 8; i++) {
    Serial.print(secuenciaTransmision[i]);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("]");
  Serial.println("[Start(1), SlaveID(4 bits), State(1), Parity(1), Stop(1)]");
  
  // Signal information
  Serial.println("\n--- SIGNAL ANALYSIS ---");
  
  // Count zeros and ones
  int zeroCount = 0, oneCount = 0;
  for (int i = 0; i < maxSamples; i++) {
    if (samples[i] == 0) zeroCount++;
    else oneCount++;
  }
  
  Serial.print("Zeros: ");
  Serial.print(zeroCount);
  Serial.print(" (");
  Serial.print((float)zeroCount / maxSamples * 100.0, 1);
  Serial.print("%)  Ones: ");
  Serial.print(oneCount);
  Serial.print(" (");
  Serial.print((float)oneCount / maxSamples * 100.0, 1);
  Serial.println("%)");
  
  // Show decoded bits
  displayDecodedBits(&detState);
  
  // Show bits captured and extract information if sequence found
  if (detState.sequenceFound) {
    displayCapturedBits(&detState);
    extractSequenceInfo(&detState);
  } else {
    Serial.println("\n--- 111000 SEQUENCE DETECTION ---");
    Serial.println("❌ 111000 sequence not found in data");
  }
  
  // Show numeric data (reduced or full sample)
  if (!showFullArray) {
    // Show reduced sample
    Serial.println("\n--- SELECTED SAMPLES ---");
    int step = maxSamples / displaySamples;
    if (step < 1) step = 1;
    
    for (int i = 0; i < maxSamples; i += step) {
      Serial.print(i);
      Serial.print(": ");
      Serial.println(samples[i]);
    }
    
    Serial.println("\nPress 'a' to show ALL samples");
  } 
  else {
    // Show complete array in [value1, value2, value3, ...] format
    Serial.println("\n--- COMPLETE DATA ARRAY ---");
    
    // Open initial bracket
    Serial.print("[");
    
    // Print all values separated by commas
    for (int i = 0; i < maxSamples; i++) {
      Serial.print(samples[i]);
      
      // If not the last one, add comma
      if (i < maxSamples - 1) {
        Serial.print(", ");
      }
      
      // Add line break every 15 values for better readability
      if ((i + 1) % 15 == 0 && i < maxSamples - 1) {
        Serial.println();
      }
    }
    
    // Close bracket
    Serial.println("]");
    
    Serial.println("\nPress 'a' to show summary view");
  }
  
  // Check if we detected a control sequence
  if (detState.sequenceFound) {
    // Convert bits to decimal value
    uint16_t decimalValue = 0;
    for (int i = 0; i < CAPTURE_LENGTH; i++) {
      decimalValue |= (detState.capturedBits[i] << (CAPTURE_LENGTH - 1 - i));
    }
    
    // Check for target sequences
    checkForTargetSequence(decimalValue);
  }
}



void findSequenceAndCaptureBits(DetectionState* state) {
    state->sequenceFound = false;
    
    // Check if we have enough bits to search for the sequence and capture data
    if (state->bitCount < SEQUENCE_LENGTH + CAPTURE_LENGTH) {
        Serial.println("\n--- 111000 SEQUENCE DETECTION ---");
        Serial.println("⚠️ Not enough bits to search for sequence and capture data");
        return;
    }
    
    // Additional validation: Check there are enough non-null bits
    int nonZeroBits = 0;
    for (int i = 0; i < state->bitCount; i++) {
        if (state->decodedBits[i] != 0) nonZeroBits++;
    }
    
    // Requires a minimum of significant bits
    if (nonZeroBits < 3) {
        Serial.println("⚠️ Signal too weak or no significant data");
        return;
    }
    
    // Search for 111000 sequence in decoded bits
    for (int i = 0; i <= state->bitCount - SEQUENCE_LENGTH - CAPTURE_LENGTH; i++) {
        bool matchFound = true;
        
        // Compare each bit of the target sequence
        for (int j = 0; j < SEQUENCE_LENGTH; j++) {
            if (state->decodedBits[i + j] != targetSequence[j]) {
                matchFound = false;
                break;
            }
        }
        
        // If we found the sequence, capture the next 8 bits
        if (matchFound) {
            state->sequenceFound = true;
            
            // Store the next 8 bits
            for (int k = 0; k < CAPTURE_LENGTH; k++) {
                state->capturedBits[k] = state->decodedBits[i + SEQUENCE_LENGTH + k];
            }
            
            // Exit loop after finding first occurrence
            break;
        }
    }
}


// Function to extract and analyze captured sequence information
void extractSequenceInfo(DetectionState* state) {
    Serial.println("\n--- CAPTURED DATA ANALYSIS ---");
    
    if (!state->sequenceFound) {
        Serial.println("❌ No valid data to analyze");
        return;
    }
    
    // Declare all necessary variables
    uint8_t startBit = state->capturedBits[0];
    uint8_t slaveID = 0;
    bool validID = false;
    uint8_t stateBit = 0;
    uint8_t parityBit = 0;
    uint8_t stopBit = 0;
    uint8_t calculatedParity = 0;
    int bitSum = 0;
    
    // Extract slave ID (4 bits)
    for (int i = 0; i < 4; i++) {
        slaveID |= (state->capturedBits[i+1] << (3-i));
    }
    
    // Check if ID is valid (1-15)
    validID = (slaveID >= 1 && slaveID <= MAX_SLAVE_VALUE);
    
    // Extract state bit
    stateBit = state->capturedBits[5];
    
    // Extract parity bit
    parityBit = state->capturedBits[6];
    
    // Extract stop bit
    stopBit = state->capturedBits[7];
    
    // Calculate parity (even parity) of ID and state bits
    for (int i = 0; i < 6; i++) {
        bitSum += state->capturedBits[i];
    }
    bitSum += state->capturedBits[7]; // Include stop bit in parity calculation
    calculatedParity = (bitSum % 2 == 0) ? 0 : 1; // Even parity
    
    // Display extracted information
    Serial.println("Frame structure: [Start(1)][SlaveID(4)][State(1)][Parity(1)][Stop(1)]");
    
    Serial.print("Start bit: ");
    Serial.println(startBit ? "1 ✓" : "0 ❌");
    
    Serial.print("Slave ID: ");
    Serial.print(slaveID);
    
    if (validID) {
        Serial.println(" ✓ (Valid ID)");
    } else {
        Serial.println(" ❌ (outside valid range 1-15)");
    }
    
    Serial.print("State bit: ");
    Serial.print(stateBit);
    Serial.println(stateBit ? " (ON)" : " (OFF)");
    
    Serial.print("Parity bit: ");
    Serial.print(parityBit);
    if (parityBit == calculatedParity) {
        Serial.println(" ✓ (correct parity)");
    } else {
        Serial.print(" ❌ (incorrect, should be ");
        Serial.print(calculatedParity);
        Serial.println(")");
    }
    
    Serial.print("Stop bit: ");
    Serial.println(stopBit ? "1 ✓" : "0 ❌");
    
    // General frame validation
    bool validFrame = (startBit == 1) && 
                      validID &&
                      (parityBit == calculatedParity) &&
                      (stopBit == 1);
    
    Serial.println();
    if (!validFrame) {
        Serial.println("❌ INVALID FRAME - Errors detected");
        return; // Don't process further if frame is invalid
    }

    Serial.println("✓ VALID FRAME");
    Serial.print("Command: Slave #");
    Serial.print(slaveID);
    Serial.print(" - ");
    Serial.println(stateBit ? "TURN ON" : "TURN OFF");
    
    // Apply command immediately if it's our ID or broadcast
    if (slaveID == slaveNumber) {
        Serial.print("➡️ Applying command to this device (ID ");
        Serial.print(slaveNumber);
        Serial.println(")");
        
        if (stateBit) {
            digitalWrite(OUTPUT_PIN, HIGH);
            Serial.println("✓ OUTPUT PIN SET TO HIGH");
        } else {
            digitalWrite(OUTPUT_PIN, LOW);
            Serial.println("✓ OUTPUT PIN SET TO LOW");
        }
        
        // Record the sequence so we know to transmit a response
        detectedSequenceType = stateBit ? 2 : 1; // 2=HIGH, 1=LOW
    }
}



void displayCapturedBits(DetectionState* state) {
    Serial.println("\n--- 111000 SEQUENCE DETECTION ---");
    
    if (!state->sequenceFound) {
        Serial.println("❌ 111000 sequence not found in data");
        return;
    }
    
    // Display captured bits
    Serial.println("✅ 111000 sequence found!");
    Serial.print("8 bits captured: [");
    for (int i = 0; i < CAPTURE_LENGTH; i++) {
        Serial.print(state->capturedBits[i]);
        if (i < CAPTURE_LENGTH - 1) Serial.print(", ");
    }
    Serial.println("]");
    
    // Convert bits to decimal value
    uint16_t decimalValue = 0;
    for (int i = 0; i < CAPTURE_LENGTH; i++) {
        decimalValue |= (state->capturedBits[i] << (CAPTURE_LENGTH - 1 - i));
    }
    
    // Display value in different formats
    Serial.print("Decimal value: ");
    Serial.println(decimalValue);
    
    Serial.print("Hexadecimal value: 0x");
    if (decimalValue < 0x10) Serial.print("0");
    if (decimalValue < 0x100) Serial.print("0");
    Serial.println(decimalValue, HEX);
}
