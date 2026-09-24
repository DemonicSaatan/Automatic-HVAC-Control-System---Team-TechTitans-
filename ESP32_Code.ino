/*****************************************************************************************
 * PROJECT: Automated Smart HVAC Controller with 3D Room Simulation & Web Terminal
 * HARDWARE: ESP32 Dev Board, 2x IR (HW-201), HLK-LD2401 mmWave Radar, HW-036 (DHT11),
 *           0.96" I2C OLED (SSD1306), HW-874 Manual PWM Speed Controller, Peltier Module
 * FEATURES:
 *  - Bidirectional Door IR People Counter (Outer <-> Inner)
 *  - HLK-LD2401 mmWave Radar Parsing (Target presence, distance, and motion tracking)
 *  - HW-036 Humidity & Temperature reading
 *  - Bluetooth Serial Connection: Receive ambient mobile location temperature & humidity
 *  - Tariff-Aware Dynamic HVAC Calculation (Adjusts setpoint & recommended fan speed)
 *  - Power Consumption, Total Runtime & Projected Monthly Bill Calculator
 *  - Continuous Running Marquee on OLED: Reports live status and failing component alerts
 *  - Embedded Web Server with Real-Time Interactive 3D Room Simulation & Web Terminal
 *****************************************************************************************/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "BluetoothSerial.h"

// ======================== HARDWARE PIN DEFINITIONS ========================
#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64
#define OLED_RESET       -1
#define OLED_ADDRESS     0x3C

#define IR_OUTER_PIN     18   // Outer Door IR Sensor (Active LOW)
#define IR_INNER_PIN     19   // Inner Room IR Sensor (Active LOW)

#define RADAR_RX_PIN     16   // ESP32 RX2 connects to Radar TX
#define RADAR_TX_PIN     17   // ESP32 TX2 connects to Radar RX

#define DHT_PIN          23   // HW-036 DHT11 Data Pin

// ======================== CONFIGURATION & DEFAULTS ========================
const char* AP_SSID      = "SmartHVAC-Controller";
const char* AP_PASS      = "12345678";   // Wi-Fi Access Point Password
const char* BT_DEV_NAME  = "ESP32-HVAC"; // Bluetooth Device Name

#define BASE_TARIFF_DEFAULT  7.50f       // Default currency/kWh
#define PELTIER_MAX_WATTS    50.0f       // Rated Peltier power (Watts)
#define FANS_MAX_WATTS       7.0f        // 2x CPU fans rated power (Watts)
#define BASE_SYS_WATTS       1.8f        // ESP32 + sensors power (Watts)
#define BASE_TARGET_TEMP     24.0f       // Ideal baseline temperature in Celsius

// ======================== OBJECT INSTANCES ================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);
BluetoothSerial SerialBT;
Preferences prefs;

// ======================== GLOBAL SYSTEM STATE =============================
int peopleCount = 0;
float roomTemp = 25.0f;
float roomHumidity = 55.0f;

// Mobile location data received via Bluetooth
float mobileTemp = 28.0f;
float mobileHumidity = 60.0f;
bool mobileConnected = false;
unsigned long lastMobilePacketTime = 0;

// Tariff and Cost Tracking
float currentTariff = BASE_TARIFF_DEFAULT;
float targetTemp = BASE_TARGET_TEMP;
int recommendedFanSpeed = 0;             // 0 to 100%
String fanLevelStr = "OFF";
float currentPowerWatts = BASE_SYS_WATTS;
double accumulatedJoules = 0.0;
unsigned long sessionStartMillis = 0;
unsigned long lastPowerCalcTime = 0;
float estMonthlyBill = 0.0f;
float estMonthlyKwh = 0.0f;

// Radar Telemetry
bool radarStreaming = false;
bool radarTargetPresent = false;
uint8_t radarTargetState = 0;            // 0=None, 1=Moving, 2=Static, 3=Both
uint16_t radarMovingDist = 0;
uint16_t radarStaticDist = 0;
uint16_t radarActiveDistance = 0;        // Centimeters
uint8_t radarEnergy = 0;
unsigned long lastRadarPacketTime = 0;
unsigned long radarByteCount = 0;
uint8_t radarRxBuffer[128];
int radarRxIndex = 0;

// IR Door State Machine
enum DoorState { DOOR_IDLE, DOOR_OUTER_TRIGGERED, DOOR_INNER_TRIGGERED };
DoorState doorStateMachine = DOOR_IDLE;
unsigned long doorTriggerTime = 0;
unsigned long lastCountEventTime = 0;

// Diagnostics & Component Health Watchdog
bool faultRadar = false;
bool faultIrOuter = false;
bool faultIrInner = false;
bool faultDHT = false;
unsigned long irOuterLowStartTime = 0;
unsigned long irInnerLowStartTime = 0;
int dhtFailStreak = 0;
String activeWarningMsg = "";

// OLED UI & Marquee Ticker
int oledPage = 0;
unsigned long lastPageSwitch = 0;
int marqueeX = SCREEN_WIDTH;
unsigned long lastMarqueeUpdate = 0;

// Terminal Log Buffer
String terminalHistory = "[SYSTEM] HVAC Controller Booted Nominal.\n";

// ======================== FORWARD DECLARATIONS ============================
bool readDHT11(float &temp, float &hum);
void parseRadarStream();
void handleDoorSensors();
void recalculateHVAC();
void checkComponentHealth();
void updateOLED();
void handleBluetooth();
void setupWebServer();

// ======================== EMBEDDED WEB DASHBOARD & 3D SIM ==================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Smart HVAC 3D Digital Twin & Terminal</title>
<style>
  :root {
    --bg: #0b0f19; --card: #151d30; --accent: #38bdf8;
    --green: #22c55e; --amber: #f59e0b; --red: #ef4444; --text: #f1f5f9;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', system-ui, sans-serif; }
  body { background: var(--bg); color: var(--text); padding: 16px; display: flex; flex-direction: column; gap: 16px; }
  header { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #2a3756; padding-bottom: 12px; }
  h1 { font-size: 1.3rem; color: var(--accent); }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 14px; }
  .card { background: var(--card); border: 1px solid #24324f; border-radius: 10px; padding: 14px; position: relative; }
  .card h3 { font-size: 0.85rem; color: #94a3b8; text-transform: uppercase; margin-bottom: 8px; }
  .big-val { font-size: 1.8rem; font-weight: bold; color: #fff; }
  .sub-val { font-size: 0.85rem; color: #64748b; margin-top: 4px; }
  .badge { display: inline-block; padding: 2px 8px; border-radius: 6px; font-size: 0.75rem; font-weight: 600; }
  .badge-ok { background: #064e3b; color: #34d399; }
  .badge-warn { background: #78350f; color: #fbbf24; }
  .badge-err { background: #7f1d1d; color: #f87171; }
  
  /* 3D Canvas Viewport */
  #sim-container { background: #0f172a; border-radius: 10px; border: 1px solid #334155; position: relative; overflow: hidden; }
  #room3d { width: 100%; height: 320px; display: block; cursor: grab; }
  .sim-legend { position: absolute; top: 10px; left: 10px; font-size: 0.75rem; background: rgba(15,23,42,0.8); padding: 6px 10px; border-radius: 6px; }
  
  /* Terminal Console */
  .terminal-box { background: #050811; border: 1px solid #1e293b; border-radius: 8px; padding: 10px; font-family: monospace; }
  #terminal-log { height: 140px; overflow-y: auto; font-size: 0.8rem; color: #38bdf8; white-space: pre-wrap; line-height: 1.4; }
  .term-input-row { display: flex; gap: 8px; margin-top: 8px; }
  .term-input-row input { flex: 1; background: #0f172a; border: 1px solid #334155; color: #fff; padding: 6px 10px; border-radius: 6px; outline: none; font-family: monospace; }
  .term-input-row button { background: var(--accent); color: #000; border: none; padding: 6px 14px; font-weight: bold; border-radius: 6px; cursor: pointer; }
</style>
</head>
<body>

<header>
  <div>
    <h1>SMART HVAC CONTROLLER</h1>
    <div style="font-size:0.8rem; color:#64748b;">Autonomous Tariff Optimization & 3D Spatial Presence</div>
  </div>
  <div id="bt-status" class="badge badge-warn">BT: Searching Mobile</div>
</header>

<div class="grid">
  <div class="card">
    <h3>Occupancy & Radar</h3>
    <div class="big-val" id="val-ppl">0 People</div>
    <div class="sub-val" id="val-radar">Radar: Scanning (0 cm)</div>
  </div>

  <div class="card">
    <h3>Room & Mobile Climate</h3>
    <div class="big-val"><span id="val-temp">--</span>°C | <span id="val-hum">--</span>%</div>
    <div class="sub-val" id="val-mobile">Mobile: No data received</div>
  </div>

  <div class="card">
    <h3>Recommended Fan Speed</h3>
    <div class="big-val" style="color:var(--accent);"><span id="val-fan">0%</span> [<span id="val-level">OFF</span>]</div>
    <div class="sub-val" id="val-knob">Set HW-874 Knob: 0.0/10</div>
  </div>

  <div class="card">
    <h3>Tariff & Projected Bill</h3>
    <div class="big-val" style="color:#fbbf24;">$<span id="val-bill">0.00</span> /mo</div>
    <div class="sub-val">Tariff: $<span id="val-tariff">0.00</span>/kWh | <span id="val-pwr">0W</span> Load</div>
  </div>
</div>

<div id="sim-container">
  <div class="sim-legend">
    <b>3D DIGITAL TWIN ROOM</b> (Click & drag to orbit)<br>
    <span style="color:#22c55e;">●</span> Doorway IR Beams &nbsp;
    <span style="color:#38bdf8;">▲</span> HLK-LD2401 Radar &nbsp;
    <span style="color:#f59e0b;">■</span> Peltier & Fans
  </div>
  <canvas id="room3d" width="800" height="360"></canvas>
</div>

<div class="card">
  <h3>Interactive Web Terminal & Hardware Console</h3>
  <div class="terminal-box">
    <div id="terminal-log"></div>
    <div class="term-input-row">
      <input type="text" id="cmd-input" placeholder="Type command (e.g. 'tariff 8.5', 'people 3', 'help', 'status')...">
      <button onclick="sendCmd()">Send</button>
    </div>
  </div>
</div>

<script>
// Canvas 3D Isometric / Perspective Room Renderer
const canvas = document.getElementById('room3d');
const ctx = canvas.getContext('2d');
let rotX = 0.45, rotY = -0.65;
let isDragging = false, lastMouseX = 0, lastMouseY = 0;

canvas.addEventListener('mousedown', e => { isDragging = true; lastMouseX = e.clientX; lastMouseY = e.clientY; });
window.addEventListener('mouseup', () => isDragging = false);
canvas.addEventListener('mousemove', e => {
  if (!isDragging) return;
  rotY += (e.clientX - lastMouseX) * 0.01;
  rotX += (e.clientY - lastMouseY) * 0.01;
  rotX = Math.max(0.1, Math.min(1.2, rotX));
  lastMouseX = e.clientX; lastMouseY = e.clientY;
});

// Telemetry state for rendering
let simData = { people: 0, radarDist: 0, radarPresent: false, fanSpeed: 0 };

function project(x, y, z) {
  const cx = canvas.width / 2;
  const cy = canvas.height / 2 + 30;
  const scale = 1.1;
  
  // Rotate around Y
  const cosY = Math.cos(rotY), sinY = Math.sin(rotY);
  const x1 = x * cosY - z * sinY;
  const z1 = z * cosY + x * sinY;
  
  // Rotate around X
  const cosX = Math.cos(rotX), sinX = Math.sin(rotX);
  const y2 = y * cosX - z1 * sinX;
  const z2 = z1 * cosX + y * sinX;
  
  const depth = 380 / (380 + z2);
  return { x: cx + x1 * depth * scale, y: cy - y2 * depth * scale, depth };
}

function drawLine3D(x1, y1, z1, x2, y2, z2, color, width=1) {
  const p1 = project(x1, y1, z1);
  const p2 = project(x2, y2, z2);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(p1.x, p1.y);
  ctx.lineTo(p2.x, p2.y);
  ctx.stroke();
}

let fanAngle = 0;
function render3D() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const RW = 180, RH = 110, RD = 240; // Room Dimensions
  
  // Draw Room Floor Grid
  ctx.lineWidth = 1;
  for (let x = -RW; x <= RW; x += 45) drawLine3D(x, 0, 0, x, 0, RD, '#1e293b');
  for (let z = 0; z <= RD; z += 40) drawLine3D(-RW, 0, z, RW, 0, z, '#1e293b');
  
  // Draw Room Bounding Box (Walls)
  drawLine3D(-RW, 0, 0, -RW, RH, 0, '#334155');
  drawLine3D(RW, 0, 0, RW, RH, 0, '#334155');
  drawLine3D(-RW, 0, RD, -RW, RH, RD, '#1e293b');
  drawLine3D(RW, 0, RD, RW, RH, RD, '#1e293b');
  drawLine3D(-RW, RH, 0, RW, RH, 0, '#334155');
  drawLine3D(-RW, RH, 0, -RW, RH, RD, '#1e293b');
  drawLine3D(RW, RH, 0, RW, RH, RD, '#1e293b');

  // Entrance Doorway on Front-Left Wall with 2 IR Sensor Beams
  drawLine3D(-RW, 0, 40, -RW, 75, 40, '#22c55e', 2);
  drawLine3D(-RW, 0, 80, -RW, 75, 80, '#10b981', 2);
  drawLine3D(-RW, 75, 40, -RW, 75, 80, '#22c55e', 2);
  // IR Beams
  drawLine3D(-RW, 25, 40, -RW + 18, 25, 40, '#4ade80', 2); // Outer IR Beam
  drawLine3D(-RW, 25, 80, -RW + 18, 25, 80, '#34d399', 2); // Inner IR Beam

  // Peltier HVAC Module & CPU Fans on Back Wall (Z = 0)
  drawLine3D(-35, 60, 0, 35, 60, 0, '#f59e0b', 3);
  drawLine3D(-35, 25, 0, 35, 25, 0, '#f59e0b', 3);
  drawLine3D(-35, 25, 0, -35, 60, 0, '#f59e0b', 3);
  drawLine3D(35, 25, 0, 35, 60, 0, '#f59e0b', 3);

  // Animated Fans
  fanAngle += (simData.fanSpeed / 100) * 0.45;
  const pFan = project(0, 42, 2);
  ctx.save();
  ctx.translate(pFan.x, pFan.y);
  ctx.rotate(fanAngle);
  ctx.fillStyle = '#38bdf8';
  ctx.fillRect(-10, -2, 20, 4);
  ctx.fillRect(-2, -10, 4, 20);
  ctx.restore();

  // Radar Sensor on Side Wall (X = RW, Y = 50, Z = 120)
  const pRadar = project(RW, 50, 120);
  ctx.fillStyle = '#38bdf8';
  ctx.beginPath(); ctx.arc(pRadar.x, pRadar.y, 5, 0, Math.PI * 2); ctx.fill();
  // Radar Scan Pulses
  if (simData.radarPresent) {
    drawLine3D(RW, 50, 120, 0, 30, Math.min(RD, simData.radarDist * 0.6), 'rgba(56, 189, 248, 0.4)', 2);
  }

  // Draw Human Occupants based on People Count and Radar Distance
  const count = Math.max(simData.people, simData.radarPresent ? 1 : 0);
  for (let i = 0; i < count; i++) {
    const spacing = (i - (count - 1) / 2) * 40;
    const targetZ = simData.radarPresent ? Math.max(30, Math.min(RD - 20, simData.radarDist * 0.6)) : (70 + i * 35);
    const pFeet = project(spacing, 0, targetZ);
    const pHead = project(spacing, 55, targetZ);

    // Human Body
    ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(pFeet.x, pFeet.y); ctx.lineTo(pHead.x, pHead.y); ctx.stroke();
    // Head
    ctx.fillStyle = '#f8fafc';
    ctx.beginPath(); ctx.arc(pHead.x, pHead.y - 7, 6, 0, Math.PI * 2); ctx.fill();
    // Presence Aura Ring on Floor
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.6)'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.ellipse(pFeet.x, pFeet.y, 14 * pFeet.depth, 6 * pFeet.depth, 0, 0, Math.PI * 2); ctx.stroke();
  }

  requestAnimationFrame(render3D);
}
render3D();

// Real-Time Telemetry Polling
function fetchTelemetry() {
  fetch('/api/data')
    .then(r => r.json())
    .then(d => {
      document.getElementById('val-ppl').innerText = d.people + " " + (d.people === 1 ? "Person" : "People");
      document.getElementById('val-radar').innerText = "Radar: " + (d.radarPresent ? "TARGET AT " + d.radarDist + " cm" : "Clear / Standby");
      document.getElementById('val-temp').innerText = d.roomTemp.toFixed(1);
      document.getElementById('val-hum').innerText = d.roomHum.toFixed(0);
      document.getElementById('val-fan').innerText = d.fanSpeed + "%";
      document.getElementById('val-level').innerText = d.fanLevel;
      document.getElementById('val-knob').innerText = "Set HW-874 Knob: ~" + (d.fanSpeed / 10).toFixed(1) + "/10";
      document.getElementById('val-bill').innerText = d.estBill.toFixed(2);
      document.getElementById('val-tariff').innerText = d.tariff.toFixed(2);
      document.getElementById('val-pwr').innerText = d.watts.toFixed(0) + "W";

      const bt = document.getElementById('bt-status');
      if (d.mobileConnected) {
        bt.className = "badge badge-ok";
        bt.innerText = "BT: " + d.mobileTemp.toFixed(1) + "°C, " + d.mobileHum.toFixed(0) + "%";
        document.getElementById('val-mobile').innerText = "Mobile Ext: " + d.mobileTemp.toFixed(1) + "°C | " + d.mobileHum.toFixed(0) + "%";
      } else {
        bt.className = "badge badge-warn";
        bt.innerText = "BT: Searching Mobile";
      }

      simData.people = d.people;
      simData.radarDist = d.radarDist;
      simData.radarPresent = d.radarPresent;
      simData.fanSpeed = d.fanSpeed;
    }).catch(() => {});
}
setInterval(fetchTelemetry, 600);

// Terminal Console
function sendCmd() {
  const input = document.getElementById('cmd-input');
  const cmd = input.value.trim();
  if (!cmd) return;
  input.value = "";
  
  const log = document.getElementById('terminal-log');
  log.innerText += "\n> " + cmd;
  log.scrollTop = log.scrollHeight;

  fetch('/api/cmd?q=' + encodeURIComponent(cmd))
    .then(r => r.text())
    .then(res => {
      log.innerText += "\n" + res;
      log.scrollTop = log.scrollHeight;
    });
}
document.getElementById('cmd-input').addEventListener('keydown', e => { if (e.key === 'Enter') sendCmd(); });
</script>
</body>
</html>
)rawliteral";

// ======================== SETUP ============================================
void setup() {
  Serial.begin(115200);

  // Initialize NVS Preferences for Tariff Storage
  prefs.begin("hvac_cfg", false);
  currentTariff = prefs.getFloat("tariff", BASE_TARIFF_DEFAULT);

  // Initialize Hardware Pins
  pinMode(IR_OUTER_PIN, INPUT_PULLUP);
  pinMode(IR_INNER_PIN, INPUT_PULLUP);
  pinMode(DHT_PIN, INPUT_PULLUP);

  // Initialize OLED (Wire on GPIO 21, 22)
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("OLED Init Failed!"));
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(10, 20);
  display.println(F("SMART HVAC BOOTING"));
  display.setCursor(10, 35);
  display.println(F("Calibrating Sensors..."));
  display.display();

  // Initialize mmWave Radar UART (Serial2 on Pins 16 RX, 17 TX)
  Serial2.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  // Initialize Bluetooth Serial
  SerialBT.begin(BT_DEV_NAME);
  Serial.println(F("Bluetooth Initialized as 'ESP32-HVAC'"));

  // Initialize SoftAP Wi-Fi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(F("Access Point Started! IP: "));
  Serial.println(WiFi.softAPIP());

  // Setup Web Server Routes
  setupWebServer();
  server.begin();

  sessionStartMillis = millis();
  lastPowerCalcTime = millis();
  delay(1200);
}

// ======================== MAIN LOOP ========================================
void loop() {
  server.handleClient();
  handleBluetooth();
  parseRadarStream();
  handleDoorSensors();

  // Read HW-036 DHT11 Every 2 Seconds
  static unsigned long lastDHTRead = 0;
  if (millis() - lastDHTRead > 2000) {
    lastDHTRead = millis();
    float t, h;
    if (readDHT11(t, h)) {
      roomTemp = t;
      roomHumidity = h;
      dhtFailStreak = 0;
      faultDHT = false;
    } else {
      dhtFailStreak++;
      if (dhtFailStreak > 4) faultDHT = true;
    }
  }

  // Recalculate HVAC & Tariff Cost Every 500ms
  static unsigned long lastHvacCalc = 0;
  if (millis() - lastHvacCalc > 500) {
    lastHvacCalc = millis();
    recalculateHVAC();
    checkComponentHealth();
  }

  // Update OLED Display & Running Marquee
  updateOLED();
}

// ======================== BIDIRECTIONAL IR DOOR LOGIC ======================
void handleDoorSensors() {
  int outerVal = digitalRead(IR_OUTER_PIN);
  int innerVal = digitalRead(IR_INNER_PIN);
  bool outerTrig = (outerVal == LOW);
  bool innerTrig = (innerVal == LOW);

  // Watchdog: detect if beam is permanently blocked/stuck
  if (outerTrig) {
    if (irOuterLowStartTime == 0) irOuterLowStartTime = millis();
    else if (millis() - irOuterLowStartTime > 15000) faultIrOuter = true;
  } else {
    irOuterLowStartTime = 0;
    faultIrOuter = false;
  }

  if (innerTrig) {
    if (irInnerLowStartTime == 0) irInnerLowStartTime = millis();
    else if (millis() - irInnerLowStartTime > 15000) faultIrInner = true;
  } else {
    irInnerLowStartTime = 0;
    faultIrInner = false;
  }

  // Directional State Machine
  const unsigned long DOOR_TIMEOUT = 2200; // ms to complete doorway transit
  if (doorStateMachine != DOOR_IDLE && (millis() - doorTriggerTime > DOOR_TIMEOUT)) {
    doorStateMachine = DOOR_IDLE; // Reset aborted transit
  }

  if (doorStateMachine == DOOR_IDLE) {
    if (outerTrig && !innerTrig && (millis() - lastCountEventTime > 700)) {
      doorStateMachine = DOOR_OUTER_TRIGGERED;
      doorTriggerTime = millis();
    } else if (innerTrig && !outerTrig && (millis() - lastCountEventTime > 700)) {
      doorStateMachine = DOOR_INNER_TRIGGERED;
      doorTriggerTime = millis();
    }
  } else if (doorStateMachine == DOOR_OUTER_TRIGGERED) {
    if (innerTrig) {
      peopleCount++;
      doorStateMachine = DOOR_IDLE;
      lastCountEventTime = millis();
      terminalHistory += "[DOOR] >> ENTRY DETECTED! Occupants: " + String(peopleCount) + "\n";
    }
  } else if (doorStateMachine == DOOR_INNER_TRIGGERED) {
    if (outerTrig) {
      if (peopleCount > 0) peopleCount--;
      doorStateMachine = DOOR_IDLE;
      lastCountEventTime = millis();
      terminalHistory += "[DOOR] << EXIT DETECTED! Occupants: " + String(peopleCount) + "\n";
    }
  }
}

// ======================== HLK-LD2401 mmWave RADAR PARSER ===================
void parseRadarStream() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    radarByteCount++;
    lastRadarPacketTime = millis();
    radarStreaming = true;

    radarRxBuffer[radarRxIndex++] = b;
    if (radarRxIndex >= sizeof(radarRxBuffer)) radarRxIndex = 0;

    // Check for frame tail: F8 F7 F6 F5
    if (radarRxIndex >= 4) {
      if (radarRxBuffer[radarRxIndex - 4] == 0xF8 &&
          radarRxBuffer[radarRxIndex - 3] == 0xF7 &&
          radarRxBuffer[radarRxIndex - 2] == 0xF6 &&
          radarRxBuffer[radarRxIndex - 1] == 0xF5) {

        // Find header: F4 F3 F2 F1
        int h = -1;
        for (int i = 0; i <= radarRxIndex - 8; i++) {
          if (radarRxBuffer[i] == 0xF4 && radarRxBuffer[i+1] == 0xF3 &&
              radarRxBuffer[i+2] == 0xF2 && radarRxBuffer[i+3] == 0xF1) {
            h = i; break;
          }
        }

        if (h >= 0) {
          int payload = h + 6;
          if (payload + 10 <= radarRxIndex - 4) {
            radarTargetState = radarRxBuffer[payload + 2];
            radarMovingDist = radarRxBuffer[payload + 3] | (radarRxBuffer[payload + 4] << 8);
            radarEnergy = radarRxBuffer[payload + 5];
            radarStaticDist = radarRxBuffer[payload + 6] | (radarRxBuffer[payload + 7] << 8);

            if (radarTargetState == 0) {
              radarTargetPresent = false;
              radarActiveDistance = 0;
            } else {
              radarTargetPresent = true;
              radarActiveDistance = (radarTargetState == 2) ? radarStaticDist : radarMovingDist;
            }
          }
        }
        radarRxIndex = 0;
      }
    }
  }

  // Radar Timeout Watchdog
  if (millis() - lastRadarPacketTime > 3500) {
    radarStreaming = false;
    radarTargetPresent = false;
    faultRadar = true;
  } else {
    faultRadar = false;
  }
}

// ======================== TARIFF-AWARE HVAC & BILLING ALGORITHM =============
void recalculateHVAC() {
  // Target Temperature shifts dynamically to lower electricity bill during peak tariffs
  float tariffSurplus = max(0.0f, currentTariff - BASE_TARIFF_DEFAULT);
  float ecoTempRelax = constrain(tariffSurplus * 0.28f, 0.0f, 2.5f); // Relax up to +2.5C
  targetTemp = BASE_TARGET_TEMP + ecoTempRelax;

  // Thermal Heat Load Calculation
  // Combine room conditions with mobile location data if available
  float effectiveTemp = mobileConnected ? (roomTemp * 0.7f + mobileTemp * 0.3f) : roomTemp;
  float effectiveHum = mobileConnected ? (roomHumidity * 0.7f + mobileHumidity * 0.3f) : roomHumidity;

  int effectiveOccupancy = max(peopleCount, radarTargetPresent ? 1 : 0);

  if (effectiveOccupancy == 0) {
    // Room Empty -> Standby ECO mode
    recommendedFanSpeed = 0;
    fanLevelStr = "OFF";
    currentPowerWatts = BASE_SYS_WATTS;
  } else {
    // Active Cooling Load
    float deltaT = effectiveTemp - targetTemp;
    float loadScore = 20.0f + (deltaT * 9.0f) + (effectiveOccupancy * 12.0f) + max(0.0f, effectiveHum - 55.0f) * 0.5f;

    // Apply Electricity Tariff Economy Dampening
    float tariffDiscount = 1.0f - constrain((tariffSurplus / BASE_TARIFF_DEFAULT) * 0.35f, 0.0f, 0.40f);
    recommendedFanSpeed = constrain((int)(loadScore * tariffDiscount), 15, 100);

    // Map to knob guidance levels
    if (recommendedFanSpeed <= 35) fanLevelStr = "LOW";
    else if (recommendedFanSpeed <= 65) fanLevelStr = "MED";
    else if (recommendedFanSpeed <= 85) fanLevelStr = "HIGH";
    else fanLevelStr = "MAX";

    // Power consumption estimation: Peltier + 2 Fans + ESP32
    currentPowerWatts = BASE_SYS_WATTS + ((float)recommendedFanSpeed / 100.0f) * (PELTIER_MAX_WATTS + FANS_MAX_WATTS);
  }

  // Energy & Monthly Bill Integration
  unsigned long now = millis();
  float elapsedSec = (now - lastPowerCalcTime) / 1000.0f;
  lastPowerCalcTime = now;
  accumulatedJoules += (currentPowerWatts * elapsedSec);

  // Projected Monthly Consumption (assuming 10 operating hours/day)
  estMonthlyKwh = (currentPowerWatts * 10.0f * 30.0f) / 1000.0f;
  estMonthlyBill = estMonthlyKwh * currentTariff;
}

// ======================== HARDWARE HEALTH MONITOR ==========================
void checkComponentHealth() {
  activeWarningMsg = "";
  if (faultRadar) activeWarningMsg += "RADAR TIMEOUT! ";
  if (faultIrOuter) activeWarningMsg += "IR1 BLOCKED! ";
  if (faultIrInner) activeWarningMsg += "IR2 BLOCKED! ";
  if (faultDHT) activeWarningMsg += "DHT11 SENSOR FAULT! ";
}

// ======================== OLED DISPLAY & RUNNING MARQUEE ===================
void updateOLED() {
  // Cycle Pages Every 4 Seconds
  if (millis() - lastPageSwitch > 4000) {
    lastPageSwitch = millis();
    oledPage = !oledPage;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (oledPage == 0) {
    // PAGE 0: OCCUPANCY & HVAC
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("SMART HVAC   PPL: ["));
    display.print(peopleCount);
    display.print(F("]"));

    display.setCursor(0, 14);
    display.print(F("T: "));
    display.print(roomTemp, 1);
    display.print(F("C  H: "));
    display.print(roomHumidity, 0);
    display.print(F("%"));

    display.setCursor(0, 26);
    display.print(F("REC FAN: "));
    display.print(recommendedFanSpeed);
    display.print(F("% ["));
    display.print(fanLevelStr);
    display.print(F("]"));

    display.setCursor(0, 38);
    display.print(F("HW-874 KNOB: ~"));
    display.print((float)recommendedFanSpeed / 10.0f, 1);
    display.print(F("/10"));
  } else {
    // PAGE 1: TARIFF & PROJECTED BILL
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("POWER & CURRENT BILL"));

    display.setCursor(0, 14);
    display.print(F("TARIFF: $"));
    display.print(currentTariff, 2);
    display.print(F("/kWh"));

    display.setCursor(0, 26);
    display.print(F("PWR: "));
    display.print((int)currentPowerWatts);
    display.print(F("W | EST: $"));
    display.print(estMonthlyBill, 1);
    display.print(F("/mo"));

    display.setCursor(0, 38);
    unsigned long upSec = (millis() - sessionStartMillis) / 1000;
    display.printf("RUNTIME: %02lu:%02lu:%02lu", upSec / 3600, (upSec % 3600) / 60, upSec % 60);
  }

  // Divider Line for Marquee Bar
  display.drawLine(0, 50, 127, 50, SSD1306_WHITE);

  // Continuous Running Marquee on Bottom (Rows 53 to 63)
  String banner;
  if (activeWarningMsg.length() > 0) {
    banner = "*** WARNING: " + activeWarningMsg + " ***";
  } else {
    banner = "SYS OK | RADAR: " + String(radarTargetPresent ? "DETECT" : "CLEAR") +
             " | TARIFF: $" + String(currentTariff, 2) + " | EST BILL: $" + String(estMonthlyBill, 2) + "/MO |";
  }

  // Marquee Position Update every 40ms
  if (millis() - lastMarqueeUpdate > 40) {
    lastMarqueeUpdate = millis();
    marqueeX -= 2;
    int pixelLength = banner.length() * 6;
    if (marqueeX < -pixelLength) marqueeX = SCREEN_WIDTH;
  }

  display.setTextWrap(false);
  display.setCursor(marqueeX, 54);
  display.print(banner);
  display.setTextWrap(true);

  display.display();
}

// ======================== BLUETOOTH MOBILE RECEIVER ========================
void handleBluetooth() {
  if (SerialBT.available()) {
    String line = SerialBT.readStringUntil('\n');
    line.trim();

    // Check for "T:xx.x H:yy.y" or "xx.x,yy.y" format
    float inT = -999, inH = -999;
    if (line.indexOf("T:") >= 0 && line.indexOf("H:") >= 0) {
      sscanf(line.c_str(), "T:%f H:%f", &inT, &inH);
    } else if (line.indexOf(',') >= 0) {
      sscanf(line.c_str(), "%f,%f", &inT, &inH);
    }

    if (inT > -50 && inT < 80 && inH >= 0 && inH <= 100) {
      mobileTemp = inT;
      mobileHumidity = inH;
      mobileConnected = true;
      lastMobilePacketTime = millis();
      SerialBT.println(F("[ACK] Mobile location weather updated & adapted to HVAC!"));
    }
  }

  // Disconnect timeout if no mobile data for 90 seconds
  if (mobileConnected && (millis() - lastMobilePacketTime > 90000)) {
    mobileConnected = false;
  }
}

// ======================== HARDWARE DHT11 BIT-BANG DRIVER ===================
bool readDHT11(float &temp, float &hum) {
  uint8_t data[5] = {0, 0, 0, 0, 0};

  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  delay(20); // Hold LOW at least 18ms
  digitalWrite(DHT_PIN, HIGH);
  delayMicroseconds(30);
  pinMode(DHT_PIN, INPUT_PULLUP);

  unsigned long start = micros();
  while (digitalRead(DHT_PIN) == HIGH) if (micros() - start > 100) return false;
  start = micros();
  while (digitalRead(DHT_PIN) == LOW) if (micros() - start > 100) return false;
  start = micros();
  while (digitalRead(DHT_PIN) == HIGH) if (micros() - start > 100) return false;

  for (int i = 0; i < 40; i++) {
    start = micros();
    while (digitalRead(DHT_PIN) == LOW) if (micros() - start > 100) return false;
    unsigned long tHigh = micros();
    while (digitalRead(DHT_PIN) == HIGH) if (micros() - tHigh > 150) return false;
    if ((micros() - tHigh) > 40) data[i / 8] |= (1 << (7 - (i % 8)));
  }

  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    hum = (float)data[0] + (float)data[1] * 0.1f;
    temp = (float)data[2] + (float)data[3] * 0.1f;
    return (hum > 0 && hum <= 100);
  }
  return false;
}

// ======================== WEB SERVER & API ENDPOINTS =======================
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/api/data", HTTP_GET, []() {
    String json = "{";
    json += "\"people\":" + String(peopleCount) + ",";
    json += "\"roomTemp\":" + String(roomTemp, 1) + ",";
    json += "\"roomHum\":" + String(roomHumidity, 1) + ",";
    json += "\"mobileTemp\":" + String(mobileTemp, 1) + ",";
    json += "\"mobileHum\":" + String(mobileHumidity, 1) + ",";
    json += "\"mobileConnected\":" + String(mobileConnected ? "true" : "false") + ",";
    json += "\"fanSpeed\":" + String(recommendedFanSpeed) + ",";
    json += "\"fanLevel\":\"" + fanLevelStr + "\",";
    json += "\"tariff\":" + String(currentTariff, 2) + ",";
    json += "\"watts\":" + String(currentPowerWatts, 1) + ",";
    json += "\"estBill\":" + String(estMonthlyBill, 2) + ",";
    json += "\"radarPresent\":" + String(radarTargetPresent ? "true" : "false") + ",";
    json += "\"radarDist\":" + String(radarActiveDistance);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/api/cmd", HTTP_GET, []() {
    if (!server.hasArg("q")) { server.send(400, "text/plain", "Missing Command"); return; }
    String cmd = server.arg("q");
    cmd.trim();
    String reply = "";

    if (cmd.startsWith("tariff ")) {
      float t = cmd.substring(7).toFloat();
      if (t > 0) {
        currentTariff = t;
        prefs.putFloat("tariff", currentTariff);
        reply = "OK: Tariff updated to $" + String(currentTariff, 2) + "/kWh (Saved in NVS)";
      } else reply = "Error: Invalid tariff value.";
    } else if (cmd.startsWith("people ")) {
      peopleCount = max(0, cmd.substring(7).toInt());
      reply = "OK: People count manually overridden to " + String(peopleCount);
    } else if (cmd == "status") {
      reply = "STATUS REPORT:\nOccupancy: " + String(peopleCount) +
              "\nRoom Temp: " + String(roomTemp, 1) + "C | Humidity: " + String(roomHumidity, 0) + "%" +
              "\nFan Rec: " + String(recommendedFanSpeed) + "% (" + fanLevelStr + ")" +
              "\nTariff: $" + String(currentTariff, 2) + "/kWh | Est Bill: $" + String(estMonthlyBill, 2) + "/mo" +
              "\nRadar: " + String(radarStreaming ? "STREAMING OK" : "OFFLINE");
    } else if (cmd == "help") {
      reply = "Available Commands:\n  tariff <val>  : Update electricity tariff (e.g. 'tariff 8.5')\n"
              "  people <n>    : Set human occupancy count\n"
              "  status        : Print full system metrics\n"
              "  help          : Show this manual";
    } else {
      reply = "Unknown command: '" + cmd + "'. Type 'help' for commands.";
    }
    server.send(200, "text/plain", reply);
  });
}
