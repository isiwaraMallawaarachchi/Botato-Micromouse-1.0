#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>

// ── Configuration ────────────────────────────────────────────────────
static const char* STA_SSID = "Dialog 4G 412";   // <-- fill these in
static const char* STA_PASS = "ddc92c62";
static const char* AP_SSID  = "botato";
static const char* AP_PASS  = "botato123";        // >= 8 chars or AP won't start

static const int  UART_RX   = 21;                  // <- STM32 PA11
static const int  UART_TX   = 20;                  // -> STM32 PA12
static const long UART_BAUD = 460800;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ── Line reassembly ──────────────────────────────────────────────────
// Bytes arrive in fragments. Accumulate until '\n' before forwarding.
static char   lineBuf[512];
static size_t lineLen = 0;

static uint32_t linesIn = 0, linesOut = 0, dropped = 0, overlong = 0;

// ── Fan-out ──────────────────────────────────────────────────────────
static void broadcast(const char* s, size_t n) {
  // Events are the record of the robot's reasoning — never thin these.
  // Telemetry is 50Hz and droppable; on a weak link, sending all of it
  // fills the client queue and the library closes the connection.
  const bool isEvent = (strstr(s, "\"k\":\"e\"") != nullptr);
  if (!isEvent) {
    static uint32_t lastTelemTx = 0;
    if (millis() - lastTelemTx < 66) return;   // cap telemetry at ~15 Hz
    lastTelemTx = millis();
  }

  if (ws.availableForWriteAll()) {
    ws.textAll(s, n);
    linesOut++;
  } else {
    dropped++;
  }
}

// ── WebSocket Handler (UPDATED FOR TWO-WAY) ──────────────────────────
static void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[ws] client %u from %s\n",
                  client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[ws] client %u gone\n", client->id());
  } else if (type == WS_EVT_DATA) {
    // ── NEW: ROUTE WEB COMMANDS TO STM32 ──
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    
    // Make sure it's text data from a single frame
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      // Push the payload (e.g. "P:1.5\n") directly out the UART to STM32
      Serial1.write(data, len); 
    }
  }
}

// ── Minimal live view (UPDATED UI) ───────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"PAGE(<!doctype html><meta charset=utf-8><title>Botato</title>
<style>
 body{background:#0d0d10;color:#ddd;font:14px system-ui,sans-serif;margin:0;padding:20px}
 h1{font-size:15px;font-weight:600;margin:0 0 20px;letter-spacing:.5px}
 #st{color:#f55} #st.on{color:#5d5}
 #rig{position:relative;width:460px;height:300px;margin:0 auto}
 .s{position:absolute;width:84px;height:58px;border-radius:10px;
    display:flex;flex-direction:column;align-items:center;justify-content:center;
    color:#fff;text-shadow:0 1px 2px rgba(0,0,0,.5);
    box-shadow:0 2px 8px rgba(0,0,0,.4);transition:background .08s linear}
 .s .n{font-size:10px;opacity:.85;letter-spacing:1px}
 .s .v{font-size:19px;font-weight:700;font-variant-numeric:tabular-nums}
 .s.wall{outline:2px solid #fff;outline-offset:2px}
 #body{position:absolute;left:155px;top:118px;width:150px;height:150px;
       background:#1c1c22;border:1px solid #333;border-radius:14px;
       display:flex;flex-direction:column;align-items:center;justify-content:center}
 #body .cap{font-size:10px;color:#777;letter-spacing:1px}
 #phase{font-size:20px;font-weight:700;color:#7ab8ff;margin:4px 0}
 #nav{font-size:11px;color:#888}
 #log{margin:24px auto 0;max-width:760px;height:150px;overflow:auto;
      white-space:pre;color:#666;font:11px ui-monospace,monospace}
 
 /* NEW: Tuning Panel Styles */
 #tune{text-align:center; margin-top:20px; display:flex; gap:15px; justify-content:center;}
 #tune label{display:flex; flex-direction:column; font-size:12px; color:#aaa; font-weight:600;}
</style>

<h1>BOTATO — <span id=st>connecting…</span> <span id=hz></span></h1>

<div id=rig>
  <div id=body>
    <div class=cap>DOING</div>
    <div id=phase>–</div>
    <div id=nav>–</div>
  </div>
</div>

<!-- NEW: Tuning Panel HTML -->
<div id=tune>
  <label>Kp <input type=range id=kp min=0 max=10 step=0.1 value=1.0><span id=kp_val>1.0</span></label>
  <label>Ki <input type=range id=ki min=0 max=10 step=0.1 value=0.0><span id=ki_val>0.0</span></label>
  <label>Kd <input type=range id=kd min=0 max=10 step=0.1 value=0.0><span id=kd_val>0.0</span></label>
</div>

<div id=log></div>

<script>
// index -> [label, left, top].  Matches the physical sensor layout.
const POS = [
  ['L',    5, 160],
  ['LF',  55,  50],
  ['F',  188,   0],
  ['RF', 321,  50],
  ['R',  371, 160],
];
const PHASE = ['SENSE','DECIDE','TURNING','DRIVING','ARRIVE'];
const NAV   = ['IDLE','SEARCH','RETURN','SPEED','DONE'];
const MODE  = ['IDLE','CALIBRATING','SEARCH_RUN','SPEED_RUN'];
const MAXMM = 400;   // distance treated as "fully green"

const rig = document.getElementById('rig');
POS.forEach(([name,x,y],i)=>{
  const d = document.createElement('div');
  d.className = 's'; d.id = 's'+i;
  d.style.left = x+'px'; d.style.top = y+'px';
  d.innerHTML = `<span class=n>${name}</span><span class=v id=v${i}>–</span>`;
  rig.appendChild(d);
});

const st=document.getElementById('st'), hz=document.getElementById('hz'),
      log=document.getElementById('log'), phaseEl=document.getElementById('phase'),
      navEl=document.getElementById('nav');

let last=null, count=0;
setInterval(()=>{ hz.textContent=count+' Hz'; count=0; },1000);

const ws=new WebSocket('ws://'+location.host+'/ws');
ws.onopen =()=>{ st.textContent='live'; st.className='on'; };
ws.onclose=()=>{ st.textContent='disconnected'; st.className=''; };
ws.onmessage=e=>{ last=e.data; count++; };

function draw(){
  if(last){
    try{
      const m=JSON.parse(last);
      if(m.k==='t'){
        for(let i=0;i<5;i++){
          const mm=m.d[i];
          const hue=Math.min(mm/MAXMM,1)*120;        // 0=red .. 120=green
          const box=document.getElementById('s'+i);
          box.style.background=`hsl(${hue} 68% 42%)`;
          box.className='s'+(((m.wm>>i)&1)?' wall':'');
          document.getElementById('v'+i).textContent=mm;
        }
        phaseEl.textContent = PHASE[m.np] ?? '?';
        navEl.textContent   = (NAV[m.ns] ?? '?')+' · '+(MODE[m.md] ?? '?');
      }
      if(log.textContent.length>20000) log.textContent='';
      log.textContent+=last+'\n'; log.scrollTop=log.scrollHeight;
    }catch(err){}
    last=null;
  }
  requestAnimationFrame(draw);
}

// NEW: Slider transmission logic
function bindSlider(id, prefix) {
  const slider = document.getElementById(id);
  const valDisp = document.getElementById(id + '_val');
  
  // Update number while dragging
  slider.oninput = () => { valDisp.textContent = slider.value; };
  
  // Send command over websocket when mouse is released
  slider.onchange = () => { 
    if(ws.readyState === 1) { // 1 = WebSocket.OPEN
       ws.send(prefix + slider.value + '\n');
    }
  };
}

bindSlider('kp', 'P:');
bindSlider('ki', 'I:');
bindSlider('kd', 'D:');

draw();
</script>
)PAGE";

// ── Network ──────────────────────────────────────────────────────────
static void startNetwork() {
  if (STA_SSID[0]) {
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    WiFi.begin(STA_SSID, STA_PASS);
    Serial.printf("[wifi] joining %s", STA_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
      delay(200); Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("[wifi] STA ok  ->  http://");
      Serial.println(WiFi.localIP());
      if (MDNS.begin("botato")) Serial.println("[wifi] also http://botato.local");
      return;
    }
    Serial.println("[wifi] STA failed, falling back to AP");
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[wifi] AP \"");
  Serial.print(AP_SSID);
  Serial.print("\"  ->  http://");
  Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // CRITICAL: before begin(). The 256-byte default overflows at 460800
  // whenever loop() is briefly delayed. After begin() it has no effect.
  Serial1.setRxBufferSize(4096);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  startNetwork();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/health", HTTP_GET, [](AsyncWebServerRequest* r) {
    char b[192];
    snprintf(b, sizeof(b),
      "{\"clients\":%u,\"in\":%lu,\"out\":%lu,\"dropped\":%lu,"
      "\"overlong\":%lu,\"heap\":%lu}",
      ws.count(), (unsigned long)linesIn, (unsigned long)linesOut,
      (unsigned long)dropped, (unsigned long)overlong,
      (unsigned long)ESP.getFreeHeap());
    r->send(200, "application/json", b);
  });

  server.begin();
  Serial.println("[http] server up");
}

void loop() {
  // Drain everything available. Never delay() in here.
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) { linesIn++; broadcast(lineBuf, lineLen); }
      lineLen = 0;
    } else if (c == '\r') {
      // ignore
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      overlong++; lineLen = 0;    // corrupt framing: resync at next '\n'
    }
  }

  // Reap dead client handles or they leak heap until the ESP32 reboots.
  ws.cleanupClients();
}