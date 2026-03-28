// ============================================================
//  TRANSMITTER ESP32-S3  (FIXED)
//  Fixes:
//   1. Altitude from DISTANCE_SENSOR (rangefinder) ONLY
//      - OPTICAL_FLOW_RAD removed (was overwriting good alt with 0)
//      - hasAlt flag: won't send 0 until first valid rangefinder reading
//   2. sendTelemetry() called right after DISTANCE_SENSOR update
//      so alt is fresh when packet fires
//   3. NRF correctly switches TX<->RX around each telemetry send
//   4. MAVLink loop reads ONE byte per loop() iteration so NRF
//      image packets are never starved
//   5. imgReceiving flag pauses telemetry TX during image transfer
// ============================================================

#include <Arduino.h>
#include <MAVLink.h>
#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <WebServer.h>

// ---- MAVLink UART ----
#define RXD1 18
#define TXD1 17

// ---- NRF pins ----
#define CE_PIN   39
#define CSN_PIN  40
#define SCK_PIN  41
#define MISO_PIN 2
#define MOSI_PIN 42

RF24 radio(CE_PIN, CSN_PIN);

const byte TELEM_ADDR[6] = "00001";  // Transmitter writes telemetry here
const byte IMAGE_ADDR[6] = "00002";  // Transmitter reads image chunks here

// ---- Telemetry struct ----
struct TelemetryData {
  float x, y, vx, vy, alt, voltage, current, percentage;
};
TelemetryData data;

bool hasAlt = false;  // Don't broadcast alt until rangefinder gives a real value

// ---- Battery config ----
const int   CELLS       = 3;
const float FULL_VOLTS  = 4.2f * CELLS;
const float EMPTY_VOLTS = 3.5f * CELLS;

// ---- Image transfer ----
#define CHUNK_PAYLOAD 25
#define PKT_SIZE      32
#define PKT_START     0
#define PKT_DATA      1
#define PKT_END       2

struct __attribute__((packed)) ImagePacket {
  uint8_t  type;
  uint16_t seqNum;
  uint16_t totalChunks;
  uint16_t chunkLen;
  uint8_t  payload[CHUNK_PAYLOAD];
};

#define MAX_IMAGE_BYTES (60 * 1024)
uint8_t  imgBuf[MAX_IMAGE_BYTES];
size_t   imgLen      = 0;
uint16_t imgTotal    = 0;
uint16_t imgReceived = 0;
bool     imgReady    = false;
bool     imgReceiving = false;

// ---- WiFi dashboard ----
const char* AP_SSID = "Nakshatra-TX";
const char* AP_PASS = "nakshatra123";
WebServer dashboard(80);

// ============================================================
//  Dashboard HTML
// ============================================================
const char DASH_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Nakshatra TX</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#0a0a0f;padding:14px;color:#fff}
.wrap{max-width:480px;margin:0 auto}
.hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:18px;padding-bottom:12px;border-bottom:1px solid #1a1a2a}
.title{font-size:15px;font-weight:bold;color:#fff}.title span{color:#34d399}
.sub{font-size:10px;color:#444;margin-top:2px}
.pill{display:flex;align-items:center;gap:5px;font-size:10px;color:#34d399;background:#081f14;border:1px solid #145232;padding:4px 10px;border-radius:99px}
.dot{width:6px;height:6px;border-radius:50%;background:#34d399;animation:blink 1.2s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.2}}
.sec{font-size:9px;color:#444;text-transform:uppercase;letter-spacing:.1em;margin-bottom:7px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:12px}
.card{border-radius:10px;padding:12px 14px}
.lbl{font-size:9px;text-transform:uppercase;letter-spacing:.08em;margin-bottom:4px;opacity:.8}
.val{font-size:22px;font-weight:bold;line-height:1}
.unit{font-size:10px;margin-left:3px;opacity:.5}
.pu{background:#1a0d40}.pu .lbl{color:#a78bfa}.pu .val{color:#e9d5ff}
.bu{background:#0c1e3a}.bu .lbl{color:#60a5fa}.bu .val{color:#bfdbfe}
.tu{background:#052e1e}.tu .lbl{color:#34d399}.tu .val{color:#a7f3d0}
.bat{background:#0b2a14;border-radius:10px;padding:13px 15px;margin-bottom:16px}
.bat-row{display:flex;justify-content:space-between;align-items:center;margin-bottom:9px}
.bat-num{font-size:28px;font-weight:bold;color:#4ade80}
.bat-lbl{font-size:9px;color:#34d399;text-transform:uppercase;letter-spacing:.08em}
.trk{height:8px;background:#0a1f0a;border-radius:99px;overflow:hidden}
.fill{height:100%;border-radius:99px;background:#4ade80;transition:width .5s}
.divhr{border-top:1px solid #1a1a2a;margin:0 0 16px}
.img-panel{background:#111018;border:1px solid #1a1a2a;border-radius:10px;padding:14px;margin-bottom:16px}
.img-hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
.img-title{font-size:12px;color:#a78bfa;font-weight:bold}
.img-meta{font-size:10px;color:#444}
.img-box{background:#0a0a0f;border-radius:8px;overflow:hidden;text-align:center;min-height:80px;display:flex;align-items:center;justify-content:center}
.img-box img{max-width:100%;border-radius:8px;display:none}
.img-ph{font-size:11px;color:#333;padding:20px}
.prog-bar{height:4px;background:#1a1a2a;border-radius:99px;overflow:hidden;margin-top:8px}
.prog-fill{height:100%;background:#a78bfa;border-radius:99px;transition:width .5s}
.img-st{font-size:10px;color:#555;margin-top:6px;text-align:center}
.st-rx{color:#fbbf24}.st-ok{color:#34d399}
</style></head><body>
<div class="wrap">
  <div class="hdr">
    <div><div class="title">&#9732; <span>Nakshatra</span> TX</div><div class="sub">Transmitter Dashboard</div></div>
    <div class="pill"><div class="dot"></div>Live</div>
  </div>

  <div class="sec">Position</div>
  <div class="g2">
    <div class="card pu"><div class="lbl">X axis</div><div class="val"><span id="x">--</span><span class="unit">cm</span></div></div>
    <div class="card pu"><div class="lbl">Y axis</div><div class="val"><span id="y">--</span><span class="unit">cm</span></div></div>
  </div>

  <div class="sec">Velocity</div>
  <div class="g2">
    <div class="card bu"><div class="lbl">Vel X</div><div class="val"><span id="vx">--</span><span class="unit">cm/s</span></div></div>
    <div class="card bu"><div class="lbl">Vel Y</div><div class="val"><span id="vy">--</span><span class="unit">cm/s</span></div></div>
  </div>

  <div class="sec">Altitude — Rangefinder</div>
  <div class="card tu" style="margin-bottom:12px">
    <div class="lbl">Height</div>
    <div class="val" style="font-size:26px"><span id="alt">--</span><span class="unit" style="font-size:11px">cm</span></div>
  </div>

  <div class="sec">Battery</div>
  <div class="bat">
    <div class="bat-row">
      <div class="bat-num"><span id="pct">--</span>%</div>
      <div class="bat-lbl" id="batst">Charge level</div>
    </div>
    <div class="trk"><div class="fill" id="bf" style="width:0%"></div></div>
  </div>

  <div class="divhr"></div>

  <div class="img-panel">
    <div class="img-hdr">
      <span class="img-title">&#128247; Received Image</span>
      <span class="img-meta" id="imgMeta">No image yet</span>
    </div>
    <div class="img-box">
      <img id="rxImg" alt="received">
      <div class="img-ph" id="imgPh">Waiting for image transfer…</div>
    </div>
    <div class="prog-bar"><div class="prog-fill" id="imgProg" style="width:0%"></div></div>
    <div class="img-st" id="imgSt">Idle</div>
  </div>
</div>
<script>
var imgLoaded=false;
function upd(){
  fetch('/tdata').then(r=>r.json()).then(d=>{
    document.getElementById('x').textContent=d.x.toFixed(1);
    document.getElementById('y').textContent=d.y.toFixed(1);
    document.getElementById('vx').textContent=d.vx.toFixed(1);
    document.getElementById('vy').textContent=d.vy.toFixed(1);
    document.getElementById('alt').textContent=d.hasAlt ? d.alt.toFixed(1) : '--';
    var pc=Math.min(100,Math.max(0,d.percentage));
    document.getElementById('pct').textContent=pc.toFixed(0);
    var f=document.getElementById('bf');
    f.style.width=pc+'%';
    f.style.background=pc>60?'#4ade80':pc>30?'#fbbf24':'#f87171';
    document.getElementById('batst').textContent=pc>60?'Good':pc>30?'Low':'Critical';
    var pr=document.getElementById('imgProg');
    var st=document.getElementById('imgSt');
    if(d.imgReceiving){
      var pct=d.imgTotal>0?Math.round(d.imgReceived/d.imgTotal*100):0;
      pr.style.width=pct+'%';
      st.textContent='Receiving '+d.imgReceived+'/'+d.imgTotal+' ('+pct+'%)';
      st.className='img-st st-rx';
      document.getElementById('imgMeta').textContent='Transferring…';
    } else if(d.imgReady){
      pr.style.width='100%';
      st.textContent='Ready — '+Math.round(d.imgLen/1024)+' KB';
      st.className='img-st st-ok';
      document.getElementById('imgMeta').textContent=Math.round(d.imgLen/1024)+' KB';
      if(!imgLoaded){
        imgLoaded=true;
        var img=document.getElementById('rxImg');
        img.onload=function(){img.style.display='block';document.getElementById('imgPh').style.display='none';};
        img.src='/img?t='+Date.now();
      }
    }
  }).catch(()=>{});
}
setInterval(upd,500); upd();
</script>
</body></html>
)rawliteral";

// ============================================================
//  Dashboard HTTP handlers
// ============================================================
void handleDashRoot() { dashboard.send(200, "text/html", DASH_HTML); }

void handleDashData() {
  String j = "{";
  j += "\"x\":"            + String(data.x,  1)              + ",";
  j += "\"y\":"            + String(data.y,  1)              + ",";
  j += "\"vx\":"           + String(data.vx, 1)              + ",";
  j += "\"vy\":"           + String(data.vy, 1)              + ",";
  j += "\"alt\":"          + String(data.alt,1)              + ",";
  j += String("\"hasAlt\":") + (hasAlt ? "true" : "false") + ",";
  j += "\"percentage\":"   + String(data.percentage, 0)      + ",";
j += String("\"imgReady\":") + (imgReady ? "true" : "false") + ",";
j += String("\"imgReceiving\":") + (imgReceiving ? "true" : "false") + ",";
  j += "\"imgTotal\":"     + String(imgTotal)                + ",";
  j += "\"imgReceived\":"  + String(imgReceived)             + ",";
  j += "\"imgLen\":"       + String(imgLen);
  j += "}";
  dashboard.send(200, "application/json", j);
}

void handleDashImg() {
  if (!imgReady || imgLen == 0) {
    dashboard.send(404, "text/plain", "no image");
    return;
  }
  const char* mime = "image/jpeg";
  if (imgLen > 3 && imgBuf[0] == 0x89 && imgBuf[1] == 'P') mime = "image/png";
  else if (imgLen > 5 && imgBuf[0] == 'G' && imgBuf[1] == 'I') mime = "image/gif";
  dashboard.sendHeader("Cache-Control", "no-cache");
  dashboard.send_P(200, mime, (const char*)imgBuf, imgLen);
}

// ============================================================
//  NRF send telemetry
//  Switches to TX, writes, then returns to RX for image chunks
// ============================================================
void sendTelemetry() {
  radio.stopListening();
  radio.openWritingPipe(TELEM_ADDR);

  bool ok = radio.write(&data, sizeof(data));

  radio.openReadingPipe(1, IMAGE_ADDR);
  radio.startListening();

  Serial.print(" | NRF: "); Serial.println(ok ? "OK" : "FAIL");
}

// ============================================================
//  Image packet handler
// ============================================================
void processImagePacket(const ImagePacket& pkt) {
  switch (pkt.type) {
    case PKT_START:
      Serial.println("[IMG] START — telemetry TX paused");
      imgReceiving = true;
      imgReady     = false;
      imgLen       = 0;
      imgTotal     = 0;
      imgReceived  = 0;
      memset(imgBuf, 0, sizeof(imgBuf));
      break;

    case PKT_DATA:
      if (!imgReceiving) break;
      imgTotal = pkt.totalChunks;
      {
        size_t offset = (size_t)pkt.seqNum * CHUNK_PAYLOAD;
        if (offset + pkt.chunkLen <= MAX_IMAGE_BYTES) {
          memcpy(imgBuf + offset, pkt.payload, pkt.chunkLen);
          size_t end = offset + pkt.chunkLen;
          if (end > imgLen) imgLen = end;
          imgReceived++;
        }
      }
      Serial.printf("[IMG] chunk %u/%u\n", pkt.seqNum + 1, pkt.totalChunks);
      break;

    case PKT_END:
      Serial.println("[IMG] END — telemetry TX resumed");
      imgReceiving = false;
      imgReady     = (imgLen > 0);
      Serial.printf("[IMG] total size: %u bytes\n", (unsigned)imgLen);
      break;
  }
}

// ============================================================
//  MAVLink stream requests
// ============================================================
void request_data_streams() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;

  // Position (x, y, vx, vy) at 10 Hz
  mavlink_msg_request_data_stream_pack(1,200,&msg,1,0,MAV_DATA_STREAM_POSITION,10,1);
  len = mavlink_msg_to_send_buffer(buf,&msg); Serial1.write(buf,len);

  // Battery at 2 Hz
  mavlink_msg_request_data_stream_pack(1,200,&msg,1,0,MAV_DATA_STREAM_EXTENDED_STATUS,2,1);
  len = mavlink_msg_to_send_buffer(buf,&msg); Serial1.write(buf,len);

  // RAW_SENSORS at 10 Hz — this stream carries DISTANCE_SENSOR
  mavlink_msg_request_data_stream_pack(1,200,&msg,1,0,MAV_DATA_STREAM_RAW_SENSORS,10,1);
  len = mavlink_msg_to_send_buffer(buf,&msg); Serial1.write(buf,len);
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(57600, SERIAL_8N1, RXD1, TXD1);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  if (!radio.begin()) {
    Serial.println("NRF FAIL — halting");
    while (1);
  }
  radio.setChannel(115);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  // Always listen on IMAGE pipe; sendTelemetry() temporarily switches to TX
  radio.openReadingPipe(1, IMAGE_ADDR);
  radio.startListening();

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("TX Dashboard: http://"); Serial.println(WiFi.softAPIP());

  dashboard.on("/",      handleDashRoot);
  dashboard.on("/tdata", handleDashData);
  dashboard.on("/img",   handleDashImg);
  dashboard.begin();
  Serial.println("TX web server started");

  delay(3000);
  request_data_streams();
}

// ============================================================
//  Loop
//  KEY FIX: MAVLink reads only ONE byte per loop() pass so the
//  NRF image-packet check runs frequently and never starves.
// ============================================================
void loop() {
  dashboard.handleClient();

  // ── Check for incoming image packets ──
  uint8_t pipe;
  if (radio.available(&pipe) && pipe == 1) {
    ImagePacket pkt;
    radio.read(&pkt, PKT_SIZE);
    processImagePacket(pkt);
  }

  // ── MAVLink: one byte at a time so NRF never starves ──
  if (Serial1.available()) {
    mavlink_message_t msg;
    mavlink_status_t  status;
    uint8_t c = Serial1.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      switch (msg.msgid) {

        // ── Position ──
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
          mavlink_local_position_ned_t lpos;
          mavlink_msg_local_position_ned_decode(&msg, &lpos);
          data.x  = lpos.x  * 100.0f;
          data.y  = lpos.y  * 100.0f;
          data.vx = lpos.vx * 100.0f;
          data.vy = lpos.vy * 100.0f;
          Serial.printf("X:%.1f Y:%.1f VX:%.1f VY:%.1f",
                        data.x, data.y, data.vx, data.vy);
          break;
        }

        // ── Rangefinder altitude (PRIMARY source) ──
        case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
          mavlink_distance_sensor_t dist;
          mavlink_msg_distance_sensor_decode(&msg, &dist);

          // Accept only readings that are in a sane range
          // DISTANCE_SENSOR is already in cm
          if (dist.current_distance > 0 && dist.current_distance < 4000) {
            data.alt = (float)dist.current_distance;
            hasAlt   = true;
            Serial.printf(" | RANGE: %.1f cm", data.alt);

            // Send immediately so alt is never stale
            if (!imgReceiving) {
              sendTelemetry();
            }
          }
          break;
        }

        // ── OPTICAL_FLOW_RAD intentionally removed ──
        // It was overwriting valid rangefinder alt with 0 whenever
        // flow.distance was invalid. Use only DISTANCE_SENSOR.

        // ── Battery ──
        case MAVLINK_MSG_ID_SYS_STATUS: {
          mavlink_sys_status_t sys;
          mavlink_msg_sys_status_decode(&msg, &sys);
          data.voltage    = sys.voltage_battery / 1000.0f;
          data.current    = sys.current_battery / 100.0f;
          float pct       = ((data.voltage - EMPTY_VOLTS) /
                             (FULL_VOLTS  - EMPTY_VOLTS)) * 100.0f;
          data.percentage = constrain(pct, 0.0f, 100.0f);

          Serial.printf(" | BAT:%.2fV %.1fA %.0f%%",
                        data.voltage, data.current, data.percentage);

          // Send telemetry on battery update too (only if not sending image
          // and we already have a valid alt so receiver doesn't see 0)
          if (!imgReceiving && hasAlt) {
            sendTelemetry();
          }
          break;
        }
      }
    }
  }
}
