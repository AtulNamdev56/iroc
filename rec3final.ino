// ============================================================
//  RECEIVER ESP32  (FIXED)
//  Fixes:
//   1. Binary POST body correctly read using server.on() with
//      body handler — ESP32 WebServer collects raw body properly
//      when you call server.collectHeaders() and use the upload
//      handler. Simplest fix: read body via server.arg("plain")
//      after enabling it with server.on(..., handler) and ensuring
//      Content-Type is NOT multipart. Works for octet-stream when
//      body is small (<4 KB). Each chunk is only 32 bytes — fine.
//   2. NRF stays in TX mode for the whole image session; only
//      switches back to RX after PKT_END is forwarded
//   3. Telemetry completely paused (NRF in TX, loop skips reads)
// ============================================================

#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <WebServer.h>

#define CE_PIN    4
#define CSN_PIN   5
#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23
#define RELAY_PIN 2

RF24 radio(CE_PIN, CSN_PIN);

const byte TELEM_ADDR[6] = "00001";  // Receiver reads telemetry from here
const byte IMAGE_ADDR[6] = "00002";  // Receiver writes image chunks here

const char* ssid     = "Team Nakshatra";
const char* password = "nakshatra123";

WebServer server(80);

bool relayState     = false;
bool nrfOK          = false;
bool imageUploading = false;

struct TelemetryData {
  float x, y, vx, vy, alt, voltage, current, percentage;
};
TelemetryData data;

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

// ── Send a pre-built 32-byte NRF packet ──
// Called while already in TX mode (set by handleImgStart)
bool nrfSendPacket(const uint8_t* pkt) {
  bool ok = false;
  for (int r = 0; r < 3; r++) {
    ok = radio.write(pkt, PKT_SIZE);
    if (ok) break;
    delay(5);
  }
  return ok;
}

// ============================================================
//  HTML
// ============================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Team Nakshatra</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#0a0a0f;padding:14px;color:#fff}
.wrap{max-width:460px;margin:0 auto}
.hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:18px;padding-bottom:12px;border-bottom:1px solid #1a1a2a}
.title{font-size:15px;font-weight:bold;color:#fff}.title span{color:#a78bfa}
.sub{font-size:10px;color:#444;margin-top:2px}
.live{display:flex;align-items:center;gap:5px;font-size:10px;color:#34d399;background:#081f14;border:1px solid #145232;padding:4px 10px;border-radius:99px}
.dot{width:6px;height:6px;border-radius:50%;background:#34d399;animation:blink 1.2s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:0.2}}
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
.sec-ctrl{font-size:9px;color:#444;text-transform:uppercase;letter-spacing:.1em;margin-bottom:10px}
.ctrl-grid{display:grid;grid-template-columns:1fr;gap:10px}
.cbtn{border:none;border-radius:10px;padding:14px 16px;cursor:pointer;font-family:monospace;display:flex;justify-content:space-between;align-items:center;width:100%}
.cbtn .cl{font-size:9px;text-transform:uppercase;letter-spacing:.08em;margin-bottom:3px}
.cbtn .cv{font-size:14px;font-weight:bold}
.cbtn .tag{font-size:10px;padding:3px 10px;border-radius:99px}
.relay-off{background:#111827}.relay-off .cl{color:#6b7280}.relay-off .cv{color:#9ca3af}.relay-off .tag{background:#1f2937;color:#6b7280}
.relay-on{background:#14532d}.relay-on .cl{color:#4ade80}.relay-on .cv{color:#bbf7d0}.relay-on .tag{background:#166534;color:#4ade80}
.arm-btn{background:#1a0d40}.arm-btn .cl{color:#a78bfa}.arm-btn .cv{color:#c4b5fd}.arm-btn .tag{background:#2e1065;color:#a78bfa}
.disarm-btn{background:#3b0d0d}.disarm-btn .cl{color:#f87171}.disarm-btn .cv{color:#fecaca}.disarm-btn .tag{background:#7f1d1d;color:#fca5a5}
.mode-manual{background:#1a1500}.mode-manual .cl{color:#fbbf24}.mode-manual .cv{color:#fef08a}.mode-manual .tag{background:#451a03;color:#fbbf24}
.mode-auto{background:#0c1e3a}.mode-auto .cl{color:#60a5fa}.mode-auto .cv{color:#bfdbfe}.mode-auto .tag{background:#1e3a5f;color:#60a5fa}
.tkoff-standby{background:#111827}.tkoff-standby .cl{color:#6b7280}.tkoff-standby .cv{color:#9ca3af}.tkoff-standby .tag{background:#1f2937;color:#6b7280}
.tkoff-ready{background:#052e1e}.tkoff-ready .cl{color:#34d399}.tkoff-ready .cv{color:#a7f3d0}.tkoff-ready .tag{background:#065f46;color:#34d399}
/* image upload */
.img-sec{margin-top:20px;padding-top:16px;border-top:1px solid #1a1a2a}
.img-drop{border:2px dashed #2a2a3a;border-radius:10px;padding:20px;text-align:center;
  cursor:pointer;position:relative;transition:border-color .2s}
.img-drop:hover{border-color:#a78bfa}
.img-drop input{position:absolute;inset:0;opacity:0;cursor:pointer;width:100%;height:100%}
.img-drop-icon{font-size:28px;margin-bottom:6px}
.img-drop-txt{font-size:11px;color:#555}
.img-fname{font-size:11px;color:#a78bfa;margin-top:6px;word-break:break-all;min-height:14px}
#preview{margin-top:10px;max-width:100%;border-radius:8px;display:none}
.send-btn{margin-top:10px;width:100%;border:none;border-radius:10px;padding:13px;
  background:#2e1065;color:#c4b5fd;font-family:monospace;font-size:13px;font-weight:bold;
  cursor:pointer;transition:background .2s}
.send-btn:hover:not(:disabled){background:#3b1585}
.send-btn:disabled{background:#1a1a2a;color:#444;cursor:not-allowed}
.prog-wrap{margin-top:10px;display:none}
.prog-bar{height:6px;background:#1a1a2a;border-radius:99px;overflow:hidden;margin-bottom:6px}
.prog-fill{height:100%;background:#a78bfa;border-radius:99px;width:0%;transition:width .3s}
.prog-txt{font-size:10px;color:#666;text-align:center}
.up-status{margin-top:8px;font-size:10px;text-align:center;color:#555;min-height:14px}
.st-ok{color:#34d399}.st-err{color:#f87171}.st-busy{color:#fbbf24}
</style></head><body>
<div class="wrap">

  <div class="hdr">
    <div><div class="title">&#9733; <span>Nakshatra</span></div><div class="sub">ESP32 Live Telemetry</div></div>
    <div class="live"><div class="dot"></div>Live</div>
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

  <div class="sec">Altitude</div>
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
  <div class="sec-ctrl">Controls</div>
  <div class="ctrl-grid">
    <button class="cbtn relay-off" id="bRelay" onclick="tRelay()">
      <div><div class="cl">Charging (GPIO 2)</div><div class="cv" id="relayTxt">Off</div></div>
      <span class="tag" id="relayTag">OFF</span>
    </button>
    <button class="cbtn arm-btn" id="bArm" onclick="tArm()">
      <div><div class="cl">Flight State</div><div class="cv" id="armTxt">Arm</div></div>
      <span class="tag" id="armTag">DISARMED</span>
    </button>
    <button class="cbtn mode-manual" id="bMode" onclick="tMode()">
      <div><div class="cl">Flight Mode</div><div class="cv" id="modeTxt">Manual</div></div>
      <span class="tag" id="modeTag">MANUAL</span>
    </button>
    <button class="cbtn tkoff-standby" id="bTkoff" onclick="tTkoff()">
      <div><div class="cl">Takeoff</div><div class="cv" id="tkoffTxt">Standby</div></div>
      <span class="tag" id="tkoffTag">STANDBY</span>
    </button>
  </div>

  <!-- Image upload -->
  <div class="img-sec">
    <div class="sec" style="margin-bottom:10px">Image Transfer</div>
    <div class="img-drop">
      <input type="file" id="imgFile" accept="image/*" onchange="onFile(this)">
      <div class="img-drop-icon">&#128247;</div>
      <div class="img-drop-txt">Tap to select an image</div>
      <div class="img-fname" id="fname"></div>
    </div>
    <img id="preview" alt="preview">
    <button class="send-btn" id="sendBtn" onclick="sendImage()" disabled>
      &#8593;&nbsp;Send Image to Drone
    </button>
    <div class="prog-wrap" id="progWrap">
      <div class="prog-bar"><div class="prog-fill" id="progFill"></div></div>
      <div class="prog-txt" id="progTxt"></div>
    </div>
    <div class="up-status" id="upStatus"></div>
  </div>

</div>
<script>
var armed=false,modeAuto=false,tkReady=false,isSending=false,selectedFile=null;

function tRelay(){
  fetch('/relay').then(r=>r.json()).then(d=>{
    var on=d.relay;
    document.getElementById('bRelay').className='cbtn '+(on?'relay-on':'relay-off');
    document.getElementById('relayTxt').textContent=on?'On':'Off';
    document.getElementById('relayTag').textContent=on?'ON':'OFF';
  });
}
function tArm(){
  armed=!armed;
  document.getElementById('bArm').className='cbtn '+(armed?'disarm-btn':'arm-btn');
  document.getElementById('armTxt').textContent=armed?'Disarm':'Arm';
  document.getElementById('armTag').textContent=armed?'ARMED':'DISARMED';
}
function tMode(){
  modeAuto=!modeAuto;
  document.getElementById('bMode').className='cbtn '+(modeAuto?'mode-auto':'mode-manual');
  document.getElementById('modeTxt').textContent=modeAuto?'Autonomous':'Manual';
  document.getElementById('modeTag').textContent=modeAuto?'AUTO':'MANUAL';
}
function tTkoff(){
  tkReady=!tkReady;
  document.getElementById('bTkoff').className='cbtn '+(tkReady?'tkoff-ready':'tkoff-standby');
  document.getElementById('tkoffTxt').textContent=tkReady?'Ready':'Standby';
  document.getElementById('tkoffTag').textContent=tkReady?'READY':'STANDBY';
}

// Telemetry poll — skipped while sending image
function upd(){
  if(isSending) return;
  fetch('/data').then(r=>r.json()).then(d=>{
    document.getElementById('x').textContent=d.x.toFixed(1);
    document.getElementById('y').textContent=d.y.toFixed(1);
    document.getElementById('vx').textContent=d.vx.toFixed(1);
    document.getElementById('vy').textContent=d.vy.toFixed(1);
    document.getElementById('alt').textContent=d.alt.toFixed(1);
    var pc=Math.min(100,Math.max(0,d.percentage));
    document.getElementById('pct').textContent=pc.toFixed(0);
    var f=document.getElementById('bf');
    f.style.width=pc+'%';
    f.style.background=pc>60?'#4ade80':pc>30?'#fbbf24':'#f87171';
    document.getElementById('batst').textContent=pc>60?'Good':pc>30?'Low':'Critical';
  }).catch(()=>{});
}
setInterval(upd,500); upd();

function onFile(inp){
  if(!inp.files.length) return;
  selectedFile=inp.files[0];
  document.getElementById('fname').textContent=
    selectedFile.name+' ('+Math.round(selectedFile.size/1024)+' KB)';
  var pr=document.getElementById('preview');
  pr.src=URL.createObjectURL(selectedFile);
  pr.style.display='block';
  document.getElementById('sendBtn').disabled=false;
}

function setSt(msg,cls){
  var el=document.getElementById('upStatus');
  el.textContent=msg; el.className='up-status '+(cls||'');
}
function setProg(done,total){
  var pct=total?Math.round(done/total*100):0;
  document.getElementById('progFill').style.width=pct+'%';
  document.getElementById('progTxt').textContent=done+' / '+total+' chunks ('+pct+'%)';
}

async function sendImage(){
  if(!selectedFile||isSending) return;
  isSending=true;
  document.getElementById('sendBtn').disabled=true;
  document.getElementById('progWrap').style.display='block';
  setProg(0,1); setSt('Pausing telemetry…','st-busy');

  // 1. Tell firmware to open NRF TX and send START packet
  try{
    const r=await fetch('/img_start',{method:'POST'});
    if(!r.ok) throw new Error('start failed');
  }catch(e){
    setSt('Failed to start: '+e.message,'st-err');
    isSending=false; document.getElementById('sendBtn').disabled=false; return;
  }
  await new Promise(r=>setTimeout(r,150)); // give NRF time to settle

  // 2. Slice file into 25-byte payloads, wrap in 32-byte packets
  const buf=await selectedFile.arrayBuffer();
  const bytes=new Uint8Array(buf);
  const CHUNK=25;
  const total=Math.ceil(bytes.length/CHUNK);
  setSt('Sending '+total+' chunks…','st-busy');

  for(let i=0;i<total;i++){
    const slice=bytes.slice(i*CHUNK, Math.min((i+1)*CHUNK, bytes.length));
    // Build 32-byte packet
    const pkt=new Uint8Array(32);
    pkt[0]=1;                                      // PKT_DATA
    pkt[1]=i&0xFF;      pkt[2]=(i>>8)&0xFF;        // seqNum LE
    pkt[3]=total&0xFF;  pkt[4]=(total>>8)&0xFF;    // totalChunks LE
    pkt[5]=slice.length; pkt[6]=0;                 // chunkLen LE
    pkt.set(slice,7);                              // payload

    // Encode as hex string and send as form field.
    // server.arg("hex") on ESP32 works 100% for
    // application/x-www-form-urlencoded — no binary parsing needed.
    var hex='';
    for(var b=0;b<32;b++){
      var hx=pkt[b].toString(16);
      hex+=(hx.length<2?'0':'')+hx;
    }

    let ok=false;
    for(let retry=0;retry<4&&!ok;retry++){
      try{
        const res=await fetch('/img_chunk',{
          method:'POST',
          headers:{'Content-Type':'application/x-www-form-urlencoded'},
          body:'hex='+hex
        });
        if(res.ok) ok=true;
        else { const t=await res.text(); console.warn('chunk '+i+':',t); }
      }catch(e){ console.warn('chunk '+i+' err:',e); }
      if(!ok) await new Promise(r=>setTimeout(r,100));
    }

    if(!ok){
      setSt('Chunk '+i+' failed after 4 retries — aborted','st-err');
      await fetch('/img_end',{method:'POST'});
      isSending=false; document.getElementById('sendBtn').disabled=false; return;
    }

    setProg(i+1,total);
    // Small gap so ESP32 WebServer can breathe between chunks
    await new Promise(r=>setTimeout(r,40));
  }

  // 3. Send END packet
  await fetch('/img_end',{method:'POST'});
  setSt('Transfer complete! Telemetry resumed.','st-ok');
  isSending=false;
  document.getElementById('sendBtn').disabled=false;
}
</script>
</body></html>
)rawliteral";

// ============================================================
//  HTTP handlers
// ============================================================
void handleRoot()  { server.send(200,"text/html",INDEX_HTML); }

void handleData() {
  String j="{";
  j+="\"x\":"          +String(data.x,1)         +",";
  j+="\"y\":"          +String(data.y,1)         +",";
  j+="\"vx\":"         +String(data.vx,1)        +",";
  j+="\"vy\":"         +String(data.vy,1)        +",";
  j+="\"alt\":"        +String(data.alt,1)       +",";
  j+="\"percentage\":"+String(data.percentage,0);
  j+="}";
  server.send(200,"application/json",j);
}

void handleRelay(){
  relayState=!relayState;
  digitalWrite(RELAY_PIN,relayState?HIGH:LOW);
  String j=String("{\"relay\":")+( relayState?"true":"false")+"}";
  server.send(200,"application/json",j);
  Serial.print("Relay: "); Serial.println(relayState?"ON":"OFF");
}

// POST /img_start
// Switch NRF to TX on IMAGE_ADDR and fire START packet.
// NRF stays in TX mode until /img_end.
void handleImgStart(){
  imageUploading=true;
  Serial.println("[IMG] START");

  if(nrfOK){
    radio.stopListening();
    radio.openWritingPipe(IMAGE_ADDR);

    ImagePacket pkt; memset(&pkt,0,sizeof(pkt));
    pkt.type=PKT_START;
    radio.write(&pkt,PKT_SIZE);
    // Stay in TX mode — do NOT call startListening() here
  }
  server.send(200,"text/plain","ok");
}

// POST /img_chunk
// JS sends the 32-byte packet as a 64-char hex string in a form field.
// server.arg("hex") is 100% reliable on ESP32 WebServer for
// application/x-www-form-urlencoded — no binary stream parsing needed.
void handleImgChunk(){
  if(!server.hasArg("hex")){
    Serial.println("[IMG] missing hex arg");
    server.send(400,"text/plain","missing hex");
    return;
  }
  String hex = server.arg("hex");
  if(hex.length() < PKT_SIZE * 2){
    Serial.printf("[IMG] hex too short: %d\n", hex.length());
    server.send(400,"text/plain","hex too short");
    return;
  }

  // Decode hex string → 32 bytes
  uint8_t pktBuf[PKT_SIZE];
  for(int i=0;i<PKT_SIZE;i++){
    char hi = hex.charAt(i*2);
    char lo = hex.charAt(i*2+1);
    auto hexVal = [](char c) -> uint8_t {
      if(c>='0'&&c<='9') return c-'0';
      if(c>='a'&&c<='f') return c-'a'+10;
      if(c>='A'&&c<='F') return c-'A'+10;
      return 0;
    };
    pktBuf[i] = (hexVal(hi) << 4) | hexVal(lo);
  }

  if(nrfOK){
    bool ok = nrfSendPacket(pktBuf);
    if(!ok){
      server.send(500,"text/plain","nrf fail");
      return;
    }
  }

  uint16_t seq = pktBuf[1] | ((uint16_t)pktBuf[2] << 8);
  uint16_t tot = pktBuf[3] | ((uint16_t)pktBuf[4] << 8);
  Serial.printf("[IMG] chunk %u/%u OK\n", seq+1, tot);
  server.send(200,"text/plain","ok");
}

// POST /img_end
// Fire END packet, then switch NRF back to RX for telemetry.
void handleImgEnd(){
  if(nrfOK){
    ImagePacket pkt; memset(&pkt,0,sizeof(pkt));
    pkt.type=PKT_END;
    radio.write(&pkt,PKT_SIZE);
    delay(10);

    // Return to listening for telemetry
    radio.openReadingPipe(0,TELEM_ADDR);
    radio.startListening();
  }
  imageUploading=false;
  Serial.println("[IMG] END — telemetry resumed");
  server.send(200,"text/plain","ok");
}

// ============================================================
//  Setup & Loop
// ============================================================
void setup(){
  Serial.begin(115200);
  delay(2000);

  pinMode(RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,LOW);

  WiFi.softAP(ssid,password);
  Serial.print("AP: "); Serial.println(ssid);
  Serial.print("http://"); Serial.println(WiFi.softAPIP());

  server.on("/",          handleRoot);
  server.on("/data",      handleData);
  server.on("/relay",     handleRelay);
  server.on("/img_start", HTTP_POST, handleImgStart);
  server.on("/img_chunk", HTTP_POST, handleImgChunk);
  server.on("/img_end",   HTTP_POST, handleImgEnd);

  // Allow reading Content-Length inside handlers
  const char* hdrs[] = {"Content-Length", "Content-Type"};
  server.collectHeaders(hdrs, 2);

  server.begin();
  Serial.println("Web server started");

  SPI.begin(SCK_PIN,MISO_PIN,MOSI_PIN,CSN_PIN);
  if(!radio.begin()){
    Serial.println("NRF NOT DETECTED");
    nrfOK=false; return;
  }
  nrfOK=true;
  radio.setChannel(115);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0,TELEM_ADDR);
  radio.startListening();
  Serial.println("NRF ready");
}

void loop(){
  server.handleClient();

  // Only read telemetry when NOT in image upload session
  if(nrfOK && !imageUploading && radio.available()){
    radio.read(&data,sizeof(data));
    Serial.printf("X:%.1f Y:%.1f VX:%.1f VY:%.1f ALT:%.1f BAT:%.0f%%\n",
      data.x,data.y,data.vx,data.vy,data.alt,data.percentage);
  }
}
