#include <Arduino.h>
#include <MAVLink.h>
#include <SPI.h>
#include <RF24.h>

#define RXD1 18  
#define TXD1 17  

// --- NRF PINS (Custom) ---
#define CE_PIN   39
#define CSN_PIN  40
#define SCK_PIN  41
#define MISO_PIN 2
#define MOSI_PIN 42

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// --- DATA STRUCT ---
struct TelemetryData {
  float x, y;
  float vx, vy;
  float alt;
  float voltage;
  float current;
  float percentage;
};

TelemetryData data;

// --- BATTERY CONFIGURATION ---
const int CELLS = 3;
const float FULL_VOLTS = 4.2 * CELLS;
const float EMPTY_VOLTS = 3.5 * CELLS;

// --- SEND FUNCTION ---
void sendTelemetry() {
  bool ok = radio.write(&data, sizeof(data));
  Serial.print(" | NRF: ");
  Serial.println(ok ? "OK" : "FAIL");
}

// --- MAVLINK REQUEST ---
void request_data_streams() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Request Position (X, Y, VX, VY)
  mavlink_msg_request_data_stream_pack(1, 200, &msg, 1, 0, MAV_DATA_STREAM_POSITION, 10, 1);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
  
  // Request Battery and Rangefinder (Extra3 usually contains distance sensor)
  mavlink_msg_request_data_stream_pack(1, 200, &msg, 1, 0, MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);

  mavlink_msg_request_data_stream_pack(1, 200, &msg, 1, 0, MAV_DATA_STREAM_EXTRA3, 5, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(57600, SERIAL_8N1, RXD1, TXD1);

  // --- NRF INIT ---
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);

  if (!radio.begin()) {
    Serial.println("NRF FAIL");
    while (1);
  }

  radio.setChannel(115);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.stopListening();

  delay(3000); 
  request_data_streams();
}

void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      
      switch (msg.msgid) {
        
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
          mavlink_local_position_ned_t lpos;
          mavlink_msg_local_position_ned_decode(&msg, &lpos);
          
          data.x = lpos.x * 100.0;
          data.y = lpos.y * 100.0;
          data.vx = lpos.vx * 100.0;
          data.vy = lpos.vy * 100.0;

          Serial.print("X: "); Serial.print(data.x, 1);
          Serial.print("cm | Y: "); Serial.print(data.y, 1);
          Serial.print("cm | VX: "); Serial.print(data.vx, 1);
          Serial.print("cm/s | VY: "); Serial.print(data.vy, 1);
          Serial.print("cm/s");
          break;
        }

        // --- MERGED DISTANCE SENSOR CASE ---
        case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
          mavlink_distance_sensor_t dist;
          mavlink_msg_distance_sensor_decode(&msg, &dist);
          
          // Only update if valid range. MAVLink DISTANCE_SENSOR is already in cm.
          if (dist.current_distance > 0 && dist.current_distance < 2000) {
              data.alt = (float)dist.current_distance; 
              Serial.print(" | RANGE ALT: "); Serial.print(data.alt, 1); Serial.print("cm");
          }
          break;
        }

        case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: {
          mavlink_optical_flow_rad_t flow;
          mavlink_msg_optical_flow_rad_decode(&msg, &flow);

          // Only update if the Distance Sensor message hasn't already provided data
          data.alt = flow.distance * 100.0;
          Serial.print(" | FLOW ALT: "); Serial.print(data.alt, 1); Serial.print("cm");
          break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS: {
          mavlink_sys_status_t sys_status;
          mavlink_msg_sys_status_decode(&msg, &sys_status);
          
          data.voltage = sys_status.voltage_battery / 1000.0;
          data.current = sys_status.current_battery / 100.0;

          float percentage = ((data.voltage - EMPTY_VOLTS) / (FULL_VOLTS - EMPTY_VOLTS)) * 100.0;
          
          if (percentage > 100) percentage = 100;
          if (percentage < 0) percentage = 0;

          data.percentage = percentage;

          Serial.print(" | BAT: "); Serial.print(data.voltage, 2); Serial.print("V");
          Serial.print(" | CUR: "); Serial.print(data.current, 2); Serial.print("A");
          Serial.print(" | "); Serial.print(data.percentage, 0); Serial.print("%");

          // Final update and broadcast via NRF
          sendTelemetry();
          break;
        }
      }
    }
  }
}
