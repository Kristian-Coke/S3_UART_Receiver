#include <Arduino.h>
#include "wifi_manager.h"
#include "model_settings.h"
#include "config.h"
#include <PubSubClient.h>
#include <WiFi.h>

static constexpr int kDebugBaud = 921600;
static constexpr int kUartBaud = 921600;
static constexpr int kUartRxPin = 44;
static constexpr int kUartTxPin = 43;

static constexpr int kP4DetectAdcPin = 8;
static constexpr int kP4DetectThreshold = 2000;

static constexpr uint8_t kSync0 = 0xAA;
static constexpr uint8_t kSync1 = 0x55;
static constexpr uint8_t kMsgTypeInference = 0x01;
static constexpr uint8_t kMsgTypeControl   = 0x02;  // S3 -> P4 control messages

// S3 -> P4 control commands
static constexpr uint8_t kCtrlAckStop        = 0x01;  // confirmed sign -> stop TX
static constexpr uint8_t kCtrlResumeJunction = 0x02;  // done tasks -> resume + junction

// Sign confirmation: require N consecutive frames of the same sign class
// with confidence above threshold before sending ACK_STOP.
// Threshold is set low (120/255 ≈ 47%) because int8-quantized models on
// ESP32-P4 often produce subdued confidence scores.  Tune per model.
static constexpr int kSignConfirmFrames = 5;
static constexpr uint8_t kSignConfirmConfidence = 120;  // 120/255 ≈ 47%

#define UartFromP4 Serial0
#define DBG Serial

struct Packet {
  uint8_t msg_type;
  uint16_t frame_id;
  uint8_t label_id;
  uint8_t confidence;
  uint8_t flags;
};

static uint32_t s_bytes_rx = 0;
static uint32_t s_packets_ok = 0;
static uint32_t s_packets_bad = 0;
static bool s_p4_connected = false;
static bool s_uart_started = false;

// Sign confirmation state machine
static bool s_sign_confirmed = false;
static uint8_t s_last_sign_label = 0xFF;
static int s_consecutive_sign_frames = 0;

// After ACK_STOP, S3 "does its tasks" for this many ms before sending RESUME_JUNCTION.
// Adjust to match your real task duration (e.g. display, logging, actuator control).
static constexpr uint32_t kTaskDurationMs = 3000;

static constexpr int kP4DetectSamples = 16;

// MQTT globals
static WiFiClient s_wifi_client;
static PubSubClient s_mqtt_client(s_wifi_client);
static QueueHandle_t s_mqtt_queue = nullptr;  // queue of Packet

static uint8_t calc_uart_checksum(const uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

static void send_control_packet(uint8_t cmd) {
  uint8_t buf[5];
  buf[0] = kSync0;          // 0xAA
  buf[1] = kSync1;          // 0x55
  buf[2] = kMsgTypeControl; // 0x02
  buf[3] = cmd;
  buf[4] = calc_uart_checksum(buf, 4);  // XOR of bytes 0-3
  UartFromP4.write(buf, sizeof(buf));
  UartFromP4.flush();

  const char *name = (cmd == kCtrlAckStop) ? "ACK_STOP" :
                     (cmd == kCtrlResumeJunction) ? "RESUME_JUNCTION" : "???";
  DBG.printf("S3 -> P4: %s (0x%02X)\n", name, cmd);
}

static int read_adc_avg() {
  int32_t sum = 0;
  for (int i = 0; i < kP4DetectSamples; i++) {
    sum += analogRead(kP4DetectAdcPin);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return (int)(sum / kP4DetectSamples);
}

static void wait_for_p4_connected_blocking() {
  pinMode(kP4DetectAdcPin, INPUT);

  uint32_t last_log_ms = 0;
  while (!s_p4_connected) {
    const int v = read_adc_avg();
      // DBG.print("adc=");
      // DBG.println(v);
    if (v >= kP4DetectThreshold) {
      s_p4_connected = true;
      DBG.print("P4 detected adc=");
      DBG.println(v);
      break;
    }
    const uint32_t now = millis();
    if (now - last_log_ms > 500) {
      last_log_ms = now;
      DBG.print("Waiting for P4 adc=");
      DBG.println(v);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static bool read_packet(Packet *out) {
  static uint8_t buf[9];
  static uint8_t idx = 0;
  static uint8_t state = 0;

  while (UartFromP4.available() > 0) {
    const uint8_t b = (uint8_t)UartFromP4.read();
    s_bytes_rx++;

    if (state == 0) {
      if (b == kSync0) {
        buf[0] = b;
        idx = 1;
        state = 1;
      }
      continue;
    }

    if (state == 1) {
      if (b == kSync1) {
        buf[1] = b;
        idx = 2;
        state = 2;
      } else if (b == kSync0) {
        buf[0] = b;
        idx = 1;
        state = 1;
      } else {
        state = 0;
      }
      continue;
    }

    buf[idx++] = b;
    if (idx < sizeof(buf)) {
      continue;
    }

    state = 0;
    idx = 0;

    if (buf[2] != kMsgTypeInference) {
      s_packets_bad++;
      return false;
    }

    if (calc_uart_checksum(buf, 8) != buf[8]) {
      s_packets_bad++;
      return false;
    }

    out->msg_type = buf[2];
    out->frame_id = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
    out->label_id = buf[5];
    out->confidence = buf[6];
    out->flags = buf[7];
    return true;
  }

  return false;
}

// MQTT task: connects to broker and publishes Packet objects received on s_mqtt_queue
static void mqtt_task(void *arg) {
  (void)arg;
  // configure server
  s_mqtt_client.setServer(mqtt_broker, mqtt_port);

  for (;;) {
    // Ensure WiFi is up
    if (!WiFiManager::isConnected()) {
      WiFiManager::checkConnectionStatus();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // Connect MQTT if needed
    if (!s_mqtt_client.connected()) {
      DBG.println("MQTT: connecting...");
      bool ok;
      if (mqtt_username != nullptr && mqtt_username[0] != '\0') {
        ok = s_mqtt_client.connect("S3_UART_Receiver", mqtt_username, mqtt_password);
      } else {
        ok = s_mqtt_client.connect("S3_UART_Receiver");
      }
      if (!ok) {
        DBG.printf("MQTT: connect failed, rc=%d\n", s_mqtt_client.state());
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
      DBG.println("MQTT: connected");
    }

    // Wait for packets to publish, but call loop frequently
    Packet pkt;
    if (xQueueReceive(s_mqtt_queue, &pkt, pdMS_TO_TICKS(500)) == pdTRUE) {
      char payload[128];
      const char *label = "Unknown";
      if (pkt.label_id < kCategoryCount) label = kCategoryLabels[pkt.label_id];
      snprintf(payload, sizeof(payload), "{\"frame\":%u,\"label\":%u,\"label_name\":\"%s\",\"confidence\":%u,\"flags\":%u}",
               (unsigned)pkt.frame_id, (unsigned)pkt.label_id, label, (unsigned)pkt.confidence, (unsigned)pkt.flags);
      const char *pub_topic = (topic != nullptr && topic[0] != '\0') ? topic : "s3/packets";
      if (!s_mqtt_client.publish(pub_topic, payload)) {
        DBG.println("MQTT: publish failed");
      } else {
        DBG.print("MQTT: published to "); DBG.print(pub_topic); DBG.print(": "); DBG.println(payload);
      }
    }

    s_mqtt_client.loop();
  }
}

static void detect_task(void *arg) {
  (void)arg;
  wait_for_p4_connected_blocking();
  vTaskDelete(nullptr);
}

static void uart_task(void *arg) {
  (void)arg;
  while (!s_p4_connected) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (!s_uart_started) {
    s_uart_started = true;
    UartFromP4.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
    DBG.printf("UART started rx=%d tx=%d baud=%d\n", kUartRxPin, kUartTxPin, kUartBaud);
  }

  uint32_t last_avail_ms = 0;
  for (;;) {
    Packet p;
    if (read_packet(&p)) {
      s_packets_ok++;
      const char *label = "Unknown";
      if (p.label_id < kCategoryCount) {
        label = kCategoryLabels[p.label_id];
      }

      DBG.print("frame=");
      DBG.print(p.frame_id);
      DBG.print(" label=");
      DBG.print(p.label_id);
      DBG.print("(");
      DBG.print(label);
      DBG.print(")");
      DBG.print(" conf=");
      DBG.print(p.confidence);

      // push packet to MQTT queue (non-blocking)
      if (s_mqtt_queue) {
        if (xQueueSend(s_mqtt_queue, &p, 0) != pdTRUE) {
          DBG.println("MQTT queue full, dropping packet");
        }
      }

      // ── Sign confirmation state machine ──
      // With the B-G pipeline every frame is a sign-detection frame
      // (no junction/sign phase distinction).  Confirm when the same
      // label appears for N consecutive frames with confidence above threshold.
      if (!s_sign_confirmed && p.confidence >= kSignConfirmConfidence) {
        if (p.label_id == s_last_sign_label) {
          s_consecutive_sign_frames++;
        } else {
          s_last_sign_label = p.label_id;
          s_consecutive_sign_frames = 1;
        }

        if (s_consecutive_sign_frames >= kSignConfirmFrames) {
          s_sign_confirmed = true;
          DBG.printf("SIGN CONFIRMED: label=%d(%s) conf=%d after %d frames\n",
                     p.label_id, label, p.confidence, s_consecutive_sign_frames);

          // Step 1: Tell P4 to stop transmitting
          bool p4_stopped = false;
          for (int retry = 0; retry < 3 && !p4_stopped; retry++) {
            if (retry > 0) {
              DBG.printf("S3: ACK_STOP retry %d/3\n", retry + 1);
              vTaskDelay(pdMS_TO_TICKS(50));
            }
            send_control_packet(kCtrlAckStop);
            uint32_t wait_start = millis();
            while (millis() - wait_start < 300) {
              vTaskDelay(pdMS_TO_TICKS(30));
              while (UartFromP4.available() > 0) (void)UartFromP4.read();
              vTaskDelay(pdMS_TO_TICKS(50));
              if (UartFromP4.available() == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (UartFromP4.available() == 0) {
                  p4_stopped = true;
                  DBG.printf("S3: P4 stopped TX after %lu ms (retry=%d)\n",
                             (unsigned long)(millis() - wait_start), retry);
                  break;
                }
              }
            }
          }
          if (!p4_stopped)
            DBG.println("S3: WARNING — P4 did not stop TX after 3 ACK_STOP retries!");

          // Step 2: Do our tasks
          DBG.printf("S3: performing tasks (%lu ms)...\n", (unsigned long)kTaskDurationMs);
          vTaskDelay(pdMS_TO_TICKS(kTaskDurationMs));

          // Step 3: Tell P4 to resume
          bool p4_resumed = false;
          for (int retry = 0; retry < 3 && !p4_resumed; retry++) {
            if (retry > 0) {
              DBG.printf("S3: RESUME_JUNCTION retry %d/3\n", retry + 1);
              vTaskDelay(pdMS_TO_TICKS(100));
            }
            send_control_packet(kCtrlResumeJunction);
            uint32_t wait_start = millis();
            while (millis() - wait_start < 500) {
              vTaskDelay(pdMS_TO_TICKS(20));
              if (UartFromP4.available() > 0) {
                p4_resumed = true;
                DBG.printf("S3: P4 resumed TX after %lu ms (retry=%d)\n",
                           (unsigned long)(millis() - wait_start), retry);
                break;
              }
            }
          }
          if (!p4_resumed)
            DBG.println("S3: WARNING — P4 did not resume TX after 3 retries!");

          // Reset for next cycle
          s_sign_confirmed = false;
          s_last_sign_label = 0xFF;
          s_consecutive_sign_frames = 0;
          DBG.println("S3: cycle complete, ready for next sign");
        }
      } else {
        // Confidence too low or different label — reset counter
        s_last_sign_label = 0xFF;
        s_consecutive_sign_frames = 0;
      }
    } else {
      const uint32_t now = millis();
      if (now - last_avail_ms >= 1000) {
        last_avail_ms = now;
        DBG.printf("UART rx_avail=%d bytes=%lu ok=%lu bad=%lu\n", UartFromP4.available(),
                   (unsigned long)s_bytes_rx, (unsigned long)s_packets_ok,
                   (unsigned long)s_packets_bad);
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

static void heartbeat_task(void *arg) {
  (void)arg;
  for (;;) {
    DBG.printf("S3 alive connected=%d bytes=%lu ok=%lu bad=%lu\n", (int)s_p4_connected,
               (unsigned long)s_bytes_rx, (unsigned long)s_packets_ok,
               (unsigned long)s_packets_bad);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {

  // Initialize the Wi-Fi Connection
  // LED blinking while connecting to WiFi
  WiFiManager::initialize();

  DBG.begin(kDebugBaud);
  vTaskDelay(pdMS_TO_TICKS(200));
  DBG.println("S3 UART receiver start");

  // Create MQTT queue and task
  s_mqtt_queue = xQueueCreate(16, sizeof(Packet));
  if (s_mqtt_queue != nullptr) {
    xTaskCreate(mqtt_task, "mqtt", 8192, nullptr, 2, nullptr);
  } else {
    DBG.println("Failed to create MQTT queue");
  }

  // xTaskCreate(heartbeat_task, "hb", 4096, nullptr, 1, nullptr);
  xTaskCreate(detect_task, "detect", 4096, nullptr, 2, nullptr);
  
  xTaskCreate(uart_task, "uart", 8192, nullptr, 2, nullptr);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
