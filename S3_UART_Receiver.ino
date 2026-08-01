---
*** Begin Patch
*** Update File: S3_UART_Receiver.ino
@@
-  // Connect MQTT if needed
-    if (!s_mqtt_client.connected()) {
-      DBG.println("MQTT: connecting...");
-      bool ok;
-      if (mqtt_username != nullptr && mqtt_username[0] != '\0') {
-        ok = s_mqtt_client.connect("S3_UART_Receiver", mqtt_username, mqtt_password);
-      } else {
-        ok = s_mqtt_client.connect("S3_UART_Receiver");
-      }
-      if (!ok) {
-        DBG.printf("MQTT: connect failed, rc=%d\n", s_mqtt_client.state());
-        vTaskDelay(pdMS_TO_TICKS(2000));
-        continue;
-      }
-      DBG.println("MQTT: connected");
-    }
+    // Connect MQTT if needed
+    if (!s_mqtt_client.connected()) {
+      // Diagnostics: print WiFi and broker info before trying to connect
+      DBG.print("WiFi status: "); DBG.print(WiFi.status()); DBG.print(" ip="); DBG.println(WiFi.localIP());
+      // Try to resolve broker hostname to help diagnose DNS issues
+      IPAddress broker_ip;
+      bool resolved = false;
+      if (mqtt_broker != nullptr && mqtt_broker[0] != '\0') {
+        resolved = WiFi.hostByName(mqtt_broker, broker_ip);
+        if (resolved) {
+          DBG.print("Resolved broker "); DBG.print(mqtt_broker); DBG.print(" -> "); DBG.println(broker_ip);
+          s_mqtt_client.setServer(broker_ip, mqtt_port);
+        } else {
+          DBG.print("Could not resolve broker "); DBG.println(mqtt_broker);
+          s_mqtt_client.setServer(mqtt_broker, mqtt_port);
+        }
+      }
+
+      DBG.println("MQTT: connecting...");
+      bool ok;
+      // Use a simple client id; if multiple devices connect, consider making this unique
+      const char *clientId = "S3_UART_Receiver";
+      if (mqtt_username != nullptr && mqtt_username[0] != '\0') {
+        ok = s_mqtt_client.connect(clientId, mqtt_username, mqtt_password);
+      } else {
+        ok = s_mqtt_client.connect(clientId);
+      }
+      if (!ok) {
+        int state = s_mqtt_client.state();
+        DBG.printf("MQTT: connect failed, state=%d\n", state);
+        // Helpful mapping for PubSubClient states (not exhaustive)
+        switch (state) {
+          case -1: DBG.println("MQTT: connection timeout (no response)"); break;
+          case 0: DBG.println("MQTT: connection successful (shouldn't see this on failure)"); break;
+          case 1: DBG.println("MQTT: connection refused - incorrect protocol version"); break;
+          case 2: DBG.println("MQTT: connection refused - invalid client identifier"); break;
+          case 3: DBG.println("MQTT: connection refused - server unavailable"); break;
+          case 4: DBG.println("MQTT: connection refused - bad username or password"); break;
+          case 5: DBG.println("MQTT: connection refused - not authorized"); break;
+          default: DBG.println("MQTT: unknown connect error"); break;
+        }
+        vTaskDelay(pdMS_TO_TICKS(2000));
+        continue;
+      }
+      DBG.println("MQTT: connected");
+    }
*** End Patch
