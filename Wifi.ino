/**
 * Copyright 2017 Dan Oprescu
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *     
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <WiFi.h>
#include <ESP32WebServer.h>

ESP32WebServer server(80);

void displayInfo() {
  Serial.println("Handle Display Info...");
  String message = "<h1>Self balancing robot\n</h1><h2> ";
  message += "<p>Position=" + String(currentPos) + "  <a href=\"update?Pos=inc\"><button>Pos++</button></a><a href=\"update?Pos=dec\"><button>Pos--</button></a></p>";
  message += "<p>Kp=" + String(Kp) + "  <a href=\"update?Kp=inc\"><button>Kp++</button></a><a href=\"update?Kp=dec\"><button>Kp--</button></a></p>";
  message += "<p>Ki=" + String(Ki) + "  <a href=\"update?Ki=inc\"><button>Ki++</button></a><a href=\"update?Ki=dec\"><button>Ki--</button></a></p>";
  message += "<p>Kd=" + String(Kd) + "  <a href=\"update?Kd=inc\"><button>Kd++</button></a><a href=\"update?Kd=dec\"><button>Kd--</button></a></p>";
  message += "<p>SP=" + String(angleSetpoint) + "  <a href=\"update?Sp=inc\"><button>SP++</button></a><a href=\"update?Sp=dec\"><button>SP--</button></a></p></h2>";
  server.send(200, "text/html", message);
}


void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: " + server.uri();
  message += "\nMethod: " + (server.method() == HTTP_GET)?" GET":" POST";
  message += "\nArguments: " + server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}


void updateVars() {
  // we expect one and only 1 var which is the name of what we need to change, and the value is "inc" or "dec"
  String var = server.argName(0);
  String action = server.arg(0);

  Serial.println("Handle " + action + " to " + var);

  if (var == "Pos") currentPos = currentPos + (action == "inc" ? 1000 : -1000);
  else if(var == "Kp") Kp = Kp + (action == "inc" ? BASE_Kp * 0.1 : -BASE_Kp * 0.1);
  else if(var == "Ki") Ki = Ki + (action == "inc" ? BASE_Ki * 0.1 : -BASE_Ki * 0.1);
  else if(var == "Kd") Kd = Kd + (action == "inc" ? BASE_Kd * 0.1 : -BASE_Kd * 0.1);
  else if(var == "Sp") angleSetpoint = angleSetpoint + (action == "inc" ? 0.1 : -0.1);
  else Serial.println("UNKNOWN var " + var );

  displayInfo();
}



void wifiLoop(void *params) {
  Serial.println("Starting thread dealing with Wifi/HTTP client requests...");
  while(true) {
    // deals with the Wifi clients and responds, calls the callbacks, etc. ...
    server.handleClient();
    delay(1);
  }
}

void setup_wifi() {
  Serial.println("\nConnecting to Wifi");
  WiFi.begin("xxx", "yyy");
  /*wait until ESP32 connect to WiFi*/
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.print("\nWiFi connected with IP address: "); Serial.println(WiFi.localIP());


  server.onNotFound(handleNotFound);
  server.on("/", displayInfo);
  server.on("/update", updateVars);
  
  
  server.begin();
  Serial.println("HTTP server started");

  // deal with WiFi/HTTP requests in a separate thread, to avoid impacting the real time balancing
  // lower number means lower priority. 1 is just above tskIDLE_PRIORITY == 0  which is the lowest priority
  // use same core as rest of Arduino code as the other one is for system tasks
  xTaskCreatePinnedToCore(wifiLoop, "wifiLoop", 4096, NULL, 1, NULL, xPortGetCoreID());
}



