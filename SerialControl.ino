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
 

HardwareSerial SerialControl(1);

/////////////// AUREL Wireless Commands /////////////////////////////////////

uint8_t _prevChar;
boolean _readingMsg = false;
uint8_t _msg[6];
uint8_t _msgPos;
boolean _validData = false;


boolean startNewMsg(uint8_t c) {
  boolean res = (_prevChar == 0) && (c == 255);
  _prevChar = c;
  return res;
}


void serialControlLoop(void *params) {
  Serial.println("\nStarting thread dealing with Serial Control requests...");
  uint8_t currChar;
  
  while(true) {
    while (SerialControl.available()) {
      currChar = SerialControl.read();
  
      if (startNewMsg(currChar)) {
        _readingMsg = true;
        _msgPos = 0;
      } else if (_readingMsg) {
        if (_msgPos >= 6) {
          // data finished, last byte is the CRC
          uint8_t crc = 0;
          for (uint8_t i = 0; i < 6; i++)
            crc += _msg[i];
  
          if (crc == currChar) {
            joystickX = _msg[0];
            joystickY = _msg[1];
            _validData = true;
          } else {
            _validData = false;
            Serial.print("Wrong CRC: ");Serial.print(currChar);Serial.print(" Expected: ");Serial.println(crc);
          }
  
          _readingMsg = false;
        } else {
          // normal data, add it to the message
          _msg[_msgPos++] = currChar;
        }
      }
    }
    
    delay(1);
  }
}

void setup_serial_control() {
  SerialControl.begin(9600, SERIAL_8N1, 17, 16);

  // deal with control requests in a separate thread, to avoid impacting the real time balancing
  // lower number means lower priority. 1 is just above tskIDLE_PRIORITY == 0  which is the lowest priority
  // use same core as rest of Arduino code as the other one is for system tasks
  xTaskCreatePinnedToCore(serialControlLoop, "serialControlLoop", 4096, NULL, 2, NULL, xPortGetCoreID());
}


boolean isValidJoystickValue(uint8_t joystick) {
  return joystick > 20 && joystick < 230;
}

