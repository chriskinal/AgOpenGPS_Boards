

uint8_t data[16384];
uint8_t buffer[4096];
int bufferIndex = 0;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71, 0x0D, 0x0A };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71, 0x0D, 0x0A };

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = { 0x80, 0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC, 0x0D, 0x0A };
int8_t PGN_253_Size = sizeof(PGN_253) - 3;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC, 0x0D, 0x0A };
int8_t PGN_250_Size = sizeof(PGN_250) - 3;

int packetSize;


void autoSteerPacketPerser() {

  int size = Serial.available();
  if (size < 5) {
    return;
  }

  Serial.readBytes(buffer, size);
  for (int i = 0; i < size; i++) {
    data[bufferIndex] = buffer[i];
    bufferIndex++;
  }

  for (int i = 0; i < bufferIndex - 1; i++) {
    if (data[i] == 13 && data[i + 1] == 10) {
      parsePacket(data, i);
      for (int x = 0; x < bufferIndex - i - 1; x++) {
        data[x] = data[i + 2 + x];
      }
      bufferIndex -= i + 2;
      i = -1;
    }
  }
}


void parsePacket(uint8_t* packet, int size) {
  if (packet[0] == 0x80 && packet[1] == 0x81 && packet[2] == 0x7F)  //Data
  {
    switch (packet[3]) {
      case 0xFE:
        {
          if (!Autosteer_running) {
            break;
          }
          gpsSpeed = ((float)(packet[5] | packet[6] << 8)) * 0.1;
          gpsSpeedUpdateTimer = 0;

          prevGuidanceStatus = guidanceStatus;

          guidanceStatus = packet[7];
          guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

          //Bit 8,9    set point steer angle * 100 is sent
          steerAngleSetPoint = ((float)(packet[8] | ((int8_t)packet[9]) << 8)) * 0.01;  //high low bytes

          if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1)) {
            watchdogTimer = WATCHDOG_FORCE_VALUE;  //turn off steering motor
          } else                                   //valid conditions to turn on autosteer
          {
            watchdogTimer = 0;  //reset watchdog
          }

          //Bit 10 Tram
          tram = packet[10];

          //Bit 11
          relay = packet[11];

          //Bit 12
          relayHi = packet[12];

          //----------------------------------------------------------------------------
          //Serial Send to agopenGPS

          int16_t sa = (int16_t)(steerAngleActual * 100);

          PGN_253[5] = (uint8_t)sa;
          PGN_253[6] = sa >> 8;

          // heading
          PGN_253[7] = (uint8_t)9999;
          PGN_253[8] = 9999 >> 8;

          // roll
          PGN_253[9] = (uint8_t)8888;
          PGN_253[10] = 8888 >> 8;

          PGN_253[11] = switchByte;
          PGN_253[12] = (uint8_t)pwmDisplay;

          //checksum
          int16_t CK_A = 0;
          for (uint8_t i = 2; i < PGN_253_Size; i++)
            CK_A = (CK_A + PGN_253[i]);

          PGN_253[PGN_253_Size] = CK_A;

          sendData(PGN_253, sizeof(PGN_253));

          //Steer Data 2 -------------------------------------------------
          if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
            if (aog2Count++ > 2) {
              //Send fromAutosteer2
              PGN_250[5] = (byte)sensorReading;

              //add the checksum for AOG2
              CK_A = 0;

              for (uint8_t i = 2; i < PGN_250_Size; i++) {
                CK_A = (CK_A + PGN_250[i]);
              }

              PGN_250[PGN_250_Size] = CK_A;

              sendData(PGN_250, sizeof(PGN_250));
              aog2Count = 0;
            }
          }
          break;
        }
      //steer settings
      case 252:
        {  //0xFC
          if (!Autosteer_running) {
            return;
          }
          //PID values
          steerSettings.Kp = ((float)packet[5]);    // read Kp from AgOpenGPS
          steerSettings.highPWM = packet[6];        // read high pwm
          steerSettings.lowPWM = (float)packet[7];  // read lowPWM from AgOpenGPS
          steerSettings.minPWM = packet[8];         //read the minimum amount of PWM for instant on
          float temp = (float)steerSettings.minPWM * 1.2;
          steerSettings.lowPWM = (byte)temp;
          steerSettings.steerSensorCounts = packet[9];   //sent as setting displayed in AOG
          steerSettings.wasOffset = (packet[10]);        //read was zero offset Lo
          steerSettings.wasOffset |= (packet[11] << 8);  //read was zero offset Hi
          steerSettings.AckermanFix = (float)packet[12] * 0.01;

          //crc
          //autoSteerUdpData[13];

          //store in EEPROM
          EEPROM.put(10, steerSettings);
          EEPROM.commit();
          // Re-Init steer settings
          steerSettingsInit();
          break;
        }
      case 251:  //251 FB - SteerConfig
        {
          uint8_t sett = packet[5];  //setting0

          if (bitRead(sett, 0)) steerConfig.InvertWAS = 1;
          else steerConfig.InvertWAS = 0;
          if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1;
          else steerConfig.IsRelayActiveHigh = 0;
          if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1;
          else steerConfig.MotorDriveDirection = 0;
          if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1;
          else steerConfig.SingleInputWAS = 0;
          if (bitRead(sett, 4)) steerConfig.CytronDriver = 1;
          else steerConfig.CytronDriver = 0;
          if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1;
          else steerConfig.SteerSwitch = 0;
          if (bitRead(sett, 6)) steerConfig.SteerButton = 1;
          else steerConfig.SteerButton = 0;
          if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1;
          else steerConfig.ShaftEncoder = 0;

          steerConfig.PulseCountMax = packet[6];

          //was speed
          //autoSteerUdpData[7];

          sett = packet[8];  //setting1 - Danfoss valve etc

          if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1;
          else steerConfig.IsDanfoss = 0;
          if (bitRead(sett, 1)) steerConfig.PressureSensor = 1;
          else steerConfig.PressureSensor = 0;
          if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1;
          else steerConfig.CurrentSensor = 0;
          if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1;
          else steerConfig.IsUseY_Axis = 0;

          //crc
          //autoSteerUdpData[13];

          EEPROM.put(40, steerConfig);
          // Re-Init
        }
      case 200:
        {  // Hello from AgIO

          if (Autosteer_running) {
            int16_t sa = (int16_t)(steerAngleActual * 100);

            helloFromAutoSteer[5] = (uint8_t)sa;
            helloFromAutoSteer[6] = sa >> 8;

            helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
            helloFromAutoSteer[8] = helloSteerPosition >> 8;
            helloFromAutoSteer[9] = switchByte;

            sendData(helloFromAutoSteer, sizeof(helloFromAutoSteer));
          }
        }
      case 202:
        {
          //make really sure this is the reply pgn
          if (packet[4] == 3 && packet[5] == 202 && packet[6] == 202) {
            //hello from AgIO
            uint8_t scanReply[] = { 128, 129, 126, 203, 7,
                                    0, 0, 0, 0,
                                    0, 0, 0, 23 };

            //checksum
            int16_t CK_A = 0;
            for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++) {
              CK_A = (CK_A + scanReply[i]);
            }
            scanReply[sizeof(scanReply) - 1] = CK_A;

            sendData(scanReply, sizeof(scanReply));
          }
        }
      case 100: //NTRIP
        {
          uint8_t NTRIPData[size-4];
          
          for (int i = 4; i <size ;i++ )
          {
            NTRIPData[i-4] = packet[i];
          }
          Serial2.write(NTRIPData, size-4);
        }
    }
  } else {
    Serial2.println("Unknown packet!!!");
  }
}

void sendData(uint8_t* data, uint8_t datalen) {
  Serial.write(data, datalen);
}
