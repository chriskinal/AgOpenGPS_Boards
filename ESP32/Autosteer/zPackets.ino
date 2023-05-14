uint8_t* autoSteerUdpData;
//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = { 0x80, 0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;

unsigned int AOGNtripPort = 2233;                 // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;             // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;              // Port of AOG that listens
IPAddress ipDes = IPAddress(255, 255, 255, 255);  //AOG IP

AsyncUDP udp;
AsyncUDP ntrip;

void startUDP() {

  if (udp.listen(AOGAutoSteerPort)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {
      autoSteerPacketPerser(packet);
    });
  }

  if (ntrip.listen(AOGNtripPort)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    ntrip.onPacket([](AsyncUDPPacket packet) {
      ntripPacketProxy(packet);
    });
  }
}

void initWifi(){
   // Create WiFiManager object
  WiFiManager wfm;
  // Supress Debug information
  wfm.setDebugOutput(false);

  if (!wfm.autoConnect("AGOpenGPS Autosteer")) {
    // Did not connect, print error message
    Serial.println("failed to connect and hit timeout");

    // Reset and try again
    ESP.restart();
    delay(1000);
  }

  myip = WiFi.localIP();
  // Connected!
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  startUDP();
}

void ntripPacketProxy(AsyncUDPPacket packet) {
  if (packet.length() > 0) {
    Serial2.write(packet.data(), packet.length());
  }
}

void autoSteerPacketPerser(AsyncUDPPacket packet) {
  if (packet.length() < 5){
    Serial.println("Packet lenght error!!");
    return;
  }
  autoSteerUdpData = packet.data();
  if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F)  //Data
  {
    switch (autoSteerUdpData[3]) {
      case 0xFE:
        {
          if (!Autosteer_running) {
            break;
          }

          gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
          gpsSpeedUpdateTimer = 0;

          prevGuidanceStatus = guidanceStatus;

          guidanceStatus = autoSteerUdpData[7];
          guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

          //Bit 8,9    set point steer angle * 100 is sent
          steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01;  //high low bytes

          //Serial.print("steerAngleSetPoint: ");
          //Serial.println(steerAngleSetPoint);

          //Serial.println(gpsSpeed);

          if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1)) {
            watchdogTimer = WATCHDOG_FORCE_VALUE;  //turn off steering motor
          } else                                   //valid conditions to turn on autosteer
          {
            watchdogTimer = 0;  //reset watchdog
          }

          //Bit 10 Tram
          tram = autoSteerUdpData[10];

          //Bit 11
          relay = autoSteerUdpData[11];

          //Bit 12
          relayHi = autoSteerUdpData[12];

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

          sendUdp(PGN_253, sizeof(PGN_253));

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

              sendUdp(PGN_250, sizeof(PGN_250));
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
          steerSettings.Kp = ((float)autoSteerUdpData[5]);  // read Kp from AgOpenGPS
          steerSettings.highPWM = autoSteerUdpData[6];  // read high pwm
          steerSettings.lowPWM = (float)autoSteerUdpData[7];  // read lowPWM from AgOpenGPS
          steerSettings.minPWM = autoSteerUdpData[8];  //read the minimum amount of PWM for instant on
          float temp = (float)steerSettings.minPWM * 1.2;
          steerSettings.lowPWM = (byte)temp;
          steerSettings.steerSensorCounts = autoSteerUdpData[9];  //sent as setting displayed in AOG
          steerSettings.wasOffset = (autoSteerUdpData[10]);  //read was zero offset Lo
          steerSettings.wasOffset |= (autoSteerUdpData[11] << 8);  //read was zero offset Hi
          steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;

          //crc
          //autoSteerUdpData[13];

          //store in EEPROM
          EEPROM.put(10, steerSettings);

          // Re-Init steer settings
          steerSettingsInit();
          break;
        }
      case 251:  //251 FB - SteerConfig
        {
          uint8_t sett = autoSteerUdpData[5];  //setting0

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

          steerConfig.PulseCountMax = autoSteerUdpData[6];

          //was speed
          //autoSteerUdpData[7];

          sett = autoSteerUdpData[8];  //setting1 - Danfoss valve etc

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
          steerConfigInit();
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

            sendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer));
          }
          /*  PANDA....
          if (useBNO08x) {
            sendUdp(helloFromIMU, sizeof(helloFromIMU));
          }*/
        }
      case 202:
        {
          //make really sure this is the reply pgn
          if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202) {
            ipDes = packet.remoteIP();
            //hello from AgIO
            uint8_t scanReply[] = { 128, 129, 126, 203, 7,
                                    myip[0], myip[1], myip[2], myip[3],
                                    myip[0], myip[1], myip[2], 23 };

            //checksum
            int16_t CK_A = 0;
            for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++) {
              CK_A = (CK_A + scanReply[i]);
            }
            scanReply[sizeof(scanReply) - 1] = CK_A;

            sendUdp(scanReply, sizeof(scanReply));
          }
        }
    }
  }
  else{
    Serial.println("Unknown packet!!!");
  }
}

void sendUdp(uint8_t* data, uint8_t datalen) {
  AsyncUDPMessage m = AsyncUDPMessage(datalen);
  m.write(data, datalen);
  udp.sendTo(m, ipDes, 9999);
}
