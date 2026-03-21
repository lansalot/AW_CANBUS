#ifndef UDPRECEIVE_H
#define UDPRECEIVE_H
#include <Arduino.h>
void udpSteerRecv(int sizeToRead)
{
  if (sizeToRead > 128)
    sizeToRead = 128;
  IPAddress src_ip = Udp.remoteIP();
  Udp.read(udpData, sizeToRead);

  if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) // Data
  {
    if (udpData[3] == 0xFE) // 254
    {
      gpsSpeed = ((float)(udpData[5] | udpData[6] << 8)) * 0.1;

      guidanceStatus = udpData[7];

      // Bit 8,9    set point steer angle * 100 is sent
      steerAngleSetPoint = ((float)(udpData[8] | ((int8_t)udpData[9]) << 8)) * 0.01; // high low bytes

      if (Brand == 8 || Brand == 9)
      {
        if (reverse_MT)
          steerAngleSetPoint *= -1.00;
      }

      // Serial.println(gpsSpeed);

      if ((bitRead(guidanceStatus, 0) == 0) || (steerSwitch == 1)) // AgOpen or Steer switch off
      {
        watchdogTimer = WATCHDOG_FORCE_VALUE; // turn off steering motor
      }
      else if (Brand != 3 && gpsSpeed < 0.1 && Brand != 5) // Speed < 0.1 and not Fendt
      {
        watchdogTimer = WATCHDOG_FORCE_VALUE; // turn off steering motor
      }
      else // valid conditions to turn on autosteer
      {
        watchdogTimer = 0; // reset watchdog
      }

      // Bit 10 Tram
      tram = udpData[10];

      // Bit 11
      relay = udpData[11];

      // Bit 12
      relayHi = udpData[12];

      //----------------------------------------------------------------------------
      // Serial Send to agopenGPS

      int16_t sa = (int16_t)(steerAngleActual * 100);

      AOG[5] = (uint8_t)sa;
      AOG[6] = sa >> 8;

      // heading
      AOG[7] = (uint8_t)9999;
      AOG[8] = 9999 >> 8;

      // roll
      AOG[9] = (uint8_t)8888;
      AOG[10] = 8888 >> 8;

      AOG[11] = switchByte;
      AOG[12] = (uint8_t)pwmDisplay;

      // checksum
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < AOGSize - 1; i++)
        CK_A = (CK_A + AOG[i]);

      AOG[AOGSize - 1] = CK_A;

      // off to AOG
      Udp.beginPacket(ipDestination, 9999);
      Udp.write(AOG, AOGSize);
      Udp.endPacket();

      // Steer Data 2 -------------------------------------------------
      if (aog2Count++ > 2)
      {
        // Send fromAutosteer2
        if (steerConfig.CurrentSensor)
          PGN_250[5] = (byte)currentReading;
        else if (steerConfig.PressureSensor)
          PGN_250[5] = (byte)pressureReading;
        else
          PGN_250[5] = 0;

        // add the checksum for AOG2
        CK_A = 0;
        for (uint8_t i = 2; i < PGN_250_Size; i++)
        {
          CK_A = (CK_A + PGN_250[i]);
        }
        PGN_250[PGN_250_Size] = CK_A;

        Udp.beginPacket(ipDestination, 9999);
        Udp.write(PGN_250, sizeof(PGN_250));
        Udp.endPacket();
        aog2Count = 0;
      }

      // Stop sending the helloAgIO message
      helloCounter = 0;
#ifdef useLED
      if (blink)
        digitalWrite(LED_TEENSY, HIGH);
      else
        digitalWrite(LED_TEENSY, LOW);
      blink = !blink;
#endif
      // Serial.println(steerAngleActual);
      //--------------------------------------------------------------------------
    }

    else if (udpData[3] == 200) // Hello from AgIO
    {
      int16_t sa = (int16_t)(steerAngleActual * 100);

      helloFromAutoSteer[5] = (uint8_t)sa;
      helloFromAutoSteer[6] = sa >> 8;

      helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
      helloFromAutoSteer[8] = helloSteerPosition >> 8;
      helloFromAutoSteer[9] = switchByte;

      Udp.beginPacket(ipDestination, 9999);
      Udp.write(helloFromAutoSteer, sizeof(helloFromAutoSteer));
      Udp.endPacket();

      if (useBNO08x || useTM171)
      {
        Udp.beginPacket(ipDestination, 9999);
        Udp.write(helloFromIMU, sizeof(helloFromIMU));
        Udp.endPacket();
      }
    }

    // Machine Data
    else if (udpData[3] == 0xEF) // 239 Machine Data
    {
      hydLift = udpData[7];

      // reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }

    // Machine Settings
    else if (udpData[3] == 0xEE) // 238 Machine Settings
    {
      aogConfig.raiseTime = udpData[5];
      aogConfig.lowerTime = udpData[6];
      // aogConfig.enableToolLift = udpData[7]; //This is wrong AgOpen is putting enable in sett,1

      // set1
      uint8_t sett = udpData[8]; // setting0
      if (bitRead(sett, 0))
        aogConfig.isRelayActiveHigh = 1;
      else
        aogConfig.isRelayActiveHigh = 0;
      if (bitRead(sett, 1))
        aogConfig.enableToolLift = 1;
      else
        aogConfig.enableToolLift = 0;

      // crc
      // udpData[13];        //crc

      // save in EEPROM and restart
      EEPROM.put(6, aogConfig);

      // reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }

    // steer settings
    else if (udpData[3] == 0xFC) // 252
    {
      // PID values
      steerSettings.Kp = udpData[5]; // read Kp from AgOpenGPS

      steerSettings.highPWM = udpData[6]; // read high pwm

      steerSettings.lowPWM = udpData[8]; // udpData[7];   // read lowPWM from AgOpenGPS

      steerSettings.minPWM = 1; // udpData[8]; //read the minimum amount of PWM for instant on

      steerSettings.steerSensorCounts = udpData[9]; // sent as setting displayed in AOG

      steerSettings.wasOffset = (udpData[10]); // read was zero offset Lo

      steerSettings.wasOffset |= (((int8_t)udpData[11]) << 8); // read was zero offset Hi

      steerSettings.AckermanFix = (float)udpData[12] * 0.01;

      // crc
      // udpData[13];

      // store in EEPROM
      EEPROM.put(10, steerSettings);

      // Send Config Via CAN
      if (Brand == 7)
        canConfig();

      // for PWM High to Low interpolator
      highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
    }

    else if (udpData[3] == 0xFB) // 251 FB - SteerConfig
    {
      uint8_t sett = udpData[5]; // setting0

      if (bitRead(sett, 0))
        steerConfig.InvertWAS = 1;
      else
        steerConfig.InvertWAS = 0;
      if (bitRead(sett, 1))
        steerConfig.IsRelayActiveHigh = 1;
      else
        steerConfig.IsRelayActiveHigh = 0;
      if (bitRead(sett, 2))
        steerConfig.MotorDriveDirection = 1;
      else
        steerConfig.MotorDriveDirection = 0;
      if (bitRead(sett, 3))
        steerConfig.SingleInputWAS = 1;
      else
        steerConfig.SingleInputWAS = 0;
      if (bitRead(sett, 4))
        steerConfig.CytronDriver = 1;
      else
        steerConfig.CytronDriver = 0;
      if (bitRead(sett, 5))
        steerConfig.SteerSwitch = 1;
      else
        steerConfig.SteerSwitch = 0;
      if (bitRead(sett, 6))
        steerConfig.SteerButton = 1;
      else
        steerConfig.SteerButton = 0;
      if (bitRead(sett, 7))
        steerConfig.ShaftEncoder = 1;
      else
        steerConfig.ShaftEncoder = 0;

      steerConfig.PulseCountMax = udpData[6];

      // was speed
      // udpData[7];

      sett = udpData[8]; // setting1 - Danfoss valve etc

      if (bitRead(sett, 0))
        steerConfig.IsDanfoss = 1;
      else
        steerConfig.IsDanfoss = 0;
      if (bitRead(sett, 1))
        steerConfig.PressureSensor = 1;
      else
        steerConfig.PressureSensor = 0;
      if (bitRead(sett, 2))
        steerConfig.CurrentSensor = 1;
      else
        steerConfig.CurrentSensor = 0;
      if (bitRead(sett, 3))
        steerConfig.IsUseY_Axis = 1;
      else
        steerConfig.IsUseY_Axis = 0;

      // crc
      // udpData[13];

      EEPROM.put(40, steerConfig);

      // Send Config Via CAN
      if (Brand == 7)
        canConfig();

      // reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;

    } // end FB

    else if (udpData[3] == 0xD0) // Corrected GPS Data
    {

      uint32_t encodedAngle;
      uint16_t encodedInt16;

      encodedAngle = ((uint32_t)(udpData[5] | udpData[6] << 8 | udpData[7] << 16 | udpData[8] << 24));
      pivotLat = (((double)encodedAngle * 0.0000001) - 210);

      encodedAngle = ((uint32_t)(udpData[9] | udpData[10] << 8 | udpData[11] << 16 | udpData[12] << 24));
      pivotLon = (((double)encodedAngle * 0.0000001) - 210);

      encodedInt16 = ((uint16_t)(udpData[13] | udpData[14] << 8));
      fixHeading = ((double)encodedInt16 / 128);

      encodedInt16 = ((uint16_t)(udpData[15] | udpData[16] << 8));
      pivotAltitude = ((double)encodedInt16 * 0.01);

      static int GPS_5hz = 0;

      if (sendGPStoISOBUS)
      {
        if (GPS_5hz > 4)
        {
          GPS_5hz = 0;
          sendISOBUS_65267_65256();
        }

        GPS_5hz++;
      }

    } // end D0

    else if (udpData[3] == 201)
    {
      // make really sure this is the subnet pgn
      if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)
      {
        networkAddress.ipOne = udpData[7];
        networkAddress.ipTwo = udpData[8];
        networkAddress.ipThree = udpData[9];

        // save in EEPROM and restart
        EEPROM.put(60, networkAddress);
        SCB_AIRCR = 0x05FA0004; // Teensy Reset
      }
    } // end 201

    // Who Am I ?
    else if (udpData[3] == 202)
    {
      // make really sure this is the reply pgn
      if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202)
      {
        // hello from AgIO
        uint8_t scanReply[] = {128, 129, 126, 203, 7,
                               ip[0], ip[1], ip[2], 126, src_ip[0], src_ip[1], src_ip[2], 23};

        // checksum
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
        {
          CK_A = (CK_A + scanReply[i]);
        }
        scanReply[sizeof(scanReply) - 1] = CK_A;

        static uint8_t ipDest[] = {255, 255, 255, 255};
        uint16_t portDest = 9999; // AOG port that listens

        // off to AOG
        Udp.beginPacket(ipDest, portDest);
        Udp.write(scanReply, sizeof(scanReply));
        Udp.endPacket();

        Serial.print("\r\nAdapter IP: ");
        Serial.print(src_ip[0]);
        Serial.print(" . ");
        Serial.print(src_ip[1]);
        Serial.print(" . ");
        Serial.print(src_ip[2]);
        Serial.print(" . ");
        Serial.print(src_ip[3]);

        Serial.print("\r\nModule  IP: ");
        Serial.print(ip[0]);
        Serial.print(" . ");
        Serial.print(ip[1]);
        Serial.print(" . ");
        Serial.print(ip[2]);
        Serial.print(" . ");
        Serial.print(ip[3]);
        Serial.println();

        Serial.println(inoVersion);
        Serial.println();

        if (Brand == 0)
          Serial.println("Brand = Claas (Set Via Service Tool)");
        else if (Brand == 1)
          Serial.println("Brand = Valtra / Massey (Set Via Service Tool)");
        else if (Brand == 2)
          Serial.println("Brand = CaseIH / New Holland (Set Via Service Tool)");
        else if (Brand == 3)
          Serial.println("Brand = Fendt SCR,S4,Gen6 (Set Via Service Tool)");
        else if (Brand == 4)
          Serial.println("Brand = JCB (Set Via Service Tool)");
        else if (Brand == 5)
          Serial.println("Brand = FendtOne (Set Via Service Tool)");
        else if (Brand == 6)
          Serial.println("Brand = Lindner (Set Via Service Tool)");
        else if (Brand == 7)
          Serial.println("Brand = AgOpenGPS (Set Via Service Tool)");
        else if (Brand == 8)
          Serial.println("Brand = Cat MT (Set Via Service Tool)");
        else
          Serial.println("No Tractor Brand Set, Set Via Service Tool");

        Serial.println("\r\nGPS Mode:");
        if (gpsMode == 1)
          Serial.println("GPS Forwarding @ 115200 (Set Via Service Tool)");
        else if (gpsMode == 2)
          Serial.println("GPS Forwarding @ 460800 (Set Via Service Tool)");
        else if (gpsMode == 3)
          Serial.println("Panda Mode @ 115200 (Set Via Service Tool)");
        else if (gpsMode == 4)
          Serial.println("Panda Mode @ 460800 (Set Via Service Tool)");
        else
          Serial.println("No GPS mode selected - Set Via Service Tool");
        Serial.println(" --------- ");
      }
    } // end 202

  } // end if 80 81 7F

} // end udp callback
#endif