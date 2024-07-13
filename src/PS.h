/*
 * ======================================================================================================================
 * PS.h - Particle Support Funtions
 * ======================================================================================================================
 */

/*
 * ======================================================================================================================
 * Output_CellBatteryInfo() - On OLED display station information
 * ======================================================================================================================
 */
void Output_CellBatteryInfo() {
#if PLATFORM_ID == PLATFORM_ARGON
  WiFiSignal sig = WiFi.RSSI();
  float SignalStrength = sig.getStrength();

  sprintf (Buffer32Bytes, "CS:%d.%02d", 
    (int)SignalStrength, (int)(SignalStrength*100)%100);
  Output(Buffer32Bytes);
#else
  CellularSignal sig = Cellular.RSSI();
  float SignalStrength = sig.getStrength();
  
  int BatteryState = System.batteryState();
  float BatteryPoC = 0.0;                 // Battery Percent of Charge

  // Read battery charge information only if battery is connected. 
  if (BatteryState>0 && BatteryState<6) {
    BatteryPoC = System.batteryCharge();
  }
  
  sprintf (Buffer32Bytes, "CS:%d.%02d B:%d,%d.%02d", 
    (int)SignalStrength, (int)(SignalStrength*100)%100,
    BatteryState, (int)BatteryPoC, (int)(BatteryPoC*100)%100);
  Output(Buffer32Bytes);
#endif
}

/*
 * ======================================================================================================================
 * DeviceReset() - Kill power to ourselves and do a cold boot
 * ======================================================================================================================
 */
void DeviceReset() {
  digitalWrite(REBOOT_PIN, HIGH);
  delay(5000);
  // Should not get here if relay / watchdog is connected.
  digitalWrite(REBOOT_PIN, LOW);
  delay(2000); 

  // May never get here if relay board / watchdog not connected.

  // Resets the device, just like hitting the reset button or powering down and back up.
  System.reset();
}

/*
 * ======================================================================================================================
 * Function_DoAction() - Handler for Particle Function DoAction     
 * ======================================================================================================================
 */
int Function_DoAction(String s) {
  if (strcmp (s,"REBOOT") == 0) {  // Reboot
    Output("DoAction:REBOOT");
    delay(1000);

    DeviceReset();

    // Never gets here and thus can never send a ack to Particle Portal
    return(0);  
  }

  else if (strcmp (s,"10MDIST") == 0) { // Set 5M Distance Sensor State File
    Output("DoAction:10MDIST");
    od_adjustment = 2.5;
    if (SD_exists) {
      if (SD.exists(SD_5M_DIST_FILE)) {
        Output ("5MDIST, EXISTS REMOVING"); 
        if (SD.remove (SD_5M_DIST_FILE)) {
          Output ("DEL 5M:OK, 10M SET");
        }
        else {
          Output ("DEL 5M:ERR");
          return(-2);
        }    
      }
      else {
        Output ("10MDIST, ALREADY SET");
      }
    }
    else {
      Output("10MDIST, SD NF"); 
      return(-1);      
    }
    return(0);
  }

  else if (strcmp (s,"5MDIST") == 0) { // Set 5M Distance Sensor State File
    Output("DoAction:5MDIST");
    od_adjustment = 1.25;
    if (SD_exists) {
      if (SD.exists(SD_5M_DIST_FILE)) {
        Output ("5MDIST, ALREADY EXISTS");      
      }
      else {
        // Touch File
        File fp = SD.open(SD_5M_DIST_FILE, FILE_WRITE);
        if (fp) {
          fp.close();
          od_adjustment = 8;
          Output ("5MDIST SET");
        }
        else {
          Output ("5MDIST OPEN ERR");
          return(-3);
        }
      }
    }
    else {
      Output("5MDIST, SD NF"); 
      return(-1);      
    }
    return(0);
  }

  else {
    Output("DoAction:UKN"); 
    return(-1);
  }
}

#if PLATFORM_ID == PLATFORM_ARGON
/*
 * ======================================================================================================================
 * WiFiChangeCheck() - Check for WIFI.TXT file and set Wireless SSID, Password            
 * ======================================================================================================================
 */
void WiFiChangeCheck() {
  File fp;
  int i=0;
  char *p, *auth, *ssid, *pw;
  char ch, buf[128];

  if (SD_exists) {
    // Test for file WIFI.TXT
    if (SD.exists(SD_wifi_file)) {
      fp = SD.open(SD_wifi_file, FILE_READ); // Open the file for reading, starting at the beginning of the file.

      if (fp) {
        // Deal with too small or too big of file
        if (fp.size()<=7 || fp.size()>127) {
          fp.close();
          Output ("WIFI:Invalid SZ");
        }
        else {
          Output ("WIFI:Open");
          // Read one line from file
          while (fp.available() && (i < 128 )) {
            ch = fp.read();

            // sprintf (msgbuf, "%02X : %c", ch, ch);
            // Output (msgbuf);

            if ((ch == 0x0A) || (ch == 0x0D) ) {  // newline or linefeed
              break;
            }
            else {
              buf[i++] = ch;
            }
          }
          fp.close();
          Output ("WIFI:Close");

          // At this point we have encountered EOF, CR, or LF
          // Now we need to terminate array with a null to make it a string
          buf[i] = (char) NULL;

          // Parse string for the following
          //   WIFI ssid password
          p = &buf[0];
          auth = strtok_r(p, ",", &p);

          if (auth == NULL) {
            Output("WIFI:ID=Null Err");
          }
          else if ( (strcmp (auth, "WEP") != 0)  &&
                    (strcmp (auth, "WPA") != 0)  &&
                    (strcmp (auth, "WPA2") != 0) ){
            sprintf (msgbuf, "WIFI:ATYPE[%s] Err", auth);          
            Output(msgbuf);
          }
          else {
            ssid = strtok_r(p, ",", &p);
            pw  = strtok_r(p, ",", &p);

            if (ssid == NULL) {
              Output("WIFI:SSID=Null Err");
            }
            else if (pw == NULL) {
              Output("WIFI:PW=Null Err");
            }
            else {
              sprintf (msgbuf, "WIFI:ATYPE[%s]", auth);          
              Output(msgbuf);
              sprintf (msgbuf, "WIFI:SSID[%s]", ssid);
              Output(msgbuf);
              sprintf (msgbuf, "WIFI:PW[%s]", pw);
              Output(msgbuf);

              // Connects to a network secured with WPA2 credentials.
              // https://docs.particle.io/reference/device-os/api/wifi/securitytype-enum/
              if (strcmp (auth, "UNSEC") == 0) {
                Output("WIFI:Credentials Cleared");
                WiFi.clearCredentials();
                Output("WIFI:Credentials Set UNSEC");
                WiFi.setCredentials(ssid, pw, UNSEC);
              }
              else if (strcmp (auth, "WEP") == 0) {
                Output("WIFI:Credentials Cleared");
                WiFi.clearCredentials();
                Output("WIFI:Credentials Set WEP");
                WiFi.setCredentials(ssid, pw, WEP);
              }
              else if (strcmp (auth, "WPA") == 0) {
                Output("WIFI:Credentials Cleared");
                WiFi.clearCredentials();
                Output("WIFI:Credentials Set WPA");
                WiFi.setCredentials(ssid, pw, WPA);
              }
              else if (strcmp (auth, "WPA2") == 0) {
                Output("WIFI:Credentials Cleared");
                WiFi.clearCredentials();
                Output("WIFI:Credentials Set WPA2");
                WiFi.setCredentials(ssid, pw, WPA2);
              }
              else if (strcmp (auth, "WPA_ENTERPRISE") == 0) {
                Output("WIFI:Credentials Cleared");
                WiFi.clearCredentials();
                Output("WIFI:Credentials Set WPAE");
                WiFi.setCredentials(ssid, pw, WPA_ENTERPRISE);
              }
              else if (strcmp (auth, "WPA2_ENTERPRISE") == 0) {
                Output("WIFI:Credentials Cleared");
                WiFi.clearCredentials();
                Output("WIFI:Credentials Set WPAE2");
                WiFi.setCredentials(ssid, pw, WPA2_ENTERPRISE);
              }
              else { 
                Output("WIFI:Credentials NOT Set");
                Output("WIFI:USING NVAUTH");
              }
            }
          }
        }
      }
      else {
        sprintf (msgbuf, "WIFI:Open[%s] Err", SD_wifi_file);          
        Output(msgbuf);
        Output ("WIFI:USING NVAUTH");
      }
    } 
    else {
      Output ("WIFI:NOFILE USING NVAUTH");
    }
  } // SD enabled
  else {
    Output ("WIFI:NOSD USING NVAUTH");
  }
}

/*
 * ======================================================================================================================
 * WiFiPrintCredentials() - Read NVRAM and print WiFi Creditials     
 * 
 * Particle's Device OS does not provide a direct method to determine which of the stored Wi-Fi credentials is currently 
 * being used for the connection. However, you can infer this indirectly by attempting to connect to each stored Wi-Fi 
 * network and checking if the connection is successful.        
 * ======================================================================================================================
 */
void WiFiPrintCredentials() {
  byte mac[6];

  WiFi.macAddress(mac);


  sprintf (msgbuf, "WIFI MAC[%02x:%02x:%02x:%02x:%02x:%02x]", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Output(msgbuf);

  WiFiAccessPoint ap[5];
  WiFi.getCredentials(ap, 5);
  int found = 0;

  for (int i = 0; i < 5; i++) {
    if (strlen(ap[i].ssid) != 0) {
      found++;
      if (found == 1) {
        Output("WIFI Creds:");
      }
      sprintf (msgbuf, "IDX[%d]", i);
      Output(msgbuf);
      sprintf (msgbuf, " SSID[%s]", ap[i].ssid);
      Output(msgbuf);
      switch (ap[i].security) {
        case WLAN_SEC_UNSEC:
          Output(" Type:Unsecured");
          break;
        case WLAN_SEC_WEP:
          Output(" Type:WEP");
          break;
        case WLAN_SEC_WPA:
          Output(" Type:WPA");
          break;
        case WLAN_SEC_WPA2:
          Output(" Type:WPA2");
          break;
        case WLAN_SEC_WPA_ENTERPRISE:
          Output(" Type:WPA Ent");
          break;
        case WLAN_SEC_WPA2_ENTERPRISE:
          Output(" Type:WPA2 Ent");
          break;
        default:
          Output(" Type:Unknown");
          break;
      }
      sprintf (msgbuf, " SigStr[%d]", ap[i].rssi);
      Output(msgbuf);
      sprintf (msgbuf, " Channel[%d]", ap[i].channel);
      Output(msgbuf);
      sprintf (msgbuf, " BMAC[%02x:%02x:%02x:%02x:%02x:%02x]", 
        ap[i].bssid[0], ap[i].bssid[1], ap[i].bssid[2], ap[i].bssid[3], ap[i].bssid[4], ap[i].bssid[5]);
      Output(msgbuf);
    }
  }
  if (!found) {
    Output("WIFI Creds:NF");
  }
}
#else
/*
 * ======================================================================================================================
 * SimChangeCheck() - Check for SIM.TXT file and set sim based on contents, after rename file to SIMOLD.TXT            
 * ======================================================================================================================
 */
void SimChangeCheck() {
  File fp;
  int i=0;
  char *p, *id, *apn, *un, *pw;
  char ch, buf[128];
  bool changed = false;

  SimType simType = Cellular.getActiveSim();

  if (simType == 1) {
    Output ("SIM:Internal");
  } else if (simType == 2) {
    Output ("SIM:External");
  }
  else {
    sprintf (msgbuf, "SIM:Unknown[%d]", simType);
    Output (msgbuf);
  }

  if (SerialConsoleEnabled && SD_exists) {
    // Test for file SIM.TXT
    if (SD.exists(SD_sim_file)) {
      fp = SD.open(SD_sim_file, FILE_READ); // Open the file for reading, starting at the beginning of the file.

      if (fp) {
        // Deal with too small or too big of file
        if (fp.size()<=7 || fp.size()>127) {
          fp.close();
          Output ("SIMF:Invalid SZ");
          if (SD.remove (SD_sim_file)) {
            Output ("SIMF->Del:OK");
          }
          else {
            Output ("SIMF->Del:Err");
          }
        }
        else {
          Output ("SIMF:Open");
          while (fp.available() && (i < 128 )) {
            ch = fp.read();

            if ((ch == 0x0A) || (ch == 0x0D) ) {  // newline or linefeed
              break;
            }
            else {
              buf[i++] = ch;
            }
          }

          // At this point we have encountered EOF, CR, or LF
          // Now we need to terminate array with a null to make it a string
          buf[i] = NULL;

          // Parse string for the following
          //   INTERNAL
          //   AUP epc.tmobile.com username passwd
          //   UP username password
          //   APN epc.tmobile.com
          p = &buf[0];
          id = strtok_r(p, " ", &p);

          if (id != NULL) {
            sprintf (msgbuf, "SIMF:ID[%s]", id);
            Output(msgbuf);
          }

          if (strcmp (id,"INTERNAL") == 0) {
            Cellular.setActiveSim(INTERNAL_SIM);
            Cellular.clearCredentials();
          }
          else if (strcmp (id,"APN") == 0) {
            apn = strtok_r(p, " ", &p);

            if (apn == NULL) {
              Output("SIMF:APN=Null Err");
            }
            else {
              Cellular.setActiveSim(EXTERNAL_SIM);
              Output("SIM:Set External-APN");

              // Connects to a cellular network by APN only
              Cellular.setCredentials(apn);
              Output("SIM:Set Credentials");
              sprintf (msgbuf, " APN[%s]", apn);
              Output(msgbuf);
              changed = true;
            }
          }
          else if (strcmp (id," UP") == 0) {
            un  = strtok_r(p, " ", &p);
            pw  = strtok_r(p, " ", &p);

            if (un == NULL) {
              Output("SIMF:Username=Null Err");
            }
            else if (pw == NULL) {
              Output("SIMF:Passwd=Null Err");
            }
            else {
              Cellular.setActiveSim(EXTERNAL_SIM);
              Output("SIM:Set External-UP");

              // Connects to a cellular network with USERNAME and PASSWORD only
              Cellular.setCredentials(un,pw);
              Output("SIM:Set Credentials");
              sprintf (msgbuf, " UN[%s]", un);
              Output(msgbuf);
              sprintf (msgbuf, " PW[%s]", pw);
              Output(msgbuf);
              changed = true;
            }
          }
          else if (strcmp (id,"AUP") == 0) {
            apn = strtok_r(p, " ", &p);
            un  = strtok_r(p, " ", &p);
            pw  = strtok_r(p, " ", &p);

            if (apn == NULL) {
              Output("SIMF:APN=Null Err");
            }
            else if (un == NULL) {
              Output("SIMF:Username=Null Err");
            }
            else if (pw == NULL) {
              Output("SIMF:Passwd=Null Err");
            }
            else {
              Cellular.setActiveSim(EXTERNAL_SIM);
              Output("SIM:Set External-AUP");

              // Connects to a cellular network with a specified APN, USERNAME and PASSWORD
              Cellular.setCredentials(apn,un,pw);
              Output("SIM:Set Credentials");
              sprintf (msgbuf, " APN[%s]", apn);
              Output(msgbuf);
              sprintf (msgbuf, "  UN[%s]", un);
              Output(msgbuf);
              sprintf (msgbuf, "  PW[%s]", pw);
              Output(msgbuf);
              changed = true;
            }
          }
          else {  // Pasrse Error
            sprintf (msgbuf, "SIMF:ID[%s] Err", id);
            Output(msgbuf);
          }
        }

        // No matter what happened with the above, rename file so we don't process again at boot
        // if SIMOLD.TXT exists then remove it before we rename SIM.TXT
        if (SD.exists(SD_simold_file)) {
          if (SD.remove (SD_simold_file)) {
            Output ("SIMF:DEL SIMOLD");
          }
        }

        if (!fp.rename(SD_simold_file)) {
          Output ("SIMF:RENAME ERROR");
        }
        else {
          Output ("SIMF:RENAME OK");
        }
        fp.close();

        // Notify user to reboot blink led forever
        if (changed) {
          Output ("==============");
          Output ("!!! REBOOT !!!");
          Output ("==============");

        }
        else {
          Output ("=====================");
          Output ("!!! SET SIM ERROR !!!");
          Output ("=====================");
        }
        
        while(true) { // wait for Host to open serial port
          Blink(1, 750);
        }
      }
      else {
        Output ("SIMF:OPEN ERROR");
      }
    } 
    else {
      Output ("SIM:NO UPDATE FILE");
    }
  } // Console and SD enabled
}
#endif

/*
 * ======================================================================================================================
 * firmwareUpdateHandler() - The firmware update handler sets a global variable when the firmware update starts, 
 *                           so we can defer sleep until it completes (or times out)
 * 
 * SEE https://docs.particle.io/reference/device-os/api/system-events/system-events-reference/#cloud_status-64-parameter-values
 * ======================================================================================================================
 */
void firmwareUpdateHandler(system_event_t event, int param) {
    switch(param) {
        case 0:  // firmware_update_begin:
            firmwareUpdateInProgress = true;
            break;
        case 1:  // firmware_update_complete:
        case -1: // firmware_update_failed:
            firmwareUpdateInProgress = false;
            break;
    }
}