/*
 * ======================================================================================================================
 * SM.h - Station Monitor - When Jumper set Display StationMonitor()
 * ======================================================================================================================
 */

/*
 * ======================================================================================================================
 * StationMonitor() - On OLED display station information
 * ======================================================================================================================
 */
void StationMonitor() {
  static int cycle = 0;
  static int count = 0;
  int r, c, len;

  // Clear display with spaces
  for (r=0; r<4; r++) {
    for (c=0; c<22; c++) {
      oled_lines [r][c] = ' ';
    }
    oled_lines [r][c] = (char) NULL;
  }
  
  // =================================================================
  // Line 0 of OLED
  // =================================================================
  stc_timestamp();
  len = (strlen (timestamp) > 21) ? 21 : strlen (timestamp);
  for (c=0; c<=len; c++) oled_lines [0][c] = *(timestamp+c);
  Serial_writeln (timestamp);

  // =================================================================
  // Line 1 of OLED Distance Median & Distance Raw
  // =================================================================
  memset(msgbuf, 0, sizeof(msgbuf));

  float distance = OBS_Median_Distance() * od_adjustment; // Pins are 12bit resolution 

  sprintf (msgbuf+strlen(msgbuf), "DM:%d.%02d DR:%d %s", 
    (int)distance, (int)(distance*100)%100, (int) analogRead(DISTANCEGAUGE),
    (od_adjustment == 1.25) ? "5M" : "10M");

  len = (strlen (msgbuf) > 21) ? 21 : strlen (msgbuf);
  for (c=0; c<=len; c++) oled_lines [1][c] = *(msgbuf+c);
  Serial_writeln (msgbuf);

  // =================================================================
  // Line 2 of OLED Wind Direction and Wind Speed
  // =================================================================
  memset(msgbuf, 0, sizeof(msgbuf));

  if (AS5600_exists) {
    sprintf (msgbuf+strlen(msgbuf), "WD:%3d WS:%02d", 
      Wind_SampleDirection(), anemometer_interrupt_count);
  }
  else {
    sprintf (msgbuf+strlen(msgbuf), "WD:NF  WS:NF");
  }
  
  len = (strlen (msgbuf) > 21) ? 21 : strlen (msgbuf);
  for (c=0; c<=len; c++) oled_lines [2][c] = *(msgbuf+c);
  Serial_writeln (msgbuf);
  
  // =================================================================
  // Line 3 of OLED Cycle between multiple sensors
  // =================================================================
  if (cycle == 0) {
    if (BMX_1_exists) {
      float bmx_pressure = 0.0;
      float bmx_temp = 0.0;
      float bmx_humid;
      
      switch (BMX_1_chip_id) {
        case BMP280_CHIP_ID :
          bmx_pressure = bmp1.readPressure()/100.0F;           // bmxp1
          bmx_temp = bmp1.readTemperature();                   // bmxt1
          break;
        
        case BME280_BMP390_CHIP_ID :
          if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
            bmx_pressure = bme1.readPressure()/100.0F;           // bmxp1
            bmx_temp = bme1.readTemperature();                   // bmxt1
            bmx_humid = bme1.readHumidity();                     // bmxh1 
          }
          else { // BMP390
            bmx_pressure = bm31.readPressure()/100.0F;
            bmx_temp = bm31.readTemperature();
          }
          break;
          
        case BMP388_CHIP_ID :
          bmx_pressure = bm31.readPressure()/100.0F;
          bmx_temp = bm31.readTemperature();
          break;
        
        default: // WTF
          break;
      }
      sprintf (msgbuf, "B1 %d.%02d %d.%02d %d.%02d", 
        (int)bmx_pressure, (int)(bmx_pressure*100)%100,
        (int)bmx_temp, (int)(bmx_temp*100)%100,
        (int)bmx_humid, (int)(bmx_humid*100)%100);
    }
    else {
      sprintf (msgbuf, "B1 NF");
    }
  }
  
  if (cycle == 1) {
    if (BMX_2_exists) {
      float bmx_pressure = 0.0;
      float bmx_temp = 0.0;
      float bmx_humid;
      
      switch (BMX_2_chip_id) {
        case BMP280_CHIP_ID :
          bmx_pressure = bmp2.readPressure()/100.0F;           // bmxp1
          bmx_temp = bmp1.readTemperature();                   // bmxt1
          break;
        
        case BME280_BMP390_CHIP_ID :
          if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
            bmx_pressure = bme2.readPressure()/100.0F;           // bmxp1
            bmx_temp = bme1.readTemperature();                   // bmxt1
            bmx_humid = bme1.readHumidity();                     // bmxh1 
          }
          else { // BMP390
            bmx_pressure = bm32.readPressure()/100.0F;
            bmx_temp = bm31.readTemperature();
          }
          break;
          
        case BMP388_CHIP_ID :
          bmx_pressure = bm32.readPressure()/100.0F;
          bmx_temp = bm31.readTemperature();
          break;
        
        default: // WTF
          break;
      }
      sprintf (msgbuf, "B2 %d.%02d %d.%02d %d.%02d", 
        (int)bmx_pressure, (int)(bmx_pressure*100)%100,
        (int)bmx_temp, (int)(bmx_temp*100)%100,
        (int)bmx_humid, (int)(bmx_humid*100)%100);
    }
    else {
      sprintf (msgbuf, "B2 NF");
    }
  }

  if (cycle == 2) {
    memset(msgbuf, 0, sizeof(msgbuf));
      
    if (MCP_1_exists) {
      float mcp_temp = mcp1.readTempC();   
      sprintf (msgbuf, "MCP1 T%d.%02d", (int)mcp_temp, (int)(mcp_temp*100)%100);
    }
    else {
      sprintf (msgbuf, "MCP1 NF");
    }
  }

  if (cycle == 3) {
    if (MCP_2_exists) {
      float mcp_temp = mcp2.readTempC();   
      sprintf (msgbuf, "MCP2 T%d.%02d", (int)mcp_temp, (int)(mcp_temp*100)%100);
    }
    else {
      sprintf (msgbuf, "MCP2 NF");
    }
  }

  if (cycle == 4) {   
    if (HTU21DF_exists) {
      float htu_humid = htu.readHumidity();
      float htu_temp = htu.readTemperature();

      sprintf (msgbuf, "HTU H:%02d.%02d T:%02d.%02d", 
        (int)htu_humid, (int)(htu_humid*100)%100, 
        (int)htu_temp, (int)(htu_temp*100)%100);
    }
    else {
      sprintf (msgbuf, "HTU NF"); 
    } 
  }

  if (cycle == 5) {   
    if (VEML7700_exists) {
      float lux = veml.readLux(VEML_LUX_AUTO);
      lux = (isnan(lux)) ? 0.0 : lux;
        sprintf (msgbuf, "LX L%02d.%1d", (int)lux, (int)(lux*10)%10);
    }
    else {
      sprintf (msgbuf, "LX NF");
    }
  }

  if (cycle == 6) {
    if (SHT_1_exists) {
      float t = sht1.readTemperature();
      float h = sht1.readHumidity();
      sprintf (msgbuf, "SHT1 T:%d.%02d H:%d.%02d", 
         (int)t, (int)(t*100)%100,
         (int)h, (int)(h*100)%100);
    }
    else {
      sprintf (msgbuf, "SHT1 NF");
    }
  }

  if (cycle == 7) {
    if (SHT_2_exists) {
      float t = sht2.readTemperature();
      float h = sht2.readHumidity();
      sprintf (msgbuf, "SHT2 T:%d.%02d H:%d.%02d", 
         (int)t, (int)(t*100)%100,
         (int)h, (int)(h*100)%100);
    }
    else {
      sprintf (msgbuf, "SHT2 NF");
    }      
  }

  if (cycle == 8) {   
    if (HIH8_exists) {
      float t = 0.0;
      float h = 0.0;
      bool status = hih8_getTempHumid(&t, &h);
      if (!status) {
        t = -999.99;
        h = 0.0;
      }
      sprintf (msgbuf, "HIH8 T%d.%02d H%d.%02d", 
         (int)t, (int)(t*100)%100,
         (int)h, (int)(h*100)%100);
    }
    else {
      sprintf (msgbuf, "HIH8 NF");
    }
  }

  if (cycle == 9) {
    if (SI1145_exists) {
      float si_vis = uv.readVisible();
      float si_ir = uv.readIR();
      float si_uv = uv.readUV()/100.0;
      sprintf (msgbuf, "SI V%d.%02d I%d.%02d U%d.%02d", 
        (int)si_vis, (int)(si_vis*100)%100,
        (int)si_ir, (int)(si_ir*100)%100,
        (int)si_uv, (int)(si_uv*100)%100);
    }
    else {
      sprintf (msgbuf, "SI NF");
    }
  }

  if (cycle == 10) {
    memset(msgbuf, 0, sizeof(msgbuf));

#if PLATFORM_ID == PLATFORM_ARGON
    WiFiSignal sig = WiFi.RSSI();
    float SignalStrength = sig.getStrength();
    int BatteryState = 0;
#else
    CellularSignal sig = Cellular.RSSI();
    float SignalStrength = sig.getStrength();
#endif
    sprintf (msgbuf+strlen(msgbuf), "CSS:%d.%02d HTH:%X",
      (int)SignalStrength, (int)(SignalStrength*100)%100,
      (int)SystemStatusBits); 
  }

  len = (strlen (msgbuf) > 21) ? 21 : strlen (msgbuf);
  for (c=0; c<=len; c++) oled_lines [3][c] = *(msgbuf+c);
  Serial_writeln (msgbuf);

  // Give the use some time to read line 3 before changing
  if (count++ >= 5) {
    cycle = ++cycle % 11;
    count = 0;
  }
  
  OLED_update();
}
