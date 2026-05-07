/*
 * ======================================================================================================================
 *  obs.cpp - Observation Functions
 * ======================================================================================================================
 */
#include <Particle.h>
#include <SdFat.h>

#include "include/qc.h"
#include "include/ssbits.h"
#include "include/sensors_i2c_44_47.h"
#include "include/sensors.h"
#include "include/eeprom.h"
#include "include/wrda.h"
#include "include/cf.h"
#include "include/sdcard.h"
#include "include/output.h"
#include "include/support.h"
#include "include/time.h"
#include "include/main.h"
#include "include/obs.h"

/*
 * ======================================================================================================================
 * Variables and Data Structures
 * =======================================================================================================================
 */
float bmx_1_pressure = 0.0;

/*
 * ======================================================================================================================
 * Fuction Definations
 * =======================================================================================================================
 */

/*
 * ======================================================================================================================
 * Particle_Publish() - Publish to Particle what is in msgbuf
 * ======================================================================================================================
 */
bool Particle_Publish(char *EventName) {
  // Calling Particle.publish() when the cloud connection has been turned off will not publish an event. 
  // This is indicated by the return success code of false. If the cloud connection is turned on and 
  // trying to connect to the cloud unsuccessfully, Particle.publish() may block for up to 20 seconds 
  // (normal conditions) to 10 minutes (unusual conditions). Checking Particle.connected() 
  // before calling Particle.publish() can help prevent this.
  //   if (Cellular.ready() && Particle.connected()) {
  if (Particle.connected()) {
    if (Particle.publish(EventName, msgbuf, WITH_ACK)) {  // PRIVATE flag is always used even when not specified
      // Currently, a device can publish at rate of about 1 event/sec, with bursts of up to 4 allowed in 1 second. 
      delay (1000);
      return(true);
    }
  }
  else {
    Output ("Particle:NotReady");
  }
  return(false);
}

/*
 * ======================================================================================================================
 * OBS_Do() - Collect Observations, Build message, Send to logging site
 * ======================================================================================================================
 */
void OBS_Do() {
  float bmx1_pressure = 0.0;
  float bmx1_temp = 0.0;
  float bmx1_humid = 0.0;
  float bmx2_pressure = 0.0;
  float bmx2_temp = 0.0;
  float bmx2_humid = 0.0;
  float htu1_temp = 0.0;
  float htu1_humid = 0.0;
  float mcp1_temp = 0.0;
  float mcp2_temp = 0.0;
  float lux = 0.0;
  float si_vis = 0.0;
  float si_ir = 0.0;
  float si_uv = 0.0;
  float heat_index = 0.0;
 
  float distance;
  float distance_stdev;
  int   distance_outliers;

  float ws=0.0;
  int   wd=0;
  float wg=0.0;
  int   wgd=0;

  // Safty Check for Vaild Time
  if (!Time.isValid()) {
    Output ("OBS_Do: Time NV");
    return;
  }
  else {
    stc_timestamp();
    sprintf (msgbuf, "OBS_Do(%s)", timestamp);
    Output(msgbuf);
  }

  float wlm = DistanceGauge_Median();
  int   wlr = analogRead(DISTANCEGAUGE) * (float) dg_adjustment; // Water Level Raw Reading

  DistanceGauge_Calc(&distance, &distance_stdev, &distance_outliers);

  if (DoWind) {
    Wind_GustUpdate(); // Update Gust and Gust Direction readings
    ws = Wind_SpeedAverage();
    ws = (isnan(ws) || (ws < QC_MIN_WS) || (ws > QC_MAX_WS)) ? QC_ERR_WS : ws;

    wd = Wind_DirectionVector();
    wd = (isnan(wd) || (wd < QC_MIN_WD) || (wd > QC_MAX_WD)) ? QC_ERR_WD : wd;

    wg = Wind_Gust();
    wg = (isnan(wg) || (wg < QC_MIN_WS) || (wg > QC_MAX_WS)) ? QC_ERR_WS : wg;

    wgd = Wind_GustDirection();
    wgd = (isnan(wgd) || (wgd < QC_MIN_WD) || (wgd > QC_MAX_WD)) ? QC_ERR_WD : wgd;
  }

  // Adafruit I2C Sensors
  if (BMX_1_exists) {
    bmx1_read(bmx1_pressure, bmx1_temp, bmx1_humid);
  }

  if (BMX_2_exists) {
    bmx2_read(bmx2_pressure, bmx2_temp, bmx2_humid);
  }

  if (HTU21DF_exists) {
    htu1_humid = htu.readHumidity();
    htu1_humid = (isnan(htu1_humid) || (htu1_humid < QC_MIN_RH) || (htu1_humid > QC_MAX_RH)) ? QC_ERR_RH : htu1_humid;

    htu1_temp = htu.readTemperature();
    htu1_temp = (isnan(htu1_temp) || (htu1_temp < QC_MIN_T)  || (htu1_temp > QC_MAX_T))  ? QC_ERR_T  : htu1_temp;
  }

  if (SI1145_exists) {
    si_vis = uv.readVisible();
    si_ir = uv.readIR();
    si_uv = uv.readUV()/100.0;

    // Additional code to force sensor online if we are getting 0.0s back.
    if ( ((si_vis+si_ir+si_uv) == 0.0) && ((si_last_vis+si_last_ir+si_last_uv) != 0.0) ) {
      // Let Reset The SI1145 and try again
      Output ("SI RESET");
      if (uv.begin()) {
        SI1145_exists = true;
        Output ("SI ONLINE");

        si_vis = uv.readVisible();
        si_ir = uv.readIR();
        si_uv = uv.readUV()/100.0;
      }
      else {
        SI1145_exists = false;
        Output ("SI OFFLINE");
      }
    }

    // Save current readings for next loop around compare
    si_last_vis = si_vis;
    si_last_ir = si_ir;
    si_last_uv = si_uv;

    // QC Checks
    si_vis = (isnan(si_vis) || (si_vis < QC_MIN_VI)  || (si_vis > QC_MAX_VI)) ? QC_ERR_VI  : si_vis;
    si_ir  = (isnan(si_ir)  || (si_ir  < QC_MIN_IR)  || (si_ir  > QC_MAX_IR)) ? QC_ERR_IR  : si_ir;
    si_uv  = (isnan(si_uv)  || (si_uv  < QC_MIN_UV)  || (si_uv  > QC_MAX_UV)) ? QC_ERR_UV  : si_uv;
  }

  if (MCP_1_exists) {
    mcp1_temp = mcp1.readTempC();
    mcp1_temp = (isnan(mcp1_temp) || (mcp1_temp < QC_MIN_T)  || (mcp1_temp > QC_MAX_T))  ? QC_ERR_T  : mcp1_temp;
  }

  if (MCP_2_exists) {
    mcp2_temp = mcp2.readTempC();
    mcp2_temp = (isnan(mcp2_temp) || (mcp2_temp < QC_MIN_T)  || (mcp2_temp > QC_MAX_T))  ? QC_ERR_T  : mcp2_temp;
  }

  if (VEML7700_exists) {
    float lux = veml.readLux(VEML_LUX_AUTO);
    lux = (isnan(lux) || (lux < QC_MIN_VLX)  || (lux > QC_MAX_VLX))  ? QC_ERR_VLX  : lux;
  }



  float SignalStrength = 0;
  int BatteryState = 0;
  float BatteryPoC = 0.0;  // Battery Percent of Charge
  byte cfr = 0;

#if PLATFORM_ID == PLATFORM_ARGON
  WiFiSignal sig = WiFi.RSSI();
  SignalStrength = sig.getStrength();
#else
  if (Cellular.ready()) {
    CellularSignal sig = Cellular.RSSI();
    SignalStrength = sig.getStrength();
  }
  // Get Battery Charger Failt Register
  cfr = pmic.getFault();

  BatteryState = System.batteryState();
  // Read battery charge information only if battery is connected.
  if (BatteryState>0 && BatteryState<6) {
    BatteryPoC = System.batteryCharge();
  }
#endif

  stc_timestamp();
  Output(timestamp);

  // Report if we have Need to Send Observations
  if (SD_exists && SD.exists(SD_n2s_file)) {
    SystemStatusBits |= SSB_N2S; // Turn on Bit
  }
  else {
    SystemStatusBits &= ~SSB_N2S; // Turn Off Bit
  }

  memset(msgbuf, 0, sizeof(msgbuf));
  JSONBufferWriter writer(msgbuf, sizeof(msgbuf)-1);
  writer.beginObject();
    writer.name("at").value(timestamp);
    writer.name("wl").value(distance, 2);
    writer.name("wld").value(distance_stdev, 2);
    writer.name("wlo").value(distance_outliers);
    writer.name("wlm").value(wlm,2);
    writer.name("wlr").value(wlr);

    if (AS5600_exists) {
      writer.name("ws").value(ws,4);
      writer.name("wd").value(wd);
      writer.name("wg").value(wg,4);
      writer.name("wgd").value(wgd);
    }
    if (BMX_1_exists) {
      writer.name("bp1").value(bmx1_pressure, 4);
      writer.name("bt1").value(bmx1_temp, 2);
      if (BMX_1_type == BMX_TYPE_BME280) {
        writer.name("bh1").value(bmx1_humid, 2);
      }
    }
    if (BMX_2_exists) {
      writer.name("bp2").value(bmx2_pressure, 4);
      writer.name("bt2").value(bmx2_temp, 2);
      if (BMX_2_type == BMX_TYPE_BME280) {
        writer.name("bh2").value(bmx2_humid, 2);
      }
    }

    sensor_i2c_44_47_obs_do(writer);

    if (HTU21DF_exists) {
      writer.name("ht1").value(htu1_temp, 2);
      writer.name("hh1").value(htu1_humid, 2);
    }
    if (MCP_1_exists) {
      writer.name("mt1").value(mcp1_temp, 2);
    }
    if (MCP_2_exists) {
      writer.name("mt2").value(mcp2_temp, 2);
    }
    if (SI1145_exists) {
      writer.name("sv1").value(si_vis, 2);
      writer.name("si1").value(si_ir, 2);
      writer.name("su1").value(si_uv, 2);
    }
    if (VEML7700_exists) {
      writer.name("vlx").value(lux, 2);
    }
    if (HI_exists) { // Heat Index Temperature
      heat_index = hi_calculate(sht1_temp, sht1_humid);
      writer.name("hi").value(heat_index, 2);
    } 
    if (MSLP_exists) {
      float mslp = (float) mslp_calculate(sht1_temp, sht1_humid, bmx_1_pressure, cf_elevation);
      writer.name("mslp").value(mslp, 2);
    }

    if (OP2_State == OP2_STATE_RAW) {
      writer.name("op2r").value(Pin_ReadAvg(OP2_PIN), 2); // OP2 Raw
    }
    else if (OP2_State == OP2_STATE_VOLTAIC) {
      float vbv = VoltaicVoltage(OP2_PIN);
      writer.name("vbv").value(vbv, 2);                   // OP2 Voltaic Battery Voltage
      writer.name("vpc").value(VoltaicPercent(vbv), 2);   // OP2 Voltaic Battery Voltage
    }

    writer.name("bcs").value(BatteryState);
    writer.name("bpc").value(BatteryPoC, 4);
    writer.name("cfr").value(cfr);
    writer.name("css").value(SignalStrength, 4);
    writer.name("hth").value(SystemStatusBits);
  writer.endObject();

  // Log Observation to SD Card
  SD_LogObservation(msgbuf);
  Serial_write (msgbuf);

  lastOBS = System.millis();

  Output ("Publish(SS)");
  if (Particle_Publish((char *) "SS")) {

    if (SD_exists) {
      Output ("Publish(OK)");
    }
    else {
      Output ("Publish(OK)-NO SD!!!");
    }

    // If we Published, Lets try send N2S observations
    SD_N2S_Publish();
  }
  else {
    Output ("Publish(FAILED)");
    
    // Set the bit so when we finally transmit the observation,
    // we know it cam from the N2S file.
    SystemStatusBits |= SSB_FROM_N2S; // Turn On Bit

    memset(msgbuf, 0, sizeof(msgbuf));
    // SEE https://docs.particle.io/reference/device-os/firmware/argon/#jsonwriter
    JSONBufferWriter writer(msgbuf, sizeof(msgbuf)-1);
    writer.beginObject();
      writer.name("at").value(timestamp);
      writer.name("wl").value(distance, 2);
      writer.name("wld").value(distance_stdev, 2);
      writer.name("wlo").value(distance_outliers);
      writer.name("wlm").value(wlm,2);
      writer.name("wlr").value(wlr);

      if (AS5600_exists) {
        writer.name("ws").value(ws,4);
        writer.name("wd").value(wd);
        writer.name("wg").value(wg,4);
        writer.name("wgd").value(wgd);
      }

      if (BMX_1_exists) {
        // sprintf (Buffer32Bytes, "%d.%02d", (int)bmx1_pressure, (int)(bmx1_pressure*100)%100);
        // writer.name("bp1").value(Buffer32Bytes);
        writer.name("bp1").value(bmx1_pressure, 4);
        writer.name("bt1").value(bmx1_temp, 4);
        writer.name("bh1").value(bmx1_humid, 4);
      }
      if (BMX_2_exists) {
        // sprintf (Buffer32Bytes, "%d.%02d", (int)bmx2_pressure, (int)(bmx2_pressure*100)%100);
        // writer.name("bp2").value(Buffer32Bytes);
        writer.name("bp2").value(bmx2_pressure, 4);
        writer.name("bt2").value(bmx2_temp, 4);
        writer.name("bh2").value(bmx2_humid, 4);
      }

      sensor_i2c_44_47_obs_do(writer);

      if (HTU21DF_exists) {
        writer.name("ht1").value(htu1_temp, 2);
        writer.name("hh1").value(htu1_humid, 2);
      }
      if (MCP_1_exists) {
        writer.name("mt1").value(mcp1_temp, 2);
      }
      if (MCP_2_exists) {
        writer.name("mt2").value(mcp2_temp, 2);
      }
      if (SI1145_exists) {
        writer.name("sv1").value(si_vis, 2);
        writer.name("si1").value(si_ir, 2);
        writer.name("su1").value(si_uv, 2);
      }
      if (VEML7700_exists) {
        writer.name("lx").value(lux, 2);
      }
      if (HI_exists) {
        heat_index = hi_calculate(sht1_temp, sht1_humid);
        writer.name("hi").value(heat_index, 2);
      } 
      if (MSLP_exists) {
        float mslp = (float) mslp_calculate(sht1_temp, sht1_humid, bmx_1_pressure, cf_elevation);
        writer.name("mslp").value(mslp, 2);
      }        
      writer.name("bcs").value(BatteryState);
      writer.name("bpc").value(BatteryPoC, 4);
      writer.name("cfr").value(cfr);
      writer.name("css").value(SignalStrength, 4);
      writer.name("hth").value(SystemStatusBits);
    writer.endObject();
    SystemStatusBits &= ~SSB_FROM_N2S; // Turn Off Bit

    SD_NeedToSend_Add(msgbuf);
  }

  Output(timestamp);

  sprintf (msgbuf, "%d.%02d %d.%02d %d.%02d", 
    (int)distance, (int)(distance*100)%100,
    (int)bmx1_pressure, (int)(bmx1_pressure*100)%100,
    (int)bmx2_pressure, (int)(bmx2_pressure*100)%100);
  Output(msgbuf);

  sprintf (msgbuf, "C%d.%02d B%d:%d.%02d %X", 
    (int)SignalStrength, (int)(SignalStrength*100)%100,
    BatteryState, 
    (int)BatteryPoC, (int)(BatteryPoC*100)%100,
    (int)SystemStatusBits);
  Output(msgbuf);
}
