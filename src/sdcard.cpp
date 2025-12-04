/*
 * ======================================================================================================================
 * sdcard.cpp - SD Card
 * ======================================================================================================================
 */
#include <Particle.h>
#include <SdFat.h>

#include "include/ssbits.h"
#include "include/main.h"
#include "include/cf.h"
#include "include/eeprom.h"
#include "include/output.h"
// #include "include/sensors.h"
#include "include/sdcard.h"

// Prototyping functions to aviod compile function unknown issue.
bool Particle_Publish(char *EventName); 
void OBS_Do();

/*
 * ======================================================================================================================
 * Variables and Data Structures
 * =======================================================================================================================
 */
SdFat SD;                               // File system object.
File SD_fp;
char SD_obsdir[] = "/OBS";              // Store our obs in this directory. At Power on, it is created if does not exist
bool SD_exists = false;                     // Set to true if SD card found at boot
char SD_n2s_file[] = "N2SOBS.TXT";          // Need To Send Observation file
uint32_t SD_n2s_max_filesz = 200 * 8 * 24;  // Keep a little over 2 days. When it fills, it is deleted and we start over.

char SD_sim_file[] = "SIM.TXT";         // File used to set Ineternal or External sim configuration
char SD_simold_file[] = "SIMOLD.TXT";   // SIM.TXT renamed to this after sim configuration set

char SD_wifi_file[] = "WIFI.TXT";       // File used to set WiFi configuration

char SD_INFO_FILE[] = "INFO.TXT";       // Store INFO information in this file. Every INFO call will overwrite content

char SD_5M_DIST_FILE[] = "5MDIST.TXT";  // If file exists use adjustment of 1.25. No file, then 10m Sensor is 2.5

char SD_ELEV_FILE[] = "ELEV.TXT";       // Set the station elevation for MSLP calculation            

/*
 * ======================================================================================================================
 * Fuction Definations
 * =======================================================================================================================
 */

/* 
 *=======================================================================================================================
 * SD_initialize()
 *=======================================================================================================================
 */
void SD_initialize() {

  if (!SD.begin(SD_ChipSelect)) {
    Output ("SD:NF");
    SystemStatusBits |= SSB_SD;
    delay (5000);
  }
  else {
    if (!SD.exists(SD_obsdir)) {
      if (SD.mkdir(SD_obsdir)) {
        Output ("SD:MKDIR OBS OK");
        Output ("SD:Online");
        SD_exists = true;
      }
      else {
        Output ("SD:MKDIR OBS ERR");
        Output ("SD:Offline");
        SystemStatusBits |= SSB_SD;  // Turn On Bit     
      } 
    }
    else {
      Output ("SD:Online");
      Output ("SD:OBS DIR Exists");
      SD_exists = true;
    }
  }
}

/* 
 *=======================================================================================================================
 * SD_RemoveFile()
 *=======================================================================================================================
 */
void SD_RemoveFile(char *f) {
  if (SD_exists) {
    if (SD.exists(f)) {
      if (SD.remove (f)) {
        sprintf (Buffer32Bytes, "SDRF:%s OK", f);
        Output (Buffer32Bytes);
      }
      else {
        sprintf (Buffer32Bytes, "SDRF:%s ERR", f);
        Output (Buffer32Bytes);
      }
    }
    else {
      sprintf (Buffer32Bytes, "SDRF:%s NF", f);
      Output (Buffer32Bytes);
    }
  }
}

/* 
 *=======================================================================================================================
 * SD_TouchFile()
 *=======================================================================================================================
 */
void SD_TouchFile(char *f) {
  if (SD_exists) {
    if (!SD.exists(f)) {
      File fp = SD.open(f, FILE_WRITE);
      if (fp) {
        fp.close();
        sprintf (Buffer32Bytes, "SDTF:%s OK", f);
        Output (Buffer32Bytes);
      }
      else {
        sprintf (Buffer32Bytes, "SDTF:%s ERR", f);
        Output (Buffer32Bytes);
      }
    }
    else {
      sprintf (Buffer32Bytes, "SDTF:%s Exists", f);
      Output (Buffer32Bytes);
    }
  }
}

/* 
 *=======================================================================================================================
 * SD_RenameFile()
 *=======================================================================================================================
 */
void SD_RenameFile(char *oldf, char *newf) {
  if (SD_exists) {
    // Test for file SIM.TXT
    if (SD.exists(oldf)) {
      File fp = SD.open(oldf, FILE_WRITE);
      if (fp) {
        if (fp.rename(newf)) {
          sprintf (Buffer32Bytes, "SDRN:%s->%s OK", oldf, newf);
          Output (Buffer32Bytes);
        }
        else {
          sprintf (Buffer32Bytes, "SDRN:%s->%s ERR", oldf, newf);
          Output (Buffer32Bytes);
        }
        fp.close();
      }
      else {
        sprintf (Buffer32Bytes, "SDRN:%s OPEN ERR", oldf);
        Output (Buffer32Bytes);
      }
    }
    else {
      sprintf (Buffer32Bytes, "SDRN:%s NF", oldf);
      Output (Buffer32Bytes);
    }
  }
}

/* 
 *=======================================================================================================================
 * SD_LogObservation()
 *=======================================================================================================================
 */
void SD_LogObservation(char *observations) {
  char SD_logfile[24];
  File fp;

  if (!SD_exists) {
    return;
  }

  if (!Time.isValid()) {
    return;
  }
  
  sprintf (SD_logfile, "%s/%4d%02d%02d.log", SD_obsdir, Time.year(), Time.month(), Time.day());
  
  fp = SD.open(SD_logfile, FILE_WRITE); 
  if (fp) {
    fp.println(observations);
    fp.close();
    SystemStatusBits &= ~SSB_SD;  // Turn Off Bit
    Output ("OBS Logged to SD");
  }
  else {
    SystemStatusBits |= SSB_SD;  // Turn On Bit - Note this will be reported on next observation
    Output ("OBS Open Log Err");
    // At thins point we could set SD_exists to false and/or set a status bit to report it
    // sd_initialize();  // Reports SD NOT Found. Library bug with SD
  }
}

/* 
 *=======================================================================================================================
 * SD_N2S_Delete()
 *=======================================================================================================================
 */
bool SD_N2S_Delete() {
  bool result;

  if (SD_exists && SD.exists(SD_n2s_file)) {
    if (SD.remove (SD_n2s_file)) {
      SystemStatusBits &= ~SSB_N2S; // Turn Off Bit
      Output ("N2S->DEL:OK");
      result = true;
    }
    else {
      Output ("N2S->DEL:ERR");
      SystemStatusBits |= SSB_SD; // Turn On Bit
      result = false;
    }
  }
  else {
    Output ("N2S->DEL:NF");
    result = true;
  }
  eeprom.n2sfp = 0;
  EEPROM_Update();
  return (result);
}

/* 
 *=======================================================================================================================
 * SD_NeedToSend_Add()
 *=======================================================================================================================
 */
void SD_NeedToSend_Add(char *observation) {
  File fp;

  if (!SD_exists) {
    return;
  }
  
  fp = SD.open(SD_n2s_file, FILE_WRITE); // Open the file for reading and writing, starting at the end of the file.
                                         // It will be created if it doesn't already exist.
  if (fp) {  
    if (fp.size() > SD_n2s_max_filesz) {
      fp.close();
      Output ("N2S:Full");
      if (SD_N2S_Delete()) {
        // Only call ourself again if we truely deleted the file. Otherwise infinate loop.
        SD_NeedToSend_Add(observation); // Now go and log the data
      }
    }
    else {
      fp.println(observation); //Print data, followed by a carriage return and newline, to the File
      fp.close();
      SystemStatusBits &= ~SSB_SD;  // Turn Off Bit
      Output ("N2S:OBS Added");
    }
  }
  else {
    SystemStatusBits |= SSB_SD;  // Turn On Bit - Note this will be reported on next observation
    Output ("N2S:Open Error");
    // At thins point we could set SD_exists to false and/or set a status bit to report it
    // sd_initialize();  // Reports SD NOT Found. Library bug with SD
  }
}

/*
 *=======================================================================================================================
 * SD_N2S_Publish()
 *=======================================================================================================================
 */
void SD_N2S_Publish() {
  File fp;
  char ch;
  int i;
  int sent=0;

  if (SD_exists && SD.exists(SD_n2s_file)) {
    Output ("N2S:Publish");

    fp = SD.open(SD_n2s_file, FILE_READ); // Open the file for reading, starting at the beginning of the file.

    if (fp) {
      // Delete Empty File or too small of file to be valid
      if (fp.size()<=20) {
        fp.close();
        Output ("N2S:Empty");
        SD_N2S_Delete();
      }
      else {
        if (eeprom.n2sfp) {
          if (fp.size()<=eeprom.n2sfp) {
            // Something wrong. Can not have a file position that is larger than the file
            eeprom.n2sfp = 0;
          }
          else {
            fp.seek(eeprom.n2sfp);  // Seek to where we left off last time.
          }
        }

        // Loop through each line / obs and transmit
        i = 0;
        while (fp.available() && (i < MAX_MSGBUF_SIZE )) {
          ch = fp.read();

          if (ch == 0x0A) {  // newline

            if (Particle_Publish((char *) "SS")) {
              sprintf (Buffer32Bytes, "N2S[%d]->PUB:OK", sent++);
              Output (Buffer32Bytes);
              Serial_write (msgbuf);

              // setup for next line in file
              i = 0;

              // file position is at the start of the next observation or at eof
              eeprom.n2sfp = fp.position();
            }
            else { // Delay then retry
              sprintf (Buffer32Bytes, "N2S[%d]->PUB:RETRY", sent);
              Output (Buffer32Bytes);
              Serial_write (msgbuf);

              // Throttle a little while (5s)
              for (int d=0; d<5; d++) {
                HeartBeat();
                delay (750); 
              }

              if (Particle_Publish((char *) "SS")) {
                sprintf (Buffer32Bytes, "N2S[%d]->PUB:OK", sent++);
                Output (Buffer32Bytes);
                // setup for next line in file
                i = 0;

                // file position is at the start of the next observation or at eof
                eeprom.n2sfp = fp.position();
              }
              else {
                sprintf (Buffer32Bytes, "N2S[%d]->PUB:ERR", sent);
                Output (Buffer32Bytes);
                // On transmit failure, stop processing file.
                break;
              }
            } // RETRY

            // At this point file pointer's position is at the first character of the next line or at eof
          } // Newline
          else if (ch == 0x0D) { // CR
            msgbuf[i] = 0; // null terminate then wait for newline to be read to process OBS
          }
          else {
            msgbuf[i++] = ch;
          }

          // Check for buffer OverRun
          if (i >= MAX_MSGBUF_SIZE) {
            sprintf (Buffer32Bytes, "N2S[%d]->BOR:ERR", sent);
            Output (Buffer32Bytes);
            fp.close();
            SD_N2S_Delete(); // Bad data in the file so delete the file
            return;
          }
        } // end while

        if (fp.available() <= 20) {
          // If at EOF or some invalid amount left then delete the file
          fp.close();
          SD_N2S_Delete();
        }
        else {
          // At this point we sent 0 or more observations but there was a problem.
          // eeprom.n2sfp was maintained in the above read loop. So we will close the
          // file and next time this function is called we will seek to eeprom.n2sfp
          // and start processing from there forward.
          fp.close();
          EEPROM_Update(); // Update file postion in the eeprom.
        }
      }
    }
    else {
        Output ("N2S->OPEN:ERR");
    }
  }
}

