/*
 * ======================================================================================================================
 *  eeprom.cpp - EEPROM Functions
 * ======================================================================================================================
 */
#include <RTClib.h>

#include "include/qc.h"
#include "include/sdcard.h"
#include "include/main.h"
#include "include/output.h"
#include "include/eeprom.h"

/*
 * ======================================================================================================================
 * Variables and Data Structures
 * =======================================================================================================================
 */
EEPROM_NVM eeprom;
int eeprom_address = 0;
bool eeprom_valid = false;

/*
 * ======================================================================================================================
 * Fuction Definations
 * =======================================================================================================================
 */

/* 
 *=======================================================================================================================
 * EEPROM_ChecksumCompute()
 *=======================================================================================================================
 */
unsigned long EEPROM_ChecksumCompute() {
  unsigned long checksum=0;
  checksum += (unsigned long) eeprom.ts;
  checksum += (unsigned long) eeprom.n2sfp;
  return (checksum);
}

/* 
 *=======================================================================================================================
 * EEPROM_ChecksumUpdate()
 *=======================================================================================================================
 */
void EEPROM_ChecksumUpdate() {
  eeprom.checksum = EEPROM_ChecksumCompute();
}

/* 
 *=======================================================================================================================
 * EEPROM_Valid()
 *=======================================================================================================================
 */
bool EEPROM_Valid() {
  unsigned long checksum = EEPROM_ChecksumCompute();

  if (checksum == eeprom.checksum) {
    if (eeprom.n2sfp > (SD_n2s_max_filesz + 1000)) {
      return (false);    
    }
    else {
      return (true);
    }
  }
  else {
    return (false);
  }
}

/* 
 *=======================================================================================================================
 * EEPROM_Reset() - Reset to default values
 *                  Requires system clock to be valid
 *=======================================================================================================================
 */
void EEPROM_Reset(time32_t current_time) {
  if (Time.isValid()) {
    eeprom.ts = current_time;
    eeprom.n2sfp = 0;
    EEPROM_ChecksumUpdate();
    EEPROM.put(eeprom_address, eeprom);
    Output("EEPROM RESET");
  }
  else {
    Output("EEPROM RESET ERROR");
  }
}

/* 
 *=======================================================================================================================
 * EEPROM_Initialize() - Check status of EEPROM information and determine status
 *                       Requires system clock to be valid
 *=======================================================================================================================
 */
void EEPROM_Initialize() {
  if (Time.isValid()) {
    time32_t current_time = Time.now();

    EEPROM.get(eeprom_address, eeprom);

    if (!EEPROM_Valid()) {
      EEPROM_Reset(current_time);
    }
    else {
      Output("EEPROM VALID");
    }
    eeprom_valid = true; 
  }
  else {
    Output("EEPROM INIT ERROR");
  }
}

/* 
 *=======================================================================================================================
 * EEPROM_Update() - Check status of EEPROM information and determine status
 *=======================================================================================================================
 */
void EEPROM_Update() {
  if (eeprom_valid) {
    eeprom.ts = Time.now();
    EEPROM_ChecksumUpdate();
    EEPROM.put(eeprom_address, eeprom);
    Output("EEPROM UPDATED");
  }
}

/* 
 *=======================================================================================================================
 * EEPROM_Dump() - 
 *=======================================================================================================================
 */
void EEPROM_Dump() {
  size_t EEPROM_length = EEPROM.length();

  EEPROM.get(eeprom_address, eeprom);

  unsigned long checksum = EEPROM_ChecksumCompute();

  Output("EEPROM DUMP");

  sprintf (msgbuf, " LEN:%d", EEPROM_length);
  Output(msgbuf);

  sprintf (Buffer32Bytes, " TS:%lu", eeprom.ts);
  Output (Buffer32Bytes);

  sprintf (Buffer32Bytes, " N2SFP:%lu", eeprom.n2sfp);
  Output (Buffer32Bytes);

  sprintf (Buffer32Bytes, " CS:%lu", eeprom.checksum);
  Output (Buffer32Bytes);

  sprintf (Buffer32Bytes, " CSC:%lu", checksum);
  Output (Buffer32Bytes);
}
