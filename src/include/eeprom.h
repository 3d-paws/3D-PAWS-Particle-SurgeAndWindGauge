/*
 * ======================================================================================================================
 *  eeprom.h - EEPROM Definations
 * ======================================================================================================================
 */

/*
 * ======================================================================================================================
 *  EEPROM NonVolitileMemory - stores rain totals in persistant memory
 * ======================================================================================================================
 */
typedef struct {
    time32_t ts;         // timestamp of last modification
    unsigned long n2sfp; // sd need 2 send file position
    unsigned long checksum;
} EEPROM_NVM;

// Extern variables
extern EEPROM_NVM eeprom;
extern bool eeprom_valid;
extern bool eeprom_exists;

// Function prototype
unsigned long EEPROM_ChecksumCompute();
void EEPROM_ChecksumUpdate();
bool EEPROM_ChecksumValid();
void EEPROM_Validate();
void EEPROM_Update();
void EEPROM_Dump();
void EEPROM_Initialize();