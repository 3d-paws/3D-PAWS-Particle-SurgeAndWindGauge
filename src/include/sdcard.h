/*
 * ======================================================================================================================
 *  sdcard.h - SD Card Definations
 * ======================================================================================================================
 */
#include <SdFat.h>

#if (PLATFORM_ID == PLATFORM_MSOM)
#define SD_ChipSelect D29               // Pin 24 A6/D29
#else
#define SD_ChipSelect D5                // GPIO 10 is Pin 10 on Feather and D5 on Particle Boron Board
#endif

// Extern variables
extern SdFat SD;
extern File SD_fp;
extern char SD_obsdir[];
extern bool SD_exists;
extern char SD_n2s_file[];
extern uint32_t SD_n2s_max_filesz;
extern char SD_sim_file[];
extern char SD_simold_file[];
extern char SD_wifi_file[];
extern char SD_INFO_FILE[];
extern char SD_5M_DIST_FILE[];
extern char SD_ELEV_FILE[];

// Function prototypes
void SD_initialize();
void SD_RemoveFile(char *f);
void SD_TouchFile(char *f);
void SD_RenameFile(char *oldf, char *newf);
void SD_LogObservation(char *observations);
bool SD_N2S_Delete();
void SD_NeedToSend_Add(char *observation);
void SD_N2S_Publish();