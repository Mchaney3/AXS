/*
 * This program will format SD/SDHC/SDXC cards.
 * Warning all data will be deleted!
 *
 * This program attempts to match the format
 * generated by SDFormatter available here:
 *
 * http://www.sdcard.org/consumers/formatter/
 *
 * For very small cards this program uses FAT16
 * and the above SDFormatter uses FAT12.
 */
#include "SdFat.h"
#include "sdios.h"

/*
  Set DISABLE_CS_PIN to disable a second SPI device.
  For example, with the Ethernet shield, set DISABLE_CS_PIN
  to 10 to disable the Ethernet controller.
*/
const int8_t DISABLE_CS_PIN = -1;
/*
  Change the value of SD_CS_PIN if you are using SPI
  and your hardware does not use the default value, SS.
  Common values are:
  Arduino Ethernet shield: pin 4
  Sparkfun SD shield: pin 8
  Adafruit SD shields and modules: pin 10
*/

/********   Working Section   **************/

#define SD_FAT_TYPE 2
//Initialize SD card on SdFat SoftSPI
// Chip select may be constant or RAM variable.
const uint8_t SD_CS_PIN = 33;
// Pin numbers in templates must be constants.
const uint8_t SOFT_MISO_PIN = 25;
const uint8_t SOFT_MOSI_PIN = 15;
const uint8_t SOFT_SCK_PIN  = 14;
// SdFat software SPI template
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
// Speed argument is ignored for software SPI.
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(50), &softSpi)
// #define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(40), &softSpi)


or for regular sd

SPI.begin(14, 15, 25, 5); //SCK, MISO, MOSI,SS

  if (SD.begin(13, SPI)) {
      Serial.println("SD opened!");
  }

// My other options
//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, &softSpi)
//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(80))
//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)

//------------------------------------------------------------------------------
void setup() {
#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

void setup() {
  Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial) {
    yield();
  }
  Serial.println("Type any character to start");
  while (!Serial.available()) {
    yield();
  }

  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt();
  }

  if (!file.open("SoftSPI.txt", O_RDWR | O_CREAT)) {
    sd.errorHalt(F("open failed"));
  }
  file.println(F("This line was printed using software SPI."));

  file.rewind();

  while (file.available()) {
    Serial.write(file.read());
  }

  file.close();

  Serial.println(F("Done."));


  //Mount as file system
  fs::SD dir = sd.openDir("/");
}
void loop() {
}