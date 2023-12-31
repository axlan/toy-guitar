//------------------------------------------------------------------------------------------------------------------------
//
// Title: ESP32 Code to Play Songs Off an SD Card on Button Pushes
//
// When a button is pressed, play the corrsponding file from the SD card.
// Create an ad-hoc Wifi AP "Guitar-AP" with an FTP server to update the songs, or upload a firmware update.
// After 2 minutes of inactivity, enter deep sleep until a button is pressed.
//
//    https://www.robopenguins.com/toy-guitar/
//
//------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------
//
// Includes

// Load Wi-Fi library
#include <WiFi.h>
#include <SD.h>         // SD Card library, usually part of the standard install
#include <Update.h>     // Firmware update library.
#include "driver/i2s.h" // Library of I2S routines, comes with ESP32 standard install

#include "ESP-FTP-Server-Lib.h"

#include "button_interface.h"
#include "wav_utils.h"

//------------------------------------------------------------------------------------------------------------------------
// Constants

//    SD Card
#define SD_CS 5 // SD Card chip select

//    I2S
#define I2S_DOUT 25 // i2S Data out oin
#define I2S_BCLK 27 // Bit clock
#define I2S_LRC 26  // Left/Right clock, also known as Frame clock or word select
#define I2S_NUM 0   // i2s port number

// Wav File reading
#define NUM_BYTES_TO_READ_FROM_FILE 1024 // How many bytes to read from wav file at a time

static constexpr float SAMPLE_SCALE_FACTOR = 0.4;

// Replace with your network credentials
const char *SSID = "Guitar-AP";

const char *FTP_USER = "happy";
const char *FTP_PASS = "halloween";

static const char *FIRMWARE_PATH = "/firmware.bin";

// Sleep after 2 minutes if no audio plays, or FTP connects.
static constexpr unsigned long SLEEP_AFTER_MS = 1000 * 120;

//------------------------------------------------------------------------------------------------------------------------
// Struct Definitions

struct PlaybackState
{
  /**
   * For the most part, we use the state of the file object to control playback.
   * We'll playback from the file as long is it's open, and rely on it to do an
   * empty when we reach the end.
   */
  File wav_file;
  /**
   * Since it takes multiple calls to the I2S library to output the data from
   * each SD car read, we use the following values to manage sending out the
   * data from the SD card read.
   */
  bool need_file_read = true;
  size_t i2s_buffer_idx = 0;
  byte samples[NUM_BYTES_TO_READ_FROM_FILE];
};

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100, // Note, this and the next two values will be updated by i2s_set_clk
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
    .dma_buf_count = 8,                       // 8 buffers
    .dma_buf_len = 64,                        // 64 bytes per buffer, so 8K of buffer space
    .use_apll = 0,
    .tx_desc_auto_clear = true,
    .fixed_mclk = -1};

// These are the physical wiring connections to our I2S decoder board/chip from the esp32, there are other connections
// required for the chips mentioned at the top (but not to the ESP32), please visit the page mentioned at the top for
// further information regarding these other connections.

static const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,          // The bit clock connectiom, goes to pin 27 of ESP32
    .ws_io_num = I2S_LRC,            // Word select, also known as word select or left right clock
    .data_out_num = I2S_DOUT,        // Data out from the ESP32, connect to DIN on 38357A
    .data_in_num = I2S_PIN_NO_CHANGE // we are not interested in I2S data into the ESP32
};

//------------------------------------------------------------------------------------------------------------------------
// Global Variables/objects

File WavFile;                                // Object for root of SD card directory
static const i2s_port_t i2s_num = I2S_NUM_0; // i2s port number
FTPServer ftp;

//------------------------------------------------------------------------------------------------------------------------
// Functions

void SDCardInit()
{
  Serial.println("SD Init");
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH); // SD card chips select, must use GPIO 5 (ESP32 SS)
  if (!SD.begin(SD_CS))
  {
    Serial.println("Error talking to SD card!");
    while (true)
      ; // end program
  }
}

/**
 * If a file `firmware.bin` is on the SD card, use it for a firmware update,
 * then delete it off the SD card.
 */
void CheckForUpdate()
{
  Serial.print(F("\nSearch for firmware.."));
  File firmware = SD.open(FIRMWARE_PATH);
  if (firmware)
  {
    Serial.println(F("found!"));
    Serial.println(F("Try to update!"));

    Update.onProgress([](size_t currSize, size_t totalSize)
                      { Serial.printf("CALLBACK:  Update process at %d of %d bytes...\n", currSize, totalSize); });

    Update.begin(firmware.size(), U_FLASH);
    Update.writeStream(firmware);
    if (Update.end())
    {
      Serial.println(F("Update finished!"));
    }
    else
    {
      Serial.println(F("Update error!"));
      Serial.println(Update.getError());
    }

    firmware.close();
    if (SD.remove(FIRMWARE_PATH))
    {
      Serial.println(F("Firmware delete successfully!"));
    }
    else
    {
      Serial.println(F("Firmware rename error!"));
    }
    delay(2000);

    ESP.restart();
  }
  else
  {
    Serial.println(F("not found!"));
  }
}

void InitWifiAP()
{
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(SSID);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

void setup()
{
  Serial.begin(115200); // Used for info/debug

  // Setup the buttons used to control playback and wake from sleep.
  InitButtonPins();
  // Input buttons to wake from sleep are 33, 14, 13, 12
  static constexpr uint64_t WAKE_MASK = (1ull << 33) | (1ull << 14) | (1ull << 13) | (1ull << 12);
  // Need to leave ESP_PD_DOMAIN_RTC_PERIPH since we need to software IO pull-downs.
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_sleep_enable_ext1_wakeup(WAKE_MASK, ESP_EXT1_WAKEUP_ANY_HIGH);

  SDCardInit();
  CheckForUpdate();

  InitWifiAP();

  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  i2s_set_pin(i2s_num, &pin_config);

  ftp.addUser(FTP_USER, FTP_PASS);
  ftp.addFilesystem("SD", &SD);
  ftp.begin();
}

/**
 * @brief Adjust the amplitude (volume) of output audio.
 * 
 * @tparam T The integer type for the audio data.
 * @param sample_data The buffer containing audio data to scale.
 * @param num_bytes The length of `sample_data`.
 * @param scale The value to multiply the audio data by.
 */
template <typename T>
void ScaleSamples(byte* sample_data, size_t num_bytes, float scale) {
  T* samples = reinterpret_cast<T*>(sample_data);
  size_t num_samples = num_bytes / sizeof(T);
  for(size_t i = 0; i < num_samples; i++) {
    samples[i] *= scale;
  }
}

/**
 * Load sound data from SD card for currently open file. 
 */
void ReadFile(PlaybackState &playback_state)
{
  size_t bytes_read = playback_state.wav_file.read(playback_state.samples, NUM_BYTES_TO_READ_FROM_FILE); // Read in the bytes from the file

  // When we get to the end of the file we'll either get a partial, or empty
  // read. For simplicity we drop it and close the file to indicate the end of playback.
  if (bytes_read != NUM_BYTES_TO_READ_FROM_FILE) // Have we read in all the data?
  {
    playback_state.wav_file.close();
    Serial.println("Song ended");
  }
  // The template type for this should really be being set based on the wav header.
  ScaleSamples<int16_t>(playback_state.samples, NUM_BYTES_TO_READ_FROM_FILE, SAMPLE_SCALE_FACTOR);
}

bool FillI2SBuffer(PlaybackState &playback_state)
{
  // Writes bytes to buffer, returns true if all bytes sent else false, keeps track itself of how many left
  // to write, so just keep calling this routine until returns true to know they've all been written, then
  // you can re-fill the buffer

  size_t BytesWritten = 0; // Returned by the I2S write routine

  // To make the code easier to understand I'm using to variables to some calculations, normally I'd write this calcs
  // directly into the line of code where they belong, but this make it easier to understand what's happening

  uint8_t *data_ptr = playback_state.samples + playback_state.i2s_buffer_idx;           // Set address to next byte in buffer to send out
  uint16_t bytes_to_send = NUM_BYTES_TO_READ_FROM_FILE - playback_state.i2s_buffer_idx; // This is amount to send (total less what we've already sent)
  i2s_write(i2s_num, data_ptr, bytes_to_send, &BytesWritten, 1);                        // Send the bytes, wait 1 RTOS tick to complete
  playback_state.i2s_buffer_idx += BytesWritten;                                        // increase by number of bytes actually written

  if (playback_state.i2s_buffer_idx >= NUM_BYTES_TO_READ_FROM_FILE)
  {
    // sent out all bytes in buffer, reset and return true to indicate this
    playback_state.i2s_buffer_idx = 0;
    return true;
  }
  else
  {
    return false; // Still more data to send to I2S so return false to indicate this
  }
}

void PlayWav(PlaybackState &playback_state)
{
  if (playback_state.need_file_read) // Read next chunk of data in from file if needed
  {
    ReadFile(playback_state);              // Read data into our memory buffer, return num bytes read in
    playback_state.need_file_read = false; // Switch to sending the buffer to the I2S
  }
  else
  {
    playback_state.need_file_read = FillI2SBuffer(playback_state); // We keep calling this routine until it returns true, at which point
                                                                   // this will swap us back to Reading the next block of data from the file.
                                                                   // Reading true means it has managed to push all the data to the I2S
                                                                   // Handler, false means there still more to do and you should call this
                                                                   // routine again and again until it returns true.
  }
}

void InitPlayback(int button, PlaybackState &playback_state)
{
  // get the wav file from the SD card
  String file_name = String("/song") + String(button) + ".wav";
  playback_state.wav_file = SD.open(file_name); // Open the wav file (i.e. /song1-6.wav)
  Serial.println("File Open: " + file_name);
  if (!playback_state.wav_file)
  {
    Serial.println("Could not open: " + file_name);
  }
  else
  {
    WavHeader wav_header;
    ReadWavHeader(playback_state.wav_file, &wav_header);
    DumpWAVHeader(&wav_header); // Dump the header data to serial, optional!
    if (ValidWavData(&wav_header))
    {
      // set sample rate, bit width, and channels.
      i2s_set_clk(i2s_num, wav_header.SampleRate,
                  (i2s_bits_per_sample_t)wav_header.BitsPerSample,
                  (i2s_channel_t)wav_header.NumChannels);
      playback_state.i2s_buffer_idx = 0;
      playback_state.need_file_read = true;
    }
  }
}

void loop()
{
  static uint8_t last_pressed = 0;
  static PlaybackState playback_state;
  static unsigned long long last_active_time = millis();

  // Check for a new button press.
  uint8_t pressed = GetLowestPressed();
  if (pressed != last_pressed)
  {
    delay(5);
    // The buttons have a lot of "bounce", so make sure the button has
    // stabilized before responding to a press.
    if (pressed == GetLowestPressed())
    {
      Serial.print("Pressed: ");
      Serial.println(pressed);
      if (pressed != 0)
      {
        InitPlayback(pressed, playback_state);
      }
      last_pressed = pressed;
    }
  }

  // Move data from the SD card to the I2S chip until the file has been closed.
  if (playback_state.wav_file)
  {
    PlayWav(playback_state); // Have to keep calling this to keep the wav file playing
    last_active_time = millis();
  }

  ftp.handle();
  if (ftp.countConnections() > 0)
  {
    last_active_time = millis();
  }

  // If it's been `SLEEP_AFTER_MS` since the last button press or an FTP
  // connection, enter deep sleep. Pressing buttons 1-4 or power cycling will
  // wake the board up.
  if (millis() - last_active_time > SLEEP_AFTER_MS)
  {
    Serial.print("Entering deep sleep.");
    esp_deep_sleep_start();
  }
}
