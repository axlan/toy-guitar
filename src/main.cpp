//------------------------------------------------------------------------------------------------------------------------
//
// Title: SD Card Wav Player
//
// Description:
//    Simple example to demonstrate the fundementals of playing WAV files (digitised sound) from an SD Card via the I2S
//    interface of the ESP32. Plays WAV file from SD card. To keep this simple the WAV must be stereo and 16bit samples.
//    The Samples Per second can be anything. On the SD Card the wav file must be in root and called wavfile.wav
//    Libraries are available to play WAV's on ESP32, this code does not use these so that we can see what is happening.
//    This is part 3 in a tutorial series on using I2S on ESP32. See the accompanying web page (which will also include
//    a tutorial video).
//
// Boring copyright/usage information:
//    (c) XTronical, www.xtronical.com
//    Use as you wish for personal or monatary gain, or to rule the world (if that sort of thing spins your bottle)
//    However you use it, no warrenty is provided etc. etc. It is not listed as fit for any purpose you perceive
//    It may damage your house, steal your lover, drink your beers and more.
//
//    http://www.xtronical.com/i2s-ep3
//
//------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------
//
// Includes

#include "SD.h"         // SD Card library, usually part of the standard install
#include "driver/i2s.h" // Library of I2S routines, comes with ESP32 standard install

#include "button_interface.h"
#include "wav_utils.h"

//------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------
// Defines

//    SD Card
#define SD_CS 5 // SD Card chip select

//    I2S
#define I2S_DOUT 25 // i2S Data out oin
#define I2S_BCLK 27 // Bit clock
#define I2S_LRC 26  // Left/Right clock, also known as Frame clock or word select
#define I2S_NUM 0   // i2s port number

// Wav File reading
#define NUM_BYTES_TO_READ_FROM_FILE 1024 // How many bytes to read from wav file at a time

struct PlaybackState
{
  File wav_file;
  byte samples[NUM_BYTES_TO_READ_FROM_FILE];
  size_t i2s_buffer_idx = 0;
  bool need_file_read = true;
};

//------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------
// structures and also variables
//  I2S configuration

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

//  Global Variables/objects

File WavFile;                                // Object for root of SD card directory
static const i2s_port_t i2s_num = I2S_NUM_0; // i2s port number

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

void setup()
{
  Serial.begin(115200); // Used for info/debug

  InitButtonPins();

  SDCardInit();
  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  i2s_set_pin(i2s_num, &pin_config);
}

void ReadFile(PlaybackState &playback_state)
{
  size_t bytes_read = playback_state.wav_file.read(playback_state.samples, NUM_BYTES_TO_READ_FROM_FILE); // Read in the bytes from the file

  if (bytes_read != NUM_BYTES_TO_READ_FROM_FILE) // Have we read in all the data?
  {
    playback_state.wav_file.close();
    Serial.println("Song ended");
  }
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
  playback_state.wav_file = SD.open(file_name); // Open the wav file (/song1-6.wav)
  Serial.println("File Open: " + file_name);
  if (!playback_state.wav_file)
  {
    Serial.println("Could not open: " + file_name);
  }
  else
  {
    WavHeader wav_header;
    ReadWavHeader(playback_state.wav_file, &wav_header);
    // We have to typecast to bytes for the "read" function
    DumpWAVHeader(&wav_header); // Dump the header data to serial, optional!
    if (ValidWavData(&wav_header))
    {
      // i2s_set_sample_rates(i2s_num, wav_header.SampleRate); // set sample rate
      i2s_set_clk(i2s_num, wav_header.SampleRate, (i2s_bits_per_sample_t)wav_header.BitsPerSample, (i2s_channel_t)wav_header.NumChannels); // set sample rate, bit width, and channels.
      playback_state.i2s_buffer_idx = 0;
      playback_state.need_file_read = true;
    }
  }
}

void loop()
{
  static uint8_t last_pressed = 0;
  static PlaybackState playback_state;
  uint8_t pressed = GetLowestPressed();
  if (pressed != last_pressed)
  {
    delay(5);
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

  if (playback_state.wav_file)
  {
    PlayWav(playback_state); // Have to keep calling this to keep the wav file playing
  }
}
