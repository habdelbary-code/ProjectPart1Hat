/*
The below code was modified from
- the SD_test inbuilt Arduino Sketch
- the SimpleWifiServer sketch.
- temp probe datasheet: https://docs.rs-online.com/8ff0/A700000007238410.pdf
- Portions of the code were generated using GPT-5.6-Luna.
- https://www.geeksforgeeks.org/c/time-h-header-file-in-c-with-examples/
- Portions of the code were generated using Claude Sonnet 5
*/

// Include all required libraries
#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "WiFi.h"
#include <GeoIP.h>
#include <OneWire.h>
#include <stdio.h>
#include <time.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>

//// Define all required global variables and pointers, including pins ////
// SD card variables
#define REASSIGN_PINS
int sck = 18;
int miso = 19;
int mosi = 23;
int cs = 5;

const char* logFilePath = "/pika_log.csv";

// SD init is retried a few times before giving up, since this card has
// proven unreliable - a single failed SD.begin() doesn't necessarily mean
// the card is actually absent/dead.
const int SD_INIT_MAX_ATTEMPTS = 5;
const unsigned long SD_INIT_RETRY_DELAY_MS = 1000;
bool sdCardPresent = false;

// ESP32 SPI peripherals (and the GPIO matrix routing used when pins are
// reassigned away from the default VSPI pins) are slower/less tolerant
// than many SD breakout boards expect. Rather than relying on the
// library's default clock (~20-25 MHz, often too fast for a reassigned-pin
// setup), step the clock up from a conservative 1 MHz until one works, and
// remember whichever speed succeeds for subsequent attempts.
const uint32_t SD_SPI_FREQUENCIES[] = {1000000, 2000000, 4000000, 8000000, 20000000};
const int SD_SPI_FREQUENCY_COUNT = sizeof(SD_SPI_FREQUENCIES) / sizeof(SD_SPI_FREQUENCIES[0]);
uint32_t sdWorkingFrequency = 0; // 0 = not yet established

// LED variable
int LED1pin = 21;

// GEO IP variables
GeoIP geoip;
location_t loc;

// Timestamping variables
struct tm* ptr;
time_t t;

// Temp probe variables
const int tempDataPin = 4;
OneWire ds(tempDataPin);

// Digital microphone (I2S, e.g. INMP441 / SPH0645) variables
#define MIC_BCLK 26 // Bit clock
#define MIC_WS 25 // Word select / LR clock
#define MIC_SD 33 // Serial data out from mic

#define SAMPLES 256
#define SAMPLING_FREQUENCY 8000 // Hz, enough for pika call band (400-1300 Hz)

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, (double)SAMPLING_FREQUENCY);

const float pikaFreqLow = 1000.0; //based on https://uh.app/lib/en/American-Pika_sounds_and_tracks
const float pikaFreqHigh = 4000.0;
const double pikaThreshold = 300.0; // tune experimentally

// wifi variables
const char *ssid = "yourwifiname";
const char *password = "yourwifipassword";

//// Helper Function Definitions ////

// Attempts to bring up the SD card, retrying a few times with a short
// delay between attempts since this particular card has proven flaky.
// Re-runs SPI.begin() with the reassigned pins on every attempt and
// verifies cardType() isn't CARD_NONE (SD.begin() can sometimes report
// success even when there's no usable card attached).
bool initSDCard() {
// If a working frequency was already found in a previous call, try it
// first - no need to re-sweep from 1 MHz every single time.
if (sdWorkingFrequency != 0) {
Serial.printf("SD card init: retrying known-good %lu Hz...\n", (unsigned long)sdWorkingFrequency);
SPI.begin(sck, miso, mosi, cs);
if (SD.begin(cs, SPI, sdWorkingFrequency)) {
if (SD.cardType() != CARD_NONE) {
Serial.println("SD card detected and initialized.");
return true;
}
}
SD.end();
Serial.println("Known-good frequency failed this time - falling back to full sweep.");
}

for (int attempt = 1; attempt <= SD_INIT_MAX_ATTEMPTS; attempt++) {
Serial.printf("SD card init attempt %d/%d...\n", attempt, SD_INIT_MAX_ATTEMPTS);

// ESP32 SPI peripherals (especially with GPIO-matrix-routed/reassigned
// pins, as used here) are often unreliable at the library's default
// clock speed. Sweep from a conservative 1 MHz upward on each attempt
// and keep whichever speed first succeeds.
for (int f = 0; f < SD_SPI_FREQUENCY_COUNT; f++) {
uint32_t freq = SD_SPI_FREQUENCIES[f];
Serial.printf("  Trying SPI clock %lu Hz...\n", (unsigned long)freq);

SPI.begin(sck, miso, mosi, cs);

if (SD.begin(cs, SPI, freq)) {
uint8_t cardType = SD.cardType();
if (cardType != CARD_NONE) {
Serial.printf("SD card detected and initialized at %lu Hz.\n", (unsigned long)freq);
sdWorkingFrequency = freq;
return true;
}
Serial.println("  SD.begin() succeeded but no card type detected.");
} else {
Serial.println("  SD.begin() failed at this clock speed.");
}

SD.end();
delay(100);
}

delay(SD_INIT_RETRY_DELAY_MS);
}

Serial.println("ERROR: SD card initialization failed after all attempts.");
Serial.println("NOTE: If this persists, double-check the card is 3.3V-compatible");
Serial.println("(most microSD cards are, but confirm your breakout board does NOT");
Serial.println("level-shift from 5V - running a 5V-logic board into the ESP32's");
Serial.println("3.3V-only GPIOs can damage the pins or cause intermittent faults).");
return false;
}

// Creates the CSV log file with a header row if it doesn't already exist.
void ensureLogFileHeader() {
if (!SD.exists(logFilePath)) {
File file = SD.open(logFilePath, FILE_WRITE);
if (file) {
file.println("time,latitude,longitude,temperature_c");
file.close();
Serial.println("Created new log file with header.");
} else {
Serial.println("ERROR: Could not create log file.");
}
}
}

// Appends one detection as a CSV row to the SD card log file. Checks the
// card is still present before writing (since it's known to drop out),
// and if the write fails, tries one re-init + retry before giving up -
// a flaky card dropping out mid-session shouldn't silently lose data
// without at least one recovery attempt.
bool logToSD(String timeStr, float lat, float lon, float temp) {
timeStr.trim(); // strip trailing newline from asctime()-style timestamp

String row = timeStr + "," + String(lat, 6) + "," + String(lon, 6) + "," + String(temp, 2);

if (!sdCardPresent) {
Serial.println("SD card not present - attempting re-init before logging...");
sdCardPresent = initSDCard();
if (sdCardPresent) {
ensureLogFileHeader();
}
}

if (sdCardPresent) {
File file = SD.open(logFilePath, FILE_APPEND);
if (file) {
file.println(row);
file.close();
Serial.println("Logged to SD: " + row);
return true;
}
Serial.println("SD write failed - attempting one re-init and retry...");
}

// Write failed (or card wasn't present) - try a single recovery attempt.
sdCardPresent = initSDCard();
if (sdCardPresent) {
ensureLogFileHeader();
File file = SD.open(logFilePath, FILE_APPEND);
if (file) {
file.println(row);
file.close();
Serial.println("Logged to SD after recovery: " + row);
return true;
}
}

Serial.println("ERROR: Could not log entry - SD card unavailable.");
return false;
}

// Starts the I2S peripheral in master/RX mode to read from the digital mic
bool startMicrophone() {
i2s_config_t config = {};
config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
config.sample_rate = SAMPLING_FREQUENCY;
config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
config.communication_format = I2S_COMM_FORMAT_I2S;
config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
config.dma_buf_count = 8;
config.dma_buf_len = 256;
config.use_apll = false;
config.tx_desc_auto_clear = false;
config.fixed_mclk = 0;

i2s_pin_config_t pins = {};
pins.bck_io_num = MIC_BCLK;
pins.ws_io_num = MIC_WS;
pins.data_out_num = I2S_PIN_NO_CHANGE;
pins.data_in_num = MIC_SD;

if (i2s_driver_install(I2S_NUM_0, &config, 0, NULL) != ESP_OK) {
return false;
}
if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) {
i2s_driver_uninstall(I2S_NUM_0);
return false;
}

i2s_zero_dma_buffer(I2S_NUM_0);
return true;
}

// Reads one raw 32-bit sample from the I2S mic, converted to a usable amplitude.
// Returns true on success and writes the sample into 'sample'.
bool readMicSample(double &sample) {
int32_t raw = 0;
size_t bytesRead = 0;

esp_err_t result = i2s_read(I2S_NUM_0, &raw, sizeof(raw), &bytesRead, portMAX_DELAY);
if (result != ESP_OK || bytesRead < sizeof(raw)) {
return false;
}

// Most I2S MEMS mics (e.g. INMP441) output 24-bit data left-justified in 32 bits
sample = (double)(raw >> 14);
return true;
}

float getTemp() {
byte data[12];
byte addr[8];
if (!ds.search(addr)) {
ds.reset_search();
return -1000;
}
if (OneWire::crc8(addr, 7) != addr[7]) {
Serial.println("CRC is not valid!");
return -1000;
}

ds.reset();
ds.select(addr);
ds.write(0x44, 1);

ds.reset();
ds.select(addr);
ds.write(0xBE);

for (int i = 0; i < 9; i++) {
data[i] = ds.read();
}
ds.reset_search();

byte MSB = data[1];
byte LSB = data[0];
float tempRead = ((MSB << 8) | LSB);
return tempRead / 16;
}

String getTime() {
time_t now = time(NULL);
struct tm* p = gmtime(&now);
return String(asctime(p));
}

// Samples the digital mic via I2S, runs FFT, and returns true if a pika call is detected
bool detectPikaCall() {
for (int i = 0; i < SAMPLES; i++) {
double sample;
if (!readMicSample(sample)) {
Serial.println("ERROR: I2S mic read failed.");
return false;
}
vReal[i] = sample;
vImag[i] = 0;
}
// Note: i2s_read() blocks until SAMPLING_FREQUENCY worth of samples are
// available (since the I2S driver is configured for that sample rate),
// so no manual timing loop is needed here as it was for analogRead().

FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
FFT.compute(FFT_FORWARD);
FFT.complexToMagnitude();

for (int i = 1; i < (SAMPLES / 2); i++) {
float freq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
if (freq >= pikaFreqLow && freq <= pikaFreqHigh) {
if (vReal[i] > pikaThreshold) {
return true;
}
}
}
return false;
}

// Blinks the LED a set number of times, used as a startup "all systems go" signal
void blinkLED(int times, int onMs = 200, int offMs = 200) {
for (int i = 0; i < times; i++) {
digitalWrite(LED1pin, HIGH);
delay(onMs);
digitalWrite(LED1pin, LOW);
delay(offMs);
}
}

// Checks that WiFi, mic, temperature probe, and SD card are all ready.
// Returns true only if everything checks out.
bool systemsReady() {
bool wifiOK = (WiFi.status() == WL_CONNECTED);

// Mic check: confirm the I2S mic is returning plausible (non-zero) values
double micSample = 0;
bool micOK = readMicSample(micSample) && (micSample != 0);

// Temp probe check: a failed read returns -1000 in getTemp()
float tempCheck = getTemp();
bool tempOK = (tempCheck != -1000);

Serial.printf("WiFi: %s | Mic: %s | Temp probe: %s | SD card: %s\n",
wifiOK ? "OK" : "FAIL",
micOK ? "OK" : "FAIL",
tempOK ? "OK" : "FAIL",
sdCardPresent ? "OK" : "FAIL");

return wifiOK && micOK && tempOK && sdCardPresent;
}

// Execution of code
void setup() {
Serial.begin(9600);

// Set pin modes before using any of them.
pinMode(LED1pin, OUTPUT);
pinMode(cs, OUTPUT);     // SD chip-select must be an output before SPI/SD.begin()
pinMode(sck, OUTPUT);
pinMode(mosi, OUTPUT);
pinMode(miso, INPUT);
pinMode(tempDataPin, INPUT); // OneWire drives this itself, but set a sane default

if (!startMicrophone()) {
Serial.println("ERROR: Digital microphone initialization failed.");
}

// Bring up the SD card before anything else that depends on it. Keeps
// retrying indefinitely (with LED feedback) since without storage there's
// nowhere to log detections.
sdCardPresent = initSDCard();
while (!sdCardPresent) {
Serial.println("SD card not ready - blinking fast and retrying...");
blinkLED(5, 100, 100); // fast blink = SD card fault indicator
sdCardPresent = initSDCard();
}
ensureLogFileHeader();

// Connect to wifi network
Serial.print("Connecting to ");
Serial.println(ssid);
WiFi.begin(ssid, password);

while (WiFi.status() != WL_CONNECTED) {
delay(500);
Serial.print(".");
}
Serial.println("\nWiFi connected");

// NTP sync: some networks (e.g. phone hotspots) block/filter NTP's UDP
// port 123 even though HTTP/HTTPS work fine. Use a bounded timeout instead
// of looping forever, so a blocked NTP request can't freeze the device.
configTime(0, 0, "pool.ntp.org", "time.nist.gov");
struct tm timeinfo;
unsigned long ntpStart = millis();
const unsigned long NTP_TIMEOUT_MS = 15000; // give up after 15s
bool timeSynced = false;
while (!(timeSynced = getLocalTime(&timeinfo, 100))) {
if (millis() - ntpStart > NTP_TIMEOUT_MS) {
Serial.println("NTP sync timed out - continuing without accurate time.");
break;
}
}
if (timeSynced) {
Serial.println("Time synced!");
}

// Confirm all components are detected and working before signalling ready.
while (!systemsReady()) {
Serial.println("Waiting for all systems to be ready...");
delay(1000);
}

Serial.println("All systems ready - gathering data securely.");
blinkLED(3); // 3 blinks = startup success signal
}

void loop() {
bool pikaDetected = detectPikaCall();

if (pikaDetected) {
Serial.println("Pika call detected!");

String currentTime = getTime();
float temperature = getTemp();
loc = geoip.getGeoFromWiFi(true);

// Log the detection regardless of whether location lookup succeeded.
// If GeoIP fails, fall back to sentinel values (NAN) rather than
// skipping the entry entirely, so every detection still makes it
// into the log.
float latitude = NAN;
float longitude = NAN;

if (loc.status) {
latitude = loc.latitude;
longitude = loc.longitude;
} else {
Serial.println("Location lookup failed - logging detection without coordinates");
}

Serial.println("Logging detection to SD card...");
logToSD(currentTime, latitude, longitude, temperature);
blinkLED(2);
}

delay(500); // short pause before next listening window
}

