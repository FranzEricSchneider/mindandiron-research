#include <SPI.h>
#include "DW1000.h"
#include "MMRanging.h"

// DecaWave 0.2 up and running
// DEVICE_ID register: 4294967295
// EUI register: 18446744073709551615
// Voltage: 3.75

const int chipSelectPin = 10;  // Teensy
SPISettings DW1000Settings(4000000, MSBFIRST, SPI_MODE0);  // Teensy

void setup() {
  Serial.begin(9600);
  Serial.println("Serial up and running");
  delay(2000);


// From DW1000.cpp ---------------------------------------------------------
  SPI.begin(); // start the SPI library:
  SPI.beginTransaction(DW1000Settings);
  pinMode(chipSelectPin, OUTPUT); // initalize the chip select pins:
  deselect(); // Chip must be deselected first

  Serial.println("SPI up and running");
  delay(2000);

  // Default clock divider is 4 MHz
  // Default significant bit is MSBFirst
  // Default mode is Mode0

  // we can do a soft reset if we want to (only needed for debugging)
  resetAll();
  // important everytime DW1000 initialises/awakes otherwise the LDE algorithm must be turned of or there's receiving malfunction see User Manual LDELOAD on p22 & p158
  loadLDE();
  Serial.println("Reset and loading");
  delay(2000);

  // Configuration TODO: make method for that
  writeRegister8(DW1000_SYS_CFG, 3, 0x20); // enable auto reenabling receiver after error
  writeRegister8(DW1000_SYS_CFG, 2, 0x03); // enable 1024 byte frames

// From MMRanging.cpp ---------------------------------------------------------
  message[0] = '\0';
  messageRX[0] = '\0';
  event_i = 0;
  counter = 0;
  setInterrupt(true, true);
  startRX();

  Serial.println("Starting RX");
  delay(2000);

// From main.cpp ---------------------------------------------------------
  Serial.println("DecaWave 0.2 up and running");
  // basic methods called to check if we have a working SPI connection
  setEUI(0xFAEDCD01FAEDCD01LL);
  Serial.print("DEVICE_ID register: ");
  Serial.println(getDeviceID());
  Serial.print("EUI register: ");
  print64(getEUI());
  Serial.print("Voltage: ");
  Serial.println(getVoltage());
  receiver = true;
}

void loop() {
  fakeISR();
  delay(100);

// From main.cpp ---------------------------------------------------------
  for(int j = 0; j < 10; j++) {
    if(event[j][0] == '!') {
      Serial.print(event[j]); Serial.print(" Time: ");
      Serial.print(eventtimes[j]*MMRANGING_TIMEUNIT); Serial.println(" us");
      event[j][0] = 'X';
    }
  }
  if (!receiver) {
    requestRanging();
    delay(1);
  }
}

void callbackRX() {
    RX_timestamp = getRXTimestamp();
    Serial.print("RX_timestamp: "); print64(RX_timestamp);
    Serial.print("TX_timestamp: "); print64(TX_timestamp);
    receiveString(messageRX);
    if (receiver) {
        message[0] = 'A';                               // acknowledge messages
        for(int i = 0; i < 10; i++)
            message[i+1] = messageRX[i];
        sendString(message);
    }
    eventtimes[event_i] = RX_timestamp - TX_timestamp;  // TODO: can give some wrong values because of timer reset after 17 seconds
    event[event_i][0] = '!';
    event[event_i][1] = 'R';
    event[event_i][2] = ' ';
    for(int i = 0; i < 10; i++)
        event[event_i][i+3] = messageRX[i];
    if (event_i == 8)
        event_i = 0;
    else
        event_i++;
    startRX();
}

void callbackTX() {
    TX_timestamp = getTXTimestamp();
    eventtimes[event_i] = 0;
    event[event_i][0] = '!';
    event[event_i][1] = 'S';
    event[event_i][2] = ' ';
    for(int i = 0; i < 10; i++)
        event[event_i][i+3] = message[i];
    if (event_i == 8)
        event_i = 0;
    else
        event_i++;
}

void requestRanging() {
    // send numbers to acknowledge
    sprintf(message, "%d", counter);
    counter++;
    sendString(message);
}

// DW1000(PinName MOSI, PinName MISO, PinName SCLK, PinName CS, PinName IRQ) : irq(IRQ), spi(MOSI, MISO, SCLK), cs(CS) {
//   setCallbacks(NULL, NULL);
  
//   // deselect();                         // Chip must be deselected first
//   // spi.format(8,0);                    // Setup the spi for standard 8 bit data and SPI-Mode 0 (GPIO5, GPIO6 open circuit or ground on DW1000)
//   // spi.frequency(1000000);             // with a 1MHz clock rate (worked up to 49MHz in our Test)
  
//   // resetAll();                         // we can do a soft reset if we want to (only needed for debugging)
//   // loadLDE();                          // important everytime DW1000 initialises/awakes otherwise the LDE algorithm must be turned of or there's receiving malfunction see User Manual LDELOAD on p22 & p158
  
//   // // Configuration TODO: make method for that
//   // writeRegister8(DW1000_SYS_CFG, 3, 0x20); // enable auto reenabling receiver after error
//   // writeRegister8(DW1000_SYS_CFG, 2, 0x03); // enable 1024 byte frames
  
//   irq.rise(this, &ISR);       // attach Interrupt handler to rising edge
// }

// void setCallbacks(void (*callbackRX)(void), void (*callbackTX)(void)) {
//   bool RX = false;
//   bool TX = false;
//   if (callbackRX) {
//     callbackRX.attach(callbackRX);
//     RX = true;
//   }
//   if (callbackTX) {
//     callbackTX.attach(callbackTX);
//     TX = true;
//   }
//   setInterrupt(RX,TX);
// }

uint32_t getDeviceID() {
  uint32_t result;
  readRegister(DW1000_DEV_ID, 0, (uint8_t*)&result, 4);
  return result;
}

uint64_t getEUI() {
  uint64_t result;
  readRegister(DW1000_EUI, 0, (uint8_t*)&result, 8);
  return result;
}

void setEUI(uint64_t EUI) {
  writeRegister(DW1000_EUI, 0, (uint8_t*)&EUI, 8);
}

float getVoltage() {
  uint8_t buffer[7] = {0x80, 0x0A, 0x0F, 0x01, 0x00};             // algorithm form User Manual p57
  writeRegister(DW1000_RF_CONF, 0x11, buffer, 2);
  writeRegister(DW1000_RF_CONF, 0x12, &buffer[2], 1);
  writeRegister(DW1000_TX_CAL, 0x00, &buffer[3], 1);
  writeRegister(DW1000_TX_CAL, 0x00, &buffer[4], 1);
  readRegister(DW1000_TX_CAL, 0x03, &buffer[5], 2);               // get the 8-Bit readings for Voltage and Temperature
  float Voltage = buffer[5] * 0.0057 + 2.3;
  //float Temperature = buffer[6] * 1.13 - 113.0;                 // TODO: getTemperature was always ~35 degree with better formula/calibration see instance_common.c row 391
  return Voltage;
}

uint64_t getStatus() {
  return readRegister40(DW1000_SYS_STATUS, 0);
}

uint64_t getRXTimestamp() {
  return readRegister40(DW1000_RX_TIME, 0);
}

uint64_t getTXTimestamp() {
  return readRegister40(DW1000_TX_TIME, 0);
}

void sendString(char* message) {
  sendFrame((uint8_t*)message, strlen(message)+1);
}

void receiveString(char* message) {
  uint16_t framelength = getFramelength();
  readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)message, framelength);  // get data from buffer
}

void sendFrame(uint8_t* message, uint16_t length) {
  if (length >= 1021) length = 1021;                              // check for maximim length a frame can have            TODO: 127 Byte mode?
  writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer

  uint8_t backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
  length += 2;                                                    // including 2 CRC Bytes
  length = ((backup & 0xFC) << 8) | (length & 0x03FF);
  writeRegister16(DW1000_TX_FCTRL, 0, length);

  stopTRX();                                                      // stop receiving
  writeRegister8(DW1000_SYS_CTRL, 0, 0x02);                       // trigger sending process by setting the TXSTRT bit
  startRX();                                                      // enable receiver again
}

void startRX() {
  writeRegister8(DW1000_SYS_CTRL, 0x01, 0x01);                    // start listening for preamble by setting the RXENAB bit
}

void stopTRX() {
  writeRegister8(DW1000_SYS_CTRL, 0, 0x40);                       // disable tranceiver go back to idle mode
}

void loadLDE() {                                            // initialise LDE algorithm LDELOAD User Manual p22
  writeRegister16(DW1000_PMSC, 0, 0x0301);                        // set clock to XTAL so OTP is reliable
  writeRegister16(DW1000_OTP_IF, 0x06, 0x8000);                   // set LDELOAD bit in OTP
  delay(1);
  writeRegister16(DW1000_PMSC, 0, 0x0200);                        // recover to PLL clock
}

void resetRX() {    
  writeRegister8(DW1000_PMSC, 3, 0xE0);   // set RX reset
  writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear RX reset
}

void resetAll() {
  writeRegister8(DW1000_PMSC, 0, 0x01);   // set clock to XTAL
  writeRegister8(DW1000_PMSC, 3, 0x00);   // set All reset
  delay(1);                               // wait for PLL to lock
  writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear All reset
}

void setInterrupt(bool RX, bool TX) {
  writeRegister16(DW1000_SYS_MASK, 0, RX*0x4000 | TX*0x0080);  // RX good frame 0x4000, TX done 0x0080
}

void fakeISR() {
  uint64_t status = getStatus();
  if (status & 0x4000) {                                        // a frame was received
    callbackRX();
    writeRegister16(DW1000_SYS_STATUS, 0, 0x6F00);              // clearing of receiving status bits
  }
  if (status & 0x80) {                                          // sending complete
    callbackTX();
    writeRegister8(DW1000_SYS_STATUS, 0, 0xF8);                 // clearing of sending status bits
  }
};

uint16_t getFramelength() {
  uint16_t framelength = readRegister16(DW1000_RX_FINFO, 0);    // get framelength
  framelength = (framelength & 0x03FF) - 2;                     // take only the right bits and subtract the 2 CRC Bytes
  return framelength;
}

// SPI Interface --------------------------------------------------------------
uint8_t readRegister8(uint8_t reg, uint16_t subaddress) {
  uint8_t result;
  readRegister(reg, subaddress, &result, 1);
  return result;
}

uint16_t readRegister16(uint8_t reg, uint16_t subaddress) {
  uint16_t result;
  readRegister(reg, subaddress, (uint8_t*)&result, 2);
  return result;
}

uint64_t readRegister40(uint8_t reg, uint16_t subaddress) {
  uint64_t result;
  readRegister(reg, subaddress, (uint8_t*)&result, 5);
  // only 40-Bit
  result &= 0xFFFFFFFFFFLL;
  return result;
}

void writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer) {
  writeRegister(reg, subaddress, &buffer, 1);
}


void writeRegister16(uint8_t reg, uint16_t subaddress, uint16_t buffer) {
  writeRegister(reg, subaddress, (uint8_t*)&buffer, 2);
}


void readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
  setupTransaction(reg, subaddress, false);
  for(int i=0; i<length; i++) {
    // get data
    buffer[i] = SPI.transfer(0x00);
  }
  deselect();
}

void writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
  setupTransaction(reg, subaddress, true);
  for(int i=0; i<length; i++)                             // put data
    SPI.transfer(buffer[i]);
  deselect();
}

void setupTransaction(uint8_t reg, uint16_t subaddress, bool write) {
  // set read/write flag
  reg |=  (write * DW1000_WRITE_FLAG);
  select();
  if (subaddress > 0) {
    // there's a subadress, we need to set flag and send second header byte
    SPI.transfer(reg | DW1000_SUBADDRESS_FLAG);
    if (subaddress > 0x7F) {
      // sub address too long, we need to set flag and send third header byte
      // say which register address we want to access
      SPI.transfer((uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG);
      SPI.transfer((uint8_t)(subaddress >> 7));
    } else {
      SPI.transfer((uint8_t)subaddress);
    }
  } else {
    SPI.transfer(reg);
  }
}

// set CS low to start transmission
void select() {
  digitalWrite(chipSelectPin, LOW);
}

// set CS high to stop transmission
void deselect() { 
  digitalWrite(chipSelectPin, HIGH);
}

void print64(unsigned long long n) {
  uint8_t base = 10;
  char buf[8 * sizeof(long long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  *str = '\0';
  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);
  Serial.println(str);
}
