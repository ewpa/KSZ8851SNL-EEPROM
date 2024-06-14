/*
  93C46 EEPROM read and write via KSZ8851SNL over SPI.
  KSZ8851SNL-EEPROM.ino Copyright (C) 2024 Ewan Parker.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

  The author may be reached at https://www.ewan.cc/ for queries.

  References:
    https://www.microchip.com/content/dam/mchp/documents/OTH/ProductDocuments/
      DataSheets/KSZ8851SNL-Single-Port-Ethernet-Controller-with-SPI-Interface-
      DS00002381B.pdf
    https://ww1.microchip.com/downloads/en/DeviceDoc/
      Atmel-5193-SEEPROM-AT93C46D-Datasheet.pdf

  Wiring example for ESP32-S3 Dev Kit:
    3.3V and GND.
    10 SS, 11 MOSI, 12 SCK, 13 MISO.
    RST, IRQ and PME not needed.
*/

#include <SPI.h>
#include <string.h>

uint8_t readReg8(uint8_t reg)
{
  static SPISettings spiSet(40000000, MSBFIRST, SPI_MODE0);
  static const uint8_t opcode_bits = 0b00; // Internal I/O Register Read
  uint8_t byte_enable_bits = 0b1 << ((1-reg) & 0b11);
  uint8_t register_addr_bits_h = reg >> 6;
  uint8_t register_addr_bits_l = (reg >> 2) & 0b1111;

  uint8_t buff[3];
  buff[0] = (opcode_bits << 6) | (byte_enable_bits << 2) | register_addr_bits_h;
  buff[1] = register_addr_bits_l << 4;

  digitalWrite(SS, LOW);
  SPI.beginTransaction(spiSet);
  SPI.transfer(buff, 3);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();
  return buff[2];
}

void writeReg8(uint8_t reg, uint8_t val)
{
  static SPISettings spiSet(40000000, MSBFIRST, SPI_MODE0);
  static const uint8_t opcode_bits = 0b01; // Internal I/O Register Write
  uint8_t byte_enable_bits = 0b1 << ((1-reg) & 0b11);
  uint8_t register_addr_bits_h = reg >> 6;
  uint8_t register_addr_bits_l = (reg >> 2) & 0b1111;

  uint8_t buff[3];
  buff[0] = (opcode_bits << 6) | (byte_enable_bits << 2) | register_addr_bits_h;
  buff[1] = register_addr_bits_l << 4;
  buff[2] = val;

  pinMode(SS, OUTPUT);
  digitalWrite(SS, LOW);
  SPI.begin();
  SPI.beginTransaction(spiSet);
  SPI.transfer(buff, 3);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();
}

uint8_t digitalReadEE(uint8_t pin)
{
  // Emulate a digitalRead from the attached EEPROM.
  uint8_t val = LOW;
  uint8_t eecr = readReg8(0x23);
  switch (pin)
  {
    case MISO :
      if (eecr & 0b1000) val = HIGH;
      break;
    case MOSI :
      if (eecr & 0b0100) val = HIGH;
      break;
    case SCK :
      if (eecr & 0b0010) val = HIGH;
      break;
    case SS :
      if (eecr & 0b0001) val = HIGH;
      break;
    default :
      assert(false);
  }
  return val;
}

void digitalWriteEE(uint8_t pin, uint8_t val)
{
  // Emulate a digitalWrite to the attached EEPROM.
  uint8_t eecr = readReg8(0x23);
  eecr |= 0b110000; // EE pin read and write enable.
  uint8_t bits;
  switch (pin)
  {
    case MISO :
      assert (pin != MISO);
      break;
    case MOSI :
      bits = 0b0100;
      break;
    case SCK :
      bits = 0b0010;
      break;
    case SS :
      bits = 0b0001;
      break;
    default :
      assert(false);
  }
  if (val != LOW) eecr |= bits; else eecr &= (~bits);
  writeReg8(0x23, eecr);
}

void _clockOutBit(uint8_t b)
{
  digitalWriteEE(MOSI, b);
  digitalWriteEE(SCK, HIGH); delayMicroseconds(1);
  digitalWriteEE(SCK, LOW); delayMicroseconds(1);
}

uint8_t _clockInBit(void)
{
  uint8_t b = 0;
  digitalWriteEE(SCK, HIGH); delayMicroseconds(1);
  uint8_t eecr = readReg8(0x23);
  eecr &= ~0b100000; // EE pin write disable and read enable.
  writeReg8(0x23, eecr);
  if (digitalReadEE(MISO) == HIGH) b = 1;
  eecr |= 0b100000; // EE pin write enable.
  writeReg8(0x23, eecr);
  digitalWriteEE(SCK, LOW); delayMicroseconds(1);
  return b;
}

uint16_t readWordEE(uint8_t addr)
{
  digitalWriteEE(SS, HIGH);
  _clockOutBit(HIGH); // Start Bit
  _clockOutBit(HIGH); _clockOutBit(LOW); // READ Opcode 0b10
  uint8_t a;
  for (a = 0b100000; a; a >>= 1) _clockOutBit((addr & a) ? HIGH : LOW);
  digitalWriteEE(MOSI, HIGH); // Ensure DO remains pulled-up.
  uint16_t val = 0;
  for (a = 0; a < 16; a++)
  {
    val <<= 1;
    val |= _clockInBit();
  }
  digitalWriteEE(SS, LOW);
  return val;
}

void writeEnableEE()
{
  digitalWriteEE(SS, HIGH);
  _clockOutBit(HIGH); // Start Bit
  _clockOutBit(LOW); _clockOutBit(LOW); // EWEN Opcode 0b00
  uint8_t a;
  for (a = 0; a < 8; a++) _clockOutBit(HIGH);
  digitalWriteEE(SS, LOW);
}

void writeDisableEE()
{
  digitalWriteEE(SS, HIGH);
  _clockOutBit(HIGH); // Start Bit
  _clockOutBit(LOW); _clockOutBit(LOW); // EWDS Opcode 0b00
  uint8_t a;
  for (a = 0; a < 8; a++) _clockOutBit(LOW);
  digitalWriteEE(SS, LOW);
}

void writeWordEE(uint8_t addr, uint16_t val)
{
  digitalWriteEE(SS, HIGH);
  _clockOutBit(HIGH); // Start Bit
  _clockOutBit(LOW); _clockOutBit(HIGH); // WRITE Opcode 0b01
  uint8_t a;
  for (a = 0b100000; a; a >>= 1) _clockOutBit((addr & a) ? HIGH : LOW);
  int8_t b;
  for (b = 15; b >= 0; b--) _clockOutBit((val & (0b1 << b)) ? HIGH : LOW);
  digitalWriteEE(SS, LOW);
  // Wait for completion.
  delay(5);
}

void dumpMacReg(uint8_t mac[6])
{
  mac[0] = readReg8(0x14); // MARH
  mac[1] = readReg8(0x15); // MARH
  mac[2] = readReg8(0x12); // MARM
  mac[3] = readReg8(0x13); // MARM
  mac[4] = readReg8(0x10); // MARL
  mac[5] = readReg8(0x11); // MARL
}

void dumpMacEE(uint8_t mac[6])
{
  uint16_t w;
  w = readWordEE(0x01); mac[4] = w >> 8; mac[5] = w & 0xff;
  w = readWordEE(0x02); mac[2] = w >> 8; mac[3] = w & 0xff;
  w = readWordEE(0x03); mac[0] = w >> 8; mac[1] = w & 0xff;
}

void writeMacEE(uint8_t mac[6])
{
  uint16_t w;
  writeEnableEE();
  w = (mac[4] << 8) | mac[5]; writeWordEE(0x01, w);
  w = (mac[2] << 8) | mac[3]; writeWordEE(0x02, w);
  w = (mac[0] << 8) | mac[1]; writeWordEE(0x03, w);
  writeDisableEE();
}

bool inputMac(uint8_t mac[6])
{
  printf("\e[1mInput MAC:\e[0m      %02x-%02x-%02x-%02x-%02x-%02x\e[18D",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  fflush(stdout);
  unsigned short pos = 0;
  char c = '0'; bool msb = true; unsigned short nibble;
  while (pos < 12 && ((c >= '0' && c <= '9')
  || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')))
  {
    if (msb) printf("\e[C\e[1;4m%02x\e[0m\e[2D", mac[pos>>1]);
    fflush(stdout);
    while (!Serial.available()) ;
    c = Serial.read();
    if (c >= 'a' && c <= 'z') c -= 32; // Capitalize.
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))
    {
      nibble = c - '0'; if (nibble > 9) nibble -= 7;
      printf("\e[36m%x\e[0m", nibble);
      if (msb) mac[pos>>1] = (nibble<<4) | (mac[pos>>1]&0x0F);
      else mac[pos>>1] = (mac[pos>>1]&0xF0) | nibble;
    }
    else printf("\e[31mX\e[0m");
    pos++; msb = 1 - msb;
  }
  printf("\n");
  return (pos == 12);
}

void setup()
{
  Serial.begin(115200);
  // Remove noise on USB CDC.
  if (Serial.available()) while (Serial.available()) Serial.read();
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  SPI.begin();
  writeReg8(0x23, 0x00); // Release EEPROM pins so we are in a known state
  printf
   ("\n\n\e[7m\e#693C46 EEPROM read and write via KSZ8851SNL over SPI.\e[0m\n");
  printf("\e[1m\e#6Copyright (C) 2024 Ewan Parker.\n\e[0m");
  printf("\e[1m\e#6https://www.ewan.cc/\n\e[0m");
}

void loop()
{
  // Remove noise on USB CDC.
  if (Serial.available()) while (Serial.available()) Serial.read();
  printf("\n\e[1;4m\e#6Current State\n\e[0m");
  uint8_t ccr = readReg8(0x08);
  bool eeprom_present = ccr & 0x02; // EED_IO latched
  unsigned short color = eeprom_present ? /*green*/ 32 : /*red*/ 31;
  printf
    ("\e[1;%dmEEPROM present:\e[22m %s\e[0m\n", color, eeprom_present ? "Yes" : "No");
  uint8_t regMac[6], eepMac[6], newMac[6];
  dumpMacReg(regMac);
  printf("\e[1mCurrent MAC:\e[0m    %02x-%02x-%02x-%02x-%02x-%02x\n", regMac[0],
    regMac[1], regMac[2], regMac[3], regMac[4], regMac[5]);
  if (eeprom_present)
  {
    dumpMacEE(eepMac);
    printf("\e[1mEEPROM MAC:\e[0m     %02x-%02x-%02x-%02x-%02x-%02x\n",
      eepMac[0], eepMac[1], eepMac[2], eepMac[3], eepMac[4], eepMac[5]);

    printf("\n\e[1;4mMenu\e[0m\n");
    printf("\e[1mChoose 'w' to write EEPROM: \e[0m"); fflush(stdout);
    char c;
    while (!Serial.available()) ;
    c = Serial.read();
    if (c >= 'a' && c <= 'z') c -= 32; // Capitalize.
    printf("\e[36m%c\e[0m\n", c);

    if (c == 'W')
    {
      memcpy(newMac, eepMac, 6); bool macChanged = inputMac(newMac);
      if (macChanged && memcmp(eepMac, newMac, 6))
      {
        writeMacEE(newMac);
        printf("\n\e[32;7mEEPROM written\e[0m\n");
      }
      else printf("\n\e[33;1mEEPROM unchanged\e[0m\n");
    }

    writeReg8(0x23, 0x00); // Release EEPROM pins
  }
  else
  {
    // No EEPROM configured, do nothing.
    while (!Serial.available()) ;
    Serial.read();
  }
}
