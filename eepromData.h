#ifndef __EEPROM_H__
#define __EEPROM_H__

#define EXPANSION_EEPROM_I2C_BUS        2
#define EXPANSION_EEPROM_I2C_ADDRESS    0x50
#define EEPROM_MAX_SIZE                 64
#define EEPROM_SIZE                     64
#define MMC_DEVICE_INDEX                1

#include <linux/string.h>

struct eeprom_info
{
    uchar content[EEPROM_SIZE];
    u16 version;//[0xfe]+256*[0xff]
    u16 size;//Max:64
 };
static struct eeprom_info AT24c02_eeprom;

struct eeprom_data_analysis
{
    u16 version;//[0xfe]+256*[0xff]
    u8 mac1[8];//0x01 0x06 00:11:22:33:44:55
    u8 mac2[8];//0x02 0x06 22:33:44:55:66:77
    u8 softid[8];//0x03 0x06 0001~9999(4 digits in tow byte) 00000001~99999999(8 digits in 4 byte)
    u8 backlight[6];//0x10 0x04 0x00 0x10 0x00 00(pwm polarity 0,1,pwm min x/256,pwm frequency [0]+256*[x])
    u8 display[6];//0x10 0x04 0x02 0x12 0x3c 0x01
    u8 logo[3];//0x11 0x01 0x01
 };
static struct eeprom_data_analysis imx6_system_info;

static unsigned int eeprom_i2c_read(unsigned int addr, int alen, uint8_t *buffer, int len);
static unsigned int eeprom_i2c_write(unsigned int addr, int alen, uint8_t *buffer, int len);
static int Load_config_from_mmc(unsigned int version);
static void write_systeminfo_eeprom(void)

#endif // __EEPROM_H__
