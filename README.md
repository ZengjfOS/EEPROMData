# EEPROM Data Format

## 跟踪代码

* [Code Work Flow](eeprom_hacking.c)
* 说明：
  * 从I2C EEPROM中读取配置信息；
  * 检查SD卡，并获取需要的配置；
  * 设置网卡MAC地址；
  * 设置U-Boot显示屏数据；
  * 设置需要传递的Linux内核参数；
  * 回写到I2C EEPROM数据；

## 采用I2C控制器编写的相关代码

* [eepromData.h](eepromData.h)
* [eepromData.c](eepromData.c)

## 数据结构

```C
    struct eeprom_data
    {
        u16 version;        //[0xfe]+256*[0xff]
        u8 mac1[8];         //0x01 0x06 00:11:22:33:44:55
        u8 mac2[8];         //0x02 0x06 22:33:44:55:66:77
        u8 softid[8];       //0x03 0x06 0001~9999(4 digits in tow byte) 00000001~99999999(8 digits in 4 byte)
        u8 backlight[6];    //0x10 0x04 0x00 0x10 0x00 00(pwm polarity 0,1,pwm min x/256,pwm frequency [0]+256*[x])
        u8 display[6];      //0x10 0x04 0x02 0x12 0x3c 0x01
        u8 logo[3];         //0x11 0x01 0x01
    };

    struct eeprom_info
    {
        u16 version;                //[0xfe]+256*[0xff]
        u16 size;                   //Max:64
        uchar content[EEPROM_SIZE];
        struct eeprom_data eeprom;
        unsigned int (* read)(unsigned int addr, int alen, uint8_t *buf, int len);
        unsigned int (* write)(unsigned int addr, int alen, uint8_t *buf, int len);
        void (* composition_eeprom)(void);
        void (* decomposition_eeprom)(void);
        int (*load_config)(unsigned int version);
    };
```
