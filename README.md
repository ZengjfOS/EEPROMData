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

* [eeprom_info.h](eeprom_info.h)
* [eeprom_info.c](eeprom_info.c)

## 数据结构及软件架构

* eeprom_info.h
    ```C
        #define EXPANSION_EEPROM_I2C_BUS        2
        #define EXPANSION_EEPROM_I2C_ADDRESS    0x50
        #define EEPROM_MAX_SIZE                 64
        #define EEPROM_SIZE                     64
        #define MMC_DEVICE_BUS                  1
     
        struct eeprom_data
        {
            /**
             * 0xFE和0xFF位址存放EEPROM Version ID, 长度为2 bytes, Low byte在0xFE, High Byte 在0xFF
             *   1. [0xfe]+256*[0xff]
             * [0xfe]+256*[0xff]     protocol version
             */
            u16 version;
            /**
             * Type 0x01 和 0x02：Ethernet MAC ID, BCD編碼格式, MSB開始
             *   1. 0x01 0x06 00:11:22:33:44:55   --> 1byte + 1byte + 6byte = 8byte
             *   2. 0x02 0x06 22:33:44:55:66:77   --> 1byte + 1byte + 6byte = 8byte
             */
            u8 mac1[8];
            u8 mac2[8];
            /**
             * Type 0x03: 軟件料號, BCD編碼格式, MSB開始
             *   1. 0x03 0x06 0001~9999(4 digits in tow byte) 00000001~99999999(8 digits in 4 byte)
             */
            u8 softid[8];
            /**
             * Type 0x04:
             * 背光控制极性:
             *     0 = 电压Level越高 或PWM High pulse宽度越宽越亮
             *     1 = 电压Level越高 或PWM High pulse宽度越宽越暗
             * 
             *   1. 0x10 0x04 0x00 0x10 0x00 00(pwm polarity 0,1,pwm min x/256,pwm frequency [0]+256*[x])
             */
            u8 backlight[6];
            /**
             * Type 0x10:
             *   1. 0x10 0x04 0x02 0x12 0x3c 0x01
             */
            u8 display[6];
            /**
             * Type 0x0a:
             *   1. 0x11 0x01 0x01
             */
            u8 logo[3];
        };

        struct eeprom_info
        {
            u16 accessable_size;                        // Max:64                accessable byte
            u16 max_size;                               // eeprom max byte
            byte i2c_bus;                               // eeprom work at i2c bus number
            byte device_addr;                           // eeprom device address
            byte address_length;                        // eeprom address register length
            byte mmc_bus;                               // load data from mmc bus number
            uchar content[EEPROM_SIZE];                 // content of eeprom from address 0 to eeprom_info.size
            struct eeprom_data data;                    // parse data of eeprom_info.content
            void (*init)(void);                         // init struct eeprom_info
            unsigned int (* read)(unsigned int addr, int alen, uint8_t *buf, int len);      // read data from eeprom
            unsigned int (* write)(unsigned int addr, int alen, uint8_t *buf, int len);     // write data to eeprom
            void (* parse_data)(void);                  // synthesis content to struct eeprom_data
            void (* synthesis_data)(void);              // parse struct eeprom_data to content
            int (*load_config)(unsigned int version);   // load data from eMMC or other device
        };
        struct eeprom_info AT24c02_eeprom;

        static unsigned int AT24C02_read_data(byte i2c_bus, unsigned int addr, int alen, uint8_t *buf, int len);
        static unsigned int AT24C02_write_data(byte i2c_bus, unsigned int addr, int alen, uint8_t *buf, int len);
        static void AT24C02_parse_data(void);
        static void AT24C02_synthesis_data(void);
        static int load_config_from_emmc(unsigned int version);
        static void AT24C02_init(void);
    ```
* eeprom_info.c  
    ```C
        static unsigned int AT24C02_read_data(byte i2c_bus, unsigned int addr, int alen, uint8_t *buf, int len)
        {
            // Todo   
        }

        static unsigned int AT24C02_write_data(byte i2c_bus, unsigned int addr, int alen, uint8_t *buf, int len)
        {
            // Todo   
        }
        
        static void AT24C02_parse_data(void)
        {
            // read eeprom protocol version
            AT24c02_eeprom.read( 0xfe, 1, buffer, 2 )

            // read eeprom 64(0x40) byte data
            AT24c02_eeprom.read(0x00, 1, AT24c02_eeprom.content, EEPROM_SIZE)

            // Todo   
        }
        
        static void AT24C02_synthesis_data(void)
        {
            // Todo   

            // write eeprom 64(0x40) byte data
            AT24c02_eeprom.write(0x00, 1, AT24c02_eeprom.content, EEPROM_SIZE)

            // write eeprom protocol version
            AT24c02_eeprom.write( 0xfe, 1, buffer, 2 )
        }
        
        static int load_config_from_emmc(unsigned int version)
        {
            // Todo   

            AT24c02_eeprom.synthesis_data(); 
        }

        static void AT24C02_init(void)
        {
            //Todo

            AT24c02_eeprom.read = AT24C02_read_data;
            AT24c02_eeprom.write = AT24C02_write_data;
            AT24c02_eeprom.parse_data = AT24C02_parse_data;
            AT24c02_eeprom.synthesis_data = AT24C02_synthesis_data;
            AT24c02_eeprom.load_config = load_config_from_emmc;

            AT24c02_eeprom.parse_data();

            //Todo
        }

    ```
* 初始化数据结构，会自动调用数据获取、解析
    ```C
        ...
        int board_init(void) 
        {
            ...
            AT24c02_eeprom.init = AT24C02_init;
            AT24c02_eeprom.init();
            ...
        }
        ...
    ```
* 加载eMMC配置数据
    ```C
        ...
        int board_mmc_init(bd_t *bis) 
        {
            ...
            AT24c02_eeprom.load_config();
            ...
        }
        ...
    ```
