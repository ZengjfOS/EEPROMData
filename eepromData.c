#include "eepromData.h"

static unsigned int eeprom_i2c_read(unsigned int addr, int alen, uint8_t *buffer, int len)
{
    i2c_set_bus_num(EXPANSION_EEPROM_I2C_BUS);

    if (i2c_read(EXPANSION_EEPROM_I2C_ADDRESS, addr, alen, buffer, len)) {
        puts("I2C read failed in eeprom()\n");
    }

    udelay(10);
    return 0;
}

static unsigned int eeprom_i2c_write(unsigned int addr, int alen, uint8_t *buffer, int len)
{
    i2c_set_bus_num(EXPANSION_EEPROM_I2C_BUS);

    if (i2c_write(EXPANSION_EEPROM_I2C_ADDRESS, addr, alen, buffer, len)) {
        puts("I2C write failed in eeprom_i2c_write()\n");
    }
    udelay(11000);
    return 0;
}

static unsigned int get_eeprom_data(void)
{
    uchar buffer[EEPROM_MAX_SIZE]={0};

    if (eeprom_i2c_read( 0xfe, 1, buffer, 2)) {
        puts("I2C read failed in eeprom 0xf8()\n");
        AT24c02_eeprom.version=0x00;
        return 0;
    }

    // sprintf(tmp,"bAT24c02_eeprom.version(0x%04x)\n",AT24c02_eeprom.version);
    AT24c02_eeprom.version=0+buffer[0]+(buffer[1]<<8);
    if(AT24c02_eeprom.version<0x0 || AT24c02_eeprom.version>0xf000)
    {
        AT24c02_eeprom.version=0x00;
    }
    else
    {
        eeprom_i2c_read( 0x00, 1, AT24c02_eeprom.content, EEPROM_SIZE);
    }
    // sprintf(tmp,"aAT24c02_eeprom.version(0x%04x)\n",AT24c02_eeprom.version);
    // puts(tmp);
    return AT24c02_eeprom.version;
}

static unsigned int get_eeprom_analysis(void)
{
    int i=0;
    int datalength=0;
    
    memset(&imx6_system_info,0,sizeof(imx6_system_info));

    imx6_system_info.version=3;        
    i=0;
    datalength=0;
    while(i<EEPROM_SIZE)
    {
        datalength=AT24c02_eeprom.content[i+1]+2;
        switch(AT24c02_eeprom.content[i])
        {
            default:
            case 0x00:
                return 0;
            case 0x01:
                memcpy(imx6_system_info.mac1,AT24c02_eeprom.content+i,datalength);
                break;
            case 0x02:
                memcpy(imx6_system_info.mac2,AT24c02_eeprom.content+i,datalength);
                break;
            case 0x03:
                memcpy(imx6_system_info.softid,AT24c02_eeprom.content+i,datalength);
                break;
            case 0x04:
                memcpy(imx6_system_info.backlight,AT24c02_eeprom.content+i,datalength);
                break;
            case 0x10:
                memcpy(imx6_system_info.display,AT24c02_eeprom.content+i,datalength);
                break;
            case 0x11:
                memcpy(imx6_system_info.logo,AT24c02_eeprom.content+i,datalength);
                break;
        }
        AT24c02_eeprom.size=i+datalength;
        i=i+datalength;
    }
    return 0;
}

static uchar datatransfer(uchar h4b,uchar l4b)
{
    uchar dtr=0x0;
    if(h4b>='0' && h4b<='9')
        dtr=(h4b-'0')<<4;
    else if(h4b>='a' && h4b<='f')
        dtr=(h4b-'a'+10)<<4;
    else if(h4b>='A' && h4b<='F')
        dtr=(h4b-'A'+10)<<4;
    
    if(l4b>='0' && l4b<='9')
        dtr+=(l4b-'0');
    else if(l4b>='a' && l4b<='f')
        dtr+=(l4b-'a'+10);
    else if(l4b>='A' && l4b<='F')
        dtr+=(l4b-'A'+10);
    return dtr;
}

static int atoi(char *string)
{
    int length;
    int retval = 0;
    int i;
    int sign = 1;

    length = strlen(string);
    for (i = 0; i < length; i++) {
        if (0 == i && string[0] == '-') {
            sign = -1;
            continue;
        }
        if (string[i] > '9' || string[i] < '0') {
            break;
        }
        retval *= 10;
        retval += string[i] - '0';
    }
    retval *= sign;
    return retval;
}

static u32 atoilength(char *string,int len)
{
    int length;
    u64 retval = 0;
    int i;
    u64 sign = 1;

    length = strlen(string);
    if(len<length)length=len;
    for (i = 0; i < length; i++) {
        if (0 == i && string[0] == '-') {
            sign = -1;
            continue;
        }
        if (string[i] > '9' || string[i] < '0') {
            break;
        }
        retval *= 10;
        retval += string[i] - '0';
    }
    retval *= sign;
    return retval;
}

static int Load_config_from_mmc(unsigned int version)
{
    struct mmc *mmc;

    mmc = find_mmc_device(MMC_DEVICE_INDEX);
    if (mmc) 
    {
        mmc_init(mmc);
    }
    if (mmc) 
    {
        long size;
        //int i;
        char tmp[128];
        uchar tbuffer[512]={0};
        //uchar buffer[128]={0};
        char * ptr;
        block_dev_desc_t *dev_desc=NULL;
        unsigned int read_version=0;
        //unsigned int tmpversion=imx6_system_info.version;
        unsigned int prm_check=0;
                
        dev_desc=get_dev("mmc",MMC_DEVICE_INDEX);
        if (dev_desc!=NULL) 
        {
            if (fat_register_device((block_dev_desc_t *)dev_desc,1)==0) 
            {
                size = file_fat_read ("aplex.cfg", (void *) tbuffer, sizeof(tbuffer));
                if(size<=0)return -1;

                puts((char *)tbuffer);
                puts("\n");

                // get protocol version
                ptr = strstr((const char *)tbuffer, "Version=");
                if (ptr != NULL) 
                {
                    read_version=atoi(ptr+8);                       // 8 is "Version=" length
                    if(read_version<0 || read_version>0xF000)
                        read_version=version;
                }
                else
                {
                    read_version=version;
                }
                sprintf(tmp,"Version=0x%04x\n",read_version);
                puts(tmp);

                // get data from SD card, so show pass Logo
                show_pass_logo=1;

                if(read_version!=0x03)
                {
                    return -2;
                }

                // get mac address
                ptr = strstr((const char *)tbuffer, "MAC1=");
                if (ptr != NULL)
                {
                    //70:B3:D5:10:6F:56 
                    //70:B3:D5:10:6F:57
                    //00:11:22:33:44:55
                    ptr+=5;                                 // 5 is "MAC1=" length
                    memset(tmp,0x0,sizeof(tmp));
                    tmp[0]=datatransfer(ptr[0], ptr[1]);    // two ascii code to a byte
                    tmp[1]=datatransfer(ptr[3], ptr[4]);
                    tmp[2]=datatransfer(ptr[6], ptr[7]);
                    tmp[3]=datatransfer(ptr[9], ptr[10]);
                    tmp[4]=datatransfer(ptr[12],ptr[13]);
                    tmp[5]=datatransfer(ptr[15],ptr[16]);
                    if (is_valid_ether_addr((const u8 *)tmp))
                    {
                        imx6_system_info.mac1[0]=0x01;
                        imx6_system_info.mac1[1]=0x06;
                        memcpy(imx6_system_info.mac1+2,tmp,6);
                    }
                    //sprintf(tmp,"%02x:%02x:%02x:%02x:%02x:%02x",imx6_system_info.mac1[2],imx6_system_info.mac1[3],imx6_system_info.mac1[4],imx6_system_info.mac1[5],imx6_system_info.mac1[6],imx6_system_info.mac1[7]);
                    //puts("nMAC: ");
                    //puts(tmp);
                    //puts("\n");
                }

                // software system number
                ptr = strstr((const char *)tbuffer, "Software_part_number=");
                if (ptr != NULL) 
                {
                    u32 softidhigh=0;
                    u32 softidlow=0;
                    softidhigh=(u32)(atoilength(ptr+21,4));
                    softidlow=(u32)(atoilength(ptr+21+4,8));
                    sprintf(tmp,"Software_part_number=(%04u)(%08u)\n",softidhigh,softidlow);
                    puts(tmp);
                    if(softidhigh<=9999 && softidhigh>0 && softidlow<=99999999 && softidlow>0)
                    {
                        imx6_system_info.softid[0]=0x03;
                        imx6_system_info.softid[1]=0x06;
                        imx6_system_info.softid[2]=(softidhigh>>8)&0xff;
                        imx6_system_info.softid[3]=(softidhigh>>0)&0xff;
                        imx6_system_info.softid[4]=(softidlow>>24)&0xff;
                        imx6_system_info.softid[5]=(softidlow>>16)&0xff;
                        imx6_system_info.softid[6]=(softidlow>> 8)&0xff;
                        imx6_system_info.softid[7]=(softidlow>> 0)&0xff;
                    }
                }

                // for backlight pwm parameter
                prm_check=0;
                ptr = strstr((const char *)tbuffer, "Backlight_polarity=");
                if (ptr != NULL) 
                {
                    prm_check++;
                    imx6_system_info.backlight[2]=(uchar)(atoi(ptr+19)&0xff)==1?1:0;
                }
                ptr = strstr((const char *)tbuffer, "Backlight_min=");
                if (ptr != NULL) 
                {
                    prm_check++;
                    imx6_system_info.backlight[3]=(uchar)(atoi(ptr+14)&0xff);
                }

                // backlight frequency
                ptr = strstr((const char *)tbuffer, "Backlight_frequency=");
                if (ptr != NULL) 
                {
                    int read_backlight_frequency=atoi(ptr+20);
                    prm_check++;
                    imx6_system_info.backlight[4]=(read_backlight_frequency>>8)&0xff;
                    imx6_system_info.backlight[5]=(read_backlight_frequency)&0xff;
                }

                if(prm_check)
                {
                    int check_backlight_frequency=imx6_system_info.backlight[4]*256+imx6_system_info.backlight[5];
                    imx6_system_info.backlight[0]=0x04;
                    imx6_system_info.backlight[1]=0x04;
                    if(imx6_system_info.backlight[2]!=1 &&imx6_system_info.backlight[2]!=0)
                        imx6_system_info.backlight[2]=0;
                    if(imx6_system_info.backlight[3]<0 || imx6_system_info.backlight[3]>40)//default :0
                        imx6_system_info.backlight[3]=0;
                    if(check_backlight_frequency<100 || check_backlight_frequency>50000)
                    {
                        check_backlight_frequency=50000;
                        imx6_system_info.backlight[4]=(check_backlight_frequency>>8)&0xff;
                        imx6_system_info.backlight[5]=(check_backlight_frequency)&0xff;
                    }
                }

                // for display parameter
                prm_check=0;
                ptr = strstr((const char *)tbuffer, "Resolution_ID=");
                if (ptr != NULL) 
                {
                    prm_check++;
                    imx6_system_info.display[2]=(uchar)(atoi(ptr+14)&0xff);
                }

                // Color depth
                ptr = strstr((const char *)tbuffer, "Color_depth=");
                if (ptr != NULL) 
                {
                    prm_check++;
                    imx6_system_info.display[3]=(uchar)(atoi(ptr+12)&0xff);
                }

                // Frame rate
                ptr = strstr((const char *)tbuffer, "Frame_rate=");
                if (ptr != NULL) 
                {
                    prm_check++;
                    imx6_system_info.display[4]=(uchar)(atoi(ptr+11)&0xff);
                }

                // Interface 
                ptr = strstr((const char *)tbuffer, "Interface=");
                if (ptr != NULL) 
                {
                    prm_check++;
                    imx6_system_info.display[5]=(uchar)(atoi(ptr+10)&0xff);
                }
                if(prm_check)
                {
                    imx6_system_info.display[0]=0x10;
                    imx6_system_info.display[1]=0x04;
                    if(imx6_system_info.display[2]<RESOLUTION_640X480 || imx6_system_info.display[2]>RESOLUTION_1920X1080)
                        imx6_system_info.display[2]=RESOLUTION_800X480;
                    if(imx6_system_info.display[3]!=0x12 && imx6_system_info.display[3]!=0x18)
                        imx6_system_info.display[3]=0x12;
                    if(imx6_system_info.display[4]<0x28 || imx6_system_info.display[4]>0x64)
                        imx6_system_info.display[4]=0x3c;
                    //
                    if(imx6_system_info.display[5]<1 || imx6_system_info.display[5]>3)
                        imx6_system_info.display[5]=1;
                }

                // Show Logo
                ptr = strstr((const char *)tbuffer, "Logo=");
                if (ptr != NULL) 
                {
                    uchar read_boot_logo=imx6_system_info.logo[2];
                    read_boot_logo=(uchar)(atoi(ptr+5)&0xff);
                    if(read_boot_logo<1 || read_boot_logo>5)
                    {
                        read_boot_logo=1;
                    }
                    imx6_system_info.logo[0]=0x11;
                    imx6_system_info.logo[1]=0x01;
                    imx6_system_info.logo[2]=0xff&read_boot_logo;
                }

                // Halt Logo
                ptr = strstr((const char *)tbuffer, "Halt");
                if (ptr != NULL) 
                {
                    setenv("Halt","y");
                }

                write_systeminfo_eeprom();
            }
        }
    }
    return 0;
}

static void write_systeminfo_eeprom(void)
{
    char tmp[128];
    uchar eepromtmp[10];

    puts("nMAC: ");
    sprintf(tmp,"write_systeminfo_eeprom:%02x:%02x:%02x:%02x:%02x:%02x\n",imx6_system_info.mac1[2],imx6_system_info.mac1[3],imx6_system_info.mac1[4],imx6_system_info.mac1[5],imx6_system_info.mac1[6],imx6_system_info.mac1[7]);
    puts(tmp);

    int write_size_offset=0;
    AT24c02_eeprom.size=0;
    memset(AT24c02_eeprom.content,0,sizeof(AT24c02_eeprom.content));

    // generate MAC1
    if(imx6_system_info.mac1[0]!=0 && imx6_system_info.mac1[1]!=0)
    {
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,imx6_system_info.mac1,imx6_system_info.mac1[1]+2);
        AT24c02_eeprom.size+=(imx6_system_info.mac1[1]+2);
    }

    // generate MAC2
    if(imx6_system_info.mac2[0]!=0 && imx6_system_info.mac2[1]!=0)
    {
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,imx6_system_info.mac2,imx6_system_info.mac2[1]+2);
        AT24c02_eeprom.size+=(imx6_system_info.mac2[1]+2);
    }

    // software system number
    if(imx6_system_info.softid[0]!=0 && imx6_system_info.softid[1]!=0)
    {
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,imx6_system_info.softid,imx6_system_info.softid[1]+2);
        AT24c02_eeprom.size+=(imx6_system_info.softid[1]+2);
    }

    // backlight
    if(imx6_system_info.backlight[0]!=0 && imx6_system_info.backlight[1]!=0)
    {
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,imx6_system_info.backlight,imx6_system_info.backlight[1]+2);
        AT24c02_eeprom.size+=(imx6_system_info.backlight[1]+2);
    }
    if(imx6_system_info.display[0]!=0 && imx6_system_info.display[1]!=0)
    {
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,imx6_system_info.display,imx6_system_info.display[1]+2);
        AT24c02_eeprom.size+=(imx6_system_info.display[1]+2);
    }

    // show logo
    if(imx6_system_info.logo[0]!=0 && imx6_system_info.logo[1]!=0)
    {
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,imx6_system_info.logo,imx6_system_info.logo[1]+2);
        AT24c02_eeprom.size+=(imx6_system_info.logo[1]+2);
    }

    // write 8 byte a time, this was a block date
    while(write_size_offset<AT24c02_eeprom.size)
    {
        eeprom_i2c_write(write_size_offset, 1, AT24c02_eeprom.content+write_size_offset, 8);
        write_size_offset+=8;
    }

    // write version back to eeprom 
    AT24c02_eeprom.version=imx6_system_info.version;
    eepromtmp[0]=0xff & imx6_system_info.version;
    eepromtmp[1]=0xff & (imx6_system_info.version>>8);
    eeprom_i2c_write(0xfe, 1, eepromtmp, 2);

    sprintf(tmp,"AT24c02_eeprom.size:%d\n",AT24c02_eeprom.size);
    puts(tmp);
}
