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

## 代码提取
