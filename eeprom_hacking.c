void board_init_f(ulong dummy)
{
    /* setup AIPS and disable watchdog */
    arch_cpu_init();

    ccgr_init();
    gpr_init();

    /* iomux and setup of i2c */
    board_early_init_f();   ---------------------------------------+
                                                                   |
    /* setup GP timer */                                           |
    timer_init();                                                  |
                                                                   |
    /* UART clocks enabled and gd valid - init serial console */   |
    preloader_console_init();                                      |
                                                                   |
    /* DDR initialization */                                       |
    spl_dram_init();                                               |
                                                                   |
    /* Clear the BSS. */                                           |
    memset(__bss_start, 0, __bss_end - __bss_start);               |
                                                                   |
    /* load/boot image from boot device */                         |
    board_init_r(NULL, 0);             ---------------------+      |
}                                                           |      |
                                                            |      |
int board_early_init_f(void)           <--------------------*------+
{                                                           |
    setup_iomux_uart();                                     |
#if defined(CONFIG_VIDEO_IPUV3)                             |
    setup_display();                                        |
#endif                                                      |
                                                            |
    return 0;                                               |
}                                                           |
                                                            |
// common/board_r.c                                         |
void board_init_r(gd_t *new_gd, ulong dest_addr)   <--------+
{
#ifdef CONFIG_NEEDS_MANUAL_RELOC
    int i;
#endif

#ifdef CONFIG_AVR32
    mmu_init_r(dest_addr);
#endif

#if !defined(CONFIG_X86) && !defined(CONFIG_ARM) && !defined(CONFIG_ARM64)
    gd = new_gd;
#endif

#ifdef CONFIG_NEEDS_MANUAL_RELOC
    for (i = 0; i < ARRAY_SIZE(init_sequence_r); i++)
        init_sequence_r[i] += gd->reloc_off;
#endif

    if (initcall_run_list(init_sequence_r))        ------------+
        hang();                                                |
                                                               |
    /* NOTREACHED - run_main_loop() does not return */         |
    hang();                                                    |
}                                                              |
                                                               |
init_fnc_t init_sequence_r[] = {                 <-------------+
    initr_trace,
    initr_reloc,
    /* TODO: could x86/PPC have this also perhaps? */
#ifdef CONFIG_ARM
    initr_caches,
#endif
    initr_reloc_global_data,
#if defined(CONFIG_SYS_INIT_RAM_LOCK) && defined(CONFIG_E500)
    initr_unlock_ram_in_cache,
#endif
    initr_barrier,
    initr_malloc,
#ifdef CONFIG_SYS_NONCACHED_MEMORY
    initr_noncached,
#endif
    bootstage_relocate,
#ifdef CONFIG_DM
    initr_dm,
#endif
#ifdef CONFIG_ARM
    board_init,    /* Setup chipselects */              ------------------------+
#endif                                                                          |
    /*                                                                          |
     * TODO: printing of the clock inforamtion of the board is now              |
     * implemented as part of bdinfo command. Currently only support for        |
     * davinci SOC's is added. Remove this check once all the board             |
     * implement this.                                                          |
     */                                                                         |
#ifdef CONFIG_CLOCKS                                                            |
    set_cpu_clk_info, /* Setup clock information */                             |
#endif                                                                          |
    stdio_init_tables,                                                          |
    initr_serial,                                                               |
    initr_announce,                                                             |
    INIT_FUNC_WATCHDOG_RESET                                                    |
#ifdef CONFIG_NEEDS_MANUAL_RELOC                                                |
    initr_manual_reloc_cmdtable,                                                |
#endif                                                                          |
#if defined(CONFIG_PPC) || defined(CONFIG_M68K)                                 |
    initr_trap,                                                                 |
#endif                                                                          |
#ifdef CONFIG_ADDR_MAP                                                          |
    initr_addr_map,                                                             |
#endif                                                                          |
#if defined(CONFIG_BOARD_EARLY_INIT_R)                                          |
    board_early_init_r,                                                         |
#endif                                                                          |
    INIT_FUNC_WATCHDOG_RESET                                                    |
#ifdef CONFIG_LOGBUFFER                                                         |
    initr_logbuffer,                                                            |
#endif                                                                          |
#ifdef CONFIG_POST                                                              |
    initr_post_backlog,                                                         |
#endif                                                                          |
    INIT_FUNC_WATCHDOG_RESET                                                    |
#ifdef CONFIG_SYS_DELAYED_ICACHE                                                |
    initr_icache_enable,                                                        |
#endif                                                                          |
#if defined(CONFIG_PCI) && defined(CONFIG_SYS_EARLY_PCI_INIT)                   |
    /*                                                                          |
     * Do early PCI configuration _before_ the flash gets initialised,          |
     * because PCU ressources are crucial for flash access on some boards.      |
     */                                                                         |
    initr_pci,                                                                  |
#endif                                                                          |
#ifdef CONFIG_WINBOND_83C553                                                    |
    initr_w83c553f,                                                             |
#endif                                                                          |
#ifdef CONFIG_ARCH_EARLY_INIT_R                                                 |
    arch_early_init_r,                                                          |
#endif                                                                          |
    power_init_board,                                                           |
#ifndef CONFIG_SYS_NO_FLASH                                                     |
    initr_flash,                                                                |
#endif                                                                          |
    INIT_FUNC_WATCHDOG_RESET                                                    |
#if defined(CONFIG_PPC) || defined(CONFIG_M68K)                                 |
    /* initialize higher level parts of CPU like time base and timers */        |
    cpu_init_r,                                                                 |
#endif                                                                          |
#ifdef CONFIG_PPC                                                               |
    initr_spi,                                                                  |
#endif                                                                          |
#if defined(CONFIG_X86) && defined(CONFIG_SPI)                                  |
    init_func_spi,                                                              |
#endif                                                                          |
#ifdef CONFIG_CMD_NAND                                                          |
    initr_nand,                                                                 |
#endif                                                                          |
#ifdef CONFIG_CMD_ONENAND                                                       |
    initr_onenand,                                                              |
#endif                                                                          |
#ifdef CONFIG_GENERIC_MMC                                                       |
    initr_mmc,                           ---------------------------------------*-----------+
#endif                                                                          |           |
#ifdef CONFIG_HAS_DATAFLASH                                                     |           |
    initr_dataflash,                                                            |           |
#endif                                                                          |           |
    initr_env,                                                                  |           |
#ifdef CONFIG_SYS_BOOTPARAMS_LEN                                                |           |
    initr_malloc_bootparams,                                                    |           |
#endif                                                                          |           |
    INIT_FUNC_WATCHDOG_RESET                                                    |           |
    initr_secondary_cpu,                                                        |           |
#ifdef CONFIG_SC3                                                               |           |
    initr_sc3_read_eeprom,                                                      |           |
#endif                                                                          |           |
#if defined(CONFIG_ID_EEPROM) || defined(CONFIG_SYS_I2C_MAC_OFFSET)             |           |
    mac_read_from_eeprom,                                                       |           |
#endif                                                                          |           |
    INIT_FUNC_WATCHDOG_RESET                                                    |           |
#if defined(CONFIG_PCI) && !defined(CONFIG_SYS_EARLY_PCI_INIT)                  |           |
    /*                                                                          |           |
     * Do pci configuration                                                     |           |
     */                                                                         |           |
    initr_pci,                                                                  |           |
#endif                                                                          |           |
    stdio_add_devices,                  ----------------------------------------*-----------*-----+
    initr_jumptable,                                                            |           |     |
#ifdef CONFIG_API                                                               |           |     |
    initr_api,                                                                  |           |     |
#endif                                                                          |           |     |
    console_init_r,        /* fully init console as a device */                 |           |     |
#ifdef CONFIG_DISPLAY_BOARDINFO_LATE                                            |           |     |
    show_board_info,                                                            |           |     |
#endif                                                                          |           |     |
#ifdef CONFIG_ARCH_MISC_INIT                                                    |           |     |
    arch_misc_init,        /* miscellaneous arch-dependent init */              |           |     |
#endif                                                                          |           |     |
#ifdef CONFIG_MISC_INIT_R                                                       |           |     |
    misc_init_r,        /* miscellaneous platform-dependent init */             |           |     |
#endif                                                                          |           |     |
    INIT_FUNC_WATCHDOG_RESET                                                    |           |     |
#ifdef CONFIG_CMD_KGDB                                                          |           |     |
    initr_kgdb,                                                                 |           |     |
#endif                                                                          |           |     |
    interrupt_init,                                                             |           |     |
#if defined(CONFIG_ARM) || defined(CONFIG_AVR32)                                |           |     |
    initr_enable_interrupts,                                                    |           |     |
#endif                                                                          |           |     |
#if defined(CONFIG_X86) || defined(CONFIG_MICROBLAZE) || defined(CONFIG_AVR32) \|           |     |
    || defined(CONFIG_M68K)                                                     |           |     |
    timer_init,        /* initialize timer */                                   |           |     |
#endif                                                                          |           |     |
#if defined(CONFIG_STATUS_LED) && defined(STATUS_LED_BOOT)                      |           |     |
    initr_status_led,                                                           |           |     |
#endif                                                                          |           |     |
    /* PPC has a udelay(20) here dating from 2002. Why? */                      |           |     |
#ifdef CONFIG_CMD_NET                                                           |           |     |
    initr_ethaddr,                                                              |           |     |
#endif                                                                          |           |     |
#ifdef CONFIG_BOARD_LATE_INIT                                                   |           |     |
    board_late_init,       -----------------------------------------------------*-----------*-+   |
#endif                                                                          |           | |   |
#ifdef CONFIG_FSL_FASTBOOT                                                      |           | |   |
    initr_fastboot_setup,                                                       |           | |   |
#endif                                                                          |           | |   |
#ifdef CONFIG_CMD_SCSI                                                          |           | |   |
    INIT_FUNC_WATCHDOG_RESET                                                    |           | |   |
    initr_scsi,                                                                 |           | |   |
#endif                                                                          |           | |   |
#ifdef CONFIG_CMD_DOC                                                           |           | |   |
    INIT_FUNC_WATCHDOG_RESET                                                    |           | |   |
    initr_doc,                                                                  |           | |   |
#endif                                                                          |           | |   |
#ifdef CONFIG_BITBANGMII                                                        |           | |   |
    initr_bbmii,                                                                |           | |   |
#endif                                                                          |           | |   |
#ifdef CONFIG_CMD_NET                                                           |           | |   |
    INIT_FUNC_WATCHDOG_RESET                                                    |           | |   |
    initr_net,                      --------------------------------------------*-----------*-*-+ |
#endif                                                                          |           | | | |
#ifdef CONFIG_POST                                                              |           | | | |
    initr_post,                                                                 |           | | | |
#endif                                                                          |           | | | |
#if defined(CONFIG_CMD_PCMCIA) && !defined(CONFIG_CMD_IDE)                      |           | | | |
    initr_pcmcia,                                                               |           | | | |
#endif                                                                          |           | | | |
#if defined(CONFIG_CMD_IDE)                                                     |           | | | |
    initr_ide,                                                                  |           | | | |
#endif                                                                          |           | | | |
#ifdef CONFIG_LAST_STAGE_INIT                                                   |           | | | |
    INIT_FUNC_WATCHDOG_RESET                                                    |           | | | |
    /*                                                                          |           | | | |
     * Some parts can be only initialized if all others (like                   |           | | | |
     * Interrupts) are up and running (i.e. the PC-style ISA                    |           | | | |
     * keyboard).                                                               |           | | | |
     */                                                                         |           | | | |
    last_stage_init,                                                            |           | | | |
#endif                                                                          |           | | | |
#ifdef CONFIG_CMD_BEDBUG                                                        |           | | | |
    INIT_FUNC_WATCHDOG_RESET                                                    |           | | | |
    initr_bedbug,                                                               |           | | | |
#endif                                                                          |           | | | |
#if defined(CONFIG_PRAM) || defined(CONFIG_LOGBUFFER)                           |           | | | |
    initr_mem,                                                                  |           | | | |
#endif                                                                          |           | | | |
#ifdef CONFIG_PS2KBD                                                            |           | | | |
    initr_kbd,                                                                  |           | | | |
#endif                                                                          |           | | | |
#ifdef CONFIG_FSL_FASTBOOT                                                      |           | | | |
    initr_check_fastboot,                                                       |           | | | |
#endif                                                                          |           | | | |
    run_main_loop,                                                              |           | | | |
};                                                                              |           | | | |
                                                                                |           | | | |
int board_init(void)                            <-------------------------------+           | | | |
{                                                                                           | | | |
    ...                                                                                     | | | |
    setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1); ---+                          | | | |
    setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2); ---+  --+                     | | | |
    ...                        |                                 |    |                     | | | |
    get_eeprom_data();      ---*---------------------------------*----*----------+          | | | |
    get_eeprom_analysis();  ---*---------------------------------*----*-------------------+ | | | |
    ...                        |                                 |    |          |        | | | | |
}                              |                                 |    |          |        | | | | |
                               V                                 |    |          |        | | | | |
// include/configs/mx6sabre_common.h                             |    |          |        | | | | |
#define CONFIG_SYS_I2C_SPEED          100000                     |    |          |        | | | | |
                                                                 |    |          |        | | | | |
static struct i2c_pads_info i2c_pad_info1 = {       <------------+    |          |        | | | | |
    .scl = {                                                     |    |          |        | | | | |
        .i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | I2C_PAD,        |    |          |        | | | | |
        .gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | I2C_PAD,     |    |          |        | | | | |
        .gp = IMX_GPIO_NR(4, 12)                                 |    |          |        | | | | |
    },                                                           |    |          |        | | | | |
    .sda = {                                                     |    |          |        | | | | |
        .i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | I2C_PAD,        |    |          |        | | | | |
        .gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | I2C_PAD,     |    |          |        | | | | |
        .gp = IMX_GPIO_NR(4, 13)                                 |    |          |        | | | | |
    }                                                            |    |          |        | | | | |
};                                                               |    |          |        | | | | |
                                                                 |    |          |        | | | | |
static struct i2c_pads_info i2c_pad_info2 = {       <------------+    |          |        | | | | |
    .scl = {                                                          |          |        | | | | |
        .i2c_mode = MX6_PAD_GPIO_3__I2C3_SCL | I2C_PAD,               |          |        | | | | |
        .gpio_mode = MX6_PAD_GPIO_3__GPIO1_IO03 | I2C_PAD,            |          |        | | | | |
        .gp = IMX_GPIO_NR(1, 3)                                       |          |        | | | | |
    },                                                                |          |        | | | | |
    .sda = {                                                          |          |        | | | | |
        .i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | I2C_PAD,               |          |        | | | | |
        .gpio_mode =  MX6_PAD_GPIO_6__GPIO1_IO06 | I2C_PAD,           |          |        | | | | |
        .gp = IMX_GPIO_NR(1, 6)                                       |          |        | | | | |
    }                                                                 |          |        | | | | |
};                                                                    |          |        | | | | |
                                                                      |          |        | | | | |
/* i2c_index can be from 0 - 2 */                                     |          |        | | | | |
int setup_i2c(unsigned i2c_index, int speed, int slave_addr,    <-----+          |        | | | | |
          struct i2c_pads_info *p)                                               |        | | | | |
{                                                                                |        | | | | |
    char name[9];                                                                |        | | | | |
    int ret;                                                                     |        | | | | |
                                                                                 |        | | | | |
    if (i2c_index >= ARRAY_SIZE(i2c_bases))                                      |        | | | | |
        return -EINVAL;                                                          |        | | | | |
                                                                                 |        | | | | |
    snprintf(name, sizeof(name), "i2c_sda%01d", i2c_index);                      |        | | | | |
    ret = gpio_request(p->sda.gp, name);                                         |        | | | | |
    if (ret)                                                                     |        | | | | |
        return ret;                                                              |        | | | | |
                                                                                 |        | | | | |
    snprintf(name, sizeof(name), "i2c_scl%01d", i2c_index);                      |        | | | | |
    ret = gpio_request(p->scl.gp, name);                                         |        | | | | |
    if (ret)                                                                     |        | | | | |
        goto err_req;                                                            |        | | | | |
                                                                                 |        | | | | |
    /* Enable i2c clock */                                                       |        | | | | |
    ret = enable_i2c_clk(1, i2c_index);   ---------------------+                 |        | | | | |
    if (ret)                                                   |                 |        | | | | |
        goto err_clk;                                          |                 |        | | | | |
                                                               |                 |        | | | | |
    /* Make sure bus is idle */                                |                 |        | | | | |
    ret = force_idle_bus(p);              ---------------------*-+               |        | | | | |
    if (ret)                                                   | |               |        | | | | |
        goto err_idle;                                         | |               |        | | | | |
                                                               | |               |        | | | | |
    bus_i2c_init(i2c_bases[i2c_index], speed, slave_addr, ---+-*-*-+             |        | | | | |
            force_idle_bus, p);                              | | | |             |        | | | | |
                                                             | | | |             |        | | | | |
    return 0;                                                | | | |             |        | | | | |
                                                             | | | |             |        | | | | |
err_idle:                                                    | | | |             |        | | | | |
err_clk:                                                     | | | |             |        | | | | |
    gpio_free(p->scl.gp);                                    | | | |             |        | | | | |
err_req:                                                     | | | |             |        | | | | |
    gpio_free(p->sda.gp);                                    | | | |             |        | | | | |
                                                             | | | |             |        | | | | |
    return ret;                                              | | | |             |        | | | | |
}                                                            | | | |             |        | | | | |
                                                             | | | |             |        | | | | |
static void * const i2c_bases[] = {          <---------------+ | | |             |        | | | | |
    (void *)I2C1_BASE_ADDR,                                    | | |             |        | | | | |
    (void *)I2C2_BASE_ADDR,                                    | | |             |        | | | | |
#ifdef I2C3_BASE_ADDR                                          | | |             |        | | | | |
    (void *)I2C3_BASE_ADDR,                                    | | |             |        | | | | |
#endif                                                         | | |             |        | | | | |
#ifdef I2C4_BASE_ADDR                                          | | |             |        | | | | |
    (void *)I2C4_BASE_ADDR,                                    | | |             |        | | | | |
#endif                                                         | | |             |        | | | | |
};                                                             | | |             |        | | | | |
                                                               | | |             |        | | | | |
#ifdef CONFIG_SYS_I2C_MXC                                      | | |             |        | | | | |
/* i2c_num can be from 0, to 1 for i.MX51 and 2 for i.MX53 */  | | |             |        | | | | |
int enable_i2c_clk(unsigned char enable, unsigned i2c_num) <---+ | |             |        | | | | |
{                                                                | |             |        | | | | |
    u32 mask;                                                    | |             |        | | | | |
                                                                 | |             |        | | | | |
#if defined(CONFIG_MX51)                                         | |             |        | | | | |
    if (i2c_num > 1)                                             | |             |        | | | | |
#elif defined(CONFIG_MX53)                                       | |             |        | | | | |
    if (i2c_num > 2)                                             | |             |        | | | | |
#endif                                                           | |             |        | | | | |
        return -EINVAL;                                          | |             |        | | | | |
    mask = MXC_CCM_CCGR_CG_MASK <<                               | |             |        | | | | |
            (MXC_CCM_CCGR1_I2C1_OFFSET + (i2c_num << 1));        | |             |        | | | | |
    if (enable)                                                  | |             |        | | | | |
        setbits_le32(&mxc_ccm->CCGR1, mask);                     | |             |        | | | | |
    else                                                         | |             |        | | | | |
        clrbits_le32(&mxc_ccm->CCGR1, mask);                     | |             |        | | | | |
    return 0;                                                    | |             |        | | | | |
}                                                                | |             |        | | | | |
#endif                                                           | |             |        | | | | |
                                                                 | |             |        | | | | |
static int force_idle_bus(void *priv)           <----------------+ |             |        | | | | |
{                                                                  |             |        | | | | |
    int i;                                                         |             |        | | | | |
    int sda, scl;                                                  |             |        | | | | |
    ulong elapsed, start_time;                                     |             |        | | | | |
    struct i2c_pads_info *p = (struct i2c_pads_info *)priv;        |             |        | | | | |
    int ret = 0;                                                   |             |        | | | | |
                                                                   |             |        | | | | |
    gpio_direction_input(p->sda.gp);                               |             |        | | | | |
    gpio_direction_input(p->scl.gp);                               |             |        | | | | |
                                                                   |             |        | | | | |
    imx_iomux_v3_setup_pad(p->sda.gpio_mode);                      |             |        | | | | |
    imx_iomux_v3_setup_pad(p->scl.gpio_mode);                      |             |        | | | | |
                                                                   |             |        | | | | |
    sda = gpio_get_value(p->sda.gp);                               |             |        | | | | |
    scl = gpio_get_value(p->scl.gp);                               |             |        | | | | |
    if ((sda & scl) == 1)                                          |             |        | | | | |
        goto exit;        /* Bus is idle already */                |             |        | | | | |
                                                                   |             |        | | | | |
    printf("%s: sda=%d scl=%d sda.gp=0x%x scl.gp=0x%x\n", __func__,|             |        | | | | |
        sda, scl, p->sda.gp, p->scl.gp);                           |             |        | | | | |
    gpio_direction_output(p->scl.gp, 1);                           |             |        | | | | |
    udelay(1000);                                                  |             |        | | | | |
    /* Send high and low on the SCL line */                        |             |        | | | | |
    for (i = 0; i < 9; i++) {                                      |             |        | | | | |
        gpio_direction_output(p->scl.gp, 1);                       |             |        | | | | |
        udelay(50);                                                |             |        | | | | |
        gpio_direction_output(p->scl.gp, 0);                       |             |        | | | | |
        udelay(50);                                                |             |        | | | | |
    }                                                              |             |        | | | | |
                                                                   |             |        | | | | |
    /* Simulate the NACK */                                        |             |        | | | | |
    gpio_direction_output(p->sda.gp, 1);                           |             |        | | | | |
    udelay(50);                                                    |             |        | | | | |
    gpio_direction_output(p->scl.gp, 1);                           |             |        | | | | |
    udelay(50);                                                    |             |        | | | | |
    gpio_direction_output(p->scl.gp, 0);                           |             |        | | | | |
    udelay(50);                                                    |             |        | | | | |
                                                                   |             |        | | | | |
    /* Simulate the STOP signal */                                 |             |        | | | | |
    gpio_direction_output(p->sda.gp, 0);                           |             |        | | | | |
    udelay(50);                                                    |             |        | | | | |
    gpio_direction_output(p->scl.gp, 1);                           |             |        | | | | |
    udelay(50);                                                    |             |        | | | | |
    gpio_direction_output(p->sda.gp, 1);                           |             |        | | | | |
    udelay(50);                                                    |             |        | | | | |
                                                                   |             |        | | | | |
    /* Get the bus status */                                       |             |        | | | | |
    gpio_direction_input(p->sda.gp);                               |             |        | | | | |
    gpio_direction_input(p->scl.gp);                               |             |        | | | | |
                                                                   |             |        | | | | |
    start_time = get_timer(0);                                     |             |        | | | | |
    for (;;) {                                                     |             |        | | | | |
        sda = gpio_get_value(p->sda.gp);                           |             |        | | | | |
        scl = gpio_get_value(p->scl.gp);                           |             |        | | | | |
        if ((sda & scl) == 1)                                      |             |        | | | | |
            break;                                                 |             |        | | | | |
        WATCHDOG_RESET();                                          |             |        | | | | |
        elapsed = get_timer(start_time);                           |             |        | | | | |
        if (elapsed > (CONFIG_SYS_HZ / 5)) {    /* .2 seconds */   |             |        | | | | |
            ret = -EBUSY;                                          |             |        | | | | |
            printf("%s: failed to clear bus, sda=%d scl=%d\n",     |             |        | | | | |
                    __func__, sda, scl);                           |             |        | | | | |
            break;                                                 |             |        | | | | |
        }                                                          |             |        | | | | |
    }                                                              |             |        | | | | |
exit:                                                              |             |        | | | | |
    imx_iomux_v3_setup_pad(p->sda.i2c_mode);                       |             |        | | | | |
    imx_iomux_v3_setup_pad(p->scl.i2c_mode);                       |             |        | | | | |
    return ret;                                                    |             |        | | | | |
}                                                                  |             |        | | | | |
                                                                   |             |        | | | | |
void bus_i2c_init(void *base, int speed, int unused,      <--------+             |        | | | | |
        int (*idle_bus_fn)(void *p), void *idle_bus_data)                        |        | | | | |
{                                                                                |        | | | | |
    struct sram_data *srdata = (void *)gd->srdata; ---+                          |        | | | | |
    int i = 0;                                        |                          |        | | | | |
    struct i2c_parms *p = srdata->i2c_data;           |                          |        | | | | |
    if (!base)                                        |                          |        | | | | |
        return;                                       |                          |        | | | | |
                                                      |                          |        | | | | |
#ifdef CONFIG_MX6                                     |                          |        | | | | |
    if (mx6_i2c_fused((u32)base)) {                   |     -----------------+   |        | | | | |
        printf("I2C@0x%x is fused, disable it\n",     |                      |   |        | | | | |
            (u32)base);                               |                      |   |        | | | | |
        return;                                       |                      |   |        | | | | |
    }                                                 |                      |   |        | | | | |
#endif                                                |                      |   |        | | | | |
                                                      |                      |   |        | | | | |
    for (;;) {                                        |                      |   |        | | | | |
        if (!p->base || (p->base == base)) {          |                      |   |        | | | | |
            p->base = base;                           |                      |   |        | | | | |
            if (idle_bus_fn) {                        |                      |   |        | | | | |
                p->idle_bus_fn = idle_bus_fn;         |                      |   |        | | | | |
                p->idle_bus_data = idle_bus_data;     |                      |   |        | | | | |
            }                                         |                      |   |        | | | | |
            break;                                    |                      |   |        | | | | |
        }                                             |                      |   |        | | | | |
        p++;                                          |                      |   |        | | | | |
        i++;                                          |                      |   |        | | | | |
        if (i >= ARRAY_SIZE(srdata->i2c_data))        |                      |   |        | | | | |
            return;                                   |                      |   |        | | | | |
    }                                                 |                      |   |        | | | | |
    bus_i2c_set_bus_speed(base, speed);         ------*----------------------*-+ |        | | | | |
}                                                     |                      | | |        | | | | |
                                                      |                      | | |        | | | | |
#define gd    get_gd()         <----------------------+                      | | |        | | | | |
                                                      |                      | | |        | | | | |
static inline gd_t *get_gd(void)      <---------------+--------------------+ | | |        | | | | |
{                                                                          | | | |        | | | | |
    gd_t *gd_ptr;                                                          | | | |        | | | | |
                                                                           | | | |        | | | | |
#ifdef CONFIG_ARM64                                                        | | | |        | | | | |
    /*                                                                     | | | |        | | | | |
     * Make will already error that reserving x18 is not supported at the  | | | |        | | | | |
     * time of writing, clang: error: unknown argument: '-ffixed-x18'      | | | |        | | | | |
     */                                                                    | | | |        | | | | |
    __asm__ volatile("mov %0, x18\n" : "=r" (gd_ptr));                     | | | |        | | | | |
#else                                                                      | | | |        | | | | |
    __asm__ volatile("mov %0, r9\n" : "=r" (gd_ptr));                      | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
                                                                           | | | |        | | | | |
    return gd_ptr;                                                         | | | |        | | | | |
}                                                                          | | | |        | | | | |
                                                                           | | | |        | | | | |
typedef struct global_data {                                               | | | |        | | | | |
    bd_t *bd;                                                              | | | |        | | | | |
    unsigned long flags;                                                   | | | |        | | | | |
    unsigned int baudrate;                                                 | | | |        | | | | |
    unsigned long cpu_clk;    /* CPU clock in Hz!        */                | | | |        | | | | |
    unsigned long bus_clk;                                                 | | | |        | | | | |
    /* We cannot bracket this with CONFIG_PCI due to mpc5xxx */            | | | |        | | | | |
    unsigned long pci_clk;                                                 | | | |        | | | | |
    unsigned long mem_clk;                                                 | | | |        | | | | |
#if defined(CONFIG_LCD) || defined(CONFIG_VIDEO)                           | | | |        | | | | |
    unsigned long fb_base;    /* Base address of framebuffer mem */        | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
#if defined(CONFIG_POST) || defined(CONFIG_LOGBUFFER)                      | | | |        | | | | |
    unsigned long post_log_word;  /* Record POST activities */             | | | |        | | | | |
    unsigned long post_log_res; /* success of POST test */                 | | | |        | | | | |
    unsigned long post_init_f_time;  /* When post_init_f started */        | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
#ifdef CONFIG_BOARD_TYPES                                                  | | | |        | | | | |
    unsigned long board_type;                                              | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
    unsigned long have_console;    /* serial_init() was called */          | | | |        | | | | |
#ifdef CONFIG_PRE_CONSOLE_BUFFER                                           | | | |        | | | | |
    unsigned long precon_buf_idx;    /* Pre-Console buffer index */        | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
#ifdef CONFIG_MODEM_SUPPORT                                                | | | |        | | | | |
    unsigned long do_mdm_init;                                             | | | |        | | | | |
    unsigned long be_quiet;                                                | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
    unsigned long env_addr;    /* Address  of Environment struct */        | | | |        | | | | |
    unsigned long env_valid;    /* Checksum of Environment valid? */       | | | |        | | | | |
                                                                           | | | |        | | | | |
    unsigned long ram_top;    /* Top address of RAM used by U-Boot */      | | | |        | | | | |
                                                                           | | | |        | | | | |
    unsigned long relocaddr;    /* Start address of U-Boot in RAM */       | | | |        | | | | |
    phys_size_t ram_size;    /* RAM size */                                | | | |        | | | | |
    unsigned long mon_len;    /* monitor len */                            | | | |        | | | | |
    unsigned long irq_sp;        /* irq stack pointer */                   | | | |        | | | | |
    unsigned long start_addr_sp;    /* start_addr_stackpointer */          | | | |        | | | | |
    unsigned long reloc_off;                                               | | | |        | | | | |
    struct global_data *new_gd;    /* relocated global data */             | | | |        | | | | |
                                                                           | | | |        | | | | |
#ifdef CONFIG_DM                                                           | | | |        | | | | |
    struct udevice    *dm_root;    /* Root instance for Driver Model */    | | | |        | | | | |
    struct udevice    *dm_root_f;    /* Pre-relocation root instance */    | | | |        | | | | |
    struct list_head uclass_root;    /* Head of core tree */               | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
                                                                           | | | |        | | | | |
    const void *fdt_blob;    /* Our device tree, NULL if none */           | | | |        | | | | |
    void *new_fdt;        /* Relocated FDT */                              | | | |        | | | | |
    unsigned long fdt_size;    /* Space reserved for relocated FDT */      | | | |        | | | | |
    struct jt_funcs *jt;        /* jump table */                           | | | |        | | | | |
    char env_buf[32];    /* buffer for getenv() before reloc. */           | | | |        | | | | |
#ifdef CONFIG_TRACE                                                        | | | |        | | | | |
    void        *trace_buff;    /* The trace buffer */                     | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
#if defined(CONFIG_SYS_I2C)                                                | | | |        | | | | |
    int        cur_i2c_bus;    /* current used i2c bus */                  | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
#ifdef CONFIG_SYS_I2C_MXC                                                  | | | |        | | | | |
    void *srdata[10];                                                      | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
    unsigned long timebase_h;                                              | | | |        | | | | |
    unsigned long timebase_l;                                              | | | |        | | | | |
#ifdef CONFIG_SYS_MALLOC_F_LEN                                             | | | |        | | | | |
    unsigned long malloc_base;    /* base address of early malloc() */     | | | |        | | | | |
    unsigned long malloc_limit;    /* limit address */                     | | | |        | | | | |
    unsigned long malloc_ptr;    /* current address */                     | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
#ifdef CONFIG_PCI                                                          | | | |        | | | | |
    struct pci_controller *hose;    /* PCI hose for early use */           | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
#ifdef CONFIG_PCI_BOOTDELAY                                                | | | |        | | | | |
    int pcidelay_done;                                                     | | | |        | | | | |
#endif                                                                     | | | |        | | | | |
    struct udevice *cur_serial_dev;    /* current serial device */         | | | |        | | | | |
    struct arch_global_data arch;    /* architecture-specific data */      | | | |        | | | | |
} gd_t;                         <------------------------------------------+ | | |        | | | | |
#endif                                                                       | | |        | | | | |
                                                                             | | |        | | | | |
u32 mx6_i2c_fused(u32 base_addr)                 <---------------------------+ | |        | | | | |
{                                                                              | |        | | | | |
    switch (base_addr) {                                                       | |        | | | | |
    case I2C1_BASE_ADDR:                                                       | |        | | | | |
        return check_module_fused(MX6_MODULE_I2C1);                            | |        | | | | |
    case I2C2_BASE_ADDR:                                                       | |        | | | | |
        return check_module_fused(MX6_MODULE_I2C2);                            | |        | | | | |
    case I2C3_BASE_ADDR:                                                       | |        | | | | |
        return check_module_fused(MX6_MODULE_I2C3);                            | |        | | | | |
#ifdef I2C4_BASE_ADDR                                                          | |        | | | | |
    case I2C4_BASE_ADDR:                                                       | |        | | | | |
        return check_module_fused(MX6_MODULE_I2C4);                            | |        | | | | |
#endif              |                                                          | |        | | | | |
    }               |                                                          | |        | | | | |
                    |                                                          | |        | | | | |
    return 0;       |                                                          | |        | | | | |
}                   |                                                          | |        | | | | |
                    V                                                          | |        | | | | |
u32 check_module_fused(enum fuse_module_type module)                           | |        | | | | |
{                                                                              | |        | | | | |
    u32 i, reg;                                                                | |        | | | | |
    for (i = 0; i < ARRAY_SIZE(mx6_fuse_descs); i++) {                         | |        | | | | |
        if (mx6_fuse_descs[i].module == module) {                              | |        | | | | |
            reg = readl(OCOTP_BASE_ADDR + mx6_fuse_descs[i].fuse_word_offset); | |        | | | | |
            if (reg & (1 << mx6_fuse_descs[i].fuse_bit_offset))                | |        | | | | |
                return 1; /* disabled */                                       | |        | | | | |
            else                                                               | |        | | | | |
                return 0; /* enabled */                                        | |        | | | | |
        }                                                                      | |        | | | | |
    }                                                                          | |        | | | | |
                                                                               | |        | | | | |
    return  0; /* Not has a fuse, always enabled */                            | |        | | | | |
}                                                                              | |        | | | | |
                                                                               | |        | | | | |
/*                                                                             | |        | | | | |
 * Set I2C Bus speed                                                           | |        | | | | |
 */                                                                            | |        | | | | |
static int bus_i2c_set_bus_speed(void *base, int speed)         <--------------+ |        | | | | |
{                                                                                |        | | | | |
    struct mxc_i2c_regs *i2c_regs = (struct mxc_i2c_regs *)base;                 |        | | | | |
    u8 clk_idx = i2c_imx_get_clk(speed);                                         |        | | | | |
    u8 idx = i2c_clk_div[clk_idx][1];                                            |        | | | | |
                                                                                 |        | | | | |
    /* Store divider value */                                                    |        | | | | |
    writeb(idx, &i2c_regs->ifdr);                                                |        | | | | |
                                                                                 |        | | | | |
    /* Reset module */                                                           |        | | | | |
    writeb(I2CR_IDIS, &i2c_regs->i2cr);                                          |        | | | | |
    writeb(0, &i2c_regs->i2sr);                                                  |        | | | | |
    return 0;                                                                    |        | | | | |
}                                                                                |        | | | | |
                                                                                 |        | | | | |
static unsigned int get_eeprom_data(void)             <--------------------------+        | | | | |
{                                                                                         | | | | |
    uchar buffer[128]={0};                                                                | | | | |
//    char tmp[128]={0};                                                                  | | | | |
                                                                                          | | | | |
    if (eeprom_i2c_read( 0xfe, 1, buffer, 2)) {            ----------------------+        | | | | |
        puts("I2C read failed in eeprom 0xf8()\n");                              |        | | | | |
        AT24c02_eeprom.version=0x00;                                             |        | | | | |
        return 0;                                                                |        | | | | |
    }                                                                            |        | | | | |
                                                                                 |        | | | | |
//    sprintf(tmp,"bAT24c02_eeprom.version(0x%04x)\n",AT24c02_eeprom.version);   |        | | | | |
    AT24c02_eeprom.version=0+buffer[0]+(buffer[1]<<8);                           |        | | | | |
    if(AT24c02_eeprom.version<0x0 || AT24c02_eeprom.version>0xf000)              |        | | | | |
    {                                                                            |        | | | | |
        AT24c02_eeprom.version=0x00;                                             |        | | | | |
    }                                                                            |        | | | | |
    else                                                                         |        | | | | |
    {                                                                            |        | | | | |
        eeprom_i2c_read( 0x00, 1, AT24c02_eeprom.content, 0x40);                 |        | | | | |
    }                                                                            |        | | | | |
//    sprintf(tmp,"aAT24c02_eeprom.version(0x%04x)\n",AT24c02_eeprom.version);   |        | | | | |
//    puts(tmp);                                                                 |        | | | | |
    return AT24c02_eeprom.version;                                               |        | | | | |
}                                                                                |        | | | | |
                                                                                 V        | | | | |
static unsigned int eeprom_i2c_read(unsigned int addr, int alen, uint8_t *buffer, int len)| | | | |
{                                                                                         | | | | |
    i2c_set_bus_num(EXPANSION_EEPROM_I2C_BUS);                   ------------+            | | | | |
    /* return BEAGLE_NO_EEPROM if eeprom doesn't respond */                  |            | | | | |
//    if (i2c_probe(EXPANSION_EEPROM_I2C_ADDRESS) == 1) {                    |            | | | | |
//        puts("get_eeprom_version() error\n");                              |            | | | | |
//        return -1;                                                         |            | | | | |
//    }                                                                      |            | | | | |
                                                                             |            | | | | |
    if (i2c_read(EXPANSION_EEPROM_I2C_ADDRESS, addr, alen, buffer, len)) { --*-+          | | | | |
        puts("I2C read failed in eeprom()\n");                               | |          | | | | |
//        return -1;                                                         | |          | | | | |
    }                                                                        | |          | | | | |
                                                                             | |          | | | | |
    //i2c_set_bus_num(0);                                                    | |          | | | | |
    udelay(10);                                                              | |          | | | | |
    return 0;                                                                | |          | | | | |
}                                                                            | |          | | | | |
                                                                             | |          | | | | |
#define EXPANSION_EEPROM_I2C_BUS 2       <-----------------------------------+ |          | | | | |
                                                                             | |          | | | | |
int i2c_set_bus_num(unsigned int bus)    <------------------------------------ |          | | | | |
{                                                                              |          | | | | |
#if defined(CONFIG_I2C_MUX)                                                    |          | | | | |
    if (bus < CONFIG_SYS_MAX_I2C_BUS) {                                        |          | | | | |
        i2c_bus_num = bus;                                                     |          | | | | |
    } else {                                                                   |          | | | | |
        int    ret;                                                            |          | | | | |
                                                                               |          | | | | |
        ret = i2x_mux_select_mux(bus);                                         |          | | | | |
        if (ret)                                                               |          | | | | |
            return ret;                                                        |          | | | | |
        i2c_bus_num = 0;                                                       |          | | | | |
    }                                                                          |          | | | | |
    i2c_bus_num_mux = bus;                                                     |          | | | | |
#else                                                                          |          | | | | |
#ifdef CONFIG_SYS_I2C2_OFFSET                                                  |          | | | | |
    if (bus > 1) {                                                             |          | | | | |
#else                                                                          |          | | | | |
    if (bus > 0) {                                                             |          | | | | |
#endif                                                                         |          | | | | |
        return -1;                                                             |          | | | | |
    }                                                                          |          | | | | |
                                                                               |          | | | | |
    i2c_bus_num = bus;                                                         |          | | | | |
#endif                                                                         |          | | | | |
    return 0;                                                                  |          | | | | |
}                                                                              |          | | | | |
                                                                               |          | | | | |
#define EXPANSION_EEPROM_I2C_ADDRESS 0x50                          <-----------*          | | | | |
                                                                               |          | | | | |
int                                                                            |          | | | | |
i2c_read(u8 dev, uint addr, int alen, u8 *data, int length)        <-----------+          | | | | |
{                                                                                         | | | | |
    int i = -1; /* signal error */                                                        | | | | |
    u8 *a = (u8*)&addr;                                                                   | | | | |
                                                                                          | | | | |
    if (i2c_wait4bus() >= 0                                                               | | | | |
        && i2c_write_addr(dev, I2C_WRITE_BIT, 0) != 0                                     | | | | |
        && __i2c_write(&a[4 - alen], alen) == alen)                                       | | | | |
        i = 0; /* No error so far */                                                      | | | | |
                                                                                          | | | | |
    if (length                                                                            | | | | |
        && i2c_write_addr(dev, I2C_READ_BIT, 1) != 0)                                     | | | | |
        i = __i2c_read(data, length);                                                     | | | | |
                                                                                          | | | | |
    writeb(I2C_CR_MEN, &i2c_dev[i2c_bus_num]->cr);                                        | | | | |
                                                                                          | | | | |
    if (i2c_wait4bus()) /* Wait until STOP */                                             | | | | |
        debug("i2c_read: wait4bus timed out\n");                                          | | | | |
                                                                                          | | | | |
    if (i == length)                                                                      | | | | |
        return 0;                                                                         | | | | |
                                                                                          | | | | |
    return -1;                                                                            | | | | |
}                                                                                         | | | | |
                                                                                          | | | | |
static unsigned int get_eeprom_analysis(void)       <-------------------------------------+ | | | |
{                                                                                           | | | |
    int i=0;                                                                                | | | |
    int datalength=0;                                                                       | | | |
                                                                                            | | | |
    memset(&imx6_system_info,0,sizeof(imx6_system_info));       -----------------------+    | | | |
    imx6_system_info.version=3;                                                        |    | | | |
    i=0;                                                                               |    | | | |
    datalength=0;                                                                      |    | | | |
    while(i<0x40)                                                                      |    | | | |
    {                                                                                  |    | | | |
        datalength=AT24c02_eeprom.content[i+1]+2;                                      |    | | | |
        switch(AT24c02_eeprom.content[i])                                              |    | | | |
        {                                                                              |    | | | |
            default:                                                                   |    | | | |
            case 0x00:                                                                 |    | | | |
                return 0;                                                              |    | | | |
            case 0x01:                                                                 |    | | | |
                memcpy(imx6_system_info.mac1,AT24c02_eeprom.content+i,datalength);     |    | | | |
                break;                                                                 |    | | | |
            case 0x02:                                                                 |    | | | |
                memcpy(imx6_system_info.mac2,AT24c02_eeprom.content+i,datalength);     |    | | | |
                break;                                                                 |    | | | |
            case 0x03:                                                                 |    | | | |
                memcpy(imx6_system_info.softid,AT24c02_eeprom.content+i,datalength);   |    | | | |
                break;                                                                 |    | | | |
            case 0x04:                                                                 |    | | | |
                memcpy(imx6_system_info.backlight,AT24c02_eeprom.content+i,datalength);|    | | | |
                break;                                                                 |    | | | |
            case 0x10:                                                                 |    | | | |
                memcpy(imx6_system_info.display,AT24c02_eeprom.content+i,datalength);  |    | | | |
                break;                                                                 |    | | | |
            case 0x11:                                                                 |    | | | |
                memcpy(imx6_system_info.logo,AT24c02_eeprom.content+i,datalength);     |    | | | |
                break;                                                                 |    | | | |
        }                                                                              |    | | | |
        AT24c02_eeprom.size=i+datalength;                                              |    | | | |
        i=i+datalength;                                                                |    | | | |
    }                                                                                  |    | | | |
    return 0;                                                                          |    | | | |
}                                                                                      |    | | | |
                                                                                       |    | | | |
struct eeprom_data_analysis                                       <--------------------+    | | | |
{                                                                                      |    | | | |
    u16 version;//[0xfe]+256*[0xff]                                                    |    | | | |
    u8 mac1[8];//0x01 0x06 00:11:22:33:44:55                                           |    | | | |
    u8 mac2[8];//0x02 0x06 22:33:44:55:66:77                                           |    | | | |
    // 0x03 0x06 0001~9999(4 digits in tow byte)                                       |    | | | |
    // 00000001~99999999(8 digits in 4 byte)                                           |    | | | |
    u8 softid[8];                                                                      |    | | | |
    // 0x10 0x04 0x00 0x10 0x00 00                                                     |    | | | |
    // (pwm polarity 0,1,pwm min x/256,pwm frequency [0]+256*[x])                      |    | | | |
    u8 backlight[6];                                                                   |    | | | |
    u8 display[6];//0x10 0x04 0x02 0x12 0x3c 0x01                                      |    | | | |
    u8 logo[3];//0x11 0x01 0x01                                                        |    | | | |
 };                                                                                    |    | | | |
static struct eeprom_data_analysis imx6_system_info;              <--------------------+    | | | |
                                                                                            | | | |
#ifdef CONFIG_GENERIC_MMC                                                                   | | | |
static int initr_mmc(void)                                        <-------------------------+ | | |
{                                                                                             | | |
    puts("MMC:   ");                                                                          | | |
    mmc_initialize(gd->bd);   -------+                                                        | | |
    return 0;                        |                                                        | | |
}                                    |                                                        | | |
#endif                               |                                                        | | |
                                     |                                                        | | |
int mmc_initialize(bd_t *bis) <------+                                                        | | |
{                                                                                             | | |
    INIT_LIST_HEAD (&mmc_devices);                                                            | | |
    cur_dev_num = 0;                                                                          | | |
                                                                                              | | |
    if (board_mmc_init(bis) < 0)  -----+                                                      | | |
        cpu_mmc_init(bis);             |                                                      | | |
                                       |                                                      | | |
#ifndef CONFIG_SPL_BUILD               |                                                      | | |
    print_mmc_devices(',');            |                                                      | | |
#endif                                 |                                                      | | |
                                       |                                                      | | |
    do_preinit();                      |                                                      | | |
    return 0;                          |                                                      | | |
}                                      |                                                      | | |
                                       |                                                      | | |
int board_mmc_init(bd_t *bis)     <----+                                                      | | |
{                                                                                             | | |
#ifndef CONFIG_SPL_BUILD                                                                      | | |
    int ret;                                                                                  | | |
    int i;                                                                                    | | |
                                                                                              | | |
    /*                                                                                        | | |
     * According to the board_mmc_init() the following map is done:                           | | |
     * (U-boot device node)    (Physical Port)                                                | | |
     * mmc0                    SD2                                                            | | |
     * mmc1                    SD3                                                            | | |
     * mmc2                    eMMC                                                           | | |
     */                                                                                       | | |
    //printf("%s:%s:%i: here i am\n", __FILE__, __func__, __LINE__);                          | | |
    for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {                                          | | |
        switch (i) {                                                                          | | |
        case 0:                                                                               | | |
            imx_iomux_v3_setup_multiple_pads(                                                 | | |
                usdhc2_pads, ARRAY_SIZE(usdhc2_pads));                                        | | |
#ifndef CONFIG_SBC7112                                                                        | | |
            gpio_direction_input(USDHC2_CD_GPIO);                                             | | |
#endif                                                                                        | | |
            usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);                            | | |
            break;                                                                            | | |
        case 1:                                                                               | | |
            imx_iomux_v3_setup_multiple_pads(                                                 | | |
                usdhc3_pads, ARRAY_SIZE(usdhc3_pads));                                        | | |
            gpio_direction_input(USDHC3_CD_GPIO);                                             | | |
            usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);                            | | |
            break;                                                                            | | |
        case 2:                                                                               | | |
            imx_iomux_v3_setup_multiple_pads(                                                 | | |
                usdhc4_pads, ARRAY_SIZE(usdhc4_pads));                                        | | |
            usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);                            | | |
            break;                                                                            | | |
        default:                                                                              | | |
            printf("Warning: you configured more USDHC controllers"                           | | |
                   "(%d) then supported by the board (%d)\n",                                 | | |
                   i + 1, CONFIG_SYS_FSL_USDHC_NUM);                                          | | |
            return -EINVAL;                                                                   | | |
        }                                                                                     | | |
                                                                                              | | |
        ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);                                       | | |
        if (ret)                                                                              | | |
            return ret;                                                                       | | |
    }                                                                                         | | |
                                                                                              | | |
    Load_config_from_mmc((unsigned int )imx6_system_info.version); -----+                     | | |
    //set_panel_env();                                                  |                     | | |
    return 0;                                                           |                     | | |
#else                                                                   |                     | | |
    struct src *psrc = (struct src *)SRC_BASE_ADDR;                     |                     | | |
    unsigned reg = readl(&psrc->sbmr1) >> 11;                           |                     | | |
    /*                                                                  |                     | | |
     * Upon reading BOOT_CFG register the following map is done:        |                     | | |
     * Bit 11 and 12 of BOOT_CFG register can determine the current     |                     | | |
     * mmc port                                                         |                     | | |
     * 0x1                  SD1                                         |                     | | |
     * 0x2                  SD2                                         |                     | | |
     * 0x3                  SD4                                         |                     | | |
     */                                                                 |                     | | |
                                                                        |                     | | |
    switch (reg & 0x3) {                                                |                     | | |
    case 0x1:                                                           |                     | | |
        imx_iomux_v3_setup_multiple_pads(                               |                     | | |
            usdhc2_pads, ARRAY_SIZE(usdhc2_pads));                      |                     | | |
        usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;                     |                     | | |
        usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);          |                     | | |
        gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;                      |                     | | |
        break;                                                          |                     | | |
    case 0x2:                                                           |                     | | |
        imx_iomux_v3_setup_multiple_pads(                               |                     | | |
            usdhc3_pads, ARRAY_SIZE(usdhc3_pads));                      |                     | | |
        usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;                     |                     | | |
        usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);          |                     | | |
        gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;                      |                     | | |
        break;                                                          |                     | | |
    case 0x3:                                                           |                     | | |
        imx_iomux_v3_setup_multiple_pads(                               |                     | | |
            usdhc4_pads, ARRAY_SIZE(usdhc4_pads));                      |                     | | |
        usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;                     |                     | | |
        usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);          |                     | | |
        gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;                      |                     | | |
        break;                                                          |                     | | |
    }                                                                   |                     | | |
                                                                        |                     | | |
    return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);                    |                     | | |
#endif                                                                  |                     | | |
}                                                                       |                     | | |
#endif                                                                  |                     | | |
                                                                        |                     | | |
int Load_config_from_mmc(unsigned int version)          <---------------+                     | | |
{                                                                                             | | |
    struct mmc *mmc;                                                                          | | |
                                                                                              | | |
    mmc = find_mmc_device(1);                                                                 | | |
    if (mmc)                                                                                  | | |
    {                                                                                         | | |
        mmc_init(mmc);                                                                        | | |
    }                                                                                         | | |
    if (mmc)                                                                                  | | |
    {                                                                                         | | |
        long size;                                                                            | | |
        //int i;                                                                              | | |
        char tmp[128];                                                                        | | |
        uchar tbuffer[512]={0};                                                               | | |
        //uchar buffer[128]={0};                                                              | | |
        char * ptr;                                                                           | | |
        block_dev_desc_t *dev_desc=NULL;                                                      | | |
        unsigned int read_version=0;                                                          | | |
        //unsigned int tmpversion=imx6_system_info.version;                                   | | |
        unsigned int prm_check=0;                                                             | | |
                                                                                              | | |
        dev_desc=get_dev("mmc",1);                                                            | | |
        if (dev_desc!=NULL)                                                                   | | |
        {                                                                                     | | |
            if (fat_register_device((block_dev_desc_t *)dev_desc,1)==0)                       | | |
            {                                                                                 | | |
                size = file_fat_read ("aplex.cfg", (void *) tbuffer, sizeof(tbuffer));        | | |
                if(size<=0)return -1;                                                         | | |
                puts((char *)tbuffer);                                                        | | |
                puts("\n");                                                                   | | |
                ptr = strstr((const char *)tbuffer, "Version=");                              | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    read_version=atoi(ptr+8);                                                 | | |
                    if(read_version<0 || read_version>0xF000)                                 | | |
                        read_version=version;                                                 | | |
                }                                                                             | | |
                else                                                                          | | |
                {                                                                             | | |
                    read_version=version;                                                     | | |
                }                                                                             | | |
                show_pass_logo=1;                                                             | | |
                sprintf(tmp,"Version=0x%04x\n",read_version);                                 | | |
                puts(tmp);                                                                    | | |
                if(read_version!=0x03)                                                        | | |
                {                                                                             | | |
                    return -2;                                                                | | |
                }                                                                             | | |
                ptr = strstr((const char *)tbuffer, "MAC1=");                                 | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    //70:B3:D5:10:6F:56                                                       | | |
                    //70:B3:D5:10:6F:57                                                       | | |
                    //00:11:22:33:44:55                                                       | | |
                    ptr+=5;                                                                   | | |
                    memset(tmp,0x0,sizeof(tmp));                                              | | |
                    tmp[0]=datatransfer(ptr[0], ptr[1]);                                      | | |
                    tmp[1]=datatransfer(ptr[3], ptr[4]);                                      | | |
                    tmp[2]=datatransfer(ptr[6], ptr[7]);                                      | | |
                    tmp[3]=datatransfer(ptr[9], ptr[10]);                                     | | |
                    tmp[4]=datatransfer(ptr[12],ptr[13]);                                     | | |
                    tmp[5]=datatransfer(ptr[15],ptr[16]);                                     | | |
                    if (is_valid_ether_addr((const u8 *)tmp))                                 | | |
                    {                                                                         | | |
                        imx6_system_info.mac1[0]=0x01;                                        | | |
                        imx6_system_info.mac1[1]=0x06;                                        | | |
                        memcpy(imx6_system_info.mac1+2,tmp,6);                                | | |
                    }                                                                         | | |
                }                                                                             | | |
                ptr = strstr((const char *)tbuffer, "Software_part_number=");                 | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    u32 softidhigh=0;                                                         | | |
                    u32 softidlow=0;                                                          | | |
                    softidhigh=(u32)(atoilength(ptr+21,4));                                   | | |
                    softidlow=(u32)(atoilength(ptr+21+4,8));                                  | | |
                    sprintf(tmp,"Software_part_number=(%04u)(%08u)\n",softidhigh,softidlow);  | | |
                    puts(tmp);                                                                | | |
                    if(softidhigh<=9999 && softidhigh>0 && softidlow<=99999999 && softidlow>0)| | |
                    {                                                                         | | |
                        imx6_system_info.softid[0]=0x03;                                      | | |
                        imx6_system_info.softid[1]=0x06;                                      | | |
                        imx6_system_info.softid[2]=(softidhigh>>8)&0xff;                      | | |
                        imx6_system_info.softid[3]=(softidhigh>>0)&0xff;                      | | |
                        imx6_system_info.softid[4]=(softidlow>>24)&0xff;                      | | |
                        imx6_system_info.softid[5]=(softidlow>>16)&0xff;                      | | |
                        imx6_system_info.softid[6]=(softidlow>> 8)&0xff;                      | | |
                        imx6_system_info.softid[7]=(softidlow>> 0)&0xff;                      | | |
                    }                                                                         | | |
                }                                                                             | | |
                                                                                              | | |
                //for backlight pwm parameter                                                 | | |
                prm_check=0;                                                                  | | |
                ptr = strstr((const char *)tbuffer, "Backlight_polarity=");                   | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    prm_check++;                                                              | | |
                    imx6_system_info.backlight[2]=(uchar)(atoi(ptr+19)&0xff)==1?1:0;          | | |
                }                                                                             | | |
                ptr = strstr((const char *)tbuffer, "Backlight_min=");                        | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    prm_check++;                                                              | | |
                    imx6_system_info.backlight[3]=(uchar)(atoi(ptr+14)&0xff);                 | | |
                }                                                                             | | |
                ptr = strstr((const char *)tbuffer, "Backlight_frequency=");                  | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    int read_backlight_frequency=atoi(ptr+20);                                | | |
                    prm_check++;                                                              | | |
                    imx6_system_info.backlight[4]=(read_backlight_frequency>>8)&0xff;         | | |
                    imx6_system_info.backlight[5]=(read_backlight_frequency)&0xff;            | | |
                }                                                                             | | |
                if(prm_check)                                                                 | | |
                {                                                                             | | |
                    int check_backlight_frequency=                                            | | |
                        imx6_system_info.backlight[4]*256+imx6_system_info.backlight[5];      | | |
                    imx6_system_info.backlight[0]=0x04;                                       | | |
                    imx6_system_info.backlight[1]=0x04;                                       | | |
                    if(imx6_system_info.backlight[2]!=1 &&imx6_system_info.backlight[2]!=0)   | | |
                        imx6_system_info.backlight[2]=0;                                      | | |
                    if(imx6_system_info.backlight[3]<0 ||                                     | | |
                            imx6_system_info.backlight[3]>40)//default :0                     | | |
                        imx6_system_info.backlight[3]=0;                                      | | |
                    if(check_backlight_frequency<100 || check_backlight_frequency>50000)      | | |
                    {                                                                         | | |
                        check_backlight_frequency=50000;                                      | | |
                        imx6_system_info.backlight[4]=(check_backlight_frequency>>8)&0xff;    | | |
                        imx6_system_info.backlight[5]=(check_backlight_frequency)&0xff;       | | |
                    }                                                                         | | |
                }                                                                             | | |
                                                                                              | | |
                //for display parameter                                                       | | |
                prm_check=0;                                                                  | | |
                ptr = strstr((const char *)tbuffer, "Resolution_ID=");                        | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    prm_check++;                                                              | | |
                    imx6_system_info.display[2]=(uchar)(atoi(ptr+14)&0xff);                   | | |
                }                                                                             | | |
                ptr = strstr((const char *)tbuffer, "Color_depth=");                          | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    prm_check++;                                                              | | |
                    imx6_system_info.display[3]=(uchar)(atoi(ptr+12)&0xff);                   | | |
                }                                                                             | | |
                ptr = strstr((const char *)tbuffer, "Frame_rate=");                           | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    prm_check++;                                                              | | |
                    imx6_system_info.display[4]=(uchar)(atoi(ptr+11)&0xff);                   | | |
                }                                                                             | | |
                ptr = strstr((const char *)tbuffer, "Interface=");                            | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    prm_check++;                                                              | | |
                    imx6_system_info.display[5]=(uchar)(atoi(ptr+10)&0xff);                   | | |
                }                                                                             | | |
                if(prm_check)                                                                 | | |
                {                                                                             | | |
                    imx6_system_info.display[0]=0x10;                                         | | |
                    imx6_system_info.display[1]=0x04;                                         | | |
                    if(imx6_system_info.display[2]<RESOLUTION_640X480 ||                      | | |
                            imx6_system_info.display[2]>RESOLUTION_1920X1080)                 | | |
                        imx6_system_info.display[2]=RESOLUTION_800X480;                       | | |
                    if(imx6_system_info.display[3]!=0x12 && imx6_system_info.display[3]!=0x18)| | |
                        imx6_system_info.display[3]=0x12;                                     | | |
                    if(imx6_system_info.display[4]<0x28 || imx6_system_info.display[4]>0x64)  | | |
                        imx6_system_info.display[4]=0x3c;                                     | | |
                    //                                                                        | | |
                    if(imx6_system_info.display[5]<1 || imx6_system_info.display[5]>3)        | | |
                        imx6_system_info.display[5]=1;                                        | | |
                }                                                                             | | |
                ptr = strstr((const char *)tbuffer, "Logo=");                                 | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    uchar read_boot_logo=imx6_system_info.logo[2];                            | | |
                    read_boot_logo=(uchar)(atoi(ptr+5)&0xff);                                 | | |
                    if(read_boot_logo<1 || read_boot_logo>5)                                  | | |
                    {                                                                         | | |
                        read_boot_logo=1;                                                     | | |
                    }                                                                         | | |
                    imx6_system_info.logo[0]=0x11;                                            | | |
                    imx6_system_info.logo[1]=0x01;                                            | | |
                    imx6_system_info.logo[2]=0xff&read_boot_logo;                             | | |
                }                                                                             | | |
                ptr = strstr((const char *)tbuffer, "Halt");                                  | | |
                if (ptr != NULL)                                                              | | |
                {                                                                             | | |
                    setenv("Halt","y");                                                       | | |
                }                                                                             | | |
                write_systeminfo_eeprom();  -----------+                                      | | |
            }                                          |                                      | | |
        }                                              |                                      | | |
    }                                                  |                                      | | |
    return 0;                                          |                                      | | |
}                                                      |                                      | | |
                                                       |                                      | | |
static void write_systeminfo_eeprom(void)   <----------+                                      | | |
{                                                                                             | | |
    char tmp[128];                                                                            | | |
    uchar eepromtmp[10];                                                                      | | |
                                                                                              | | |
    puts("nMAC: ");                                                                           | | |
    sprintf(tmp,"write_systeminfo_eeprom:%02x:%02x:%02x:%02x:%02x:%02x\n",                    | | |
        imx6_system_info.mac1[2],imx6_system_info.mac1[3],                                    | | |
        imx6_system_info.mac1[4],imx6_system_info.mac1[5],                                    | | |
        imx6_system_info.mac1[6],imx6_system_info.mac1[7]);                                   | | |
    puts(tmp);                                                                                | | |
    int write_size_offset=0;                                                                  | | |
    AT24c02_eeprom.size=0;                                                                    | | |
    memset(AT24c02_eeprom.content,0,sizeof(AT24c02_eeprom.content));                          | | |
    if(imx6_system_info.mac1[0]!=0 && imx6_system_info.mac1[1]!=0)                            | | |
    {                                                                                         | | |
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,                                    | | |
            imx6_system_info.mac1,imx6_system_info.mac1[1]+2);                                | | |
        AT24c02_eeprom.size+=(imx6_system_info.mac1[1]+2);                                    | | |
    }                                                                                         | | |
    if(imx6_system_info.mac2[0]!=0 && imx6_system_info.mac2[1]!=0)                            | | |
    {                                                                                         | | |
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,                                    | | |
            imx6_system_info.mac2,imx6_system_info.mac2[1]+2);                                | | |
        AT24c02_eeprom.size+=(imx6_system_info.mac2[1]+2);                                    | | |
    }                                                                                         | | |
    if(imx6_system_info.softid[0]!=0 && imx6_system_info.softid[1]!=0)                        | | |
    {                                                                                         | | |
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,                                    | | |
            imx6_system_info.softid,imx6_system_info.softid[1]+2);                            | | |
        AT24c02_eeprom.size+=(imx6_system_info.softid[1]+2);                                  | | |
    }                                                                                         | | |
    if(imx6_system_info.backlight[0]!=0 && imx6_system_info.backlight[1]!=0)                  | | |
    {                                                                                         | | |
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,                                    | | |
            imx6_system_info.backlight,imx6_system_info.backlight[1]+2);                      | | |
        AT24c02_eeprom.size+=(imx6_system_info.backlight[1]+2);                               | | |
    }                                                                                         | | |
    if(imx6_system_info.display[0]!=0 && imx6_system_info.display[1]!=0)                      | | |
    {                                                                                         | | |
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,                                    | | |
            imx6_system_info.display,imx6_system_info.display[1]+2);                          | | |
        AT24c02_eeprom.size+=(imx6_system_info.display[1]+2);                                 | | |
    }                                                                                         | | |
    if(imx6_system_info.logo[0]!=0 && imx6_system_info.logo[1]!=0)                            | | |
    {                                                                                         | | |
        memcpy(AT24c02_eeprom.content+AT24c02_eeprom.size,                                    | | |
            imx6_system_info.logo,imx6_system_info.logo[1]+2);                                | | |
        AT24c02_eeprom.size+=(imx6_system_info.logo[1]+2);                                    | | |
    }                                                                                         | | |
    while(write_size_offset<AT24c02_eeprom.size)                                              | | |
    {                                                                                         | | |
        eeprom_i2c_write(write_size_offset, 1,                                                | | |
            AT24c02_eeprom.content+write_size_offset, 8);                                     | | |
        write_size_offset+=8;                                                                 | | |
    }                                                                                         | | |
    AT24c02_eeprom.version=imx6_system_info.version;                                          | | |
    eepromtmp[0]=0xff & imx6_system_info.version;                                             | | |
    eepromtmp[1]=0xff & (imx6_system_info.version>>8);                                        | | |
    eeprom_i2c_write(0xfe, 1, eepromtmp, 2);                                                  | | |
    sprintf(tmp,"AT24c02_eeprom.size:%d\n",AT24c02_eeprom.size);                              | | |
    puts(tmp);                                                                                | | |
}                                                                                             | | |
                                                                                              | | |
int board_late_init(void)                               <-------------------------------------+ | |
{                                                                                               | |
    unsigned char * pData;                                                                      | |
    u32 clocksource=0;                                                                          | |
#ifdef CONFIG_UBOOT_LOGO_ENABLE_OVER_8BITS    //for 16bpp or 32bpp logo                         | |
    unsigned int size = DISPLAY_WIDTH * DISPLAY_HEIGHT * (DISPLAY_BPP / 8);                     | |
    unsigned int start, count;                                                                  | |
    int i, bmpReady = 0;                                                                        | |
#if 1                                                                                           | |
    int mmc_dev = mmc_get_env_devno();                                                          | |
    struct mmc *mmc = find_mmc_device(mmc_dev);                                                 | |
#endif                                                                                          | |
                                                                                                | |
    pData = (unsigned char *)CONFIG_FB_BASE;                                                    | |
#if 0                                                                                           | |
    if (mmc)    {                                                                               | |
        if (mmc_init(mmc) == 0) {                                                               | |
            start = ALIGN(UBOOT_LOGO_BMP_ADDR, mmc->read_bl_len) / mmc->read_bl_len;            | |
            count = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;                           | |
            mmc->block_dev.block_read(mmc_dev, start, count, pData);                            | |
            bmpReady = 1;                                                                       | |
        }                                                                                       | |
    }                                                                                           | |
#endif                                                                                          | |
                                                                                                | |
    if (bmpReady == 0) {                                                                        | |
        // Fill RGB frame buffer                                                                | |
        // Red                                                                                  | |
        for (i = 0; i < (DISPLAY_WIDTH * DISPLAY_HEIGHT * (DISPLAY_BPP / 8) / 3);               | |
                i += (DISPLAY_BPP / 8)) {                                                       | |
#if (DISPLAY_BPP == 16)                                                                         | |
            pData[i + 0] = 0x00;                                                                | |
            pData[i + 1] = 0xF8;                                                                | |
#else                                                                                           | |
            pData[i + 0] = 0x00;                                                                | |
            pData[i + 1] = 0x00;                                                                | |
            pData[i + 2] = 0xFF;                                                                | |
            pData[i + 3] = 0x00;                                                                | |
#endif                                                                                          | |
        }                                                                                       | |
                                                                                                | |
        // Green                                                                                | |
        for (; i < (DISPLAY_WIDTH * DISPLAY_HEIGHT * (DISPLAY_BPP / 8) / 3) * 2;                | |
                i += (DISPLAY_BPP / 8)) {                                                       | |
#if (DISPLAY_BPP == 16)                                                                         | |
            pData[i + 0] = 0xE0;                                                                | |
            pData[i + 1] = 0x07;                                                                | |
#else                                                                                           | |
            pData[i + 0] = 0x00;                                                                | |
            pData[i + 1] = 0xFF;                                                                | |
            pData[i + 2] = 0x00;                                                                | |
            pData[i + 3] = 0x00;                                                                | |
#endif                                                                                          | |
        }                                                                                       | |
                                                                                                | |
        // Blue                                                                                 | |
        for (; i < DISPLAY_WIDTH * DISPLAY_HEIGHT * (DISPLAY_BPP / 8);                          | |
                i += (DISPLAY_BPP / 8)) {                                                       | |
#if (DISPLAY_BPP == 16)                                                                         | |
            pData[i + 0] = 0x1F;                                                                | |
            pData[i + 1] = 0x00;                                                                | |
#else                                                                                           | |
            pData[i + 0] = 0xFF;                                                                | |
            pData[i + 1] = 0x00;                                                                | |
            pData[i + 2] = 0x00;                                                                | |
            pData[i + 3] = 0x00;                                                                | |
#endif                                                                                          | |
        }                                                                                       | |
    }                                                                                           | |
#ifndef CONFIG_SYS_DCACHE_OFF                                                                   | |
    flush_dcache_range((u32)pData,                                                              | |
            (u32)(pData + DISPLAY_WIDTH * DISPLAY_HEIGHT * (DISPLAY_BPP / 8)));                 | |
#endif                                                                                          | |
                                                                                                | |
#ifdef IPU_OUTPUT_MODE_LVDS                                                                     | |
    setup_lvds_iomux();                                                                         | |
#endif                                                                                          | |
                                                                                                | |
#ifdef IPU_OUTPUT_MODE_LCD                                                                      | |
    ipu_iomux_config();                                                                         | |
    setup_lcd_iomux();                                                                          | |
#endif                                                                                          | |
                                                                                                | |
#ifdef IPU_OUTPUT_MODE_HDMI                                                                     | |
    setup_hdmi_iomux();                                                                         | |
#endif                                                                                          | |
                                                                                                | |
    ipu_display_setup(IPU_NUM, DI_NUM);                                                         | |
#else    //test                                                                                 | |
    customer_lvds();                                                                            | |
    pData = (unsigned char *)CONFIG_FB_BASE;                                                    | |
    clocksource=MXC_IPU1_LVDS_DI1_CLK;                                                          | |
    switch(imx6_system_info.display[2])                                                         | |
    {                                                                                           | |
        //setenv("bootargs","console=ttymxc0,115200 init=/init vi\                              | |
        // deo=mxcfb0:dev=ldb,800x480M@70,if=RGB666,bpp=32 vi\                                  | |
        // deo=mxcfb1:off video=mxcfb2:off fbmem=40M fb0base=0x27b00000 vma\                    | |
        // lloc=400M androidboot.console=ttymxc0 androidboot.hardware=fr\                       | |
        // eescale mem=1024M\0");                                                               | |
        case RESOLUTION_640X480:                                                                | |
            display_clk_config(clocksource, 34285715);                                          | |
            set_kernel_env(640,480);                           -------------------------+       | |
            copy_bmp_screen((char *)pData,(char *)fsl_bmp_reversed_600x400,640,480);    |       | |
            break;                                                                      |       | |
        default:                                                                        |       | |
        case RESOLUTION_800X480:                                                        |       | |
            display_clk_config(clocksource, 38000000);                                  |       | |
            set_kernel_env(800,480);                                                    |       | |
            copy_bmp_screen((char *)pData,(char *)fsl_bmp_reversed_600x400,800,480);    |       | |
            break;                                                                      |       | |
        case RESOLUTION_800X600:                                                        |       | |
            display_clk_config(clocksource, 38000000);                                  |       | |
            set_kernel_env(800,600);                                                    |       | |
            copy_bmp_screen((char *)pData,(char *)fsl_bmp_reversed_600x400,800,600);    |       | |
            break;                                                                      |       | |
        case RESOLUTION_1024X600:                                                       |       | |
            display_clk_config(clocksource, 51206400);                                  |       | |
            set_kernel_env(1024,600);                                                   |       | |
            copy_bmp_screen((char *)pData,(char *)fsl_bmp_reversed_600x400,1024,600);   |       | |
            break;                                                                      |       | |
        case RESOLUTION_1024X768:                                                       |       | |
            display_clk_config(clocksource, 64000000);                                  |       | |
            set_kernel_env(1024,768);                                                   |       | |
            copy_bmp_screen((char *)pData,(char *)fsl_bmp_reversed_600x400,1024,768);   |       | |
            break;                                                                      |       | |
        case RESOLUTION_1280X800:                                                       |       | |
            display_clk_config(clocksource, 65000000);                                  |       | |
            set_kernel_env(1280,800);                                                   |       | |
            copy_bmp_screen((char *)pData,(char *)fsl_bmp_reversed_600x400,1280,800);   |       | |
            break;                                                                      |       | |
        case RESOLUTION_1366X768:                                                       |       | |
            display_clk_config(clocksource, 74000000);                                  |       | |
            set_kernel_env(1366,768);                                                   |       | |
            copy_bmp_screen((char *)pData,(char *)fsl_bmp_reversed_600x400,1366,768);   |       | |
            break;                                                                      |       | |
        case RESOLUTION_1920X1080:                                                      |       | |
            copy_bmp_screen((char *)pData,(char *)fsl_bmp_reversed_600x400,1920,1080);  |       | |
            display_clk_config(MXC_IPU1_LVDS_DI0_CLK, 84000000);                        |       | |
            set_kernel_env(1920,1080);                                                  |       | |
            break;                                                                      |       | |
    }                                                                                   |       | |
//    pData = (unsigned char *)CONFIG_FB_BASE;                                          |       | |
//    memcpy(pData,fsl_bmp_reversed_600x400,fsl_bmp_reversed_600x400_size);             |       | |
    video_display_bitmap((ulong)pData,0,0);                                             |       | |
#endif                                                                                  |       | |
                                                                                        |       | |
#ifdef CONFIG_CMD_BMODE                                                                 |       | |
    add_board_boot_modes(board_boot_modes);                                             |       | |
#endif                                                                                  |       | |
                                                                                        |       | |
#ifdef CONFIG_ENV_IS_IN_MMC                                                             |       | |
    board_late_mmc_env_init();                                                          |       | |
#endif                                                                                  |       | |
    return 0;                                                                           |       | |
}                                                                                       |       | |
                                                                                        |       | |
void set_kernel_env(int width, int height)                         <--------------------+       | |
{                                                                                               | |
    char videoprm[128]={0};                                                                     | |
    char Backlightprm[128]={0};                                                                 | |
    char consoleprm[128]={0};                                                                   | |
    char envprm[512]={0};                                                                       | |
                                                                                                | |
    if(imx6_system_info.display[4]<30 || imx6_system_info.display[4]>100)                       | |
        imx6_system_info.display[4]=60;                                                         | |
    if(imx6_system_info.display[3]!=18 && imx6_system_info.display[3]!=24)                      | |
        imx6_system_info.display[3]=18;                                                         | |
    if(imx6_system_info.display[2]==RESOLUTION_1920X1080)                                       | |
        sprintf(videoprm,"video=mxcfb0:dev=ldb,%dx%dM@%u,if=RGB%d,bpp=32 ldb=spl1 vi\           | |
            deo=mxcfb1:off video=mxcfb2:off vmalloc=384M", width, height,                       | |
            imx6_system_info.display[4], imx6_system_info.display[3]==18?666:24);               | |
    else                                                                                        | |
        sprintf(videoprm,"video=mxcfb0:dev=ldb,%dx%dM@%u,if=RGB%d,bpp=32 vi\                    | |
            deo=mxcfb1:off video=mxcfb2:off vmalloc=256M", width, height,                       | |
            imx6_system_info.display[4], imx6_system_info.display[3]==18?666:24);               | |
#ifdef CONSOLE_READWRITE_ABLE                                                                   | |
    sprintf(consoleprm,"androidboot.console=ttymxc0 androidboot.selinux=dis\                    | |
            abled consoleblank=0");                                                             | |
#else                                                                                           | |
    sprintf(consoleprm,"androidboot.console=ttymxc0 consoleblank=0");                           | |
#endif                                                                                          | |
    if(imx6_system_info.backlight[0]==0x04 && imx6_system_info.backlight[1]==0x04)              | |
    {                                                                                           | |
        int check_backlight_frequency=imx6_system_info.backlight[4]*256+                        | |
            imx6_system_info.backlight[5];                                                      | |
        sprintf(Backlightprm,"Backlight_polarity=%d,Backlight_min=%d,Backlight_fr\              | |
                equency=%d",imx6_system_info.backlight[2]?1:0,                                  | |
                imx6_system_info.backlight[3],check_backlight_frequency);                       | |
        sprintf(envprm,"console=ttymxc0,115200 init=/init %s %s %s androidboot.ha\              | |
                rdware=freescale cma=384M",videoprm,consoleprm,Backlightprm);                   | |
    }                                                                                           | |
    else                                                                                        | |
    {                                                                                           | |
        sprintf(envprm,"console=ttymxc0,115200 init=/init %s %s androidboot.hardwa\             | |
                re=freescale cma=384M",videoprm,consoleprm);                                    | |
    }                                                                                           | |
    setenv("bootargs",envprm);                                                                  | |
    if(is_valid_ether_addr(imx6_system_info.mac1+2))                                            | |
    {                                                                                           | |
        sprintf(envprm,"%02x:%02x:%02x:%02x:%02x:%02x", imx6_system_info.mac1[2],               | |
            imx6_system_info.mac1[3], imx6_system_info.mac1[4],                                 | |
            imx6_system_info.mac1[5], imx6_system_info.mac1[6], imx6_system_info.mac1[7]);      | |
        setenv("ethaddr",envprm);                                                               | |
        setenv("fec_addr",envprm);                                                              | |
    }                                                                                           | |
}                                                                                               | |
                                                                                                | |
#ifdef CONFIG_CMD_NET                                                                           | |
static int initr_net(void)                                          <---------------------------+ |
{                                                                                                 |
    puts("Net:   ");                                                                              |
    eth_initialize(gd->bd);      ------+                                                          |
#if defined(CONFIG_RESET_PHY_R)        |                                                          |
    debug("Reset Ethernet PHY\n");     |                                                          |
    reset_phy();                       |                                                          |
#endif                                 |                                                          |
    return 0;                          |                                                          |
}                                      |                                                          |
#endif                                 |                                                          |
                                       |                                                          |
int eth_initialize(bd_t *bis)    <-----+                                                          |
{                                                                                                 |
    int num_devices = 0;                                                                          |
    eth_devices = NULL;                                                                           |
    eth_current = NULL;                                                                           |
                                                                                                  |
    bootstage_mark(BOOTSTAGE_ID_NET_ETH_START);                                                   |
#if defined(CONFIG_MII) || defined(CONFIG_CMD_MII) || defined(CONFIG_PHYLIB)                      |
    miiphy_init();                                                                                |
#endif                                                                                            |
                                                                                                  |
#ifdef CONFIG_PHYLIB                                                                              |
    phy_init();                  ---------------------------------+                               |
#endif                                                            |                               |
                                                                  |                               |
    eth_env_init(bis);                                            |                               |
                                                                  |                               |
    /*                                                            |                               |
     * If board-specific initialization exists, call it.          |                               |
     * If not, call a CPU-specific one                            |                               |
     */                                                           |                               |
    if (board_eth_init != __def_eth_init) {                       |                               |
        if (board_eth_init(bis) < 0)                              |                               |
            printf("Board Net Initialization Failed\n");          |                               |
    } else if (cpu_eth_init != __def_eth_init) {                  |                               |
        if (cpu_eth_init(bis) < 0)          ----------------------*-+                             |
            printf("CPU Net Initialization Failed\n");            | |                             |
    } else                                                        | |                             |
        printf("Net Initialization Skipped\n");                   | |                             |
                                                                  | |                             |
    if (!eth_devices) {                                           | |                             |
        puts("No ethernet found.\n");                             | |                             |
        bootstage_error(BOOTSTAGE_ID_NET_ETH_START);              | |                             |
    } else {                                                      | |                             |
        struct eth_device *dev = eth_devices;                     | |                             |
        char *ethprime = getenv("ethprime");                      | |                             |
                                                                  | |                             |
        bootstage_mark(BOOTSTAGE_ID_NET_ETH_INIT);                | |                             |
        do {                                                      | |                             |
            if (dev->index)                                       | |                             |
                puts(", ");                                       | |                             |
                                                                  | |                             |
            printf("%s", dev->name);                              | |                             |
                                                                  | |                             |
            if (ethprime && strcmp(dev->name, ethprime) == 0) {   | |                             |
                eth_current = dev;                                | |                             |
                puts(" [PRIME]");                                 | |                             |
            }                                                     | |                             |
                                                                  | |                             |
            if (strchr(dev->name, ' '))                           | |                             |
                puts("\nWarning: eth device name has a space!"    | |                             |
                    "\n");                                        | |                             |
                                                                  | |                             |
            eth_write_hwaddr(dev, "eth", dev->index); ------------*-*-------+                     |
                                                                  | |       |                     |
            dev = dev->next;                                      | |       |                     |
            num_devices++;                                        | |       |                     |
        } while (dev != eth_devices);                             | |       |                     |
                                                                  | |       |                     |
        eth_current_changed();                                    | |       |                     |
        putc('\n');                                               | |       |                     |
    }                                                             | |       |                     |
                                                                  | |       |                     |
    return num_devices;                                           | |       |                     |
}                                                                 | |       |                     |
                                                                  | |       |                     |
int phy_init(void)               <--------------------------------+ |       |                     |
{                                                                   |       |                     |
#ifdef CONFIG_PHY_AQUANTIA                                          |       |                     |
    phy_aquantia_init();                                            |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_ATHEROS                                           |       |                     |
    phy_atheros_init();                                             |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_BROADCOM                                          |       |                     |
    phy_broadcom_init();                                            |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_CORTINA                                           |       |                     |
    phy_cortina_init();                                             |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_DAVICOM                                           |       |                     |
    phy_davicom_init();                                             |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_ET1011C                                           |       |                     |
    phy_et1011c_init();                                             |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_LXT                                               |       |                     |
    phy_lxt_init();                                                 |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_MARVELL                                           |       |                     |
    phy_marvell_init();                                             |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_MICREL                                            |       |                     |
    phy_micrel_init();                                              |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_NATSEMI                                           |       |                     |
    phy_natsemi_init();                                             |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_REALTEK                                           |       |                     |
    phy_realtek_init();                                             |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_SMSC                                              |       |                     |
    phy_smsc_init();                                                |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_TERANETICS                                        |       |                     |
    phy_teranetics_init();                                          |       |                     |
#endif                                                              |       |                     |
#ifdef CONFIG_PHY_VITESSE                                           |       |                     |
    phy_vitesse_init();                                             |       |                     |
#endif                                                              |       |                     |
                                                                    |       |                     |
    return 0;                                                       |       |                     |
}                                                                   |       |                     |
                                                                    |       |                     |
                                                                    |       |                     |
int cpu_eth_init(bd_t *bis)              <--------------------------+       |                     |
{                                                                           |                     |
    int rc = -ENODEV;                                                       |                     |
                                                                            |                     |
#if defined(CONFIG_FEC_MXC)                                                 |                     |
    rc = fecmxc_initialize(bis);       ----------+                          |                     |
#endif                                           |                          |                     |
                                                 |                          |                     |
    return rc;                                   |                          |                     |
}                                                |                          |                     |
                                                 |                          |                     |
#ifdef CONFIG_FEC_MXC_PHYADDR                    |                          |                     |
int fecmxc_initialize(bd_t *bd)        <---------+                          |                     |
{                                                                           |                     |
    return fecmxc_initialize_multi(bd, -1, CONFIG_FEC_MXC_PHYADDR,          |                     |
            IMX_FEC_BASE); |                                                |                     |
}                          |                                                |                     |
#endif                     |                                                |                     |
                           V                                                |                     |
int fecmxc_initialize_multi(bd_t *bd, int dev_id, int phy_id, uint32_t addr)|                     |
{                                                                           |                     |
    uint32_t base_mii;                                                      |                     |
    struct mii_dev *bus = NULL;                                             |                     |
#ifdef CONFIG_PHYLIB                                                        |                     |
    struct phy_device *phydev = NULL;                                       |                     |
#endif                                                                      |                     |
    int ret;                                                                |                     |
                                                                            |                     |
#ifdef CONFIG_MX6                                                           |                     |
    if (mx6_enet_fused(addr)) {                                             |                     |
        printf("Ethernet@0x%x is fused, disable it\n", addr);               |                     |
        return -2;                                                          |                     |
    }                                                                       |                     |
#endif                                                                      |                     |
                                                                            |                     |
#ifdef CONFIG_MX28                                                          |                     |
    /*                                                                      |                     |
     * The i.MX28 has two ethernet interfaces, but they are not equal.      |                     |
     * Only the first one can access the MDIO bus.                          |                     |
     */                                                                     |                     |
    base_mii = MXS_ENET0_BASE;                                              |                     |
#else                                                                       |                     |
#ifdef CONFIG_FEC_MXC_MDIO_BASE                                             |                     |
    base_mii = CONFIG_FEC_MXC_MDIO_BASE;                                    |                     |
#else                                                                       |                     |
    base_mii = addr;                                                        |                     |
#endif                                                                      |                     |
#endif                                                                      |                     |
    debug("eth_init: fec_probe(bd, %i, %i) @ %08x\n", dev_id, phy_id, addr);|                     |
    bus = fec_get_miibus(base_mii, dev_id);                                 |                     |
    if (!bus)                                                               |                     |
        return -ENOMEM;                                                     |                     |
#ifdef CONFIG_PHYLIB                                                        |                     |
    phydev = phy_find_by_mask(bus, 1 << phy_id, PHY_INTERFACE_MODE_RGMII);  |                     |
    if (!phydev) {                                                          |                     |
        free(bus);                                                          |                     |
        return -ENOMEM;                                                     |                     |
    }                                                                       |                     |
    ret = fec_probe(bd, dev_id, addr, bus, phydev);   ---------+            |                     |
#else                                                          |            |                     |
    ret = fec_probe(bd, dev_id, addr, bus, phy_id);   ---------*-+          |                     |
#endif                                                         | |          |                     |
    if (ret) {                                                 | |          |                     |
#ifdef CONFIG_PHYLIB                                           | |          |                     |
        free(phydev);                                          | |          |                     |
#endif                                                         | |          |                     |
        free(bus);                                             | |          |                     |
    }                                                          | |          |                     |
    return ret;                                                | |          |                     |
}                                                              | |          |                     |
                                                               | |          |                     |
#ifdef CONFIG_PHYLIB                                           | |          |                     |
int fec_probe(bd_t *bd, int dev_id, uint32_t base_addr,  <-----+ |          |                     |
        struct mii_dev *bus, struct phy_device *phydev)          |          |                     |
#else                                                            |          |                     |
static int fec_probe(bd_t *bd, int dev_id, uint32_t base_addr,<--+          |                     |
        struct mii_dev *bus, int phy_id)                                    |                     |
#endif                                                                      |                     |
{                                                                           |                     |
    struct eth_device *edev;                                                |                     |
    struct fec_priv *fec;                                                   |                     |
    unsigned char ethaddr[6];                                               |                     |
    uint32_t start;                                                         |                     |
    int ret = 0;                                                            |                     |
                                                                            |                     |
    /* create and fill edev struct */                                       |                     |
    edev = (struct eth_device *)malloc(sizeof(struct eth_device));          |                     |
    if (!edev) {                                                            |                     |
        puts("fec_mxc: not enough malloc memory for eth_device\n");         |                     |
        ret = -ENOMEM;                                                      |                     |
        goto err1;                                                          |                     |
    }                                                                       |                     |
                                                                            |                     |
    fec = (struct fec_priv *)malloc(sizeof(struct fec_priv));               |                     |
    if (!fec) {                                                             |                     |
        puts("fec_mxc: not enough malloc memory for fec_priv\n");           |                     |
        ret = -ENOMEM;                                                      |                     |
        goto err2;                                                          |                     |
    }                                                                       |                     |
                                                                            |                     |
    memset(edev, 0, sizeof(*edev));                                         |                     |
    memset(fec, 0, sizeof(*fec));                                           |                     |
                                                                            |                     |
    ret = fec_alloc_descs(fec);                                             |                     |
    if (ret)                                                                |                     |
        goto err3;                                                          |                     |
                                                                            |                     |
    edev->priv = fec;                                                       |                     |
    edev->init = fec_init;                                                  |                     |
    edev->send = fec_send;                                                  |                     |
    edev->recv = fec_recv;                                                  |                     |
    edev->halt = fec_halt;                                                  |                     |
    edev->write_hwaddr = fec_set_hwaddr;                                    |                     |
                                                                            |                     |
    fec->eth = (struct ethernet_regs *)base_addr;                           |                     |
    fec->bd = bd;                                                           |                     |
                                                                            |                     |
    fec->xcv_type = CONFIG_FEC_XCV_TYPE;                                    |                     |
                                                                            |                     |
    /* Reset chip. */                                                       |                     |
    writel(readl(&fec->eth->ecntrl) | FEC_ECNTRL_RESET, &fec->eth->ecntrl); |                     |
    start = get_timer(0);                                                   |                     |
    while (readl(&fec->eth->ecntrl) & FEC_ECNTRL_RESET) {                   |                     |
        if (get_timer(start) > (CONFIG_SYS_HZ * 5)) {                       |                     |
            printf("FEC MXC: Timeout reseting chip\n");                     |                     |
            goto err4;                                                      |                     |
        }                                                                   |                     |
        udelay(10);                                                         |                     |
    }                                                                       |                     |
                                                                            |                     |
    fec_reg_setup(fec);                                                     |                     |
    fec_set_dev_name(edev->name, dev_id);                                   |                     |
    fec->dev_id = (dev_id == -1) ? 0 : dev_id;                              |                     |
    fec->bus = bus;                                                         |                     |
    fec_mii_setspeed(bus->priv);                                            |                     |
#ifdef CONFIG_PHYLIB                                                        |                     |
    fec->phydev = phydev;                                                   |                     |
    phy_connect_dev(phydev, edev);                                          |                     |
    /* Configure phy */                                                     |                     |
    phy_config(phydev);                                                     |                     |
#else                                                                       |                     |
    fec->phy_id = phy_id;                                                   |                     |
#endif                                                                      |                     |
    eth_register(edev);                                                     |                     |
                                                                            |                     |
    if (fec_get_hwaddr(edev, dev_id, ethaddr) == 0) {   --------------+     |                     |
        debug("got MAC%d address from fuse: %pM\n", dev_id, ethaddr); |     |                     |
        memcpy(edev->enetaddr, ethaddr, 6);                           |     |                     |
        if (!getenv("ethaddr"))                                       |     |                     |
            eth_setenv_enetaddr("ethaddr", ethaddr);                  |     |                     |
    }                                                                 |     |                     |
    return ret;                                                       |     |                     |
err4:                                                                 |     |                     |
    fec_free_descs(fec);                                              |     |                     |
err3:                                                                 |     |                     |
    free(fec);                                                        |     |                     |
err2:                                                                 |     |                     |
    free(edev);                                                       |     |                     |
err1:                                                                 |     |                     |
    return ret;                                                       |     |                     |
}                                                                     |     |                     |
                                                                      |     |                     |
static int fec_get_hwaddr(struct eth_device *dev, int dev_id, <-------+     |                     |
                        unsigned char *mac)                                 |                     |
{                                                                           |                     |
    imx_get_mac_from_fuse(dev_id, mac);    -----------------------+         |                     |
    return !is_valid_ether_addr(mac);                             |         |                     |
}                                                                 |         |                     |
                                                                  |         |                     |
#if defined(CONFIG_FEC_MXC)                                       |         |                     |
void imx_get_mac_from_fuse(int dev_id, unsigned char *mac)  <-----+         |                     |
{                                                                           |                     |
    struct ocotp_regs *ocotp = (struct ocotp_regs *)OCOTP_BASE_ADDR;        |                     |
    struct fuse_bank *bank = &ocotp->bank[4];                               |                     |
    struct fuse_bank4_regs *fuse =                                          |                     |
            (struct fuse_bank4_regs *)bank->fuse_regs;                      |                     |
                                                                            |                     |
#if (defined(CONFIG_MX6SX) || defined(CONFIG_MX6UL))                        |                     |
    if (0 == dev_id) {                                                      |                     |
        u32 value = readl(&fuse->mac_addr1);                                |                     |
        mac[0] = (value >> 8);                                              |                     |
        mac[1] = value ;                                                    |                     |
                                                                            |                     |
        value = readl(&fuse->mac_addr0);                                    |                     |
        mac[2] = value >> 24 ;                                              |                     |
        mac[3] = value >> 16 ;                                              |                     |
        mac[4] = value >> 8 ;                                               |                     |
        mac[5] = value ;                                                    |                     |
    } else {                                                                |                     |
        u32 value = readl(&fuse->mac_addr2);                                |                     |
        mac[0] = value >> 24 ;                                              |                     |
        mac[1] = value >> 16 ;                                              |                     |
        mac[2] = value >> 8 ;                                               |                     |
        mac[3] = value ;                                                    |                     |
                                                                            |                     |
        value = readl(&fuse->mac_addr1);                                    |                     |
        mac[4] = value >> 24 ;                                              |                     |
        mac[5] = value >> 16 ;                                              |                     |
    }                                                                       |                     |
#else                                                                       |                     |
    u32 value = readl(&fuse->mac_addr_high);                                |                     |
    mac[0] = (value >> 8);                                                  |                     |
    mac[1] = value ;                                                        |                     |
                                                                            |                     |
    value = readl(&fuse->mac_addr_low);                                     |                     |
    mac[2] = value >> 24 ;                                                  |                     |
    mac[3] = value >> 16 ;                                                  |                     |
    mac[4] = value >> 8 ;                                                   |                     |
    mac[5] = value ;                                                        |                     |
                                                                            |                     |
#endif                                                                      |                     |
}                                                                           |                     |
#endif                                                                      |                     |
                                                                            |                     |
int eth_write_hwaddr(struct eth_device *dev, const char *base_name,   <-----+                     |
           int eth_number)                                                                        |
{                                                                                                 |
    unsigned char env_enetaddr[6];                                                                |
    int ret = 0;                                                                                  |
                                                                                                  |
    eth_getenv_enetaddr_by_index(base_name, eth_number, env_enetaddr);  ----+                     |
                                                                            |                     |
    if (eth_address_set(env_enetaddr)) {                                    |                     |
        if (eth_address_set(dev->enetaddr) &&                               |                     |
                memcmp(dev->enetaddr, env_enetaddr, 6)) {                   |                     |
            printf("\nWarning: %s MAC addresses don't match:\n",            |                     |
                dev->name);                                                 |                     |
            printf("Address in SROM is         %pM\n",                      |                     |
                dev->enetaddr);                                             |                     |
            printf("Address in environment is  %pM\n",                      |                     |
                env_enetaddr);                                              |                     |
        }                                                                   |                     |
                                                                            |                     |
        memcpy(dev->enetaddr, env_enetaddr, 6);                             |                     |
    } else if (is_valid_ether_addr(dev->enetaddr)) {                        |                     |
        eth_setenv_enetaddr_by_index(base_name, eth_number,                 |                     |
                         dev->enetaddr);                                    |                     |
        printf("\nWarning: %s using MAC address from net device\n",         |                     |
            dev->name);                                                     |                     |
    } else if (!(eth_address_set(dev->enetaddr))) {                         |                     |
        printf("\nError: %s address not set.\n",                            |                     |
               dev->name);                                                  |                     |
        return -EINVAL;                                                     |                     |
    }                                                                       |                     |
                                                                            |                     |
    if (dev->write_hwaddr && !eth_mac_skip(eth_number)) {                   |                     |
        if (!is_valid_ether_addr(dev->enetaddr)) {                          |                     |
            printf("\nError: %s address %pM illegal value\n",               |                     |
                 dev->name, dev->enetaddr);                                 |                     |
            return -EINVAL;                                                 |                     |
        }                                                                   |                     |
                                                                            |                     |
        ret = dev->write_hwaddr(dev);                                       |                     |
        if (ret)                                                            |                     |
            printf("\nWarning: %s failed to set MAC address\n", dev->name); |                     |
    }                                                                       |                     |
                                                                            |                     |
    return ret;                                                             |                     |
}                                                                           |                     |
                                                                            |                     |
int eth_getenv_enetaddr_by_index(const char *base_name, int index,   <------+                     |
                 uchar *enetaddr)                                                                 |
{                                                                                                 |
    char enetvar[32];                                                                             |
    sprintf(enetvar, index ? "%s%daddr" : "%saddr", base_name, index);                            |
    return eth_getenv_enetaddr(enetvar, enetaddr);                                                |
}                                                                                                 |
                                                                                                  |
int stdio_add_devices(void)                       <-----------------------------------------------+
{
#ifdef CONFIG_SYS_I2C
    i2c_init_all();
#else
#if defined(CONFIG_HARD_I2C)
    i2c_init (CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif
#endif
#ifdef CONFIG_LCD
    drv_lcd_init ();
#endif
#if defined(CONFIG_VIDEO) || defined(CONFIG_CFB_CONSOLE)
    drv_video_init ();              -------------+
#endif                                           |
#ifdef CONFIG_KEYBOARD                           |
    drv_keyboard_init ();                        |
#endif                                           |
#ifdef CONFIG_LOGBUFFER                          |
    drv_logbuff_init ();                         |
#endif                                           |
    drv_system_init ();                          |
    serial_stdio_init ();                        |
#ifdef CONFIG_USB_TTY                            |
    drv_usbtty_init ();                          |
#endif                                           |
#ifdef CONFIG_NETCONSOLE                         |
    drv_nc_init ();                              |
#endif                                           |
#ifdef CONFIG_JTAG_CONSOLE                       |
    drv_jtag_console_init ();                    |
#endif                                           |
#ifdef CONFIG_CBMEM_CONSOLE                      |
    cbmemc_init();                               |
#endif                                           |
                                                 |
    return 0;                                    |
}                                                |
                                                 |
int drv_video_init(void)              <----------+
{
    int skip_dev_init;
    struct stdio_dev console_dev;

    /* Check if video initialization should be skipped */
    //printf("%s:%s:%i: here i am\n", __FILE__, __func__, __LINE__);
#ifdef CONFIG_SBC7112
    set_panel_env();                  -----------------------------+
#endif                                                             |
    if (board_video_skip())           -----------------------------*--------------+
        return 0;                                                  |              |
                                                                   |              |
    /* Init video chip - returns with framebuffer cleared */       |              |
    skip_dev_init = (video_init() == -1);                          |              |
                                                                   |              |
    if (board_cfb_skip())                                          |              |
        return 0;                                                  |              |
                                                                   |              |
#if !defined(CONFIG_VGA_AS_SINGLE_DEVICE)                          |              |
    debug("KBD: Keyboard init ...\n");                             |              |
    skip_dev_init |= (VIDEO_KBD_INIT_FCT == -1);                   |              |
#endif                                                             |              |
                                                                   |              |
    if (skip_dev_init)                                             |              |
        return 0;                                                  |              |
                                                                   |              |
    /* Init vga device */                                          |              |
    memset(&console_dev, 0, sizeof(console_dev));                  |              |
    strcpy(console_dev.name, "vga");                               |              |
    console_dev.ext = DEV_EXT_VIDEO;    /* Video extensions */     |              |
    console_dev.flags = DEV_FLAGS_OUTPUT | DEV_FLAGS_SYSTEM;       |              |
    console_dev.putc = video_putc;    /* 'putc' function */        |              |
    console_dev.puts = video_puts;    /* 'puts' function */        |              |
                                                                   |              |
#if !defined(CONFIG_VGA_AS_SINGLE_DEVICE)                          |              |
    /* Also init console device */                                 |              |
    console_dev.flags |= DEV_FLAGS_INPUT;                          |              |
    console_dev.tstc = VIDEO_TSTC_FCT;    /* 'tstc' function */    |              |
    console_dev.getc = VIDEO_GETC_FCT;    /* 'getc' function */    |              |
#endif /* CONFIG_VGA_AS_SINGLE_DEVICE */                           |              |
                                                                   |              |
    if (stdio_register(&console_dev) != 0)                         |              |
        return 0;                                                  |              |
                                                                   |              |
    /* Return success */                                           |              |
    return 1;                                                      |              |
}                                                                  |              |
                                                                   |              |
void set_panel_env(void)                <--------------------------+              |
{                                                                                 |
    char tmpchar[128]={0};                                                        |
    switch(imx6_system_info.display[2])                                           |
    {                                                                             |
        case RESOLUTION_640X480:        ----------------------------------------+ |
            sprintf(tmpchar,"panel640x480d%d",imx6_system_info.display[3]);     | |
            break;                                                              | |
        default:                                                                | |
        case RESOLUTION_800X480:                                                | |
            sprintf(tmpchar,"panel800x480d%d",imx6_system_info.display[3]);     | |
            break;                                                              | |
        case RESOLUTION_800X600:                                                | |
            sprintf(tmpchar,"panel800x600d%d",imx6_system_info.display[3]);     | |
            break;                                                              | |
        case RESOLUTION_1024X600:                                               | |
            sprintf(tmpchar,"panel1024x600d%d",imx6_system_info.display[3]);    | |
            break;                                                              | |
        case RESOLUTION_1024X768:                                               | |
            sprintf(tmpchar,"panel1024x768d%d",imx6_system_info.display[3]);    | |
            break;                                                              | |
        case RESOLUTION_1280X800:                                               | |
            sprintf(tmpchar,"panel1280x800d%d",imx6_system_info.display[3]);    | |
            break;                                                              | |
        case RESOLUTION_1366X768:                                               | |
            sprintf(tmpchar,"panel1366x768d%d",imx6_system_info.display[3]);    | |
            break;                                                              | |
        case RESOLUTION_1920X1080:                                              | |
            sprintf(tmpchar,"panel1920x1080d%d",imx6_system_info.display[3]);   | |
            break;                                                              | |
    }                                                                           | |
    setenv("panel",tmpchar);                                                    | |
}                                                                               | |
                                                                                | |
#define RESOLUTION_640X480        0x01            //    1          <------------+ |
#define RESOLUTION_800X480        0x02            //    2                         |
#define RESOLUTION_800X600        0x03            //    3                         |
#define RESOLUTION_1024X600        0x04           //    4                         |
#define RESOLUTION_1024X768        0x05           //    5                         |
#define RESOLUTION_1280X800        0x06           //    6                         |
#define RESOLUTION_1366X768        0x07           //    7                         |
#define RESOLUTION_1920X1080        0x08          //    8                         |
                                                                                  |
int board_video_skip(void)                                         <--------------+
{
    int i;
    int ret;
    char const *panel = getenv("panel");

    if (!panel) {
        for (i = 0; i < display_count; i++) {
            struct display_info_t const *dev = displays+i;        --------------+
            if (dev->detect && dev->detect(dev)) {                              |
                panel = dev->mode.name;                                         |
                printf("auto-detected panel %s\n", panel);                      |
                break;                                                          |
            }                                                                   |
        }                                                                       |
        if (!panel) {                                                           |
            panel = displays[0].mode.name;                                      |
            printf("No panel detected: default to %s\n", panel);                |
            i = 0;                                                              |
        }                                                                       |
    } else {                                                                    |
        for (i = 0; i < display_count; i++) {                                   |
            if (!strcmp(panel, displays[i].mode.name))                          |
                break;                                                          |
        }                                                                       |
    }                                                                           |
                                                                                |
    if (i < display_count) {                                                    |
        ret = ipuv3_fb_init(&displays[i].mode, 0,                               |
                    displays[i].pixfmt);                                        |
        if (!ret) {                                                             |
            if (displays[i].enable)                                             |
                displays[i].enable(displays + i);                               |
                                                                                |
            printf("Display: %s (%ux%u)\n",                                     |
                   displays[i].mode.name,                                       |
                   displays[i].mode.xres,                                       |
                   displays[i].mode.yres);                                      |
        } else                                                                  |
            printf("LCD %s cannot be configured: %d\n",                         |
                   displays[i].mode.name, ret);                                 |
    } else {                                                                    |
        printf("unsupported panel %s\n", panel);                                |
        return -EINVAL;                                                         |
    }                                                                           |
                                                                                |
    return 0;                                                                   |
}                                                                               |
                                                                                |
struct display_info_t const displays[] = {{                     <---------------+
    .bus    = -1,
    .addr    = 0,
    .pixfmt    = IPU_PIX_FMT_RGB24,
    .detect    = NULL,
    .enable    = do_enable_hdmi,
    .mode    = {
        .name           = "HDMI",
        .refresh        = 60,
        .xres           = 640,
        .yres           = 480,
        .pixclock       = 39721,
        .left_margin    = 48,
        .right_margin   = 16,
        .upper_margin   = 33,
        .lower_margin   = 10,
        .hsync_len      = 96,
        .vsync_len      = 2,
        .sync           = 0,
        .vmode          = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt    = IPU_PIX_FMT_RGB666,
    .detect    = NULL,
    .enable    = enable_lvds,
    .mode    = {
        .name           = "panel800x480d18",
        .refresh        = 67,
        .xres           = 800,    //800+263=1070
        .yres           = 480,    //480+59=538
        .pixclock       = 29166,
        .left_margin    = 100,    //100+160+10=270
        .right_margin   = 160,
        .upper_margin   = 39,    //38+10+10=58
        .lower_margin   = 10,
        .hsync_len      = 10,
        .vsync_len      = 10,
        .sync           = 0,
        .vmode          = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB24,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel800x480d24",
        .refresh        = 67,
        .xres            = 800,    //800+263=1070
        .yres            = 480,    //480+59=538
        .pixclock        = 29166,
        .left_margin    = 100,    //100+160+10=270
        .right_margin    = 160,
        .upper_margin    = 39,    //38+10+10=58
        .lower_margin    = 10,
        .hsync_len        = 10,
        .vsync_len        = 10,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB666,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel800x600d18",
        .refresh        = 60,
        .xres            = 800,
        .yres            = 600,
        .pixclock        = 26143,
        .left_margin    = 112,
        .right_margin    = 32,
        .upper_margin    = 3,
        .lower_margin    = 17,
        .hsync_len        = 80,
        .vsync_len        = 4,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB24,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel800x600d24",
        .refresh        = 60,
        .xres            = 800,
        .yres            = 600,
        .pixclock        = 26143,
        .left_margin    = 112,
        .right_margin    = 32,
        .upper_margin    = 3,
        .lower_margin    = 17,
        .hsync_len        = 80,
        .vsync_len        = 4,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB666,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel1024x600d18",
        .refresh        = 60,
        .xres            = 1024,    //1024+320=1344
        .yres            = 600,    //600+35=635
        .pixclock        = 19528,
        .left_margin    = 220,    //220+40+60=320
        .right_margin    = 40,
        .upper_margin    = 18,    //18+7+10=35
        .lower_margin    = 7,
        .hsync_len        = 60,
        .vsync_len        = 10,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB24,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel1024x600d24",
        .refresh        = 60,
        .xres            = 1024, //1024+320=1344
        .yres            = 600,    //600+35=635
        .pixclock        = 19528,
        .left_margin    = 220,    //220+40+60=320
        .right_margin    = 40,
        .upper_margin    = 18,    //18+7+10=35
        .lower_margin    = 7,
        .hsync_len        = 60,
        .vsync_len        = 10,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB666,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel1024x768d18",
        .refresh        = 60,
        .xres            = 1024,
        .yres            = 768,
        .pixclock        = 15748,
        .left_margin    = 151,
        .right_margin    = 48,
        .upper_margin    = 4,
        .lower_margin    = 24,
        .hsync_len        = 104,
        .vsync_len        = 4,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB24,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel1024x768d24",
        .refresh        = 60,
        .xres            = 1024,
        .yres            = 768,
        .pixclock        = 15748,
        .left_margin    = 152,
        .right_margin    = 48,
        .upper_margin    = 3,
        .lower_margin    = 23,
        .hsync_len        = 104,
        .vsync_len        = 4,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB666,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel1280x800d18",
        .refresh        = 60,
        .xres            = 1280,
        .yres            = 800,    //600+35=635
        .pixclock        = 13966,
        .left_margin    = 110,    //220+40+60=320
        .right_margin    = 53,
        .upper_margin    = 8,    //18+7+10=35
        .lower_margin    = 8,
        .hsync_len        = 7,
        .vsync_len        = 7,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB24,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel1280x800d24",
        .refresh        = 60,
        .xres            = 1280,
        .yres            = 800,    //600+35=635
        .pixclock        = 13966,
        .left_margin    = 110,    //220+40+60=320
        .right_margin    = 53,
        .upper_margin    = 8,    //18+7+10=35
        .lower_margin    = 8,
        .hsync_len        = 7,
        .vsync_len        = 7,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB666,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel1366x768d18",
        .refresh        = 60,
        .xres            = 1366,
        .yres            = 768,
        .pixclock        = 13250,
        .left_margin    = 50,
        .right_margin    = 50,
        .upper_margin    = 9,
        .lower_margin    = 9,
        .hsync_len        = 93,
        .vsync_len        = 20,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB24,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel1366x768d24",
        .refresh        = 60,
        .xres            = 1366,
        .yres            = 768,
        .pixclock        = 13250,
        .left_margin    = 50,
        .right_margin    = 50,
        .upper_margin    = 9,
        .lower_margin    = 9,
        .hsync_len        = 93,
        .vsync_len        = 20,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB666,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel1920x1080d18",
        .refresh        = 60,
        .xres            = 1920,
        .yres            = 1080,
        .pixclock        = 13250,
        .left_margin    = 50,
        .right_margin    = 50,
        .upper_margin    = 10,
        .lower_margin    = 10,
        .hsync_len        = 80,
        .vsync_len        = 18,
        .sync            = 0,
        .vmode            = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 1,
    .addr    = 0,
    .pixfmt = IPU_PIX_FMT_RGB24,
    .detect = NULL,
    .enable = enable_lvds,
    .mode    = {
        .name            = "panel1920x1080d24",
#if 1
        .refresh        = 60,
        .xres            = 1920,
        .yres            = 1080,
        .pixclock        = 13250,
        .left_margin    = 50,
        .right_margin    = 50,
        .upper_margin    = 10,
        .lower_margin    = 10,
        .hsync_len        = 80,
        .vsync_len        = 18,
        .sync            = 0,
#else
        .refresh        = 60,
        .xres            = 1920,
        .yres            = 1080,
        .pixclock        = 11560,
        .left_margin    = 328,
        .right_margin    = 128,
        .upper_margin    = 3,
        .lower_margin    = 32,
        .hsync_len        = 200,
        .vsync_len        = 5,
        .sync            = 0,
#endif
        .vmode            = FB_VMODE_NONINTERLACED
} } };
//
size_t display_count = ARRAY_SIZE(displays);
