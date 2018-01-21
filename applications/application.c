/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-07-12     aozima       update for auto initial.
 */

/**
 * @addtogroup ESP8266
 */
/*@{*/

#include <rtthread.h>

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#include "esp_common.h"

void rt_init_thread_entry(void* parameter)
{
    uint8_t t = 0;

    rt_hw_board_init();

#ifdef RT_USING_FINSH
    /* init finsh */
    finsh_system_init();
#endif

#if 1
    wifi_set_opmode_current(STATION_MODE);

//    wifi_station_set_auto_connect(TRUE);

    struct station_config config;

    memset(&config, 0, sizeof(config));

    strcpy(config.ssid, "360WiFi-FB321E");
    strcpy(config.password, "20112012pw54321");

//    strcpy(config.ssid, "Xiaomi_7937");
//    strcpy(config.password, "chenyisong12345");
    
    wifi_station_set_config_current(&config);
    
    wifi_station_connect();
    
    while(1)
    {
        vTaskDelay(100);
//        os_printf("%s\n", __FUNCTION__);
//        rt_kprintf("%s %d\n", __FUNCTION__, t ++);
    }
#endif
}



/*@}*/
