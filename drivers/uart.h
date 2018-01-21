/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-10-05     flyingcys    add uart.h to this bsp
 */

#ifndef __UART_H__
#define __UART_H__

#include "esp_common.h"
#include "drv_uart.h"

void rt_hw_uart_init(void);

#endif

