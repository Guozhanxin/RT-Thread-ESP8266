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
 * 2017-10-05     flyingcys    first implementation
 */
 
#include <rthw.h>
#include <rtthread.h>
#include "board.h"

void xPortSysTickHandle(void)
{
    /* enter interrupt */
	rt_interrupt_enter();

	rt_tick_increase();

    /* leave interrupt */
	rt_interrupt_leave();
}

void rt_hw_console_output(const char *str)
{
//   os_printf(str);
}

/**
 * This function will initial ESP8266 board.
 */
void rt_hw_board_init(void)
{
    rt_hw_uart_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
}

