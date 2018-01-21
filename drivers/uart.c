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
#include <rtdevice.h>
#include "uart.h"

struct esp_uart
{
    UART_Port device;
};

#ifdef RT_USING_UART1
struct esp_uart uart0;
struct rt_serial_device serial0;

void uart0_rx_intr_handler(void *para)
{
    /* uart0 and uart1 intr combine togther, when interrupt occur, see reg 0x3ff20020, bit2, bit0 represents
    * uart1 and uart0 respectively
    */
    uint8 RcvChar;
    uint8 uart_no = UART0;//UartDev.buff_uart_no;
    uint8 fifo_len = 0;
    uint8 buf_idx = 0;
    uint8 fifo_tmp[128] = {0};

    uint32 uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)) ;

    while (uart_intr_status != 0x0) 
    {
        if (UART_FRM_ERR_INT_ST == (uart_intr_status & UART_FRM_ERR_INT_ST)) 
        {
            //printf("FRM_ERR\r\n");
            WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_FRM_ERR_INT_CLR);
        } 
        else if (UART_RXFIFO_FULL_INT_ST == (uart_intr_status & UART_RXFIFO_FULL_INT_ST)) 
        {
            rt_interrupt_enter();
            rt_hw_serial_isr(&serial0, RT_SERIAL_EVENT_RX_IND);
            rt_interrupt_leave();

            WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_FULL_INT_CLR);
        } 
        else if (UART_RXFIFO_TOUT_INT_ST == (uart_intr_status & UART_RXFIFO_TOUT_INT_ST)) 
        {
            rt_interrupt_enter();
            rt_hw_serial_isr(&serial0, RT_SERIAL_EVENT_RX_IND);
            rt_interrupt_leave();
            
            WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_TOUT_INT_CLR);
        }
        else if (UART_TXFIFO_EMPTY_INT_ST == (uart_intr_status & UART_TXFIFO_EMPTY_INT_ST))
        {
            WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_TXFIFO_EMPTY_INT_CLR);
            CLEAR_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_TXFIFO_EMPTY_INT_ENA);
        }
        else
        {
            //skip
        }

        uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)) ;
    }
}
#endif


rt_err_t uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    UART_ConfigTypeDef uart_config;
    UART_IntrConfTypeDef uart_intr;
    struct esp_uart *uart;

    RT_ASSERT(serial != RT_NULL);

    uart = (struct esp_uart *)serial->parent.user_data;

    uart_config.baud_rate = cfg->baud_rate;
    
    if(cfg->data_bits == DATA_BITS_8)
        uart_config.data_bits = UART_WordLength_8b;
    else if(cfg->data_bits == DATA_BITS_5)
        uart_config.data_bits = UART_WordLength_5b;
    else if(cfg->data_bits == DATA_BITS_6)
        uart_config.data_bits = UART_WordLength_6b;
    else if(cfg->data_bits == DATA_BITS_7)
        uart_config.data_bits = UART_WordLength_7b;

    if(cfg->parity == PARITY_NONE)
        uart_config.parity = USART_Parity_None;
    else if(cfg->parity == PARITY_ODD)
        uart_config.parity = USART_Parity_Odd;
    else if(cfg->parity == PARITY_EVEN)
        uart_config.parity = USART_Parity_Even;

    if(cfg->stop_bits == STOP_BITS_1)
        uart_config.stop_bits = USART_StopBits_1;
    else if(cfg->stop_bits == STOP_BITS_2)
        uart_config.stop_bits = USART_StopBits_2;
    
    uart_config.flow_ctrl = USART_HardwareFlowControl_None;
    uart_config.UART_RxFlowThresh = 120;
    uart_config.UART_InverseMask = UART_None_Inverse;
    
    UART_ParamConfig(uart->device, &uart_config);

    uart_intr.UART_IntrEnMask = UART_RXFIFO_TOUT_INT_ENA | UART_FRM_ERR_INT_ENA | UART_RXFIFO_FULL_INT_ENA | UART_TXFIFO_EMPTY_INT_ENA;
    uart_intr.UART_RX_FifoFullIntrThresh = 10;
    uart_intr.UART_RX_TimeOutIntrThresh = 2;
    uart_intr.UART_TX_FifoEmptyIntrThresh = 20;

    UART_IntrConfig(uart->device, &uart_intr);
}

rt_err_t uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    RT_ASSERT(serial != RT_NULL);

    switch(cmd)
    {
        case RT_DEVICE_CTRL_SET_INT:
            ETS_UART_INTR_ENABLE();
        break;

        case RT_DEVICE_CTRL_CLR_INT:        
            ETS_UART_INTR_ENABLE();
        break;
        
        default:

        break;
    }

    return RT_EOK;
}

int uart_putc(struct rt_serial_device *serial, char c)
{
    struct esp_uart *uart;

    RT_ASSERT(serial != RT_NULL);

    uart = (struct esp_uart *)serial->parent.user_data;
    
    while (true) {
        uint32 fifo_cnt = READ_PERI_REG(UART_STATUS(uart->device)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);

        if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) {
            break;
        }
    }

    WRITE_PERI_REG(UART_FIFO(uart->device) , c);

    return 1;
}

int uart_getc(struct rt_serial_device *serial)
{
    int ch;
    rt_uint8_t fifo_len;
    struct esp_uart *uart;

    RT_ASSERT(serial != RT_NULL);

    uart = (struct esp_uart *)serial->parent.user_data;

    ch = -1;
    fifo_len = (READ_PERI_REG(UART_STATUS(uart->device)) >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;

    if(fifo_len > 0)
    {
//        printf("len:%d\n", fifo_len);
        ch = READ_PERI_REG(UART_FIFO(uart->device)) & 0xFF;
    }
    
    return ch;
}

static const struct rt_uart_ops ops = 
{
//    uart_configure,
    NULL,
    uart_control,
    uart_putc,
    uart_getc,
};

static void esp_uart_intr_config(UART_Port uart)
{
    UART_WaitTxFifoEmpty(UART0);
    UART_WaitTxFifoEmpty(UART1);

    UART_ConfigTypeDef uart_config;
    uart_config.baud_rate    = BIT_RATE_74880;
    uart_config.data_bits     = UART_WordLength_8b;
    uart_config.parity          = USART_Parity_None;
    uart_config.stop_bits     = USART_StopBits_1;
    uart_config.flow_ctrl      = USART_HardwareFlowControl_None;
    uart_config.UART_RxFlowThresh = 120;
    uart_config.UART_InverseMask = UART_None_Inverse;
    UART_ParamConfig(UART0, &uart_config);

    UART_IntrConfTypeDef uart_intr;
    uart_intr.UART_IntrEnMask = UART_RXFIFO_TOUT_INT_ENA | UART_FRM_ERR_INT_ENA | UART_RXFIFO_FULL_INT_ENA | UART_TXFIFO_EMPTY_INT_ENA;
    uart_intr.UART_RX_FifoFullIntrThresh = 10;
    uart_intr.UART_RX_TimeOutIntrThresh = 2;
    uart_intr.UART_TX_FifoEmptyIntrThresh = 20;
    UART_IntrConfig(UART0, &uart_intr);

    UART_SetPrintPort(UART0);
    UART_intr_handler_register(uart0_rx_intr_handler, NULL);
    ETS_UART_INTR_DISABLE();
}

void rt_hw_uart_init(void)
{
    struct esp_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef RT_USING_UART1
    esp_uart_intr_config(UART0);

    uart = &uart0;
    uart->device = UART0;
    
    config.baud_rate = BIT_RATE_74880;

    serial0.ops = &ops;
    serial0.config = config;

    rt_hw_serial_register(&serial0, "uart1", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
#endif

#ifdef RT_USING_UART2
    esp_uart_intr_config(UART0);

    uart = &uart0;
    uart->device = UART0;
    
    config.baud_rate = BIT_RATE_74880;

    serial1.ops = &ops;
    serial1.config = config;

    rt_hw_serial_register(&serial0, "uart2", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
#endif
}

