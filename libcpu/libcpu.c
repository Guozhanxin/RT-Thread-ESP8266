#include "rtthread.h"
#include "rthw.h"
#include "board.h"

/* Scheduler includes. */
#include <xtensa/config/core.h>
#include <xtensa/tie/xt_interrupt.h>
#include <xtensa/tie/xt_timer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_rtos.h"


#define PORT_ASSERT(x) do { if (!(x)) {ets_printf("%s %u\n", "rtos_port", __LINE__); while(1){}; }} while (0)

extern char NMIIrqIsOn;
static char HdlMacSig = 0;
static char SWReq = 0;
static char PendSvIsPosted = 0;

unsigned cpu_sr;

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static unsigned portBASE_TYPE uxCriticalNesting = 0;


portSTACK_TYPE *volatile pxCurrentTCB = NULL;
portSTACK_TYPE *volatile pxSaveTCB = NULL;


/*
 * See header file for description.
 */
portSTACK_TYPE * ICACHE_FLASH_ATTR
pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
	#define SET_STKREG(r,v) sp[(r) >> 2] = (portSTACK_TYPE)(v)
    portSTACK_TYPE *sp, *tp;

    /* Create interrupt stack frame aligned to 16 byte boundary */
    sp = (portSTACK_TYPE*) (((INT32U)(pxTopOfStack + 1) - XT_CP_SIZE - XT_STK_FRMSZ) & ~0xf);

    /* Clear the entire frame (do not use memset() because we don't depend on C library) */
    for (tp = sp; tp <= pxTopOfStack; ++tp)
        *tp = 0;

    /* Explicitly initialize certain saved registers */
    SET_STKREG( XT_STK_PC,      pxCode                        );  /* task entrypoint                  */
    SET_STKREG( XT_STK_A0,      0                           );  /* to terminate GDB backtrace       */
    SET_STKREG( XT_STK_A1,      (INT32U)sp + XT_STK_FRMSZ   );  /* physical top of stack frame      */
    SET_STKREG( XT_STK_A2,      pvParameters   );           /* parameters      */
    SET_STKREG( XT_STK_EXIT,    _xt_user_exit               );  /* user exception exit dispatcher   */

    /* Set initial PS to int level 0, EXCM disabled ('rfe' will enable), user mode. */
    #ifdef __XTENSA_CALL0_ABI__
    SET_STKREG( XT_STK_PS,      PS_UM | PS_EXCM     );
    #else
    /* + for windowed ABI also set WOE and CALLINC (pretend task was 'call4'd). */
    SET_STKREG( XT_STK_PS,      PS_UM | PS_EXCM | PS_WOE | PS_CALLINC(1) );
    #endif

	return sp;
}

void PendSV( char req )
{
	char tmp=0;

	if( NMIIrqIsOn == 0 )
	{
		vPortEnterCritical();
		tmp = 1;
	}

	if(req ==1)
	{
		SWReq = 1;
	}
	else if(req ==2)
		HdlMacSig= 1;

	if(PendSvIsPosted == 0)
	{
		PendSvIsPosted = 1;
		xthal_set_intset(1<<ETS_SOFT_INUM);
	}

	if(tmp == 1)
		vPortExitCritical();
}

extern portBASE_TYPE MacIsrSigPostDefHdl(void);

void SoftIsrHdl(void *arg)
{
	PendSvIsPosted = 0;
	portBASE_TYPE xHigherPriorityTaskWoken=pdFALSE ;
	if(HdlMacSig == 1)
	{
		xHigherPriorityTaskWoken = MacIsrSigPostDefHdl();
		HdlMacSig = 0;
	}
	
	if(xHigherPriorityTaskWoken || (SWReq==1)) 
	{
		_xt_timer_int1();
		SWReq = 0;
	}
}


/*
 * See header file for description.
 */
portBASE_TYPE ICACHE_FLASH_ATTR
xPortStartScheduler( void )
{
	//set pendsv and systemtick as lowest priority ISR.
	//pendsv setting

			/*******software isr*********/
   	_xt_isr_attach(ETS_SOFT_INUM, SoftIsrHdl, NULL);
    _xt_isr_unmask(1<<ETS_SOFT_INUM);

    /* Initialize system tick timer interrupt and schedule the first tick. */
    _xt_tick_timer_init();

    vTaskSwitchContext();

	/* Restore the context of the first task that is going to run. */
	XT_RTOS_INT_EXIT();

	/* Should not get here as the tasks are now running! */
	return pdTRUE;
}

void ICACHE_FLASH_ATTR
vPortEndScheduler( void )
{
	/* It is unlikely that the CM3 port will require this function as there
	is nothing to return to.  */
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

static unsigned int tick_lock=0;
static char ClosedLv1Isr = 0;

void vPortEnterCritical( void )
{
	if(NMIIrqIsOn == 0)
	{	
		if( ClosedLv1Isr !=1 )
		{
			portDISABLE_INTERRUPTS();
			ClosedLv1Isr = 1;
		}
		uxCriticalNesting++;
	}
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
	if(NMIIrqIsOn == 0)
	{
		if(uxCriticalNesting > 0)
		{	
			uxCriticalNesting--;
			if( uxCriticalNesting == 0 )
			{
				if( ClosedLv1Isr ==1 )
				{
					ClosedLv1Isr = 0;
					portENABLE_INTERRUPTS();
				}
			}
		}		
		else
		{
			ets_printf("E:C:%d\n",uxCriticalNesting);
			PORT_ASSERT((uxCriticalNesting>0));
		}
	}
}

void ShowCritical(void)
{
	ets_printf("ShowCritical:%d\n",uxCriticalNesting);
    ets_delay_us(50000);
}


void vPortETSIntrLock( void )
{
	ETS_INTR_LOCK();
}

void vPortETSIntrUnlock( void )
{
	ETS_INTR_UNLOCK();
}

void 
PortDisableInt_NoNest( void )
{
	if(NMIIrqIsOn == 0)	
	{
		if( ClosedLv1Isr !=1 )
		{
			portDISABLE_INTERRUPTS();
			ClosedLv1Isr = 1;
		}
	}
}

void 
PortEnableInt_NoNest( void )
{
	if(NMIIrqIsOn == 0)
	{		
		if( ClosedLv1Isr ==1 )
		{
			ClosedLv1Isr = 0;
			portENABLE_INTERRUPTS();
		}
	}
}

/*-----------------------------------------------------------*/
void ICACHE_FLASH_ATTR ResetCcountVal( unsigned int cnt_val )
{
//  XT_WSR_CCOUNT(cnt_val);
    asm volatile("wsr a2, ccount");
}

_xt_isr_entry isr[16];
char _xt_isr_status = 0;

void ICACHE_FLASH_ATTR
_xt_isr_attach(uint8 i, _xt_isr func, void *arg)
{
    isr[i].handler = func;
    isr[i].arg = arg;
}

uint16 _xt_isr_handler(uint16 i)
{
	uint8 index;

	if (i & (1 << ETS_WDT_INUM)) {
		index = ETS_WDT_INUM;
	} 
	else if (i & (1 << ETS_GPIO_INUM)) {
		index = ETS_GPIO_INUM;
	}else {
		index = __builtin_ffs(i) - 1;

		if (index == ETS_MAX_INUM) {
			i &= ~(1 << ETS_MAX_INUM);
			index = __builtin_ffs(i) - 1;
		}
	}

    _xt_clear_ints(1<<index);

    _xt_isr_status = 1;
    isr[index].handler(isr[index].arg);
    _xt_isr_status = 0;

    return i & ~(1 << index);
}


static void thread_exit_entry(void *parameter)
{
    extern void rt_thread_exit(void);

    rt_thread_t tid;
    void (*entry)(void* parameter);

    tid = rt_thread_self();

    entry = tid->entry;
    entry(parameter);

    /* invoke thread_exit */
    rt_thread_exit();
}


rt_uint8_t *rt_hw_stack_init(void *tentry, void *parameter, rt_uint8_t *stack_addr, void *texit)
{
    rt_uint8_t *ret_sp;

    ret_sp = (rt_uint8_t *)pxPortInitialiseStack((portSTACK_TYPE *)stack_addr, thread_exit_entry, parameter);

    return ret_sp;
}


rt_base_t rt_hw_interrupt_disable(void)
{
    vPortEnterCritical();
    return cpu_sr;
}

void rt_hw_interrupt_enable(rt_base_t level)
{
    cpu_sr = level;
    vPortExitCritical();
}
/*
 * Context interfaces
 */

 void rt_hw_context_switch_to(rt_uint32_t to)
{
    pxSaveTCB = (portSTACK_TYPE *)to;
}

void rt_hw_context_switch(rt_uint32_t from, rt_uint32_t to)
{
    pxSaveTCB = (portSTACK_TYPE *)to;
    portYIELD();
}

void rt_hw_context_switch_interrupt(rt_uint32_t from, rt_uint32_t to)
{
    pxSaveTCB = (portSTACK_TYPE *)to;
    portEND_SWITCHING_ISR(1);
}

#if 0
void rt_hw_context_switch(rt_uint32_t from, rt_uint32_t to)
{
    ets_printf("[FUNC]:%s [LINE]:%d\n", __FUNCTION__, __LINE__);
    pxCurrentTCB = (portSTACK_TYPE *)to;
    PendSV(1);
}

void rt_hw_context_switch_to(rt_uint32_t to)
{
    ets_printf("[FUNC]:%s [LINE]:%d to:%d\n", __FUNCTION__, __LINE__, to);
    pxCurrentTCB = (portSTACK_TYPE *)to;
}
void rt_hw_context_switch_interrupt(rt_uint32_t from, rt_uint32_t to)
{
    ets_printf("[FUNC]:%s [LINE]:%d\n", __FUNCTION__, __LINE__);
    pxCurrentTCB = (portSTACK_TYPE *)to;
    PendSV(1);
}
#endif

#define queueQUEUE_TYPE_BASE				( ( unsigned char ) 0U )
#define queueQUEUE_TYPE_SET					( ( unsigned char ) 0U )
#define queueQUEUE_TYPE_MUTEX 				( ( unsigned char ) 1U )
#define queueQUEUE_TYPE_COUNTING_SEMAPHORE	( ( unsigned char ) 2U )
#define queueQUEUE_TYPE_BINARY_SEMAPHORE	( ( unsigned char ) 3U )
#define queueQUEUE_TYPE_RECURSIVE_MUTEX		( ( unsigned char ) 4U )


extern rt_thread_t rt_current_thread;
static volatile unsigned short tm_index = 0;
static unsigned short mx_index = 0;
static unsigned short mq_index = 0;


signed portBASE_TYPE ICACHE_FLASH_ATTR
xTaskResumeAll( void )
{
    rt_exit_critical();
    return pdTRUE;
}


void ICACHE_FLASH_ATTR
vTaskSuspendAll( void )
{
    rt_enter_critical();
}

portTickType ICACHE_FLASH_ATTR
xTaskGetTickCount( void )
{
    return rt_tick_get();
}


void ICACHE_FLASH_ATTR
vTaskStepTick( portTickType xTicksToJump )
{
    ets_printf("[FUNC]:%s [LINE]:%d\n", __FUNCTION__, __LINE__);

}

portTickType ICACHE_FLASH_ATTR prvGetExpectedIdleTime( void )
{
    ets_printf("[FUNC]:%s [LINE]:%d\n", __FUNCTION__, __LINE__);

}

void ICACHE_FLASH_ATTR
vTaskDelay( portTickType xTicksToDelay )
{
	rt_thread_delay(xTicksToDelay);
}


void ICACHE_FLASH_ATTR
vTaskStartScheduler( void )
{
    /* start scheduler */
    rt_system_scheduler_start();
    
    xPortStartScheduler();
}

void
vTaskSwitchContext( void )
{
    if (pxSaveTCB != NULL)
    {
        pxCurrentTCB = pxSaveTCB;
        pxSaveTCB = NULL;
        return;
    }
    rt_schedule();
}

void ICACHE_FLASH_ATTR
vTaskDelete( xTaskHandle xTaskToDelete )
{
    rt_thread_t thread = xTaskToDelete;

    if(0 == thread)
    {
        thread = rt_current_thread;
    }

#ifdef SHOW_TSK_DEBUG_INFO
    rt_kprintf("TaskDelete cur:%s name:%s\n",
                (rt_current_thread) ? (rt_current_thread->name) : ("NULL"),\
                thread->name);
#endif

    rt_thread_delete(thread);
    rt_schedule();
}

unsigned portBASE_TYPE ICACHE_FLASH_ATTR
uxTaskGetStackHighWaterMark( xTaskHandle xTask )
{
    ets_printf("[FUNC]:%s [LINE]:%d\n", __FUNCTION__, __LINE__);

}

portBASE_TYPE
xTaskIncrementTick( void )
{
    ets_printf("[FUNC]:%s [LINE]:%d\n", __FUNCTION__, __LINE__);

}


typedef void * xQueueHandle;
typedef void (*pdTASK_CODE)( void * );

unsigned portBASE_TYPE
uxQueueMessagesWaitingFromISR( const xQueueHandle xQueue )
{
    ets_printf("[FUNC]:%s [LINE]:%d\n", __FUNCTION__, __LINE__);

}

signed portBASE_TYPE ICACHE_FLASH_ATTR
xTaskGenericCreate( pdTASK_CODE pxTaskCode, const signed char * const pcName, unsigned short usStackDepth, void *pvParameters, unsigned portBASE_TYPE uxPriority, xTaskHandle *pxCreatedTask, portSTACK_TYPE *puxStackBuffer, const xMemoryRegion * const xRegions )
{
    rt_thread_t thread;
    portBASE_TYPE xReturn = errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
    
    uint16_t usStackDepthC = usStackDepth * 4;
    os_printf("pcName:%s uxPriority:%d usStackDepth:%d\n", pcName, uxPriority, usStackDepthC);
    uxPriority = RT_THREAD_PRIORITY_MAX - 5 - uxPriority;
    thread = rt_thread_create(pcName, pxTaskCode, pvParameters, usStackDepthC, uxPriority, 10);

#ifdef SHOW_TSK_DEBUG_INFO    
        rt_kprintf("TaskCreate cur:%s name:%s pri:%d size:%d\n\n",
                    (rt_current_thread) ? (rt_current_thread->name): ("NULL"),
                    pcName, uxPriority, usStackDepthC);
#endif
    if(RT_NULL != thread)
    {
        rt_thread_startup(thread);
        xReturn = pdPASS;
    }
    else
    {
#ifdef SHOW_TSK_DEBUG_INFO    
        rt_kprintf("TaskCreate failed!\n");
#endif
        return xReturn;
    }


    if(pxCreatedTask != RT_NULL)
    {

        *pxCreatedTask = thread;
    }

    return xReturn;
}


extern rt_mailbox_t rt_fmq_create(const char *name, rt_size_t item, rt_size_t size, rt_uint8_t flag);
extern rt_err_t rt_fmq_send(rt_mailbox_t mb, void *value, rt_int32_t pos, rt_int32_t timeout);
extern rt_err_t rt_fmq_delete(rt_mailbox_t mb);
extern rt_err_t rt_fmq_recv(rt_mailbox_t mb, void *value, rt_int32_t peek, rt_int32_t timeout);


xQueueHandle ICACHE_FLASH_ATTR
xQueueGenericCreate( unsigned portBASE_TYPE uxQueueLength, unsigned portBASE_TYPE uxItemSize, unsigned char ucQueueType )
{
   char name[10] = {0};
    rt_object_t obj = 0;

    if(uxItemSize <= 0 || uxQueueLength <= 0)
    {
        if(ucQueueType == queueQUEUE_TYPE_MUTEX)
        {
            sprintf(name, "mux%02d", ((++ mx_index) % 100));
            obj = (rt_object_t)rt_sem_create(name, 1, RT_IPC_FLAG_PRIO);
        }
        else if(ucQueueType == queueQUEUE_TYPE_RECURSIVE_MUTEX)
        {
            sprintf(name, "rmx%02d", ((++ mx_index) % 100));
            obj = (rt_object_t)rt_sem_create(name, 1, RT_IPC_FLAG_PRIO);
        }
        else
        {
            sprintf(name, "sem%02d", ((++ mx_index) % 100));
            obj = (rt_object_t)rt_sem_create(name, 0, RT_IPC_FLAG_PRIO);
        }
    }
    else
    {
        sprintf(name, "fmq%02d", ((++ mq_index) % 100));
        obj = (rt_object_t)rt_fmq_create(name, uxItemSize, uxQueueLength, RT_IPC_FLAG_PRIO);
    }

#ifdef SHOW_QUE_DEBUG_INFO
    rt_kprintf("QueueCreate cur:%s name:%s count:%d size:%d type:%d\n\n",
                (rt_current_thread) ? (rt_current_thread->name) : ("NULL"),
                name, uxQueueLength, uxItemSize, ucQueueType);
#endif

    return obj;
}

signed portBASE_TYPE ICACHE_FLASH_ATTR
xQueueGenericSend( xQueueHandle xQueue, const void * const pvItemToQueue, portTickType xTicksToWait, portBASE_TYPE xCopyPosition )
{
        rt_object_t obj = (rt_object_t)xQueue;
    
#ifdef SHOW_QUE_DEBUG_INFO
        rt_kprintf("QueueSend cur:%s name:%s wait:%d pos:%d\n\n",
                    (rt_current_thread) ? (rt_current_thread->name) :("NULL"),
                    obj->name, xTicksToWait, xCopyPosition);
#endif
    
        rt_err_t err = RT_EOK;
        if(obj->type == RT_Object_Class_Semaphore)
        {
            err = rt_sem_release((rt_sem_t)obj);
        }
        else if(obj->type == RT_Object_Class_Mutex)
        {
            err = rt_mutex_release((rt_mutex_t)obj);
        }
        else
        {
            err = rt_fmq_send((rt_mailbox_t)obj, (void *)pvItemToQueue, xCopyPosition, xTicksToWait);
        }
    
        return (err == RT_EOK) ? pdPASS : errQUEUE_FULL;
}

signed portBASE_TYPE ICACHE_FLASH_ATTR
xQueueGenericReceive( xQueueHandle xQueue, const void * const pvBuffer, portTickType xTicksToWait, portBASE_TYPE xJustPeeking )
{
        rt_object_t obj = (rt_object_t)xQueue;
        
#ifdef SHOW_QUE_DEBUG_INFO
        rt_kprintf("QueueReceive cur:%s name:%s wait:%x\n\n",
                    (rt_current_thread) ? (rt_current_thread->name) :("NULL"),
                    obj->name, xTicksToWait);
#endif
    
        rt_err_t err = RT_EOK;
        if(obj->type == RT_Object_Class_Semaphore)
        {
            err = rt_sem_take((rt_sem_t)obj, xTicksToWait);
        }
        else if(obj->type == RT_Object_Class_Mutex)
        {
            err = rt_mutex_take((rt_mutex_t)obj, xTicksToWait);
        }
        else
        {
            err = rt_fmq_recv((rt_mailbox_t)obj, (void *)pvBuffer, xJustPeeking, xTicksToWait);
        }
    
        return (err == RT_EOK) ? pdPASS : errQUEUE_EMPTY;

}

signed portBASE_TYPE ICACHE_FLASH_ATTR
xQueueGenericSendFromISR( xQueueHandle xQueue, const void * const pvItemToQueue, signed portBASE_TYPE *pxHigherPriorityTaskWoken, portBASE_TYPE xCopyPosition )
{
    rt_object_t obj = (rt_object_t)xQueue;
    
    rt_interrupt_enter();

    rt_err_t err = RT_EOK;
    if(obj->type == RT_Object_Class_Semaphore)
    {
        err = rt_sem_release((rt_sem_t)obj);
    }
    else
    {
        err = rt_fmq_send((rt_mailbox_t)obj, (void *)pvItemToQueue, xCopyPosition, 0);
    }
    
    if(pxHigherPriorityTaskWoken)
    {
        *pxHigherPriorityTaskWoken = pdFALSE;
    }
    
    rt_interrupt_leave();
    return (err == RT_EOK) ? pdTRUE : errQUEUE_FULL;

}

typedef void * xTimerHandle;

/* Define the prototype to which timer callback functions must conform. */
typedef void (*tmrTIMER_CALLBACK)( xTimerHandle xTimer );

xTimerHandle ICACHE_FLASH_ATTR
xTimerCreate( const signed char * const pcTimerName, portTickType xTimerPeriodInTicks, unsigned portBASE_TYPE uxAutoReload, void *pvTimerID, tmrTIMER_CALLBACK pxCallbackFunction )
{
    ets_printf("[FUNC]:%s [LINE]:%d\n", __FUNCTION__, __LINE__);

}

portBASE_TYPE ICACHE_FLASH_ATTR
xTimerGenericCommand( xTimerHandle xTimer, portBASE_TYPE xCommandID, portTickType xOptionalValue, signed portBASE_TYPE *pxHigherPriorityTaskWoken, portTickType xBlockTime )
{
    ets_printf("[FUNC]:%s [LINE]:%d\n", __FUNCTION__, __LINE__);

}

static unsigned short heapSTRUCT_SIZE;//  = ( ( sizeof ( xBlockLink ) + ( portBYTE_ALIGNMENT - 1 ) ) & ~portBYTE_ALIGNMENT_MASK );

static unsigned short portBYTE_ALIGNMENT_MASK_v;
static unsigned short portBYTE_ALIGNMENT_v;

size_t xPortWantedSizeAlign(size_t xWantedSize)
{
    heapSTRUCT_SIZE = 8;
    portBYTE_ALIGNMENT_MASK_v = 7;
    portBYTE_ALIGNMENT_v = 8;

	xWantedSize += heapSTRUCT_SIZE;

	/* Ensure that blocks are always aligned to the required number
	of bytes. */
	if( ( xWantedSize & portBYTE_ALIGNMENT_MASK_v ) != 0x00 )
	{
		/* Byte alignment required. */
		xWantedSize += ( portBYTE_ALIGNMENT_v - ( xWantedSize & portBYTE_ALIGNMENT_MASK_v ) );
	}

//    ets_printf("heapSTRUCT_SIZE:%d portBYTE_ALIGNMENT_MASK_v:%d portBYTE_ALIGNMENT_v:%d", heapSTRUCT_SIZE, portBYTE_ALIGNMENT_MASK_v, portBYTE_ALIGNMENT_v);
	return xWantedSize;
}

size_t ICACHE_FLASH_ATTR
xPortGetFreeHeapSize( void )
{
//	return xFreeBytesRemaining;
   rt_uint32_t total, used, max_used;
    
    rt_memory_info(&total, &used, &max_used);

 //   ets_printf("total:%d used:%d free:%d\n", total, used, (total - used));
	return (total - used);
}

int rt_application_init(void)
{

    return 0;
}

/**
 * This function will startup RT-Thread RTOS.
 */
void rtthread_startup(void)
{
	/* init board */
//    rt_hw_board_init();

	/* show version */
	rt_show_version();

	/* init tick */
	rt_system_tick_init();

	/* init kernel object */
	rt_system_object_init();

	/* init timer system */
	rt_system_timer_init();

#ifdef RT_USING_HEAP
    rt_system_heap_init((void*)&_heap_start, (void*)ESP8266_SRAM_END);
#endif

	/* init scheduler system */
	rt_system_scheduler_init();

#ifdef RT_USING_SIGNALS
    /* signal system initialization */
    rt_system_signal_init();
#endif
    
    /* create init_thread */
    rt_application_init();

    /* init timer thread */
    rt_system_timer_thread_init();

	/* init idle thread */
	rt_thread_idle_init();

	return ;
}



void *pvPortMalloc( size_t xWantedSize )
{
    void *pvReturn = NULL;
    static rt_uint8_t rt_init = 0;
    

//	ETS_INTR_LOCK();

    if(rt_init == 0)
    {
        rtthread_startup();
        rt_init = 1;
    }
        
    pvReturn = rt_malloc(xWantedSize);
    
//	ETS_INTR_UNLOCK();
	
    return pvReturn;
}


void vPortFree( void *pv )
{
    rt_free(pv);
}

void *pvPortCalloc(size_t count, size_t size)
{
  void *p;

  p = rt_malloc(count * size);
  if (p) {
    memset(p, 0, count * size);
  }
  return p;
}

void *pvPortZalloc(size_t size)
{
     return pvPortCalloc(1, size);	
}

void *zalloc(size_t nbytes) __attribute__((alias("pvPortZalloc")));




