/******************************************************************************
 Copyright (C) 2011  R&D Institute of HuaCai Co.,Ltd.
 Module         : main function head file
 File Name      : main.h
 Description    :
 Author         :
 Build Date     :
 others         :
 ------------------------------------------------------------------------------
 Modification History:
 <No.>  <version>       <time>      <author>        <contents>
   2、
   1、     1.00       2011-05-09     azh       create
******************************************************************************/

#ifndef __MAIN_H
#define __MAIN_H

//#define	PD_MANAGE_VER12	//掉电管理V1.2(4.8V)以下和V1.3(2.4V)不同的

/*----------------------------- macro definition ----------------------------*/
#define VER_MAIN            0x02                        // 主版本号
#define VER_PARTICULAR      0x01                        // 次版本号
#define ISSUE_YMD_V         0x120413                    // 软件发行年月日

#define MC55_GPRS_MODULE    0xa1
#define TELIT_GPRS_MODULE   0xa2

#if CODE_VER_TYPE==GPRS_MODULE_PRJ		//带CPU_GPRS模块
#define     INIT_TASK_STACK_SIZE            (200 / 8)
//#define     CRAWL_TASK_STACK_SIZE           (400 / 8)
#define     PROTOCOL_TASK_STACK_SIZE        (1024 / 8)
#define     TRANSMIT_TASK_STACK_SIZE        (400 / 8)
#define     MANAGE_TASK_STACK_SIZE          (200 / 8)
#define     TIMING_TASK_STACK_SIZE          (400 / 8)//(400 / 8)
#define     UART_TASK_STACK_SIZE            (300 / 8)//(400 / 8)
#define     TCP_POLL_STACK_SIZE             (200 / 8)
//#define     TICK_TIMER_TASK_STACK_SIZE      (200 / 8)
#define     PROTOCOL_STATUS_STACK_SIZE      (200 / 8)
//#define     SNTP_TASK_STACK_SIZE            (200 / 8)
//AZH
#define     LCD_DISPLAY_TASK_STACK_SIZE     (320 / 8)//(200 / 8)//(400 / 8)
#define     GPRS_TASK_STACK_SIZE            (1024 / 8)//(800 / 8)
#else
#define     INIT_TASK_STACK_SIZE            (400 / 8)
//#define     CRAWL_TASK_STACK_SIZE           (400 / 8)
#define     PROTOCOL_TASK_STACK_SIZE        (700 / 8)
#define     TRANSMIT_TASK_STACK_SIZE        (400 / 8)
#define     MANAGE_TASK_STACK_SIZE          (200 / 8)
#define     TIMING_TASK_STACK_SIZE          (400 / 8)//(400 / 8)
#define     UART_TASK_STACK_SIZE            (400 / 8)
#define     TCP_POLL_STACK_SIZE             (200 / 8)
//#define     TICK_TIMER_TASK_STACK_SIZE      (200 / 8)
#define     PROTOCOL_STATUS_STACK_SIZE      (200 / 8)
//#define     SNTP_TASK_STACK_SIZE            (200 / 8)
//AZH
#define     LCD_DISPLAY_TASK_STACK_SIZE     (400 / 8)//(200 / 8)//(400 / 8)
#define     GPRS_TASK_STACK_SIZE            (800 / 8)
#endif



#if CODE_VER_TYPE==H_METER_PRJ					//配合高压表
#define     UART0_COMM_PARA                 (UART_8BIT_DATA | UART_EVEN_PARITY | UART_1_STOP_BIT)
	#define     UART0_BAUD_RATE                 9600/////2400    //485_1

#define     UART1_COMM_PARA                 (UART_8BIT_DATA | UART_NO_PARITY | UART_1_STOP_BIT)
	#define     UART1_BAUD_RATE                 115200  //zigbee

	#define     UART2_COMM_PARA                 (UART_8BIT_DATA | UART_EVEN_PARITY | UART_1_STOP_BIT)
	#define     UART2_BAUD_RATE                 9600    //485_2
#elif CODE_VER_TYPE==WLS_CUR_PRJ				//配合无线采集器
	#define     UART0_COMM_PARA                 (UART_8BIT_DATA | UART_EVEN_PARITY | UART_1_STOP_BIT)
	#define     UART0_BAUD_RATE                 9600/////2400    //485_1

	#define     UART1_COMM_PARA                 (UART_8BIT_DATA | UART_NO_PARITY | UART_1_STOP_BIT)//
	#define     UART1_BAUD_RATE                 2400//2400//115200  //zigbee -> internal com modle

	#define     UART2_COMM_PARA                 (UART_8BIT_DATA | UART_EVEN_PARITY | UART_1_STOP_BIT)
	#define     UART2_BAUD_RATE                 9600    //485_2

#elif CODE_VER_TYPE==GPRS_MODULE_PRJ			//配合带CPU_GPRS模块
	#define     UART0_COMM_PARA                 (UART_8BIT_DATA | UART_EVEN_PARITY | UART_1_STOP_BIT)
	#define     UART0_BAUD_RATE                 1200/////红外
	#ifdef ZIGBEE_TEST
		#define     UART1_COMM_PARA                 (UART_8BIT_DATA | UART_NO_PARITY | UART_1_STOP_BIT)//
		#define     UART1_BAUD_RATE                 115200  //zigbee
	#else
		#define     UART1_COMM_PARA                 (UART_8BIT_DATA | UART_EVEN_PARITY | UART_1_STOP_BIT)//
		#define     UART1_BAUD_RATE                 9600//2400//115200  //zigbee -> internal com modle
	#endif//ZIGBEE_TEST	
	#define     UART2_COMM_PARA                 (UART_8BIT_DATA | UART_EVEN_PARITY | UART_1_STOP_BIT)
	#define     UART2_BAUD_RATE                 9600    //485_2
#endif

//#define     UART2_COMM_PARA                 (UART_8BIT_DATA | UART_EVEN_PARITY | UART_1_STOP_BIT)
//#define     UART2_BAUD_RATE                 9600    //485_2
#define     UART3_COMM_PARA                 (UART_8BIT_DATA | UART_NO_PARITY | UART_1_STOP_BIT) //GPRS
#define     UART3_BAUD_RATE                 57600//115200  //GPRS

#define     UART0_SRC_DLY                   OS_DLY_1S       //485_1
#define     UART1_SRC_DLY                   OS_DLY_500MS    //zigbee
#define     UART2_SRC_DLY                   OS_DLY_1S       //485_2
#define     UART3_SRC_DLY                   OS_DLY_500MS    //GPRS

#define     LED_FLASH_TIMES                 4           //LED闪烁次数

#define     SOFTWARE_RESET_DELAY            100         // 软件复位等待延迟时间, 单位0.1S

#define     TIME_OUT_FLAG                   (1<<0)      // 任务事件标志
#define     POWER_DOWN_FLAG                 (1<<1)
#define     LOAD_PROFILE_FLAG               (1<<2)
#define     TARIFF_UPDATE_FLAG              (1<<3)
#define     SI_RECORD_FLAG                  (1<<4)
#define     EXTCMD_RST_FLAG                 (1<<5)      //外部复位命令 azh

#define     ETHERNET_LINK_FLAG              (1<<0)      // 状态标志
#define     HOST_CONNECT_FLAG               (1<<1)
#define     METER_CONNECT_FLAG              (1<<2)
#define     SNTP_SUCCESS_FLAG               (1<<3)

#define     RATE_1_FLAG                     (1<<8)
#define     RATE_2_FLAG                     (1<<9)

#define     TMP_RAM_ARRAY_MAX               128//255
//显示终端一些全局状态标志比如编程及错误
#define     CON_LOCAL_STATE_PROG        (1<<0)//编程状态
#define     CON_GPRS_LED_STATE          (1<<1)//gprs模块灯0.5秒闪烁(在有信号又没有在线的情况)

#define     CON_ZIGBEE_DISABLE          0xA55A
//
#define     LOCAL_STATE_IS_HIG(index)   (gLocal_State_Flag & index)
#define     SET_LOCAL_STATE(index)      (gLocal_State_Flag |= index)
#define     CLR_LOCAL_STATE(index)      (gLocal_State_Flag &= ~index)
#define     REV_LOCAL_STATE(index)      if(LOCAL_STATE_IS_HIG(index))   \
                                        {                               \
                                            LOCAL_STATE_IS_LOW(index);  \
                                        }                               \
                                        else                            \
                                        {                               \
                                            LOCAL_STATE_IS_HIG(index);  \
                                        }
/*----------------------------- type definition -----------------------------*/
typedef enum
{
    INVALID_PRIO,
    MANAGE_TASK_PRIO,               //优先级最低 但是看门狗是放在这的
    TCP_POLL_TASK_PRIO,             //必须最低优先级
    LCD_DISPLAY_PRIO,//AZH
//    MANAGE_TASK_PRIO,
//    CRAWL_TASK_PRIO,
//    SNTP_TASK_PRIO,
//    PROTOCOL_TASK_PRIO,//azh 有些命令 比如任务在执行时 通信不能打断
    GPRS_TASK_PRIO,
    TRANSMIT_TASK_PRIO,
	PROTOCOL_TASK_PRIO,
    PROTOCOL_STATUS_TASK_PRIO,
    UART_TASK_PRIO,
    TIMING_TASK_PRIO,
    INIT_TASK_PRIO,                 //必须次最高优先级
//    TICK_TIMER_TASK_PRIO            //必须最高优先级
}TASK_PRIO;

typedef struct
{
    U16     Report_Cycle;
    U8      Report_Num;
}REPORT_PARA_STRUCT;

/*-------------------------- functions declaration --------------------------*/
__EXTERN    __task  void    init_task(void);
__EXTERN            void    sign_task_schedule(U8 prio);
__EXTERN    __irq   void    WDT_IRQ_handler(void);
__EXTERN    void enable_modules_int(void);
__EXTERN    void disable_modules_int(void);

/*--------------------------- variable declaration --------------------------*/
//--------------------------- variable declaration ----------------------------
#if CODE_VER_TYPE!=GPRS_MODULE_PRJ		//不是CPU_GPRS模块
__EXTERN U16 Eng_Up_Flg;       //电量数据块更新标识，用于判别是否更新电量结算日指针
//bit[0:10]:组合有功电能数据块~反向视在电能数据块(按DI2排序)
//bit[11:12]:合分相关联电能量数据块~分相电能数据块[厂家扩展]
__EXTERN U16 Dmd_Up_Flg;       //需量数据块更新标识，用于判别是否更新需量结算日指针
//bit[0:9]:正向有功最大需量及发生时间数据块~反向视在最大需量及发生时间数据块(按DI2排序)
//bit[10:12]:分相A/B/C最大需量及发生时间数据块[厂家扩展]
#endif
__EXTERN    U64     Init_Task_Stack[INIT_TASK_STACK_SIZE];
//__EXTERN    U64     Crawl_Task_Stack[CRAWL_TASK_STACK_SIZE];
#ifdef LCD_VALID
__EXTERN    U64     Lcd_Display_Task_Stack[LCD_DISPLAY_TASK_STACK_SIZE];//AZH
#endif
//__EXTERN    U64     Transmit_Task_Stack[TRANSMIT_TASK_STACK_SIZE];//AZH
__EXTERN    U64     Protocol_Task_Stack[PROTOCOL_TASK_STACK_SIZE];
//__EXTERN    U64     Manage_Task_Stack[MANAGE_TASK_STACK_SIZE];
__EXTERN    U64     Timing_Task_Stack[TIMING_TASK_STACK_SIZE];
__EXTERN    U64     UART_Task_Stack[UART_TASK_STACK_SIZE];
__EXTERN    U64     Gprs_Task_Stack[GPRS_TASK_STACK_SIZE];
//__EXTERN    U64     TCP_Poll_Stack[TCP_POLL_STACK_SIZE];
//__EXTERN    U64     Tick_Timer_Task_Stack[TICK_TIMER_TASK_STACK_SIZE];
//__EXTERN    U64     Protocol_Status_Stack[PROTOCOL_STATUS_STACK_SIZE];
//__EXTERN    U64     SNTP_Task_Stack[SNTP_TASK_STACK_SIZE];

//__EXTERN    OS_TID  Crawl_Task_ID;
__EXTERN    OS_TID  Protocol_Analyse_Task_ID;
__EXTERN    OS_TID  System_Manage_Task_ID;
__EXTERN    OS_TID  Timing_Task_ID;
__EXTERN    OS_TID  UART_Task_ID;
__EXTERN    OS_TID  TCP_Poll_Task_ID;
//__EXTERN    OS_TID  Tick_Timer_Task_ID;
//__EXTERN    OS_TID  Protocol_Status_Task_ID;
//__EXTERN    OS_TID  SNTP_Task_ID;

//AZH
#ifdef LCD_VALID
__EXTERN    OS_TID  Lcd_Display_Task_ID;
#endif
__EXTERN    OS_TID  Transmit_Analyse_Task_ID;
__EXTERN    OS_TID  Gprs_Task_ID;

__EXTERN    U32     Require_Software_Reset;
__EXTERN    U8      Power_Down_Flag;
__EXTERN    U32     Task_Schedule_Flag;
__EXTERN    U32     Task_Schedule_Record;
//__EXTERN    U16     Global_Status_Word;

//__EXTERN    U16     Meter_Status_Word[7];          //电表运行状态字，按DL/T645-2007

__EXTERN    REPORT_PARA_STRUCT      Report_Para;

//__EXTERN    os_mbx_declare(Mbx_Load_Profiler, 2);               //mailbox for receive load profiler message
__EXTERN    os_mbx_declare(Mbx_Comm_Data, 16);                  //mailbox for receive communicated data
__EXTERN    os_mbx_declare(Mbx_UART_Data, 256);//azh 120312 512 //mailbox for receive UART data
//__EXTERN    os_mbx_declare(Mbx_UART_Data, 256);               //mailbox for receive UART data
#if CODE_VER_TYPE!=GPRS_MODULE_PRJ		//不是CPU_GPRS模块
//__EXTERN    os_mbx_declare(Mbx_Transmit_Data, 8);//16);         //mailbox for transmit data
#endif
//-------------------------- functions declaration ----------------------------
#define ENABLE_ISR(channel, prio)   ((VU8 *)0xE000E400)[channel - 16] = prio; \
                                    ((VU32 *)0xE000E100)[(channel - 16) / 32] = 1ul << ((channel - 16) % 32)     //设置中断优先级，使能中断，仅用于外设，即通道号16开始的中断
#define DISABLE_ISR(channel)        ((VU32 *)0xE000E180)[(channel - 16) / 32] = 1ul << ((channel - 16) % 32)     //禁止中断，仅用于外设，即通道号16开始的中断


//--------------------------- variable declaration ----------------------------
__EXTERN U16 Eng_Up_Flg;       //电量数据块更新标识，用于判别是否更新电量结算日指针
//bit[0:10]:组合有功电能数据块~反向视在电能数据块(按DI2排序)
//bit[11:12]:合分相关联电能量数据块~分相电能数据块[厂家扩展]
__EXTERN U16 Dmd_Up_Flg;       //需量数据块更新标识，用于判别是否更新需量结算日指针
//bit[0:9]:正向有功最大需量及发生时间数据块~反向视在最大需量及发生时间数据块(按DI2排序)
//bit[10:12]:分相A/B/C最大需量及发生时间数据块[厂家扩展]
__EXTERN U8 gucPoweronGetTimeOver;//上电后已经同步到高压表的时间数据
__EXTERN U8 gucPoweron;
__EXTERN U8 Reset_Source;
__EXTERN U8 Reset_Source_Copy;
__EXTERN U8 gucGetZigbeeAddrFlag;//远程有设置高压表地址时 标志第一帧去抄表用广播 然后回应帧中获取Zigbee地址并进行保存
__EXTERN U8 gMatch_Zigbee_Addr[PAR_HVMETER_ZIGBEE_ADR_LEN+1];   //搜索到的或者设置的匹配高压表的zigbee模块的mac地址
__EXTERN U8 gMatch_HVMeter_Addr[PAR_HVMETER_ADR_LEN+1];         //高压表地址
__EXTERN U8 gLocal_Disply_Addr[6];          //本地显示终端地址
__EXTERN U8 gTmpArray[TMP_RAM_ARRAY_MAX];     //临时显示某些参变量或某条事件记录 所在RAM全局数组
__EXTERN U32 gVarArray[VAR_RAM_ARRAY_MAX];     //瞬时量 所在RAM全局数组
__EXTERN U8 gEngArray[ENG_RAM_ARRAY_MAX];   //当前电量 所在RAM全局数组,存储顺序与外存一致
__EXTERN U8 gDmdArray[DMD_RAM_ARRAY_MAX];   //当前需量 所在RAM全局数组,存储顺序与外存一致
__EXTERN U16 gMeter_State[7];               //电表状态字
__EXTERN U16 gLocal_State_Flag;             //显示终端一些全局状态标志比如编程及错误
__EXTERN U8  Meter_LED_Cnt[MAX_PORT_NUM];//每个串口一个灯次数控制

__EXTERN unsigned short gucAtCmd_WatchOut_Flag;//AT监控命令输出使能标志
__EXTERN unsigned short gucZigbee_Disable_Flag;//ZIGBEE使能标志

__EXTERN unsigned char  gucGprs_Module_Type;//azh 141030 
__EXTERN unsigned char  gMatch_aim_addr[PAR_HVMETER_ZIGBEE_ADR_LEN+1];//azh 171011 电子式高压表GPRS表上行规约被动模式下返回地址
//-----------------------------------------------------------------------------

#endif//__MAIN_H
/******************************************************************************
                            End Of File
 *****************************************************************************/
