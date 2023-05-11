	
/**--------------------------------------------------------------------------------------------------------
** Created by:			zjjohn
** Created date:		2022-08-20
** Version:			    1.0
** Descriptions:		SHT30温度、湿度传感器（单路）			
**********************************************************************************************************/

#include  "config.h"
#include	"uart.h"
#include "sht30.h"
#include	"wdt.h" 
#include 	"receservice.h" 
#include "flashrom.h"


/***************************************************************************
													全局变量定义

 **************************************************************************/
extern uint8 dipkey;							//dip8开关的状态，需要初始化，并实时监测
extern uint8 loarkey;
extern uint8 lora_add, lora_netID, lora_channel;
//uint8 CHNUM = 8;									//温度传感器的路数1路或8路

extern void lora_config_init(void);				//设置lora的参数

/***************************************************************************
 * 描  述 : 主函数
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************/
void  main()
{		 
	uint8 autoTxCount = 0;
	uint8 i=4,j;
	
	IO_init();											//IO口开机初始化、LED测试	
	dipkey_lora_init();							//获取LORA设置在DIP开关信息，0-3位信道（23-48），4、5位网络ID，6、7位模块地址

	//uart1_rece_ack();		//调试用  向主机回送测温信息；
	//uart1_auto_tx();		//调试用  自动回传	
	//EEPROM_Init();			//调试用  
		
	if(dipkey_init())	EEPROM_Init();	
	Timer0Init();	   			//初始化定时器0,作串口帧结束的定时器，３.５个接收baut时间长度
	Uart1_Init();         //串口1初始化，缺省９６００
	SHT3X_Init();
	EA = 1;               //总中断打开	

	SendStringByUart1("Ver:1.0B 20220902"), delay_ms(1000);							//无线通讯启动，需要延时

	SendDataByUart1(BaudAddr[0]),SendDataByUart1(BaudAddr[1]),SendDataByUart1(BaudAddr[2]);	//显示初始工作参数
	SendDataByUart1(lora_channel), SendDataByUart1(lora_add), SendDataByUart1(lora_netID);	//串口打印信道频率号，地址，网络ID
	
	if(lora_channel != 1) lora_config_init();				//设置lora的参数，如果模块地址为0，则不作设置，设置标志为1来判断

	WDT_config();           //看门狗初始化  T   12M频率下9 秒
	SHT3X_Get(); 						//保证第一次获取数据
	while(1)
	{
/***********************同时获取1路温度和1路湿度*********500毫秒更新*********************/			
		if(i++==100)
		{
				SHT3X_Get();  								//SHT30函数 获取温度、湿度 100*5=500MS
				i=0;
				LED_R = ~LED_R;  
		}
		
		uart1_rece_proc();								//查询是否有轮询命令收到？
		delay_ms(5);     									//5ms间隔查询中断
			
		if((BaudAddr[2]) && (j++==200))		//如果不为0，则以此为间隔周期，自动发送,同时步进速度1秒
																			//5ms X 200 = 1000ms
		{
			j=0;
			if(autoTxCount++ == BaudAddr[2])	
			{
				uart1_auto_tx();							//自动发送温度和湿度数据
				autoTxCount = 0; 
			}
		}
		
/***********************DIP开关变化，动态适应*****************************/		
		if(dipkey != ~P2)						
		{
				if((P2 & 0x1f) == 0x1f)	EEPROM_Factory();				//DIP开关地址为0，不起作用，出厂设置
				IAP_CONTR = 0x20;									//重启动     //检测如果开关变化，重启动，初始化
		}
/*******************LORA DIP开关变化，动态适应*****************************/		
		if(loarkey != ~P0)						
		{			
				IAP_CONTR = 0x20;									//重启动     //检测如果开关变化，重启动，初始化
		}		
						
		Wdt_Feed();     											//喂狗9秒之内需要喂狗
	}	
}  


/******************************* END*****************************************/






