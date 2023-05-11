
//	本程序是STC系列的内置flash读写程序。
#include "flashrom.h"
#include	"uart.h"
#include "config.h"

u8 BaudAddr[3] = {BaudFLAG, DefaultAddress, AutoTxTime};		  		//实际通讯串口1，485接口缺省  速率标志、地址、是否自动回传（1：自动回传）
u8 EEPROMTemp[3] = {BaudFLAG, DefaultAddress, AutoTxTime};				//通讯串口1，485接口缺省  速率标志、地址、是否自动回传（1：自动回传）

u16 Addr_Baud = MOVC_ShiftAddress;    				//eeprom 第1个扇区（0开始）
u16 Addr_485 = 0x0200 +MOVC_ShiftAddress;    	//记住 一定要把偏移量加上，第2个扇区（512开始）
u16 Addr_autoT = 0x0400 +MOVC_ShiftAddress;    //记住 一定要把偏移量加上,eeprom 第3个扇区（1K开始）

uint8 dipkey;
uint8 loarkey;
uint8 lora_add, lora_netID, lora_channel;

//========================================================================
// 函数: void	ISP_Disable(void)
// 描述: 禁止访问ISP/IAP.
// 参数: non.
// 返回: non.
//========================================================================
void	DisableEEPROM(void)
{
	ISP_CONTR = 0;			//禁止ISP/IAP操作
	ISP_CMD   = 0;			//去除ISP/IAP命令
	ISP_TRIG  = 0;			//防止ISP/IAP命令误触发
	ISP_ADDRH = 0xff;		//清0地址高字节
	ISP_ADDRL = 0xff;		//清0地址低字节，指向非EEPROM区，防止误操作
}

//========================================================================
// 函数: void EEPROM_read_n(u16 EE_address,u8 *DataAddress,u16 number)
// 描述: 从指定EEPROM首地址读出n个字节放指定的缓冲.
// 参数: EE_address:  读出EEPROM的首地址.
//       DataAddress: 读出数据放缓冲的首地址.
//       number:      读出的字节长度.
// 返回: non.
//========================================================================
void EEPROM_read_n(u16 EE_address,u8 *DataAddress,u16 number)
{
	EA = 0;		//禁止中断
	ISP_CONTR = (ISP_EN + ISP_WAIT_FREQUENCY);	//设置等待时间，允许ISP/IAP操作，送一次就够
	ISP_READ();									//送字节读命令，命令不需改变时，不需重新送命令
	do
	{
		ISP_ADDRH = EE_address / 256;		//送地址高字节（地址需要改变时才需重新送地址）
		ISP_ADDRL = EE_address % 256;		//送地址低字节
		ISP_TRIG();							//先送5AH，再送A5H到ISP/IAP触发寄存器，每次都需要如此
											//送完A5H后，ISP/IAP命令立即被触发启动
											//CPU等待IAP完成后，才会继续执行程序。
		_nop_();
		*DataAddress = ISP_DATA;			//读出的数据送往
		EE_address++;
		DataAddress++;
	}while(--number);

	DisableEEPROM();
	EA = 1;		//重新允许中断
}


/******************** 扇区擦除函数 *****************/
//========================================================================
// 函数: void EEPROM_SectorErase(u16 EE_address)
// 描述: 把指定地址的EEPROM扇区擦除.
// 参数: EE_address:  要擦除的扇区EEPROM的地址.
// 返回: non.
//========================================================================
void EEPROM_SectorErase(u16 EE_address)
{
	EA = 0;		//禁止中断
											//只有扇区擦除，没有字节擦除，512字节/扇区。
											//扇区中任意一个字节地址都是扇区地址。
	ISP_CONTR = (ISP_EN + ISP_WAIT_FREQUENCY);	//设置等待时间，允许ISP/IAP操作，送一次就够
	ISP_ADDRH = EE_address / 256;			//送扇区地址高字节（地址需要改变时才需重新送地址）
	ISP_ADDRL = EE_address % 256;			//送扇区地址低字节
	ISP_CONTR = (ISP_EN + ISP_WAIT_FREQUENCY);	//设置等待时间，允许ISP/IAP操作，送一次就够
	ISP_ERASE();							//送扇区擦除命令，命令不需改变时，不需重新送命令
	ISP_TRIG();
	_nop_();
	DisableEEPROM();
	EA = 1;		//重新允许中断
}

//========================================================================
// 函数: void EEPROM_write_n(u16 EE_address,u8 *DataAddress,u16 number)
// 描述: 把缓冲的n个字节写入指定首地址的EEPROM.
// 参数: EE_address:  写入EEPROM的首地址.
//       DataAddress: 写入源数据的缓冲的首地址.
//       number:      写入的字节长度.
// 返回: non.
//========================================================================
void EEPROM_write_n(u16 EE_address,u8 *DataAddress,u16 number)
{
	EA = 0;		//禁止中断

	ISP_CONTR = (ISP_EN + ISP_WAIT_FREQUENCY);	//设置等待时间，允许ISP/IAP操作，送一次就够
	ISP_WRITE();							//送字节写命令，命令不需改变时，不需重新送命令
	do
	{
		ISP_ADDRH = EE_address / 256;		//送地址高字节（地址需要改变时才需重新送地址）
		ISP_ADDRL = EE_address % 256;		//送地址低字节
		ISP_DATA  = *DataAddress;			//送数据到ISP_DATA，只有数据改变时才需重新送
		ISP_TRIG();
		_nop_();
		EE_address++;
		DataAddress++;
	}while(--number);

	DisableEEPROM();
	EA = 1;		//重新允许中断
}

//========================================================================
// 函数: void EEPROM_Init()
// 初始化
// 读取EEPROM保存的通信速率标志，从机地址，如果没有读到，就用缺省值，定义在EEPROMTemp
// 返回: non.
//========================================================================

void EEPROM_Init(void)
{
	unsigned char strTemp[8];
	
	EEPROM_read_n(Addr_Baud,BaudAddr,1);	//在FLASH的首地址为0xE000处读取个字节存入数组中
	if(*BaudAddr == 0xff)	
		{
				EEPROM_SectorErase(Addr_Baud);    	   //对FLASH的首地址为0xE000处的扇区进行扇区擦除
				EEPROM_write_n(Addr_Baud,EEPROMTemp,1);		//在FLASH的首地址为写入1个字节
		}
		
	EEPROM_read_n(Addr_485,BaudAddr+1,1);	//在FLASH的首地址为0xE000处读取1个字节存入数组中		
	if(*(BaudAddr+1) == 0xff)	
	{
			EEPROM_SectorErase(Addr_485);    	   //对FLASH的首地址为0x????+200处的扇区进行扇区擦除
			EEPROM_write_n(Addr_485,EEPROMTemp+1,1);		//在FLASH的首地址为写入1个字节
	}	
	
	EEPROM_read_n(Addr_autoT,BaudAddr+2,1);		//在FLASH的首地址为0x????处读取1个字节存入数组中
	if(*(BaudAddr+2) == 0xff)	
	{
			EEPROM_SectorErase(Addr_autoT);    	   //对FLASH的首地址为0x????+400处的扇区进行扇区擦除
			EEPROM_write_n(Addr_autoT,EEPROMTemp+2,1);		//在FLASH的首地址为写入1个字节
	}
	
			EEPROM_read_n(Addr_Baud,BaudAddr,1);	//在FLASH的首地址为0xE000处读取1个字节存入数组中
			EEPROM_read_n(Addr_485,BaudAddr+1,1);	//在FLASH的首地址为0xE000+400 处读取1个字节存入数组中
			EEPROM_read_n(Addr_autoT,BaudAddr+2,1);	//在FLASH的首地址为0xE000+400 处读取1个字节存入数组中
		
			sprintf(strTemp, "%c%c%c\r\n", BaudAddr[0],BaudAddr[1],BaudAddr[2]); //浮点数转成字符串 
			SendStringByUart1(strTemp); 
}

//========================================================================
// 函数: void EEPROM_Factory()
// 恢复出厂设置
// 读取EEPROM保存的通信速率标志，从机地址，如果没有读到，就用缺省值，定义在EEPROMTemp
// 返回: non.
//========================================================================

void EEPROM_Factory(void)
{				
		EEPROM_write_1(Addr_Baud, EEPROMTemp);	//BaudAddr[0] 写入eeprom  9600
		EEPROM_write_1(Addr_485, EEPROMTemp+1);	//BaudAddr[1] 写入eeprom  站址1
		EEPROM_write_1(Addr_autoT, EEPROMTemp+2);	//BaudAddr[2] 写入eeprom  0不自动发送
}


//========================================================================
// 函数: void EEPROM_write(u16 EE_address,u8 *DataAddress)
// 描述: 把缓冲的n个字节写入指定首地址的EEPROM.
// 参数: EE_address:  写入EEPROM的首地址.
//       DataAddress: 写入源数据的缓冲的首地址.

// 返回: non.
//========================================================================
uint8 EEPROM_write_1(u16 EE_address,u8 *DataAddress)
{
	uint8 write_count = 3;
	uint8 tempdata;
	
	while(write_count--)
	{
			EEPROM_SectorErase(EE_address);    	   				//对FLASH的首地址为0xE000+400处的扇区进行扇区擦除
			EEPROM_write_n(EE_address, DataAddress, 1);		//在FLASH的首地址为写入1个字节
			EEPROM_read_n(EE_address, &tempdata,1);			//在FLASH的首地址为0xE000处读取1个字节存入数组中
			if(*DataAddress == tempdata)
				return 0;																	//成功写入
	}
	return 1;
	
}

/******************************************************
函数名：dipkey_init(void)
功能：读取配置监测终端的3个参数的开关设置
参数：无参数
*******************************************************/

u8 dipkey_init(void)
{
	//uint8 dipkey;

	dipkey = ~P2;					//取反
	if((dipkey&0x1f) == 0)	return 1;			//如果低5们表示的地址为0，需要通过串口设置，返回标志1
	 
	if((dipkey&0x80) == 0)	   BaudAddr[0] = 5;					//波特率0----9600 （代号5） 
	if((dipkey&0x80) == 0x80)	 BaudAddr[0] = 6;					//波特率1----19200（代号6）
	
	if((dipkey&0x60) == 0)   	 BaudAddr[2] = 0;					//0----不自动发送
	if((dipkey&0x60) == 0x20)  BaudAddr[2] = 1;					//0010 0000----自动发送间隔1秒
	if((dipkey&0x60) == 0x40)  BaudAddr[2] = 3;					//0100 0000----自动发送间隔3秒
	if((dipkey&0x60) == 0x60)  BaudAddr[2] = 5;					//0110 0000----自动发送间隔5秒
	
	BaudAddr[1] = dipkey&0x1f;												//站址  低5位
	
	EEPROM_write_1(Addr_Baud, BaudAddr);							//BaudAddr[0] 写入eeprom  9600
	EEPROM_write_1(Addr_485, BaudAddr+1);							//BaudAddr[1] 写入eeprom  站址1
	EEPROM_write_1(Addr_autoT, BaudAddr+2);						//BaudAddr[2] 写入eeprom  0不自动发送
	
	return 0;
}	

/******************************************************
函数名：dipkey_lora_init(void)
功能：读取配置LORA的3个参数的开关设置
参数：无参数
*******************************************************/

u8 dipkey_lora_init(void)
{

	LORA_EN = 0;					//禁止LORA设置，进入传输状态
	loarkey = ~P0;						//取反

	if((loarkey&0x0F) == 0)	//如果低4位表示的信道地址为0，则DIP设置不起作用，其设置由亿百特的专用软件设定
	{
			lora_add = 00;
			lora_netID =	00;
			lora_channel = 1;				//此为标志位作用，本程序不对LORA进行设置
	}
	else
	{														//低4位为1-15，则所有8位的LORA设置起作用，3个参数
			lora_add = (loarkey & 0xC0) >> 6 ;
			lora_netID =	(loarkey & 0x30) >> 4;
			lora_channel = 22 + (loarkey & 0x0F);
	}

	return 1;
}

/******************************************************
函数名：lora_config_init(void)
功能：对LORA进行3个参数的设置，其它参数出厂设置
参数：无参数
*******************************************************/

void lora_config_init(void)
{
		unsigned char strTemp[9];
		uint8 i;
	
		//S1_USE_P30P31();					//串口1切换到 P30 P31 调试口	
	
		LORA_EN = 1;								//进入LORA设置状态	
		delay_ms(1000);	
		strTemp[0] = 0xC0 ;
		strTemp[1] = 0x00 ;
		strTemp[2] = 0x06 ;
		strTemp[3] = 0x00 ;
		strTemp[4] = lora_add ;
		strTemp[5] = lora_netID ;
		strTemp[6] = 0x64 ;					//62--2.4K  64--9.6K  air speed,  com speed 9.6k
		strTemp[7] = 0x00 ;
		strTemp[8] = lora_channel ;
		
		for(i=0; i<9; i++)					//发送设置命令，共设置6个寄存器
		{
				SendDataByUart1(strTemp[i]);
		}	
		
		//S1_USE_P36P37();					//串口1切换 P36,P37
		
		delay_ms(1000);	
		LORA_EN = 0;								//退出LORA设置，进入传输

}	


