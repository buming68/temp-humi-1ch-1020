#include	"receservice.h"

//extern uint8 Uart1_SEND_Buffer[SEND_MAX_LEN];		//发送缓冲区
//extern uint8 CHNUM;
float fTempM[10]=0 ;			//8路温度全局变量,初始化为0度

/******************************************************
函数名：uart1_rece_proc(void)
功能：处理串口接收的有效数据
参数：无参数
*******************************************************/
void uart1_rece_proc(void)
{
		unsigned char strTemp[20];
		uint8 i;

		if(Recv_OK)
		{	
			uart1_rece_ack();				//向主机回送测温信息；向串口发送数据
			
			S1_USE_P30P31();					//串口1切换到 P30 P31 调试口，输出				
		
			sprintf(strTemp, "%d-%d%% ", (int)fTempM[0],(int)fTempM[1]); //浮点数转成字符串 
			SendStringByUart1(strTemp);       //串口打印出温度值
			
			if(Uart1_RX_Buffer[0] == 0xC1)		//仅输出LORA设置返回信息
			{
				for(i=0; i<Recv_Cnt; i++)
				{
						SendDataByUart1(Uart1_RX_Buffer[i]);
				}		
			}
			
			S1_USE_P36P37();					//串口1切换回 P36,P37


			REN = 1;							//允许串口接收
			Recv_Cnt = 0;					//串口1接收计数清0
			Recv_OK = 0;					//清串口1接收标志
		}	
}

/******************************************************
函数名：uart1_auto_tx(void)
功能：自动定时发送8个传感器的温度
参数：无参数
*******************************************************/
void uart1_auto_tx(void)
{			
		uint16 crc16bit;	
	
		Recv_Cnt = 8;
			
		Uart1_RX_Buffer[0] = BaudAddr[1];		//从站地址
		Uart1_RX_Buffer[1] = 0x03;					//模拟查询功能码 03
		Uart1_RX_Buffer[2] = 0x00;
		Uart1_RX_Buffer[3] = 0x00;
		Uart1_RX_Buffer[4] = 0x00;
		Uart1_RX_Buffer[5] = 0x02;					//读取2个温度和湿度数值
//		if(CHNUM == 1)	Uart1_RX_Buffer[5] = 0x01;					//读取1个温度数值
	
		crc16bit = crc16(Uart1_RX_Buffer, Recv_Cnt - 2);		//计算CRC
		Uart1_RX_Buffer[6] = crc16bit /256;		
		Uart1_RX_Buffer[7] = crc16bit %256;	

		uart1_rece_ack();				//向主机回送测温信息；自动回传	
	
		REN = 1;							//允许串口接收
		Recv_Cnt = 0;					//串口1接收计数清0
		Recv_OK = 0;					//清串口1接收标志
}

/******************************************************
函数名：uart1_rece_ack()
功能：根据接收到的Uart1_RX_Buffer[]数据，向上位机返回发送数据
参数：无参数（处理全局数组）
*******************************************************/
void uart1_rece_ack()				//向主机回送测温信息；
{
//	uint8 Uart1_RX_Buffer[8] = {0x30,0x03,0x00,0x00,0x00,0x01,0x80,0x2B};  //校验码 802B
//  uint8 Uart1_RX_Buffer[8] = {0x30,0x03,0x00,0x00,0x00,0x02,0xC0,0x2A};
//  uint8 Uart1_RX_Buffer[8] = {0x00,0x03,0x00,0x00,0x00,0x01,0x85,0xDB};  //广播测试地址
//	uint8 Uart1_RX_Buffer[8] = {0x00,0x03,0x00,0x01,0x00,0x01,0xD4,0x1B};  //查询从机地址49=0x31
//	uint8 Uart1_RX_Buffer[8] = {0x00,0x06,0x00,0x01,0x00,0x31,0x18,0x0F};  //设置从机地址
//	uint8 Uart1_RX_Buffer[8] = {0x00,0x06,0x00,0x25,0x55,0xAA,0x26,0xFF};  //软件复位命令
//	uint8 Uart1_RX_Buffer[8] = {0x30,0x03,0x00,0x00,0x00,0x02,0xC0,0x2A};  //双路温度回传命令
//	uint8 Uart1_RX_Buffer[8] = {0x30,0x03,0x00,0x00,0x00,0x03,0x01,0xEA};  //三路温度回传命令
//	uint8 Uart1_RX_Buffer[8] = {0x00,0x06,0x01,0x09,0x12,0x34,0x54,0x92};  //恢复出厂设置命令05 01 00	
//	uint8	Recv_Cnt = 8;
	
	uint16 crc16bit;	
	uint8	send_cnt = 0;
	uint8 rsp_cnt, err=RECV_NO_ERROR;
	uint8 i;
	
	crc16bit = crc16(Uart1_RX_Buffer, Recv_Cnt - 2);
	if(memcmp(Uart1_RX_Buffer + Recv_Cnt - 2 , &crc16bit, 2)==0)
	{				
			memcpy(Uart1_SEND_Buffer, Uart1_RX_Buffer, 2);    //modbus addr + func code
		
			rsp_cnt = Modbus_Req_Func_Match(Uart1_RX_Buffer+1, Recv_Cnt, Uart1_SEND_Buffer+2,&err);
			if(rsp_cnt == 0)           //帧长度错误时，rsp_cnt为0，不处理
        return;

			if (err != RECV_NO_ERROR){
					Uart1_SEND_Buffer[1] += 0x80;
					Uart1_SEND_Buffer[2] = err;
					rsp_cnt = 3;
			}
			else rsp_cnt += 2;         //返回pdu长度不包含地址和功能码
			
			send_cnt = rsp_cnt; 				//发送的有效数据长度；不含2位CRC
			
			crc16bit = crc16(Uart1_SEND_Buffer, send_cnt);	//获CRC16
			Uart1_SEND_Buffer[send_cnt] = crc16bit>>8 ;
			Uart1_SEND_Buffer[send_cnt+1] = crc16bit;

			TXEN485 = 1;			//控制485发送功能，需要发送时设定
			for(i=0; i<send_cnt+2; i++)
			{
				SendDataByUart1(Uart1_SEND_Buffer[i]);			//发送应答回送数据
			}		
			TXEN485 = 0;			//控制485接收功能
	}
	
}

//Modbus命令请求功能码匹配
/**
  *	  正常情况下返回响应帧全部payload的字节数，不包括帧头的modbus地址，功能码，和帧尾CRC校验个数
  *   请求帧长度错误时，返回0
  *   非法的请求地址、长度或数据值时和设备服务故障时，err不为0，返回-1
 */
uint8 Modbus_Req_Func_Match(uint8 *recv_pdu, uint8 recv_cnt, uint8 *send_pdu, uint8 *err)
{
    uint8 rsp_cnt;
    uint8 func_code;
    
    func_code = *recv_pdu++;

    switch(func_code){						//只支持04、06功能码
        case READ_COIL:						//01、02功能码使用时不作区分
        case READ_INPUT_COIL:*err = ILLEGAL_FUNCTION;break;  //不支持的功能码先返回01错误码
        case READ_HOLD_REG:				//03、04功能码使用时不作区分
        case READ_INPUT_REG:			//04功能码
							rsp_cnt=Read_Mb_Reg_Rsp(recv_pdu, recv_cnt, send_pdu, err);
							break;
        case WRITE_COIL:
        case WRITE_HOLD_REG:			//08功能码
							rsp_cnt=Write_Mb_Reg_Rsp(recv_pdu, recv_cnt, send_pdu, err);
							break;
        case WRITE_MULTI_COIL:
        case WRITE_MULTI_REG:
        default:*err = ILLEGAL_FUNCTION;break;              //不支持的功能码先返回01错误码
    }
    return rsp_cnt;
}

//读单个寄存器值，功能码0x03、0x04
uint8 Read_Mb_Reg_Rsp(uint8 *req_pdu, uint8 recv_cnt, uint8 *send_pdu, uint8 *err)
{
    uint16 req_addr,req_len,pdu_len;
    uint16 *pdu_ptr;
	
    if(recv_cnt != 8)           //功能码03、04的请求长度只能为8byte 
        return 0;

	pdu_ptr = (uint16 *)req_pdu;
	req_addr =  (*pdu_ptr++)*1000;
	req_len  = *pdu_ptr;
	
	if(req_len == 0 || req_len > 125    //标准请求帧读出的总长度
    || req_len > (SEND_MAX_LEN-5)/2){	//本地发送限制
		*err = ILLEGAL_DATA_LEN;
		return -1;
	}

	*send_pdu++ = pdu_len = 2 * req_len;
	pdu_ptr = (uint16 *)send_pdu;
	
	while(req_len--){
		*pdu_ptr++ = Read_Modbus_Reg(req_addr++, err);	//按地址读取对应的数值
		if(*err != RECV_NO_ERROR)
			return -1;
	}
    return pdu_len+1;
}

//读modbus寄存器点表,功能码0x03，0x04调用
uint16 Read_Modbus_Reg(uint16 req_addr, uint8 *err)
{
	uint16 reg = 0xFFFF;   //预留的modbus地址，给个特定返回值以示区别，也可以为0
		
	switch(req_addr){
        case 0:
						reg = (int) fTempM[0];
						break;
        case 1:		
						reg = (int) fTempM[1] ;	
						break;
				case 2:		
						reg = (int) fTempM[2];	
						break;
				case 3:		
						reg = (int) fTempM[3];	
						break;
				case 4:
						reg = (int) fTempM[4];
						break;
        case 5:		
						reg = (int) fTempM[5] ;	
						break;
				case 6:		
						reg = (int) fTempM[6];	
						break;
				case 7:		
						reg = (int) fTempM[7];	
						break;
				case 8:		
						reg = (int) fTempM[8];	
						break;				
				case 9:		
						reg = (int) fTempM[9];	
						break;

        //case 8:break;                                           //预留
        //case 9:break;                                           
        //case 10:break;                                           
				case 1000:									//查询485地址从机地址
					reg = BaudAddr[1];
					break;                   
				case 15000:									//查询从机是否自动发送，0--不自动，数字---间隔
					reg = BaudAddr[2];
					break;  
				case 3000:									//查询从机是通讯速率，5--9600 6---19200
					reg = BaudAddr[0];
					break; 
				
		default:*err=ILLEGAL_DATA_ADDR;break;                   //非法地址
	}
	return reg;
}

//写单个寄存器值，功能码0x06
uint8 Write_Mb_Reg_Rsp(uint8 *req_pdu, uint8 recv_cnt, uint8 *send_pdu, uint8 *err)
{
    uint16 req_addr,req_data;
		//uint16 req_len,pdu_len;
    uint16 *pdu_ptr;
		uint8 i;
	
    if(recv_cnt != 8)           //功能码06的请求长度只能为8byte 
        return 0;

		for(i=0; i<4; i++)	*(send_pdu+i) = *(req_pdu+i);
		
		pdu_ptr = (uint16 *)req_pdu;
		req_addr = *pdu_ptr;
		req_data = (uint16)(*(pdu_ptr+1));
		Write_Modbus_Reg(req_addr, req_data, err);	//按地址读取对应的数值	

    return 4;
}

//读modbus寄存器点表,功能码0x06调用
uint16 Write_Modbus_Reg(uint16 req_addr, uint16 req_data, uint8 *err)
{
	uint16 reg = 0xFFFF;   //预留的modbus地址，给个特定返回值以示区别，也可以为0
	
	switch(req_addr){
        case 1:						//设置从站地址	
						BaudAddr[1] = (uint8) req_data;				
						EEPROM_write_1(Addr_485, BaudAddr+1);	//BaudAddr[1] 写入eeprom
						//delay_ms(1);
						break;
				case 0x0025:				//软启动
						IAP_CONTR = 0x20;
						break;
				case 0x0109:				//恢复出厂设置 9600，站址1  不自动发送
						if(req_data != 0x1234)	break;						//恢复出厂命令：00 06 01 09 12 34 54 92
						EEPROM_Factory();
						IAP_CONTR = 0x20;													//重启动
						break;	
				case 0x000F:				//设置是否自动回传，非0：自动回传，0：不回传
						BaudAddr[2] = (uint8) req_data;				
						EEPROM_write_1(Addr_autoT, BaudAddr+2);	//BaudAddr[1] 写入eeprom
						
						IAP_CONTR = 0x20;
				
						//delay_ms(1);
						break;	
				
					case 0x0003:				//设置通讯速率，5--9600   6-19200
						if((req_data==5) || (req_data==6))				
						{
								BaudAddr[0] = (uint8) req_data;				
								EEPROM_write_1(Addr_Baud, BaudAddr);	//BaudAddr[0] 写入eeprom
								
								IAP_CONTR = 0x20;										//重新启动
						}

						break;	
						
				//case 1:reg=PWM.Duty;break;	                //uint8_t
				//case 2:reg=ADC.ADC_Value;break;					        //uint16_t
				//case 3:reg=*(uint16*)&NTC.NTC_Temperature;break;  	//float
				//case 3:
				//case 4:reg=*((uint16_t*)&NTC.NTC_Temperature+1);break;		   
        //case 5:break;
        //case 6:reg=*(uint16_t*)&NTC.NTC_Voltage;break;
        //case 7:reg=*((uint16_t*)&NTC.NTC_Voltage+1);break;
        //case 8:break;                                           //预留
        //case 9:break;                                           
        //case 10:break;                                           
				//case 1001:reg=Modbus.uintaddr;break;                    //modbus从机地址  
        //case 1002:*err = DEVICE_FAILURE;break;                  //模拟读寄存器失败,错误码04
				
			default:*err=ILLEGAL_DATA_ADDR;break;                   //非法地址
	}
	return reg;
}



//modbusrtu定义部分
/*
								unsigned char crc_high8;//modbusrtu_crc16效验
								unsigned char crc_low8;//modbusrtu_crc16效验
								//modbusrtu_crc16效验测试数据包	//
								//unsigned char data_CRC[6] = {0x03,0x10,0x00,0x01,0x00,0x08};//校验码91ED
								//unsigned char data_CRC[6] = {0x01,0x05,0x00,0x00,0xFF,0x00};//校验码8C3A
								//modbusrtu_crc16效验测试数据包	//
*/
//modbusrtu定义部分

//modbusrtu函数部分
/* CRC高位字节值表*/ 
const unsigned char code auchCRCHi[] = { //code表示存入ram
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40} ;/* CRC高位字节值表*/  
/* CRC低位字节值表*/ 
const unsigned char code auchCRCLo[] = { //code表示存入ram
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 
0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 
0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 
0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 
0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 
0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 
0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 
0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 
0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 
0x43, 0x83, 0x41, 0x81, 0x80, 0x40} ;/* CRC低位字节值表*/ 

unsigned int crc16(unsigned char *puchMsg, unsigned int usDataLen) //（待校验数据首地址，待校验数据位数）
{ 
	unsigned char uchCRCHi = 0xFF ; /* 高CRC字节初始化 */ 
	unsigned char uchCRCLo = 0xFF ; /* 低CRC 字节初始化 */ 
	unsigned long uIndex ; /* CRC循环中的索引 */ 
	while (usDataLen--) /* 传输消息缓冲区 */ 
	{ 
		
		uIndex = uchCRCHi ^ *puchMsg++ ; /* 计算CRC */ 
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ; /* 计算CRC高位 */ 
		uchCRCLo = auchCRCLo[uIndex] ; 		/* 计算CRC低位 */ 
	} 
				//将得到的数据储存到crc	
				//crc_high8=uchCRCHi;
				//crc_low8=uchCRCLo;
	
				//将得到的数据放到串口接受用的数组中
	return (uchCRCHi << 8 | uchCRCLo) ; //返回值返回  
}
//modbusrtu函数部分
//modbusrtu函数运行部分
				//运行下面函数可以得到效验数据 一般是放到数组里面再发出去所以去掉//返回值返回  这一行
				//可以主程序里加下面两行，上面的（//返回值返回  ）这行取消注释
				//crc_high8=crc16(data_CRC,6)>>8;//crc高8位
				//crc_low8=crc16(data_CRC,6) & 0xFF;//crc低8位

				//crc16(data_CRC,6);//也可以在主程序中加上这个函数，函数运行后会把效验数据放到crc_high8,crc_low8这两个全局变量里
//modbusrtu函数运行部分



/*********************************END FILE********************************************/	