/*************************************************************************
/项目名称：探测安防小车
/功能：串口接收命令控制智能小车的前后左右运动，接收命令控制小车探照灯的开关
/      单片机接收温湿度传感器模块的信息，超声波模块的信息，码盘测速的信息，
/      并通过串口转发出去
/文件名：Main.c
/平台：HJ-4WD/HJ-E/HL-1智能小车 + KeilU4 + STC89C52
/日期:2015年5月5日
/编写：@bear+XX
/*************************************************************************/
//说明1：
//

#include<reg52.h>
#include<intrins.h>
/******************************变量定义**************************************/
sbit P1_0 = P1^0;			  
sbit P1_1 = P1^1;
sbit P1_2 = P1^2;
sbit P1_3 = P1^3;
sbit P1_4 = P1^4;
sbit P1_5 = P1^5;
sbit P1_6 = P1^6;
sbit P1_7 = P1^7; 
#define LeftGo      {P1_4=1,P1_5=0,P1_6=1,P1_7=0;}   	//左边两个电机向前走
#define LeftBack    {P1_4=0,P1_5=1,P1_6=0,P1_7=1;}    	//左边两个电机向后转
#define LeftStop    {P1_4=0,P1_5=0,P1_6=0,P1_7=0;}    	//左边两个电机停转                     
#define RightGo     {P1_0=1,P1_1=0,P1_2=1,P1_3=0;}		//右边两个电机向前走
#define RightBack   {P1_0=0,P1_1=1,P1_2=0,P1_3=1;}		//右边两个电机向后走
#define RightStop   {P1_0=0,P1_1=0,P1_2=0,P1_3=0;}		//右边两个电机停转    


sbit DhtData = P2^3;			  				//温湿度数据输入口 
unsigned char TH,TL,RH,RL,CheckDht;		  		//温湿度的高和低8位
unsigned char THtemp,TLtemp,RHtemp,RLtemp,CheckNum;	//温湿度读取的临时变量
unsigned char DhtNum = 0;						//用于温湿度while循环计数，
unsigned char DhtTemp;							//用于温湿度读取时的变量

sbit Hummaninfrare = P2^7;						//人体红外感应结果输入口

unsigned char command;							//单片机接收的命令 

sbit Light = P2^0;								//探照灯输出口

sbit  TRIG = P2^5;								//超声波发送数据口
sbit  ECHO = P2^4;								//超声波接收数据口,外部中断0

unsigned int time =0; 							//超声波接收高电平的时间
unsigned long S=0;						  		//超声波测出来的障碍物距离
unsigned int wave_count;
unsigned int count_start;
unsigned int count_end;
bit wave_flag1 = 0;							 	//超声波接收开始计数的标志，0不开始计数
bit wave_flag2 = 0;
bit flag_start = 0;								//超声波开始是否超出范围的标志
bit flag_end = 0;								//超声波接收是否超出范围的标志
bit flag_send = 0;								// 是否发送超声波到上位机
bit distance_flag = 0;

unsigned char disbuffer[4] = {0,0,0,0};	  		//存储障碍物距离各个位数数值

unsigned int count;						    	//记录左电机码盘产生的脉冲数(一个脉冲相当于行驶1cm).
unsigned char buffer[4] = {0,0,0,0};			//存储小车行驶距离各位数值数组


/*****************************函数声明*****************************************/
void UartInit(void);					//串口初始化
void SendByte(unsigned char dat);		//发送一个字节
void InitTimer0(void);
void delayms(unsigned int t);
//---------------------温湿度相关函数的声明------------------------------------
void Delay10Us(void);					//读温湿度需要的两个延时函数
void DelayDht(unsigned int j);
unsigned char ReadOneData(void);		//读一个字节数据
void GetHumiture(void);					//得到温湿度的数值
//---------------------超声波相关函数的声明------------------------------------
void WaveStart(void);					//超声波测距启动函数
void DisCount(void);					//超声波距离测量计算函数
//-----------------------行驶距离相关函数的声明---------------------------------

/*******************************主函数*****************************************/
void main(void)
{	
	UartInit();							//串口初始化
	InitTimer0();						//定时器0初始化
	Light = 0;							//初始化时不打开探照灯.

	while(1)
	{
		//湿度模块
		TR0 = 0;	
		ES = 0;
		GetHumiture();
		delayms(10);
		ES = 1;
		SendByte('r');
	    SendByte(RH/10+0x30);
		SendByte(RH%10+0x30);
		SendByte('\r');
		SendByte('\n');
		delayms(50);	
		//温度
		SendByte('t');
		SendByte(TH/10+0x30);
		SendByte(TH%10+0x30);
		SendByte('\r');
		SendByte('\n');	
		delayms(10);
		
		//码盘测速得行驶距离模块		
		EX0 = 1;				 		//开启外部中断1.
		IT0 = 1;				 		//下降沿有效
		IE0=0;					 		//外部中断请求标志 							
		buffer[0]=count/1000;		 	//更新显示																	
	    buffer[1]=count%1000/100;
	    buffer[2]=count%1000%100/10;
        buffer[3]=count %1000%100%10;	
		SendByte('j');
		SendByte(buffer[0]+0x30);
		SendByte(buffer[1]+0x30);
		SendByte(buffer[2]+0x30);
		SendByte(buffer[3]+0x30);
		SendByte('\r');
		SendByte('\n');
		delayms(50);
		TR0 = 1;						//开启计时器的计数

		
		//超声波测距离模块
		WaveStart();				  	//启动超声波模块
		wave_flag1 = 1;					//
		while((!ECHO)&(!flag_start));			//等待，当ECHO为1时，超声波开始接收，当等待超过100ms时，
										//认为超出范围,
		ES = 1;							//打开串口中断
		if(flag_start ==1)					//超出范围
		{
			disbuffer[0] = 9;		
	 		disbuffer[1] = 9;
			disbuffer[2] = 9; 
	 		disbuffer[3] = 9;	
			flag_start = 0;
		}else							//ECHO接收到高电平
		{
			wave_flag1 = 0;
			count_start = 0;

			wave_flag2 = 1;				//开启是否接收超出范围的中断
			distance_flag = 1;			//开始计数高电平时间
			while((ECHO)&(!flag_end));

			if(flag_end ==1)			//超出范围
			{
	 			disbuffer[0] = 9;		
	 			disbuffer[1] = 9;
				disbuffer[2] = 9; 
	 			disbuffer[3] = 9;	
				flag_end = 0;
			}else
			{
				wave_flag2 = 0;
				count_end = 0;
				flag_send = 1;
 				distance_flag = 0;
    			ES = 0;
				DisCount();
				wave_count = 0;			
			}
		}
		wave_flag1 = 0;
		wave_flag2 = 0;
		distance_flag = 0;
		ES = 1;		
		SendByte('z');
		SendByte(disbuffer[0]+0x30);
		SendByte(disbuffer[1]+0x30);
		SendByte(disbuffer[2]+0x30);
		SendByte(0x30);
		SendByte('\r');
		SendByte('\n');		
		TR0 = 0;			   
		delayms(50);
		//小车运动控制部分(在串口中断中处理)		
		//探照灯检测部分(在串口中断中处理)		
		
		//人体红外感应部分
		if(Hummaninfrare==0)			//没有检测到高电平
		{
			SendByte('n');
			SendByte(0x30);
			SendByte('\r');
			SendByte('\n');
		}else
		{
			SendByte('p');
			SendByte(0x30);
			SendByte('\r');
			SendByte('\n');			
		}
		delayms(50);			 			    
	}
}

/*******************************函数的定义****************************************/
//------------------------------读温湿度需要的两个延时函数-------------------------
//Function：在读取温湿度信息时需要的一个延时序列
//Input:整数
//Output：无
void DelayDht(unsigned int j)
{   
	unsigned char i;
    for(;j>0;j--)
    {
        for(i=0;i<27;i++);

    }
}
//Function：在读取温湿度信息时需要的一个延时序列，延时微妙级
//Input:无
//Output：无
void  Delay10Us(void)
{
    unsigned char i;
    i--;
    i--;
    i--;
    i--;
    i--;
    i--;
}
//------------------------------串口初始化------------------------------------------
//Function：设置串口的模式，通信方式，波特率等，以及打开相应的中断
void UartInit(void)
{
	TMOD |= 0x21;   					//TMOD：timer1,mode2, 8bit重装
	SCON = 0x50;   						//SCON:模式1,8-bit UART,使能接收
	TH1=TL1=0xFD;						//为定时器赋初值，晶振为11.0592MHZ时，波特率为9600
	EA = 1;								//总中断打开
	ES = 1;								//打开串口中断
	TR1 = 1;							//定时器1打开
}
//-----------------------------定时器0初始化----------------------------------------
//Function：设置定时器的初值,
void InitTimer0(void)
{
	TH0=0xFF;TL0=0xF6;
	ET0 = 1;							//定时器中断打开
	//TR0 = 1;  		   				//定时器开关打开
	//TR0 = 1;						//开启定时器0的计数	
}

/*----------------------------定时器0中断服务子程序-------------------------------*/
void Timer0_ISR(void) interrupt 1			 //每次中断为10us
{
	TH0=0xFF;
	TL0=0xF6;


	if(wave_flag1)
	{	
		count_start++; 
		if(count_start>=12000)
		{
	 		count_start=0;
			flag_start = 1;
		}
	}
	
	if(wave_flag2)
	{
		count_end++;
		if(count_end>=13000)
		{
		 	count_end = 0;
			flag_end = 1;
		}	
	}
	
	if(distance_flag)
	{
		wave_count++;	
	}		
}

/*-----------------------------发送一个字节-----------------------------------------*/
//Function：发送字符串
void sendByte(unsigned char dat)
{
	TI = 0;								//发送标志位清零
	SBUF = dat;							//将要发送的字节写入发送缓冲器
	while(!TI);
	TI = 0;
}


/*----------------------------串口中断服务子程序-------------------------------*/
//接收串口发过来的数据，处理小车的运动
void Uart_isr(void) interrupt 4
{
   if(RI)								//判断是否为接收中断
   {
   		ES = 0;							//关闭串口中断
		RI = 0;							//标志位清零
	    command = SBUF;		 			//得到接收缓存器的值，并赋给接收变量command
   }else
   	   return;
	delayms(200);
	//小车运动控制部分
	switch(command)
   	{
		case 'a':						//向前
		{
			LeftGo;RightGo;
			break;
		}		
   		case 'b':						//向后
		{
			LeftBack;RightBack;
			break;
		}
		case 'c':						//向左
		{
			LeftBack;RightGo;
			break;
		}
		case 'd':						//向右
		{
			LeftGo;RightBack;
			break;
		}
	 	case 'e':						//停止
		{
			LeftStop;RightStop;
			break;
		}
		default:
		{
			LeftStop;RightStop;
			break;
		}
	}
	 switch(command)
    {
		case 'f':						//打开灯
		{
	   		Light = 1;break;
		}
		case 'h':
		{
		    Light = 0;break;				//关闭灯
		}
		default:
		{
		 	break;
		}
	}
   ES = 1;								//打开串口中断
}
    
/*----------------------------------外部中断0----------------------------------------*/
//function：下降沿来的时候计数一次
void Inter0_ISR(void) interrupt 0 using 1
{
   if(command=='a')
   {
     count++;
   }
   else if(command=='b')
   {
   	 if(count==0)
	 	count=0;
	 else
	 	count--;
   }
}

															 
/*------------------------------温湿度相关的函数------------------------------------*/
//------------------------------读温湿度数据------------------------------------------
//function:获得一个字节的数据				
unsigned char ReadOneData(void)   		//接收一个8位数据，先高位后低位
{
	unsigned char i;
	unsigned char dat = 0;
	for(i=0;i<8;i++)
	{
		DhtNum = 2;						//用于while循环计数，超时跳出循环
		while((!DhtData)&&DhtNum++);	//dht_data数据线由DHT11拉低发送数据	
		Delay10Us();
		Delay10Us();
		Delay10Us();  					
		DhtTemp = 0;
		if(DhtData)DhtTemp=1;
		DhtNum = 2;
		while((DhtData)&&DhtNum++);
		//超时则跳出for循环
		if(DhtNum==1)break;
		dat<<=1;
		dat|=DhtTemp;
	}
	return dat;
}

//---------------------------------得温湿度数据-------------------------------------
void GetHumiture(void)
{
	DhtData = 0;						//单片机拉低数据线
	DelayDht(180);
	DhtData = 1;						//单片机给起始脉冲信号
	Delay10Us();
	Delay10Us();
	Delay10Us();
	Delay10Us();
	DhtData = 1;						//稍作延时，等待DHT11返回响应（响应为低电平）
	
	if(!DhtData)						//有响应就接收数据，否则不做处理
	{
	   DhtNum = 2;
	   //判断从机是否发出80us的低电平响应是否结束
	   while((!DhtData)&&DhtNum++);
	   DhtNum = 2;
	   //判断从机是否发出80us的高电平，如果发出进入数据接收状态
	   while((DhtData)&&DhtNum++);
	   //数据接收状态
	   RHtemp = ReadOneData();
	   RLtemp = ReadOneData(); 
	   THtemp = ReadOneData();
	   TLtemp = ReadOneData();
	   CheckDht = ReadOneData();
	   DhtData = 1;						//总线有上拉电阻拉高，进入空闲状态

	   CheckNum=(RHtemp+RLtemp+THtemp+TLtemp);
	   if(CheckDht == CheckNum)
	   {
		  RH = RHtemp;
		  RL = RLtemp;
		  TH = THtemp;
		  TL = TLtemp;
	   }
	}
	delayms(2);							//稍作延时处理	 
}
//----------------------------------超声波测距启动函数-----------------------------------
void WaveStart(void)
{
    //延时600ms最好
	TRIG = 1;	   //启动超声波模块
	//延时10us,使TRIG在这段时间维持高电平
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	TRIG = 0; 
}

//-----------------------------------超声波距离测量函数-----------------------------------		
void DisCount(void)
{

	 S = (wave_count*1.7)/10;				//算出来的单位为CM
	 disbuffer[0] = S%1000/100;		
	 disbuffer[1] = S%1000%100/10;
	 disbuffer[2] = S%1000%10%10; 
	 disbuffer[3] = 0;
}


//ms级别的延时函数
void delayms(unsigned int t)
{
	 unsigned char temp; 
	 while(t--)
	 {
		 temp = 245;
		 while(temp--);
	 }
}
