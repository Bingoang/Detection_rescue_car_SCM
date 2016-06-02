/*************************************************************************
/��Ŀ���ƣ�̽�ⰲ��С��
/���ܣ����ڽ��������������С����ǰ�������˶��������������С��̽�յƵĿ���
/      ��Ƭ��������ʪ�ȴ�����ģ�����Ϣ��������ģ�����Ϣ�����̲��ٵ���Ϣ��
/      ��ͨ������ת����ȥ
/�ļ�����Main.c
/ƽ̨��HJ-4WD/HJ-E/HL-1����С�� + KeilU4 + STC89C52
/����:2015��5��5��
/��д��@bear+XX
/*************************************************************************/
//˵��1��
//

#include<reg52.h>
#include<intrins.h>
/******************************��������**************************************/
sbit P1_0 = P1^0;			  
sbit P1_1 = P1^1;
sbit P1_2 = P1^2;
sbit P1_3 = P1^3;
sbit P1_4 = P1^4;
sbit P1_5 = P1^5;
sbit P1_6 = P1^6;
sbit P1_7 = P1^7; 
#define LeftGo      {P1_4=1,P1_5=0,P1_6=1,P1_7=0;}   	//������������ǰ��
#define LeftBack    {P1_4=0,P1_5=1,P1_6=0,P1_7=1;}    	//�������������ת
#define LeftStop    {P1_4=0,P1_5=0,P1_6=0,P1_7=0;}    	//����������ͣת                     
#define RightGo     {P1_0=1,P1_1=0,P1_2=1,P1_3=0;}		//�ұ����������ǰ��
#define RightBack   {P1_0=0,P1_1=1,P1_2=0,P1_3=1;}		//�ұ�������������
#define RightStop   {P1_0=0,P1_1=0,P1_2=0,P1_3=0;}		//�ұ��������ͣת    


sbit DhtData = P2^3;			  				//��ʪ����������� 
unsigned char TH,TL,RH,RL,CheckDht;		  		//��ʪ�ȵĸߺ͵�8λ
unsigned char THtemp,TLtemp,RHtemp,RLtemp,CheckNum;	//��ʪ�ȶ�ȡ����ʱ����
unsigned char DhtNum = 0;						//������ʪ��whileѭ��������
unsigned char DhtTemp;							//������ʪ�ȶ�ȡʱ�ı���

sbit Hummaninfrare = P2^7;						//��������Ӧ��������

unsigned char command;							//��Ƭ�����յ����� 

sbit Light = P2^0;								//̽�յ������

sbit  TRIG = P2^5;								//�������������ݿ�
sbit  ECHO = P2^4;								//�������������ݿ�,�ⲿ�ж�0

unsigned int time =0; 							//���������ոߵ�ƽ��ʱ��
unsigned long S=0;						  		//��������������ϰ������
unsigned int wave_count;
unsigned int count_start;
unsigned int count_end;
bit wave_flag1 = 0;							 	//���������տ�ʼ�����ı�־��0����ʼ����
bit wave_flag2 = 0;
bit flag_start = 0;								//��������ʼ�Ƿ񳬳���Χ�ı�־
bit flag_end = 0;								//�����������Ƿ񳬳���Χ�ı�־
bit flag_send = 0;								// �Ƿ��ͳ���������λ��
bit distance_flag = 0;

unsigned char disbuffer[4] = {0,0,0,0};	  		//�洢�ϰ���������λ����ֵ

unsigned int count;						    	//��¼�������̲�����������(һ�������൱����ʻ1cm).
unsigned char buffer[4] = {0,0,0,0};			//�洢С����ʻ�����λ��ֵ����


/*****************************��������*****************************************/
void UartInit(void);					//���ڳ�ʼ��
void SendByte(unsigned char dat);		//����һ���ֽ�
void InitTimer0(void);
void delayms(unsigned int t);
//---------------------��ʪ����غ���������------------------------------------
void Delay10Us(void);					//����ʪ����Ҫ��������ʱ����
void DelayDht(unsigned int j);
unsigned char ReadOneData(void);		//��һ���ֽ�����
void GetHumiture(void);					//�õ���ʪ�ȵ���ֵ
//---------------------��������غ���������------------------------------------
void WaveStart(void);					//�����������������
void DisCount(void);					//����������������㺯��
//-----------------------��ʻ������غ���������---------------------------------

/*******************************������*****************************************/
void main(void)
{	
	UartInit();							//���ڳ�ʼ��
	InitTimer0();						//��ʱ��0��ʼ��
	Light = 0;							//��ʼ��ʱ����̽�յ�.

	while(1)
	{
		//ʪ��ģ��
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
		//�¶�
		SendByte('t');
		SendByte(TH/10+0x30);
		SendByte(TH%10+0x30);
		SendByte('\r');
		SendByte('\n');	
		delayms(10);
		
		//���̲��ٵ���ʻ����ģ��		
		EX0 = 1;				 		//�����ⲿ�ж�1.
		IT0 = 1;				 		//�½�����Ч
		IE0=0;					 		//�ⲿ�ж������־ 							
		buffer[0]=count/1000;		 	//������ʾ																	
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
		TR0 = 1;						//������ʱ���ļ���

		
		//�����������ģ��
		WaveStart();				  	//����������ģ��
		wave_flag1 = 1;					//
		while((!ECHO)&(!flag_start));			//�ȴ�����ECHOΪ1ʱ����������ʼ���գ����ȴ�����100msʱ��
										//��Ϊ������Χ,
		ES = 1;							//�򿪴����ж�
		if(flag_start ==1)					//������Χ
		{
			disbuffer[0] = 9;		
	 		disbuffer[1] = 9;
			disbuffer[2] = 9; 
	 		disbuffer[3] = 9;	
			flag_start = 0;
		}else							//ECHO���յ��ߵ�ƽ
		{
			wave_flag1 = 0;
			count_start = 0;

			wave_flag2 = 1;				//�����Ƿ���ճ�����Χ���ж�
			distance_flag = 1;			//��ʼ�����ߵ�ƽʱ��
			while((ECHO)&(!flag_end));

			if(flag_end ==1)			//������Χ
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
		//С���˶����Ʋ���(�ڴ����ж��д���)		
		//̽�յƼ�ⲿ��(�ڴ����ж��д���)		
		
		//��������Ӧ����
		if(Hummaninfrare==0)			//û�м�⵽�ߵ�ƽ
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

/*******************************�����Ķ���****************************************/
//------------------------------����ʪ����Ҫ��������ʱ����-------------------------
//Function���ڶ�ȡ��ʪ����Ϣʱ��Ҫ��һ����ʱ����
//Input:����
//Output����
void DelayDht(unsigned int j)
{   
	unsigned char i;
    for(;j>0;j--)
    {
        for(i=0;i<27;i++);

    }
}
//Function���ڶ�ȡ��ʪ����Ϣʱ��Ҫ��һ����ʱ���У���ʱ΢�
//Input:��
//Output����
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
//------------------------------���ڳ�ʼ��------------------------------------------
//Function�����ô��ڵ�ģʽ��ͨ�ŷ�ʽ�������ʵȣ��Լ�����Ӧ���ж�
void UartInit(void)
{
	TMOD |= 0x21;   					//TMOD��timer1,mode2, 8bit��װ
	SCON = 0x50;   						//SCON:ģʽ1,8-bit UART,ʹ�ܽ���
	TH1=TL1=0xFD;						//Ϊ��ʱ������ֵ������Ϊ11.0592MHZʱ��������Ϊ9600
	EA = 1;								//���жϴ�
	ES = 1;								//�򿪴����ж�
	TR1 = 1;							//��ʱ��1��
}
//-----------------------------��ʱ��0��ʼ��----------------------------------------
//Function�����ö�ʱ���ĳ�ֵ,
void InitTimer0(void)
{
	TH0=0xFF;TL0=0xF6;
	ET0 = 1;							//��ʱ���жϴ�
	//TR0 = 1;  		   				//��ʱ�����ش�
	//TR0 = 1;						//������ʱ��0�ļ���	
}

/*----------------------------��ʱ��0�жϷ����ӳ���-------------------------------*/
void Timer0_ISR(void) interrupt 1			 //ÿ���ж�Ϊ10us
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

/*-----------------------------����һ���ֽ�-----------------------------------------*/
//Function�������ַ���
void sendByte(unsigned char dat)
{
	TI = 0;								//���ͱ�־λ����
	SBUF = dat;							//��Ҫ���͵��ֽ�д�뷢�ͻ�����
	while(!TI);
	TI = 0;
}


/*----------------------------�����жϷ����ӳ���-------------------------------*/
//���մ��ڷ����������ݣ�����С�����˶�
void Uart_isr(void) interrupt 4
{
   if(RI)								//�ж��Ƿ�Ϊ�����ж�
   {
   		ES = 0;							//�رմ����ж�
		RI = 0;							//��־λ����
	    command = SBUF;		 			//�õ����ջ�������ֵ�����������ձ���command
   }else
   	   return;
	delayms(200);
	//С���˶����Ʋ���
	switch(command)
   	{
		case 'a':						//��ǰ
		{
			LeftGo;RightGo;
			break;
		}		
   		case 'b':						//���
		{
			LeftBack;RightBack;
			break;
		}
		case 'c':						//����
		{
			LeftBack;RightGo;
			break;
		}
		case 'd':						//����
		{
			LeftGo;RightBack;
			break;
		}
	 	case 'e':						//ֹͣ
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
		case 'f':						//�򿪵�
		{
	   		Light = 1;break;
		}
		case 'h':
		{
		    Light = 0;break;				//�رյ�
		}
		default:
		{
		 	break;
		}
	}
   ES = 1;								//�򿪴����ж�
}
    
/*----------------------------------�ⲿ�ж�0----------------------------------------*/
//function���½�������ʱ�����һ��
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

															 
/*------------------------------��ʪ����صĺ���------------------------------------*/
//------------------------------����ʪ������------------------------------------------
//function:���һ���ֽڵ�����				
unsigned char ReadOneData(void)   		//����һ��8λ���ݣ��ȸ�λ���λ
{
	unsigned char i;
	unsigned char dat = 0;
	for(i=0;i<8;i++)
	{
		DhtNum = 2;						//����whileѭ����������ʱ����ѭ��
		while((!DhtData)&&DhtNum++);	//dht_data��������DHT11���ͷ�������	
		Delay10Us();
		Delay10Us();
		Delay10Us();  					
		DhtTemp = 0;
		if(DhtData)DhtTemp=1;
		DhtNum = 2;
		while((DhtData)&&DhtNum++);
		//��ʱ������forѭ��
		if(DhtNum==1)break;
		dat<<=1;
		dat|=DhtTemp;
	}
	return dat;
}

//---------------------------------����ʪ������-------------------------------------
void GetHumiture(void)
{
	DhtData = 0;						//��Ƭ������������
	DelayDht(180);
	DhtData = 1;						//��Ƭ������ʼ�����ź�
	Delay10Us();
	Delay10Us();
	Delay10Us();
	Delay10Us();
	DhtData = 1;						//������ʱ���ȴ�DHT11������Ӧ����ӦΪ�͵�ƽ��
	
	if(!DhtData)						//����Ӧ�ͽ������ݣ�����������
	{
	   DhtNum = 2;
	   //�жϴӻ��Ƿ񷢳�80us�ĵ͵�ƽ��Ӧ�Ƿ����
	   while((!DhtData)&&DhtNum++);
	   DhtNum = 2;
	   //�жϴӻ��Ƿ񷢳�80us�ĸߵ�ƽ����������������ݽ���״̬
	   while((DhtData)&&DhtNum++);
	   //���ݽ���״̬
	   RHtemp = ReadOneData();
	   RLtemp = ReadOneData(); 
	   THtemp = ReadOneData();
	   TLtemp = ReadOneData();
	   CheckDht = ReadOneData();
	   DhtData = 1;						//�����������������ߣ��������״̬

	   CheckNum=(RHtemp+RLtemp+THtemp+TLtemp);
	   if(CheckDht == CheckNum)
	   {
		  RH = RHtemp;
		  RL = RLtemp;
		  TH = THtemp;
		  TL = TLtemp;
	   }
	}
	delayms(2);							//������ʱ����	 
}
//----------------------------------�����������������-----------------------------------
void WaveStart(void)
{
    //��ʱ600ms���
	TRIG = 1;	   //����������ģ��
	//��ʱ10us,ʹTRIG�����ʱ��ά�ָߵ�ƽ
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();
	TRIG = 0; 
}

//-----------------------------------�����������������-----------------------------------		
void DisCount(void)
{

	 S = (wave_count*1.7)/10;				//������ĵ�λΪCM
	 disbuffer[0] = S%1000/100;		
	 disbuffer[1] = S%1000%100/10;
	 disbuffer[2] = S%1000%10%10; 
	 disbuffer[3] = 0;
}


//ms�������ʱ����
void delayms(unsigned int t)
{
	 unsigned char temp; 
	 while(t--)
	 {
		 temp = 245;
		 while(temp--);
	 }
}
