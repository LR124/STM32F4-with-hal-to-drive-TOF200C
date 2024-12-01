#include "vl53l0x_i2c.h"


//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK MiniV3 STM32������
//VL53L0X IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

// VL53L0X I2C��ʼ��
// void VL53L0X_i2c_init(void)
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );	//ʹ��GPIOAʱ��
	 
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;    //�˿�����
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;       //�������
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //50Mhz�ٶ�
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);

// 	GPIO_SetBits(GPIOA,GPIO_Pin_2|GPIO_Pin_3);//PA2,PA3 �����	

// }

//����IIC��ʼ�ź�
void delay_us(uint32_t n)
{
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;
       
	reload = SysTick->LOAD;                
    ticks = n * (SystemCoreClock / 1000000);	 /* ?????? */  
    
    tcnt = 0;
    told = SysTick->VAL;             /* ????????? */
 
    while (1)
    {
        tnow = SysTick->VAL;    
        if (tnow != told)
        {    
            /* SYSTICK????????? */    
            if (tnow < told)
            {
                tcnt += told - tnow;    
            }
            /* ?????? */
            else
            {
                tcnt += reload - tnow + told;    
            }        
            told = tnow;
 
            /* ????/????????,??? */
            if (tcnt >= ticks)
            {
            	break;
            }
        }  
    }
} 
//因为SDA需要改变输入输出模式，这么一想王老师的书实在是太好了，已经给了软件i2c的代码
void SoftI2C_SDA_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct={0};
    GPIO_InitStruct.Pin=T_SDA_PIN;
	GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(T_SDA_GPIO_Port,&GPIO_InitStruct);
}

void SoftI2C_SDA_IN()
{//SDA设置为输入上拉
	GPIO_InitTypeDef  GPIO_InitStruct={0};
    GPIO_InitStruct.Pin=T_SDA_PIN;
	GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(T_SDA_GPIO_Port,&GPIO_InitStruct);
}

void VL_IIC_Start(void)
{
	VL_SDA_OUT();//sda�����
	//VL_IIC_SDA=1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
	//VL_IIC_SCL=1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	delay_us(4);
 	//VL_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	delay_us(4);
	//VL_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
}

//����IICֹͣ�ź�
void VL_IIC_Stop(void)
{
	VL_SDA_OUT();//sda�����
	//VL_IIC_SCL=0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	//VL_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
 	delay_us(4);
	//VL_IIC_SCL=1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	//VL_IIC_SDA=1;//����I2C���߽����ź�
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
	delay_us(4);							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t VL_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	VL_SDA_IN();  //SDA����Ϊ����  
	//VL_IIC_SDA=1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
	delay_us(1);	   
	//VL_IIC_SCL=1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	delay_us(1); 
	while(VL_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			VL_IIC_Stop();
			return 1;
		}
	}
	//VL_IIC_SCL=0;//ʱ�����0 
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);   
	return 0;  
}

//����ACKӦ��
void VL_IIC_Ack(void)
{
	//VL_IIC_SCL=0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	VL_SDA_OUT();
	//VL_IIC_SDA=0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	delay_us(2);
	//VL_IIC_SCL=1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	delay_us(2);
	//VL_IIC_SCL=0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
}

//������ACKӦ��		    
void VL_IIC_NAck(void)
{
	//VL_IIC_SCL=0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	VL_SDA_OUT();
	//VL_IIC_SDA=1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
	delay_us(2);
	//VL_IIC_SCL=1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	delay_us(2);
	//VL_IIC_SCL=0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
}

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void VL_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	VL_SDA_OUT(); 	    
    //VL_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
			//VL_IIC_SDA=1;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
		else
			//VL_IIC_SDA=0;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
		txd<<=1; 	  
		delay_us(2);  
		//VL_IIC_SCL=1;
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
		delay_us(2); 
		//VL_IIC_SCL=0;	
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
		delay_us(2);
    }	 
} 

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 VL_IIC_Read_Byte(void)
{
	unsigned char i,receive=0;
	VL_SDA_IN();//SDA����Ϊ����
	//VL_IIC_SDA = 1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
	delay_us(4);
	for(i=0;i<8;i++ )
	{
		receive<<=1;
		//VL_IIC_SCL=0; 
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
		delay_us(4);
	  //VL_IIC_SCL=1;
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
		delay_us(4);
		if(VL_READ_SDA)
			receive |= 0x01;   
	  delay_us(4); //1
	}	
  //VL_IIC_SCL = 0;	
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	return receive;
}

//IICдһ���ֽ�����
u8 VL_IIC_Write_1Byte(u8 SlaveAddress, u8 REG_Address,u8 REG_data)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);
	if(VL_IIC_Wait_Ack())
	{
		VL_IIC_Stop();//�ͷ�����
		return 1;//ûӦ�����˳�

	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();	
	VL_IIC_Send_Byte(REG_data);
	VL_IIC_Wait_Ack();	
	VL_IIC_Stop();

	return 0;
}

//IIC��һ���ֽ�����
u8 VL_IIC_Read_1Byte(u8 SlaveAddress, u8 REG_Address,u8 *REG_data)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);//��д����
	if(VL_IIC_Wait_Ack())
	{
		 VL_IIC_Stop();//�ͷ�����
		 return 1;//ûӦ�����˳�
	}		
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	VL_IIC_Start(); 
	VL_IIC_Send_Byte(SlaveAddress|0x01);//��������
	VL_IIC_Wait_Ack();
	*REG_data = VL_IIC_Read_Byte();
	VL_IIC_Stop();

	return 0;
}

//IICдn�ֽ�����
u8 VL_IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address,u16 len, u8 *buf)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);//��д����
	if(VL_IIC_Wait_Ack()) 
	{
		VL_IIC_Stop();//�ͷ�����
		return 1;//ûӦ�����˳�
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	while(len--)
	{
		VL_IIC_Send_Byte(*buf++);//����buff������
		VL_IIC_Wait_Ack();	
	}
	VL_IIC_Stop();//�ͷ�����

	return 0;
	
}

//IIC��n�ֽ�����
u8 VL_IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address,u16 len,u8 *buf)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);//��д����
	if(VL_IIC_Wait_Ack()) 
	{
		VL_IIC_Stop();//�ͷ�����
		return 1;//ûӦ�����˳�
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
  	//delay_ms(1);
	HAL_Delay(1);
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress|0x01);//��������
	VL_IIC_Wait_Ack();
	while(len)
	{
		*buf = VL_IIC_Read_Byte();
		if(len==1)
		{
			VL_IIC_NAck();
		}
		else
		{
			VL_IIC_Ack();
		}
		buf++;
		len--;
	}
	VL_IIC_Stop();//�ͷ�����

	return 0;
	
}

//VL53L0X д�������
//address:��ַ
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:���� ���65535
u8 VL53L0X_write_multi(u8 address, u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OK;

	if(VL_IIC_Write_nByte(address,index,count,pdata))
	{
	   status  = STATUS_FAIL;

	}

	return status;
}


//VL53L0X ���������
//address:��ַ
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:���� ���65535
u8 VL53L0X_read_multi(u8 address,u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OK;

	if(VL_IIC_Read_nByte(address,index,count,pdata))
	{
	  status  = STATUS_FAIL;
	}

	return status;
}

//VL53L0X д1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(8λ)
u8 VL53L0X_write_byte(u8 address,u8 index,u8 data)
{
	u8 status = STATUS_OK;

	status = VL53L0X_write_multi(address,index,&data,1);

	return status;
}

//VL53L0X д1������(˫�ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(16λ)
u8 VL53L0X_write_word(u8 address,u8 index,u16 data)
{
	u8 status = STATUS_OK;
	
	u8 buffer[2];
	
	//��16λ���ݲ�ֳ�8λ
	buffer[0] = (u8)(data>>8);//�߰�λ
	buffer[1] = (u8)(data&0xff);//�Ͱ�λ
	
	if(index%2==1)
	{  
		//����ͨ�Ų��ܴ����Է�2�ֽڶ���Ĵ������ֽ�
		status = VL53L0X_write_multi(address,index,&buffer[0],1);
		status = VL53L0X_write_multi(address,index,&buffer[0],1);
	}else
	{
		status = VL53L0X_write_multi(address,index,buffer,2);
	}
	
	return status;
}

//VL53L0X д1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(32λ)
u8 VL53L0X_write_dword(u8 address,u8 index,u32 data)
{
	
    u8 status = STATUS_OK;

    u8 buffer[4];	
	
	//��32λ���ݲ�ֳ�8λ
	buffer[0] = (u8)(data>>24);
	buffer[1] = (u8)((data&0xff0000)>>16);
	buffer[2] = (u8)((data&0xff00)>>8);
	buffer[3] = (u8)(data&0xff);
	
	status = VL53L0X_write_multi(address,index,buffer,4);
	
	return status;
	
}


//VL53L0X ��1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(8λ)
u8 VL53L0X_read_byte(u8 address,u8 index,u8 *pdata)
{
	u8 status = STATUS_OK;
	 
	status = VL53L0X_read_multi(address,index,pdata,1);
	
	return status;
	 
}

//VL53L0X ��������(˫�ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(16λ)
u8 VL53L0X_read_word(u8 address,u8 index,u16 *pdata)
{
	u8 status = STATUS_OK;
	
	u8 buffer[2];
	
	status = VL53L0X_read_multi(address,index,buffer,2);
	
	*pdata = ((u16)buffer[0]<<8)+(u16)buffer[1];
	
	return status;
	
}

//VL53L0X ��1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(32λ)
u8 VL53L0X_read_dword(u8 address,u8 index,u32 *pdata)
{
	u8 status = STATUS_OK;
	
	u8 buffer[4];
	
	status = VL53L0X_read_multi(address,index,buffer,4);
	
	*pdata = ((u32)buffer[0]<<24)+((u32)buffer[1]<<16)+((u32)buffer[2]<<8)+((u32)buffer[3]);
	
	return status;
	
}
