#ifndef __VL53L0_I2C_H
#define __VL53L0_I2C_H

#include "main.h"
#include "gpio.h"
#include "vl53l0x_i2c.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK MiniV3 STM32������
//VL53L0X IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���?
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//IO��������



#define VL_SDA_IN()  SoftI2C_SDA_IN()
#define VL_SDA_OUT() SoftI2C_SDA_OUT()

#define T_SCL_PIN   GPIO_PIN_3
#define T_SCL_GPIO_Port GPIOA

#define T_SDA_PIN   GPIO_PIN_2
#define T_SDA_GPIO_Port GPIOA

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t

//IO��������	 
// #define VL_IIC_SCL    PAout(3)      //SCL
// #define VL_IIC_SCL=1    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET)//SCL=1;
// #define VL_IIC_SCL=0    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET)//SCL=0;
// #define VL_IIC_SDA=1    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET)//SDA=1;
// #define VL_IIC_SDA=0    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET)//SDA=1;
// #define VL_IIC_SDA    PAout(2) 		//SDA	 
// #define VL_READ_SDA   PAin(2) 		//����SDA 
#define VL_READ_SDA   HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)
//״̬
#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

//IIC��������
//void VL53L0X_i2c_init(void);//��ʼ��IIC��IO��

uint8_t VL53L0X_write_byte(uint8_t address,uint8_t index,uint8_t data);              //IICдһ��8λ����
uint8_t VL53L0X_write_word(uint8_t address,uint8_t index,uint16_t data);             //IICдһ��16λ����
uint8_t VL53L0X_write_dword(uint8_t address,uint8_t index,uint32_t data);            //IICдһ��32λ����
uint8_t VL53L0X_write_multi(uint8_t address, uint8_t index,uint8_t *pdata,uint16_t count);//IIC����д

uint8_t VL53L0X_read_byte(uint8_t address,uint8_t index,uint8_t *pdata);             //IIC��һ��8λ����
uint8_t VL53L0X_read_word(uint8_t address,uint8_t index,uint16_t *pdata);            //IIC��һ��16λ����
uint8_t VL53L0X_read_dword(uint8_t address,uint8_t index,uint32_t *pdata);           //IIC��һ��32λ����
uint8_t VL53L0X_read_multi(uint8_t address,uint8_t index,uint8_t *pdata,uint16_t count);  //IIC������


#endif 

// #ifndef delay_us
// #define delay_us
// void delay_us(uint32_t n)
// {
//     uint32_t ticks;
//     uint32_t told;
//     uint32_t tnow;
//     uint32_t tcnt = 0;
//     uint32_t reload;
       
// 	reload = SysTick->LOAD;                
//     ticks = n * (SystemCoreClock / 1000000);	 /* ?????? */  
    
//     tcnt = 0;
//     told = SysTick->VAL;             /* ????????? */
 
//     while (1)
//     {
//         tnow = SysTick->VAL;    
//         if (tnow != told)
//         {    
//             /* SYSTICK????????? */    
//             if (tnow < told)
//             {
//                 tcnt += told - tnow;    
//             }
//             /* ?????? */
//             else
//             {
//                 tcnt += reload - tnow + told;    
//             }        
//             told = tnow;
 
//             /* ????/????????,??? */
//             if (tcnt >= ticks)
//             {
//             	break;
//             }
//         }  
//     }
// } 
// #endif
