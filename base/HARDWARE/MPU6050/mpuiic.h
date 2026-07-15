#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"

//IO方向设置 设置SDA-PB9为输入或者输出
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}




////IO操作函数	 
//#define MPU_IIC_SCL    PBout(10) 		//SCL
//#define MPU_IIC_SDA    PBout(11) 		//SDA	 
//#define MPU_READ_SDA   PBin(11) 		//输入SDA 

//IO操作函数	 
#define MPU_IIC_SCL_Hige    	HAL_GPIO_WritePin(SCL_6050_GPIO_Port,SCL_6050_Pin,GPIO_PIN_SET)//设置SCL高电平
#define MPU_IIC_SCL_Low      	HAL_GPIO_WritePin(SCL_6050_GPIO_Port,SCL_6050_Pin,GPIO_PIN_RESET)//设置SCL低电平
#define MPU_IIC_SDA_Hige        HAL_GPIO_WritePin(SDA_6050_GPIO_Port,SDA_6050_Pin,GPIO_PIN_SET)   //设置SDA高电平
#define MPU_IIC_SDA_Low         HAL_GPIO_WritePin(SDA_6050_GPIO_Port,SDA_6050_Pin,GPIO_PIN_RESET) //设置SDA低电平
  	 
#define MPU_READ_SDA            HAL_GPIO_ReadPin(SDA_6050_GPIO_Port,SDA_6050_Pin)    //读SDA电平

//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
