/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		ICM20602
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		IAR 8.3 or MDK 5.24
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 * @note		
					���߶��壺
					------------------------------------ 
					SCL                 �鿴STM32F103ZETx HAL ICM20602 Hardware SPI.ioc�ļ�
					SDA                 �鿴STM32F103ZETx HAL ICM20602 Hardware SPI.ioc�ļ�
					------------------------------------ 
					
					
					MISO   SDO
					MOSI   SDI
					
 ********************************************************************************************************************/

#include "stm32f1xx_hal.h"
#include "icm20602.h"
#include "main.h"
extern SPI_HandleTypeDef hspi1;

int icm_gyro_x,icm_gyro_y,icm_gyro_z;
int icm_acc_x,icm_acc_y,icm_acc_z;




//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPIд�Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      val     ��Ҫд�������
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_w_reg_byte(uint8_t cmd, uint8_t val)
{
    uint8_t dat[2];
	HAL_GPIO_WritePin(SPI1_CS_PIN_GPIO_Port, SPI1_CS_PIN_Pin,GPIO_PIN_RESET);
	
    dat[0] = cmd | ICM20602_SPI_W;
    dat[1] = val;
    HAL_SPI_Transmit(&hspi1,dat,2,500);

	HAL_GPIO_WritePin(SPI1_CS_PIN_GPIO_Port, SPI1_CS_PIN_Pin,GPIO_PIN_SET);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI���Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      *val    �������ݵĵ�ַ
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------

void icm_spi_r_reg_byte(uint8_t cmd, uint8_t *val)
{
    uint8_t dat[2];

	HAL_GPIO_WritePin(SPI1_CS_PIN_GPIO_Port, SPI1_CS_PIN_Pin,GPIO_PIN_RESET);
    dat[0] = cmd | ICM20602_SPI_R;
    dat[1] = *val;
	HAL_SPI_TransmitReceive(&hspi1,dat,dat,2,500);
	
	HAL_GPIO_WritePin(SPI1_CS_PIN_GPIO_Port, SPI1_CS_PIN_Pin,GPIO_PIN_SET);

    *val = dat[1];
}
  
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI���ֽڶ��Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      *val    �������ݵĵ�ַ
//  @param      num     ��ȡ����
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_r_reg_bytes(uint8_t * val, uint8_t num)
{
	HAL_GPIO_WritePin(SPI1_CS_PIN_GPIO_Port, SPI1_CS_PIN_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,val,val,num,500);
	HAL_GPIO_WritePin(SPI1_CS_PIN_GPIO_Port, SPI1_CS_PIN_Pin,GPIO_PIN_SET);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602�Լ캯��
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self3_check(void)
{
    uint8_t dat=0;
    while(0x12 != dat)   //��ȡICM20602 ID
    {
        icm_spi_r_reg_byte(ICM20602_WHO_AM_I,&dat);
        HAL_Delay(10);
        //��������ԭ�������¼���
        //1 MPU6050���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
    }

}
     
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��ICM20602
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_spi(void)
{
    uint8_t val = 0x0;

    HAL_Delay(10);  //�ϵ���ʱ
    

    icm20602_self3_check();//���
    
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);//��λ�豸
    HAL_Delay(2);
    do
    {//�ȴ���λ�ɹ�
        icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
    }while(0x41 != val);
    
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01);            //ʱ������
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00);            //���������Ǻͼ��ٶȼ�
    icm_spi_w_reg_byte(ICM20602_CONFIG,         0x01);            //176HZ 1KHZ
    icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x07);            //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18);            //��2000 dps
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x10);            //��8g
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x03);            //Average 4 samples   44.8HZ   //0x23 Average 16 samples
	//ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ� ��λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_spi(void)
{
    struct
    {
        uint8_t reg;
        uint8_t dat[6];
    }buf;

    buf.reg = ICM20602_ACCEL_XOUT_H | ICM20602_SPI_R;
    
    icm_spi_r_reg_bytes(&buf.reg, 7);
    icm_acc_x = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    icm_acc_y = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    icm_acc_z = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_spi(void)
{
    struct
    {
        uint8_t reg;
        uint8_t dat[6];
    }buf;

    buf.reg = ICM20602_GYRO_XOUT_H | ICM20602_SPI_R;
    
    icm_spi_r_reg_bytes(&buf.reg, 7);
    icm_gyro_x = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    icm_gyro_y = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    icm_gyro_z = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
}







//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ICM20602����ת��Ϊ��������λ������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
float unit_icm_gyro_x;
float unit_icm_gyro_y;
float unit_icm_gyro_z;

float unit_icm_acc_x;
float unit_icm_acc_y;
float unit_icm_acc_z;


void icm20602_data_change(void)
{
    //ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ� ��λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
//    unit_icm_gyro_x = (float)icm_gyro_x/131;
//    unit_icm_gyro_y = (float)icm_gyro_y/131;
//    unit_icm_gyro_z = (float)icm_gyro_z/131;

    unit_icm_acc_x = (float)icm_acc_x/4096;
    unit_icm_acc_y = (float)icm_acc_y/4096;
    unit_icm_acc_z = (float)icm_acc_z/4096;

}


