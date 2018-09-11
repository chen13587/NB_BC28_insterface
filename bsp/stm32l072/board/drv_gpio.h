/*
 * File      : gpio.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-08-29     Aubr.Cool      the first version
 */
#include <rthw.h>
#include <rtdevice.h>
#include <board.h>

#ifdef RT_USING_PIN

#include "stm32l0xx.h"

#define __RCC_GPIO_CLK_ENABLE(BIT)   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->IOPENR, BIT);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOCEN);\
                                        UNUSED(tmpreg); \
                                      } while(0)
#define __RCC_GPIO_CLK_DISABLE(BIT)  CLEAR_BIT(RCC->IOPENR, BIT)

#define __STM32_PIN(index, gpio, gpio_index) {index, GPIO##gpio##_CLK_ENABLE, GPIO##gpio, GPIO_PIN_##gpio_index}

#define __STM32_PIN_DEFAULT {-1, 0, 0}

#define DRV_POWER_5V_PORT    GPIOB
#define DRV_POWER_5V_PIN     GPIO_PIN_13
#define DRV_POWER_24_PORT     GPIOB
#define DRV_POWER_24_PIN      GPIO_PIN_12


//ʹ����
//flash
#define PIN_CS_W      32    //W25Q8 CSƬѡ
#define PIN_MISO_W    33    //W25Q8 MISO
#define PIN_WP_W      34    //W25Q8 WP
#define PIN_MOSI_W    35    //W25Q8 MOSI
#define PIN_POW_FLASH 38    //W25Q8 ��Դ
#define PIN_HOLD_W    37    //W25Q8 ��д���� 
#define PIN_SCK_W     36    //W25Q8 SCK

//ˮѹ
#define PIN_POW_WT    3     //��ˮ����Դ
#define PIN_WT1       4     //��ˮ1
#define PIN_WT2       5     //��ˮ2
#define PIN_ADC_IN1   16    //4~20mA����1
#define PIN_ADC_IN2   17    //4~20mA����2

//����
#define PIN_POW_GAS   10    //�����Դ
#define PIN_GAS_IN    11    //��������
#define PIN_AD_GAS    15    //����ad

//����
#define PIN_TX2_NOS   25    //����2 ��������
#define PIN_RX2_NOS   26    //����2 ��������
#define PIN_POW_NOS   29    //������Դ

//��Դ��ѹ���
#define PIN_BAT_UP    30    //��Դ��ѹ���AD��
#define PIN_BAT_DO    31    //��Դ��ѹ��

//LED
#define PIN_LED1      98    //LED1
#define PIN_LED2      97    //LED2

//��ʪ��
#define PIN_POW_TH    96    //��ʪ�ȵ�Դ
#define PIN_TH_SDA    95    //��ʪ�� I2C SDA
#define PIN_TH_SCL    93    //��ʪ�� I2C SCL

//GPS
#define PIN_POW_GPS   91    //GPS��Դ
#define PIN_TX5_GPS   90    //����5 GPS����
#define PIN_RX5_GPS   89    //����5 GPS����
#define PIN_RST_GPS   88    //GPS��λ
#define PIN_T1S_GPS   87    //GPS 1������
#define PIN_EXT_GPS   86    //GPS �͹��Ļ���

//����
#define PIN_RX5_BLE   83    //����5 ��������
#define PIN_EN_BLE    82    //������Դ
#define PIN_RST_BLE   81    //������λ
#define PIN_TX5_BLE   80    //����5 ��������

//NB
#define PIN_RXL1_NB   79    //�͹��Ĵ���1 NB����
#define PIN_TXL1_NB   78    //�͹��Ĵ���1 NB����
#define PIN_RES_MCU   77    //NB ��λMCU
#define PIN_ON_OFF    71    //NB PWK

//����
#define PIN_VDD_ADXL  64    //�����Դ
#define PIN_SCLK_AD   63    //���� SCLK
#define PIN_MOSI_AD   62    //���� MOSI
#define PIN_MISO_AD   61    //���� MISO 
#define PIN_CS_AD     60    //���� CS
#define PIN_INT2_AD   59    //���� �ж�2
#define PIN_INT1_AD   58    //���� �ж�1

//�ƴ�
#define PIN_LED_CTR   48    //�ƴ�CTR

//5V��Դ
#define PIN_POW_5V    52    //5V��Դ

//24V��Դ
#define PIN_POW_24V   51    //24V��Դ

//����
#define PIN_RX4_T     40    //����4 ���Խ���
#define PIN_TX4_T     39    //����4 ���Է���


//δʹ��
#define PIN_OSC32_IN  8     //�ⲿ32.768kʱ��
#define PIN_OSC32_OUT 9
#define PIN_OSC_IN    12    //�ⲿ8Mʱ��
#define PIN_OSC_OUT   13

#define PIN_SWDIO     72    //��д�� SWDIO��
#define PIN_SWCLK     76    //��д�� SWCLK��

#define PIN_TX4_SP    23    //����4 ���÷���
#define PIN_RX4_SP    24    //����4 ���ý���

#define PIN_RX1_485   69    //����1 485����
#define PIN_TX1_485   68    //����1 485����
#define PIN_485_C     67    //485 Ƭѡ
#define PIN_BUZ       65    //������

#define PIN_RXL1_SP   56    //�͹��Ĵ���1 ���ý���
#define PIN_TXL1_SP   55    //�͹��Ĵ���1 ���÷���

#define PIN_RFID_MISO 46    //RFID MISO
#define PIN_RFID_MOSI 45    //RFID MOSI
#define PIN_RFID_CLK  44    //RFID CLK
#define PIN_RFID_SS   43    //RFID SS
#define PIN_RFIS_SSI  42    //RFID SSi
#define PIN_POW_RFID  41    //RFID ��Դ

#define PIN_RESERVE1  1
#define PIN_RESERVE2  2
#define PIN_RESERVE3  7
#define PIN_RESERVE4  18
#define PIN_RESERVE5  92
#define PIN_RESERVE6  85
#define PIN_RESERVE7  84
#define PIN_RESERVE8  70
#define PIN_RESERVE9  66
#define PIN_RESERVE10 57
#define PIN_RESERVE11 54
#define PIN_RESERVE12 53
#define PIN_RESERVE13 47

#define ITEM_NUM(items) sizeof(items)/sizeof(items[0])


#endif
