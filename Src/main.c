/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "cryp.h"
#include "dma.h"
#include "fatfs.h"
#include "fdcan.h"
#include "hash.h"
#include "i2c.h"
#include "iwdg.h"
#include "lwip.h"
#include "quadspi.h"
#include "rng.h"
#include "rtc.h"
#include "sdmmc.h"
#include "spi.h"
//#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dmc_fat.h"
#include <dmc_rtc.h>
#include <dmc_leds.h>
#include <dmc_dipswitch.h>
#include <dmc_mcu.h>
#include <dmc_net.h>
#include <dmc_print.h>
#include <dmc_rtc_mcp79412.h>
#include <KSZ8851SNL_0.h>
#include <KSZ8851SNL_1.h>
#include <string.h>
#include <dmc_tcp.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   (uint8_t) 192
#define IP_ADDR1   (uint8_t) 168
#define IP_ADDR2   (uint8_t) 25
#define IP_ADDR3   (uint8_t) 238

/*NETMASK*/
#define NETMASK_ADDR0   (uint8_t) 255
#define NETMASK_ADDR1   (uint8_t) 255
#define NETMASK_ADDR2   (uint8_t) 255
#define NETMASK_ADDR3   (uint8_t) 0

/*Gateway Address*/
#define GW_ADDR0   (uint8_t) 192
#define GW_ADDR1   (uint8_t) 168
#define GW_ADDR2   (uint8_t) 25
#define GW_ADDR3   (uint8_t) 253

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
struct netif gnetif;
static void Netif_Config(void);

/* USER CODE BEGIN PV */
uint32_t msTick = 0;
uint32_t msTickPrevious = 0;
uint32_t msTickPrevious2 = 0;
uint32_t Interval = 500;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void HAL_IncTicks(void);
void HAL_StartTicks(void);
uint8_t HAL_GetTicks(uint32_t ms);
void HAL_StartTicks2(void);
uint8_t HAL_GetTicks2(uint32_t ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t pTXData[4096] = { 0, };
//uint8_t pTXData[2048] __attribute__((section(".dma_buffer"))) = { 0, };
//uint8_t pRXData[4096] = { 0, };
//uint8_t pRXData[2048] __attribute__((section(".dma_buffer"))) = { 0, };

// For DMA1, buffers should be located in RMA-D2 memory (0x30000000)
// To work correctly with DMA, the memory must be 32-bit aligned
#define __SECTION_RAM_D2 __attribute__ ((section(".RAM_D2"))) __attribute__ ((aligned (32))) /* AHB SRAM (D2 domain): */
//uint8_t pTXData[ETH_MAX_PACKET_SIZE] __SECTION_RAM_D2;
//uint8_t pRXData[ETH_MAX_PACKET_SIZE] __SECTION_RAM_D2;

#if defined ( __CC_ARM )  /* MDK ARM Compiler */

#elif defined ( __GNUC__ ) /* GNU Compiler */

// https://community.st.com/s/article/FAQ-Ethernet-not-working-on-STM32H7x3
// Please check STM32H753VI_FLASH.ld
//uint8_t pRXData[ETH_MAX_PACKET_SIZE] __ALIGNED(32) __attribute__ ((section(".MicrelRxArraySection"))); /* Ethernet Receive Buffer */
//uint8_t pTXData[ETH_MAX_PACKET_SIZE] __ALIGNED(32) __attribute__ ((section(".MicrelTxArraySection"))); /* Ethernet Transmit Buffer */

//uint8_t pRXData[ETH_MAX_PACKET_SIZE] __attribute__ ((section(".RAM_D2"))) __attribute__ ((aligned (32))); /* Ethernet Receive Buffer */

uint8_t pRXData[ETH_MAX_PACKET_SIZE] __attribute__ ((section(".dma_buffer"))) __attribute__ ((aligned (32))); /* Ethernet Receive Buffer */


#endif

/* Global Vars */
RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
time_t timestamp;
struct tm currTime;

volatile uint32_t ping_time = 0;

#define ARP_CACHE_SIZE 16

struct KSZ8851ARP
{
  uint8_t IP[4];
  uint8_t MAC[6];
};

struct KSZ8851ARPCACHE
{
  struct KSZ8851ARP ArpTable[ARP_CACHE_SIZE];
  uint8_t ArpTableLength;
};

// If the data is a static or global variable, it is zero-filled by default, so just declare it
struct KSZ8851ARPCACHE ArpCache = { 0, };


uint8_t ARP_Delete(uint8_t *IP)
{
  // Find ARP entry and delete it
  for (uint8_t i = 0; i < ARP_CACHE_SIZE; i++)
  {
    if (memcmp(ArpCache.ArpTable[i].IP, IP, 4) == 0)
    {
      memset(ArpCache.ArpTable[i].IP, 0, 4);
      memset(ArpCache.ArpTable[i].MAC, 0, 6);
      return TRUE; // We have deleted it!
    }
  }
  return FALSE;
}

uint8_t ARP_Add(uint8_t *IP, uint8_t *MAC)
{
  // Look if the ARP entry is there already
  for (uint8_t i = 0; i < ARP_CACHE_SIZE; i++)
  {
    if (memcmp(ArpCache.ArpTable[i].IP, IP, 4) == 0)
    {
      return FALSE; // We have it already!
    }
  }
  // Find empty spot in the ARP table
  uint8_t IP_zero[] = { 0, 0, 0, 0 };
  for (uint8_t i = 0; i < ARP_CACHE_SIZE; i++)
  {
    if (memcmp(ArpCache.ArpTable[i].IP, IP_zero, 4) == 0)
    {
      memcpy(ArpCache.ArpTable[i].IP, IP, 4);
      memcpy(ArpCache.ArpTable[i].MAC, MAC, 6);
    }
  }
  // Count used ARP entries
  ArpCache.ArpTableLength = 0;
  for (uint8_t i = 0; i < ARP_CACHE_SIZE; i++)
  {
    if (memcmp(ArpCache.ArpTable[i].IP, IP_zero, 4) != 0)
    {
      ArpCache.ArpTableLength++;
    }
  }
  return TRUE;
}

uint8_t ARP_GetMACfromIP(uint8_t *IP, uint8_t *MAC)
{
  // Find and return ARP entry
  for (uint8_t i = 0; i < ARP_CACHE_SIZE; i++)
  {
    if (memcmp(ArpCache.ArpTable[i].IP, IP, 4) == 0)
    {
      memcpy(MAC, ArpCache.ArpTable[i].MAC, 6);
      return TRUE;
    }
  }
  return FALSE;
}

// Convert epoch time to Date/Time structures
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
  uint32_t tm;
  uint32_t t1;
  uint32_t a;
  uint32_t b;
  uint32_t c;
  uint32_t d;
  uint32_t e;
  uint32_t m;
  int16_t year = 0;
  int16_t month = 0;
  int16_t dow = 0;
  int16_t mday = 0;
  int16_t hour = 0;
  int16_t min = 0;
  int16_t sec = 0;
  uint64_t JD = 0;
  uint64_t JDN = 0;

  // These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

  JD = ((epoch + 43200) / (86400 >> 1)) + (2440587 << 1) + 1;
  JDN = JD >> 1;

  tm = epoch;
  t1 = tm / 60;
  sec = tm - (t1 * 60);
  tm = t1;
  t1 = tm / 60;
  min = tm - (t1 * 60);
  tm = t1;
  t1 = tm / 24;
  hour = tm - (t1 * 24);

  dow = JDN % 7;
  a = JDN + 32044;
  b = ((4 * a) + 3) / 146097;
  c = a - ((146097 * b) / 4);
  d = ((4 * c) + 3) / 1461;
  e = c - ((1461 * d) / 4);
  m = ((5 * e) + 2) / 153;
  mday = e - (((153 * m) + 2) / 5) + 1;
  month = m + 3 - (12 * (m / 10));
  year = (100 * b) + d - 4800 + (m / 10);

  date->Year = year - 2000;
  date->Month = month;
  date->Date = mday;
  date->WeekDay = dow;
  time->Hours = hour;
  time->Minutes = min;
  time->Seconds = sec;
}

uint8_t CheckMACIP(uint8_t *pRXData)
{
  // Check IP is for Modbus master or slave device
  uint8_t valid = 0;
  if ((pRXData[0] == 0x00) && (pRXData[1] == 0x30) && (pRXData[2] == 0xdb) && (pRXData[3] == 0x02) && (pRXData[4] == 0xaf) && (pRXData[5] == 0x62))
  {
//    dmc_puts("A");
    valid = 1;
  }
  if ((pRXData[0] == 0x00) && (pRXData[1] == 0x30) && (pRXData[2] == 0xdb) && (pRXData[3] == 0x02) && (pRXData[4] == 0xaf) && (pRXData[5] == 0x62))
  {
//    dmc_puts("B");
    valid = 1;
  }
  if ((pRXData[6] == 0x00) && (pRXData[7] == 0x30) && (pRXData[8] == 0xdb) && (pRXData[9] == 0x02) && (pRXData[10] == 0xb0) && (pRXData[11] == 0x15))
  {
//    dmc_puts("C");
    valid = 1;
  }
  if ((pRXData[6] == 0x00) && (pRXData[7] == 0x30) && (pRXData[8] == 0xdb) && (pRXData[9] == 0x02) && (pRXData[10] == 0xb0) && (pRXData[11] == 0x15))
  {
//    dmc_puts("D");
    valid = 1;
  }
  // ARP
  if ((pRXData[12] == 0x08) && (pRXData[13] == 0x06)) // ARP
  {
//    dmc_puts("a");
    valid = 1;
  }
  if ((pRXData[12] == 0x08) && (pRXData[13] == 0x00)) //
  {
//    dmc_puts("a");
    valid = 1;
  }
  if ((pRXData[26] == 0xc0) && (pRXData[27] == 0xa8) && (pRXData[28] == 0x19) && (pRXData[29] == 0xed))
  {
//    dmc_puts("A");
    valid = 1;
  }
  if ((pRXData[26] == 0xc0) && (pRXData[27] == 0xa8) && (pRXData[28] == 0x19) && (pRXData[29] == 0xef))
  {
//    dmc_puts("B");
    valid = 1;
  }
  if ((pRXData[26] == 0xc0) && (pRXData[27] == 0xa8) && (pRXData[28] == 0x19) && (pRXData[29] == 0x94)) // PC
  {
//    dmc_puts("B");
    valid = 1;
  }
  if ((pRXData[30] == 0xc0) && (pRXData[31] == 0xa8) && (pRXData[32] == 0x19) && (pRXData[33] == 0xed))
  {
//    dmc_puts("C");
    valid = 1;
  }
  if ((pRXData[30] == 0xc0) && (pRXData[31] == 0xa8) && (pRXData[32] == 0x19) && (pRXData[33] == 0xef))
  {
//    dmc_puts("D");
    valid = 1;
  }
  if ((pRXData[30] == 0xc0) && (pRXData[31] == 0xa8) && (pRXData[32] == 0x19) && (pRXData[33] == 0x94)) // PC
  {
//    dmc_puts("D");
    valid = 1;
  }

  valid = 1;

  return valid;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/

  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  DmcLedsOn();

  MX_UART7_Init();
  MX_USART2_UART_Init();

  HAL_GPIO_WritePin(UART7_TXE_GPIO_Port, UART7_TXE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(USART2_TXE_GPIO_Port, USART2_TXE_Pin, GPIO_PIN_SET);

  dmc_put_sethuart(&huart2);

  dmc_puts("\n");
  dmc_puts("--------------------------------------------------------------------------------\n");
  dmc_puts("MX_DMA_Init\n");
  MX_DMA_Init();
  dmc_puts("MX_QUADSPI_Init\n");
  MX_QUADSPI_Init();
//  dmc_puts("MX_IWDG1_Init\n");
//  MX_IWDG1_Init();
  dmc_puts("MX_SPI1_Init\n");
  MX_SPI1_Init();
  dmc_puts("MX_SPI4_Init\n");
  MX_SPI4_Init();
  dmc_puts("MX_FDCAN1_Init\n");
  MX_FDCAN1_Init();
  dmc_puts("MX_FDCAN2_Init\n");
  MX_FDCAN2_Init();
  dmc_puts("MX_I2C4_Init\n");
  MX_I2C4_Init();
  dmc_puts("MX_RTC_Init\n");
  MX_RTC_Init();
//  dmc_puts("MX_CRYP_Init\n");
//  MX_CRYP_Init();
//  dmc_puts("MX_HASH_Init\n");
//  MX_HASH_Init();
//  dmc_puts("MX_RNG_Init\n");
//  MX_RNG_Init();
//  dmc_puts("MX_CRC_Init\n");
//  MX_CRC_Init();
  dmc_puts("MX_FATFS_Init\n");
  MX_FATFS_Init();

//  dmc_puts("MX_SDMMC1_SD_Init\n");
//  MX_SDMMC1_SD_Init();  // Problem
  dmc_puts("MX_LWIP_Init\n");
  MX_LWIP_Init();  // Problem
//  MX_USB_HOST_Init();  // Problem

  dmc_puts("Done\n");
  HAL_Delay(1000);

  /* USER CODE BEGIN 2 */

  dmc_puts("HAL_StartTicks\n");
  HAL_StartTicks();


  uint8_t s1 = ReadDipSwitches();
  dmc_puts("dipswitch: ");
  dmc_puthex2cr(s1);


  HAL_GPIO_TogglePin(GPIOC, LED_RUN_Pin|LED_CAN1_OK_Pin|LED_CAN2_OK_Pin);
  HAL_GPIO_TogglePin(GPIOA, LED_RS1_OK_Pin|LED_RS1_ERR_Pin|LED_RS2_OK_Pin|LED_RS2_ERR_Pin);

  dmc_puts("--------------------------------------------------------------------------------\n");
  dmc_puts("MCU family     : ");
  dmc_puts(GetMCUFamily());
  dmc_puts("\n");
  dmc_puts("MCU type       : ");
  dmc_puts(GetMCUType());
  dmc_puts("\n");
  uint32_t SystemCoreClockMHz = SystemCoreClock / 1000000;
  dmc_putstrintstr("SystemCoreClock: ", SystemCoreClockMHz, " MHz\n");     // 400000000 Hz

  DMC_I2cRtcInit(hi2c4);

  // RTC
  char * DaysOfWeek[] = { "", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

  struct DMC_MCU_RTC_DATE_TIME DateTime;
  DateTime.Second = 50;
  DateTime.Minute = 18;
  DateTime.Hour   = 07;
  DateTime.DayOfMonth = 18;
  DateTime.Month = 3;
  DateTime.Year = 19;
  DateTime.DayOfWeek = 0; // Not required when setting date, it gets calculated anyway

  // Set RTC
  uint8_t SetDateTime = 1;
  if (SetDateTime)
  {
    DMC_I2cRtcSetDateTime(&DateTime);
  }

  DmcLedsOff();

  HAL_Delay(1000);

  ksz8851_init_0();
  ksz8851_init_1();

  /* USER CODE END 2 */
  uint32_t NTP_Interval = 2000;
  uint16_t pRXLength;

  HAL_StartTicks();
  HAL_StartTicks2();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_IWDG_Refresh(&hiwdg1);

    if (HAL_GetTicks2(500))
    {
      DmcLedToggle(LED_RUN);
    }

    if (HAL_GetTicks(NTP_Interval))
    {
      DmcLedToggle(LED_RS1_OK);
    }

//    if(ksz8851_has_data_0())
    {
      // Executed every 500 mS

      pRXLength = ksz8851_Receive(&KSZ8851_interface_1, pRXData, MAX_FRAMELEN);
      // Pass if packet is > size of headers: ETH(14) + IP(20) + TCP(20)
      if (pRXLength > MB_HDR_OFFSET)
      {
        // We need to do this for this IF, for a connected MB-master, but not for a connected MB-slave
        ksz8851snl_reset_rx(&KSZ8851_interface_1);

        if (CheckMACIP(pRXData))
        {
//          ksz8851snl_reset_tx(&KSZ8851_interface_0);
//          dmc_puts("0");
          ksz8851_Send(&KSZ8851_interface_0, pRXData, pRXLength);
        }

          // Some packet
//        dmc_puts("RX1 ");
//        dmc_puts(TERMINAL_MAGENTA);
//        for (uint16_t i = 0x00; i < 40; i++)
//        {
//          dmc_puthex2(pRXData[i]); // Length
//          dmc_putc(' ');
//        }
//        dmc_putc('\n');
//        dmc_puts(TERMINAL_DEFAULT);


//        uint8_t respond = 1;
//
//        // Check it is a TCP packet
//        if (pRXData[IP_HDR_OFFSET + IP_HDR_PROTOCOL] == IPPROTO_TCP)
//        {
//          // TCP
//          if (((pRXData[0x22] == 0x01) && (pRXData[0x23] == 0xf6)) || ((pRXData[0x24] == 0x01) && (pRXData[0x25] == 0xf6)))
//          {
////            dmc_puts("502 ");
//
//            respond = 1;
//
//            // Check Function Code and dip switches
//            uint8_t s1 = ReadDipSwitches();
//            if ((s1 > 0) && (pRXData[MB_HDR_OFFSET + MB_HDR_FC] == 0x10))
//            {
//              // Illegal FC, Do not send to slave, but return an error
//              swap_buf_all(pRXData);
//              pRXLength = mb_set_exception(pRXData, MB_EXCEPTION_ILLEGAL_FUNCTION);
//
//              uint8_t valid = 0;
//              for (uint i = 0; i < 6; i++)
//              {
//                if (pRXData[MB_HDR_OFFSET + i] != 0x00)
//                {
//                  valid = 1;
//                }
//              }
////              if (valid)
//              {
////              ksz8851snl_reset_tx(&KSZ8851_interface_0);
//              ksz8851_Send(&KSZ8851_interface_0, pRXData, pRXLength);
//              }
////              else
////              {
////                ksz8851snl_reset_rx(&KSZ8851_interface_1);
////                ksz8851snl_reset_rx(&KSZ8851_interface_0);
////              }
//
//              respond = 0;
//            }
//          }
//        }


//        if (respond)
//        {
//          uint8_t valid = 0;
////          for (uint i = 0; i < 6; i++)
////          {
////            if (pRXData[MB_HDR_OFFSET + i] != 0x00)
////            {
////              valid = 1;
////            }
////          }
//          // Check IP is for Modbus master or slave device
//          valid = 0;
//          if (CheckMACIP(pRXData))
//          {
//            valid = 1;
//          }
//          if (valid)
//          {
//            ksz8851snl_reset_tx(&KSZ8851_interface_0);
//            ksz8851_Send(&KSZ8851_interface_0, pRXData, pRXLength);
//          }
////          }
////          else
////          {
////            ksz8851snl_reset_rx(&KSZ8851_interface_1);
////            ksz8851snl_reset_rx(&KSZ8851_interface_0);
////          }
//        }
      }

      // Ignore first 11 bytes
      pRXLength = ksz8851_Receive(&KSZ8851_interface_0, pRXData, MAX_FRAMELEN);
//      RxData = &pRXData[0];
      // Pass if packet is > size of headers: ETH(14) + IP(20) + TCP(20)
      if (pRXLength > MB_HDR_OFFSET)
      {
        // We need to do this for this IF, for a connected MB-master, but not for a connected MB-slave
        ksz8851snl_reset_rx(&KSZ8851_interface_0);

        if (CheckMACIP(pRXData))
        {
//          ksz8851snl_reset_tx(&KSZ8851_interface_1);
//          dmc_puts("1");
          ksz8851_Send(&KSZ8851_interface_1, pRXData, pRXLength);
        }
//        // Some packet
//        dmc_puts("RX0 ");
//        dmc_puts(TERMINAL_MAGENTA);
//        for (uint16_t i = 0x00; i < 40; i++)
//        {
//          dmc_puthex2(pRXData[i]); // Length
//          dmc_putc(' ');
//        }
//        dmc_putc('\n');
//        dmc_puts(TERMINAL_DEFAULT);

//        // Check it is a TCP packet
//        if (pRXData[IP_HDR_OFFSET + IP_HDR_PROTOCOL] == IPPROTO_TCP)
//        {
//          // TCP
//          if (((pRXData[0x22] == 0x01) && (pRXData[0x23] == 0xf6)) || ((pRXData[0x24] == 0x01) && (pRXData[0x25] == 0xf6)))
//          {
//          }
//          dmc_putc('.');
//        }
//
//
//        uint8_t valid = 0;
////        for (uint i = 0; i < 6; i++)
////        {
////          if (pRXData[MB_HDR_OFFSET + i] != 0x00)
////          {
////            valid = 1;
////          }
////        }
//        // Check IP is for Modbus master or slave device
//        valid = 0;
//        if (CheckMACIP(pRXData))
//        {
//          valid = 1;
//        }
//
//        if (valid)
//        {
//          ksz8851snl_reset_tx(&KSZ8851_interface_1);
//          ksz8851_Send(&KSZ8851_interface_1, pRXData, pRXLength);
//        }
////        else
////        {
////          ksz8851snl_reset_rx(&KSZ8851_interface_0);
////          ksz8851snl_reset_rx(&KSZ8851_interface_1);
////        }


      }
//      memset(pRXData, 0, RxLength);
//      RxLength = 0;
//      ksz8851snl_reset_rx();

    }


  }

}

/**
  * @brief  Setup the network interface
  * @param  None
  * @retval None
  */
static void Netif_Config(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;

#if LWIP_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else

  /* IP address default setting */
  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

#endif

  /* add the network interface */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

  /*  Registers the default network interface */
  netif_set_default(&gnetif);

  ethernet_link_status_updated(&gnetif);

#if LWIP_NETIF_LINK_CALLBACK
  netif_set_link_callback(&gnetif, ethernet_link_status_updated);
#endif
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Supply configuration update enable 
  */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
      | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USART2
      | RCC_PERIPHCLK_UART7 | RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_RNG | RCC_PERIPHCLK_SPI4
      | RCC_PERIPHCLK_SPI1 | RCC_PERIPHCLK_SDMMC | RCC_PERIPHCLK_I2C4 | RCC_PERIPHCLK_USB
      | RCC_PERIPHCLK_QSPI;
//  PeriphClkInitStruct.PLL2.PLL2M = 4;
//  PeriphClkInitStruct.PLL2.PLL2N = 128;
//  PeriphClkInitStruct.PLL2.PLL2P = 2;
//  PeriphClkInitStruct.PLL2.PLL2Q = 4;
//  PeriphClkInitStruct.PLL2.PLL2R = 2;
//  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
//  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
//  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

//  RCC->AHB2ENR |= (RCC_AHB2ENR_D2SRAM1EN | RCC_AHB2ENR_D2SRAM2EN | RCC_AHB2ENR_D2SRAM3EN);   // Enable the SRAM

//  DMAMUX1_Channel0->CCR = 38;
//  DMAMUX1_Channel1->CCR = 37;
//  DMAMUX1_Channel2->CCR = 83;
//  DMAMUX1_Channel3->CCR = 84;

  SPI1->CR1 |= SPI_CR1_CSTART;
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI4->CR1 |= SPI_CR1_CSTART;
  SPI4->CR1 |= SPI_CR1_SPE;
}

/* USER CODE BEGIN 4 */
void HAL_IncTicks(void)
{
  msTick += (uint32_t) 1;
}

void HAL_StartTicks(void)
{
  msTickPrevious = msTick;
}

uint8_t HAL_GetTicks(uint32_t ms)
{
  if ((msTick - msTickPrevious) >= ms)
  {
    msTickPrevious = msTick;
    return TRUE;
  }
  return FALSE;
}

void HAL_StartTicks2(void)
{
  msTickPrevious2 = msTick;
}

uint8_t HAL_GetTicks2(uint32_t ms)
{
  if ((msTick - msTickPrevious2) >= ms)
  {
    msTickPrevious2 = msTick;
    return TRUE;
  }
  return FALSE;
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Initialize and configure the Region and the memory to be protected */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Initialize and configure the Region and the memory to be protected */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Initialize and configure the Region and the memory to be protected */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

//  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
//  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  DMC_ErrorHandler(file, line);
//
//  dmc_puts("_Error_Handler(");
//  dmc_puts(file);
//  dmc_puts(", ");
//  dmc_putint(line);
//  dmc_puts(")\n");
//  while (1)
//  {
////    printf("_Error_Handler(%s, %d)\n", file, line);
//  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
