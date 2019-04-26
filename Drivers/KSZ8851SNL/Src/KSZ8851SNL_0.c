#include "ksz8851snl_0.h"

/**
 * @brief KSZ8851 initialization
 * @param None
 * @return None
 */
void ksz8851_init_0(void)
{
  uint8_t MAC_address[] = { 0x00, 0x80, 0xe1, 0x00, 0x00, 0x01 };

  ksz8851_init(&KSZ8851_interface_0, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0, 0, MAC_address);
}
