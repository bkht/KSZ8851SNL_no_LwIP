#include "ksz8851snl_1.h"

/**
 * @brief KSZ8851 initialization
 * @param None
 * @return None
 */
void ksz8851_init_1(void)
{
  uint8_t MAC_address[] = { 0x00, 0x80, 0xe1, 0x00, 0x00, 0x02 };

  ksz8851_init(&KSZ8851_interface_1, &hspi4, SPI4_CS_GPIO_Port, SPI4_CS_Pin, 0, 0, MAC_address);
}
