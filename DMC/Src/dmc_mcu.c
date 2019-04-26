#include "dmc_mcu.h"

uint32_t Get_RCC_CSR(void)
{
	uint32_t RCC_CSR = RCC->CSR;
	/* Clear source Reset Flags */
	__HAL_RCC_CLEAR_RESET_FLAGS();
    return RCC_CSR;
}

void Show_RCC_CSR(uint32_t RCC_CSR)
{
	if (RCC_CSR & (1 << 0))
	{
		dmc_puts("LSI RC oscillator ON\n");
	}
	if (RCC_CSR & (1 << 1))
	{
		dmc_puts("LSI RC oscillator ready\n");
	}
	if (RCC_CSR & (1 << 24))
	{
		dmc_puts("Clear the reset flags\n");
	}
	if (RCC_CSR & (1 << 25))
	{
		dmc_puts("POR/PDR or BOR reset occurred\n");
	}
	if (RCC_CSR & (1 << 26))
	{
		dmc_puts("Reset from NRST pin occurred\n");
	}
	if (RCC_CSR & (1 << 27))
	{
		dmc_puts("POR/PDR reset occurred\n");
	}
	if (RCC_CSR & (1 << 28))
	{
		dmc_puts("Software reset occurred\n");
	}
	if (RCC_CSR & (1 << 29))
	{
		dmc_puts("Watchdog reset occurred\n");
	}
	if (RCC_CSR & (1 << 30))
	{
		dmc_puts("Window watchdog reset occurred\n");
	}
	if (RCC_CSR & (1 << 31))
	{
		dmc_puts("Low-power management reset occurred\n");
	}
}

char* GetMCUFamily(void)
{
#if defined (STM32F0)
	return "STM32F0";
#endif
#if defined (STM32F1)
	return "STM32F1";
#endif
#if defined (STM32F2)
	return "STM32F2";
#endif
#if defined (STM32F3)
	return "STM32F3";
#endif
#if defined (STM32F4)
	return "STM32F4";
#endif
#if defined (STM32F7)
  return "STM32F7";
#endif
#if defined (STM32H7)
  return "STM32H7";
#endif
}

char* GetMCUType(void)
{
#if defined (STM32H743xx)
  return "STM32H743xx";
#endif
#if defined (STM32H753xx)
  return "STM32H753xx";
#endif
#if defined (STM32H750xx)
  return "STM32H750xx";
#endif
#if defined (STM32F722xx)
  return "STM32F722xx";
#endif
#if defined (STM32F723xx)
	return "STM32F723xx";
#endif
#if defined (STM32F730xx)
	return "STM32F730xx";
#endif
#if defined (STM32F732xx)
	return "STM32F732xx";
#endif
#if defined (STM32F733xx)
	return "STM32F733xx";
#endif
#if defined (STM32F745xx)
	return "STM32F745xx";
#endif
#if defined (STM32F746xx)
	return "STM32F746xx";
#endif
#if defined (STM32F750xx)
	return "STM32F750xx";
#endif
#if defined (STM32F756xx)
	return "STM32F756xx";
#endif
#if defined (STM32F767xx)
	return "STM32F767xx";
#endif
#if defined (STM32F769xx)
	return "STM32F769xx";
#endif
#if defined (STM32F777xx)
	return "STM32F777xx";
#endif
#if defined (STM32F779xx)
	return "STM32F779xx";
#endif
}
