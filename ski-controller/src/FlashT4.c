// FlashT4.c
// Teensy 4.x flash write and sector erase for FlashTxx.c. Copied from the
// Teensy core's cores/teensy4/eeprom.c (Teensyduino 1.62), where they are
// public as eepromemu_flash_*(). The core this project is pinned to
// (platform teensy@4.12.0, Teensyduino 1.53) has them as static functions,
// so FlasherX cannot reach them there.
//
// On Teensy 4 code runs from ITCM (RAM) unless marked FLASHMEM, so these keep
// running while flash is busy being written.

#if defined(__IMXRT1062__)

#include "imxrt.h"
#include <string.h>
#include "FlashTxx.h"

#define LUT0(opcode, pads, operand) (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)))
#define LUT1(opcode, pads, operand) (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)) << 16)
#define CMD_SDR         FLEXSPI_LUT_OPCODE_CMD_SDR
#define ADDR_SDR        FLEXSPI_LUT_OPCODE_RADDR_SDR
#define READ_SDR        FLEXSPI_LUT_OPCODE_READ_SDR
#define WRITE_SDR       FLEXSPI_LUT_OPCODE_WRITE_SDR
#define PINS1           FLEXSPI_LUT_NUM_PADS_1
#define PINS4           FLEXSPI_LUT_NUM_PADS_4

static void flash_wait()
{
	FLEXSPI_LUT60 = LUT0(CMD_SDR, PINS1, 0x05) | LUT1(READ_SDR, PINS1, 1); // 05 = read status
	FLEXSPI_LUT61 = 0;
	uint8_t status;
	do {
		FLEXSPI_IPRXFCR = FLEXSPI_IPRXFCR_CLRIPRXF; // clear rx fifo
		FLEXSPI_IPCR0 = 0;
		FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(15) | FLEXSPI_IPCR1_IDATSZ(1);
		FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
		while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE)) {;}
		FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE;
		asm("":::"memory");
		status = *(uint8_t *)&FLEXSPI_RFDR0;
	} while (status & 1);
	FLEXSPI_MCR0 |= FLEXSPI_MCR0_SWRESET; // purge stale data from FlexSPI's AHB FIFO
	while (FLEXSPI_MCR0 & FLEXSPI_MCR0_SWRESET) ; // wait
	__enable_irq();
}

// write bytes into flash memory (which is already erased to 0xFF)
void flashtxx_t4_write(void *addr, const void *data, uint32_t len)
{
	__disable_irq();
	FLEXSPI_LUTKEY = FLEXSPI_LUTKEY_VALUE;
	FLEXSPI_LUTCR = FLEXSPI_LUTCR_UNLOCK;
	FLEXSPI_IPCR0 = 0;
	FLEXSPI_LUT60 = LUT0(CMD_SDR, PINS1, 0x06); // 06 = write enable
	FLEXSPI_LUT61 = 0;
	FLEXSPI_LUT62 = 0;
	FLEXSPI_LUT63 = 0;
	FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(15);
	FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
	arm_dcache_delete(addr, len); // purge old data from ARM's cache
	while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE)) ; // wait
	FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE;
	FLEXSPI_LUT60 = LUT0(CMD_SDR, PINS1, 0x32) | LUT1(ADDR_SDR, PINS1, 24); // 32 = quad write
	FLEXSPI_LUT61 = LUT0(WRITE_SDR, PINS4, 1);
	FLEXSPI_IPTXFCR = FLEXSPI_IPTXFCR_CLRIPTXF; // clear tx fifo
	FLEXSPI_IPCR0 = (uint32_t)addr & 0x00FFFFFF;
	FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(15) | FLEXSPI_IPCR1_IDATSZ(len);
	FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
	const uint8_t *src = (const uint8_t *)data;
	uint32_t n;
	while (!((n = FLEXSPI_INTR) & FLEXSPI_INTR_IPCMDDONE)) {
		if (n & FLEXSPI_INTR_IPTXWE) {
			uint32_t wrlen = len;
			if (wrlen > 8) wrlen = 8;
			if (wrlen > 0) {
				memcpy((void *)&FLEXSPI_TFDR0, src, wrlen);
				src += wrlen;
				len -= wrlen;
			}
			FLEXSPI_INTR = FLEXSPI_INTR_IPTXWE;
		}
	}
	FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE | FLEXSPI_INTR_IPTXWE;
	flash_wait();
}

// erase a 4K sector
void flashtxx_t4_erase_sector(void *addr)
{
	__disable_irq();
	FLEXSPI_LUTKEY = FLEXSPI_LUTKEY_VALUE;
	FLEXSPI_LUTCR = FLEXSPI_LUTCR_UNLOCK;
	FLEXSPI_LUT60 = LUT0(CMD_SDR, PINS1, 0x06); // 06 = write enable
	FLEXSPI_LUT61 = 0;
	FLEXSPI_LUT62 = 0;
	FLEXSPI_LUT63 = 0;
	FLEXSPI_IPCR0 = 0;
	FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(15);
	FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
	arm_dcache_delete((void *)((uint32_t)addr & 0xFFFFF000), 4096); // purge data from cache
	while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE)) ; // wait
	FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE;
	FLEXSPI_LUT60 = LUT0(CMD_SDR, PINS1, 0x20) | LUT1(ADDR_SDR, PINS1, 24); // 20 = sector erase
	FLEXSPI_IPCR0 = (uint32_t)addr & 0x00FFF000;
	FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(15);
	FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
	while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE)) ; // wait
	FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE;
	flash_wait();
}

#endif // __IMXRT1062__
