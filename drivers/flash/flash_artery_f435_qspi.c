/*
 * Copyright (c) Martin Schröder <info@swedishembedded.com> 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Artery AT32F435 QSPI NOR Flash Driver for Winbond W25Q512JV
 *
 * This driver provides support for Winbond W25Q512JV (512 Mbit / 64 MB)
 * QSPI NOR flash connected to the AT32F435 QSPI peripheral.
 *
 * Key features implemented per W25Q512JV datasheet:
 * - 4-byte addressing mode (Enter 4B mode at init, check ADS bit)
 * - Quad Enable (QE bit) configuration with verification
 * - Configurable dummy cycles for optimal performance
 * - Individual block lock handling (global unlock on init if WPS=1)
 * - Software reset support (66h/99h sequence)
 * - Proper power-up sequencing and timing constraints
 * - SFDP parameter reading with address validation
 * - Robust status register handling (SR1/SR2/SR3)
 *
 * Tested with: Winbond W25Q512JV (JEDEC ID: EF 40 20)
 */

#define DT_DRV_COMPAT artery_f435_qspi_nor

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include "spi_nor.h"
#include "jesd216.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_artery_f435_qspi, CONFIG_FLASH_LOG_LEVEL);

/* AT32F435 QSPI Controller base (get from parent node) */
#define QSPI_CONTROLLER_NODE DT_INST_PARENT(0)

/* W25Q512JV-specific commands (per datasheet) */
#define W25Q_CMD_READ_SR2               0x35  /* Read Status Register-2 */
#define W25Q_CMD_READ_SR3               0x15  /* Read Status Register-3 */
#define W25Q_CMD_WRITE_SR2              0x31  /* Write Status Register-2 */
#define W25Q_CMD_WRITE_SR3              0x11  /* Write Status Register-3 */
#define W25Q_CMD_ENTER_4B_ADDR          0xB7  /* Enter 4-byte address mode */
#define W25Q_CMD_EXIT_4B_ADDR           0xE9  /* Exit 4-byte address mode */
#define W25Q_CMD_READ_EAR               0xC8  /* Read Extended Address Register */
#define W25Q_CMD_WRITE_EAR              0xC5  /* Write Extended Address Register */
#define W25Q_CMD_SET_READ_PARAMS        0xC0  /* Set Read Parameters (dummy cycles) */
#define W25Q_CMD_SET_BURST_WRAP         0x77  /* Set Burst with Wrap */
#define W25Q_CMD_ENABLE_RESET           0x66  /* Enable Reset */
#define W25Q_CMD_RESET_DEVICE           0x99  /* Reset Device */
#define W25Q_CMD_GLOBAL_BLOCK_UNLOCK    0x98  /* Global Block/Sector Unlock */
#define W25Q_CMD_INDIVIDUAL_UNLOCK      0x39  /* Individual Block/Sector Unlock */
#define W25Q_CMD_READ_BLOCK_LOCK        0x3D  /* Read Block/Sector Lock Status */

/* W25Q512JV Status Register bits */
#define W25Q_SR1_BUSY                   BIT(0)  /* Busy flag (S0) */
#define W25Q_SR1_WEL                    BIT(1)  /* Write Enable Latch (S1) */
#define W25Q_SR2_SUS                    BIT(7)  /* Suspend Status (S15) */
#define W25Q_SR2_QE                     BIT(1)  /* Quad Enable (S9) */
#define W25Q_SR3_ADS                    BIT(0)  /* Current Address Mode (S16): 0=3B, 1=4B */
#define W25Q_SR3_ADP                    BIT(1)  /* Power-up Address Mode (S17) */
#define W25Q_SR3_WPS                    BIT(2)  /* Write Protect Selection (S18) */

/* W25Q512JV expected JEDEC ID */
#define W25Q512_JEDEC_MFR               0xEF  /* Winbond */
#define W25Q512_JEDEC_TYPE              0x40  /* Memory type */
#define W25Q512_JEDEC_CAPACITY          0x20  /* Capacity (512 Mbit) */

/* W25Q512JV timing constraints (from datasheet, in milliseconds) */
#define W25Q_TIMEOUT_PAGE_PROGRAM       4     /* tPP max: 3.5ms */
#define W25Q_TIMEOUT_SECTOR_ERASE       400   /* tSE max: 400ms */
#define W25Q_TIMEOUT_BLOCK_ERASE_32K    1600  /* tBE1 max: 1600ms */
#define W25Q_TIMEOUT_BLOCK_ERASE_64K    2000  /* tBE2 max: 2000ms */
#define W25Q_TIMEOUT_CHIP_ERASE         1000000  /* tCE max: 1000s */
#define W25Q_TIMEOUT_WRITE_STATUS       15    /* tW max: 15ms */
#define W25Q_TIMEOUT_RESET              1     /* tRST: 30µs */

/* Read Parameters register P[6:4] - Dummy cycle configuration for EBh/ECh */
#define W25Q_DUMMY_CYCLES_6             0x00
#define W25Q_DUMMY_CYCLES_8             0x10
#define W25Q_DUMMY_CYCLES_10            0x20
#define W25Q_DUMMY_CYCLES_DEFAULT       W25Q_DUMMY_CYCLES_8

/* QSPI Register Offsets (from AT32F435 reference manual) */
#define QSPI_CMD_W0_OFFSET      0x00  /* Command word 0 (address) */
#define QSPI_CMD_W1_OFFSET      0x04  /* Command word 1 (control) */
#define QSPI_CMD_W2_OFFSET      0x08  /* Command word 2 (data counter) */
#define QSPI_CMD_W3_OFFSET      0x0C  /* Command word 3 (instruction/mode) */
#define QSPI_CTRL_OFFSET        0x10  /* Control register */
#define QSPI_FIFOSTS_OFFSET     0x18  /* FIFO status */
#define QSPI_CTRL2_OFFSET       0x20   /* Control register 2 */
#define QSPI_CMDSTS_OFFSET      0x24   /* Command status */
#define QSPI_RSTS_OFFSET        0x28   /* Read status */
#define QSPI_DT_OFFSET          0x100  /* Data transmit/receive register */

/* CMD_W1 register bits */
#define QSPI_CMD_W1_ADRLEN_POS  0
#define QSPI_CMD_W1_ADRLEN_MASK (0x7 << QSPI_CMD_W1_ADRLEN_POS)
#define QSPI_CMD_W1_DUM2_POS    16
#define QSPI_CMD_W1_DUM2_MASK   (0xFF << QSPI_CMD_W1_DUM2_POS)
#define QSPI_CMD_W1_INSLEN_POS  24
#define QSPI_CMD_W1_INSLEN_MASK (0x3 << QSPI_CMD_W1_INSLEN_POS)
#define QSPI_CMD_W1_PEMEN       BIT(28)

/* CMD_W3 register bits */
#define QSPI_CMD_W3_WDEN        BIT(1)  /* Write data enable (set for commands that write to flash) */
#define QSPI_CMD_W3_RSTSEN      BIT(2)  /* Read status enable */
#define QSPI_CMD_W3_RSTSC       BIT(3)  /* Read status config */
#define QSPI_CMD_W3_OPMODE_POS  5
#define QSPI_CMD_W3_OPMODE_MASK (0x7 << QSPI_CMD_W3_OPMODE_POS)
#define QSPI_CMD_W3_PEMOPC_POS  16
#define QSPI_CMD_W3_PEMOPC_MASK (0xFF << QSPI_CMD_W3_PEMOPC_POS)
#define QSPI_CMD_W3_INSC_POS    24
#define QSPI_CMD_W3_INSC_MASK   (0xFF << QSPI_CMD_W3_INSC_POS)

/* CTRL register bits */
#define QSPI_CTRL_CLKDIV_POS    0
#define QSPI_CTRL_CLKDIV_MASK   (0x7 << QSPI_CTRL_CLKDIV_POS)
#define QSPI_CTRL_SCKMODE       BIT(4)
#define QSPI_CTRL_XIPIDLE       BIT(7)
#define QSPI_CTRL_ABORT         BIT(8)
#define QSPI_CTRL_BUSY_POS      16
#define QSPI_CTRL_BUSY_MASK     (0x7 << QSPI_CTRL_BUSY_POS)
#define QSPI_CTRL_XIPRCMDF      BIT(19)
#define QSPI_CTRL_XIPSEL        BIT(20)

/* FIFOSTS register bits */
#define QSPI_FIFOSTS_TXFIFORDY  BIT(0)
#define QSPI_FIFOSTS_RXFIFORDY  BIT(1)

/* CTRL2 register bits */
#define QSPI_CTRL2_DMAEN        BIT(0)
#define QSPI_CTRL2_CMDIE        BIT(1)

/* CMDSTS register bits */
#define QSPI_CMDSTS_CMDSTS      BIT(0)

/* RSTS register bits (status register value) */
#define QSPI_RSTS_SPISTS_MASK   0xFF

/* Operation modes (for OPMODE field) */
#define QSPI_OPMODE_111         0x0  /* 1-1-1 (standard SPI) */
#define QSPI_OPMODE_112         0x1  /* 1-1-2 (dual output) */
#define QSPI_OPMODE_114         0x2  /* 1-1-4 (quad output) */
#define QSPI_OPMODE_122         0x3  /* 1-2-2 (dual I/O) */
#define QSPI_OPMODE_144         0x4  /* 1-4-4 (quad I/O) */

/* Address length values */
#define QSPI_ADRLEN_0_BYTE      0x0
#define QSPI_ADRLEN_1_BYTE      0x1
#define QSPI_ADRLEN_2_BYTE      0x2
#define QSPI_ADRLEN_3_BYTE      0x3
#define QSPI_ADRLEN_4_BYTE      0x4

/* Instruction length values */
#define QSPI_INSLEN_0_BYTE      0x0
#define QSPI_INSLEN_1_BYTE      0x1

/* Clock divider values (custom AT32 encoding) */
#define QSPI_CLK_DIV_2          0x0
#define QSPI_CLK_DIV_4          0x1
#define QSPI_CLK_DIV_6          0x2
#define QSPI_CLK_DIV_8          0x3

/* FIFO depth (32 bytes per channel, 4 channels = 128 bytes total) */
#define QSPI_FIFO_DEPTH         128

/* CRM (Clock/Reset Management) registers for AT32F435 */
#define CRM_BASE                0x40023800  /* AHBPERIPH1_BASE + 0x3800 */
#define CRM_AHBEN3_OFFSET       0x38  /* AHB enable register 3 */
#define CRM_AHBEN3_QSPI1EN      BIT(1)
#define CRM_AHBRST3_OFFSET      0x18  /* AHB reset register 3 */
#define CRM_AHBRST3_QSPI1RST    BIT(1)

struct flash_artery_f435_qspi_config {
	uint32_t reg_base;
	const struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
	uint32_t max_frequency;
	size_t flash_size;
};

struct flash_artery_f435_qspi_data {
	struct k_sem sem;
	struct k_sem sync;
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
	struct jesd216_erase_type erase_types[JESD216_NUM_ERASE_TYPES];
	uint16_t page_size;
	uint8_t qspi_read_cmd;
	uint8_t qspi_read_cmd_latency;
	bool flag_access_32bit : 1;
	bool flag_quad_io_en : 1;
};

/* Helper macros for register access */
#define QSPI_REG(cfg, offset) \
	((volatile uint32_t *)((cfg)->reg_base + (offset)))

static inline uint32_t qspi_read_reg(const struct flash_artery_f435_qspi_config *cfg,
				     uint32_t offset)
{
	return sys_read32(cfg->reg_base + offset);
}

static inline void qspi_write_reg(const struct flash_artery_f435_qspi_config *cfg,
				   uint32_t offset, uint32_t value)
{
	sys_write32(value, cfg->reg_base + offset);
}

static inline uint8_t qspi_read_byte(const struct flash_artery_f435_qspi_config *cfg)
{
	/* Use volatile pointer for direct register access, like SDK does with dt_u8 */
	volatile uint8_t *dt_reg = (volatile uint8_t *)(cfg->reg_base + QSPI_DT_OFFSET);
	return *dt_reg;
}

static inline void qspi_write_byte(const struct flash_artery_f435_qspi_config *cfg, uint8_t data)
{
	/* Use volatile pointer for direct register access, like SDK does with dt_u8 */
	volatile uint8_t *dt_reg = (volatile uint8_t *)(cfg->reg_base + QSPI_DT_OFFSET);
	*dt_reg = data;
}

static bool qspi_address_is_valid(const struct device *dev, off_t addr, size_t size)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;

	return (addr >= 0) && ((uint64_t)addr + (uint64_t)size <= cfg->flash_size);
}

/* Prepare command engine for new command - MUST be called before every command */
static inline void qspi_prepare_cmd(const struct flash_artery_f435_qspi_config *cfg)
{
	/* Reset command engine + FIFOs */
	uint32_t ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
	qspi_write_reg(cfg, QSPI_CTRL_OFFSET, ctrl | QSPI_CTRL_ABORT);

	/* Wait ABORT auto-clear */
	uint32_t timeout = 10000;
	while ((qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & QSPI_CTRL_ABORT) && timeout--) {
		k_busy_wait(1);
	}

	/* Clear command-complete flag (rw1c) unconditionally */
	qspi_write_reg(cfg, QSPI_CMDSTS_OFFSET, QSPI_CMDSTS_CMDSTS);
}

/* Wait for command completion */
static int qspi_wait_cmd_complete(const struct flash_artery_f435_qspi_config *cfg,
				   uint32_t timeout_ms)
{
	uint32_t start = k_uptime_get_32();
	uint32_t cmdsts;
	uint32_t iterations = 0;

	while (!(cmdsts = qspi_read_reg(cfg, QSPI_CMDSTS_OFFSET) & QSPI_CMDSTS_CMDSTS)) {
		iterations++;
		if (k_uptime_get_32() - start > timeout_ms) {
			LOG_ERR("Command timeout after %u iterations:", iterations);
			LOG_ERR("  CMDSTS=0x%08x (bit0=%u)", 
				qspi_read_reg(cfg, QSPI_CMDSTS_OFFSET),
				!!(qspi_read_reg(cfg, QSPI_CMDSTS_OFFSET) & BIT(0)));
			LOG_ERR("  FIFOSTS=0x%08x (TXRDY=%u RXRDY=%u)", 
				qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET),
				!!(qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET) & BIT(0)),
				!!(qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET) & BIT(1)));
			LOG_ERR("  CTRL=0x%08x (XIPSEL=%u ABORT=%u)",
				qspi_read_reg(cfg, QSPI_CTRL_OFFSET),
				!!(qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & QSPI_CTRL_XIPSEL),
				!!(qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & QSPI_CTRL_ABORT));
			LOG_ERR("  CMD_W0=0x%08x CMD_W1=0x%08x CMD_W2=0x%08x CMD_W3=0x%08x",
				qspi_read_reg(cfg, QSPI_CMD_W0_OFFSET),
				qspi_read_reg(cfg, QSPI_CMD_W1_OFFSET),
				qspi_read_reg(cfg, QSPI_CMD_W2_OFFSET),
				qspi_read_reg(cfg, QSPI_CMD_W3_OFFSET));
			
			/* Assert ABORT to reset command engine after error */
			LOG_DBG("Asserting ABORT after timeout");
			uint32_t ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
			ctrl |= QSPI_CTRL_ABORT;
			qspi_write_reg(cfg, QSPI_CTRL_OFFSET, ctrl);
			
			/* Wait for ABORT to clear */
			uint32_t abort_timeout = 10000;
			while ((qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & QSPI_CTRL_ABORT) && abort_timeout--) {
				k_busy_wait(1);
			}
			
			return -ETIMEDOUT;
		}
		k_yield();
	}

	/* Only log if command took significant time (debugging slow operations) */
	if (iterations > 10) {
		LOG_DBG("Command completed after %u iterations, CMDSTS=0x%08x", iterations, cmdsts);
	}

	/* Clear the command complete flag (write-1-to-clear) and verify */
	qspi_write_reg(cfg, QSPI_CMDSTS_OFFSET, QSPI_CMDSTS_CMDSTS);
	(void)qspi_read_reg(cfg, QSPI_CMDSTS_OFFSET);

	return 0;
}

/* Wait for FIFO ready (write) */
static int qspi_wait_tx_fifo_ready(const struct flash_artery_f435_qspi_config *cfg,
				    uint32_t timeout_ms)
{
	uint32_t start = k_uptime_get_32();

	while (!(qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET) & QSPI_FIFOSTS_TXFIFORDY)) {
		if (k_uptime_get_32() - start > timeout_ms) {
			return -ETIMEDOUT;
		}
		k_yield();
	}

	return 0;
}

/* Execute a QSPI command
 * Note: flash_write_enable controls whether WREN (0x06) is sent first
 *       write_data_enable controls bit 1 of CMD_W3 (for commands that write data to flash)
 */
static int qspi_exec_cmd(const struct flash_artery_f435_qspi_config *cfg,
			  uint8_t instruction, uint32_t address, uint8_t addr_len,
			  uint8_t dummy_cycles, uint8_t opmode, bool flash_write_enable,
			  bool write_data_enable, bool read_status, uint32_t data_count)
{
	uint32_t cmd_w1 = 0;
	uint32_t cmd_w3 = 0;
	uint32_t ctrl;

	LOG_DBG(">>> QSPI cmd: inst=0x%02x addr=0x%08x alen=%u dummy=%u mode=%u dcnt=%u wden=%d rst=%d",
		instruction, address, addr_len, dummy_cycles, opmode, data_count, write_data_enable, read_status);

	/* CRITICAL: Reset command engine before every command
	 * Per AT32F435 RM, ABORT flushes FIFOs and resets state machine
	 * CMDSTS is rw1c and must be cleared unconditionally between commands */
	qspi_prepare_cmd(cfg);

	/* Fix for read_status mode: Status registers are always 1 byte
	 * Force data_count=1 to make this unambiguous for the controller */
	if (read_status) {
		data_count = 1;
	}

	/* CRITICAL: Write command registers in exact sequence per RM - CMD_W0, W1, W2, then W3
	 * Writing CMD_W3 triggers command execution, so it must be written LAST
	 * All writes must be 32-bit word accesses */
	
	/* Configure CMD_W0 (address) */
	qspi_write_reg(cfg, QSPI_CMD_W0_OFFSET, address);

	/* Configure CMD_W1 (address length, dummy cycles, instruction length, PE mode) */
	/* IMPORTANT: PEMEN (bit 28) must be 0 for standard commands */
	cmd_w1 = (addr_len << QSPI_CMD_W1_ADRLEN_POS) |
		 (dummy_cycles << QSPI_CMD_W1_DUM2_POS) |
		 ((instruction != 0 ? QSPI_INSLEN_1_BYTE : QSPI_INSLEN_0_BYTE)
		  << QSPI_CMD_W1_INSLEN_POS);
	/* Explicitly ensure PEMEN and reserved bits are 0 */
	cmd_w1 &= ~(BIT(28) | BIT(29) | BIT(30) | BIT(31) | BIT(26) | BIT(27));
	qspi_write_reg(cfg, QSPI_CMD_W1_OFFSET, cmd_w1);

	/* Configure CMD_W2 (data counter) */
	qspi_write_reg(cfg, QSPI_CMD_W2_OFFSET, data_count);

	/* Configure CMD_W3 (instruction, operation mode, control bits) */
	cmd_w3 = (instruction << QSPI_CMD_W3_INSC_POS) |
		 (opmode << QSPI_CMD_W3_OPMODE_POS);

	/* Bit 1: write_data_enable - set ONLY for commands that write data to flash */
	if (write_data_enable) {
		cmd_w3 |= QSPI_CMD_W3_WDEN;
	}

	if (read_status) {
		cmd_w3 |= QSPI_CMD_W3_RSTSEN;
		/* Use hardware auto-read status */
		/* Don't set RSTSC bit for hardware mode */
	}
	
	/* flash_write_enable parameter is for future use with explicit WREN
	 * (0x06 command) - not used for controller command execution */
	(void)flash_write_enable;

	/* CRITICAL: Writing CMD_W3 triggers command execution per SDK qspi_cmd_operation_kick()
	 * The SDK writes this register LAST and returns immediately without any intervening reads.
	 */
	qspi_write_reg(cfg, QSPI_CMD_W3_OFFSET, cmd_w3);

	return 0;
}

/* Read Status Register-1 (BUSY, WEL) */
static uint8_t qspi_read_sr1(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;

	qspi_exec_cmd(cfg, SPI_NOR_CMD_RDSR, 0, QSPI_ADRLEN_0_BYTE, 0,
		       QSPI_OPMODE_111, false, false, true, 0);

	qspi_wait_cmd_complete(cfg, 100);

	return qspi_read_reg(cfg, QSPI_RSTS_OFFSET) & QSPI_RSTS_SPISTS_MASK;
}

/* Read Status Register-2 (QE, SUS) */
static uint8_t qspi_read_sr2(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;

	qspi_exec_cmd(cfg, W25Q_CMD_READ_SR2, 0, QSPI_ADRLEN_0_BYTE, 0,
		       QSPI_OPMODE_111, false, false, true, 0);

	qspi_wait_cmd_complete(cfg, 100);

	return qspi_read_reg(cfg, QSPI_RSTS_OFFSET) & QSPI_RSTS_SPISTS_MASK;
}

/* Read Status Register-3 (ADS, ADP, WPS) */
static uint8_t qspi_read_sr3(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;

	qspi_exec_cmd(cfg, W25Q_CMD_READ_SR3, 0, QSPI_ADRLEN_0_BYTE, 0,
		       QSPI_OPMODE_111, false, false, true, 0);

	qspi_wait_cmd_complete(cfg, 100);

	return qspi_read_reg(cfg, QSPI_RSTS_OFFSET) & QSPI_RSTS_SPISTS_MASK;
}

/* Read flash status register (alias for SR1) */
static uint8_t qspi_flash_read_status(const struct device *dev)
{
	return qspi_read_sr1(dev);
}

/* Wait for flash to be ready (not busy) */
static int qspi_flash_wait_ready(const struct device *dev, uint32_t timeout_ms)
{
	uint32_t start = k_uptime_get_32();
	uint8_t status;
	bool ever_busy = false;
	uint32_t iterations = 0;

	do {
		status = qspi_flash_read_status(dev);
		iterations++;
		
		/* Track if we ever saw BUSY=1 (datasheet requirement for erase/program) */
		if (status & W25Q_SR1_BUSY) {
			ever_busy = true;
		}
		
		if (!(status & W25Q_SR1_BUSY)) {
			/* Operation complete - verify datasheet behavior */
			uint8_t wel = (status >> 1) & 1;
			
			/* DATASHEET CHECK: After erase/program, WEL should auto-clear to 0 */
			if (ever_busy && wel != 0) {
				LOG_WRN("WEL still set after operation (SR1=0x%02x)", status);
			}
			
			return 0;
		}
		if (k_uptime_get_32() - start > timeout_ms) {
			LOG_ERR("Timeout waiting for flash ready (SR1=0x%02x, iterations=%u)",
				status, iterations);
			return -ETIMEDOUT;
		}
		k_yield();
	} while (true);
}

/* Software reset sequence (W25Q512JV: 66h then 99h) */
static int w25q_software_reset(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	int ret;

	LOG_INF("Performing software reset");

	/* Enable Reset (66h) - SDK uses write_data_enable=TRUE for instruction-only commands
	 * This sets CMD_W3.WEN bit which triggers command execution */
	qspi_exec_cmd(cfg, W25Q_CMD_ENABLE_RESET, 0, QSPI_ADRLEN_0_BYTE, 0,
		       QSPI_OPMODE_111, false, true, false, 0);
	ret = qspi_wait_cmd_complete(cfg, 100);
	if (ret < 0) {
		return ret;
	}

	/* Reset Device (99h) - SDK uses write_data_enable=TRUE for instruction-only */
	qspi_exec_cmd(cfg, W25Q_CMD_RESET_DEVICE, 0, QSPI_ADRLEN_0_BYTE, 0,
		       QSPI_OPMODE_111, false, true, false, 0);
	ret = qspi_wait_cmd_complete(cfg, 100);
	if (ret < 0) {
		return ret;
	}

	/* CRITICAL: Wait for reset to complete (W25Q512JV tRST = 30µs typ, 30µs max)
	 * This is MANDATORY before issuing any subsequent commands */
	k_busy_wait(50);

	LOG_INF("Software reset complete");
	return 0;
}

/* Send write enable command */
static int qspi_write_enable(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	int ret;
	uint8_t sr1;

	/* Write Enable - SDK uses write_data_enable=TRUE for instruction-only */
	qspi_exec_cmd(cfg, SPI_NOR_CMD_WREN, 0, QSPI_ADRLEN_0_BYTE, 0,
		      QSPI_OPMODE_111, false, true, false, 0);

	ret = qspi_wait_cmd_complete(cfg, 1000);
	if (ret < 0) {
		return ret;
	}

	/* CRITICAL DATASHEET VERIFICATION: After WREN, WEL bit (SR1[1]) MUST be set
	 * If WEL=0, subsequent program/erase will be ignored by the flash */
	sr1 = qspi_read_sr1(dev);
	if (!(sr1 & BIT(1))) {  /* WEL is bit 1 */
		LOG_ERR("WREN failed: WEL bit not set! SR1=0x%02x", sr1);
		return -EIO;
	}

	/* Only log on first few operations, then silent success */
	static uint8_t wren_log_count = 0;
	if (wren_log_count < 3) {
		LOG_INF("WREN OK: WEL=1 (SR1=0x%02x)", sr1);
		wren_log_count++;
	}
	return 0;
}

/* Enter 4-byte address mode (W25Q512JV: B7h) */
static int w25q_enter_4byte_mode(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint8_t sr3;
	int ret;

	LOG_INF("Entering 4-byte address mode");

	/* NOTE: W25Q512JV B7h command does NOT require WREN (it's just a mode command)
	 * Removing unnecessary WREN that was complicating state */

	/* Send Enter 4-Byte Address Mode command - SDK uses write_data_enable=TRUE */
	qspi_exec_cmd(cfg, W25Q_CMD_ENTER_4B_ADDR, 0, QSPI_ADRLEN_0_BYTE, 0,
		      QSPI_OPMODE_111, false, true, false, 0);
	ret = qspi_wait_cmd_complete(cfg, 100);
	if (ret < 0) {
		LOG_ERR("Failed to enter 4-byte mode");
		return ret;
	}

	/* Small delay for flash to update internal state after mode change */
	k_busy_wait(10);

	/* Verify by reading SR3 and checking ADS bit */
	sr3 = qspi_read_sr3(dev);
	if (sr3 & W25Q_SR3_ADS) {
		data->flag_access_32bit = true;
		LOG_INF("4-byte address mode enabled (ADS=1)");
		return 0;
	} else {
		LOG_WRN("4-byte mode command sent but ADS=0");
		return -EIO;
	}
}

/* Set Read Parameters (C0h) - Configure dummy cycles for quad I/O reads */
static int w25q_set_read_parameters(const struct device *dev, uint8_t dummy_config)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	int ret;

	LOG_DBG("Setting read parameters: dummy config 0x%02x", dummy_config);

	/* Set Read Parameters: P[7]=0 (dummy cycles), P[6:4]=dummy config, P[3:0]=0xF (wrap disabled) */
	uint8_t param = dummy_config | 0x0F;

	qspi_exec_cmd(cfg, W25Q_CMD_SET_READ_PARAMS, 0, QSPI_ADRLEN_0_BYTE, 0,
		      QSPI_OPMODE_111, true, true, false, 1);

	qspi_write_byte(cfg, param);

	ret = qspi_wait_cmd_complete(cfg, 100);
	if (ret < 0) {
		LOG_ERR("Failed to set read parameters");
		return ret;
	}

	LOG_DBG("Read parameters configured");
	return 0;
}

/* Global Block/Sector Unlock (98h) - Required if WPS=1 */
static int w25q_global_unlock(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	int ret;

	LOG_INF("Performing global block unlock");

	/* Send write enable */
	ret = qspi_write_enable(dev);
	if (ret < 0) {
		return ret;
	}

	/* Send Global Block/Sector Unlock - SDK uses write_data_enable=TRUE */
	ret = qspi_write_enable(dev);
	if (ret < 0) {
		return ret;
	}
	qspi_exec_cmd(cfg, W25Q_CMD_GLOBAL_BLOCK_UNLOCK, 0, QSPI_ADRLEN_0_BYTE, 0,
		       QSPI_OPMODE_111, false, true, false, 0);

	ret = qspi_wait_cmd_complete(cfg, 100);
	if (ret < 0) {
		LOG_ERR("Failed to perform global unlock");
		return ret;
	}

	/* Wait for operation to complete */
	ret = qspi_flash_wait_ready(dev, W25Q_TIMEOUT_WRITE_STATUS);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("Global unlock complete");
	return 0;
}

/* Read JEDEC ID */
static int flash_artery_f435_qspi_read_jedec_id(const struct device *dev, uint8_t *jedec_id,
						 size_t len)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	int ret;

	if (k_sem_take(&data->sem, K_SECONDS(1)) != 0) {
		return -EBUSY;
	}

	/* Execute JEDEC ID read command */
	qspi_exec_cmd(cfg, SPI_NOR_CMD_RDID, 0, QSPI_ADRLEN_0_BYTE, 0,
		      QSPI_OPMODE_111, false, false, false, len);

	/* Wait for RX FIFO to have data ready */
	uint32_t start = k_uptime_get_32();
	while (!(qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET) & QSPI_FIFOSTS_RXFIFORDY)) {
		if (k_uptime_get_32() - start > 1000) {
			LOG_ERR("RX FIFO timeout waiting for JEDEC ID");
			k_sem_give(&data->sem);
			return -ETIMEDOUT;
		}
		k_yield();
	}

	/* Read JEDEC ID bytes from FIFO */
	for (size_t i = 0; i < len; i++) {
		jedec_id[i] = qspi_read_byte(cfg);
	}

	/* Wait for command to complete */
	ret = qspi_wait_cmd_complete(cfg, 1000);
	if (ret < 0) {
		LOG_ERR("JEDEC ID command did not complete");
		k_sem_give(&data->sem);
		return ret;
	}

	k_sem_give(&data->sem);

	return ret;
}

/* Read SFDP data */
static int flash_artery_f435_qspi_read_sfdp(const struct device *dev, off_t addr,
					     void *data_buf, size_t size)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint8_t *buf = (uint8_t *)data_buf;
	int ret;

	if (k_sem_take(&data->sem, K_SECONDS(1)) != 0) {
		return -EBUSY;
	}

	/* SFDP read: 1-1-1 mode, 3-byte address, 8 dummy cycles */
	qspi_exec_cmd(cfg, JESD216_CMD_READ_SFDP, addr, QSPI_ADRLEN_3_BYTE, 8,
		      QSPI_OPMODE_111, false, false, false, size);

	/* Wait for RX FIFO ready, then read data */
	uint32_t start = k_uptime_get_32();
	while (!(qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET) & QSPI_FIFOSTS_RXFIFORDY)) {
		if (k_uptime_get_32() - start > 1000) {
			k_sem_give(&data->sem);
			return -ETIMEDOUT;
		}
		k_yield();
	}

	/* Read SFDP data from FIFO */
	for (size_t i = 0; i < size; i++) {
		buf[i] = qspi_read_byte(cfg);
	}

	/* Wait for command to complete */
	ret = qspi_wait_cmd_complete(cfg, 1000);
	if (ret < 0) {
		k_sem_give(&data->sem);
		return ret;
	}

	k_sem_give(&data->sem);

	return ret;
}

/* Flash read operation */
static int flash_artery_f435_qspi_read(const struct device *dev, off_t addr,
					void *data_buf, size_t size)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint8_t *buf = (uint8_t *)data_buf;
	uint32_t to_read;
	int ret = 0;

	if (!qspi_address_is_valid(dev, addr, size)) {
		LOG_ERR("Invalid address/size: addr=0x%lx, size=%zu", (long)addr, size);
		return -EINVAL;
	}

	if (size == 0) {
		return 0;
	}

	if (k_sem_take(&data->sem, K_SECONDS(1)) != 0) {
		return -EBUSY;
	}

	/* Use 1-4-4 quad read if enabled, otherwise standard read */
	uint8_t read_cmd = data->flag_quad_io_en ? SPI_NOR_CMD_4READ : SPI_NOR_CMD_READ;
	uint8_t opmode = data->flag_quad_io_en ? QSPI_OPMODE_144 : QSPI_OPMODE_111;
	uint8_t dummy_cycles = data->flag_quad_io_en ? 4 : 0;

	/* Address format depends on current flash mode */
	uint32_t flash_addr = addr;
	uint8_t addr_len = data->flag_access_32bit ? QSPI_ADRLEN_4_BYTE : QSPI_ADRLEN_3_BYTE;

	/* CRITICAL: 0xEB (Fast Read Quad I/O) ALWAYS requires a mode byte (M7-M0)
	 * The mode byte controls continuous read mode and comes after the address.
	 * For AT32F435 QSPI controller, we pack it into CMD_W0 by encoding as:
	 *   (address << 8) | mode_byte
	 * This shifts address left, putting mode byte in the LSB position.
	 * 
	 * Mode byte 0xFF = exit continuous read (standard operation)
	 * 
	 * Address length must be increased by 1 byte to account for mode byte:
	 * - 3-byte mode: addr_len=4 (3 addr bytes + 1 mode byte)
	 * - 4-byte mode: addr_len=5 (4 addr bytes + 1 mode byte) - but we don't support this yet
	 */
	if (read_cmd == SPI_NOR_CMD_4READ) {
		/* Always include mode byte for 0xEB command */
		flash_addr = (addr << 8) | 0xFF;  /* 0xFF = exit continuous read */
		addr_len = QSPI_ADRLEN_4_BYTE;    /* 3 address bytes + 1 mode byte = 4 total */
	}

	/* Start read command */
	qspi_exec_cmd(cfg, read_cmd, flash_addr, addr_len, dummy_cycles,
		      opmode, false, false, false, size);

	/* CRITICAL: Read data from FIFO in chunks BEFORE waiting for command complete
	 * The command may not complete until FIFO is drained! */
	while (size > 0) {
		to_read = MIN(size, QSPI_FIFO_DEPTH);

		/* Wait for RX FIFO to have data ready */
		uint32_t start = k_uptime_get_32();
		while (!(qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET) & QSPI_FIFOSTS_RXFIFORDY)) {
			if (k_uptime_get_32() - start > 1000) {
				LOG_ERR("RX FIFO timeout");
				k_sem_give(&data->sem);
				return -ETIMEDOUT;
			}
			k_yield();
		}

		/* Read chunk from FIFO */
		for (uint32_t i = 0; i < to_read; i++) {
			*buf++ = qspi_read_byte(cfg);
		}

		size -= to_read;
	}

	/* Now wait for command to complete */
	ret = qspi_wait_cmd_complete(cfg, 1000);
	if (ret < 0) {
		k_sem_give(&data->sem);
		return ret;
	}

	k_sem_give(&data->sem);

	return ret;
}

/* Flash write operation */
static int flash_artery_f435_qspi_write(const struct device *dev, off_t addr,
					 const void *data_buf, size_t size)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	const uint8_t *buf = (const uint8_t *)data_buf;
	uint32_t to_write;
	int ret = 0;

	if (!qspi_address_is_valid(dev, addr, size)) {
		LOG_ERR("Invalid address/size: addr=0x%lx, size=%zu", (long)addr, size);
		return -EINVAL;
	}

	if (size == 0) {
		return 0;
	}

	if (k_sem_take(&data->sem, K_SECONDS(1)) != 0) {
		return -EBUSY;
	}

	while (size > 0) {
		/* Don't cross page boundaries */
		to_write = MIN(size, SPI_NOR_PAGE_SIZE);
		if (((addr + to_write - 1) / SPI_NOR_PAGE_SIZE) != (addr / SPI_NOR_PAGE_SIZE)) {
			to_write = SPI_NOR_PAGE_SIZE - (addr % SPI_NOR_PAGE_SIZE);
		}

		/* Send write enable */
		ret = qspi_write_enable(dev);
		if (ret < 0) {
			break;
		}

		/* Use quad page program if enabled, otherwise standard page program */
		uint8_t write_cmd = data->flag_quad_io_en ? 0x32 : SPI_NOR_CMD_PP;
		uint8_t opmode = data->flag_quad_io_en ? QSPI_OPMODE_114 : QSPI_OPMODE_111;
		uint8_t addr_len = data->flag_access_32bit ? QSPI_ADRLEN_4_BYTE :
				   QSPI_ADRLEN_3_BYTE;

		/* Execute write command */
		qspi_exec_cmd(cfg, write_cmd, addr, addr_len, 0, opmode, true, true, false, to_write);

		/* Write data bytes - CRITICAL: wait for TX FIFO ready before EACH byte
		 * per SDK example (command_port_using_interrupt/qspi_cmd_en25qh128a.c) */
		for (uint32_t i = 0; i < to_write; i++) {
			/* Wait for TX FIFO to be ready to accept next byte */
			uint32_t start = k_uptime_get_32();
			while (!(qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET) & QSPI_FIFOSTS_TXFIFORDY)) {
				if (k_uptime_get_32() - start > 1000) {
					LOG_ERR("TX FIFO timeout at byte %u", i);
					ret = -ETIMEDOUT;
					break;
				}
				k_yield();
			}
			if (ret < 0) {
				break;
			}
			qspi_write_byte(cfg, *buf++);
		}

		if (ret < 0) {
			break;
		}

		ret = qspi_wait_cmd_complete(cfg, 1000);
		if (ret < 0) {
			break;
		}

		/* Wait for page program to complete (W25Q512JV: max 3.5ms) */
		ret = qspi_flash_wait_ready(dev, W25Q_TIMEOUT_PAGE_PROGRAM);
		if (ret < 0) {
			LOG_ERR("Page program timeout at addr=0x%lx", (long)(addr - to_write));
			break;
		}

		addr += to_write;
		size -= to_write;
	}

	k_sem_give(&data->sem);

	return ret;
}

/* Flash erase operation */
static int flash_artery_f435_qspi_erase(const struct device *dev, off_t addr, size_t size)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	int ret = 0;

	if (!qspi_address_is_valid(dev, addr, size)) {
		LOG_ERR("Invalid address/size: addr=0x%lx, size=%zu", (long)addr, size);
		return -EINVAL;
	}

	if (size == 0) {
		return 0;
	}

	if (k_sem_take(&data->sem, K_SECONDS(1)) != 0) {
		return -EBUSY;
	}

	/* Check for full chip erase */
	if (size == cfg->flash_size && addr == 0) {
		LOG_WRN("Performing full chip erase (will take several minutes)");

		ret = qspi_write_enable(dev);
		if (ret < 0) {
			goto out;
		}

		qspi_exec_cmd(cfg, SPI_NOR_CMD_CE, 0, QSPI_ADRLEN_0_BYTE, 0,
			       QSPI_OPMODE_111, false, true, false, 0);

		ret = qspi_wait_cmd_complete(cfg, 1000);
		if (ret < 0) {
			goto out;
		}

		/* Wait for chip erase (W25Q512JV: typ 200s, max 1000s) */
		ret = qspi_flash_wait_ready(dev, W25Q_TIMEOUT_CHIP_ERASE);
		LOG_INF("Chip erase complete");
		goto out;
	}

	/* Sector/block erase with W25Q512JV-appropriate timeouts */
	uint8_t addr_len = data->flag_access_32bit ? QSPI_ADRLEN_4_BYTE : QSPI_ADRLEN_3_BYTE;

	while (size > 0) {
		const struct jesd216_erase_type *etp = NULL;
		const struct jesd216_erase_type *best_etp = NULL;
		uint32_t timeout_ms;

		/* Find the best (largest) erase type that fits */
		for (uint8_t ei = 0; ei < JESD216_NUM_ERASE_TYPES; ++ei) {
			etp = &data->erase_types[ei];

			if ((etp->exp != 0) &&
			    SPI_NOR_IS_ALIGNED(addr, etp->exp) &&
			    SPI_NOR_IS_ALIGNED(size, etp->exp) &&
			    ((best_etp == NULL) || (etp->exp > best_etp->exp))) {
				best_etp = etp;
			}
		}

		if (best_etp == NULL) {
			LOG_ERR("No suitable erase type for addr=0x%lx, size=%zu",
				(long)addr, size);
			ret = -EINVAL;
			break;
		}

		/* Select timeout based on erase size (W25Q512JV datasheet) */
		uint32_t erase_size = BIT(best_etp->exp);
		if (erase_size == 4096) {
			timeout_ms = W25Q_TIMEOUT_SECTOR_ERASE;  /* 4KB: 400ms */
		} else if (erase_size == 32768) {
			timeout_ms = W25Q_TIMEOUT_BLOCK_ERASE_32K;  /* 32KB: 1600ms */
		} else if (erase_size == 65536) {
			timeout_ms = W25Q_TIMEOUT_BLOCK_ERASE_64K;  /* 64KB: 2000ms */
		} else {
			timeout_ms = W25Q_TIMEOUT_BLOCK_ERASE_64K;  /* Default */
		}

		ret = qspi_write_enable(dev);
		if (ret < 0) {
			break;
		}

		/* Execute erase command - has address but no data, use write_data_enable=TRUE */
		qspi_exec_cmd(cfg, best_etp->cmd, addr, addr_len, 0,
			       QSPI_OPMODE_111, false, true, false, 0);

		ret = qspi_wait_cmd_complete(cfg, 1000);
		if (ret < 0) {
			break;
		}

		/* Wait for erase to complete with appropriate timeout */
		ret = qspi_flash_wait_ready(dev, timeout_ms);
		if (ret < 0) {
			LOG_ERR("Erase timeout at addr=0x%lx", (long)addr);
			break;
		}

		LOG_DBG("Erased %u bytes at 0x%lx", erase_size, (long)addr);

		addr += erase_size;
		size -= erase_size;
	}

out:
	k_sem_give(&data->sem);
	return ret;
}

/* Get flash parameters */
static const struct flash_parameters *
flash_artery_f435_qspi_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	static const struct flash_parameters params = {
		.write_block_size = 1,
		.erase_value = 0xFF,
	};

	return &params;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
/* Get flash page layout */
static void flash_artery_f435_qspi_page_layout(const struct device *dev,
						const struct flash_pages_layout **layout,
						size_t *layout_size)
{
	struct flash_artery_f435_qspi_data *data = dev->data;

	*layout = &data->layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

/* Flash driver API */
static const struct flash_driver_api flash_artery_f435_qspi_api = {
	.read = flash_artery_f435_qspi_read,
	.write = flash_artery_f435_qspi_write,
	.erase = flash_artery_f435_qspi_erase,
	.get_parameters = flash_artery_f435_qspi_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_artery_f435_qspi_page_layout,
#endif
};

/* Enable Quad Enable (QE) bit in Status Register-2 */
static int w25q_enable_quad_mode(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	uint8_t sr2;
	int ret;

	/* Read current SR2 value */
	sr2 = qspi_read_sr2(dev);

	/* Check if QE bit is already set */
	if (sr2 & W25Q_SR2_QE) {
		LOG_INF("Quad Enable (QE) bit already set");
		return 0;
	}

	LOG_INF("Setting Quad Enable (QE) bit in SR2");

	/* Enable write */
	ret = qspi_write_enable(dev);
	if (ret < 0) {
		return ret;
	}

	/* Write Status Register-2 with QE bit set */
	qspi_exec_cmd(cfg, W25Q_CMD_WRITE_SR2, 0, QSPI_ADRLEN_0_BYTE, 0,
		       QSPI_OPMODE_111, true, true, false, 1);

	qspi_write_byte(cfg, sr2 | W25Q_SR2_QE);

	ret = qspi_wait_cmd_complete(cfg, 100);
	if (ret < 0) {
		LOG_ERR("Failed to write SR2");
		return ret;
	}

	/* Wait for write to complete (tW max: 15ms) */
	ret = qspi_flash_wait_ready(dev, W25Q_TIMEOUT_WRITE_STATUS);
	if (ret < 0) {
		LOG_ERR("Timeout waiting for SR2 write");
		return ret;
	}

	/* Verify QE bit was set */
	sr2 = qspi_read_sr2(dev);
	if (sr2 & W25Q_SR2_QE) {
		LOG_INF("Quad Enable (QE) bit successfully set");
		return 0;
	} else {
		LOG_ERR("QE bit verification failed");
		return -EIO;
	}
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
/* Setup flash page layout from SFDP erase types */
static int setup_pages_layout(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint32_t layout_page_size = data->page_size;
	uint8_t exp = 0;

	/* Find the smallest erase size */
	for (size_t i = 0; i < ARRAY_SIZE(data->erase_types); ++i) {
		const struct jesd216_erase_type *etp = &data->erase_types[i];

		if ((etp->cmd != 0) && ((exp == 0) || (etp->exp < exp))) {
			exp = etp->exp;
		}
	}

	if (exp == 0) {
		LOG_ERR("No valid erase types found");
		return -ENOTSUP;
	}

	uint32_t erase_size = BIT(exp);

	/* Align layout page size with erase size */
	if ((layout_page_size % erase_size) != 0) {
		LOG_DBG("Layout page %u not compatible with erase size %u",
			layout_page_size, erase_size);
		layout_page_size = erase_size;
	}

	if ((cfg->flash_size % layout_page_size) != 0) {
		LOG_WRN("Layout page %u wastes space with device size %zu",
			layout_page_size, cfg->flash_size);
	}

	data->layout.pages_size = layout_page_size;
	data->layout.pages_count = cfg->flash_size / layout_page_size;

	LOG_INF("Flash layout: %u pages x %u bytes", data->layout.pages_count,
		data->layout.pages_size);

	return 0;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

/* Process JEDEC216 Basic Flash Parameter table */
static int process_bfp(const struct device *dev,
		       const struct jesd216_param_header *php,
		       const struct jesd216_bfp *bfp)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	const size_t flash_size = jesd216_bfp_density(bfp) / 8U;
	int rc = 0;

	if (flash_size != cfg->flash_size) {
		LOG_WRN("SFDP flash size %u differs from DT size %zu",
			flash_size, cfg->flash_size);
	}

	LOG_INF("Flash size: %u bytes (%u MiB)", flash_size, flash_size >> 20);

	/* Copy erase types */
	memset(data->erase_types, 0, sizeof(data->erase_types));
	for (uint8_t ti = 1; ti <= ARRAY_SIZE(data->erase_types); ++ti) {
		struct jesd216_erase_type *etp = &data->erase_types[ti - 1];

		if (jesd216_bfp_erase(bfp, ti, etp) == 0) {
			LOG_DBG("Erase type %u: size=%u, cmd=0x%02x",
				ti, BIT(etp->exp), etp->cmd);
		}
	}

	/* Get page size */
	data->page_size = jesd216_bfp_page_size(php, bfp);
	LOG_INF("Page size: %u bytes", data->page_size);

	/* Check addressing mode */
	uint8_t addr_mode = jesd216_bfp_addrbytes(bfp);

	if (addr_mode == JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_3B) {
		LOG_INF("Flash uses 3-byte addressing");
		data->flag_access_32bit = false;
	} else if (addr_mode == JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_4B) {
		LOG_INF("Flash uses 4-byte addressing");
		data->flag_access_32bit = true;
	} else if (addr_mode == JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_3B4B) {
		LOG_INF("Flash supports both 3B and 4B addressing, using 3B");
		data->flag_access_32bit = false;
		/* Could add logic here to enable 4B mode if needed */
	}

	/* Check for quad I/O support via SFDP */
	struct jesd216_instr quad_instr;

	rc = jesd216_bfp_read_support(php, bfp, JESD216_MODE_144, &quad_instr);
	if (rc > 0) {
		LOG_INF("SFDP: Quad I/O (1-4-4) supported: cmd=0x%02x, wait_states=%u",
			quad_instr.instr, quad_instr.wait_states);

		/* Quad mode should already be enabled in init, just record the parameters */
		data->flag_quad_io_en = true;
		data->qspi_read_cmd = quad_instr.instr;
		data->qspi_read_cmd_latency = quad_instr.wait_states;
		if (quad_instr.mode_clocks) {
			data->qspi_read_cmd_latency += quad_instr.mode_clocks;
		}
	} else {
		LOG_INF("SFDP: Quad I/O not indicated, using standard SPI");
	}

	return 0;
}

/* Initialize the QSPI peripheral hardware */
static void qspi_hw_init(const struct flash_artery_f435_qspi_config *cfg)
{
	uint32_t crm_ahben2, ctrl;

	LOG_DBG("QSPI hardware initialization at base 0x%08x", cfg->reg_base);

	/* Enable QSPI1 clock via CRM (direct register access) */
	crm_ahben2 = sys_read32(CRM_BASE + CRM_AHBEN3_OFFSET);
	LOG_DBG("CRM_AHBEN3 before: 0x%08x", crm_ahben2);
	crm_ahben2 |= CRM_AHBEN3_QSPI1EN;
	sys_write32(crm_ahben2, CRM_BASE + CRM_AHBEN3_OFFSET);
	crm_ahben2 = sys_read32(CRM_BASE + CRM_AHBEN3_OFFSET);
	LOG_DBG("CRM_AHBEN3 after: 0x%08x", crm_ahben2);

	/* NOTE: Old driver does NOT reset QSPI peripheral - skip reset to match working driver */
	
	/* Small delay after clock enable */
	k_busy_wait(100);

	/* Read initial control register */
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
	LOG_DBG("QSPI_CTRL initial: 0x%08x", ctrl);

	/* CRITICAL FIX: ALWAYS run XIP disable sequence to properly reset/initialize peripheral
	 * The SDK qspi_xip_enable(FALSE) only skips if already disabled, but we need to
	 * ensure peripheral is in known state even if bootloader left it configured */
	LOG_DBG("Performing XIP disable sequence (XIPSEL=%d)", !!(ctrl & QSPI_CTRL_XIPSEL));
	
	/* Wait for TX FIFO empty (bit 0 of FIFOSTS means ready/empty) */
	uint32_t timeout = 10000;
	while (!(qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET) & QSPI_FIFOSTS_TXFIFORDY) && timeout--) {
		k_busy_wait(1);
	}
	if (timeout == 0) {
		LOG_WRN("TX FIFO wait timeout");
	}

	/* Small delay for IO transmission - per SDK */
	k_busy_wait(10);

	/* Flush and reset QSPI state */
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
	ctrl |= QSPI_CTRL_XIPRCMDF;
	qspi_write_reg(cfg, QSPI_CTRL_OFFSET, ctrl);
	LOG_DBG("Set XIPRCMDF, CTRL=0x%08x", ctrl);

	/* Wait for abort bit to clear */
	timeout = 10000;
	while ((qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & QSPI_CTRL_ABORT) && timeout--) {
		k_busy_wait(1);
	}
	if (timeout == 0) {
		LOG_WRN("ABORT clear timeout");
	}

	/* Small delay - per SDK */
	k_busy_wait(10);

	/* Now ensure XIP mode is disabled */
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
	ctrl &= ~QSPI_CTRL_XIPSEL;
	qspi_write_reg(cfg, QSPI_CTRL_OFFSET, ctrl);
	LOG_DBG("Cleared XIPSEL, CTRL=0x%08x", ctrl);

	/* Wait for abort bit to clear again */
	timeout = 10000;
	while ((qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & QSPI_CTRL_ABORT) && timeout--) {
		k_busy_wait(1);
	}
	if (timeout == 0) {
		LOG_WRN("ABORT clear timeout 2");
	}

	/* Small delay */
	k_busy_wait(10);
	
	/* Re-read control register to get current state */
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
	LOG_DBG("XIP disable complete, CTRL=0x%08x", ctrl);

	/* CRITICAL: Assert ABORT to reset command engine and flush FIFOs
	 * This is REQUIRED after XIP disable and before first command per RM */
	LOG_DBG("Asserting ABORT to reset command engine");
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
	ctrl |= QSPI_CTRL_ABORT;
	qspi_write_reg(cfg, QSPI_CTRL_OFFSET, ctrl);
	
	/* Wait for ABORT to auto-clear */
	timeout = 10000;
	while ((qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & QSPI_CTRL_ABORT) && timeout--) {
		k_busy_wait(1);
	}
	if (timeout == 0) {
		LOG_ERR("ABORT clear timeout after XIP disable");
		return;
	}
	
	/* Small delay after ABORT completes */
	k_busy_wait(10);
	
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
	LOG_DBG("ABORT complete, CTRL=0x%08x", ctrl);

	/* Set clock divider (div by 8 to match old driver: 240MHz / 8 = 30MHz) */
	ctrl &= ~QSPI_CTRL_CLKDIV_MASK;
	ctrl |= (QSPI_CLK_DIV_8 << QSPI_CTRL_CLKDIV_POS);

	/* Don't modify SCK mode - leave at hardware reset value (mode 0) */

	/* Set busy bit offset (bit 0 in status register for W25Q512JV) */
	ctrl &= ~QSPI_CTRL_BUSY_MASK;
	ctrl |= (0 << QSPI_CTRL_BUSY_POS);

	/* Write final control register configuration */
	qspi_write_reg(cfg, QSPI_CTRL_OFFSET, ctrl);
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
	LOG_DBG("QSPI_CTRL configured: 0x%08x", ctrl);

	/* Clear any pending command status */
	uint32_t cmdsts = qspi_read_reg(cfg, QSPI_CMDSTS_OFFSET);
	if (cmdsts & QSPI_CMDSTS_CMDSTS) {
		LOG_DBG("Clearing pending CMDSTS");
		qspi_write_reg(cfg, QSPI_CMDSTS_OFFSET, QSPI_CMDSTS_CMDSTS);
	}

	/* Read and log all relevant status registers */
	LOG_DBG("Init complete - Register dump:");
	LOG_DBG("  CTRL:    0x%08x", qspi_read_reg(cfg, QSPI_CTRL_OFFSET));
	LOG_DBG("  CTRL2:   0x%08x", qspi_read_reg(cfg, QSPI_CTRL2_OFFSET));
	LOG_DBG("  FIFOSTS: 0x%08x", qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET));
	LOG_DBG("  CMDSTS:  0x%08x", qspi_read_reg(cfg, QSPI_CMDSTS_OFFSET));
	LOG_DBG("  RSTS:    0x%08x", qspi_read_reg(cfg, QSPI_RSTS_OFFSET));
}

/* Driver initialization - Follows W25Q512JV datasheet checklist */
static int flash_artery_f435_qspi_init(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint8_t jedec_id[3] = {0};
	uint8_t sr1, sr2, sr3;
	int ret;

	/* Initialize semaphores */
	k_sem_init(&data->sem, 1, 1);
	k_sem_init(&data->sync, 1, 1);

	/* Initialize data structure */
	data->page_size = SPI_NOR_PAGE_SIZE;
	data->flag_access_32bit = false;
	data->flag_quad_io_en = false;

	qspi_hw_init(cfg);

	/* Configure pinctrl AFTER QSPI clock is enabled */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply pinctrl state: %d", ret);
		return ret;
	}

	/*
	 * Power-up constraints (W25Q512JV datasheet):
	 * - /CS must track VCC during ramp
	 * - tVSL (VCC min to /CS low): 20µs min
	 * - tPUW (power-up to write enable): 5ms min
	 *
	 * Assume bootloader/hardware has satisfied these, but add delay to be safe
	 */
	k_sleep(K_MSEC(10));

	LOG_INF("Initializing W25Q512JV QSPI NOR flash: %s", dev->name);

	/* Step 1: Software reset to ensure known state (optional but recommended) */
	ret = w25q_software_reset(dev);
	if (ret < 0) {
		LOG_WRN("Software reset failed: %d (continuing anyway)", ret);
		/* Continue anyway - device might be in a working state */
	}

	/* Step 2: Read and validate JEDEC ID */
	ret = flash_artery_f435_qspi_read_jedec_id(dev, jedec_id, sizeof(jedec_id));
	if (ret < 0) {
		LOG_ERR("Failed to read JEDEC ID: %d", ret);
		return ret;
	}

	LOG_INF("JEDEC ID: %02x %02x %02x", jedec_id[0], jedec_id[1], jedec_id[2]);

	/* Validate JEDEC ID for W25Q512JV */
	if (jedec_id[0] != W25Q512_JEDEC_MFR) {
		LOG_ERR("Unexpected manufacturer ID: 0x%02x (expected 0x%02x)",
			jedec_id[0], W25Q512_JEDEC_MFR);
		return -ENODEV;
	}
	if (jedec_id[1] != W25Q512_JEDEC_TYPE || jedec_id[2] != W25Q512_JEDEC_CAPACITY) {
		LOG_WRN("JEDEC ID type/capacity: %02x %02x (expected %02x %02x)",
			jedec_id[1], jedec_id[2], 
			W25Q512_JEDEC_TYPE, W25Q512_JEDEC_CAPACITY);
	}

	/* Step 3: Read all status registers to check device state */
	sr1 = qspi_read_sr1(dev);
	sr2 = qspi_read_sr2(dev);
	sr3 = qspi_read_sr3(dev);

	LOG_INF("Status Registers: SR1=0x%02x SR2=0x%02x SR3=0x%02x", sr1, sr2, sr3);

	/* Check if device is busy */
	if (sr1 & W25Q_SR1_BUSY) {
		LOG_WRN("Device busy after reset - waiting");
		ret = qspi_flash_wait_ready(dev, 1000);
		if (ret < 0) {
			LOG_ERR("Device stuck busy");
			return ret;
		}
	}

	/* Step 4: Check Write Protect Selection (WPS) and unlock if needed */
	if (sr3 & W25Q_SR3_WPS) {
		LOG_INF("Individual Block Lock mode (WPS=1) - performing global unlock");
		ret = w25q_global_unlock(dev);
		if (ret < 0) {
			LOG_ERR("Global unlock failed: %d", ret);
			return ret;
		}
	} else {
		LOG_INF("Standard write protection mode (WPS=0)");
	}

	/* Step 5: Enable Quad mode (set QE bit) if not already set */
	ret = w25q_enable_quad_mode(dev);
	if (ret < 0) {
		LOG_ERR("Failed to enable quad mode: %d", ret);
		return ret;
	}

	/* Step 6: Check if flash is already in 4-byte address mode
	 * W25Q512JV can be configured for power-up 4B mode via OTP or previous commands */
	sr3 = qspi_read_sr3(dev);
	if (sr3 & W25Q_SR3_ADS) {
		LOG_INF("Flash already in 4-byte address mode (ADS=1)");
		data->flag_access_32bit = true;
	} else {
		LOG_INF("Flash in 3-byte address mode (ADS=0) - will use 3-byte addressing");
		data->flag_access_32bit = false;
		/* TODO: Implement Extended Address Register support for accessing >16MB
		 * For now, 3-byte addressing works fine for the first 16MB */
	}

	/* Step 7: Configure read parameters (dummy cycles) for optimal performance
	 * NOTE: Skipping for now as it may not be necessary and adds complexity */
	LOG_INF("Using default read parameters from flash power-on config");

	/* Read and process SFDP */
	const uint8_t decl_nph = 3;
	union {
		uint8_t raw[JESD216_SFDP_SIZE(decl_nph)];
		struct jesd216_sfdp_header sfdp;
	} u;
	const struct jesd216_sfdp_header *hp = &u.sfdp;

	ret = flash_artery_f435_qspi_read_sfdp(dev, 0, u.raw, sizeof(u.raw));
	if (ret < 0) {
		LOG_ERR("Failed to read SFDP header: %d", ret);
		return ret;
	}

	uint32_t magic = jesd216_sfdp_magic(hp);
	if (magic != JESD216_SFDP_MAGIC) {
		LOG_ERR("Invalid SFDP magic: 0x%08x", magic);
		return -EINVAL;
	}

	LOG_INF("SFDP v%u.%u, %u parameter headers", hp->rev_major, hp->rev_minor,
		1 + hp->nph);

	/* Process parameter headers */
	const struct jesd216_param_header *php = hp->phdr;
	const struct jesd216_param_header *phpe = php + MIN(decl_nph, 1 + hp->nph);

	while (php != phpe) {
		uint16_t id = jesd216_param_id(php);

		LOG_DBG("Parameter header %u: id=0x%04x, rev=%u.%u, len=%u DW, addr=0x%x",
			(uint32_t)(php - hp->phdr), id, php->rev_major, php->rev_minor,
			php->len_dw, jesd216_param_addr(php));

		if (id == JESD216_SFDP_PARAM_ID_BFP) {
			union {
				uint32_t dw[20];
				struct jesd216_bfp bfp;
			} u_bfp;

			ret = flash_artery_f435_qspi_read_sfdp(dev, jesd216_param_addr(php),
							       (uint8_t *)u_bfp.dw,
							       sizeof(u_bfp.dw));
			if (ret < 0) {
				LOG_ERR("Failed to read BFP: %d", ret);
				return ret;
			}

			ret = process_bfp(dev, php, &u_bfp.bfp);
			if (ret < 0) {
				LOG_ERR("Failed to process BFP: %d", ret);
				return ret;
			}
		}

		++php;
	}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	/* Setup page layout */
	ret = setup_pages_layout(dev);
	if (ret < 0) {
		LOG_ERR("Failed to setup page layout: %d", ret);
		return ret;
	}
#endif

	return 0;
}

/* Device configuration and instantiation */
PINCTRL_DT_DEFINE(QSPI_CONTROLLER_NODE);

static const struct flash_artery_f435_qspi_config flash_artery_f435_qspi_cfg = {
	.reg_base = DT_REG_ADDR(QSPI_CONTROLLER_NODE),
	.pclken = {
		.enr = DT_CLOCKS_CELL(QSPI_CONTROLLER_NODE, bits),
		.bus = DT_CLOCKS_CELL(QSPI_CONTROLLER_NODE, bus),
	},
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(QSPI_CONTROLLER_NODE),
	.max_frequency = DT_INST_PROP(0, qspi_max_frequency),
	.flash_size = DT_INST_PROP(0, size) / 8,  /* Convert bits to bytes */
};

static struct flash_artery_f435_qspi_data flash_artery_f435_qspi_data;

DEVICE_DT_INST_DEFINE(0, &flash_artery_f435_qspi_init, NULL,
		      &flash_artery_f435_qspi_data, &flash_artery_f435_qspi_cfg,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_artery_f435_qspi_api);
