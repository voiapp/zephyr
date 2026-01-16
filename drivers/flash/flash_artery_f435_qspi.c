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

/* Generic NOR flash commands (JESD216 compatible) */
#define CMD_READ_SR2                    0x35  /* Read Status Register-2 (common) */
#define CMD_READ_SR3                    0x15  /* Read Status Register-3 (Winbond/others) */
#define CMD_WRITE_SR2                   0x31  /* Write Status Register-2 */
#define CMD_FAST_READ                   0x0B  /* Fast Read (1-1-1) */
#define CMD_FAST_READ_DUAL_OUT          0x3B  /* Fast Read Dual Output (1-1-2) */
#define CMD_FAST_READ_QUAD_OUT          0x6B  /* Fast Read Quad Output (1-1-4) */
#define CMD_QUAD_PAGE_PROGRAM           0x32  /* Quad Input Page Program */
#define CMD_ENTER_4B_ADDR               0xB7  /* Enter 4-byte address mode */
#define CMD_EXIT_4B_ADDR                0xE9  /* Exit 4-byte address mode */
#define CMD_ENABLE_RESET                0x66  /* Enable Reset */
#define CMD_RESET_DEVICE                0x99  /* Reset Device */

/* Common erase commands (JESD216 typical) */
#define CMD_ERASE_4K                    0x20  /* 4 KiB Sector Erase */
#define CMD_ERASE_32K                   0x52  /* 32 KiB Block Erase */
#define CMD_ERASE_64K                   0xD8  /* 64 KiB Block Erase */

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
#define QSPI_OPMODE_114         0x2  /* 1-1-4 (quad output) - use for 0x6B */
#define QSPI_OPMODE_122         0x3  /* 1-2-2 (dual I/O) */
#define QSPI_OPMODE_144         0x4  /* 1-4-4 (quad I/O) - avoid for W25Q512 */

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
	
	/* DTS overrides for quirky parts */
	uint8_t override_addr_bytes;     /* 0=auto, 3/4=force */
	uint8_t override_read_opcode;    /* 0=auto */
	uint8_t override_read_dummy;     /* 0=auto */
	bool disable_sfdp;               /* Skip SFDP entirely */
};

/* Runtime capability structure for model-agnostic operation */
struct flash_capabilities {
	/* Read capabilities */
	uint8_t read_opcode;
	uint8_t read_opmode;
	uint8_t read_dummy_cycles;
	
	/* Program capabilities */
	uint8_t prog_opcode;
	uint8_t prog_opmode;
	
	/* Erase capabilities (from SFDP or fallback) */
	struct jesd216_erase_type erase_types[JESD216_NUM_ERASE_TYPES];
	
	/* Addressing */
	uint8_t addr_bytes;  /* 3 or 4 */
	
	/* Status polling */
	uint8_t busy_opcode;
	uint8_t busy_bit;
	
	/* Flags */
	bool quad_enabled : 1;
	bool supports_4b_opcodes : 1;
	bool in_4b_mode : 1;
	bool reset_supported : 1;
};

struct flash_artery_f435_qspi_data {
	struct k_sem sem;
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
	struct flash_capabilities caps;
	uint16_t page_size;
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

/* Generic FIFO wait helper */
static int qspi_wait_fifo(const struct flash_artery_f435_qspi_config *cfg,
			  uint32_t fifo_bit, uint32_t timeout_ms)
{
	uint32_t start = k_uptime_get_32();
	while (!(qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET) & fifo_bit)) {
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

	LOG_DBG(">>> QSPI cmd: inst=0x%02x addr=0x%08x alen=%u dummy=%u mode=%u dcnt=%u wden=%d rst=%d",
		instruction, address, addr_len, dummy_cycles, opmode, data_count, write_data_enable, read_status);

	qspi_prepare_cmd(cfg);  /* Reset command engine and clear CMDSTS */

	/* Enforce constraints for read_status mode */
	if (read_status) {
		if (data_count != 0 || write_data_enable) {
			LOG_ERR("Invalid params for read_status: dcnt=%u wden=%d", data_count, write_data_enable);
			return -EINVAL;
		}
	}

	/* Write command registers in sequence: CMD_W0, W1, W2, then W3 (W3 triggers execution) */
	qspi_write_reg(cfg, QSPI_CMD_W0_OFFSET, address);

	cmd_w1 = (addr_len << QSPI_CMD_W1_ADRLEN_POS) |
		 (dummy_cycles << QSPI_CMD_W1_DUM2_POS) |
		 ((instruction != 0 ? QSPI_INSLEN_1_BYTE : QSPI_INSLEN_0_BYTE) << QSPI_CMD_W1_INSLEN_POS);
	cmd_w1 &= ~(BIT(28) | BIT(29) | BIT(30) | BIT(31) | BIT(26) | BIT(27));  /* Clear PEMEN and reserved */
	qspi_write_reg(cfg, QSPI_CMD_W1_OFFSET, cmd_w1);
	qspi_write_reg(cfg, QSPI_CMD_W2_OFFSET, data_count);

	cmd_w3 = (instruction << QSPI_CMD_W3_INSC_POS) | (opmode << QSPI_CMD_W3_OPMODE_POS);
	if (write_data_enable) {
		cmd_w3 |= QSPI_CMD_W3_WDEN;
	}
	if (read_status) {
		cmd_w3 |= QSPI_CMD_W3_RSTSEN;
	}
	
	(void)flash_write_enable;  /* Unused parameter */
	qspi_write_reg(cfg, QSPI_CMD_W3_OFFSET, cmd_w3);  /* Triggers execution */

	return 0;
}

/* Generic status register read with error handling */
static int qspi_read_sr(const struct device *dev, uint8_t cmd, uint8_t *sr_out)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	int ret;

	qspi_exec_cmd(cfg, cmd, 0, QSPI_ADRLEN_0_BYTE, 0, QSPI_OPMODE_111, false, false, true, 0);
	ret = qspi_wait_cmd_complete(cfg, 100);
	if (ret < 0) {
		return ret;
	}

	*sr_out = qspi_read_reg(cfg, QSPI_RSTS_OFFSET) & QSPI_RSTS_SPISTS_MASK;
	return 0;
}

static int qspi_read_sr1(const struct device *dev, uint8_t *sr1)
{
	return qspi_read_sr(dev, SPI_NOR_CMD_RDSR, sr1);
}

static int qspi_read_sr2(const struct device *dev, uint8_t *sr2)
{
	return qspi_read_sr(dev, CMD_READ_SR2, sr2);
}

static int qspi_read_sr3(const struct device *dev, uint8_t *sr3)
{
	return qspi_read_sr(dev, CMD_READ_SR3, sr3);
}

/* Get current address byte count (centralized) */
static inline uint8_t qspi_addr_bytes_for_op(const struct device *dev)
{
	struct flash_artery_f435_qspi_data *data = dev->data;
	return data->caps.addr_bytes;
}

/* Read flash busy status using configured opcode */
static int qspi_flash_read_status(const struct device *dev, uint8_t *status)
{
	struct flash_artery_f435_qspi_data *data = dev->data;
	return qspi_read_sr(dev, data->caps.busy_opcode, status);
}

/* Wait for flash to be ready (not busy) */
static int qspi_flash_wait_ready(const struct device *dev, uint32_t timeout_ms)
{
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint32_t start = k_uptime_get_32();
	uint8_t status;
	int ret;

	do {
		ret = qspi_flash_read_status(dev, &status);
		if (ret < 0) {
			return ret;
		}
		
		if (!(status & data->caps.busy_bit)) {
			return 0;
		}
		
		if (k_uptime_get_32() - start > timeout_ms) {
			LOG_ERR("Timeout waiting for flash ready (status=0x%02x)", status);
			return -ETIMEDOUT;
		}
		
		k_sleep(K_MSEC(1));  /* Avoid hot spinning */
	} while (true);
}

/* Software reset sequence (generic: 66h then 99h) - optional, recovers from stuck states */
static int flash_software_reset(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	int ret;

	if (!data->caps.reset_supported) {
		return 0;  /* Skip if not supported */
	}

	LOG_DBG("Performing software reset");

	/* Enable Reset (66h) then Reset Device (99h) */
	qspi_exec_cmd(cfg, CMD_ENABLE_RESET, 0, QSPI_ADRLEN_0_BYTE, 0,
		       QSPI_OPMODE_111, false, true, false, 0);
	if ((ret = qspi_wait_cmd_complete(cfg, 100)) < 0) {
		LOG_WRN("Reset enable failed, ignoring");
		return 0;  /* Non-fatal */
	}

	qspi_exec_cmd(cfg, CMD_RESET_DEVICE, 0, QSPI_ADRLEN_0_BYTE, 0,
		       QSPI_OPMODE_111, false, true, false, 0);
	if ((ret = qspi_wait_cmd_complete(cfg, 100)) < 0) {
		LOG_WRN("Reset device failed, ignoring");
		return 0;  /* Non-fatal */
	}

	k_busy_wait(50);  /* Wait tRST */
	LOG_INF("Software reset complete");
	return 0;
}

/* Send write enable command */
static int qspi_write_enable(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	int ret;
	uint8_t sr1;

	qspi_exec_cmd(cfg, SPI_NOR_CMD_WREN, 0, QSPI_ADRLEN_0_BYTE, 0,
		      QSPI_OPMODE_111, false, true, false, 0);

	if ((ret = qspi_wait_cmd_complete(cfg, 1000)) < 0) {
		return ret;
	}

	/* Verify WEL bit is set after WREN */
	if ((ret = qspi_read_sr1(dev, &sr1)) < 0) {
		return ret;
	}

	if (!(sr1 & W25Q_SR1_WEL)) {
		LOG_ERR("WREN failed: WEL bit not set! SR1=0x%02x", sr1);
		return -EIO;
	}

	return 0;
}

/* Enter 4-byte address mode (B7h) - for parts that support mode switching */
static int flash_enter_4byte_mode(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint8_t sr3;
	int ret;

	LOG_INF("Entering 4-byte address mode");

	qspi_exec_cmd(cfg, CMD_ENTER_4B_ADDR, 0, QSPI_ADRLEN_0_BYTE, 0,
		      QSPI_OPMODE_111, false, true, false, 0);
	if ((ret = qspi_wait_cmd_complete(cfg, 100)) < 0) {
		LOG_WRN("Failed to enter 4-byte mode command: %d", ret);
		return ret;
	}

	k_busy_wait(10);

	/* Try to verify ADS bit in SR3 (Winbond/similar) - optional */
	if (qspi_read_sr3(dev, &sr3) == 0) {
		if (sr3 & W25Q_SR3_ADS) {
			LOG_INF("4-byte mode verified (ADS=1)");
		} else {
			LOG_WRN("SR3.ADS=0 after enter 4B command, may not be supported");
		}
	}

	data->caps.in_4b_mode = true;
	data->caps.addr_bytes = 4;
	LOG_INF("4-byte addressing mode enabled");
	return 0;
}

/* Exit 4-byte address mode (E9h) - cleanup for pm_device */
__maybe_unused
static int flash_exit_4byte_mode(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	int ret;

	if (!data->caps.in_4b_mode) {
		return 0;
	}

	LOG_DBG("Exiting 4-byte address mode");

	qspi_exec_cmd(cfg, CMD_EXIT_4B_ADDR, 0, QSPI_ADRLEN_0_BYTE, 0,
		      QSPI_OPMODE_111, false, true, false, 0);
	if ((ret = qspi_wait_cmd_complete(cfg, 100)) < 0) {
		LOG_WRN("Failed to exit 4-byte mode: %d", ret);
		return ret;
	}

	k_busy_wait(10);
	data->caps.in_4b_mode = false;
	data->caps.addr_bytes = 3;
	return 0;
}


/* Global Block/Sector Unlock (98h) - Winbond-specific, try but don't fail if unsupported */
static int flash_global_unlock(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	int ret;

	LOG_DBG("Attempting global block unlock");

	if ((ret = qspi_write_enable(dev)) < 0) {
		return ret;
	}

	qspi_exec_cmd(cfg, 0x98, 0, QSPI_ADRLEN_0_BYTE, 0,  /* 0x98 = Global Block Unlock */
		       QSPI_OPMODE_111, false, true, false, 0);

	if ((ret = qspi_wait_cmd_complete(cfg, 100)) < 0 ||
	    (ret = qspi_flash_wait_ready(dev, W25Q_TIMEOUT_WRITE_STATUS)) < 0) {
		LOG_WRN("Global unlock failed (may not be supported): %d", ret);
		return 0;  /* Non-fatal */
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

	qspi_exec_cmd(cfg, SPI_NOR_CMD_RDID, 0, QSPI_ADRLEN_0_BYTE, 0,
		      QSPI_OPMODE_111, false, false, false, len);

	if ((ret = qspi_wait_fifo(cfg, QSPI_FIFOSTS_RXFIFORDY, 1000)) < 0) {
		LOG_ERR("RX FIFO timeout");
		k_sem_give(&data->sem);
		return ret;
	}

	for (size_t i = 0; i < len; i++) {
		jedec_id[i] = qspi_read_byte(cfg);
	}

	ret = qspi_wait_cmd_complete(cfg, 1000);
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

	/* Use runtime-selected read command from capabilities */
	uint8_t read_cmd = data->caps.read_opcode;
	uint8_t opmode = data->caps.read_opmode;
	uint8_t dummy_cycles = data->caps.read_dummy_cycles;
	uint8_t addr_len = qspi_addr_bytes_for_op(dev);

	qspi_exec_cmd(cfg, read_cmd, addr, addr_len, dummy_cycles, opmode, false, false, false, size);

	/* Read data from FIFO in chunks before waiting for command complete */
	while (size > 0) {
		to_read = MIN(size, QSPI_FIFO_DEPTH);
		if ((ret = qspi_wait_fifo(cfg, QSPI_FIFOSTS_RXFIFORDY, 1000)) < 0) {
			LOG_ERR("RX FIFO timeout");
			k_sem_give(&data->sem);
			return ret;
		}
		for (uint32_t i = 0; i < to_read; i++) {
			*buf++ = qspi_read_byte(cfg);
		}
		size -= to_read;
	}

	ret = qspi_wait_cmd_complete(cfg, 1000);
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

		/* Use runtime-selected program command from capabilities */
		uint8_t write_cmd = data->caps.prog_opcode;
		uint8_t opmode = data->caps.prog_opmode;
		uint8_t addr_len = qspi_addr_bytes_for_op(dev);

		qspi_exec_cmd(cfg, write_cmd, addr, addr_len, 0, opmode, true, true, false, to_write);

		/* Write data bytes - wait for TX FIFO ready before each byte */
		for (uint32_t i = 0; i < to_write; i++) {
			if ((ret = qspi_wait_fifo(cfg, QSPI_FIFOSTS_TXFIFORDY, 1000)) < 0) {
				LOG_ERR("TX FIFO timeout at byte %u", i);
				break;
			}
			qspi_write_byte(cfg, *buf++);
		}

		if (ret < 0 || (ret = qspi_wait_cmd_complete(cfg, 1000)) < 0 ||
		    (ret = qspi_flash_wait_ready(dev, W25Q_TIMEOUT_PAGE_PROGRAM)) < 0) {
			LOG_ERR("Page program failed at addr=0x%lx", (long)addr);
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

		if ((ret = qspi_write_enable(dev)) < 0) {
			goto out;
		}

		qspi_exec_cmd(cfg, SPI_NOR_CMD_CE, 0, QSPI_ADRLEN_0_BYTE, 0,
			       QSPI_OPMODE_111, false, true, false, 0);

		if ((ret = qspi_wait_cmd_complete(cfg, 1000)) < 0 ||
		    (ret = qspi_flash_wait_ready(dev, W25Q_TIMEOUT_CHIP_ERASE)) < 0) {
			goto out;
		}

		LOG_INF("Chip erase complete");
		goto out;
	}

	/* Use capability-driven erase (from SFDP or fallback) */
	uint8_t addr_len = qspi_addr_bytes_for_op(dev);

	while (size > 0) {
		const struct jesd216_erase_type *best_etp = NULL;
		uint32_t timeout_ms;

		/* Find best (largest) erase type that fits */
		for (uint8_t ei = 0; ei < JESD216_NUM_ERASE_TYPES; ++ei) {
			const struct jesd216_erase_type *etp = &data->caps.erase_types[ei];

			if ((etp->cmd != 0) && (etp->exp != 0) &&
			    SPI_NOR_IS_ALIGNED(addr, etp->exp) &&
			    SPI_NOR_IS_ALIGNED(size, etp->exp) &&
			    ((best_etp == NULL) || (etp->exp > best_etp->exp))) {
				best_etp = etp;
			}
		}

		if (best_etp == NULL) {
			LOG_ERR("No suitable erase type for addr=0x%lx, size=%zu", (long)addr, size);
			ret = -EINVAL;
			break;
		}

		uint32_t erase_size = BIT(best_etp->exp);
		
		/* Select timeout based on erase size */
		if (erase_size <= 4096) {
			timeout_ms = W25Q_TIMEOUT_SECTOR_ERASE;
		} else if (erase_size <= 32768) {
			timeout_ms = W25Q_TIMEOUT_BLOCK_ERASE_32K;
		} else {
			timeout_ms = W25Q_TIMEOUT_BLOCK_ERASE_64K;
		}

		if ((ret = qspi_write_enable(dev)) < 0) {
			break;
		}

		qspi_exec_cmd(cfg, best_etp->cmd, addr, addr_len, 0, QSPI_OPMODE_111, false, true, false, 0);

		if ((ret = qspi_wait_cmd_complete(cfg, 1000)) < 0 ||
		    (ret = qspi_flash_wait_ready(dev, timeout_ms)) < 0) {
			LOG_ERR("Erase failed at addr=0x%lx", (long)addr);
			break;
		}

		LOG_DBG("Erased %u bytes at 0x%lx (cmd=0x%02x)", erase_size, (long)addr, best_etp->cmd);

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

/* Enable Quad - abstracted for different QER methods (SFDP 15h DW15[22:20]) */
static int flash_enable_quad(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint8_t sr2;
	int ret;

	/* Try common "QE in SR2 bit 1" method (QER=001b or 010b in SFDP) */
	if ((ret = qspi_read_sr2(dev, &sr2)) < 0) {
		LOG_WRN("Cannot read SR2, quad mode disabled");
		return -ENOTSUP;
	}

	if (sr2 & W25Q_SR2_QE) {
		LOG_INF("Quad Enable (QE) bit already set");
		data->caps.quad_enabled = true;
		return 0;
	}

	LOG_INF("Setting Quad Enable (QE) bit in SR2");

	if ((ret = qspi_write_enable(dev)) < 0) {
		return ret;
	}

	qspi_exec_cmd(cfg, CMD_WRITE_SR2, 0, QSPI_ADRLEN_0_BYTE, 0,
		       QSPI_OPMODE_111, true, true, false, 1);
	qspi_write_byte(cfg, sr2 | W25Q_SR2_QE);

	if ((ret = qspi_wait_cmd_complete(cfg, 100)) < 0 ||
	    (ret = qspi_flash_wait_ready(dev, W25Q_TIMEOUT_WRITE_STATUS)) < 0) {
		LOG_WRN("Failed to write SR2, quad disabled");
		return -EIO;
	}

	if ((ret = qspi_read_sr2(dev, &sr2)) < 0 || !(sr2 & W25Q_SR2_QE)) {
		LOG_WRN("QE bit verification failed, quad disabled");
		return -EIO;
	}
	
	data->caps.quad_enabled = true;
	LOG_INF("Quad Enable (QE) bit successfully set");
	return 0;
}

/* Read SFDP data (uses 3-byte addressing always) */
static int flash_read_sfdp(const struct device *dev, off_t addr, void *data_buf, size_t size)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint8_t *buf = (uint8_t *)data_buf;
	int ret;

	if (k_sem_take(&data->sem, K_SECONDS(1)) != 0) {
		return -EBUSY;
	}

	/* SFDP read: always 3-byte address, 8 dummy cycles */
	qspi_exec_cmd(cfg, JESD216_CMD_READ_SFDP, addr, QSPI_ADRLEN_3_BYTE, 8,
		      QSPI_OPMODE_111, false, false, false, size);

	if ((ret = qspi_wait_fifo(cfg, QSPI_FIFOSTS_RXFIFORDY, 1000)) < 0) {
		k_sem_give(&data->sem);
		return ret;
	}

	for (size_t i = 0; i < size; i++) {
		buf[i] = qspi_read_byte(cfg);
	}

	ret = qspi_wait_cmd_complete(cfg, 1000);
	k_sem_give(&data->sem);
	return ret;
}

/* Setup conservative fallback capabilities (no SFDP) */
static void setup_fallback_caps(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;

	LOG_INF("Using conservative fallback capabilities");

	/* Read: 1-1-1 fast read (0x0B) with 8 dummy cycles - universally supported */
	data->caps.read_opcode = CMD_FAST_READ;
	data->caps.read_opmode = QSPI_OPMODE_111;
	data->caps.read_dummy_cycles = 8;

	/* Program: standard 1-1-1 page program */
	data->caps.prog_opcode = SPI_NOR_CMD_PP;
	data->caps.prog_opmode = QSPI_OPMODE_111;

	/* Erase: assume common 4KB + 64KB */
	data->caps.erase_types[0].cmd = CMD_ERASE_4K;
	data->caps.erase_types[0].exp = 12;  /* 4KB */
	data->caps.erase_types[1].cmd = CMD_ERASE_64K;
	data->caps.erase_types[1].exp = 16;  /* 64KB */

	/* Addressing: 3-byte unless flash >16MB */
	if (cfg->flash_size > (16 * 1024 * 1024)) {
		LOG_INF("Flash >16MB, will need 4-byte addressing");
		data->caps.addr_bytes = 4;
	} else {
		data->caps.addr_bytes = 3;
	}

	/* Status polling: standard RDSR */
	data->caps.busy_opcode = SPI_NOR_CMD_RDSR;
	data->caps.busy_bit = W25Q_SR1_BUSY;

	/* Flags */
	data->caps.quad_enabled = false;
	data->caps.supports_4b_opcodes = false;
	data->caps.in_4b_mode = false;
	data->caps.reset_supported = true;  /* Try reset, ignore if fails */
}

/* Parse SFDP and setup capabilities */
static int setup_caps_from_sfdp(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	const uint8_t decl_nph = 3;
	union {
		uint8_t raw[JESD216_SFDP_SIZE(decl_nph)];
		struct jesd216_sfdp_header sfdp;
	} u;
	const struct jesd216_sfdp_header *hp = &u.sfdp;
	int ret;

	/* Try to read SFDP header */
	ret = flash_read_sfdp(dev, 0, u.raw, sizeof(u.raw));
	if (ret < 0) {
		LOG_WRN("SFDP read failed: %d, using fallback", ret);
		return -ENOTSUP;
	}

	uint32_t magic = jesd216_sfdp_magic(hp);
	if (magic != JESD216_SFDP_MAGIC) {
		LOG_WRN("Invalid SFDP magic: 0x%08x, using fallback", magic);
		return -EINVAL;
	}

	LOG_INF("SFDP v%u.%u, %u parameter headers", hp->rev_major, hp->rev_minor, 1 + hp->nph);

	/* Find and process BFP */
	const struct jesd216_param_header *php = hp->phdr;
	const struct jesd216_param_header *phpe = php + MIN(decl_nph, 1 + hp->nph);

	while (php != phpe) {
		uint16_t id = jesd216_param_id(php);
		if (id == JESD216_SFDP_PARAM_ID_BFP) {
			union {
				uint32_t dw[20];
				struct jesd216_bfp bfp;
			} u_bfp;

			ret = flash_read_sfdp(dev, jesd216_param_addr(php),
					      (uint8_t *)u_bfp.dw, sizeof(u_bfp.dw));
			if (ret < 0) {
				LOG_WRN("BFP read failed: %d", ret);
				return ret;
			}

			/* Extract density */
			size_t flash_size = jesd216_bfp_density(&u_bfp.bfp) / 8U;
			if (flash_size != cfg->flash_size) {
				LOG_WRN("SFDP size %u differs from DT size %zu",
					flash_size, cfg->flash_size);
			}
			LOG_INF("SFDP flash size: %u bytes (%u MiB)", flash_size, flash_size >> 20);

			/* Extract erase types */
			memset(data->caps.erase_types, 0, sizeof(data->caps.erase_types));
			for (uint8_t ti = 1; ti <= JESD216_NUM_ERASE_TYPES; ++ti) {
				struct jesd216_erase_type *etp = &data->caps.erase_types[ti - 1];
				if (jesd216_bfp_erase(&u_bfp.bfp, ti, etp) == 0 && etp->cmd != 0) {
					LOG_INF("Erase type %u: size=%lu, cmd=0x%02x",
						ti, (unsigned long)BIT(etp->exp), etp->cmd);
				}
			}

			/* Extract page size */
			data->page_size = jesd216_bfp_page_size(php, &u_bfp.bfp);
			LOG_INF("Page size: %u bytes", data->page_size);

			/* Try quad output read (1-1-4) first, fallback to fast read */
			struct jesd216_instr quad_instr;
			if (jesd216_bfp_read_support(php, &u_bfp.bfp, JESD216_MODE_114, &quad_instr) > 0) {
				LOG_INF("SFDP: Quad output (1-1-4) supported: cmd=0x%02x, wait=%u",
					quad_instr.instr, quad_instr.wait_states);
				data->caps.read_opcode = quad_instr.instr;
				data->caps.read_opmode = QSPI_OPMODE_114;
				data->caps.read_dummy_cycles = quad_instr.wait_states;
			} else {
				LOG_INF("SFDP: Using 1-1-1 fast read");
				data->caps.read_opcode = CMD_FAST_READ;
				data->caps.read_opmode = QSPI_OPMODE_111;
				data->caps.read_dummy_cycles = 8;
			}

			/* Addressing mode */
			uint8_t addr_mode = jesd216_bfp_addrbytes(&u_bfp.bfp);
			if (addr_mode == JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_4B) {
				data->caps.addr_bytes = 4;
			} else if (flash_size > (16 * 1024 * 1024)) {
				/* Need 4-byte for >16MB */
				data->caps.addr_bytes = 4;
			} else {
				data->caps.addr_bytes = 3;
			}

			return 0;
		}
		++php;
	}

	LOG_WRN("No BFP in SFDP, using fallback");
	return -ENOENT;
}

/* Detect and setup capabilities: SFDP → fallback */
static int detect_and_setup_capabilities(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	int ret;

	/* Initialize defaults */
	data->caps.busy_opcode = SPI_NOR_CMD_RDSR;
	data->caps.busy_bit = W25Q_SR1_BUSY;
	data->caps.prog_opcode = SPI_NOR_CMD_PP;
	data->caps.prog_opmode = QSPI_OPMODE_111;
	data->caps.reset_supported = true;

	/* Apply DTS overrides first */
	if (cfg->override_addr_bytes != 0) {
		data->caps.addr_bytes = cfg->override_addr_bytes;
		LOG_INF("DTS override: addr_bytes=%u", data->caps.addr_bytes);
	}

	/* Try SFDP if not disabled */
	if (!cfg->disable_sfdp) {
		ret = setup_caps_from_sfdp(dev);
		if (ret == 0) {
			LOG_INF("Capabilities configured from SFDP");
			goto apply_overrides;
		}
	}

	/* Fallback to conservative capabilities */
	setup_fallback_caps(dev);

apply_overrides:
	/* Apply remaining DTS overrides */
	if (cfg->override_read_opcode != 0) {
		data->caps.read_opcode = cfg->override_read_opcode;
		LOG_INF("DTS override: read_opcode=0x%02x", data->caps.read_opcode);
	}
	if (cfg->override_read_dummy != 0) {
		data->caps.read_dummy_cycles = cfg->override_read_dummy;
		LOG_INF("DTS override: read_dummy=%u", data->caps.read_dummy_cycles);
	}

	return 0;
}

/* Print comprehensive capability information for debugging */
static void print_flash_capabilities(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	const char *mode_str[] = {"111", "112", "114", "122", "144"};
	const char *addr_str = data->caps.in_4b_mode ? "4B mode" : 
			       data->caps.supports_4b_opcodes ? "4B opcodes" : "3B";

	LOG_DBG("=== Flash Capabilities ===");
	LOG_DBG("Read: 0x%02x mode=%s dummy=%u | Prog: 0x%02x mode=%s",
		data->caps.read_opcode, mode_str[data->caps.read_opmode], data->caps.read_dummy_cycles,
		data->caps.prog_opcode, mode_str[data->caps.prog_opmode]);
	
	LOG_DBG("Erase: ", "");
	for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
		const struct jesd216_erase_type *etp = &data->caps.erase_types[i];
		if (etp->cmd != 0 && etp->exp != 0) {
			LOG_DBG("  0x%02x=%luKB", etp->cmd, (unsigned long)BIT(etp->exp) / 1024);
		}
	}
	
	LOG_DBG("Addr: %uB (%s) | Busy: 0x%02x/0x%02x | Quad: %c | Reset: %c | Size: %zuMB",
		data->caps.addr_bytes, addr_str, data->caps.busy_opcode, data->caps.busy_bit,
		data->caps.quad_enabled ? 'Y' : 'N', data->caps.reset_supported ? 'Y' : 'N',
		cfg->flash_size / (1024 * 1024));
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
/* Setup flash page layout from capabilities */
static int setup_pages_layout(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint32_t min_erase_size = 0;

	/* Find smallest erase size */
	for (size_t i = 0; i < JESD216_NUM_ERASE_TYPES; ++i) {
		const struct jesd216_erase_type *etp = &data->caps.erase_types[i];
		if (etp->cmd != 0 && etp->exp != 0) {
			uint32_t erase_size = BIT(etp->exp);
			if (min_erase_size == 0 || erase_size < min_erase_size) {
				min_erase_size = erase_size;
			}
		}
	}

	if (min_erase_size == 0) {
		LOG_ERR("No valid erase types found");
		return -ENOTSUP;
	}

	data->layout.pages_size = min_erase_size;
	data->layout.pages_count = cfg->flash_size / min_erase_size;
	LOG_INF("Flash layout: %u pages x %u bytes", data->layout.pages_count, data->layout.pages_size);

	return 0;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */


/* Initialize the QSPI peripheral hardware */
static void qspi_hw_init(const struct flash_artery_f435_qspi_config *cfg)
{
	uint32_t ctrl, timeout;

	LOG_DBG("QSPI hardware initialization at base 0x%08x", cfg->reg_base);

	/* Enable QSPI1 clock via CRM */
	uint32_t crm_ahben2 = sys_read32(CRM_BASE + CRM_AHBEN3_OFFSET);
	sys_write32(crm_ahben2 | CRM_AHBEN3_QSPI1EN, CRM_BASE + CRM_AHBEN3_OFFSET);
	k_busy_wait(100);

	/* XIP disable sequence - ensure peripheral is in known state */
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
	LOG_DBG("QSPI_CTRL initial: 0x%08x", ctrl);
	
	/* Wait for TX FIFO empty */
	timeout = 10000;
	while (!(qspi_read_reg(cfg, QSPI_FIFOSTS_OFFSET) & QSPI_FIFOSTS_TXFIFORDY) && timeout--) {
		k_busy_wait(1);
	}
	k_busy_wait(10);

	/* Flush and reset QSPI state */
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET) | QSPI_CTRL_XIPRCMDF;
	qspi_write_reg(cfg, QSPI_CTRL_OFFSET, ctrl);

	/* Wait for abort bit to clear */
	timeout = 10000;
	while ((qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & QSPI_CTRL_ABORT) && timeout--) {
		k_busy_wait(1);
	}
	k_busy_wait(10);

	/* Disable XIP mode */
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & ~QSPI_CTRL_XIPSEL;
	qspi_write_reg(cfg, QSPI_CTRL_OFFSET, ctrl);

	/* Wait for abort to clear again */
	timeout = 10000;
	while ((qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & QSPI_CTRL_ABORT) && timeout--) {
		k_busy_wait(1);
	}
	k_busy_wait(10);

	/* Assert ABORT to reset command engine */
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET) | QSPI_CTRL_ABORT;
	qspi_write_reg(cfg, QSPI_CTRL_OFFSET, ctrl);
	
	timeout = 10000;
	while ((qspi_read_reg(cfg, QSPI_CTRL_OFFSET) & QSPI_CTRL_ABORT) && timeout--) {
		k_busy_wait(1);
	}
	k_busy_wait(10);
	
	ctrl = qspi_read_reg(cfg, QSPI_CTRL_OFFSET);
	LOG_DBG("XIP disable and ABORT complete, CTRL=0x%08x", ctrl);

	/* Configure control register: clock divider (div 8 = 30MHz) and busy bit offset */
	ctrl = (ctrl & ~(QSPI_CTRL_CLKDIV_MASK | QSPI_CTRL_BUSY_MASK)) |
	       (QSPI_CLK_DIV_8 << QSPI_CTRL_CLKDIV_POS);
	qspi_write_reg(cfg, QSPI_CTRL_OFFSET, ctrl);

	/* Clear any pending command status */
	if (qspi_read_reg(cfg, QSPI_CMDSTS_OFFSET) & QSPI_CMDSTS_CMDSTS) {
		qspi_write_reg(cfg, QSPI_CMDSTS_OFFSET, QSPI_CMDSTS_CMDSTS);
	}

	LOG_DBG("Init complete - CTRL: 0x%08x", qspi_read_reg(cfg, QSPI_CTRL_OFFSET));
}

/* Driver initialization - Follows W25Q512JV datasheet checklist */
static int flash_artery_f435_qspi_init(const struct device *dev)
{
	const struct flash_artery_f435_qspi_config *cfg = dev->config;
	struct flash_artery_f435_qspi_data *data = dev->data;
	uint8_t jedec_id[3] = {0};
	uint8_t sr1, sr2, sr3;
	int ret;

	/* Initialize semaphore */
	k_sem_init(&data->sem, 1, 1);

	/* Initialize data structure */
	data->page_size = SPI_NOR_PAGE_SIZE;
	memset(&data->caps, 0, sizeof(data->caps));

	qspi_hw_init(cfg);

	/* Configure pinctrl AFTER QSPI clock is enabled */
	if ((ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT)) < 0) {
		LOG_ERR("Failed to apply pinctrl state: %d", ret);
		return ret;
	}

	/* Power-up delay for W25Q512JV (tVSL: 20µs min, tPUW: 5ms min) */
	k_sleep(K_MSEC(10));
	LOG_INF("Initializing QSPI NOR flash: %s", dev->name);

	/* Step 1: Software reset to recover from stuck states (optional) */
	if ((ret = flash_software_reset(dev)) < 0) {
		LOG_DBG("Software reset failed: %d (continuing)", ret);
	}

	/* Step 2: Read and validate JEDEC ID */
	if ((ret = flash_artery_f435_qspi_read_jedec_id(dev, jedec_id, sizeof(jedec_id))) < 0) {
		LOG_ERR("Failed to read JEDEC ID: %d", ret);
		return ret;
	}

	LOG_INF("JEDEC ID: %02x %02x %02x", jedec_id[0], jedec_id[1], jedec_id[2]);

	/* Validate JEDEC ID for W25Q512JV */
	if (jedec_id[0] != W25Q512_JEDEC_MFR) {
		LOG_ERR("Unexpected manufacturer ID: 0x%02x (expected 0x%02x)", jedec_id[0], W25Q512_JEDEC_MFR);
		return -ENODEV;
	}
	if (jedec_id[1] != W25Q512_JEDEC_TYPE || jedec_id[2] != W25Q512_JEDEC_CAPACITY) {
		LOG_WRN("JEDEC ID type/capacity: %02x %02x (expected %02x %02x)",
			jedec_id[1], jedec_id[2], W25Q512_JEDEC_TYPE, W25Q512_JEDEC_CAPACITY);
	}

	/* Step 3: Read all status registers to check device state */
	if ((ret = qspi_read_sr1(dev, &sr1)) < 0 ||
	    (ret = qspi_read_sr2(dev, &sr2)) < 0 ||
	    (ret = qspi_read_sr3(dev, &sr3)) < 0) {
		LOG_ERR("Failed to read status registers: %d", ret);
		return ret;
	}
	LOG_INF("Status Registers: SR1=0x%02x SR2=0x%02x SR3=0x%02x", sr1, sr2, sr3);

	if ((sr1 & W25Q_SR1_BUSY) && (ret = qspi_flash_wait_ready(dev, 1000)) < 0) {
		LOG_ERR("Device stuck busy");
		return ret;
	}

	/* Detect capabilities from SFDP or use fallbacks */
	if ((ret = detect_and_setup_capabilities(dev)) < 0) {
		LOG_ERR("Failed to detect capabilities: %d", ret);
		return ret;
	}

	/* Print detected capabilities for debugging */
	print_flash_capabilities(dev);

	/* Try global unlock (Winbond-specific, non-fatal) */
	if (sr3 & W25Q_SR3_WPS) {
		flash_global_unlock(dev);
	}

	/* Try to enable quad mode if read command needs it */
	if (data->caps.read_opmode == QSPI_OPMODE_114 || data->caps.read_opmode == QSPI_OPMODE_144) {
		if ((ret = flash_enable_quad(dev)) < 0) {
			LOG_WRN("Quad enable failed: %d, falling back to 1-1-1", ret);
			/* Fall back to 1-1-1 fast read */
			data->caps.read_opcode = CMD_FAST_READ;
			data->caps.read_opmode = QSPI_OPMODE_111;
			data->caps.read_dummy_cycles = 8;
			data->caps.quad_enabled = false;
		}
	}

	/* Update program opcode based on quad status */
	if (data->caps.quad_enabled) {
		data->caps.prog_opcode = CMD_QUAD_PAGE_PROGRAM;
		data->caps.prog_opmode = QSPI_OPMODE_114;
		LOG_INF("Quad enabled: using 0x32 program");
	}

	/* Enter 4-byte mode if needed (>16MB flash) */
	if (data->caps.addr_bytes == 4 && !data->caps.in_4b_mode) {
		if ((ret = flash_enter_4byte_mode(dev)) < 0) {
			LOG_ERR("Failed to enter 4-byte mode: %d", ret);
			return ret;
		}
	}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	/* Setup page layout from capabilities */
	if ((ret = setup_pages_layout(dev)) < 0) {
		LOG_ERR("Failed to setup page layout: %d", ret);
		return ret;
	}
#endif

	LOG_INF("QSPI NOR flash initialization complete - %zu bytes accessible",
		cfg->flash_size);
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
	
	/* DTS overrides (0 = auto) */
	.override_addr_bytes = DT_INST_PROP_OR(0, override_addr_bytes, 0),
	.override_read_opcode = DT_INST_PROP_OR(0, override_read_opcode, 0),
	.override_read_dummy = DT_INST_PROP_OR(0, override_read_dummy, 0),
	.disable_sfdp = DT_INST_PROP_OR(0, disable_sfdp, false),
};

static struct flash_artery_f435_qspi_data flash_artery_f435_qspi_data;

DEVICE_DT_INST_DEFINE(0, &flash_artery_f435_qspi_init, NULL,
		      &flash_artery_f435_qspi_data, &flash_artery_f435_qspi_cfg,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_artery_f435_qspi_api);
