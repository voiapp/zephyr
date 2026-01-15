/*
 * Copyright (c) 2026 Voi Technology AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT artery_f435_flash_controller

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <zephyr/arch/cpu.h>
#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(flash_artery_f435, CONFIG_FLASH_LOG_LEVEL);

/* AT32F435 Flash register offsets */
#define FLASH_PSR_OFFSET      0x00
#define FLASH_UNLOCK_OFFSET   0x04
#define FLASH_STS_OFFSET      0x0C
#define FLASH_CTRL_OFFSET     0x10
#define FLASH_ADDR_OFFSET     0x14
#define FLASH_UNLOCK2_OFFSET  0x44
#define FLASH_STS2_OFFSET     0x4C
#define FLASH_CTRL2_OFFSET    0x50
#define FLASH_ADDR2_OFFSET    0x54
#define FLASH_CONTR_OFFSET    0x58

/* Flash status register (STS) bits */
#define FLASH_STS_OBF         BIT(0)   /* Operate busy flag */
#define FLASH_STS_PRGMERR     BIT(2)   /* Program error */
#define FLASH_STS_EPPERR      BIT(4)   /* Erase/program protection error */
#define FLASH_STS_ODF         BIT(5)   /* Operate done flag */

/* Flash control register (CTRL) bits */
#define FLASH_CTRL_FPRGM      BIT(0)   /* Flash program */
#define FLASH_CTRL_SECERS     BIT(1)   /* Sector erase */
#define FLASH_CTRL_BANKERS    BIT(2)   /* Bank erase */
#define FLASH_CTRL_BLKERS     BIT(3)   /* Block erase */
#define FLASH_CTRL_ERSTR      BIT(6)   /* Erase start */
#define FLASH_CTRL_OPLK       BIT(7)   /* Operation lock */

/* Flash unlock keys */
#define FLASH_UNLOCK_KEY1     0x45670123U
#define FLASH_UNLOCK_KEY2     0xCDEF89ABU

/* Flash bank addresses - AT32F435RMT7 (4032KB variant) */
#define FLASH_BANK1_START     0x08000000U
#define FLASH_BANK1_END       0x081FFFFFU
#define FLASH_BANK2_START     0x08200000U
#define FLASH_BANK2_END       0x083EFFFFU

/* Flash geometry */
#define FLASH_SECTOR_SIZE     4096U     /* 4 KB sectors */
#define FLASH_WRITE_ALIGN     4U        /* 32-bit write alignment */
#define FLASH_ERASE_VALUE     0xFFU

/* Timeouts (in microseconds) */
#define FLASH_TIMEOUT_MS      5000U     /* 5 seconds max for operations */

struct flash_artery_f435_config {
	uint32_t base_addr;
	uint32_t flash_size;
};

struct flash_artery_f435_data {
	struct k_sem sem;
};

static inline uint32_t flash_read_reg(const struct device *dev, uint32_t offset)
{
	const struct flash_artery_f435_config *cfg = dev->config;
	return sys_read32(cfg->base_addr + offset);
}

static inline void flash_write_reg(const struct device *dev, uint32_t offset, uint32_t value)
{
	const struct flash_artery_f435_config *cfg = dev->config;
	sys_write32(value, cfg->base_addr + offset);
}

static bool is_bank2_addr(uint32_t addr)
{
	return (addr >= FLASH_BANK2_START && addr <= FLASH_BANK2_END);
}

static int flash_wait_ready(const struct device *dev, uint32_t addr, uint32_t timeout_ms)
{
	/* Select correct status register based on address */
	uint32_t sts_offset = is_bank2_addr(addr) ? FLASH_STS2_OFFSET : FLASH_STS_OFFSET;
	uint32_t start = k_uptime_get_32();

	while (k_uptime_get_32() - start < timeout_ms) {
		uint32_t sts = flash_read_reg(dev, sts_offset);

		if (!(sts & FLASH_STS_OBF)) {
			/* Check for errors */
			if (sts & FLASH_STS_PRGMERR) {
				LOG_ERR("Flash program error at 0x%08x", addr);
				/* Clear error flag */
				flash_write_reg(dev, sts_offset, FLASH_STS_PRGMERR);
				return -EIO;
			}
			if (sts & FLASH_STS_EPPERR) {
				LOG_ERR("Flash erase/program protection error at 0x%08x", addr);
				/* Clear error flag */
				flash_write_reg(dev, sts_offset, FLASH_STS_EPPERR);
				return -EACCES;
			}
			return 0;
		}
		k_busy_wait(100);
	}

	LOG_ERR("Flash operation timeout at 0x%08x", addr);
	return -ETIMEDOUT;
}

static void flash_unlock(const struct device *dev)
{
	/* Unlock bank1 */
	flash_write_reg(dev, FLASH_UNLOCK_OFFSET, FLASH_UNLOCK_KEY1);
	flash_write_reg(dev, FLASH_UNLOCK_OFFSET, FLASH_UNLOCK_KEY2);

	/* Unlock bank2 */
	flash_write_reg(dev, FLASH_UNLOCK2_OFFSET, FLASH_UNLOCK_KEY1);
	flash_write_reg(dev, FLASH_UNLOCK2_OFFSET, FLASH_UNLOCK_KEY2);
}

static void flash_lock(const struct device *dev)
{
	uint32_t ctrl;

	/* Lock bank1 */
	ctrl = flash_read_reg(dev, FLASH_CTRL_OFFSET);
	ctrl |= FLASH_CTRL_OPLK;
	flash_write_reg(dev, FLASH_CTRL_OFFSET, ctrl);

	/* Lock bank2 */
	ctrl = flash_read_reg(dev, FLASH_CTRL2_OFFSET);
	ctrl |= FLASH_CTRL_OPLK;
	flash_write_reg(dev, FLASH_CTRL2_OFFSET, ctrl);
}

/*
 * Sector erase routine - should run from RAM on some configurations
 * to avoid flash fetch stalls during erase operation
 */
static __ramfunc int flash_erase_sector_ramfunc(const struct device *dev, uint32_t addr)
{
	uint32_t ctrl_offset = is_bank2_addr(addr) ? FLASH_CTRL2_OFFSET : FLASH_CTRL_OFFSET;
	uint32_t addr_offset = is_bank2_addr(addr) ? FLASH_ADDR2_OFFSET : FLASH_ADDR_OFFSET;
	uint32_t sts_offset = is_bank2_addr(addr) ? FLASH_STS2_OFFSET : FLASH_STS_OFFSET;
	uint32_t ctrl;
	int ret;

	/* Clear any previous error flags */
	flash_write_reg(dev, sts_offset, FLASH_STS_PRGMERR | FLASH_STS_EPPERR);

	/* Wait for flash to be ready */
	ret = flash_wait_ready(dev, addr, FLASH_TIMEOUT_MS);
	if (ret < 0) {
		return ret;
	}

	/* Set sector erase mode */
	ctrl = flash_read_reg(dev, ctrl_offset);
	ctrl |= FLASH_CTRL_SECERS;
	flash_write_reg(dev, ctrl_offset, ctrl);

	/* Write address */
	flash_write_reg(dev, addr_offset, addr);

	/* Start erase */
	ctrl = flash_read_reg(dev, ctrl_offset);
	ctrl |= FLASH_CTRL_ERSTR;
	flash_write_reg(dev, ctrl_offset, ctrl);

	/* Wait for completion */
	ret = flash_wait_ready(dev, addr, FLASH_TIMEOUT_MS);

	/* Clear sector erase bit */
	ctrl = flash_read_reg(dev, ctrl_offset);
	ctrl &= ~FLASH_CTRL_SECERS;
	flash_write_reg(dev, ctrl_offset, ctrl);

	return ret;
}

/*
 * Word programming routine - should run from RAM to avoid fetch stalls
 */
static __ramfunc int flash_program_word_ramfunc(const struct device *dev, uint32_t addr,
						 uint32_t data)
{
	uint32_t ctrl_offset = is_bank2_addr(addr) ? FLASH_CTRL2_OFFSET : FLASH_CTRL_OFFSET;
	uint32_t sts_offset = is_bank2_addr(addr) ? FLASH_STS2_OFFSET : FLASH_STS_OFFSET;
	uint32_t ctrl;
	int ret;
	volatile uint32_t *flash_ptr = (volatile uint32_t *)addr;

	/* Clear any previous error flags */
	flash_write_reg(dev, sts_offset, FLASH_STS_PRGMERR | FLASH_STS_EPPERR);

	/* Wait for flash to be ready */
	ret = flash_wait_ready(dev, addr, FLASH_TIMEOUT_MS);
	if (ret < 0) {
		return ret;
	}

	/* Enable programming */
	ctrl = flash_read_reg(dev, ctrl_offset);
	ctrl |= FLASH_CTRL_FPRGM;
	flash_write_reg(dev, ctrl_offset, ctrl);

	/* Write data (triggers programming) */
	*flash_ptr = data;

	/* Wait for completion */
	ret = flash_wait_ready(dev, addr, FLASH_TIMEOUT_MS);

	/* Disable programming */
	ctrl = flash_read_reg(dev, ctrl_offset);
	ctrl &= ~FLASH_CTRL_FPRGM;
	flash_write_reg(dev, ctrl_offset, ctrl);

	return ret;
}

static int flash_artery_f435_read(const struct device *dev, off_t offset,
				   void *data, size_t len)
{
	const struct flash_artery_f435_config *cfg = dev->config;
	struct flash_artery_f435_data *dev_data = dev->data;
	uint32_t flash_addr;

	if (!data) {
		return -EINVAL;
	}

	if (offset < 0 || (offset + len) > cfg->flash_size) {
		LOG_ERR("Read out of bounds: offset=%ld, len=%zu, size=%u",
			offset, len, cfg->flash_size);
		return -EINVAL;
	}

	k_sem_take(&dev_data->sem, K_FOREVER);

	flash_addr = FLASH_BANK1_START + offset;

	/* Direct memory-mapped read - flash is accessible via AHB bus */
	memcpy(data, (const void *)flash_addr, len);

	k_sem_give(&dev_data->sem);

	return 0;
}

static int flash_artery_f435_write(const struct device *dev, off_t offset,
				    const void *data, size_t len)
{
	const struct flash_artery_f435_config *cfg = dev->config;
	struct flash_artery_f435_data *dev_data = dev->data;
	const uint8_t *src = (const uint8_t *)data;
	uint32_t flash_addr;
	uint32_t key;
	int ret = 0;

	if (!data) {
		return -EINVAL;
	}

	if (offset < 0 || (offset + len) > cfg->flash_size) {
		LOG_ERR("Write out of bounds: offset=%ld, len=%zu, size=%u",
			offset, len, cfg->flash_size);
		return -EINVAL;
	}

	/* Check write alignment */
	if ((offset % FLASH_WRITE_ALIGN) != 0) {
		LOG_ERR("Write offset not aligned to %u bytes", FLASH_WRITE_ALIGN);
		return -EINVAL;
	}

	k_sem_take(&dev_data->sem, K_FOREVER);

	/* Disable interrupts during flash operations */
	key = irq_lock();

	flash_unlock(dev);

	flash_addr = FLASH_BANK1_START + offset;

	/* Write in 32-bit words */
	size_t words = len / 4;
	for (size_t i = 0; i < words; i++) {
		uint32_t word;
		memcpy(&word, &src[i * 4], sizeof(word));

		ret = flash_program_word_ramfunc(dev, flash_addr + (i * 4), word);
		if (ret < 0) {
			LOG_ERR("Failed to program word at 0x%08x", flash_addr + (i * 4));
			goto done;
		}
	}

	/* Handle trailing bytes (if any) - read-modify-write */
	size_t remaining = len % 4;
	if (remaining > 0) {
		uint32_t word = 0xFFFFFFFF;  /* Start with erased value */
		size_t base_offset = words * 4;

		/* Read existing word to preserve unwritten bytes */
		uint32_t existing = *((volatile uint32_t *)(flash_addr + base_offset));
		memcpy(&word, &existing, sizeof(word));

		/* Update with new bytes */
		for (size_t i = 0; i < remaining; i++) {
			((uint8_t *)&word)[i] = src[base_offset + i];
		}

		ret = flash_program_word_ramfunc(dev, flash_addr + base_offset, word);
		if (ret < 0) {
			LOG_ERR("Failed to program trailing bytes");
			goto done;
		}
	}

done:
	flash_lock(dev);
	irq_unlock(key);
	k_sem_give(&dev_data->sem);

	return ret;
}

static int flash_artery_f435_erase(const struct device *dev, off_t offset, size_t len)
{
	const struct flash_artery_f435_config *cfg = dev->config;
	struct flash_artery_f435_data *dev_data = dev->data;
	uint32_t flash_addr;
	uint32_t key;
	int ret = 0;

	if (offset < 0 || (offset + len) > cfg->flash_size) {
		LOG_ERR("Erase out of bounds: offset=%ld, len=%zu, size=%u",
			offset, len, cfg->flash_size);
		return -EINVAL;
	}

	/* Check sector alignment */
	if ((offset % FLASH_SECTOR_SIZE) != 0) {
		LOG_ERR("Erase offset not aligned to sector size (%u)", FLASH_SECTOR_SIZE);
		return -EINVAL;
	}

	/* Round up to sector boundary */
	uint32_t num_sectors = (len + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;

	k_sem_take(&dev_data->sem, K_FOREVER);

	/* Disable interrupts during flash operations */
	key = irq_lock();

	flash_unlock(dev);

	flash_addr = FLASH_BANK1_START + offset;

	for (uint32_t i = 0; i < num_sectors; i++) {
		uint32_t sector_addr = flash_addr + (i * FLASH_SECTOR_SIZE);

		LOG_DBG("Erasing sector %u/%u at 0x%08x", i + 1, num_sectors, sector_addr);

		ret = flash_erase_sector_ramfunc(dev, sector_addr);
		if (ret < 0) {
			LOG_ERR("Failed to erase sector at 0x%08x", sector_addr);
			goto done;
		}
	}

done:
	flash_lock(dev);
	irq_unlock(key);
	k_sem_give(&dev_data->sem);

	return ret;
}

static const struct flash_parameters *
flash_artery_f435_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	static const struct flash_parameters params = {
		.write_block_size = FLASH_WRITE_ALIGN,
		.erase_value = FLASH_ERASE_VALUE,
	};

	return &params;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_artery_f435_layout[] = {
	{
		.pages_count = 32 * 16,  /* Bank1: 32 blocks * 16 sectors */
		.pages_size = FLASH_SECTOR_SIZE,
	},
	{
		.pages_count = 31 * 16,  /* Bank2: 31 blocks * 16 sectors */
		.pages_size = FLASH_SECTOR_SIZE,
	},
};

static void flash_artery_f435_page_layout(const struct device *dev,
					   const struct flash_pages_layout **layout,
					   size_t *layout_size)
{
	ARG_UNUSED(dev);

	*layout = flash_artery_f435_layout;
	*layout_size = ARRAY_SIZE(flash_artery_f435_layout);
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api flash_artery_f435_api = {
	.read = flash_artery_f435_read,
	.write = flash_artery_f435_write,
	.erase = flash_artery_f435_erase,
	.get_parameters = flash_artery_f435_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_artery_f435_page_layout,
#endif
};

static int flash_artery_f435_init(const struct device *dev)
{
	struct flash_artery_f435_data *data = dev->data;

	k_sem_init(&data->sem, 1, 1);

	LOG_INF("Artery AT32F435 flash driver initialized");

	return 0;
}

static struct flash_artery_f435_data flash_artery_f435_data_0;

static const struct flash_artery_f435_config flash_artery_f435_config_0 = {
	.base_addr = DT_INST_REG_ADDR(0),
	.flash_size = DT_REG_SIZE(DT_NODELABEL(flash0)),
};

DEVICE_DT_INST_DEFINE(0, flash_artery_f435_init, NULL,
		      &flash_artery_f435_data_0, &flash_artery_f435_config_0,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_artery_f435_api);
