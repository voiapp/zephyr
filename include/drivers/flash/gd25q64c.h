#ifndef GD25Q64C_H_
#define GD25Q64C_H_

#include <device.h>
#include <stdint.h>
#include <sys/types.h>

int gd25q64c_lock_otp_register(const struct device *dev, uint8_t reg_idx);
int gd25q64c_erase_otp_register(const struct device *dev, uint8_t reg_idx);
int gd25q64c_program_otp_register(const struct device *dev, uint8_t reg_idx,
				  size_t addr, uint8_t *data, size_t size);
int gd25q64c_read_otp_register(const struct device *dev, uint8_t reg_idx,
			       uint32_t addr, uint8_t *buf, size_t size);
/*
 * @brief Read OTP register lock status.
 *
 * @param dev Device struct
 * @param reg_idx OTP register index
 * @param[inout] bool Lock status
 * @return 0 on success, negative errno code otherwise
 */
int gd25q64c_read_otp_register_lock_status(const struct device *dev,
					   uint8_t reg_idx, bool *status);

#endif /* GD25Q64C_H_ */
