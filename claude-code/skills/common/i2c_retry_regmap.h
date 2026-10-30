/* SPDX-License-Identifier: GPL-2.0 */
/*
 * i2c_retry_regmap.h - reference for the gmsl-camera-pipeline skill's I2C retry workaround.
 *
 * WORKAROUND, not a fix: retries an I2C transfer that times out or is not acknowledged.
 * Use it only after the user agreed (I2C check, last table row), and say that it hides
 * an I2C problem that still needs a logic analyzer.
 *
 * Replaces devm_regmap_init_i2c() / devm_cci_regmap_init_i2c() for devices with 16-bit
 * registers and 8-bit values (the ADI GMSL serializers/deserializers and most CCI
 * sensors). The regmap has no bus, only reg_read/reg_write: regmap then performs
 * update_bits, bulk and multi_reg_write operations as single-register accesses, which
 * is what these drivers use. Raw regmap I/O (regmap_raw_read/write) is not supported.
 *
 * Usage, in each driver that needs it:
 *	#include "i2c_retry_regmap.h"
 *	priv->regmap = i2c_retry_devm_regmap_init(client, &my_regmap_config);
 * and for raw i2c_transfer() calls (e.g. max_ser_read_reg/write_reg in max_ser.c):
 *	ret = i2c_retry_transfer(adapter, msgs, num);   (returns 0 or a negative errno)
 *
 * Each translation unit gets its own copy (static functions) and its own recovery
 * counter; failures and recoveries are logged.
 */
#ifndef I2C_RETRY_REGMAP_H
#define I2C_RETRY_REGMAP_H

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

#ifndef I2C_RETRY_COUNT
#define I2C_RETRY_COUNT		1
#endif

static unsigned int i2c_retry_recovered __maybe_unused;

#define I2C_RETRY_SAVE_LEN	4

static int __maybe_unused i2c_retry_transfer(struct i2c_adapter *adapter,
					     struct i2c_msg *msgs, int num)
{
	unsigned int reg = msgs[0].len >= 2 ? (msgs[0].buf[0] << 8) | msgs[0].buf[1] : 0;
	/*
	 * A failed transfer may already have stored read data in a buffer that is also
	 * sent (max_ser_read_reg() reads into its register address buffer): keep the
	 * bytes of the first write message and put them back before each retry.
	 */
	bool save = num >= 1 && !(msgs[0].flags & I2C_M_RD) &&
		    msgs[0].len <= I2C_RETRY_SAVE_LEN;
	u8 saved[I2C_RETRY_SAVE_LEN];
	unsigned int i;
	int ret;

	if (save)
		memcpy(saved, msgs[0].buf, msgs[0].len);

	for (i = 0; ; i++) {
		if (i && save)
			memcpy(msgs[0].buf, saved, msgs[0].len);
		ret = i2c_transfer(adapter, msgs, num);
		if (ret == num) {
			if (i) {
				i2c_retry_recovered++;
				pr_warn("i2c-retry: %s 0x%02x reg 0x%04x OK after %u retries (%u recovered)\n",
					adapter->name, msgs[0].addr, reg, i, i2c_retry_recovered);
			}
			return 0;
		}

		if (i >= I2C_RETRY_COUNT)
			break;

		pr_warn("i2c-retry: %s 0x%02x reg 0x%04x failed: %d, retrying\n",
			adapter->name, msgs[0].addr, reg, ret);
		usleep_range(1000, 2000);
	}

	return ret < 0 ? ret : -EIO;
}

static int i2c_retry_regmap_read(void *context, unsigned int reg, unsigned int *val)
{
	struct i2c_client *client = context;
	u8 buf[2] = { reg >> 8, reg & 0xff };
	u8 data;
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0, .len = 2, .buf = buf },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = 1, .buf = &data },
	};
	int ret;

	ret = i2c_retry_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret)
		return ret;

	*val = data;

	return 0;
}

static int i2c_retry_regmap_write(void *context, unsigned int reg, unsigned int val)
{
	struct i2c_client *client = context;
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_msg msg = {
		.addr = client->addr, .flags = 0, .len = 3, .buf = buf,
	};

	return i2c_retry_transfer(client->adapter, &msg, 1);
}

/*
 * Copies @config (reg_bits must be 16, val_bits 8) and adds the retrying accessors.
 * For a CCI sensor, pass { .reg_bits = 16, .val_bits = 8, .disable_locking = true },
 * as devm_cci_regmap_init_i2c() does.
 */
static __maybe_unused struct regmap *
i2c_retry_devm_regmap_init(struct i2c_client *client, const struct regmap_config *config)
{
	struct regmap_config cfg = *config;

	if (cfg.reg_bits != 16 || cfg.val_bits != 8)
		return ERR_PTR(-EINVAL);

	cfg.reg_read = i2c_retry_regmap_read;
	cfg.reg_write = i2c_retry_regmap_write;

	return devm_regmap_init(&client->dev, NULL, client, &cfg);
}

#endif /* I2C_RETRY_REGMAP_H */
