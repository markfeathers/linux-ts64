/*
 * Copyright (c) 2017 <mark@embeddedarm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define DRIVER_DESC "Driver for TS-N64 Controller board"
#define MODULE_DEVICE_ALIAS "tsn64"

MODULE_AUTHOR("Mark Featherston <mark@embeddecarm.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/* 32-bit i2c reg at 0x0 */
#define P1			0
#define P2			1
#define P3			2
#define P4			3
#define PLAYER_OFFSET		30
#define PLAYER_MASK 0x3

#define CON_L		29
#define CON_R		28
#define CON_CUP		27
#define CON_CDN		26
#define CON_CL		25
#define CON_CR		24
#define CON_A		23
#define CON_B		22
#define CON_Z		21
#define CON_START	20
#define CON_DUP		19
#define CON_DDN		18
#define CON_DL		17
#define CON_DR		16
#define ANA_X_OFFSET	8
#define ANA_Y_OFFSET	0
#define ANA_MASK	0xff

struct tsn64_device {
	struct input_dev *input_dev;
	struct i2c_client *client;
	int irq;
	int irq_gpio;
};

static int ts64_i2c_read(struct i2c_client *client, int addr, uint32_t *status)
{
	uint8_t addr_msg[2];
	uint8_t status_msg[4];
	int ret;
	struct i2c_msg msgs[2];

	addr_msg[0] = ((addr >> 8) & 0xff);
	addr_msg[1] = (addr & 0xff);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len	= 2;
	msgs[0].buf	= addr_msg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
	msgs[1].len	= 4;
	msgs[1].buf	= status_msg;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: read error, ret=%d\n",
			__func__, ret);
		return -EIO;
	}

	status[0] = status_msg[0];
	status[0] |= status_msg[1] << 8;
	status[0] |= status_msg[2] << 16;
	status[0] |= status_msg[3] << 24;

	return 0;
}

static irqreturn_t ts64_irq_thread(int irq, void *dev_id)
{
	struct tsn64_device *tsn64 = dev_id;

	do {
		uint32_t status = 0;
		int err;
		int player;
		err = ts64_i2c_read(tsn64->client, 0x0, &status);
		if(err){
			printk(KERN_INFO "TS-N64 failed to read i2c.  %d\n", err);
			break;
		}
		player = (status >> PLAYER_OFFSET) & PLAYER_MASK;

		input_report_key(tsn64->input_dev, BTN_TL, status & (1 << CON_L));
		input_report_key(tsn64->input_dev, BTN_TR, status & (1 << CON_R));
		input_report_key(tsn64->input_dev, BTN_FORWARD, status & (1 << CON_CUP));
		input_report_key(tsn64->input_dev, BTN_BACK, status & (1 << CON_CDN));
		input_report_key(tsn64->input_dev, BTN_LEFT, status & (1 << CON_CL));
		input_report_key(tsn64->input_dev, BTN_RIGHT, status & (1 << CON_CR));
		input_report_key(tsn64->input_dev, BTN_A, status & (1 << CON_A));
		input_report_key(tsn64->input_dev, BTN_B, status & (1 << CON_B));
		input_report_key(tsn64->input_dev, BTN_Z, status & (1 << CON_Z));
		input_report_key(tsn64->input_dev, BTN_START, status & (1 << CON_START));
		input_report_key(tsn64->input_dev, BTN_TRIGGER_HAPPY3, status & (1 << CON_DUP));
		input_report_key(tsn64->input_dev, BTN_TRIGGER_HAPPY4, status & (1 << CON_DDN));
		input_report_key(tsn64->input_dev, BTN_TRIGGER_HAPPY1, status & (1 << CON_DL));
		input_report_key(tsn64->input_dev, BTN_TRIGGER_HAPPY2, status & (1 << CON_DR));

		input_report_abs(tsn64->input_dev, ABS_X, (status >> ANA_X_OFFSET) & ANA_MASK);
		input_report_abs(tsn64->input_dev, ABS_Y, status & ANA_MASK);
		input_sync(tsn64->input_dev);

		printk(KERN_INFO "TS-N64: Got back 0x%X\n", status);
	} while(gpio_get_value(tsn64->irq_gpio) == 1);

	return IRQ_HANDLED;
}

static int tsn64_probe_dt(struct i2c_client *client, struct tsn64_device *tsn64)
{
	struct device_node *np = client->dev.of_node;

	if (!np) {
		dev_err(&client->dev, "missing device tree data\n");
		return -EINVAL;
	}

	tsn64->irq_gpio = of_get_gpio(np, 0);
	if (!gpio_is_valid(tsn64->irq_gpio))
		dev_warn(&client->dev,
			 "GPIO not specified in DT (of_get_gpio returned %d)\n",
			 tsn64->irq_gpio);

	return 0;
}

static int tsn64_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tsn64_device *tsn64;
	struct input_dev *input_dev;
	int irq;
	int err;

	input_dev = input_allocate_device();
	tsn64 = kmalloc(sizeof(struct tsn64_device), GFP_KERNEL);

	printk(KERN_INFO "TS-64: Probed\n");

	if (!tsn64 || !input_dev) {
		dev_err(&client->dev,
			"Can't allocate memory for device structure\n");
		err = -ENOMEM;
		goto err_free_mem;
	}

	tsn64->client = client;
	tsn64->input_dev = input_dev;

	input_dev->name = "TS-64 Controller Driver";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	err = tsn64_probe_dt(client, tsn64);
	if (err){
		printk(KERN_INFO "TS-64: Failed to probe dt\n");
		goto err_free_mem;
	}

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -128, 128, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, -128, 128, 0, 0);

	__set_bit(EV_KEY, input_dev->evbit);

	__set_bit(BTN_TL, input_dev->keybit);
	__set_bit(BTN_TR, input_dev->keybit);
	__set_bit(BTN_FORWARD, input_dev->keybit);
	__set_bit(BTN_BACK, input_dev->keybit);
	__set_bit(BTN_LEFT, input_dev->keybit);
	__set_bit(BTN_RIGHT, input_dev->keybit);
	__set_bit(BTN_A, input_dev->keybit);
	__set_bit(BTN_B, input_dev->keybit);
	__set_bit(BTN_Z, input_dev->keybit);
	__set_bit(BTN_START, input_dev->keybit);
	__set_bit(BTN_TRIGGER_HAPPY3, input_dev->keybit);
	__set_bit(BTN_TRIGGER_HAPPY4, input_dev->keybit);
	__set_bit(BTN_TRIGGER_HAPPY1, input_dev->keybit);
	__set_bit(BTN_TRIGGER_HAPPY2, input_dev->keybit);

	tsn64->irq = gpio_to_irq(tsn64->irq_gpio);
	if (irq < 0) {
		dev_err(&client->dev,
			"Failed to get irq number for button gpio\n");
		err = irq;
		goto err_free_gpio;
	}

	err = request_threaded_irq(tsn64->irq,
				     NULL, ts64_irq_thread,
				     IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				     "tsn64_button", tsn64);

	if (err < 0) {
		dev_err(&client->dev,
			"Can't allocate button irq %d\n", tsn64->irq);
		goto err_free_gpio;
	}

	err = input_register_device(tsn64->input_dev);
	if (err) {
		dev_err(&client->dev, "Failed to register input device\n");
		goto err_free_irq;
	}

	printk(KERN_INFO "TS-64: Loaded\n");

	i2c_set_clientdata(client, tsn64);

	return 0;

err_free_irq:
	free_irq(tsn64->irq, tsn64);
err_free_gpio:
	gpio_free(tsn64->irq_gpio);
err_free_mem:
	input_free_device(input_dev);
	kfree(tsn64);

	return err;
}

static int tsn64_remove(struct i2c_client *client)
{
	struct tsn64_device *tsn64 = i2c_get_clientdata(client);

	free_irq(tsn64->irq, tsn64);
	gpio_free(tsn64->irq_gpio);

	input_unregister_device(tsn64->input_dev);
	kfree(tsn64);

	return 0;
}

static const struct of_device_id tsn64_of_match[] = {
	{ .compatible = "ts,tsn64" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tsn64_of_match);

static const struct i2c_device_id tsn64_id[] = {
	{ MODULE_DEVICE_ALIAS, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tsn64_id);

static struct i2c_driver tsn64_driver = {
	.driver = {
		.name = "tsn64",
	},
	.probe		= tsn64_probe,
	.remove		= tsn64_remove,
	.id_table	= tsn64_id,
};

module_i2c_driver(tsn64_driver);
