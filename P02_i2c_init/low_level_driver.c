#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/platform_data/i2c-omap.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include "i2c_char.h"

static struct omap_i2c_dev i2c_dev;

inline void omap_i2c_write_reg(struct omap_i2c_dev *i2c_dev,
                                      int reg, u16 val)
{       
        __raw_writew(val, i2c_dev->base + i2c_dev->regs[reg]);
}

inline u16 omap_i2c_read_reg(struct omap_i2c_dev *i2c_dev, int reg)
{
        return __raw_readw(i2c_dev->base + i2c_dev->regs[reg]);
}

inline void omap_i2c_ack_stat(struct omap_i2c_dev *dev, u16 stat)
{
        omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, stat);
}

/*
 * Waiting on Bus Busy
 */
int omap_i2c_wait_for_bb(struct omap_i2c_dev *dev)
{
        unsigned long timeout;

        timeout = jiffies + OMAP_I2C_TIMEOUT;
        while (omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG) & OMAP_I2C_STAT_BB) {
                if (time_after(jiffies, timeout)) {
                        printk("timeout waiting for bus ready\n");
                        return -ETIMEDOUT;
                }
                msleep(1);
        }

        return 0;
}

void flush_fifo(struct omap_i2c_dev *dev)
{
        unsigned long timeout;
        u32 status;
       timeout = jiffies + OMAP_I2C_TIMEOUT;
        while ((status = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG)) & OMAP_I2C_STAT_RRDY) {
                omap_i2c_read_reg(dev, OMAP_I2C_DATA_REG);
                omap_i2c_ack_stat(dev, OMAP_I2C_STAT_RRDY);
                if (time_after(jiffies, timeout)) {
                        printk(KERN_ALERT "timeout waiting for bus ready\n");
                        break;
                }
                msleep(1);
        }
}

u16 wait_for_event(struct omap_i2c_dev *dev)
{
        unsigned long timeout = jiffies + OMAP_I2C_TIMEOUT;
        u16 status;

        while (!((status = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG)) &
                                (OMAP_I2C_STAT_ROVR | OMAP_I2C_STAT_XUDF |
                                 OMAP_I2C_STAT_XRDY | OMAP_I2C_STAT_RRDY |
                                 OMAP_I2C_STAT_ARDY | OMAP_I2C_STAT_NACK |
                                 OMAP_I2C_STAT_AL))) {
                if (time_after(jiffies, timeout)) {
                        printk("time-out waiting for event\n");
                        omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, 0XFFFF);
                        return 0;
                }
                mdelay(1);
        }
        return status;
}

int i2c_transmit(struct i2c_msg *msg, size_t count)
{
	//TODO: 3.1 Update the cnt to 3
	u16 status, cnt = 2, w = 0;
	int i2c_error = 0;
	/* Initialize the loop variable */
	int k = 7;
	int i = 0;

	//TODO: 3.2 Declare an array of 3
    // Index 0 - Eeprom address upper 8 bits
    // Index 1 - Eeprom address lower 8 bits
    // Index 2 - data
	u8 a[] = {0x00,0x60,0x99};

	ENTER();

	//TODO 2.6 Set the TX FIFO Threshold to 0 and clear the FIFO's
	omap_i2c_write_reg(&i2c_dev,OMAP_I2C_BUF_REG,0);
	

	//TODO 2.7: Update the slave addresss register with 0x50
	omap_i2c_write_reg(&i2c_dev, OMAP_I2C_SA_REG, 0x50);

	//TODO 2.8: Update the count register with cnt 
	omap_i2c_write_reg(&i2c_dev,OMAP_I2C_CNT_REG, cnt);

	printk("##### Sending %d byte(s) on the I2C bus ####\n", cnt);

	/*
	 * TODO 2.9: Update the configuration register (OMAP_I2C_CON_REG) to start the
	 * transaction with master mode and direction as transmit. Also, enable the
	 * I2c module and set the start and stop bits.
	 * The naming convention for the bits is OMAP_I2C_<REG NAME>_<BIT NAME>
	 * So, for start bit, the macro is OMAP_I2C_CON_STT. Check I2c_char.h for other bits
	 */
	w = OMAP_I2C_CON_STT | OMAP_I2C_CON_MST | OMAP_I2C_CON_STP | OMAP_I2C_CON_TRX | OMAP_I2C_CON_EN;
	omap_i2c_write_reg(&i2c_dev, OMAP_I2C_CON_REG, w); /* Control Register */

	while (k--) {
		// Wait for status to be updated
		status = wait_for_event(&i2c_dev);
		if (status == 0) {
			i2c_error = -ETIMEDOUT;
			goto wr_exit;
		}
		//TODO 2.10: Check the status to verify if XRDY is received
		if (status & OMAP_I2C_STAT_XRDY) {
			printk("Got XRDY\n");
			//TODO 2.11: Update the data register with data to be transmitted
			//omap_i2c_write_reg(&i2c_dev, OMAP_I2C_DATA_REG, a);
			
			//TODO 3.3: Write an array into the data register
			omap_i2c_write_reg(&i2c_dev, OMAP_I2C_DATA_REG, a[i++]);

			//TODO 2.12: Clear the XRDY event with omap_i2c_ack_stat
			omap_i2c_ack_stat(&i2c_dev, OMAP_I2C_STAT_XRDY);
			continue;
		}
		//TODO 2.13: Check the status to verify if ARDY is received
		if (status & OMAP_I2C_STAT_ARDY) {
			printk("Got ARDY\n");
			//TODO 2.14: Clear the XRDY event with omap_i2c_ack_stat and break out of loop
			omap_i2c_ack_stat(&i2c_dev, OMAP_I2C_STAT_XRDY);
			break;
		}
	}
	if (k <= 0) {
		printk("TX Timed out\n");
		i2c_error = -ETIMEDOUT;
	}
wr_exit:
	flush_fifo(&i2c_dev);
	omap_i2c_write_reg(&i2c_dev, OMAP_I2C_STAT_REG, 0XFFFF);
	return i2c_error;

}

int i2c_receive(struct i2c_msg *msg, size_t count)
{
	u16 status, cnt = 3, w = 0;
	int i2c_error = 0;
	/* Initialize the loop variable */
	int k = 7;
	int i = 0;
	u16 a;

	ENTER();

	//TODO 4.1: Set the RX FIFO Threshold to 0 and clear the FIFO's
	omap_i2c_write_reg(&i2c_dev,OMAP_I2C_BUF_REG,0);
		
	//TODO 4.2: Update the slave addresss register with 0x50
	omap_i2c_write_reg(&i2c_dev, OMAP_I2C_SA_REG, 0x50);

	//TODO 4.3: Update the count register with 3
	printk("##### Receiving %d byte(s) on the I2C bus ####\n", cnt);
	omap_i2c_write_reg(&i2c_dev,OMAP_I2C_CNT_REG, cnt);

	/*
	 * TODO 4.4: Update the configuration register (OMAP_I2C_CON_REG) to start the
	 * transaction with master mode and direction as receive. Also, enable the
	 * I2c module and set the start and stop bits.
	 * The naming convention for the bits is OMAP_I2C_<REG NAME>_<BIT NAME>
	 * So, for start bit, the macro is OMAP_I2C_CON_STT. Check I2c_char.h for other bits
	 */
	w = OMAP_I2C_CON_STT | OMAP_I2C_CON_MST | OMAP_I2C_CON_STP | OMAP_I2C_CON_EN;
	omap_i2c_write_reg(&i2c_dev, OMAP_I2C_CON_REG, w); /* Control Register */

	while (k--) {
		// Wait for status to be updated
		status = wait_for_event(&i2c_dev);
		if (status == 0) {
			i2c_error = -ETIMEDOUT;
			goto wr_exit;
		}
		//TODO 4.5: Check the status to verify if RRDY is received
		if (status & OMAP_I2C_STAT_RRDY) {
			printk("Got RRDY\n");
			//TODO 4.6: Read Data register
			a = omap_i2c_read_reg(&i2c_dev, OMAP_I2C_DATA_REG);
                    	printk("Received %x\n", a);

			//TODO 4.7: Clear the RRDY event with omap_i2c_ack_stat
			omap_i2c_ack_stat(&i2c_dev, OMAP_I2C_STAT_RRDY);
			continue;
		}
		//TODO 4.8: Check the status to verify if ARDY is received
		if (status & OMAP_I2C_STAT_ARDY) {
			printk("Got ARDY\n");
			//TODO 4.9: Clear the XRDY event with omap_i2c_ack_stat and break out of loop
			omap_i2c_ack_stat(&i2c_dev, OMAP_I2C_STAT_ARDY);
			break;
		}
	}
	if (k <= 0) {
		printk("RX Timed out\n");
		i2c_error = -ETIMEDOUT;
	}
wr_exit:
	flush_fifo(&i2c_dev);
	omap_i2c_write_reg(&i2c_dev, OMAP_I2C_STAT_REG, 0XFFFF);
	return i2c_error;
	
	ENTER();
	return 0;
}

static void omap_i2c_set_speed(struct omap_i2c_dev *dev)
{
        u16 psc = 0;
        unsigned long fclk_rate = 48000;
        unsigned long internal_clk = 24000; // Recommended as per TRM
        unsigned long scl, scll, sclh;

        /* Compute prescaler divisor */
        psc = fclk_rate / internal_clk;

        //TODO 2.4: Update the prescalar register with psc - 1
	omap_i2c_write_reg(dev, OMAP_I2C_PSC_REG,1);

        // Hard coding the speed to 400KHz
        dev->speed = 400;
        scl = internal_clk / dev->speed;
        // 50% duty cycle
        scl /= 2;
        scll = scl - 7;
        sclh = scl - 5;

        //TODO 2.5: Update the SCL low and high registers as per above calculations
	omap_i2c_write_reg(dev, OMAP_I2C_SCLL_REG, scll);
	omap_i2c_write_reg(dev, OMAP_I2C_SCLH_REG, sclh);
}

int omap_i2c_init(struct omap_i2c_dev *dev)
{
        omap_i2c_set_speed(dev);
        omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);

        /* Take the I2C module out of reset: */
        omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);

        // TODO 2.2: Update the 'iestate' field with desired events such as XRDY and ARDY
	i2c_dev.iestate = OMAP_I2C_IE_XRDY | OMAP_I2C_IE_ARDY;

		// TODO 4.10: Update the 'iestate' to enable RRDY event
	i2c_dev.iestate = OMAP_I2C_IE_XRDY | OMAP_I2C_IE_ARDY | OMAP_I2C_IE_RRDY;

        // TODO 2.3: Update the OMAP_I2C_IE_REG
 	omap_i2c_write_reg(dev, OMAP_I2C_IE_REG, i2c_dev.iestate);

        flush_fifo(dev);
        omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, 0XFFFF);
        omap_i2c_wait_for_bb(dev);

        return 0;
}

static int __init i2c_init_driver(void)
{
	/*
         * TODO 2.1: Get the virtual address for the i2c0 base address and store it
         * in 'base' field of omap_i2c_dev.
         * Use API void __iomem* ioremap((resource_size_t offset, unsigned long size)
        */

	i2c_dev.base = ioremap(0x44e0b000, 0x1000);
	
        if (IS_ERR(i2c_dev.base)) {
                printk(KERN_ERR "Unable to ioremap\n");
                return PTR_ERR(i2c_dev.base);
        }

        i2c_dev.regs = (u8 *)reg_map_ip_v2;
        omap_i2c_init(&i2c_dev);

	// TODO 1.1 : Initialize the character driver interface
        chrdrv_init(&i2c_dev);


	return 0;
}

static void __exit i2c_exit_driver(void)
{
	// TODO 1.2: De-initialize the character driver interface
        chrdrv_exit(&i2c_dev);

}

module_init(i2c_init_driver);
module_exit(i2c_exit_driver);

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Low level I2C driver");
MODULE_LICENSE("GPL");
