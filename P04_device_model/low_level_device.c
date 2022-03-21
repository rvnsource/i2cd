/* Assignment for device model */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "I2C_PLDRV"

//TODO 5.6: Intialize the start address and end address for I2C 0
#define RESOURCE1_START_ADDRESS 0x44e0b000
#define RESOURCE1_END_ADDRESS 0x44e0bFFF

/* Specifying my resources information */
//TODO 5.7: Populate the memory resource
static struct resource sample_resources[] = {
	{
		.start = RESOURCE1_START_ADDRESS,
		.end = RESOURCE1_END_ADDRESS,
		.flags = IORESOURCE_MEM,
	}
};

//TODO 5.8: Define and initialize the clock frequence to 400KHz
static u16 clock_freq  = 400;

//TODO 5.9: Populate the platform device structure
static struct platform_device sample_device = 
{
	.name = DRIVER_NAME,
	.id = -1,
	.num_resources = ARRAY_SIZE(sample_resources),
	.resource = sample_resources,
	.dev = {
		.platform_data = &clock_freq,
	}
};

static __init int init_platform_dev(void)
{
	printk("sample Platform driver(device).... \n");
	//TODO 5.10: Register the platform device
	platform_device_register(&sample_device);
	return 0;
}

static void __exit exit_platform_dev(void)
{
	//TODO 5.11: Un-register the platform device
	platform_device_unregister(&sample_device);
	printk("Exiting sample Platform(device) driver... \n");
}

module_init(init_platform_dev);
module_exit(exit_platform_dev);

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Sample Platform Driver");
MODULE_LICENSE("GPL");
