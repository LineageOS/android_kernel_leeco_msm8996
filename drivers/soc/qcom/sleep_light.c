#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define SLEEP_LIGHT_OFF 0x0
#define SLEEP_LIGHT_ON 0x1
static void __iomem *sleep_light_addr;

int sleep_light_control(bool enable)
{
	if (!sleep_light_addr)
		return -ENODEV;

	if (enable){
		__raw_writel(SLEEP_LIGHT_ON, sleep_light_addr);
		pr_err("%s, sleep light: %s", __func__, enable? "on": "off");
	}else {
		__raw_writel(SLEEP_LIGHT_OFF, sleep_light_addr);
		pr_err("%s, sleep light: %s", __func__, enable? "on": "off");
	}
	return 0;

}

static int __init sleep_light_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-sleep_light");
	if (!np){
		pr_err("%s, unable to find the sleep light node\n", __func__);
		return -ENODEV;
	}

	sleep_light_addr = of_iomap(np, 0);
	if (!sleep_light_addr) {
		pr_err("%s, unable to map imem dump table offset\n", __func__);
		return -ENOMEM;
	}

	__raw_writel(SLEEP_LIGHT_OFF, sleep_light_addr);

	pr_err("%s: sleep light init done\n", __func__);
	return 0;
}
early_initcall(sleep_light_init);
