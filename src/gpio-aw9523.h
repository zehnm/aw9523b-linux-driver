#ifndef _AW9523_H
#define _AW9523_H

#define AWINIC_DEBUG 0
#if (defined(AWINIC_DEBUG) && (AWINIC_DEBUG != 0))
#define AW_DEBUG(dev, args...)                                                 \
	do {                                                                   \
		dev_dbg(dev, ##args);                                          \
	} while (0)
#else
#define AW_DEBUG(dev, ...)                                                     \
	do {                                                                   \
	} while (0)
#endif

#define AW9523B_ID	 0x23
#define AW9523B_MAX_LEDS 16

/* Registers */
#define REG_GPIO_IN_P0	   0x00 /* [00H]:[7:0] -> P0_7:P0_0 ; 0: Low, 1:High */
#define REG_GPIO_IN_P1	   0x01 /* [01H]:[7:0] -> P1_7:P1_0 ; 0: Low, 1:High */
#define REG_GPIO_OUT_P0	   0x02 /* [02H]:[7:0] -> P0_7:P0_0 ; 0: Low, 1:High */
#define REG_GPIO_OUT_P1	   0x03 /* [03H]:[7:0] -> P1_7:P1_0 ; 0: Low, 1:High */
#define REG_GPIO_CFG_P0	   0x04 /* [04H]:[7:0] -> P0_7:P0_0 ; 0: Out, 1:In */
#define REG_GPIO_CFG_P1	   0x05 /* [05H]:[7:0] -> P1_7:P1_0 ; 0: Out, 1:In */
#define REG_INT_P0	   0x06 /* Interrupt enable. 0: enabled, 1: disabled */
#define REG_INT_P1	   0x07 /* Default: 0x00 */
#define REG_ID		   0x10 /* ID Reg; ID of AW9523B should be 23H */
#define REG_CTL		   0x11 /* D[4] -> P0 mode 0: Open-Drain; 1: Push-Pull */
#define REG_LED_MODE_P0	   0x12 /* LED mode switch. 0: LED, 1: GPIO */
#define REG_LED_MODE_P1	   0x13 /* Default: 0xFF */
#define REG_LED_DIM0_P1_0  0x20 /* 256 setep dimming. Default: 0x00 */
#define REG_LED_DIM1_P1_1  0x21
#define REG_LED_DIM2_P1_2  0x22
#define REG_LED_DIM3_P1_3  0x23
#define REG_LED_DIM4_P0_0  0x24
#define REG_LED_DIM5_P0_1  0x25
#define REG_LED_DIM6_P0_2  0x26
#define REG_LED_DIM7_P0_3  0x27
#define REG_LED_DIM8_P0_4  0x28
#define REG_LED_DIM9_P0_5  0x29
#define REG_LED_DIM10_P0_6 0x2A
#define REG_LED_DIM11_P0_7 0x2B
#define REG_LED_DIM12_P1_4 0x2C
#define REG_LED_DIM13_P1_5 0x2D
#define REG_LED_DIM14_P1_6 0x2E
#define REG_LED_DIM15_P1_7 0x2F
#define REG_RESET	   0x7F /* 00 : Software Reset */

static const struct reg_default aw9523b_defaults[] = {
	{ .reg = REG_GPIO_CFG_P0, .def = 0x00 },
	{ .reg = REG_GPIO_CFG_P1, .def = 0x00 },
	{ .reg = REG_CTL, .def = 0x00 },
	{ .reg = REG_LED_MODE_P0, .def = 0xFF },
	{ .reg = REG_LED_MODE_P1, .def = 0xFF },
	{ .reg = REG_LED_DIM0_P1_0, .def = 0x00 },
	{ .reg = REG_LED_DIM1_P1_1, .def = 0x00 },
	{ .reg = REG_LED_DIM2_P1_2, .def = 0x00 },
	{ .reg = REG_LED_DIM3_P1_3, .def = 0x00 },
	{ .reg = REG_LED_DIM4_P0_0, .def = 0x00 },
	{ .reg = REG_LED_DIM5_P0_1, .def = 0x00 },
	{ .reg = REG_LED_DIM6_P0_2, .def = 0x00 },
	{ .reg = REG_LED_DIM7_P0_3, .def = 0x00 },
	{ .reg = REG_LED_DIM8_P0_4, .def = 0x00 },
	{ .reg = REG_LED_DIM9_P0_5, .def = 0x00 },
	{ .reg = REG_LED_DIM10_P0_6, .def = 0x00 },
	{ .reg = REG_LED_DIM11_P0_7, .def = 0x00 },
	{ .reg = REG_LED_DIM12_P1_4, .def = 0x00 },
	{ .reg = REG_LED_DIM13_P1_5, .def = 0x00 },
	{ .reg = REG_LED_DIM14_P1_6, .def = 0x00 },
	{ .reg = REG_LED_DIM15_P1_7, .def = 0x00 },
};

static const struct regmap_range aw9523b_volatile_ranges[] = {
	regmap_reg_range(REG_GPIO_IN_P0, REG_INT_P1),
	regmap_reg_range(REG_ID, REG_ID),
	regmap_reg_range(REG_RESET, REG_RESET),
};

static const struct regmap_access_table aw9523b_volatile_table = {
	.yes_ranges   = aw9523b_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(aw9523b_volatile_ranges),
};

// FIXME check precious regs
static const struct regmap_range aw9523b_precious_range = {
	/* When GPIO state is read, the interrupt is cleared */
	.range_min = REG_GPIO_IN_P0,
	.range_max = REG_GPIO_IN_P1,
};

static const struct regmap_access_table aw9523b_precious_table = {
	.yes_ranges   = &aw9523b_precious_range,
	.n_yes_ranges = 1,
};

static const struct regmap_range aw9523b_readable_ranges[] = {
	regmap_reg_range(REG_GPIO_IN_P0, REG_INT_P1),
	regmap_reg_range(REG_ID, REG_LED_MODE_P1),
};

static const struct regmap_access_table aw9523b_readable_table = {
	.yes_ranges   = aw9523b_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(aw9523b_readable_ranges),
};

static const struct regmap_range aw9523b_writeable_ranges[] = {
	regmap_reg_range(REG_GPIO_OUT_P0, REG_INT_P1),
	regmap_reg_range(REG_CTL, REG_LED_MODE_P1),
	regmap_reg_range(REG_LED_DIM0_P1_0, REG_LED_DIM15_P1_7),
	regmap_reg_range(REG_RESET, REG_RESET),
};

static const struct regmap_access_table aw9523b_writeable_table = {
	.yes_ranges   = aw9523b_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(aw9523b_writeable_ranges),
};

static const struct regmap_config aw9523b_regmap = {
	.name	  = "aw9523b_regmap",
	.reg_bits = 8,
	.val_bits = 8,

	.reg_stride   = 1,
	.max_register = REG_RESET,

	.volatile_table = &aw9523b_volatile_table,
	.precious_table = &aw9523b_precious_table,
	.rd_table	= &aw9523b_readable_table,
	.wr_table	= &aw9523b_writeable_table,

	.reg_defaults	   = aw9523b_defaults,
	.num_reg_defaults  = ARRAY_SIZE(aw9523b_defaults),
	.cache_type	   = REGCACHE_FLAT,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

/* LED current limitation */
typedef enum {
	NO_LIMIT      = 0, /* 37mA */
	THREE_QUARTER = 1, /* 37mA * 3/4 */
	HALF	      = 2, /* 37mA * 2/4 */
	ONE_QUARTER   = 3, /* 37mA * 1/4 */
	E_VAL_MIN     = 0, /* helper value to validate enum value */
	E_VAL_MAX     = 3, /* helper value to validate enum value */
} aw9523_imax_t;

struct aw9523b {
	struct gpio_chip chip;
	struct regmap *	 regmap;
	struct device *	 dev;

	struct irq_chip irqchip;
	struct mutex	lock;
	unsigned	status; /* current status */
	unsigned int	irq_parent;
	unsigned	irq_enabled; /* enabled irqs */
	struct gpio_desc *reset_gpio;

	struct aw9523b_led *led_data; /* optional LED feature */
};

struct aw9523b_led_dev {
	struct led_classdev cdev;
	unsigned int	    led_mask;
	/* current control is only a write register */
	u32		cur_brightness;
	u32		max_brightness;
	struct aw9523b *parent;
};

struct aw9523b_led {
	unsigned int		led_mask; /* main led mask of all led devices */
	aw9523_imax_t		imax;
	u8			count; /* number of led devices */
	struct aw9523b_led_dev *led_devs; /* array of led devices */
	u8 brightness_adjust[AW9523B_MAX_LEDS]; /* adjustment in percent */
};

#endif
