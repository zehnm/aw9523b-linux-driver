# AW9523B Linux Driver

A little while ago I came across this cheap multipurpose i2c IO expander and LED driver.

- 16 multi-function I/O, each for LED drive (current-source dimming) or GPIO mode
- 256 steps linear dimming in LED drive mode
- Interrupt support
- Imax of 37mA

More information: <https://www.awinic.com/en/index/pageview/catid/19/id/15.html>  
The english datasheet is easily found with Google.

The only downside for makers is its tiny QFN 4mm X 4mm-24L package. This is solvable with a QFN24 to DIP24 adapter board ([e.g. from AliExpress](https://www.aliexpress.com/item/1005001847723862.html)), solder paste and a heat gun.  
Or simply get the new [Adafruit AW9523 GPIO Expander and LED Driver Breakout](https://www.adafruit.com/product/4886)!

Since I wanted to learn writing Linux device drivers and understand kernel frameworks, this looked like a better alternative than the usual hello world exercises.

## Driver Features

- Developed for Linux kernel 4.19
- Regmap API for i2c communication and register access
- Interrupt controller
  - Tested with [gpio_keys](https://www.kernel.org/doc/Documentation/devicetree/bindings/input/gpio-keys.txt) driver
- [LED class](https://www.kernel.org/doc/html/latest/leds/leds-class.html) support.  
  Configurable features in DTS:
  - Imax: full, 3/4, 1/2, 1/4
  - Default and max brightness
    - Max brightness can be limited in case 1/4 Imax is still too high for a LED
  - Individual LEDs or group of LEDs
    - Multiple leds on different output lines can be grouped together in the same LED class
  - Brightness adjustment factor for individual LEDs in a group
    - Useful to achieve an uniform lightning, e.g. for backlights etc.

## TODO

- Document DTS configuration
- Mastering involved Linux driver frameworks.
  - I'm sure there are some quirks in the code! It's my first driver ;-)
  - See all the TODOs in the code.
- Linux kernel 5.4 and newer
- Power management support
- Hardware reset line support

Possible enhancements:

- Add pinctrl support.
- Rewrite as Multi Function Device (MFD) driver.  
  This driver provides GPIO and LED handling.
- Dimming in and out when adjusting brightness
- Custom LED trigger functions, e.g. breathing

## Usage

### Device Tree

GPIO expander example with gpio-keys:

```dts
&i2c0 {
	aw9523b: gpio@58 {
		compatible = "awinic,aw9523b";
		reg = <0x58>;

		status = "okay";

		interrupt-parent = <&gpio>;
		interrupts = <24 IRQ_TYPE_LEVEL_LOW>;
		gpio-controller;
		#gpio-cells = <2>;
		interrupt-controller;
		#interrupt-cells = <2>;

		pinctrl-names = "default";
		pinctrl-0 = <&keys_int>;

		gpio-line-names = "P0_0", "P0_1", "P0_2", "P0_3", "P0_4", "P0_5", "P0_6", "P0_7", 
			"P1_0", "P1_1", "P1_2", "P1_3", "P1_4", "P1_5", "P1_6", "P1_7";
	};
};

/ {
	gpio_keys {
		compatible = "gpio-keys";
		autorepeat;

		select {
			label = "Expander key SELECT";
			linux,code = <28>; /* KEY_ENTER */
			gpios = <&aw9523b 0 GPIO_ACTIVE_LOW>;
		};
		up {
			label = "Expander key UP";
			linux,code = <103>; /* KEY_UP */
			gpios = <&aw9523b 1 GPIO_ACTIVE_LOW>;
		};
		down {
			label = "GPIO key DOWN";
			linux,code = <108>; /* KEY_DOWN */
			gpios = <&aw9523b 2 GPIO_ACTIVE_LOW>;
		};
		right {
			label = "Expander key RIGHT";
			linux,code = <106>; /* KEY_RIGHT */
			gpios = <&aw9523b 3 GPIO_ACTIVE_LOW>;
		};
		left {
			label = "Expander key LEFT";
			linux,code = <105>; /* KEY_LEFT */
			gpios = <&aw9523b 4 GPIO_ACTIVE_LOW>;
		};
	};
};
```

Pure LED class example:

```dts
&i2c0 {
	key_backlight: gpio@5b {
		compatible = "awinic,aw9523b";
		reg = <0x5b>;

		status = "okay";

		aw9523,p0-output-push-pull;
		aw9523,led {
			aw9523,default_imax = <3>; // 0: Imax, 1: 3/4 Imax, 2: 1/2 Imax, 3: 1/4 Imax
			backlight {
				label = ":backlight"; // led sys class name. Name according to: https://www.kernel.org/doc/html/latest/leds/leds-class.html
				aw9523,leds = <0 1 8 9 10 11>; // The led index(es). 0..15
				aw9523,default_brightness = <20>; // 0..max_brightness
				aw9523,max_brightness = <127>; // 1..255. 127: ~ half of configured default_imax
				aw9523,leds_bright_adjust = <70 70 70 100 100 100>; // optional brightness adjustment in %
			};
			disk {
				label = "red:disk";
				linux,default-trigger = "mmc0"; // optional trigger
				aw9523,leds = <2>;
				aw9523,max_brightness = <80>;
			};
			wlan {
				label = "phy1:green:wlan";
				aw9523,leds = <3>;
			};
		};
	};
};
```

The GPIO expander functionality can also be combined with the LED class.

### Build

_TODO_

### Load driver

    modprobe gpio_aw9523

### Inspect logs and interrupts

    tail -F /var/log/messages

    watch -n 1 cat /proc/interrupts

### regmap debugfs

Dump all registers (except precious ones):

    mount -t debugfs none /sys/kernel/debug
    cat /sys/kernel/debug/regmap/0-0058-aw9523b_regmap/registers

### GPIO access

    # find GPIO number
    gpiodetect
    gpioinfo
    lsgpio

    # example
    export GPIO=488

    echo $GPIO > /sys/class/gpio/export
    echo "out" > /sys/class/gpio/gpio$GPIO/direction
    cat /sys/class/gpio/gpio$GPIO/value
    echo 1 > /sys/class/gpio/gpio$GPIO/value
