# bcm63xx-phone

For router running OpenWrt and based on Broadcom 6358 SoC, this project aims to add support for telephony.

Now it just supports my router, a Huawei HW553, that has two FXS lines managed by a Legerity/Zarlink/Microsemi Le88221 device.

The Broadcom SoC interfaces with Le88221 using PCM and SPI buses.

There are three subdirectories : 
- bcm63xx-phone : the kernel driver. It uses the Microsemi Voice Path API Lite SDK, so adding support for other Microsemi devices should be quite easy. I think that support for routed based on Broadcom 6368 SoC should be quite easy too but of course it needs testing.
- bcm63xx-phone-test : a basic test program for the kernel driver.
- bcm63xx-ast-chan : an Asterisk 11 channel, to connect Asterisk with the kernel driver.

## Compilation ##

Compilation is simple : just define the directory containing the three subdirectories as custom-feed in OpenWrt (http://wiki.openwrt.org/doc/devel/feeds) and compile the three packages as any other OpenWrt packages.
