# bcm63xx-phone

For router running LEDE/OpenWrt and based on Broadcom 6358 SoC, this project aims to add support for telephony.

Now it supports :

- my router, a Huawei HW553, that has two FXS lines managed by a Legerity/Zarlink/Microsemi Le88221 device. The Broadcom 6358 SoC interfaces with Le88221 using PCM and SPI buses.

- in theory, the Huawei HW556, that has also two FXS lines managed by a Legerity/Zarlink/Microsemi Le88266 this time. Once again the Broadcom 6358 SoC interfaces with Le88266 using PCM and SPI buses.<BR>
BUT as I don't have a Huawei HW556, I never made tests with this router (only with my HW553).

- the Pirelli FastWeb DRG A226M, that has also two FXS lines managed by a Legerity/Zarlink/Microsemi Le88266. Once again the Broadcom 6358 SoC interfaces with Le88266 using PCM and SPI buses.<BR>
I don't have this router, but it's reported to work (see https://github.com/pgid69/bcm63xx-phone/issues/5)

There are three subdirectories :

- bcm63xx-phone : it contains two kernel drivers in two subdirectories.
  * bcm63xx-phone-dahdi : this driver must be used with DAHDI kernel driver and Asterisk channel chan_dahdi.so. Being a DAHDI driver it benefits all the features that DAHDI provides for analog lines (like choice of multiple echo canceller, location specific tones and ring cadencing, ...) but also its limitations (the only one i found is that Caller ID support is not configurable and can only be Bell FSK signal sent after first ring).
  * bcm63xx-phone : this is the first kernel driver i developped. It must be used in conjunction with its dedicated Asterisk channel (see below bcm63xx-ast-chan). It uses OSLEC echo canceller. Caller ID handling, tone generation, ring cadencing and DTMF detection are delegated to the Asterisk channel.

Both kernel drivers use the Microsemi Voice Path API Lite SDK, so adding support for other Microsemi devices should be quite easy. I think that support for routers based on Broadcom 6368 SoC should be quite easy too but of course it needs testing.<BR>

WARNING : by default the two kernel drivers manage SPI bus themselves in a way more efficient than bcm63xx-spi/spi-bcm63xx kernel driver (because Le88221/Le88266 requires that CS signal toggles between each byte transferred on the SPI bus and that incurs big overhead if using bcm63xx-spi/spi-bcm63xx kernel driver), so it conflicts with bcm63xx-spi/spi-bcm63xx.<BR>
    - With OpenWrt 12.09 (AA) you must rmmod bcm63xx-spi if you want to insmod bcm63xx-phone or bcm63xx-phone-dahdi.<BR>
    - With LEDE/OpenWrt above 12.09, you must change the default config of Linux kernel and recompile it. So you must apply the patch located in one of the subdirectories of patches, at the root of LEDE/OpenWrt source tree, remove the build directory of the Linux kernel (eg for OpenWrt 15.05 this is build_dir/target-mips_mips32_uClibc-0.9.33.2/linux-brcm63xx_generic/linux-3.18.23/) if it exists, and recompile LEDE/OpenWrt.<BR>
    - With LEDE 17.01, instead of changing the kernel config you can also patch spi-bcm63xx and edit the file inc/config.h to define symbol BCMPH_USE_SPI_DRIVER. I don't test very much this option and unless there are other SPI devices, there's no advantage as it requires to recompile the kernel too. Open an issue if you're interested.<BR>

- bcm63xx-phone-test : a basic test program for the kernel driver bcm63xx-phone.

- bcm63xx-ast-chan : an Asterisk channel, to connect Asterisk with the kernel driver bcm63xx-phone. It can be compiled for Asterisk version 1.8, 11 and 13.


## Compilation ##

Compilation is simple : just define the directory containing the three subdirectories as custom-feed in LEDE/OpenWrt (http://wiki.openwrt.org/doc/devel/feeds) and compile the packages as any other LEDE/OpenWrt packages.

## Asterisk configuration for bcm63xx-ast-chan channel ##

Configuration is shared between two files

The first is /etc/asterisk/bcm3xx_phone.conf.
It eventually needs to be adapted to your specific needs.
By default only line 1 is enabled.

The second is /etc/asterisk/extensions.conf.

Here's an example of lines added to file extensions.conf.
Section [sip-provider-in] is for calls coming from outside.
Here calls should be answered by phone connected to line 1 of modem.

Section [bcmph-line-1] (name of the section is configured in bcm63xx_phone.conf, parameter context) is for calls originating from phone connected to line 1 of modem.
Here calls are forwarded to sip-provider.
```
[sip-provider-in]
exten => s,1,Dial(Bcm63xxPhone/1,120,t)
  same => n,Hangup(16)

[bcmph-line-1]
exten => _X.,1,Dial(SIP/sip-provider/${EXTEN})
  same => n,Hangup(16)
```
