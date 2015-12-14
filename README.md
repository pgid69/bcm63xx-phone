# bcm63xx-phone

For router running OpenWrt and based on Broadcom 6358 SoC, this project aims to add support for telephony.

Now it supports :

- my router, a Huawei HW553, that has two FXS lines managed by a Legerity/Zarlink/Microsemi Le88221 device. The Broadcom 6358 SoC interfaces with Le88221 using PCM and SPI buses.

- and, in theory, the Huawei HW556, that has also two FXS lines managed by a Legerity/Zarlink/Microsemi Le88266 this time. Once again the Broadcom 6358 SoC interfaces with Le88266 using PCM and SPI buses.<BR>
BUT as I don't have a Huawei HW556, I never made tests with this router (only with my HW553).

There are three subdirectories :

- bcm63xx-phone : the kernel driver. It uses the Microsemi Voice Path API Lite SDK, so adding support for other Microsemi devices should be quite easy. I think that support for routers based on Broadcom 6368 SoC should be quite easy too but of course it needs testing.<BR>
There are mainly three limitations of the kernel driver :
 * no handling of caller id (this is a limitation of Microsemi Voice Path API Lite SDK)
 * no detection of DTMF digits (and neither the Le88221 nor the Le88266 can do it in hardware). As Asterisk can do it, i just add an option in the Asterisk channel bcm63xx-ast-chan to enable or disable DTMF detection.
 * no echo cancellation. Maybe Asterisk can do it, but i never take time to try to configure Asterisk.<BR>

WARNING : the kernel driver manages SPI bus itself in a way more efficient than bcm63xx-spi kernel driver (because Le88221/Le88266 requires that CS signal toggles between each byte transferred on the SPI bus and that incurs big overhead if using bcm63xx-spi kernel driver), so it conflicts with bcm63xx-spi.
You must rmmod bcm63xx-spi if you want to insmod bcm63xx-phone.

- bcm63xx-phone-test : a basic test program for the kernel driver.

- bcm63xx-ast-chan : an Asterisk channel, to connect Asterisk with the kernel driver. It can be compiled for Asterisk version 1.8, 11 and 13.


## Compilation ##

Compilation is simple : just define the directory containing the three subdirectories as custom-feed in OpenWrt (http://wiki.openwrt.org/doc/devel/feeds) and compile the three packages as any other OpenWrt packages.

## Asterisk configuration ##

Configuration is shared between two files

The first is /etc/asterisk/bcm3xx_phone.conf.
It eventually needs to be adpated to your specific needs.
By default only line 1 is enabled.

The second is /etc/asterisk/extensions.conf.

Here's an example of lines added to file extensions.conf.
Section [sip-provider-in] is for calls coming from outside.
Here calls should be answered by phone connected to line 1 of modem.

Section [bcmph-line-1] (name of the section is configured in bcm63xx_phone.conf, parameter context) is for calls originating from phone connected to line 1 of modem.
Here calls are forwarded to sip-provider.

[sip-provider-in]
exten => s,1,Dial(Bcm63xxPhone/1,120,t)
  same => n,Hangup(16)

[bcmph-line-1]
exten => _X.,1,Dial(SIP/sip-provider/${EXTEN})
  same => n,Hangup(16)
