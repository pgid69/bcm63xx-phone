#
# Copyright (C) 2017
# Gilles Mazoyer <mazoyer.gilles@omega.ovh>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=bcm63xx-phone-dahdi
PKG_VERSION=0.2
PKG_RELEASE:=1
PKG_BUILD_DIR := $(KERNEL_BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define KernelPackage/bcm63xx-phone-dahdi
  SUBMENU:=Voice over IP
  TITLE:=DAHDI driver for Le88221 and Le88266
  DEPENDS:=@(TARGET_brcm63xx) kmod-dahdi @!PACKAGE_kmod-bcm63xx-phone
  FILES:=$(PKG_BUILD_DIR)/bcm63xx-phone-dahdi.$(LINUX_KMOD_SUFFIX)
endef

define KernelPackage/bcm63xx-phone-dahdi/description
  DAHDI kernel driver for Le88221 and Le88266 for Broadcom 6358
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ../src/* $(PKG_BUILD_DIR)/
	sed -e "s/module\\.o/$(PKG_NAME)\\.o/g" -e "s/module-objs/$(PKG_NAME)-objs/g" "$(PKG_BUILD_DIR)/Makefile.src" > "$(PKG_BUILD_DIR)/Makefile"
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		ARCH="$(LINUX_KARCH)" \
		SUBDIRS="$(PKG_BUILD_DIR)" BCMPH_EFLAGS="-DBCMPH_MODULE_NAME=bcm63xx_phone_dahdi -DBCMPH_MODULE_VERSION=$(PKG_VERSION) -DBCMPH_DAHDI_DRIVER -DBCMPH_DEBUG" \
		modules V=1
endef

$(eval $(call KernelPackage,bcm63xx-phone-dahdi))
