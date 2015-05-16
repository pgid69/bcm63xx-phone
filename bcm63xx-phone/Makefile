#
# Copyright (C) 2015
# Gilles Mazoyer <mazoyer.gilles@omega.ovh>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=bcm63xx-phone
PKG_VERSION=0.1
PKG_RELEASE:=1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define KernelPackage/bcm63xx-phone
  SUBMENU:=Voice over IP
  DEPENDS:=@TARGET_brcm63xx
  TITLE:=Kernel driver for Le88221 for Broadcom 6358
  FILES:=$(PKG_BUILD_DIR)/bcm63xx-phone.ko
endef

define KernelPackage/bcm63xx-phone/description
  Kernel driver for Le88221 for Broadcom 6358 (conflicts with module spi-bcm63xx).
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		ARCH="$(LINUX_KARCH)" \
		SUBDIRS="$(PKG_BUILD_DIR)" \
		modules V=1
endef

$(eval $(call KernelPackage,bcm63xx-phone))
