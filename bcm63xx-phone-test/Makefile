#
# Copyright (C) 2015
# Gilles Mazoyer <mazoyer.gilles@omega.ovh>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=bcm63xx-phone-test
PKG_RELEASE:=$(LINUX_VERSION)

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/bcm63xx-phone-test
  SECTION:=utils
  CATEGORY:=Utilities
  DEPENDS:=+kmod-bcm63xx-phone
  TITLE:=Testing utility for driver bcm63xx-phone
  VERSION:=$(LINUX_VERSION)-$(PKG_RELEASE)
  URL:=http://www.kernel.org
endef

define Package/bcm63xx-phone-test/description
  Testing utility for driver bcm63xx-phone.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Configure
endef

define Build/Compile
	cd $(PKG_BUILD_DIR) && rm -f *.o *.a
	cd $(PKG_BUILD_DIR) && $(TARGET_CC) -I$(BUILD_DIR)/bcm63xx-phone/inc \
		$(TARGET_CFLAGS) -o $(PKG_BUILD_DIR)/bcm63xx-phone-test main.c
endef

define Package/bcm63xx-phone-test/install
	$(INSTALL_DIR) $(1)/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/bcm63xx-phone-test $(1)/bin/
endef

$(eval $(call BuildPackage,bcm63xx-phone-test))
