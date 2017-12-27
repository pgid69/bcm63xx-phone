#
# Copyright (C) 2015
# Gilles Mazoyer <mazoyer.gilles@omega.ovh>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=asterisk13-chan-bcm63xx-phone
PKG_RELEASE:=$(LINUX_VERSION)

PKG_BUILD_DIR := $(BUILD_DIR)/asterisk-13-chan-bcm63xx-phone

include $(INCLUDE_DIR)/package.mk

define Package/asterisk13-chan-bcm63xx-phone
  SECTION:=net
  CATEGORY:=Network
  SUBMENU:=Telephony
  DEPENDS:=asterisk13 +asterisk13-codec-alaw +asterisk13-codec-a-mu \
		+asterisk13-format-sln +asterisk13-res-musiconhold +kmod-bcm63xx-phone
  TITLE:=bcm63xx-phone channel
  VERSION:=$(LINUX_VERSION)-$(PKG_RELEASE)
  URL:=http://www.kernel.org
endef

define Package/asterisk13-chan-bcm63xx-phone/description
  Asterisk 13 channel using bcm63xx-phone kernel module
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)/src
	$(CP) ../src/* $(PKG_BUILD_DIR)/src/
	mkdir -p $(PKG_BUILD_DIR)/configs
	$(CP) ../configs/* $(PKG_BUILD_DIR)/configs/
	mkdir -p $(PKG_BUILD_DIR)/obj
endef

define Build/Configure
endef

TARGET_CFLAGS += $(TARGET_CFLAGS) -DBCMPH_EXPORT_DEV_FILE \
	-DBCMPH_MODULE_NAME=chan_bcm63xx_phone -DBCMPH_DEBUG \
	-DAST_VERSION=130 -DLOW_MEMORY -D_FORTIFY_SOURCE=2 \
	-I$(KERNEL_BUILD_DIR)/bcm63xx-phone/inc \
	-I$(STAGING_DIR)/usr/include/asterisk-13/include \
	-Wall -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations \
	-fPIC -pthread

define Build/Compile
	rm -f $(PKG_BUILD_DIR)/obj/* $(PKG_BUILD_DIR)/*.so
	$(TARGET_CC) $(TARGET_CFLAGS) \
		-c $(PKG_BUILD_DIR)/src/chan_bcm63xx_phone.c \
		-o $(PKG_BUILD_DIR)/obj/chan_bcm63xx_phone.o
	$(TARGET_CC) $(TARGET_LDFLAGS) -shared -fPIC -pthread \
		$(PKG_BUILD_DIR)/obj/chan_bcm63xx_phone.o \
		-o $(PKG_BUILD_DIR)/chan_bcm63xx_phone.so
endef

define Package/asterisk13-chan-bcm63xx-phone/install
	$(INSTALL_DIR) $(1)/etc/asterisk
	$(INSTALL_DATA) $(PKG_BUILD_DIR)/configs/bcm63xx_phone.conf.sample $(1)/etc/asterisk/
	$(INSTALL_DIR) $(1)/usr/lib/asterisk/modules
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/chan_bcm63xx_phone.so $(1)/usr/lib/asterisk/modules/
endef

$(eval $(call BuildPackage,asterisk13-chan-bcm63xx-phone))
