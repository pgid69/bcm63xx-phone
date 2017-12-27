#!/bin/sh

#
# Copyright (C) 2015
# Gilles Mazoyer <mazoyer.gilles@omega.ovh>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

# Makefile used to compile the driver in test mode that can be insmod
# on a PC.

set -x

if [ "$1" = "clean" ]; then
   find . -name "*.ko" -type f -exec rm {} \;
   find . -name "*.o" -type f -exec rm {} \;
   find . -name ".*.ko.cmd" -type f -exec rm {} \;
   find . -name ".*.o.cmd" -type f -exec rm {} \;
   find . -name "*.mod.c" -type f -exec rm {} \;
   rm modules.order Module.symvers Makefile
   rm -fr .tmp_versions
   exit $?
elif [ "$1" = "drv" ]; then
   PKG_NAME=bcm63xx-phone
   BCMPH_EFLAGS="-DBCMPH_NOHW -DBCMPH_MODULE_NAME=bcm63xx_phone -DBCMPH_MODULE_VERSION=0.2 -DBCMPH_EXPORT_DEV_FILE -DBCMPH_DEBUG"
elif [ "$1" = "dahdi" ]; then
   PKG_NAME=bcm63xx-phone-dahdi
   BCMPH_EFLAGS="-DBCMPH_NOHW -DBCMPH_MODULE_NAME=bcm63xx_phone_dahdi -DBCMPH_MODULE_VERSION=0.2 -DBCMPH_DAHDI_DRIVER -DBCMPH_DEBUG -I/usr/include"
else
   echo "Parameter #1 must be clean, dahdi or driver"
   exit 1
fi

set -e
set -u

sed -e "s/module\\.o/${PKG_NAME}\\.o/g" -e "s/module-objs/${PKG_NAME}-objs/g" "Makefile.src" > "Makefile"
make -C "/lib/modules/$(uname -r)/build/" "V=1" "SUBDIRS=$(pwd)" "BCMPH_EFLAGS=${BCMPH_EFLAGS}" "CONFIG_DEBUG_SECTION_MISMATCH=y" modules

