#!/bin/sh

#
# Copyright (C) 2015
# Gilles Mazoyer <mazoyer.gilles@gilande.com>
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
   rm modules.order Module.symvers
   rm -fr .tmp_versions
else
   set -e
   set -u

   make -C "/lib/modules/$(uname -r)/build/" "V=1" "SUBDIRS=$(pwd)" "BCMPH_EFLAGS=-DBCMPH_NOHW" "CONFIG_DEBUG_SECTION_MISMATCH=y" modules
fi

