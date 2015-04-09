#
# Copyright (C) 2015
# Gilles Mazoyer <mazoyer.gilles@gilande.com>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
ccflags-y += -Wall -DBCMPH_DEBUG -I$(SUBDIRS)/inc
#ccflags-y += -DBCMPH_TEST_PCM
ifneq ($(strip $(BCMPH_EFLAGS)),)
ccflags-y += $(BCMPH_EFLAGS)
endif

ZARLINK_VERSION=2.24.0.lite

ccflags-y += -I$(SUBDIRS)/extern/zarlink/arch/bcm63xx
ccflags-y += -I$(SUBDIRS)/extern/zarlink/$(ZARLINK_VERSION)/api_lib/includes
ccflags-y += -I$(SUBDIRS)/extern/zarlink/$(ZARLINK_VERSION)/api_lib/vp880_api


BCMPH_SRCS += board/board.c
BCMPH_SRCS += inc/bcm63xx_line_state.c
BCMPH_SRCS += inc/bcm63xx_ring_buf.c
BCMPH_SRCS += mpi/mpi.c
BCMPH_SRCS += pcm/pcm.c
BCMPH_SRCS += phone/phone.c
BCMPH_SRCS += phone/phone_mgr.c
BCMPH_SRCS += phone/zarlink/le88221/slic_le88221.c
BCMPH_SRCS += phone/zarlink/zarlink_common.c
BCMPH_SRCS += timer/timer.c
BCMPH_SRCS += wait_queue/wait_queue.c
BCMPH_SRCS += $(subst $(SUBDIRS)/,,$(wildcard $(SUBDIRS)/phone/zarlink/profiles/*.c))
BCMPH_SRCS += $(subst $(SUBDIRS)/,,$(wildcard $(SUBDIRS)/extern/zarlink/$(ZARLINK_VERSION)/api_lib/vp880_api/*.c))
BCMPH_SRCS += $(subst $(SUBDIRS)/,,$(wildcard $(SUBDIRS)/extern/zarlink/$(ZARLINK_VERSION)/api_lib/common/*.c))
BCMPH_SRCS += $(subst $(SUBDIRS)/,,$(wildcard $(SUBDIRS)/extern/zarlink/arch/bcm63xx/*.c))

BCMPH_OBJS := $(patsubst %.c,%.o,$(BCMPH_SRCS))

obj-m := bcm63xx-phone.o

bcm63xx-phone-objs := $(BCMPH_OBJS)
bcm63xx-phone-objs += main.o ioctl.o
