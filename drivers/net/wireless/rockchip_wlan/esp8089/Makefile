# SPDX-License-Identifier: GPL-2.0
# copyright (c) 2011 - 2012 espressif system
DRIVER_NAME := es8089
include drivers/net/wireless/rockchip_wlan/esp8089/esp_config.mk

EXTRA_CFLAGS += -DDEBUG -DSIP_DEBUG -DFAST_TX_STATUS -DKERNEL_IV_WAR -DRX_SENDUP_SYNC -DHAS_FW -DTEST_MODE -DESP_ANDROID_LOGGER  -DESP_CLASS -DESP_PREALLOC -DHAS_INIT_DATA

obj-$(CONFIG_ESP8089) := $(DRIVER_NAME).o
$(DRIVER_NAME)-y += esp_init.o
$(DRIVER_NAME)-y += esp_debug.o
$(DRIVER_NAME)-y += sdio_sif_esp.o
$(DRIVER_NAME)-y += spi_sif_esp.o
$(DRIVER_NAME)-y += esp_io.o
$(DRIVER_NAME)-y += esp_file.o
$(DRIVER_NAME)-y += esp_main.o
$(DRIVER_NAME)-y += esp_sip.o
$(DRIVER_NAME)-y += esp_ext.o
$(DRIVER_NAME)-y += esp_ctrl.o
$(DRIVER_NAME)-y += esp_mac80211.o
$(DRIVER_NAME)-y += esp_debug.o
$(DRIVER_NAME)-y += esp_utils.o
$(DRIVER_NAME)-y += esp_pm.o
$(DRIVER_NAME)-y += testmode.o

#ARCH := arm
#CROSS_COMPILE := $(TOOLCHAIN_PREFIX)
#KSRC := $(KERNEL_DIR)
#MODPATH := $(shell pwd)

####################### NORMAL OPTION ########################################
EXTRA_CFLAGS += -DESP_PRE_MEM              # normal mode, support SDIO
#EXTRA_CFLAGS += -DESP_PRE_MEM -DESP_SPI   # normal mode, support SPI
##############################################################################

####################### ADVANCE OPTION ###########################
#EXTRA_CFLAGS += -DESP_SLAB                # ex mode, most users unlikely use this mode, if someone use, please add this macro extractly
##############################################################################
$(DRIVER_NAME)-y += esp_mem.o
$(DRIVER_NAME)-y += esp_slab.o
#$(DRIVER_NAME)-y += esp_mem_init.o

