LIBKERNELFLINGER_LOCAL_PATH := $(call my-dir)
include $(call all-subdir-makefiles)
LOCAL_PATH := $(LIBKERNELFLINGER_LOCAL_PATH)

include $(CLEAR_VARS)

PNG2C := $(HOST_OUT_EXECUTABLES)/png2c$(HOST_EXECUTABLE_SUFFIX)
GEN_IMAGES := $(LOCAL_PATH)/tools/gen_images.sh
GEN_FONTS := $(LOCAL_PATH)/tools/gen_fonts.sh

res_intermediates := $(call intermediates-dir-for,STATIC_LIBRARIES,libkernelflinger)

font_res := $(res_intermediates)/res/font_res.h
img_res := $(res_intermediates)/res/img_res.h

$(LOCAL_PATH)/ui_font.c: $(font_res)
$(LOCAL_PATH)/ui_image.c: $(img_res)

ifndef TARGET_KERNELFLINGER_IMAGES_DIR
TARGET_KERNELFLINGER_IMAGES_DIR := $(LOCAL_PATH)/res/images/
endif
ifndef TARGET_KERNELFLINGER_FONT_DIR
TARGET_KERNELFLINGER_FONT_DIR := $(LOCAL_PATH)/res/fonts/
endif

KERNELFLINGER_IMAGES := $(wildcard $(TARGET_KERNELFLINGER_IMAGES_DIR)/*.png)
KERNELFLINGER_FONTS := $(wildcard $(TARGET_KERNELFLINGER_FONT_DIR)/*.png)

$(img_res): $(KERNELFLINGER_IMAGES) $(PNG2C) $(GEN_IMAGES)
	$(hide) mkdir -p $(dir $@)
	$(hide) $(GEN_IMAGES) $(TARGET_KERNELFLINGER_IMAGES_DIR) $@

$(font_res): $(KERNELFLINGER_FONTS) $(PNG2C) $(GEN_FONTS)
	$(hide) mkdir -p $(dir $@)
	$(hide) $(GEN_FONTS) $(TARGET_KERNELFLINGER_FONT_DIR) $@

LOCAL_MODULE := libkernelflinger-$(TARGET_BUILD_VARIANT)
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/../include/libkernelflinger
LOCAL_CFLAGS := -DKERNELFLINGER -Wall -Wextra -Werror
LOCAL_STATIC_LIBRARIES := libefi libgnuefi libopenssl-efi libcryptlib

ifeq ($(TARGET_BUILD_VARIANT),user)
    LOCAL_CFLAGS += -DUSER -DUSERDEBUG
endif

ifeq ($(TARGET_BUILD_VARIANT),userdebug)
    LOCAL_CFLAGS += -DUSERDEBUG
endif

ifeq ($(TARGET_USE_USERFASTBOOT),true)
    LOCAL_CFLAGS += -DUSERFASTBOOT
endif

LOCAL_SRC_FILES := \
	android.c \
	efilinux.c \
	acpi.c \
	lib.c \
	options.c \
	security.c \
	asn1.c \
	keystore.c \
	vars.c \
	ui.c \
	ui_font.c \
	ui_textarea.c \
	ui_image.c \
	ui_boot_menu.c \
	uefi_em.c \
	log.c

LOCAL_C_INCLUDES := $(LOCAL_PATH)/../include/libkernelflinger \
		$(res_intermediates)

include $(BUILD_EFI_STATIC_LIBRARY)
