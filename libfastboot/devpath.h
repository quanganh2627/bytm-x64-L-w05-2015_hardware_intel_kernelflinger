/*
 * Copyright (c) 2014, Intel Corporation
 * All rights reserved.
 *
 * Authors: Leo Sartre <leox.sartre@intel.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer
 *      in the documentation and/or other materials provided with the
 *      distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __DEVPATH_H__
#define __DEVPATH_H__

#include <efi.h>

#define PCI_DEVICE_PATH_NODE(Func, Dev)				\
	{							\
		{						\
			HARDWARE_DEVICE_PATH,			\
			HW_PCI_DP,				\
			{					\
				(UINT8) (sizeof (PCI_DEVICE_PATH)),	\
				(UINT8) ((sizeof (PCI_DEVICE_PATH)) >> 8)	\
			}	\
		},		\
			(Func),	\
			(Dev)	\
	}

#define PNPID_DEVICE_PATH_NODE(PnpId)	\
	{				\
		{			\
			ACPI_DEVICE_PATH,	\
			ACPI_DP,		\
			{			\
				(UINT8) (sizeof (ACPI_HID_DEVICE_PATH)),	\
				(UINT8) ((sizeof (ACPI_HID_DEVICE_PATH)) >> 8)	\
			},			 			    	\
		},								\
		EISA_PNP_ID((PnpId)),	\
		0			\
	}

#define gPciRootBridge PNPID_DEVICE_PATH_NODE(0x0A03)

#define gEndEntire				\
	{					\
		END_DEVICE_PATH_TYPE,		\
		END_ENTIRE_DEVICE_PATH_SUBTYPE,	\
		{				\
			END_DEVICE_PATH_LENGTH,	\
			0			\
		}				\
	}

#define CONTROLLER_DEVICE_PATH_NODE(Ctrl)	\
	{					\
		{				\
			HARDWARE_DEVICE_PATH,	\
			HW_CONTROLLER_DP,	\
			{			\
				sizeof (CONTROLLER_DEVICE_PATH),	\
				0      					\
			},						\
		},							\
		(Ctrl)							\
	}

struct _PLATFORM_PCI_DEVICE_PATH_EMMC
{
	ACPI_HID_DEVICE_PATH PciRootBridge;
	PCI_DEVICE_PATH Pci;
	CONTROLLER_DEVICE_PATH Ctrl;
	EFI_DEVICE_PATH End;
} __attribute__ ((packed));

typedef struct _PLATFORM_PCI_DEVICE_PATH_EMMC PLATFORM_PCI_DEVICE_PATH_EMMC;

#endif /* __DEVPATH_H__ */
