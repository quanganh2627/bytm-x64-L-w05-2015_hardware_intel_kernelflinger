/*
 * Copyright (c) 2014, Intel Corporation
 * All rights reserved.
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
#include "acpi.h"
#include "lib.h"

#define DEVICE_INFO_PROTOCOL {0xE4F3260B, 0xD35F, 0x4AF1, {0xB9, 0x0E, 0x91, 0x0F, 0x5A, 0xD2, 0xE3, 0x26}}
static EFI_GUID DeviceInfoProtocolGuid = DEVICE_INFO_PROTOCOL;

#define USB_CHARGER_SDP (1 << 0);
#define USB_CHARGER_DCP (1 << 1);
#define USB_CHARGER_CDP (1 << 2);
#define USB_CHARGER_ACA (1 << 3);

typedef UINT8 USB_CHARGER_TYPE;
typedef UINT8 BATT_CAPACITY;
typedef UINT16 BATT_VOLTAGE;

typedef EFI_STATUS (EFIAPI *GET_BATTERY_STATUS) (
	OUT BOOLEAN *BatteryPresent,
	OUT BOOLEAN *BatteryValid,
	OUT BOOLEAN *CapacityReadable,
	OUT BATT_VOLTAGE *BatteryVoltageLevel,
	OUT BATT_CAPACITY *BatteryCapacityLevel);

typedef EFI_STATUS (EFIAPI *GET_ACDC_CHARGER_STATUS) (
	OUT BOOLEAN *ACDCChargerPresent);

typedef EFI_STATUS (EFIAPI *GET_USB_CHARGER_STATUS) (
	OUT BOOLEAN *UsbChargerPresent,
	OUT USB_CHARGER_TYPE *UsbChargerType);

struct _DEVICE_INFO_PROTOCOL {
	UINT32 Revision;
	GET_BATTERY_STATUS GetBatteryStatus;
	GET_ACDC_CHARGER_STATUS GetAcDcChargerStatus;
	GET_USB_CHARGER_STATUS GetUsbChargerStatus;
};

struct battery_status {
	BOOLEAN BatteryPresent;
	BOOLEAN BatteryValid;
	BOOLEAN CapacityReadable;
	BATT_VOLTAGE BatteryVoltageLevel;
	BATT_CAPACITY BatteryCapacityLevel;
};

static EFI_STATUS get_battery_status(struct battery_status *status)
{
	struct _DEVICE_INFO_PROTOCOL *dev_info;
	EFI_STATUS ret;

	ret = LibLocateProtocol(&DeviceInfoProtocolGuid, (VOID **)&dev_info);
	if (EFI_ERROR(ret) || !dev_info)
		goto error;

	ret = uefi_call_wrapper(dev_info->GetBatteryStatus, 5,
		&status->BatteryPresent,
		&status->BatteryValid,
		&status->CapacityReadable,
		&status->BatteryVoltageLevel,
		&status->BatteryCapacityLevel);

	if (EFI_ERROR(ret))
		goto error;
	return ret;

error:
	error(L"Failed to get battery status: %r\n", ret);
	return ret;
}

BOOLEAN is_battery_below_vbattfreqlmt(void)
{
	struct battery_status status;
	EFI_STATUS ret;
	UINT16 vbattfreqlmt, value;

	ret = get_battery_status(&status);
	if (EFI_ERROR(ret)) {
		error(L"Failed to get battery status: %r\n", ret);
		return FALSE;
	}

	value = status.BatteryVoltageLevel;
	vbattfreqlmt = oem1_get_ia_vbattfreqlmt();
	debug(L"Battery: %dmV Vbattfreqlmt: %dmV\n", value, vbattfreqlmt);

	return value < vbattfreqlmt;
}

BOOLEAN is_charger_plugged_in(void)
{
	struct _DEVICE_INFO_PROTOCOL *dev_info;
	BOOLEAN present;
	USB_CHARGER_TYPE type;
	EFI_STATUS ret;

	ret = LibLocateProtocol(&DeviceInfoProtocolGuid, (VOID **)&dev_info);
	if (EFI_ERROR(ret) || !dev_info)
		goto error;

	ret = uefi_call_wrapper(dev_info->GetUsbChargerStatus, 2, &present, &type);
	if (EFI_ERROR(ret))
		goto error;

	return present;

error:
	error(L"Failed to get charger status: %r", ret);
	return FALSE;
}

BOOLEAN is_battery_bellow_boot_OS_threshold(void)
{
	struct battery_status status;
	EFI_STATUS ret;
	UINTN value, threshold;
	UINT8 ia_apps_to_use;

	ret = get_battery_status(&status);
	if (EFI_ERROR(ret))
		return NORMAL_BOOT;

	ia_apps_to_use = oem1_get_ia_apps_to_use();
	if (ia_apps_to_use == (UINT8)-1) {
		error(L"OEM1 ACPI table parse error");
		return NORMAL_BOOT;
	}

	if (ia_apps_to_use == OEM1_USE_IA_APPS_CAP) {
		value = status.BatteryCapacityLevel;
		threshold = oem1_get_ia_apps_cap();
		debug(L"Battery: %d%% Threshold: %d%%", value, threshold);
		if (threshold == 0) {
			value = status.BatteryVoltageLevel;
			threshold = oem1_get_ia_apps_run();
			debug(L"Battery: %dmV Threshold: %dmV", value, threshold);
		}
	} else {
		value = status.BatteryVoltageLevel;
		threshold = oem1_get_ia_apps_run();
		debug(L"Battery: %dmV Threshold: %dmV", value, threshold);
		if (threshold == 0) {
			value = status.BatteryCapacityLevel;
			threshold = oem1_get_ia_apps_cap();
			debug(L"Battery: %d%% Threshold: %d%%", value, threshold);
		}
	}

	return value < threshold;
}
