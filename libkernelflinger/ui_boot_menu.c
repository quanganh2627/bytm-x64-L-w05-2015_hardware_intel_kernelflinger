/*
 * Copyright (c) 2014, Intel Corporation
 * All rights reserved.
 *
 * Author:  Jeremy Compostella <jeremy.compostella@intel.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
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

#include <efi.h>
#include <efilib.h>
#include <lib.h>

#include "ui.h"

ui_boot_menu_t *ui_boot_menu_create(ui_boot_action_t *actions, ui_font_t *font)
{
	ui_boot_menu_t *menu;
	UINTN i;

	if (!font)
		return NULL;

	for (i = 0; actions[i].img_name; i++) {
		actions[i].image = ui_image_get(actions[i].img_name);
		if (!actions[i].image)
			return NULL;
	}

	menu = AllocateZeroPool(sizeof(*menu));
	if (!menu)
		return NULL;

	menu->actions = actions;
	menu->action_nb = i;
	menu->font = font;

	return menu;
}

static const UINTN MARGIN = 20;

static EFI_STATUS ui_boot_menu_redraw(ui_boot_menu_t *menu, UINTN *y)
{
	EFI_STATUS ret;
	ui_textline_t lines[] = {
		{ &COLOR_LIGHTGRAY, "Volume DOWN button to choose boot option", TRUE },
		{ &COLOR_LIGHTGRAY, "Volume UP button to select boot option", TRUE },
		{ NULL, NULL, TRUE }
	};

	*y = menu->y;

	ui_image_t *image = menu->actions[menu->cur].image;
	if (!image)
		return EFI_UNSUPPORTED;

	ret = ui_image_draw(image, menu->x, *y);
	if (EFI_ERROR(ret))
		return ret;

	*y += image->height + MARGIN;

	return ui_textarea_display_text(lines, menu->font, menu->x, y);
}

EFI_STATUS ui_boot_menu_draw(ui_boot_menu_t *menu, UINTN x, UINTN *y)
{
	menu->x = x;
	menu->y = *y;
	return ui_boot_menu_redraw(menu, y);
}

enum boot_target ui_boot_menu_event_handler(ui_boot_menu_t *menu, ui_events_t event)
{
	UINTN y;

	switch (event) {
	case EV_UP:
		return menu->actions[menu->cur].target;
	case EV_DOWN:
		menu->cur = (menu->cur + 1) % menu->action_nb;
		ui_boot_menu_redraw(menu, &y);
	default:
		break;
	}

	return UNKNOWN_TARGET;
}

void ui_boot_menu_free(ui_boot_menu_t *menu)
{
	FreePool(menu);
}
