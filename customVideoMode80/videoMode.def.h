/*
 *  Uzebox Kernel - Video Mode 80
 *  Copyright (C) 2019 Sandor Zsuga (Jubatian)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Uzebox is a reserved trade mark
*/

/**
 * ===========================================================================
 *
 * Global defines for custom video mode 80
 *
 * ===========================================================================
 */

#pragma once

#define VMODE_ASM_SOURCE QUOTE(VIDEO_MODE_PATH/videoModeCore.s)
#define VMODE_C_SOURCE QUOTE(VIDEO_MODE_PATH/videoMode.c)
#define VMODE_C_PROTOTYPES QUOTE(VIDEO_MODE_PATH/videoMode.h)
#define VMODE_FUNC sub_video_mode80
/*
#define VMODE_ASM_SOURCE <videoModeCore.s>
#define VMODE_C_SOURCE <videoMode.c>
#define VMODE_C_PROTOTYPES <videoMode.h>
#define VMODE_FUNC sub_video_mode80
*/

#ifndef TILE_HEIGHT
	#define TILE_HEIGHT 8
#endif

#define TILE_WIDTH 8

#ifndef SCREEN_TILES_H
	#define SCREEN_TILES_H 80
#endif

#define VRAM_TILES_H SCREEN_TILES_H

#ifndef SCREEN_TILES_V
	#define SCREEN_TILES_V 28
#endif

#ifndef VRAM_TILES_V
	#define VRAM_TILES_V SCREEN_TILES_V
#endif

#ifndef FIRST_RENDER_LINE
	#define FIRST_RENDER_LINE 20
#endif

#ifndef FRAME_LINES
	#define FRAME_LINES (SCREEN_TILES_V * TILE_HEIGHT)
#endif

#ifndef FONT_TILE_INDEX
	#define FONT_TILE_INDEX 0x20
#endif

/* VRAM characteristics */

#define VRAM_SIZE (VRAM_TILES_H * VRAM_TILES_V)
#define VRAM_ADDR_SIZE 1
#define VRAM_PTR_TYPE unsigned char

/* Maximum free cycles usable by the hysnc and audio (Note: more than 230 has
** no use since the kernel has such limitation elsewhere) */

#define HSYNC_USABLE_CYCLES 230
