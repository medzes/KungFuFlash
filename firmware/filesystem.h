/*
 * Copyright (c) 2019-2020 Kim Jørgensen
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

static bool filesystem_mount(void);
static bool filesystem_unmount(void);
static bool file_open(FIL *file, const char *file_name, uint8_t mode);
static uint32_t file_read(FIL *file, void *buffer, size_t bytes);
static bool file_seek(FIL *file, FSIZE_t offset);
static uint32_t file_write(FIL *file, void *buffer, size_t bytes);
static bool file_truncate(FIL *file);
static bool file_sync(FIL *file);
static bool file_close(FIL *file);
static bool file_stat(const char *file_name, FILINFO *file_info);
static bool file_delete(const char *file_name);
static bool dir_change(const char *path);
static bool dir_current(char *path, size_t path_size);
static bool dir_open(DIR *dir, const char *pattern);
static bool dir_read(DIR *dir, FILINFO *file_info);
static bool dir_close(DIR *dir);
