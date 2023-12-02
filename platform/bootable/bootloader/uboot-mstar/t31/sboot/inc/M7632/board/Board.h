/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2019 MediaTek Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef _BOARD_H_
#define _BOARD_H_

#ifndef _ENUM_DEVICE_H_
#include "Enum_Device.h"
#endif

#ifndef __MEMORY_MAP_SEL_H__
#include "mmap/MMap.h"
#endif

//------------------------------------------------------------------------------
#define ENABLE                          1
#define DISABLE                         0

//------------------------------UNUSED MACROS-----------------------------------
#define _FUNC_NOT_USED()                do {} while ( 0 )

//------------------------------MS_BOARD_TYPE_SEL-------------------------------
// Demo board made in Taiwan

#define BD_FPGA_M7632                   0xC000
#define BD_MTV19006_MT5871_M7632        0xC001
#define BD_MTV19008_MT5871_M7632        0xC002
#define BD_MT164B_10AT_M7632            0xC010
#define BD_MT164B_10AT_M7632_DUCKIE     0xC011
#define BD_MT164B_10AT_M7632_BRANDENBURG     0xC011
#define BD_MT164B_10AT_M7632_SKIPPER    0xC011
#define BD_MT164B_10AT_M7632_ANNA       0xC011
#define BD_MT164B_10AT_M7632_HAILEY     0xC011
#define BD_MT164B_10AT_M7632_TEDDY      0xC011
#define BD_MT164B_10AT_M7632_JULIANA      0xC011
#define BD_MT164B_10AT_M7632_SHELLY     0xC011


#define BD_UNKNOWN                      0xFFFF

#ifndef MS_BOARD_TYPE_SEL
#if defined (CONFIG_MSTAR_TITANIA_BD_FPGA)
#define MS_BOARD_TYPE_SEL               BD_FPGA_M7632
#include "BD_FPGA_M7632.h"

#elif defined (CONFIG_MTK_BD_MTV19006_MT5871_M7632)
    #define MS_BOARD_TYPE_SEL       	BD_MTV19006_MT5871_M7632
    #include "BD_MTV19006_MT5871_M7632.h"
#elif defined (CONFIG_MTK_BD_MTV19008_MT5871_M7632)
    #define MS_BOARD_TYPE_SEL       	BD_MTV19008_MT5871_M7632
    #include "BD_MTV19008_MT5871_M7632.h"
#elif defined (CONFIG_MTK_BD_MT164B_10AT_M7632)
    #define MS_BOARD_TYPE_SEL       	BD_MT164B_10AT_M7632
    #include "BD_MT164B_10AT_M7632.h"
#elif defined (CONFIG_MTK_BD_MT164B_10AT_M7632_DUCKIE)
    #define MS_BOARD_TYPE_SEL       	BD_MT164B_10AT_M7632_DUCKIE
    #include "BD_MT164B_10AT_M7632_DUCKIE.h"
#elif defined (CONFIG_MTK_BD_MT164B_10AT_M7632_BRANDENBURG)
    #define MS_BOARD_TYPE_SEL       	BD_MT164B_10AT_M7632_BRANDENBURG
    #include "BD_MT164B_10AT_M7632_BRANDENBURG.h"
#elif defined (CONFIG_MTK_BD_MT164B_10AT_M7632_SKIPPER)
    #define MS_BOARD_TYPE_SEL       	BD_MT164B_10AT_M7632_SKIPPER
    #include "BD_MT164B_10AT_M7632_SKIPPER.h"
#elif defined (CONFIG_MTK_BD_MT164B_10AT_M7632_ANNA)
    #define MS_BOARD_TYPE_SEL       	BD_MT164B_10AT_M7632_ANNA
    #include "BD_MT164B_10AT_M7632_ANNA.h"
#elif defined (CONFIG_MTK_BD_MT164B_10AT_M7632_HAILEY)
    #define MS_BOARD_TYPE_SEL       	BD_MT164B_10AT_M7632_HAILEY
    #include "BD_MT164B_10AT_M7632_HAILEY.h"
#elif defined (CONFIG_MTK_BD_MT164B_10AT_M7632_TEDDY)
    #define MS_BOARD_TYPE_SEL           BD_MT164B_10AT_M7632_TEDDY
    #include "BD_MT164B_10AT_M7632_TEDDY.h"
#elif defined (CONFIG_MTK_BD_MT164B_10AT_M7632_JULIANA)
    #define MS_BOARD_TYPE_SEL           BD_MT164B_10AT_M7632_JULIANA
    #include "BD_MT164B_10AT_M7632_JULIANA.h"
#elif defined (CONFIG_MTK_BD_MT164B_10AT_M7632_SHELLY)
    #define MS_BOARD_TYPE_SEL       	BD_MT164B_10AT_M7632_SHELLY
    #include "BD_MT164B_10AT_M7632_SHELLY.h"
#else
#error "Unknown Board Selection"

#endif
#endif

#endif // _BOARD_H_
