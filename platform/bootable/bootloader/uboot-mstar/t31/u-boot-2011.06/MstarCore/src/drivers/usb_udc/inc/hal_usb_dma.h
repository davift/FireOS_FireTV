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

#ifndef __HAL_USB_DMA_H__
#define __HAL_USB_DMA_H__

#define M_REG_DMA_INTR		(0x200<<otgOffShift)

#define DMA_BASE_ADDRESS()		OTGBaseAddress + (0x200<<otgOffShift)

#define DMA_CNTL_REGISTER(channel)		(U16 volatile *)(DMA_BASE_ADDRESS() + ((0x10 * (channel - 1) + 4)<<otgOffShift))

#ifdef  MIU_16BIT
#define DMA_ADDR_REGISTER(channel)		(U16 volatile*)(DMA_BASE_ADDRESS() + ((0x10 * (channel - 1) + 8)<<otgOffShift))
#define DMA_COUNT_REGISTER(channel)		(U16 volatile*)(DMA_BASE_ADDRESS() + ((0x10 * (channel - 1) + 0xc)<<otgOffShift))
#else
#define DMA_ADDR_REGISTER(channel)		(U32 volatile*)(DMA_BASE_ADDRESS() + ((0x10 * (channel - 1) + 8)<<otgOffShift))
#define DMA_COUNT_REGISTER(channel)		(U32 volatile*)(DMA_BASE_ADDRESS() + ((0x10 * (channel - 1) + 0xc)<<otgOffShift))
#endif

#define DMA_TX		0x2
#define DMA_RX		0x0
#define DMA_MODE_ZERO		0x0
#define DMA_MODE_ONE		0x4
#define DMA_IRQ_ENABLE		0x8
#define DMA_IRQ_DISABLE		0x0
#define DMA_MODE_MASK		(DMA_TX | DMA_MODE_ONE)
#define DMA_TX_ZERO_IRQ		(DMA_TX | DMA_MODE_ZERO | DMA_IRQ_ENABLE)
#define DMA_RX_ZERO_IRQ		(DMA_RX | DMA_MODE_ZERO | DMA_IRQ_ENABLE)
#define DMA_TX_ONE_IRQ		(DMA_TX | DMA_MODE_ONE | DMA_IRQ_ENABLE)
#define DMA_RX_ONE_IRQ		(DMA_RX | DMA_MODE_ONE | DMA_IRQ_ENABLE)

#define DMA_BurstMode		0x03

#define RXCSR2_MODE1		(M_RXCSR2_AUTOCLEAR | M_RXCSR2_DMAENAB | M_RXCSR2_DMAMODE)
#define TXCSR2_MODE1		(M_TXCSR2_DMAENAB | M_TXCSR2_AUTOSET | M_TXCSR2_DMAMODE)

#define DMA_ENABLE_BIT		0x0001
#define DMA_BUSERROR_BIT	0x0100
#define DMA_ENDPOINT_SHIFT	4

#define EP_IRQ_ENABLE		1
#define EP_IRQ_DISABLE		0
#define EP_IRQ_RX		0
#define EP_IRQ_TX		2

#define Enable_TX_EP_Interrupt(endpoint) \
	_HalUsbDmaControlEpInterrupt(endpoint, (EP_IRQ_ENABLE | EP_IRQ_TX), ptUsb);

#define Enable_RX_EP_Interrupt(endpoint) \
	_HalUsbDmaControlEpInterrupt(endpoint, (EP_IRQ_ENABLE | EP_IRQ_RX), ptUsb);

#define Disable_TX_EP_Interrupt(endpoint) \
	_HalUsbDmaControlEpInterrupt(endpoint, (EP_IRQ_DISABLE | EP_IRQ_TX), ptUsb);

#define Disable_RX_EP_Interrupt(endpoint) \
	_HalUsbDmaControlEpInterrupt(endpoint, (EP_IRQ_DISABLE | EP_IRQ_RX), ptUsb);

void HalUsbDmaIrqHandler(u8, pUsbVar ptUsb);
void _HalUsbDmaControlEpInterrupt(s8 nEp, u32 nMode, pUsbVar ptUsb);
u16 HalUsbDmaReadDmaControl(s8 nChannel);
s8 HalUsbDmaSetDMA(s8 nEp, u32 nMode, pUsbVar ptUsb);
#endif
