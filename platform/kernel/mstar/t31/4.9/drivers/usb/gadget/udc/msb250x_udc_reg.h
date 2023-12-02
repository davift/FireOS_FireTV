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

/*------------------------------------------------------------------------------
    PROJECT: MSB250x Linux BSP
    DESCRIPTION:
          MSB250x dual role USB device controllers


    HISTORY:
         6/11/2010     Calvin Hung    First Revision
-------------------------------------------------------------------------------*/
#ifndef _MSB250X_UDC_REG_H
#define _MSB250X_UDC_REG_H

/*--------------------------------------------------------------------
                         OTG USB IP Register Definition
--------------------------------------------------------------------*/
//#define GET_UDC_REG_ADDR(x, y)  (x+(y)*4)
#define GET_UDC_REG_ADDR(x, y)  (x+(y*2))
//#define IO_ADDRESS(x)   x

//#define REG_CLK_OTG20_EN        GET_UDC_REG_ADDR(CHIPTOP_BASE_ADDR, 0x2d)
#define UTMI_SIGNAL_STATUS      GET_UDC_REG_ADDR(USBC_BASE_ADDR, 0x04)
#define UTMI_INTERRUPT_STATUS   GET_UDC_REG_ADDR(USBC_BASE_ADDR, 0x03)
//#define REG_PM_IRQ_MASK         GET_UDC_REG_ADDR(PM_BASE, 0x57) /* A000695C */

#define CPU_OFF_SHIFT 		0 /* 16 bit */
#define MSB250X_USBCREG(y)  ((y * 2) + OTG0_BASE_ADDR)

/* 00h ~ 0Fh */
#define MSB250X_UDC_FADDR_REG			MSB250X_USBCREG(0x00)
#define MSB250X_UDC_PWR_REG				MSB250X_UDC_FADDR_REG + 1
#define MSB250X_UDC_INTRTX_REG    		MSB250X_USBCREG(0x02)
/* 03h reserved */
#define MSB250X_UDC_INTRRX_REG    		MSB250X_USBCREG(0x04)
/* 05h reserved */
#define MSB250X_UDC_INTRTXE_REG  		MSB250X_USBCREG(0x06)
#define MSB250X_UDC_INTRTX1E_REG       MSB250X_USBCREG(0x06)
#define MSB250X_UDC_INTRTX2E_REG       MSB250X_UDC_INTRTX1E_REG + 1
/* 07h reserved */
#define MSB250X_UDC_INTRRXE_REG  		MSB250X_USBCREG(0x08)
#define MSB250X_UDC_INTRRX1E_REG  		MSB250X_USBCREG(0x08)
#define MSB250X_UDC_INTRRX2E_REG  		MSB250X_UDC_INTRRX1E_REG + 1
/* 09h reserved */
#define MSB250X_UDC_INTRUSB_REG  		MSB250X_USBCREG(0x0A)
#define MSB250X_UDC_INTRUSBE_REG 		MSB250X_UDC_INTRUSB_REG + 1
#define MSB250X_UDC_FRAME_L_REG   		MSB250X_USBCREG(0x0C)
#define MSB250X_UDC_FRAME_H_REG  		MSB250X_UDC_FRAME_L_REG + 1
#define MSB250X_UDC_INDEX_REG       		MSB250X_USBCREG(0x0E)
#define MSB250X_UDC_TESTMODE_REG  		MSB250X_UDC_INDEX_REG + 1

/* 10h ~ 1Fh */
#define MSB250X_UDC_TXMAP_L_REG  		MSB250X_USBCREG(0x10)
#define MSB250X_UDC_TXMAP_H_REG  		MSB250X_UDC_TXMAP_L_REG +1

/* for EP_SEL = 0, 12h ~ 1Fh */
#define MSB250X_UDC_CSR0_REG         		MSB250X_USBCREG(0x12)
#define MSB250X_UDC_CSR0_FLSH_REG  		MSB250X_UDC_CSR0_REG + 1
/* 14h ~ 17h reserved */
#define MSB250X_UDC_COUNT0_REG 			MSB250X_USBCREG(0x18)
/* 19h ~ 1Eh reserved */
#define MSB250X_UDC_CONFDATA_REG 		    MSB250X_USBCREG(0x1F)

/* for EP_SEL != 0, 12h ~ 1Fh (EP1 ~ EP3) */
#define MSB250X_UDC_TXCSR1_REG 			MSB250X_USBCREG(0x12)
#define MSB250X_UDC_TXCSR2_REG 			MSB250X_UDC_TXCSR1_REG + 1
#define MSB250X_UDC_RXMAP_L_REG 		MSB250X_USBCREG(0x14)
#define MSB250X_UDC_RXMAP_H_REG 		MSB250X_UDC_RXMAP_L_REG + 1
#define MSB250X_UDC_RXCSR1_REG 			MSB250X_USBCREG(0x16)
#define MSB250X_UDC_RXCSR2_REG 			MSB250X_UDC_RXCSR1_REG + 1
#define MSB250X_UDC_RXCOUNT_L_REG 		MSB250X_USBCREG(0x18)
#define MSB250X_UDC_RXCOUNT_H_REG 		MSB250X_UDC_RXCOUNT_L_REG + 1

#define MSB250X_UDC_TXTYPE             MSB250X_USBCREG(0x1A)
#define MSB250X_UDC_TXINTERVAL         MSB250X_USBCREG(0x1B)
#define MSB250X_UDC_RXTYPE             MSB250X_USBCREG(0x1C)
#define MSB250X_UDC_RXINTERVAL         MSB250X_USBCREG(0x1D)

#define MSB250X_UDC_FIFOSIZE_REG 		MSB250X_USBCREG(0x1F)

/* 20h ~ 2Fh */
/* Transfers to and from FIFOs may be 8-bit, 16-bit or 32-bit as required */
#define MSB250X_UDC_EP0_FIFO_ACCESS_L   	MSB250X_USBCREG(0x20)
#define MSB250X_UDC_EP0_FIFO_ACCESS_M1 MSB250X_UDC_EP0_FIFO_ACCESS_L + 1
#define MSB250X_UDC_EP0_FIFO_ACCESS_M2 MSB250X_USBCREG(0x22)
#define MSB250X_UDC_EP0_FIFO_ACCESS_H   MSB250X_UDC_EP0_FIFO_ACCESS_M2 + 1
#define MSB250X_UDC_EP1_FIFO_ACCESS_L   	MSB250X_USBCREG(0x24)
#define MSB250X_UDC_EP1_FIFO_ACCESS_M1 MSB250X_UDC_EP1_FIFO_ACCESS_L + 1
#define MSB250X_UDC_EP1_FIFO_ACCESS_M2 MSB250X_USBCREG(0x26)
#define MSB250X_UDC_EP1_FIFO_ACCESS_H   MSB250X_UDC_EP1_FIFO_ACCESS_M2 + 1
#define MSB250X_UDC_EP2_FIFO_ACCESS_L    MSB250X_USBCREG(0x28)
#define MSB250X_UDC_EP2_FIFO_ACCESS_M1  MSB250X_UDC_EP2_FIFO_ACCESS_L + 1
#define MSB250X_UDC_EP2_FIFO_ACCESS_M2 MSB250X_USBCREG(0x2A)
#define MSB250X_UDC_EP2_FIFO_ACCESS_H   	MSB250X_UDC_EP2_FIFO_ACCESS_M2 + 1
#define MSB250X_UDC_EP3_FIFO_ACCESS_L  	 MSB250X_USBCREG(0x2C)
#define MSB250X_UDC_EP3_FIFO_ACCESS_M1 MSB250X_UDC_EP3_FIFO_ACCESS_L + 1
#define MSB250X_UDC_EP3_FIFO_ACCESS_M2 MSB250X_USBCREG(0x2E)
#define MSB250X_UDC_EP3_FIFO_ACCESS_H   MSB250X_UDC_EP3_FIFO_ACCESS_M2 + 1
#define MSB250X_UDC_EP4_FIFO_ACCESS_L  	 MSB250X_USBCREG(0x30)
#define MSB250X_UDC_EP4_FIFO_ACCESS_M1 MSB250X_UDC_EP4_FIFO_ACCESS_L + 1
#define MSB250X_UDC_EP4_FIFO_ACCESS_M2 MSB250X_USBCREG(0x32)
#define MSB250X_UDC_EP4_FIFO_ACCESS_H   MSB250X_UDC_EP4_FIFO_ACCESS_M2 + 1
#define MSB250X_UDC_EP5_FIFO_ACCESS_L  	 MSB250X_USBCREG(0x34)
#define MSB250X_UDC_EP5_FIFO_ACCESS_M1 MSB250X_UDC_EP5_FIFO_ACCESS_L + 1
#define MSB250X_UDC_EP5_FIFO_ACCESS_M2 MSB250X_USBCREG(0x36)
#define MSB250X_UDC_EP5_FIFO_ACCESS_H   MSB250X_UDC_EP5_FIFO_ACCESS_M2 + 1
#define MSB250X_UDC_EP6_FIFO_ACCESS_L  	 MSB250X_USBCREG(0x38)
#define MSB250X_UDC_EP6_FIFO_ACCESS_M1 MSB250X_UDC_EP6_FIFO_ACCESS_L + 1
#define MSB250X_UDC_EP6_FIFO_ACCESS_M2 MSB250X_USBCREG(0x3A)
#define MSB250X_UDC_EP6_FIFO_ACCESS_H   MSB250X_UDC_EP6_FIFO_ACCESS_M2 + 1
#define MSB250X_UDC_EP7_FIFO_ACCESS_L  	 MSB250X_USBCREG(0x3C)
#define MSB250X_UDC_EP7_FIFO_ACCESS_M1 MSB250X_UDC_EP7_FIFO_ACCESS_L + 1
#define MSB250X_UDC_EP7_FIFO_ACCESS_M2 MSB250X_USBCREG(0x3E)
#define MSB250X_UDC_EP7_FIFO_ACCESS_H   MSB250X_UDC_EP7_FIFO_ACCESS_M2 + 1

/* 30h ~ 5Fh reserved */

/* 60h */
#define MSB250X_UDC_DEVCTL_REG 			MSB250X_USBCREG(0x60)

/* 80h ~ 8Fh */
#define MSB250X_UDC_USB_CFG0_L 			MSB250X_USBCREG(0x80)
#define MSB250X_UDC_USB_CFG0_H 			MSB250X_UDC_USB_CFG0_L + 1
#define MSB250X_UDC_USB_CFG1_L  			MSB250X_USBCREG(0x82)
#define MSB250X_UDC_USB_CFG1_H  			MSB250X_UDC_USB_CFG1_L + 1
#define MSB250X_UDC_USB_CFG2_L  			MSB250X_USBCREG(0x84)
#define MSB250X_UDC_USB_CFG2_H  			MSB250X_UDC_USB_CFG2_L + 1
#define MSB250X_UDC_USB_CFG3_L  			MSB250X_USBCREG(0x86)
#define MSB250X_UDC_USB_CFG3_H  			MSB250X_UDC_USB_CFG3_L + 1
#define MSB250X_UDC_USB_CFG4_L  			MSB250X_USBCREG(0x88)
#define MSB250X_UDC_USB_CFG4_H  			MSB250X_UDC_USB_CFG4_L + 1
#define MSB250X_UDC_USB_CFG5_L  			MSB250X_USBCREG(0x8A)
#define MSB250X_UDC_USB_CFG5_H  			MSB250X_UDC_USB_CFG5_L + 1
#define MSB250X_UDC_USB_CFG6_L   			MSB250X_USBCREG(0x8C)
#define MSB250X_UDC_USB_CFG6_H  			MSB250X_UDC_USB_CFG6_L + 1
#define MSB250X_UDC_USB_CFG7_L   	   MSB250X_USBCREG(0x8E)
#define MSB250X_UDC_USB_CFG7_H  	   MSB250X_UDC_USB_CFG7_L + 1

#define MSB250X_UDC_EP_BULKOUT MSB250X_UDC_USB_CFG3_L
#define MSB250X_UDC_DMA_MODE_CTL MSB250X_UDC_USB_CFG5_L
#define MSB250X_UDC_DMA_MODE_CTL1	   (MSB250X_UDC_USB_CFG0_L+1)


/* 200h */
#define MSB250X_UDC_DMA_INTR               MSB250X_USBCREG(0x200)
#define MSB250X_UDC_DMA_CNTL(x)            MSB250X_USBCREG((0x204+(x*0x0010ul)))
#define MSB250X_UDC_DMA_ADDR_LW(x)         MSB250X_USBCREG((0x208+(x*0x0010ul)))
#define MSB250X_UDC_DMA_ADDR_HW(x)         MSB250X_USBCREG((0x20A+(x*0x0010ul)))
#define MSB250X_UDC_DMA_CNL_LW(x)          MSB250X_USBCREG((0x20C+(x*0x0010ul)))
#define MSB250X_UDC_DMA_CNL_HW(x)          MSB250X_USBCREG(((x*0x0010ul)+0x20E))



/* MSB250X_UDC_PWR_REG */ /* RW */
#define MSB250X_UDC_PWR_ISOUP		(1 << 7)
#define MSB250X_UDC_PWR_SOFT_CONN  (1 << 6)
#define MSB250X_UDC_PWR_HS_EN           (1 << 5)
#define MSB250X_UDC_PWR_HS_MODE      (1 << 4)
#define MSB250X_UDC_PWR_RESET		(1 << 3)
#define MSB250X_UDC_PWR_RESUME		(1 << 2)
#define MSB250X_UDC_PWR_SUSPEND	(1 << 1)
#define MSB250X_UDC_PWR_ENSUSPEND	(1 << 0)

/* MSB250X_UDC_INTRTX_REG */ /* RO */
#define MSB250X_UDC_INTRTX_EP3          (1 << 3)
#define MSB250X_UDC_INTRTX_EP2          (1 << 2)
#define MSB250X_UDC_INTRTX_EP1          (1 << 1)
#define MSB250X_UDC_INTRTX_EP0          (1 << 0)

/* MSB250X_UDC_INTRRX_REG */ /* RO*/
#define MSB250X_UDC_INTRRX_EP3          (1 << 3)
#define MSB250X_UDC_INTRRX_EP2          (1 << 2)
#define MSB250X_UDC_INTRRX_EP1          (1 << 1)

/* MSB250X_UDC_INTRTXE_REG */ /* RW */
#define MSB250X_UDC_INTRTXE_EP3          (1 << 3)
#define MSB250X_UDC_INTRTXE_EP2          (1 << 2)
#define MSB250X_UDC_INTRTXE_EP1          (1 << 1)
#define MSB250X_UDC_INTRTXE_EP0          (1 << 0)

/* MSB250X_UDC_INTRRXE_REG */ /* RW */
#define MSB250X_UDC_INTRRXE_EP3          (1 << 3)
#define MSB250X_UDC_INTRRXE_EP2          (1 << 2)
#define MSB250X_UDC_INTRRXE_EP1          (1 << 1)

/* MSB250X_UDC_INTRUSB_REG */ /* RO */
#define MSB250X_UDC_INTRUSB_VBUS_ERR (1 << 7)
#define MSB250X_UDC_INTRUSB_SESS_REQ (1 << 6)
#define MSB250X_UDC_INTRUSB_DISCONN   (1 << 5)
#define MSB250X_UDC_INTRUSB_CONN         (1 << 4)
#define MSB250X_UDC_INTRUSB_SOF           (1 << 3)
#define MSB250X_UDC_INTRUSB_RESET       (1 << 2)
#define MSB250X_UDC_INTRUSB_RESUME    (1 << 1)
#define MSB250X_UDC_INTRUSB_SUSPEND  (1 << 0)

/* MSB250X_UDC_INTRUSBE_REG */ /* RW */
#define MSB250X_UDC_INTRUSBE_VBUS_ERR (1 << 7)
#define MSB250X_UDC_INTRUSBE_SESS_REQ (1 << 6)
#define MSB250X_UDC_INTRUSBE_DISCONN   (1 << 5)
#define MSB250X_UDC_INTRUSBE_CONN         (1 << 4)
#define MSB250X_UDC_INTRUSBE_SOF           (1 << 3)
#define MSB250X_UDC_INTRUSBE_RESET       (1 << 2)
#define MSB250X_UDC_INTRUSBE_BABBLE      (1 << 2)
#define MSB250X_UDC_INTRUSBE_RESUME    (1 << 1)
#define MSB250X_UDC_INTRUSBE_SUSPEND  (1 << 0)

/* MSB250X_UDC_INDEX_REG */ /* RW */
#define MSB250X_UDC_INDEX_EP0		(0x00)

/* MSB250X_UDC_CSR0_REG */ /* RO, WO */
#define MSB250X_UDC_CSR0_SSETUPEND  (1 << 7)
#define MSB250X_UDC_CSR0_SRXPKTRDY  (1 << 6)
#define MSB250X_UDC_CSR0_SENDSTALL  (1 << 5)
#define MSB250X_UDC_CSR0_SETUPEND    (1 << 4)
#define MSB250X_UDC_CSR0_DATAEND     (1 << 3)
#define MSB250X_UDC_CSR0_SENTSTALL  (1 << 2)
#define MSB250X_UDC_CSR0_TXPKTRDY    (1 << 1)
#define MSB250X_UDC_CSR0_RXPKTRDY    (1 << 0)

/* CSR0 in host mode */
#define MSB250X_UDC_CSR0_STATUSPACKET (1 << 6)
#define MSB250X_UDC_CSR0_REQPACKET    (1 << 5)
#define MSB250X_UDC_CSR0_SETUPPACKET  (1 << 3)
#define MSB250X_UDC_CSR0_RXSTALL      (1 << 2)


/* MSB250X_UDC_TXCSR1_REG */ /* RO, WO */
#define MSB250X_UDC_TXCSR1_AUTOSET      ((u16)1 << 15)
#define MSB250X_UDC_TXCSR1_MODE         ((u16)1 << 13)
#define MSB250X_UDC_TXCSR1_DMAREQENAB   ((u16)1 << 12)
#define MSB250X_UDC_TXCSR1_FRCDATAOG    ((u16)1 << 11)
#define MSB250X_UDC_TXCSR1_DMAREQMODE   ((u16)1 << 10)
#define MSB250X_UDC_TXCSR1_CLRDATAOTG   (1 << 6)
#define MSB250X_UDC_TXCSR1_SENTSTALL      (1 << 5)
#define MSB250X_UDC_TXCSR1_SENDSTALL      (1 << 4)
#define MSB250X_UDC_TXCSR1_FLUSHFIFO      (1 << 3)
#define MSB250X_UDC_TXCSR1_UNDERRUN       (1 << 2)
#define MSB250X_UDC_TXCSR1_FIFONOEMPTY (1 << 1)
#define MSB250X_UDC_TXCSR1_TXPKTRDY        (1 << 0)

/* host mode */
#define MSB250X_UDC_TXCSR1_RXSTALL         (1 << 5)

/* MSB250X_UDC_TXCSR2_REG */ /* RW */
#define MSB250X_UDC_TXCSR2_AUTOSET         (1 << 7)
#define MSB250X_UDC_TXCSR2_ISOC               (1 << 6)
#define MSB250X_UDC_TXCSR2_MODE               (1 << 5)
#define MSB250X_UDC_TXCSR2_DMAREQENAB  (1 << 4)
#define MSB250X_UDC_TXCSR2_FRCDATAOG     (1 << 3)
#define MSB250X_UDC_TXCSR2_DMAREQMODE  (1 << 2)

/* MSB250X_UDC_RXCSR1_REG */ /* RW, RO */
#define MSB250X_UDC_RXCSR1_CLRDATATOG   (1 << 7)
#define MSB250X_UDC_RXCSR1_SENTSTALL      (1 << 6)
#define MSB250X_UDC_RXCSR1_SENDSTALL      (1 << 5)
#define MSB250X_UDC_RXCSR1_FLUSHFIFO      (1 << 4)
#define MSB250X_UDC_RXCSR1_DATAERROR     (1 << 3)
#define MSB250X_UDC_RXCSR1_OVERRUN         (1 << 2)
#define MSB250X_UDC_RXCSR1_FIFOFULL         (1 << 1)
#define MSB250X_UDC_RXCSR1_RXPKTRDY        (1 << 0)

/* host mode */
#define MSB250X_UDC_RXCSR1_RXSTALL         (1 << 6)
#define MSB250X_UDC_RXCSR1_REQPKT          (1 << 5)

/* MSB250X_UDC_RXCSR2_REG */ /* RW */
#define MSB250X_UDC_RXCSR2_AUTOCLR         (1 << 7)
#define MSB250X_UDC_RXCSR2_ISOC            (1 << 6)
#define MSB250X_UDC_RXCSR2_DMAREQEN        (1 << 5)
#define MSB250X_UDC_RXCSR2_DISNYET         (1 << 4)
#define MSB250X_UDC_RXCSR2_DMAREQMD        (1 << 3)


/* MSB250X_UDC_DEVCTL_REG */
#define MSB250X_UDC_B_DEVIC                (1 << 7)
#define MSB250X_UDC_FS_DEVIC               (1 << 6)
#define MSB250X_UDC_LS_DEVIC               (1 << 5)
#define MSB250X_UDC_HOST_MODE              (1 << 2)
#define MSB250X_UDC_HOST_REQ               (1 << 1)
#define MSB250X_UDC_SESSION                (1 << 0)

/* CH_DMA_CNTL */
#define MSB250X_UDC_BURST_MODE             ((u16)3 << 9)
#define MSB250X_UDC_DMA_INT_EN             (1 << 3)
#define MSB250X_UDC_DMA_AUTO               (1 << 2)
#define MSB250X_UDC_DMA_TX                 (1 << 1)
#define MSB250X_UDC_EN_DMA                 (1 << 0)





/* DMA registers */
/* 200h ~  */
#define MSB250X_UDC_DMA_INTR_REG 			MSB250X_USBCREG(0x200)

#define MSB250X_UDC_DMA_CTRL1_REG 			MSB250X_USBCREG(0x204)
#define MSB250X_UDC_DMA_ADDR1_L_REG        MSB250X_USBCREG(0x208)
#define MSB250X_UDC_DMA_ADDR1_H_REG        MSB250X_USBCREG(0x20A)
#define MSB250X_UDC_DMA_COUNT1_L_REG 		MSB250X_USBCREG(0x20C)
#define MSB250X_UDC_DMA_COUNT1_H_REG 		MSB250X_USBCREG(0x20E)

#define MSB250X_UDC_DMA_CTRL2_REG 			MSB250X_USBCREG(0x214)
#define MSB250X_UDC_DMA_ADDR2_L_REG 		MSB250X_USBCREG(0x218)
#define MSB250X_UDC_DMA_ADDR2_H_REG        MSB250X_USBCREG(0x21A)
#define MSB250X_UDC_DMA_COUNT2_L_REG 		MSB250X_USBCREG(0x21C)
#define MSB250X_UDC_DMA_COUNT2_H_REG 		MSB250X_USBCREG(0x21E)

#define MSB250X_UDC_DMA_CTRL3_REG 			MSB250X_USBCREG(0x224)
#define MSB250X_UDC_DMA_ADDR3_L_REG 		MSB250X_USBCREG(0x228)
#define MSB250X_UDC_DMA_ADDR3_H_REG        MSB250X_USBCREG(0x22A)
#define MSB250X_UDC_DMA_COUNT3_L_REG 		MSB250X_USBCREG(0x22C)
#define MSB250X_UDC_DMA_COUNT3_H_REG 		MSB250X_USBCREG(0x22E)


/* USBC Registers */
#define UDC_USBC_MIU_CLK_REG	0x20
#define UDC_USBC_MIU_CLK_EN		0x8000

#endif /* _MSB250X_UDC_REG_H */
