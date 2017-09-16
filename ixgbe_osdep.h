/******************************************************************************

  Copyright (c) 2001-2012, Intel Corporation 
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are met:
  
   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.
  
   2. Redistributions in binary form must reproduce the above copyright 
      notice, this list of conditions and the following disclaimer in the 
      documentation and/or other materials provided with the distribution.
  
   3. Neither the name of the Intel Corporation nor the names of its 
      contributors may be used to endorse or promote products derived from 
      this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

******************************************************************************/
/*$FreeBSD$*/

#pragma once

#include <linux/string.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <asm/byteorder.h>

#define DELAY(x) udelay(x)
#define usec_delay(x) DELAY(x)
#define msec_delay(x) mdelay(x)

#define FALSE               0
#define TRUE                1

#define IXGBE_NTOHL(_i)	ntohl(_i)
#define IXGBE_NTOHS(_i)	ntohs(_i)
#define IXGBE_CPU_TO_LE32(_i)  cpu_to_le32(_i)
#define IXGBE_LE32_TO_CPUS(_i) le32_to_cpu(_i)

typedef uint8_t		u8;
typedef int8_t		s8;
typedef uint16_t	u16;
typedef uint32_t	u32;
typedef int32_t		s32;
typedef uint64_t	u64;

#define IXGBE_PCI_REG(reg) (*((volatile uint32_t *)(reg)))

static inline uint32_t ixgbe_read_addr(volatile void *addr)
{
	return IXGBE_PCI_REG(addr);
}

#define IXGBE_PCI_REG_WRITE(reg, value) do { \
	IXGBE_PCI_REG((reg)) = (value); \
} while(0)

#define IXGBE_PCI_REG_ADDR(hw, reg) \
	((volatile uint32_t *)((char *)(hw)->hw_addr + (reg)))

#define IXGBE_PCI_REG_ARRAY_ADDR(hw, reg, index) \
	IXGBE_PCI_REG_ADDR((hw), (reg) + ((index) << 2))

/* Not implemented !! */
#define IXGBE_READ_PCIE_WORD(hw, reg) 0	
#define IXGBE_WRITE_PCIE_WORD(hw, reg, value) do { } while(0)

#define IXGBE_WRITE_FLUSH(a) IXGBE_READ_REG(a, IXGBE_STATUS)

#define IXGBE_READ_REG(hw, reg) \
	ixgbe_read_addr(IXGBE_PCI_REG_ADDR((hw), (reg)))

#define IXGBE_WRITE_REG(hw, reg, value) \
	IXGBE_PCI_REG_WRITE(IXGBE_PCI_REG_ADDR((hw), (reg)), (value))

#define IXGBE_READ_REG_ARRAY(hw, reg, index) \
	IXGBE_PCI_REG(IXGBE_PCI_REG_ARRAY_ADDR((hw), (reg), (index)))

#define IXGBE_WRITE_REG_ARRAY(hw, reg, index, value) \
	IXGBE_PCI_REG_WRITE(IXGBE_PCI_REG_ARRAY_ADDR((hw), (reg), (index)), (value))
