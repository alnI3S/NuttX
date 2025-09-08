/****************************************************************************
 * drivers/mtd/w25n01.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Standard SPI Instructions (p. 13)  Mode 0 and  Mode 3 are supported. */
/* Mode		CPOL		CPHA	*/
/* 0		0		0	*/
/* 1		0		1	*/
/* 2		1		0	*/
/* 3		1		1	*/
/* For Mode 0, the CLK signal is normally low on the falling and rising edges
 * of /CS. For Mode 3, the CLK signal is normally high on the falling and
 * rising edges of /CS.
 * In this normal state of the CLK the SPI bus master is in standby and data
 * is not being transfered to the Serial Flash.
 */

 /* QuadSPI Instructions: is mode still mining? */
#ifndef CONFIG_W25N01GV_QSPIMODE
#define CONFIG_W25N01GV_QSPIMODE QSPIDEV_MODE0
#endif

/* QuadSPI Frequency per data sheet:
 *
 * In this implementation, only "Quad" reads are performed.
 */

#ifndef CONFIG_W25N01GV_QSPI_FREQUENCY
/* If you haven't specified frequency, default to 100 MHz which will work
 * with all commands. up to 133MHz.
 */
#define CONFIG_W25N01GV_QSPI_FREQUENCY 104000000
#endif

#ifndef CONFIG_W25N01GV_DUMMIES
/* If you haven't specified the number of dummy cycles for quad reads,
 * provide a reasonable default.  The actual number of dummies needed is
 * clock and IO command dependent.(four to six times according to data sheet)
 */

#define CONFIG_W25N01GV_DUMMIES 6
#endif

/* W25N01GV Commands (p. 25) ************************************************/

#define W25N01GV_DEVICE_RESET    	0xFF	/* Device Reset	*/
#define W25N01GV_JEDEC_ID      	 	0x9F	/* JEDEC ID: EF AA 21	*/

#define W25N01GV_READ_STATUS		0x05	/* (or 0x0F) followed by SR-i Addr
						0xAx, 0xBx or 0xCx     */
#define W25N01GV_WRITE_STATUS		0x01	/* or 0x1F 	*/
#define W25N01GV_WRITE_ENABLE		0x06 	/* Write enable                      */
#define W25N01GV_WRITE_DISABLE		0x04 	/* Write disable                     */

#define W25N01GV_BB_MANAGEMENT  	0xA1 	/* Swap Blocks */
#define W25N01GV_READ_BBM       	0xA5	/* Read BBM LUT	*/
#define W25N01GV_LAST_EEC       	0xA9	/* Last ECC failure page address	*/
#define W25N01GV_BLOCK_ERASE 		0xD8	/* Block Erase (64 KB)    */

#define W25N01GV_DATA_LOAD   		0x02	/* reset buffer	*/
#define W25N01GV_RAND_DATA_LOAD 	0x84 	/* Random Program Data Load */
#define W25N01GV_DATA_LOAD_QUAD 	0x32  	/* Quad reset buffer             */
#define W25N01GV_RAND_DATA_LOAD_QUAD 	0x34  	/* quad Random Program Data Load           */

#define W25N01GV_PROGRAM_EXECUTE 	0x10	/* Program Execute	*/

#define W25N01GV_READ_PAGE    		0x13	/* Read Page Data	*/
#define W25N01GV_READ_DATA    		0x03	/* Read Data Bytes	*/

#define W25N01GV_FAST_READ    		0x0B	/* Fast Read Data Bytes	*/
#define W25N01GV_FAST_READ_4B		0x0C	/* Fast Read w/ 4-Bytes addr	*/
#define W25N01GV_FAST_READ_DUAL    	0x3B	/* Fast Read Dual Output	*/
#define W25N01GV_FAST_READ_DUAL_4B	0x3C	/* Fast Read Dual Output w/ 4-Bytes addr	*/
#define W25N01GV_FAST_READ_QUAD		0x6B	/* Fast Read Quad Output	*/
#define W25N01GV_FAST_READ_QUAD_4B	0x6C	/* Fast Read Quad Output w/ 4-Bytes addr	*/
#define W25N01GV_FAST_READ_DUALIO	0xBB	/* Fast Read Dual I/O	*/
#define W25N01GV_FAST_READ_DUALIO_4B	0xBC	/* Fast Read Dual I/O w/ 4-Bytes addr	*/
#define W25N01GV_FAST_READ_QUADIO	0xEB	/* Fast Read Quad I/O	*/
#define W25N01GV_FAST_READ_QUADIO_4B	0xEC	/* Fast Read Quad I/O w/ 4-Bytes addr	*/

/* W25N01GV Register Addresses **********************************************/
#define W25N01GV_PROT_ADDR              0xA0 	/* Protection SR-1, r/w	*/
#define W25N01GV_CONF_ADDR              0xB0	/* Configure SR-2, r/w	*/
#define W25N01GV_STATUS_ADDR		0xC0	/* Status SR-3, read only	*/

/* Status register 1 bit definitions                                      */
/* see p. 16-17
 * Bit  |   7  |  6  |  5  |  4  |  3  |  2  |   1  |   0  |
 *	--------------------------------------------------------------
 *	| SRP0 | BP3 | BP2 | BP1 | BP0 | TB  | WP-E | SRP1 |
 */
#define STATUS_SRP1_MASK     		(1 << 0) /* Bit 0: Status register protect-1  */
#define STATUS_SRP1_UNLOCKED  		(0 << 0) /*   see blow for details           */
#define STATUS_SRP1_LOCKED    		(1 << 0) /*   see blow for details           */
#define STATUS_WPE_MASK      		(1 << 1) /* Bit 1: /WP enable bit */
#define STATUS_WPE_UNLOCKED 		(0 << 1) /* 0 + SRP1 + SRP0 = Software protection         */
#define STATUS_WPE_LOCKED   		(1 << 1) /* 1 + SRP1 + SRP0 = Hardware protection              */
#define STATUS_BP_SHIFT      		(3)      /* Bits 3-6: 4 Block protect bits     */
#define STATUS_BP_4_MASK     		(15 << STATUS_BP_SHIFT)	/* all 1111 for BP0-3 */
/* Some chips have top/bottom bit at sixth bit                            */
#define STATUS_TB_6_MASK     (1 << 6) /* Bit 6: Top / Bottom Protect      */
/* Status Register Protect (SRP, SRL)
 * | SRP1 | SRP0 | WP-E | /WP / IO2 |	Status Register 	Description
 *    0      0      0      X  		Software Protection     No /WP functionality
 * 								/WP pin will always function as IO2
 *
 *    0      1      0      0            			SR-1 cannot be changed
 * 								/WP pin will function as IO2 for Quad operations
 *
 *    0      1      0      1               			SR-1 can be changed
 * 								/WP pin will function as IO2 for Quad operations
 *
 *    1      0      0  	   X					Power Lock-Down  SR-1
 * 								/WP pin will always function as IO2
 *
 *    1      1      0      X               			Enter OTP to protect SR-1 (allow SR1-L=1)
 * 								/WP pin will always function as IO2
 *
 * | SRP1 | SRP0 | WP-E | /WP only |	Status Register 	Description
 *    0      X      1      VCC  	Hardware Protected    SR-1 can be changed
 *
 *    1      0      1      VCC              			Power Lock-Down  SR-1
 *
 *    1      1      1      VCC               			Enter OTP mode to protect SR-1 (allow SR1-L=1)
 *
 *    X      X      1      GND  				All "Write/Program/Erase" commands are blocked
 * 								Entire device (SRs, Array, OTP area) is read-only
 */
#define STATUS_TB_MASK       		(1 << 2) /* Bit 2: Top / Bottom Protect      */
#define STATUS_TB_TOP        		(0 << 2) /*   0 = BP3-BP0 protect Top down   */
#define STATUS_TB_BOTTOM     		(1 << 2) /*   1 = BP3-BP0 protect Bottom up  */
#define STATUS_BP_MASK       		(15 << STATUS_BP_SHIFT)
#define STATUS_BP_NONE       		(0 << STATUS_BP_SHIFT)	/* all 0000 for BP0-3 */
#define STATUS_BP_ALL        		(31 << STATUS_TB_MASK)	/* all 11111 for TB-BP3 */
#define STATUS_SRP_0_MASK      		(1 << 7) /* Bit 7: Status register protect-0 */
#define STATUS_SRP_0_UNLOCKED  		(0 << 7) /*   see above for details           */
#define STATUS_SRP_0_LOCKED    		(1 << 7) /*   see above for details           */

/* Status register 2 bit definitions                                      */
/* see p. 18-19
 * Bit  |   7   |   6   |   5   |   4   |  3  |  2  |  1  |  0  |
 * 	--------------------------------------------------------------
 * 	| OTP-L | OTP-E | SR1-L | ECC-E | BUF | (R) | (R) | (R) |
*/
#define STATUS2_READ_MODE_MASK      	(3 << 3) /* Bit 3-4: Read Mode          */
#define STATUS2_CONTINUOUS_READ  	(0 << 4) /*  00 = Continuous read, Output 2048 */
#define STATUS2_BUFFER_READ   		(1 << 3) /*  01 = Buffer read, Output 2048 + 64 */
#define STATUS2_PAGE_READ     		(3 << 3) /*  11 = Buffer read, Page based, Output 2048 + 64 */
#define STATUS2_ECC_E_MASK       	(1 << 4) /* Bits 4: Enable ECC          */
#define STATUS2_ECC_E_DISABLED    	(0 << 4) /*  0 = ECC-E disabled     */
#define STATUS2_ECC_E_ENABLED     	(1 << 4) /*  1 = ECC-E enabled    */
#define STATUS2_SR1_L_MASK       	(1 << 5) /* Bit 5: Status Register-1 Lock          */
#define STATUS2_SR1_L_UNLOCKED    	(0 << 5) /*  0 = Status Register-1 unlocked     */
#define STATUS2_SR1_L_LOCKED     	(1 << 5) /*  1 = Status Register-1 locked    */
#define STATUS2_OTP_E_MASK      	(1 << 6) /* Bit 6: One Time Program Mode          */
#define STATUS2_OTP_E_DISABLED    	(0 << 6) /*  0 = OTP mode disabled     */
#define STATUS2_OTP_E_ENABLED     	(1 << 6) /*  1 = Enter OTP mode    */
#define STATUS2_OTP_L_MASK     		(1 << 7) /* Bit 7: OTP data page lock          */
#define STATUS2_OTP_L_UNLOCKED    	(0 << 7) /*  0 = OTP data page unlocked     */
#define STATUS2_OTP_L_LOCKED     	(1 << 7) /*  1 = OTP data page locked    */

/* Status register 3 bit definitions 					*/
/* see p. 20-21:
 * Note: this register is read-only
 * Bit  |  7  |   6   |   5   |   4   |   3    |    2   |  1  |   0  |
 * 	--------------------------------------------------------------
 * 	| (R) | LUT-F | ECC-1 | ECC-0 | P-FAIL | E-FAIL | WEL | BUSY |
 */
#define STATUS3_BUSY_MASK     		(1 << 0) /* Bit 0: Device ready/busy status  */
#define STATUS3_READY        		(0 << 0) /*   0 = Not Busy                   */
#define STATUS3_BUSY         		(1 << 0) /*   1 = Busy                       */
#define STATUS3_WEL_MASK     		(1 << 1) /* Bit 1: Write enable latch status */
#define STATUS3_WEL_DISABLED 		(0 << 1) /*   0 = Not Write Enabled          */
#define STATUS3_WEL_ENABLED  		(1 << 1) /*   1 = Write Enabled              */
#define STATUS3_E_FAIL_MASK 		(1 << 2) /* Bit 2: Erase fail flag           */
#define STATUS3_E_FAIL       		(0 << 2) /*   0 = No Erase Fail               */
#define STATUS3_E_FAIL_TRUE  		(1 << 2) /*   1 = Erase Fail                  */
#define STATUS3_P_FAIL_MASK 		(1 << 3) /* Bit 3: Program fail flag         */
#define STATUS3_P_FAIL       		(0 << 3) /*   0 = No Program Fail             */
#define STATUS3_P_FAIL_TRUE  		(1 << 3) /*   1 = Program Fail                */
#define STATUS3_ECC_MASK     		(3 << 4) /* Bits 4-5: ECC0-1 Status              */
#define STATUS3_ECC_NO      		(0 << 4) /*   00 = No ECC required, successful */
#define STATUS3_ECC_1      		(1 << 4) /*   01 = 1~4 bit/page ECC correction, successful      */
#define STATUS3_ECC_2      		(2 << 4) /*   10 = more than 4 bits errors, 1 page not repairable      */
#define STATUS3_ECC_UNCOR   		(3 << 4) /*   11 = data is not suitable to use    */
#define STATUS3_LUT_F_MASK 		(1 << 6) /* Bit 6: LUT (failure) Full flag      */
#define STATUS3_LUT_F       		(0 << 6) /*   0 = default, no full              */
#define STATUS3_LUT_F_TRUE  		(1 << 6) /*   1 = more memory block links may be established. */

/* Chip Geometries **********************************************************/
/* W25N01 (128 MB (1 Gb)) memory capacity */

#define W25N01GV_BLOCKS          1024     /* 1024 * 128KiB = 128MiB */
#define W25N01GV_PAGES_PER_BLK   64
#define W25N01GV_PAGE_SIZE       2048
#define W25N01GV_OOB_SIZE        64
#define W25N01GV_CACHE_READ_DUMMY 8       /* per datasheet for 6Bh (x4) */ /* TODO: confirm */

/* Cache flags **************************************************************/
// TODO: check these
#define W25N01GV_CACHE_VALID       (1 << 0)  /* 1=Cache has valid data */
#define W25N01GV_CACHE_DIRTY       (1 << 1)  /* 1=Cache is dirty */
#define W25N01GV_CACHE_ERASED      (1 << 2)  /* 1=Backing FLASH is erased */

#define IS_VALID(p)                 ((((p)->flags) & W25N01GV_CACHE_VALID) != 0)
#define IS_DIRTY(p)                 ((((p)->flags) & W25N01GV_CACHE_DIRTY) != 0)
#define IS_ERASED(p)                ((((p)->flags) & W25N01GV_CACHE_ERASED) != 0)

#define SET_VALID(p)                do { (p)->flags |= W25N01GV_CACHE_VALID; } while (0)
#define SET_DIRTY(p)                do { (p)->flags |= W25N01GV_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)               do { (p)->flags |= W25N01GV_CACHE_ERASED; } while (0)

#define CLR_VALID(p)                do { (p)->flags &= ~W25N01GV_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)                do { (p)->flags &= ~W25N01GV_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)               do { (p)->flags &= ~W25N01GV_CACHE_ERASED; } while (0)

/* 512 byte sector support **************************************************/

#define W25N01GV_SECTOR512_SHIFT     9
#define W25N01GV_SECTOR512_SIZE      (1 << 9)
#define W25N01GV_ERASED_STATE        0xff

/****************************************************************************
 * Private Types
 ****************************************************************************/
/**
 * Minimal W25Nxx geometry for W25N01 (1 Gbit)
 *  - 2 048B page + 64B spare; 64 pages per block (128 KiB blocks)
 *  - 1024 blocks total (approx 128 MiB)
 */
struct w25n01gv_geometry {
    uint32_t page_size_bytes;      // 2048
    uint32_t spare_size_bytes;     // 64
    uint32_t pages_per_block;      // 64
    uint32_t block_size_bytes;     // 131072 (64*2048)
    uint32_t num_blocks;           // 1024
};

/* This type represents the state of the MTD device. The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct w25n01gv_dev_s.
 */
// TODO check to replace.remove parameters not needed
struct w25n01gv_dev_s
{
  struct mtd_dev_s       mtd;         /* MTD interface */
  FAR struct qspi_dev_s *qspi;        /* Saved QuadSPI interface instance */
  uint16_t               nsectors;    /* Number of erase sectors */
  uint8_t                sectorshift; /* Log2 of sector size */
  uint8_t                pageshift;   /* Log2 of page size */
  uint8_t                addresslen;  /* Length of address 3 or 4 bytes */
  uint8_t                protectmask; /* Mask for protect bits in status register */
  uint8_t                tbmask;      /* Mask for top/bottom bit in status register */
  FAR uint8_t           *cmdbuf;      /* Allocated command buffer */
  FAR uint8_t           *readbuf;     /* Allocated status read buffer */

#ifdef CONFIG_W25N01GV_SECTOR512
  uint8_t                flags;       /* Buffered sector flags */
  uint16_t               esectno;     /* Erase sector number in the cache */
  FAR uint8_t           *sector;      /* Allocated sector data */
#endif
};

/* Functions from ChatGPT */
// TODO check
static int  w25n01gv_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t w25n01gv_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR uint8_t *buf);
static ssize_t w25n01gv_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR const uint8_t *buf);
static int  w25n01gv_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);
static int  w25n01gv_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes, FAR uint8_t *buffer);
static int  w25n01gv_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes, FAR const uint8_t *buffer);

/* --- QSPI helpers (sketch; adapt to your NuttX qspi API) --- */

static int qspi_send1(struct qspi_dev_s *q, uint8_t cmd)
{
  struct qspi_mem_s mem;
  memset(&mem, 0, sizeof(mem));
  mem.flags   = QSPIMEM_FCMD;
  mem.cmd     = cmd;
  return qspi_memory(q, &mem); /* or qspi_command() in older trees */
}

static int w25n01gv_getfeature(struct qspi_dev_s *q, uint8_t addr, uint8_t *val)
{
  struct qspi_mem_s m = {0};
  m.flags = QSPIMEM_FCMD | QSPIMEM_FADDR | QSPIMEM_FDATAIN;
  m.cmd   = W25N_CMD_GETFEATURE;
  m.addr  = addr;
  m.naddr = 1;
  m.buf   = val;
  m.nbytes= 1;
  return qspi_memory(q, &m);
}

static int w25n01gv_setfeature(struct qspi_dev_s *q, uint8_t addr, uint8_t val)
{
  struct qspi_mem_s m = {0};
  m.flags = QSPIMEM_FCMD | QSPIMEM_FADDR | QSPIMEM_FDATAOUT;
  m.cmd   = W25N_CMD_SETFEATURE;
  m.addr  = addr;
  m.naddr = 1;
  m.buf   = &val;
  m.nbytes= 1;
  return qspi_memory(q, &m);
}

static int w25n01gv_wait_ready(struct qspi_dev_s *q, uint32_t timeout_ms)
{
  /* poll status until OIP==0; check ECC bits for read/program result */
  uint8_t st;
  uint32_t t0 = clock_systime_ticks();
  do {
    if (w25n01gv_getfeature(q, W25N_FEAT_STAT, &st) < 0) return -EIO;
    if (!(st & W25N_STAT_OIP)) return 0;
  } while (clock_systime_elapsed(t0) < MSEC2TICK(timeout_ms));
  return -ETIMEDOUT;
}

/* --- Core NAND ops (minimal happy path) --- */

static int w25n01gv_page_to_cache(struct w25n01gv_dev_s *priv, uint32_t page)
{
  struct qspi_mem_s m = {0};
  m.flags = QSPIMEM_FCMD | QSPIMEM_FADDR;
  m.cmd   = W25N_CMD_PAGE2CACHE;
  m.addr  = page;
  m.naddr = 3;                  /* 24-bit row addr (page index) */
  int ret = qspi_memory(priv->qspi, &m);
  if (ret < 0) return ret;
  return w25n01gv_wait_ready(priv->qspi, 50);
}

static int w25n01gv_read_cache_x4(struct w25n01gv_dev_s *priv, uint16_t col, uint8_t *buf, uint32_t len)
{
  struct qspi_mem_s m = {0};
  m.flags   = QSPIMEM_FCMD | QSPIMEM_FADDR | QSPIMEM_FDUMMY | QSPIMEM_FDATAIN | QSPIMEM_FDATAWIDTH;
  m.cmd     = W25N_CMD_READCACHE_X4;
  m.addr    = col;
  m.naddr   = 2;                /* 2-byte column */
  m.ndummy  = W25N_CACHE_READ_DUMMY;
  m.buf     = buf;
  m.nbytes  = len;
  m.width   = QSPIWIDTH_4WIRE;  /* Quad data lines */
  return qspi_memory(priv->qspi, &m);
}

static int w25n01gv_prog_load_x4(struct w25n01gv_dev_s *priv, uint16_t col, const uint8_t *buf, uint32_t len)
{
  struct qspi_mem_s m = {0};
  m.flags   = QSPIMEM_FCMD | QSPIMEM_FADDR | QSPIMEM_FDATAOUT | QSPIMEM_FDATAWIDTH;
  m.cmd     = W25N_CMD_PROGLOAD_X4;
  m.addr    = col;
  m.naddr   = 2;
  m.buf     = (uint8_t*)buf;
  m.nbytes  = len;
  m.width   = QSPIWIDTH_4WIRE;
  return qspi_memory(priv->qspi, &m);
}

static int w25n01gv_prog_exec(struct w25n01gv_dev_s *priv, uint32_t page)
{
  struct qspi_mem_s m = {0};
  m.flags = QSPIMEM_FCMD | QSPIMEM_FADDR;
  m.cmd   = W25N_CMD_PROGEXEC;
  m.addr  = page;
  m.naddr = 3;
  int ret = qspi_memory(priv->qspi, &m);
  if (ret < 0) return ret;
  return w25n01gv_wait_ready(priv->qspi, 400);
}

static int w25n01gv_block_erase(struct w25n01gv_dev_s *priv, uint32_t page0_of_block)
{
  struct qspi_mem_s m = {0};
  m.flags = QSPIMEM_FCMD | QSPIMEM_FADDR;
  m.cmd   = W25N_CMD_BLKERASE;
  m.addr  = page0_of_block;
  m.naddr = 3;
  int ret = qspi_memory(priv->qspi, &m);
  if (ret < 0) return ret;
  return w25n01gv_wait_ready(priv->qspi, 1000);
}

/* --- MTD methods (very abbreviated) --- */

static int w25n01gv_erase(FAR struct mtd_dev_s *dev, off_t block, size_t nblocks)
{
  struct w25n01gv_dev_s *p = (struct w25n01gv_dev_s*)dev;
  while (nblocks--) {
    uint32_t page0 = (block++) * W25N_PAGES_PER_BLK;
    int ret = w25n01gv_block_erase(p, page0);
    if (ret < 0) return ret;
  }
  return OK;
}

static ssize_t w25n01gv_bread(FAR struct mtd_dev_s *dev, off_t block, size_t nblocks, FAR uint8_t *buf)
{
  struct w25n01gv_dev_s *p = (struct w25n01gv_dev_s*)dev;
  for (size_t i=0; i<nblocks; i++) {
    uint32_t page = (block+i) * W25N_PAGES_PER_BLK; /* FTL will pass “logical blocks”; map to pages as you design */
    int ret = w25n01gv_page_to_cache(p, page);
    if (ret < 0) return ret;
    /* read full block (or your page granularity) */
    ret = w25n01gv_read_cache_x4(p, 0, &buf[i*W25N_PAGE_SIZE], W25N_PAGE_SIZE);
    if (ret < 0) return ret;
    /* TODO: ECC status check via status feature */
  }
  return nblocks;
}

static ssize_t w25n01gv_bwrite(FAR struct mtd_dev_s *dev, off_t block, size_t nblocks, FAR const uint8_t *buf)
{
  struct w25n01gv_dev_s *p = (struct w25n01gv_dev_s*)dev;
  for (size_t i=0; i<nblocks; i++) {
    uint32_t page = (block+i) * W25N_PAGES_PER_BLK;
    int ret = w25n01gv_prog_load_x4(p, 0, &buf[i*W25N_PAGE_SIZE], W25N_PAGE_SIZE);
    if (ret < 0) return ret;
    ret = w25n01gv_prog_exec(p, page);
    if (ret < 0) return ret;
    /* TODO: check ECC/program status */
  }
  return nblocks;
}

static int w25n01gv_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  struct w25n01gv_dev_s *p = (struct w25n01gv_dev_s*)dev;
  switch (cmd) {
    case MTDIOC_GEOMETRY: {
      FAR struct mtd_geometry_s *g = (FAR struct mtd_geometry_s *)((uintptr_t)arg);
      if (!g) return -EINVAL;
      g->blocksize    = W25N_PAGE_SIZE;         /* or choose erase block for FTL layer */
      g->erasesize    = W25N_PAGES_PER_BLK * W25N_PAGE_SIZE;
      g->neraseblocks = W25N_BLOCKS;
      return OK;
    }
    /* implement MTDIOC_BULKERASE, etc., as needed */
  }
  return -ENOTTY;
}

static const struct mtd_ops_s g_ops = {
  .erase  = w25n01gv_erase,
  .bread  = w25n01gv_bread,
  .bwrite = w25n01gv_bwrite,
  .read   = w25n01gv_read,   /* optional raw read */
  .ioctl  = w25n01gv_ioctl,
};

FAR struct mtd_dev_s *w25nxx_initialize(FAR struct qspi_dev_s *qspi)
{
  struct w25n01gv_dev_s *p = kmm_zalloc(sizeof(*p));
  if (!p) return NULL;

  p->qspi   = qspi;
  p->pagebuf= kmm_malloc(W25N_PAGE_SIZE);
  p->oobbuf = kmm_malloc(W25N_OOB_SIZE);
  if (!p->pagebuf || !p->oobbuf) goto err;

  /* Reset, read ID, enable quad mode if required, configure timings */
  qspi_send1(qspi, W25N_CMD_RESET);
  w25n01gv_wait_ready(qspi, 5);

  /* TODO: read ID (0xEF AA 21 for W25N01GV), set CONF feature for QUAD if needed, clear protection */

  p->mtd.ops = &g_ops;
  return &p->mtd;
err:
  if (p->oobbuf) kmm_free(p->oobbuf);
  if (p->pagebuf) kmm_free(p->pagebuf);
  kmm_free(p);
  return NULL;
}
// Helpers
#define to_dev(dev) ((w25n01gv_dev*)(dev))

// Forward decls for mtd ops
static int w25n01gv_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t w25n01gv_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR uint8_t *buf);
static ssize_t w25n01gv_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR const uint8_t *buf);
static int w25n01gv_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);
static int w25n01gv_unlink(FAR struct mtd_dev_s *dev);

// Low-level SPI helpers
static inline void spi_select(struct w25n01gv_dev *priv, bool selected)
{
    /* PX4 SPI framework handles CS via bus/selected device; typically just ensures bus acquired */
    if (selected) { SPI_LOCK(priv->spi, true); } else { SPI_LOCK(priv->spi, false); }
}

static void send_cmd(struct w25n01gv_dev *priv, uint8_t cmd)
{
    SPI_SETFREQUENCY(priv->spi, priv->spi_freq_hz);
    SPI_SETMODE(priv->spi, SPIDEV_MODE0);
    SPI_SELECT(priv->spi, SPIDEV_FLASH, true);
    SPI_SNDBLOCK(priv->spi, &cmd, 1);
    SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
}

static int read_jedec_id(struct w25n01gv_dev *priv)
{
    uint8_t cmd = CMD_READ_ID;
    uint8_t id[3] = {};
    SPI_SETFREQUENCY(priv->spi, priv->spi_freq_hz);
    SPI_SETMODE(priv->spi, SPIDEV_MODE0);
    SPI_SELECT(priv->spi, SPIDEV_FLASH, true);
    SPI_SNDBLOCK(priv->spi, &cmd, 1);
    SPI_RECVBLOCK(priv->spi, id, sizeof(id));
    SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
    priv->manufacturer_id = id[0];
    priv->device_id = (uint16_t)(id[1] << 8 | id[2]);
    return 0;
}

static int hw_reset(struct w25n01gv_dev *priv)
{
    send_cmd(priv, CMD_RESET);
    up_mdelay(2);
    return 0;
}

// --- MTD ops implementations (skeleton) ---
static int w25n01gv_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
    auto *priv = to_dev(dev);
    // TODO: translate logical erase blocks to physical block addresses
    // Issue WRITE_ENABLE + BLOCK_ERASE for each block, check status/ECC
    PX4_INFO("erase: start=%ld blocks=%u", (long)startblock, (unsigned)nblocks);
    return 0; // return -errno on failure
}

static ssize_t w25n01gv_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR uint8_t *buf)
{
    auto *priv = to_dev(dev);
    // TODO: for each block/page: PAGE_READ to cache, then READ_CACHE data out
    // Check ECC status; if uncorrectable -> signal error & mark bad
    PX4_INFO("bread: start=%ld nblocks=%u", (long)startblock, (unsigned)nblocks);
    memset(buf, 0xFF, nblocks * 512); // placeholder
    return nblocks;
}

static ssize_t w25n01gv_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR const uint8_t *buf)
{
    auto *priv = to_dev(dev);
    // TODO: PROG_LOAD pages into cache then PROG_EXEC, manage page boundaries
    PX4_INFO("bwrite: start=%ld nblocks=%u", (long)startblock, (unsigned)nblocks);
    return nblocks;
}

static int w25n01gv_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
    auto *priv = to_dev(dev);
    switch (cmd) {
    case MTDIOC_GEOMETRY: {
        FAR struct mtd_geometry *geo = (FAR struct mtd_geometry *)arg;
        if (!geo) return -EINVAL;
        // Expose 512-byte logical blocks via FTL above us
        const uint32_t total_bytes = priv->geo.block_size_bytes * priv->geo.num_blocks;
        geo->blocksize = 512;
        geo->erasesize = priv->geo.block_size_bytes;
        geo->neraseblocks = priv->geo.num_blocks;
        return 0;
    }
    default:
        return -ENOTTY;
    }
}

static int w25n01gv_unlink(FAR struct mtd_dev_s *dev)
{
    // Optional: cleanup
    return 0;
}

extern "C" FAR struct mtd_dev_s *w25n01gv_initialize(FAR struct spi_dev_s *spi, uint32_t maxfreq)
{
    if (!spi) return nullptr;

    auto *priv = (w25n01gv_dev*)kmm_zalloc(sizeof(w25n01gv_dev));
    if (!priv) return nullptr;

    priv->spi = spi;
    priv->spi_freq_hz = maxfreq ? maxfreq : 24000000; // conservative default

    hw_reset(priv);
    read_jedec_id(priv);
    PX4_INFO("W25Nxx JEDEC: mfr=0x%02X dev=0x%04X", priv->manufacturer_id, priv->device_id);

    // TODO: verify ID matches W25N01/02; set geometry accordingly
    priv->geo = {2048, 64, 64, 131072, 1024};

    // Bind MTD ops
    priv->mtd.erase  = w25n01gv_erase;
    priv->mtd.bread  = w25n01gv_bread;
    priv->mtd.bwrite = w25n01gv_bwrite;
    priv->mtd.ioctl  = w25n01gv_ioctl;
    priv->mtd.unlink = w25n01gv_unlink;

    return &priv->mtd;
}
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Locking */

static void w25n01gv_lock(FAR struct qspi_dev_s *qspi);
static inline void w25n01gv_unlock(FAR struct qspi_dev_s *qspi);

/* Low-level message helpers */

static int  w25n01gv_command(FAR struct qspi_dev_s *qspi, uint8_t cmd);
static int  w25n01gv_command_address(FAR struct qspi_dev_s *qspi,
                                      uint8_t cmd,
                                      off_t addr,
                                      uint8_t addrlen);
static int  w25n01gv_command_read(FAR struct qspi_dev_s *qspi,
                                   uint8_t cmd,
                                   FAR void *buffer,
                                   size_t buflen);
static int  w25n01gv_command_write(FAR struct qspi_dev_s *qspi,
                                    uint8_t cmd,
                                    FAR const void *buffer,
                                    size_t buflen);
static uint8_t w25n01gv_read_status(FAR struct w25n01gv_dev_s *priv);
static void w25n01gv_write_status(FAR struct w25n01gv_dev_s *priv);
#if 0
static uint8_t w25n01gv_read_volcfg(FAR struct w25n01gv_dev_s *priv);
static void w25n01gv_write_volcfg(FAR struct w25n01gv_dev_s *priv);
#endif
static void w25n01gv_write_enable(FAR struct w25n01gv_dev_s *priv);
static void w25n01gv_write_disable(FAR struct w25n01gv_dev_s *priv);
static void w25n01gv_quad_enable(FAR struct w25n01gv_dev_s *priv);

static int  w25n01gv_readid(FAR struct w25n01gv_dev_s *priv);
static int  w25n01gv_protect(FAR struct w25n01gv_dev_s *priv,
              off_t startblock, size_t nblocks);
static int  w25n01gv_unprotect(FAR struct w25n01gv_dev_s *priv,
              off_t startblock, size_t nblocks);
static bool w25n01gv_isprotected(FAR struct w25n01gv_dev_s *priv,
              uint8_t status, off_t address);
static int  w25n01gv_erase_sector(FAR struct w25n01gv_dev_s *priv,
                                   off_t offset);
static int  w25n01gv_erase_chip(FAR struct w25n01gv_dev_s *priv);
static int  w25n01gv_read_byte(FAR struct w25n01gv_dev_s *priv,
                                FAR uint8_t *buffer,
                                off_t address,
                                size_t nbytes);
static int  w25n01gv_write_page(FAR struct w25n01gv_dev_s *priv,
                                 FAR const uint8_t *buffer,
                                 off_t address,
                                 size_t nbytes);
#ifdef CONFIG_W25N01GV_SECTOR512
static int  w25n01gv_flush_cache(struct w25n01gv_dev_s *priv);
static FAR uint8_t *w25n01gv_read_cache(struct w25n01gv_dev_s *priv,
                                         off_t sector);
static void w25n01gv_erase_cache(struct w25n01gv_dev_s *priv,
                                  off_t sector);
static int  w25n01gv_write_cache(FAR struct w25n01gv_dev_s *priv,
                                  FAR const uint8_t *buffer,
                                  off_t sector,
                                  size_t nsectors);
#endif

/* MTD driver methods */

static int  w25n01gv_erase(FAR struct mtd_dev_s *dev,
                            off_t startblock,
                            size_t nblocks);
static ssize_t w25n01gv_bread(FAR struct mtd_dev_s *dev,
                               off_t startblock,
                               size_t nblocks,
                               FAR uint8_t *buf);
static ssize_t w25n01gv_bwrite(FAR struct mtd_dev_s *dev,
                                off_t startblock,
                                size_t nblocks,
                                FAR const uint8_t *buf);
static ssize_t w25n01gv_read(FAR struct mtd_dev_s *dev,
                              off_t offset,
                              size_t nbytes,
                              FAR uint8_t *buffer);
static int  w25n01gv_ioctl(FAR struct mtd_dev_s *dev,
                            int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: w25n01gv_lock
 ****************************************************************************/

static void w25n01gv_lock(FAR struct qspi_dev_s *qspi)
{
  /* On QuadSPI buses where there are multiple devices, it will be necessary
   * to lock QuadSPI to have exclusive access to the buses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive
   * access to the QuadSPI bus.  We will retain that exclusive access until
   * the bus is unlocked.
   */

  (void)QSPI_LOCK(qspi, true);

  /* After locking the QuadSPI bus, the we also need call the setfrequency,
   * setbits, and setmode methods to make sure that the QuadSPI is properly
   * configured for the device. If the QuadSPI bus is being shared, then it
   * may have been left in an incompatible state.
   */

  QSPI_SETMODE(qspi, CONFIG_W25N01GV_QSPIMODE);
  QSPI_SETBITS(qspi, 8);
  (void)QSPI_SETFREQUENCY(qspi, CONFIG_W25N01GV_QSPI_FREQUENCY);
}

/****************************************************************************
 * Name: w25n01gv_unlock
 ****************************************************************************/

static inline void w25n01gv_unlock(FAR struct qspi_dev_s *qspi)
{
  (void)QSPI_LOCK(qspi, false);
}

/****************************************************************************
 * Name: w25n01gv_command
 ****************************************************************************/

static int w25n01gv_command(FAR struct qspi_dev_s *qspi, uint8_t cmd)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x\n", cmd);

  cmdinfo.flags   = 0;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: w25n01gv_command_address
 ****************************************************************************/

static int w25n01gv_command_address(FAR struct qspi_dev_s *qspi,
                                     uint8_t cmd,
                                     off_t addr,
                                     uint8_t addrlen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x Address: %04lx addrlen=%d\n",
         cmd,
         (unsigned long)addr,
          addrlen);

  cmdinfo.flags   = QSPICMD_ADDRESS;
  cmdinfo.addrlen = addrlen;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = addr;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: w25n01gv_command_read
 ****************************************************************************/

static int w25n01gv_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                  FAR void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_READDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: w25n01gv_command_write
 ****************************************************************************/

static int w25n01gv_command_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                   FAR const void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_WRITEDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = (FAR void *)buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: w25n01gv_read_status
 ****************************************************************************/

static uint8_t w25n01gv_read_status(FAR struct w25n01gv_dev_s *priv)
{
  DEBUGVERIFY(w25n01gv_command_read(priv->qspi, W25N01GV_READ_STATUS_1,
                                     (FAR void *)&priv->readbuf[0], 1));
  return priv->readbuf[0];
}

/****************************************************************************
 * Name:  w25n01gv_write_status
 ****************************************************************************/

static void w25n01gv_write_status(FAR struct w25n01gv_dev_s *priv)
{
  w25n01gv_write_enable(priv);

  /* Keep in Software Protection */

  priv->cmdbuf[0] &= ~STATUS_SRP_MASK;

  w25n01gv_command_write(priv->qspi, W25N01GV_WRITE_STATUS_1,
                       (FAR const void *)priv->cmdbuf, 1);
  w25n01gv_write_disable(priv);
}

/****************************************************************************
 * Name:  w25n01gv_write_enable
 ****************************************************************************/

static void w25n01gv_write_enable(FAR struct w25n01gv_dev_s *priv)
{
  uint8_t status;

  do
    {
      w25n01gv_command(priv->qspi, W25N01GV_WRITE_ENABLE);
      status = w25n01gv_read_status(priv);
    }
  while ((status & STATUS_WEL_MASK) != STATUS_WEL_ENABLED);
}

/****************************************************************************
 * Name:  w25n01gv_write_disable
 ****************************************************************************/

static void w25n01gv_write_disable(FAR struct w25n01gv_dev_s *priv)
{
  uint8_t status;

  do
    {
      w25n01gv_command(priv->qspi, W25N01GV_WRITE_DISABLE);
      status = w25n01gv_read_status(priv);
    }
  while ((status & STATUS_WEL_MASK) != STATUS_WEL_DISABLED);
}

/****************************************************************************
 * Name:  w25n01gv_quad_enable
 ****************************************************************************/

static void w25n01gv_quad_enable(FAR struct w25n01gv_dev_s *priv)
{
  w25n01gv_command_read(priv->qspi, W25N01GV_READ_STATUS_2,
                         (FAR void *)priv->cmdbuf, 1);

  if ((priv->cmdbuf[0] & STATUS2_QE_MASK) != STATUS2_QE_ENABLED)
    {
      w25n01gv_write_enable(priv);

      priv->cmdbuf[0] &= ~STATUS2_QE_MASK;
      priv->cmdbuf[1] |= STATUS2_QE_ENABLED;

      w25n01gv_command_write(priv->qspi, W25N01GV_WRITE_STATUS_2,
                              (FAR const void *)priv->cmdbuf, 1);

      w25n01gv_write_disable(priv);
    }
}

/****************************************************************************
 * Name: w25n01gv_readid
 ****************************************************************************/

static inline int w25n01gv_readid(struct w25n01gv_dev_s *priv)
{
  /* Lock the QuadSPI bus and configure the bus. */

  w25n01gv_lock(priv->qspi);

  /* Read the JEDEC ID */

  w25n01gv_command_read(priv->qspi, W25N01GV_JEDEC_ID, priv->cmdbuf, 3);

  /* Unlock the bus */

  w25n01gv_unlock(priv->qspi);

  finfo("Manufacturer: %02x Device Type %02x, Capacity: %02x\n",
        priv->cmdbuf[0], priv->cmdbuf[1], priv->cmdbuf[2]);

  /* Check for a recognized memory device type */

  if (priv->cmdbuf[1] != W25N01GVQ_JEDEC_DEVICE_TYPE &&
      priv->cmdbuf[1] != W25N01GVM_JEDEC_DEVICE_TYPE)
    {
      ferr("ERROR: Unrecognized device type: 0x%02x\n", priv->cmdbuf[1]);
      return -ENODEV;
    }

  /* Check for a supported capacity */

  switch (priv->cmdbuf[2])
    {
      case W25Q016_JEDEC_CAPACITY:
        priv->sectorshift = W25Q016_SECTOR_SHIFT;
        priv->pageshift   = W25Q016_PAGE_SHIFT;
        priv->nsectors    = W25Q016_SECTOR_COUNT;
        priv->addresslen  = 3;
        priv->protectmask = STATUS_BP_MASK;
        priv->tbmask      = STATUS_TB_MASK;
        break;

      case W25Q032_JEDEC_CAPACITY:
        priv->sectorshift = W25Q032_SECTOR_SHIFT;
        priv->pageshift   = W25Q032_PAGE_SHIFT;
        priv->nsectors    = W25Q032_SECTOR_COUNT;
        priv->addresslen  = 3;
        priv->protectmask = STATUS_BP_MASK;
        priv->tbmask      = STATUS_TB_MASK;
        break;

      case W25Q064_JEDEC_CAPACITY:
        priv->sectorshift = W25Q064_SECTOR_SHIFT;
        priv->pageshift   = W25Q064_PAGE_SHIFT;
        priv->nsectors    = W25Q064_SECTOR_COUNT;
        priv->addresslen  = 3;
        priv->protectmask = STATUS_BP_4_MASK;
        priv->tbmask      = STATUS_TB_6_MASK;
        break;

      case W25Q128_JEDEC_CAPACITY:
        priv->sectorshift = W25Q128_SECTOR_SHIFT;
        priv->pageshift   = W25Q128_PAGE_SHIFT;
        priv->nsectors    = W25Q128_SECTOR_COUNT;
        priv->addresslen  = 3;
        priv->protectmask = STATUS_BP_MASK;
        priv->tbmask      = STATUS_TB_MASK;
        break;

      case W25Q256_JEDEC_CAPACITY:
        priv->sectorshift = W25Q256_SECTOR_SHIFT;
        priv->pageshift   = W25Q256_PAGE_SHIFT;
        priv->nsectors    = W25Q256_SECTOR_COUNT;
        priv->addresslen  = 4;
        priv->protectmask = STATUS_BP_4_MASK;
        priv->tbmask      = STATUS_TB_6_MASK;
        break;

      case W25Q512_JEDEC_CAPACITY:
        priv->sectorshift = W25Q512_SECTOR_SHIFT;
        priv->pageshift   = W25Q512_PAGE_SHIFT;
        priv->nsectors    = W25Q512_SECTOR_COUNT;
        priv->addresslen  = 4;
        priv->protectmask = STATUS_BP_4_MASK;
        priv->tbmask      = STATUS_TB_6_MASK;
        break;

      case W25Q01_JEDEC_CAPACITY:
        priv->sectorshift = W25Q01_SECTOR_SHIFT;
        priv->pageshift   = W25Q01_PAGE_SHIFT;
        priv->nsectors    = W25Q01_SECTOR_COUNT;
        priv->addresslen  = 4;
        priv->protectmask = STATUS_BP_4_MASK;
        priv->tbmask      = STATUS_TB_6_MASK;
        break;

      /* Support for this part is not implemented yet */

      default:
        ferr("ERROR: Unsupported memory capacity: %02x\n", priv->cmdbuf[2]);
        return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: w25n01gv_protect
 ****************************************************************************/

static int w25n01gv_protect(FAR struct w25n01gv_dev_s *priv,
                             off_t startblock, size_t nblocks)
{
  /* Get the status register value to check the current protection */

  priv->cmdbuf[0] = w25n01gv_read_status(priv);

  if ((priv->cmdbuf[0] & priv->protectmask) ==
                           (STATUS_BP_ALL & priv->protectmask))
    {
      /* Protection already enabled */

      return 0;
    }

  /* set the BP bits as necessary to protect the range of sectors. */

  priv->cmdbuf[0] |= (STATUS_BP_ALL & priv->protectmask);
  w25n01gv_write_status(priv);

  /* Check the new status */

  priv->cmdbuf[0] = w25n01gv_read_status(priv);
  if ((priv->cmdbuf[0] & priv->protectmask) !=
                            (STATUS_BP_ALL & priv->protectmask))
    {
      return -EACCES;
    }

  return OK;
}

/****************************************************************************
 * Name: w25n01gv_unprotect
 ****************************************************************************/

static int w25n01gv_unprotect(FAR struct w25n01gv_dev_s *priv,
                               off_t startblock, size_t nblocks)
{
  /* Get the status register value to check the current protection */

  priv->cmdbuf[0] = w25n01gv_read_status(priv);

  if ((priv->cmdbuf[0] & priv->protectmask) == STATUS_BP_NONE)
    {
      /* Protection already disabled */

      return 0;
    }

  /* Set the protection mask to zero (and not complemented).
   * REVISIT:  This logic should really just re-write the BP bits as
   * necessary to unprotect the range of sectors.
   */

  priv->cmdbuf[0] &= ~priv->protectmask;
  w25n01gv_write_status(priv);

  /* Check the new status */

  priv->cmdbuf[0] = w25n01gv_read_status(priv);
  if ((priv->cmdbuf[0] & (STATUS_SRP_MASK | priv->protectmask)) != 0)
    {
      return -EACCES;
    }

  return OK;
}

/****************************************************************************
 * Name: w25n01gv_isprotected
 ****************************************************************************/

static bool w25n01gv_isprotected(FAR struct w25n01gv_dev_s *priv,
                                  uint8_t status,
                                  off_t address)
{
  off_t protstart;
  off_t protend;
  off_t protsize;
  unsigned int bp;

  /* The BP field is spread across non-contiguous bits */

  bp = (status & priv->protectmask) >> STATUS_BP_SHIFT;

  /* the BP field is essentially the power-of-two of the number of 64k
   * sectors, saturated to the device size.
   */

  if (0 == bp)
    {
      return false;
    }

  protsize = 0x00010000;
  protsize <<= (protsize << (bp - 1));
  protend = (1 << priv->sectorshift) * priv->nsectors;
  if (protsize > protend)
    {
      protsize = protend;
    }

  /* The final protection range then depends on if the protection region is
   * configured top-down or bottom up  (assuming CMP=0).
   */

  if ((status & priv->tbmask) != 0)
    {
      protstart = 0x00000000;
      protend   = protstart + protsize;
    }
  else
    {
      protstart = protend - protsize;

      /* protend already computed above */
    }

  return (address >= protstart && address < protend);
}

/****************************************************************************
 * Name:  w25n01gv_erase_sector
 ****************************************************************************/

static int w25n01gv_erase_sector(FAR struct w25n01gv_dev_s *priv,
                                  off_t sector)
{
  off_t address;
  uint8_t status;

  finfo("sector: %08lx\n", (unsigned long)sector);

  /* Check that the flash is ready and unprotected */

  status = w25n01gv_read_status(priv);
  if ((status & STATUS_BUSY_MASK) != STATUS_READY)
    {
      ferr("ERROR: Flash busy: %02x", status);
      return -EBUSY;
    }

  /* Get the address associated with the sector */

  address = (off_t)sector << priv->sectorshift;

  if ((status & priv->protectmask) != 0 &&
       w25n01gv_isprotected(priv, status, address))
    {
      ferr("ERROR: Flash protected: %02x", status);
      return -EACCES;
    }

  /* Send the sector erase command */

  w25n01gv_write_enable(priv);
  w25n01gv_command_address(priv->qspi,
                            W25N01GV_SECTOR_ERASE,
                            address, priv->addresslen);

  /* Wait for erasure to finish */

  while ((w25n01gv_read_status(priv) & STATUS_BUSY_MASK) != 0);

  return OK;
}

/****************************************************************************
 * Name:  w25n01gv_erase_chip
 ****************************************************************************/

static int w25n01gv_erase_chip(FAR struct w25n01gv_dev_s *priv)
{
  uint8_t status;

  /* Check if the FLASH is protected */

  status = w25n01gv_read_status(priv);
  if ((status & priv->protectmask) != 0)
    {
      ferr("ERROR: FLASH is Protected: %02x", status);
      return -EACCES;
    }

  /* Erase the whole chip */

  w25n01gv_write_enable(priv);
  w25n01gv_command(priv->qspi, W25N01GV_CHIP_ERASE);

  /* Wait for the erasure to complete */

  status = w25n01gv_read_status(priv);
  while ((status & STATUS_BUSY_MASK) != 0)
    {
      nxsig_usleep(200  *1000);
      status = w25n01gv_read_status(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: w25n01gv_read_byte
 ****************************************************************************/

static int w25n01gv_read_byte(FAR struct w25n01gv_dev_s *priv,
                               FAR uint8_t *buffer,
                               off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;

  finfo("address: %08lx nbytes: %d\n", (long)address, (int)buflen);

  meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
  meminfo.addrlen = priv->addresslen;
  meminfo.dummies = CONFIG_W25N01GV_DUMMIES;
  meminfo.buflen  = buflen;
  meminfo.cmd     = W25N01GV_FAST_READ_QUADIO;
  meminfo.addr    = address;
  meminfo.buffer  = buffer;

  return QSPI_MEMORY(priv->qspi, &meminfo);
}

/****************************************************************************
 * Name:  w25n01gv_write_page
 ****************************************************************************/

static int w25n01gv_write_page(struct w25n01gv_dev_s *priv,
                                FAR const uint8_t *buffer,
                                off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;
  unsigned int pagesize;
  unsigned int npages;
  int ret;
  int i;

  finfo("address: %08lx buflen: %u\n",
        (unsigned long)address,
        (unsigned)buflen);

  npages   = (buflen >> priv->pageshift);
  pagesize = (1 << priv->pageshift);

  /* Set up non-varying parts of transfer description */

  meminfo.flags   = QSPIMEM_WRITE;
  meminfo.cmd     = W25N01GV_PAGE_PROGRAM;
  meminfo.addrlen = priv->addresslen;
  meminfo.buflen  = pagesize;
  meminfo.dummies = 0;

  /* Then write each page */

  for (i = 0; i < npages; i++)
    {
      /* Set up varying parts of the transfer description */

      meminfo.addr   = address;
      meminfo.buffer = (void *)buffer;

      /* Write one page */

      w25n01gv_write_enable(priv);
      ret = QSPI_MEMORY(priv->qspi, &meminfo);
      w25n01gv_write_disable(priv);

      if (ret < 0)
        {
          ferr("ERROR: QSPI_MEMORY failed writing address=%06"PRIxOFF"\n",
               address);
          return ret;
        }

      /* Update for the next time through the loop */

      buffer  += pagesize;
      address += pagesize;
      buflen  -= pagesize;
    }

  /* The transfer should always be an even number of sectors and hence also
   * pages.  There should be no remainder.
   */

  DEBUGASSERT(buflen == 0);

  return OK;
}

/****************************************************************************
 * Name: w25n01gv_flush_cache
 ****************************************************************************/

#ifdef CONFIG_W25N01GV_SECTOR512
static int w25n01gv_flush_cache(struct w25n01gv_dev_s *priv)
{
  int ret = OK;

  /* If the cache is dirty (meaning that it no longer matches the old FLASH
   * contents) or was erased (with the cache containing the correct FLASH
   * contents), then write the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      off_t address;

      /* Convert the erase sector number into a FLASH address */

      address = (off_t)priv->esectno << priv->sectorshift;

      /* Write entire erase block to FLASH */

      ret = w25n01gv_write_page(priv,
                                 priv->sector,
                                 address,
                                 1 << priv->sectorshift);
      if (ret < 0)
        {
          ferr("ERROR: w25n01gv_write_page failed: %d\n", ret);
        }

      /* The cache is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: w25n01gv_read_cache
 ****************************************************************************/

#ifdef CONFIG_W25N01GV_SECTOR512
static FAR uint8_t *w25n01gv_read_cache(struct w25n01gv_dev_s *priv,
                                         off_t sector)
{
  off_t esectno;
  int   shift;
  int   index;
  int   ret;

  /* Convert from the 512 byte sector to the erase sector size of the device.
   * For example, if the actual erase sector size is 4Kb (1 << 12), then we
   * first shift to the right by 3 to get the sector number in 4096
   * increments.
   */

  shift    = priv->sectorshift - W25N01GV_SECTOR512_SHIFT;
  esectno  = sector >> shift;
  finfo("sector: %ld esectno: %d shift=%d\n", sector, esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      ret = w25n01gv_flush_cache(priv);
      if (ret < 0)
        {
          ferr("ERROR: w25n01gv_flush_cache failed: %d\n", ret);
          return NULL;
        }

      /* Read the erase block into the cache */

      ret = w25n01gv_read_byte(priv, priv->sector,
                             (esectno << priv->sectorshift),
                             (1 << priv->sectorshift));
      if (ret < 0)
        {
          ferr("ERROR: w25n01gv_read_byte failed: %d\n", ret);
          return NULL;
        }

      /* Mark the sector as cached */

      priv->esectno = esectno;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

  /* Get the index to the 512 sector in the erase block that holds the
   * argument
   */

  index = sector & ((1 << shift) - 1);

  /* Return the address in the cache that holds this sector */

  return &priv->sector[index << W25N01GV_SECTOR512_SHIFT];
}
#endif

/****************************************************************************
 * Name: w25n01gv_erase_cache
 ****************************************************************************/

#ifdef CONFIG_W25N01GV_SECTOR512
static void w25n01gv_erase_cache(struct w25n01gv_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is
   * in the cache.
   */

  dest = w25n01gv_read_cache(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase
   * sector is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >>
                      (priv->sectorshift - W25N01GV_SECTOR512_SHIFT);
      finfo("sector: %ld esectno: %d\n", sector, esectno);

      DEBUGVERIFY(w25n01gv_erase_sector(priv, esectno));
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mark the cache as
   * dirty(but don't update the FLASH yet.  The caller will do that at a
   * more optimal time).
   */

  memset(dest, W25N01GV_ERASED_STATE, W25N01GV_SECTOR512_SIZE);
  SET_DIRTY(priv);
}
#endif

/****************************************************************************
 * Name: w25n01gv_write_cache
 ****************************************************************************/

#ifdef CONFIG_W25N01GV_SECTOR512
static int w25n01gv_write_cache(FAR struct w25n01gv_dev_s *priv,
                                 FAR const uint8_t *buffer, off_t sector,
                                 size_t nsectors)
{
  FAR uint8_t *dest;
  int ret;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector is
       * in memory.
       */

      dest = w25n01gv_read_cache(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase
       * sector is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  = sector >>
                           (priv->sectorshift - W25N01GV_SECTOR512_SHIFT);
          finfo("sector: %ld esectno: %d\n", sector, esectno);

          ret = w25n01gv_erase_sector(priv, esectno);
          if (ret < 0)
            {
              ferr("ERROR: w25n01gv_erase_sector failed: %d\n", ret);
              return ret;
            }

          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, W25N01GV_SECTOR512_SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += W25N01GV_SECTOR512_SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  return w25n01gv_flush_cache(priv);
}
#endif

/****************************************************************************
 * Name: w25n01gv_erase
 ****************************************************************************/

static int w25n01gv_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks)
{
  FAR struct w25n01gv_dev_s *priv = (FAR struct w25n01gv_dev_s *)dev;
  size_t blocksleft = nblocks;
#ifdef CONFIG_W25N01GV_SECTOR512
  int ret;
#endif

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  w25n01gv_lock(priv->qspi);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

#ifdef CONFIG_W25N01GV_SECTOR512
      w25n01gv_erase_cache(priv, startblock);
#else
      w25n01gv_erase_sector(priv, startblock);
#endif
      startblock++;
    }

#ifdef CONFIG_W25N01GV_SECTOR512
  /* Flush the last erase block left in the cache */

  ret = w25n01gv_flush_cache(priv);
  if (ret < 0)
    {
      nblocks = ret;
    }
#endif

  w25n01gv_unlock(priv->qspi);

  return (int)nblocks;
}

/****************************************************************************
 * Name: w25n01gv_bread
 ****************************************************************************/

static ssize_t w25n01gv_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, FAR uint8_t *buffer)
{
#ifndef CONFIG_W25N01GV_SECTOR512
  FAR struct w25n01gv_dev_s *priv = (FAR struct w25n01gv_dev_s *)dev;
#endif
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

#ifdef CONFIG_W25N01GV_SECTOR512
  nbytes = w25n01gv_read(dev, startblock << W25N01GV_SECTOR512_SHIFT,
                       nblocks << W25N01GV_SECTOR512_SHIFT, buffer);
  if (nbytes > 0)
    {
      nbytes >>= W25N01GV_SECTOR512_SHIFT;
    }
#else
  nbytes = w25n01gv_read(dev, startblock << priv->pageshift,
                       nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }
#endif

  return nbytes;
}

/****************************************************************************
 * Name: w25n01gv_bwrite
 ****************************************************************************/

static ssize_t w25n01gv_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                                size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct w25n01gv_dev_s *priv = (FAR struct w25n01gv_dev_s *)dev;
  int ret = (int)nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the QuadSPI bus and write all of the pages to FLASH */

  w25n01gv_lock(priv->qspi);

#if defined(CONFIG_W25N01GV_SECTOR512)
  ret = w25n01gv_write_cache(priv, buffer, startblock, nblocks);
  if (ret < 0)
    {
      ferr("ERROR: w25n01gv_write_cache failed: %d\n", ret);
    }

#else
  ret = w25n01gv_write_page(priv, buffer, startblock << priv->pageshift,
                          nblocks << priv->pageshift);
  if (ret < 0)
    {
      ferr("ERROR: w25n01gv_write_page failed: %d\n", ret);
    }
#endif

  w25n01gv_unlock(priv->qspi);

  return ret < 0 ? ret : nblocks;
}

/****************************************************************************
 * Name: w25n01gv_read
 ****************************************************************************/

static ssize_t w25n01gv_read(FAR struct mtd_dev_s *dev,
                              off_t offset,
                              size_t nbytes,
                              FAR uint8_t *buffer)
{
  FAR struct w25n01gv_dev_s *priv = (FAR struct w25n01gv_dev_s *)dev;
  int ret;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the QuadSPI bus and select this FLASH part */

  w25n01gv_lock(priv->qspi);
  ret = w25n01gv_read_byte(priv, buffer, offset, nbytes);
  w25n01gv_unlock(priv->qspi);

  if (ret < 0)
    {
      ferr("ERROR: w25n01gv_read_byte returned: %d\n", ret);
      return (ssize_t)ret;
    }

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

/****************************************************************************
 * Name: w25n01gv_ioctl
 ****************************************************************************/

static int w25n01gv_ioctl(FAR struct mtd_dev_s *dev,
                           int cmd,
                           unsigned long arg)
{
  FAR struct w25n01gv_dev_s *priv = (FAR struct w25n01gv_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the
               * client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

#ifdef CONFIG_W25N01GV_SECTOR512
              geo->blocksize    = (1 << W25N01GV_SECTOR512_SHIFT);
              geo->erasesize    = (1 << W25N01GV_SECTOR512_SHIFT);
              geo->neraseblocks = priv->nsectors <<
                                  (priv->sectorshift -
                                   W25N01GV_SECTOR512_SHIFT);
#else
              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;
#endif
              ret               = OK;

              finfo("blocksize: %lu erasesize: %lu neraseblocks: %lu\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)arg;
          if (info != NULL)
            {
#ifdef CONFIG_W25N01GV_SECTOR512
              info->numsectors  = priv->nsectors <<
                             (priv->sectorshift - W25N01GV_SECTOR512_SHIFT);
              info->sectorsize  = 1 << W25N01GV_SECTOR512_SHIFT;
#else
              info->numsectors  = priv->nsectors <<
                                  (priv->sectorshift - priv->pageshift);
              info->sectorsize  = 1 << priv->pageshift;
#endif
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          w25n01gv_lock(priv->qspi);
          ret = w25n01gv_erase_chip(priv);
          w25n01gv_unlock(priv->qspi);
        }
        break;

      case MTDIOC_PROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          ret = w25n01gv_protect(priv, prot->startblock, prot->nblocks);
        }
        break;

      case MTDIOC_UNPROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          ret = w25n01gv_unprotect(priv, prot->startblock, prot->nblocks);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = W25N01GV_ERASED_STATE;

          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY; /* Bad/unsupported command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: w25n01gv_initialize
 *
 * Description:
 *   Create an initialize MTD device instance for the QuadSPI-based W25N01GV
 *   FLASH part.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 ****************************************************************************/
/* See nand.h for
 * FAR struct mtd_dev_s *nand_initialize(FAR struct nand_raw_s *raw);

 * ChatGPT suggests argument: spi_dev_s *spi and uint32_t maxfreq
 */

FAR struct mtd_dev_s *w25n01gv_initialize(FAR struct qspi_dev_s *qspi,
                                           bool unprotect)
{
  FAR struct w25n01gv_dev_s *priv;
  int ret;

  finfo("qspi: %p\n", qspi);
  DEBUGASSERT(qspi != NULL);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same QuadSPI
   * bus.
   */

  priv = (FAR struct w25n01gv_dev_s *)
         kmm_zalloc(sizeof(struct w25n01gv_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = w25n01gv_erase;
      priv->mtd.bread  = w25n01gv_bread;
      priv->mtd.bwrite = w25n01gv_bwrite;
      priv->mtd.read   = w25n01gv_read;
      priv->mtd.ioctl  = w25n01gv_ioctl;
      priv->mtd.name   = "w25n01gv";
      priv->qspi       = qspi;

      /* Allocate a 4-byte buffer to support DMA-able command data */

      priv->cmdbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 4);
      if (priv->cmdbuf == NULL)
        {
          ferr("ERROR Failed to allocate command buffer\n");
          goto errout_with_priv;
        }

      /* Allocate a one-byte buffer to support DMA-able status read data */

      priv->readbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 1);
      if (priv->readbuf == NULL)
        {
          ferr("ERROR Failed to allocate read buffer\n");
          goto errout_with_cmdbuf;
        }

      /* Identify the FLASH chip and get its capacity */

      ret = w25n01gv_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR Unrecognized QSPI device\n");
          goto errout_with_readbuf;
        }

      /* Enter 4-byte address mode if chip is 4-byte addressable */

      if (priv->addresslen == 4)
        {
          w25n01gv_lock(priv->qspi);
          ret = w25n01gv_command(priv->qspi, W25N01GV_ENTER_4BT_MODE);
          if (ret != OK)
            {
              ferr("ERROR: Failed to enter 4 byte mode\n");
            }

          w25n01gv_unlock(priv->qspi);
        }

      /* Unprotect FLASH sectors if so requested. */

      if (unprotect)
        {
          ret = w25n01gv_unprotect(priv, 0, priv->nsectors - 1);
          if (ret < 0)
            {
              ferr("ERROR: Sector unprotect failed\n");
            }
        }

      /* Enable Quad SPI mode, if not already enabled. */

      w25n01gv_quad_enable(priv);

#ifdef CONFIG_W25N01GV_SECTOR512  /* Simulate a 512 byte sector */
      /* Allocate a buffer for the erase block cache */

      priv->sector = (FAR uint8_t *)QSPI_ALLOC(qspi, 1 << priv->sectorshift);
      if (priv->sector == NULL)
        {
          /* Allocation failed! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR: Sector allocation failed\n");
          goto errout_with_readbuf;
        }
#endif
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;

errout_with_readbuf:
  QSPI_FREE(qspi, priv->readbuf);

errout_with_cmdbuf:
  QSPI_FREE(qspi, priv->cmdbuf);

errout_with_priv:
  kmm_free(priv);
  return NULL;
}
