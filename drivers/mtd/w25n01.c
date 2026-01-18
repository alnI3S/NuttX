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

/* Driver for SPI-based W25n01 1Gb (128MB) NAND
 * FLASH from Winbond
 *
 * This driver supports only Standard SPI mode (104 MHz) because I write it for
 * the Kakute H7 Mini v1.3 board, which connects the W25N01GVEIG to the
 * STM32H743VIH6's SPI1 peripheral that does not support Dual/Quad SPI mode.
 *
 * Features:
 * - Standard SPI mode (up to 104 MHz)
 * - Bad Block Management (swap bad blocks with reserved good blocks)
 * - support buffer/continuous read
 * - HW ECC enabled (default)
 *
 * For futur use with other boards, Dual/Quad SPI mode could be added.
 */

// TODO: remove confusing terms sector/block
// seems that fs.h: partition_info_s sector is w25n01 page

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SPI bus operation Mode 0 (0,0) and 3 (1,1) are supported. The primary
 * difference between Mode 0 and Mode 3 concerns the normal state of the CLK
 * signal when the SPI bus master is in standby and data is not being
 * transferred to the Serial Flash. For Mode 0, the CLK signal is normally low
 * on the falling and rising edges of /CS. For Mode 3, the CLK signal is
 * normally high on the falling and rising edges of /CS.
 *
 * Mode	(spi.h)		CPOL		CPHA	supported
 * SPIDEV_MODE0		0			0		yes
 * SPIDEV_MODE1		0			1		no
 * SPIDEV_MODE2		1			0		no
 * SPIDEV_MODE3		1			1		yes
 *
 */

#ifndef CONFIG_W25N01_SPIMODE
#  define CONFIG_W25N01_SPIMODE SPIDEV_MODE0
#endif

/* Standard SPI Frequency up to 104MHz.
 *
 * I use 96 MHz as default value because I want to match it with stm32h7 clocks
 * configuration in board.h that uses PLL2P as clock source for SPI1,2,3.
 *
 */

#ifndef CONFIG_W25N01_SPIFREQUENCY
#  define CONFIG_W25N01_SPIFREQUENCY 30000000
#endif

#define W25N01_DEFAULT_TIMEOUT_MS         5000  // wait ready timeout
#define W25N01_TIMEOUT_PAGE_READ_US        60   // tREmax = 60us (ECC enabled)
#define W25N01_TIMEOUT_PAGE_PROGRAM_US     700  // tPPmax = 700us
#define W25N01_TIMEOUT_BLOCK_ERASE_MS      10   // tBEmax = 10ms
#define W25N01_TIMEOUT_RESET_US            500  // tRSTmax = 500us

#define W25N01_DUMMY         0x00	/* 1 dummy byte */

/* for W25N01GVxxIG: Default BUF=1 after power up */


/* Chip IDs *****************************************************************/
#define JEDEC_MANUFACTURER_ID		0xEF  /* Winbond manufacturer ID */
#define W25N01_DEVID				0x21   /* W25N01 device ID bits 7:0 */
#define W25N01_MEMORY_TYPE			0xAA  /* W25N01 memory type bits 15:8 */


/* W25N01 Commands *******************************************************/
// refer to page 23 of datasheet

/* Reset Operations */
#define W25N01_DEVICE_RESET      	0xFF	/* Device Reset	*/

/* Identification Operations */
#define W25N01_JEDEC_ID      	 	0x9F	/* JEDEC ID: EF + Dummy -> output: AA 21	*/

/* Register Operations */
#define W25N01_READ_STATUS			0x0F	/* 0xO5 also. Read status register           */
#define W25N01_WRITE_STATUS			0x01	/* 0x1F also. Write Status Register          */

/* Read Operations */
#define W25N01_PAGE_DATA_READ    	0x13	/* Read Page Data	*/

#define W25N01_READ_DATA      	 	0x03	/* Read Data Bytes	*/
#define W25N01_FAST_READ	      	0x0B	/* Fast Read Data Bytes	*/
#define W25N01_FAST_READ_4B	      	0x0C	/* Fast Read Data Bytes	*/

/* Write Operations */
#define W25N01_WRITE_ENABLE			0x06 	/* Write enable                      */
#define W25N01_WRITE_DISABLE		0x04 	/* Write disable                     */

/* Bad Block Operations */
#define W25N01_BB_MANAGEMENT   		0xA1 	/* Swap Blocks */
#define W25N01_READ_BBM_LUT       	0xA5	/* Read BBM LUT	*/
#define W25N01_LAST_EEC_FAIL_PAGE_ADDR 0xA9	/* Last ECC failure page address	*/
#define W25N01_READ_PARAMETER_PAGE 	0xC0	/* Read Parameter Page	*/

/* The datasheet’s “Bad Block Marker = non-FFh byte” is a flash-level
 * convention. This refers to physical NAND storage content:
 * - Location : Page 0 Column 0,
 * - Value : any value other than 0xFF indicates a bad block.
 *
 * 0xFFFF is a software-level sentinel used inside this driver. This has
 * nothing to do with the NAND marker byte. It is used as a driver-internal
 * sentinel value meaning: "There is no valid block number here."
 * W25N01GV has 1024 blocks → valid range: 0 … 1023. Thus 0xFFFF is out of
 * range so can never be a real block.
 * */
#define W25N01_INVALID_BLOCK 		0xFFFF
#define W25N01_INVALID_PAGE  		0xFFFF

#define W25N01_BAD_BLOCK_MARKER        0xFF  /* Good block marker! */
#define W25N01_FACTORY_BAD_BLOCK       0x00  /* Factory marked bad block */


#define W25N01_BBM_MAX_ENTRIES		20   /* max entries in LUT per datasheet */
#define W25N01_BLOCK_ADDR_MASK		0x03FF  /* 10-bit block address field, LBA/PBA[9:0]*/

/* Program Operations */
#define W25N01_PROGRAM_DATA_LOAD   		0x02	/* reset buffer	*/
#define W25N01_RAND_PROGRAM_DATA_LOAD 	0x84 	/* Random Program Data Load */
#define W25N01_PROGRAM_EXECUTE  		0x10	/* Program Execute	*/

/* Erase Operations */
#define W25N01_BLOCK_ERASE 				0xD8	/* Block Erase (64 KB)    */

/* W25 Registers ************************************************************/
// refer to page 16 of datasheet

/* NAND erase sets all cells in the block to 1s
*   A freshly erased page (2,048 bytes + 64 spare) will read back as all 0xFF bytes.
*   Programming changes bits from 1 → 0, but you can’t flip them back to 1 without an erase.
*/
#define W25N01_ERASED_STATE			0xFF

/* Register Addresses *******************************************************/
#define PROTECT_REG_ADDR			0xA0
#define CONFIG_REG_ADDR				0xB0
#define STATUS_REG_ADDR				0xC0

/* Status register bit definitions ******************************************/

/* Protection register
 * Bit  |   7  |  6  |  5  |  4  |  3  |  2  |   1  |   0  |
 *	--------------------------------------------------------------
 *	    | SRP0 | BP3 | BP2 | BP1 | BP0 | TB  | WP-E | SRP1 |
 */
#define STATUS_SRP1_MASK     		(1 << 0) /* Bit 0: Status register protect-1  */
#define STATUS_WPE_MASK      		(1 << 1) /* Bit 1: /WP enable bit */
#define STATUS_TB_MASK       		(1 << 2) /* Bit 2: Top / Bottom Protect      */
#define STATUS_BP_SHIFT      		(3)      /* Bits 3-6: 4 Block protect bits     */
#define STATUS_BP_4_MASK     		(15 << STATUS_BP_SHIFT)	/* all 1111 for BP0-3 */
#define STATUS_SRP0_MASK      		(1 << 7) /* Bit 7: Status register protect-0  */

/* Configuration register
 * Bit  |   7   |   6   |   5   |   4   |  3  |  2  |  1  |  0  |
 * 	--------------------------------------------------------------
 * 	    | OTP-L | OTP-E | SR1-L | ECC-E | BUF | (R) | (R) | (R) |
 */
#define STATUS2_BUF_MASK			(1 << 3) /* Bit 3           */
#define STATUS2_ECC_E_MASK       	(1 << 4) /* Bits 4: Enable ECC          */
#define STATUS2_SR1_L_MASK       	(1 << 5) /* Bit 5: Status Register-1 Lock          */
#define STATUS2_OTP_E_MASK      	(1 << 6) /* Bit 6: One Time Program Mode          */
#define STATUS2_OTP_L_MASK     		(1 << 7) /* Bit 7: OTP data page lock		  */

/* Status register
 * Note: this register is read-only
 * Bit  |  7  |   6   |   5   |   4   |   3    |    2   |  1  |   0  |
 * 	--------------------------------------------------------------
 *      | (R) | LUT-F | ECC-1 | ECC-0 | P-FAIL | E-FAIL | WEL | BUSY |
*/
#define STATUS3_BUSY_MASK     		(1 << 0) /* Bit 0: Device ready/busy status  */
#define STATUS3_WEL_MASK     		(1 << 1) /* Bit 1: Write enable latch status */
#define STATUS3_EFAIL_MASK    		(1 << 2) /* Bit 2: Erase fail flag               */
#define STATUS3_PFAIL_MASK    		(1 << 3) /* Bit 3: Program fail flag           */
#define STATUS3_ECC_SHIFT  			4
#define STATUS3_ECC_MASK	   		(3 << 4) /* Bits 4-5: ECC status               */
#define STATUS3_LUTF_MASK	   		(1 << 6) /* Bit 6: LUT fail flag               */

/* Cache flags */
// #define W25N01_CACHE_VALID            (1 << 0)  /* 1=Cache has valid data */
// #define W25N01_CACHE_DIRTY            (1 << 1)  /* 1=Cache is dirty */
// #define W25N01_CACHE_ERASED           (1 << 2)  /* 1=Backing FLASH is erased */

// #define IS_VALID(p)                ((((p)->flags) & W25N01_CACHE_VALID) != 0)
// #define IS_DIRTY(p)                ((((p)->flags) & W25N01_CACHE_DIRTY) != 0)
// #define IS_ERASED(p)               ((((p)->flags) & W25N01_CACHE_ERASED) != 0)

// #define SET_VALID(p)               do { (p)->flags |= W25N01_CACHE_VALID; } while (0)
// #define SET_DIRTY(p)               do { (p)->flags |= W25N01_CACHE_DIRTY; } while (0)
// #define SET_ERASED(p)              do { (p)->flags |= W25N01_CACHE_ERASED; } while (0)

// #define CLR_VALID(p)               do { (p)->flags &= ~W25N01_CACHE_VALID; } while (0)
// #define CLR_DIRTY(p)               do { (p)->flags &= ~W25N01_CACHE_DIRTY; } while (0)
// #define CLR_ERASED(p)              do { (p)->flags &= ~W25N01_CACHE_ERASED; } while (0)


/* Size of the flash */
#define W25N01_FLASH_SIZE         (128*1024*1024) /* 128MB */

/* Chip Geometries **********************************************************/
/* W25N01 (128 MB (1 Gb)) memory capacity */
#define W25N01_BLOCKS				1024     /* 1024 * 128KiB = 128MiB */
#define W25N01_PAGES_PER_BLOCK		64
#define W25N01_PAGE_SIZE			2048
#define W25N01_SPARE_SIZE			64		/* 64 (4 * 16) bytes */
#define W25N01_PAGE_MASK     (W25N01_PAGE_SIZE - 1)

/* by default configuration register Bit4 (ECC-E=1) Data Buffer size is 2048
 * if ECC-E=0, Data buffer size is 2048 + 64 = 2112 */
// #define W25N01_DATA_BUFFER_MASK			(W25N01_PAGE_SIZE + W25N01_SPARE_SIZE - 1)

#define W25N01_BLOCK_SHIFT        17        /* log2(131072 byte per block) = 17 */
#define W25N01_BLOCK_SIZE         (1 << 17) /* Sector size 1 << 17 = 128KB */
#define W25N01_PAGE_SHIFT          11        /* 2**11 = 2048 */

#define W25N01_BLOCK2PAGE_SHIFT  6    /* log2(64 pages per block) */

/****************************************************************************
 * Private Types
 ****************************************************************************/

 /**
* Minimal W25Nxx geometry for W25N01 (1 Gbit)
*  - 2 048B page + 64B spare; 64 pages per block (128 KiB blocks)
*  - 1024 blocks total (approx 128 MiB)
*   [ Block 0 ][ Block 1 ] ... [ Block 1023 ]
*       |           |
*       V           V
*   [Page 0..63] [Page 0..63]
*       |
*       V
*   [2048B data + 64B spare]
*
*/
// struct w25n01_geometry_s {
// 	uint32_t page_size_bytes;       // 2048 (data) + 64 (spare)
// 	uint32_t spare_size_bytes;      // 64
// 	uint32_t block_size;            // 64 pages per block
// 	uint32_t block_size_bytes;      // 131072 (64*2048)
// 	uint16_t total_blocks;          // 1024
// 	uint8_t blockshift;             /* Log2 of block size */
// 	uint8_t pageshift;              /* Log2 of page size */
// 	uint8_t block2pageshift; 		/* log2(pages per block) */
// };

struct w25n01_bbm_entry_s {
	uint16_t bad_block;   /* original bad block */
	uint16_t good_block;  /* replacement block */
};

struct w25n01_bbm_s {
	uint16_t num_entries;
	struct w25n01_bbm_entry_s entries[W25N01_BLOCKS];
};

/* This type represents the state of the MTD device.
 * The struct mtd_dev_s must appear at the beginning of the definition so
 * that you can freely cast between pointers to struct mtd_dev_s and struct
 * w25n01_dev_s.
 */
struct w25n01_dev_s
{
	struct mtd_dev_s      	mtd;         /* MTD interface */
	FAR struct spi_dev_s 	*spi;        /* Saved SPI interface instance */
	uint16_t 				devid;       /* SPI device ID to manage CS lines in board */
	uint32_t 				speed;       /* Overridable via ioctl */
	// struct w25n01_geometry_s 	geom;    /* Geometry of the flash */
	struct w25n01_bbm_entry_s 	bbm[W25N01_BBM_MAX_ENTRIES]; /* Bad block table */
	uint8_t 				*bbm_table;  /* Another Bad block management table */
	// uint16_t 				bb_count;    /* Number of bad blocks */
	uint16_t 				nbadblocks;            /* Another Number of bad blocks found */
	uint16_t               	nsectors;    /* Number of erase sectors */
	uint8_t                	protectmask; /* Mask for protect bits in status register */
	uint8_t                	tbmask;      /* Mask for top/bottom bit in status register */
	FAR uint8_t           	*cmdbuf;     /* Allocated command buffer */
	FAR uint8_t           	*readbuf;    /* Allocated status read buffer */
	// uint8_t               	prev_instr;  /* Previous instruction given to W25 device */
	// uint8_t					id[3];       /* Manufacturer and device ID */
	bool 					initialized;
};

/* Lookup table: divisor = fraction of array to protect */
static const uint16_t w25n01gv_bp_divisor[12] =
{
    0,   /* BP=0000 -> No protect */
    512,  /* BP=0001 -> 1/512 */
    256,  /* BP=0010 -> 1/256 */
    128,  /* BP=0011 -> 1/128 */
    64,  /* BP=0100 -> 1/64 */
    32,  /* BP=0101 -> 1/32 */
    16,  /* BP=0110 -> 1/16 */
    8,   /* BP=0111 -> 1/8 */
    4,   /* BP=1000 -> 1/4 */
    2,   /* BP=1001 -> 1/2 */
    1,  /* BP=1010 -> All blocks */
    1   /* BP=1000..1111 also -> All blocks (alias) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

// SPI helpers
static void w25n01_lock(FAR struct spi_dev_s *spi);
static inline void w25n01_unlock(FAR struct spi_dev_s *spi);

// static void w25n01_select(FAR struct w25n01_dev_s *priv);
// static void w25n01_deselect(FAR struct w25n01_dev_s *priv);

// deprecated:
// static uint8_t w25n01_waitwritecomplete(FAR struct w25n01_dev_s *priv);
static int w25n01_wait_ready(FAR struct w25n01_dev_s *priv,
							uint32_t timeout_ms);

/* Implements from datasheet */

/* Low-level Commands: Reset and status */
static void w25n01_reset(FAR struct w25n01_dev_s *priv);

static inline int w25n01_readid(FAR struct w25n01_dev_s *priv);
static uint8_t w25n01_read_status(FAR struct w25n01_dev_s *priv,
									uint8_t status_addr);
static void w25n01_write_status(FAR struct w25n01_dev_s *priv,
								uint8_t status_addr, uint8_t value);
static inline void w25n01_write_enable(FAR struct w25n01_dev_s *priv);
static inline void w25n01_write_disable(FAR struct w25n01_dev_s *priv);

#ifndef CONFIG_W25N01_READONLY
static void     w25n01_unprotect(FAR struct w25n01_dev_s *priv);
#endif

/* Bad Block Management */
static int w25n01_read_bbm_lut(FAR struct w25n01_dev_s *priv);
static int w25n01_bbm(FAR struct w25n01_dev_s *priv, uint16_t lba, uint16_t *pba);
static uint16_t w25n01_last_ecc_failure_page(FAR struct w25n01_dev_s *priv);

/* Program (Write): Page/Block operations */
static int w25n01_block_erase(FAR struct w25n01_dev_s *priv, uint16_t block);

static void w25n01_program_data_load(FAR struct w25n01_dev_s *priv,
									uint16_t column_addr,
									FAR const uint8_t *buffer, size_t buflen,
									bool random);
static void w25n01_program_execute(FAR struct w25n01_dev_s *priv, uint16_t page);

/* Read */
static void w25n01_page_data_read(FAR struct w25n01_dev_s *priv, uint16_t page);

static void w25n01_read_data(FAR struct w25n01_dev_s *priv, uint16_t column_addr,
							FAR uint8_t *buffer, size_t buflen);
static void w25n01_fast_read(FAR struct w25n01_dev_s *priv, uint16_t column_addr,
							FAR uint8_t *buffer, size_t buflen);
static void w25n01_fast_read_4b(FAR struct w25n01_dev_s *priv, uint16_t column_addr,
							FAR uint8_t *buffer, size_t buflen);

/* Helpers */
static int w25n01_scan_bad_blocks(FAR struct w25n01_dev_s *priv);
static bool w25n01_is_bad_block(FAR struct w25n01_dev_s *priv, uint16_t block);
static int w25n01_mark_bad_block(FAR struct w25n01_dev_s *priv, uint16_t block);

static int w25n01_chip_erase(FAR struct w25n01_dev_s *priv);
static bool w25n01_is_erased(struct w25n01_dev_s *priv, uint16_t page,
							size_t nbytes);


static int w25n01_page_read(FAR struct w25n01_dev_s *priv, uint16_t page,
							uint16_t column_addr, FAR uint8_t *buffer,
							size_t buflen, uint8_t mode);
#ifndef CONFIG_W25N01_READONLY
static int w25n01_page_write(FAR struct w25n01_dev_s *priv, uint16_t page,
                            FAR const uint8_t *data, size_t datalen,
							bool random);
#endif

/* Helpers */
static void w25n01_byteread(FAR struct w25n01_dev_s *priv, FAR uint8_t *buffer,
							uint16_t address, size_t nbytes);
#if defined(CONFIG_MTD_BYTE_WRITE) && !defined(CONFIG_W25N01_READONLY)
static inline void w25n01_bytewrite(FAR struct w25n01_dev_s *priv,
							FAR const uint8_t *buffer,
							uint32_t offset, size_t nbytes);
#endif

/* MTD driver methods */

static int      w25n01_erase(FAR struct mtd_dev_s *dev, off_t startblock,
							 size_t nblocks);
static ssize_t  w25n01_bread(FAR struct mtd_dev_s *dev,
							 off_t startblock,
							 size_t nblocks,
							 FAR uint8_t *buffer);
static ssize_t  w25n01_bwrite(FAR struct mtd_dev_s *dev,
							  off_t startblock,
							  size_t nblocks,
							  FAR const uint8_t *buffer);
static ssize_t  w25n01_read(FAR struct mtd_dev_s *dev,
							off_t offset,
							size_t nbytes,
							FAR uint8_t *buffer);
#if defined(CONFIG_MTD_BYTE_WRITE) && !defined(CONFIG_W25N01_READONLY)
static ssize_t  w25n01_write(FAR struct mtd_dev_s *dev,
							 off_t offset,
							 size_t nbytes,
							 FAR const uint8_t *buffer);
#endif
static int      w25n01_ioctl(FAR struct mtd_dev_s *dev,
							 int cmd,
							 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: w25n01_lock
 ****************************************************************************/
static void w25n01_lock(FAR struct spi_dev_s *spi)
{
	/* On SPI buses where there are multiple devices, it will be necessary to
	* lock SPI to have exclusive access to the buses for a sequence of
	* transfers.  The bus should be locked before the chip is selected.
	*
	* This is a blocking call and will not return until we have exclusive
	* access to the SPI bus.
	* We will retain that exclusive access until the bus is unlocked.
	*/
	SPI_LOCK(spi, true);

	/* After locking the SPI bus, the we also need call the setfrequency,
	* setbits, and setmode methods to make sure that the SPI is properly
	* configured for the device.
	* If the SPI bus is being shared, then it may have been left in an
	* incompatible state.
	*/
	SPI_SETMODE(spi, CONFIG_W25N01_SPIMODE);
	SPI_SETBITS(spi, 8);
	SPI_HWFEATURES(spi, 0);
	SPI_SETFREQUENCY(spi, CONFIG_W25N01_SPIFREQUENCY);
}

/****************************************************************************
 * Name: w25n01_unlock
 ****************************************************************************/
static inline void w25n01_unlock(FAR struct spi_dev_s *spi)
{
	SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: w25n01_select
 ****************************************************************************/
// static void w25n01_select(FAR struct w25n01_dev_s *priv)
// {
// 	SPI_LOCK(priv->spi, true);
// 	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
// }

/****************************************************************************
 * Name: w25n01_deselect
 ****************************************************************************/
// static void w25n01_deselect(FAR struct w25n01_dev_s *priv)
// {
// 	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
// 	SPI_LOCK(priv->spi, false);
// }

/****************************************************************************
 * Name: w25n01_waitwritecomplete (deprecated)
 ****************************************************************************/
// static uint8_t w25n01_waitwritecomplete(FAR struct w25n01_dev_s *priv)
// {
//   uint8_t status;

//   /* Loop as long as the memory is busy with a write cycle. Device sets BUSY
//    * flag to a 1 state whhen previous write or erase command is still
//    * executing and during this time, device will ignore further instructions
//    * except for "Read Status Register" and "Erase/Program Suspend"
//    * instructions. */

// 	do
// 	{
// 		status = w25n01_read_status(priv, STATUS_REG_ADDR);
// 	}
// 	while ((status & STATUS3_BUSY_MASK) != 0);

// 	return status;
// }

/****************************************************************************
* Name: w25n01_wait_ready
****************************************************************************/
static int w25n01_wait_ready(FAR struct w25n01_dev_s *priv,
							uint32_t timeout_ms)
{
	clock_t start_ticks;
	uint8_t status;
	/* Get the start time (in clock ticks) */
	start_ticks = clock_systime_ticks();
	/* Loop until the device is ready or until we time out */
	do
	{
		/* Read the status register */
		status = w25n01_read_status(priv, STATUS_REG_ADDR);
		/* Check if the device is ready */
		if ((status & STATUS3_BUSY_MASK) == 0)
		{
			return OK;
		}
		up_mdelay(100); // wait 100 milliseconds
		/* Check for timeout */
	}
	while (TICK2MSEC(clock_systime_ticks() - start_ticks) < timeout_ms);
	/* Timed out */
	ferr("ERROR: Timeout waiting for ready\n");
	return -ETIMEDOUT;
}

/************************************************************************
 * Name: w25n01_reset
 ****************************************************************************/
static void w25n01_reset(FAR struct w25n01_dev_s *priv)
{
	finfo("priv: %p\n", priv);

	/* Lock and configure the SPI bus */
	w25n01_lock(priv->spi);

	/* Wait for any preceding write or erase operation to complete. */
	w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);

	/* Select this FLASH part. */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	/* Send the "Device Reset" command */
	SPI_SEND(priv->spi, W25N01_DEVICE_RESET);

	/* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
	/* Deselect the FLASH and unlock the bus */
	// w25n01_deselect(priv);

	/* Wait 500 us for the flash to complete the reset */
	nxsig_usleep(W25N01_TIMEOUT_RESET_US);

	/* could check SR-2 and SR-3 bits here for successful reset */
}

/****************************************************************************
 * Name: w25n01_readid
 ****************************************************************************/
static inline int w25n01_readid(FAR struct w25n01_dev_s *priv)
{

	uint8_t id[3];

	finfo("priv: %p\n", priv);

	/* Select this FLASH part. */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);

	/* Send the "Read ID" command and read the first three ID bytes */
	//   need 8 dummies clocks after 0x9F command
    SPI_SEND(priv->spi, W25N01_JEDEC_ID);
	SPI_SEND(priv->spi, W25N01_DUMMY); // dummy byte

	// SPI_RECVBLOCK(priv->spi, id, 3);
	id[0] = (uint8_t)SPI_SEND(priv->spi, W25N01_DUMMY);
	id[1] = (uint8_t)SPI_SEND(priv->spi, W25N01_DUMMY);
	id[2] = (uint8_t)SPI_SEND(priv->spi, W25N01_DUMMY);

	/* Deselect the FLASH and unlock the bus */
	// w25n01_deselect(priv);
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);


	finfo("Manufacturer: %02x Memory: %02x Capacity: %02x\n",
		  id[0], id[1], id[2]);

	if (id[0] != JEDEC_MANUFACTURER_ID)
	{
		ferr("ERROR: Unexpected manufacturer ID: 0x%02x\n", id[0]);
		return -ENODEV;
	}
	if ((id[1] != W25N01_MEMORY_TYPE ) || (id[2] != W25N01_DEVID))
	{
		// priv->geom.blockshift = W25N01_BLOCK_SHIFT;
		// priv->geom.pageshift = W25N01_PAGE_SHIFT;
		// priv->geom.block2pageshift = W25N01_BLOCK2PAGE_SHIFT;
		// priv->geom.page_size_bytes = W25N01_PAGE_SIZE;
		// priv->geom.spare_size_bytes = W25N01_SPARE_SIZE;
		// priv->geom.block_size = W25N01_PAGES_PER_BLOCK;
		// priv->geom.block_size_bytes = W25N01_BLOCK_SIZE;
		// priv->geom.total_blocks = W25N01_BLOCKS;
		// priv->nsectors = W25N01_BLOCKS;
	// }
	// else {
		/* We don't understand the manufacturer or the memory type */
		ferr("ERROR: Unrecognized manufacturer/memory type: %02x/%02x\n",
		id[0], id[1]);
		return -ENODEV;
	}

	return OK;
}

/****************************************************************************
 * Name: w25n01_read_status
 ****************************************************************************/
static uint8_t w25n01_read_status(FAR struct w25n01_dev_s *priv,
									uint8_t status_addr)
{
	uint8_t status;

	/* Select this FLASH part */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);

	/* Send "Read Status Register" instruction */
	SPI_SEND(priv->spi, W25N01_READ_STATUS);
	/* Send SR addr */
	SPI_SEND(priv->spi, (uint8_t)status_addr);

	/* Receive the status register */
	// SPI_RECVBLOCK(priv->spi, &status, 1);
	status = (uint8_t)SPI_SEND(priv->spi, W25N01_DUMMY);

	/* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);

	return status;
}

/****************************************************************************
 * Name: w25n01_write_status
 * Description: on Kekute H7 Mini v1.3 /WP (and /HOLD) pin is tied to VCC,
 * so it is hardware protection.
 * Writable status register bits include:
 * - Protection Register bits: 0,2..7: SRP0-1, SRP1, TB, BP0-3.
 * - Configuration Register bits: 3..7: BUF, ECC-E, SR1-L, OTP-E, OTP-L.
 * inputs:
 * - status_addr: address of the status register to write (PROTECT_REG_ADDR
 * (0xAi) or CONFIG_REG_ADDR (0xBi))
 * - value: value to write into the status register
 * Returns: none
 *
 ****************************************************************************/
static void w25n01_write_status(FAR struct w25n01_dev_s *priv,
								uint8_t status_addr, uint8_t value)
{
	/* Select this FLASH part */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	/* Send "Write Status Register" instruction */
	SPI_SEND(priv->spi, W25N01_WRITE_STATUS);
	/* Send SR addr */
	SPI_SEND(priv->spi, (uint8_t)status_addr);
	/* Send the status register value */
	SPI_SEND(priv->spi, value);
	/* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
}

/****************************************************************************
 * Name:  w25n01_write_enable
 ****************************************************************************/
static inline void w25n01_write_enable(FAR struct w25n01_dev_s *priv)
{
	/* Select this FLASH part */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);

	/* Send "Write Enable" command */
	SPI_SEND(priv->spi, W25N01_WRITE_ENABLE);

	/* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
}

/****************************************************************************
 * Name:  w25n01_write_disable
 ****************************************************************************/
static inline void w25n01_write_disable(FAR struct w25n01_dev_s *priv)
{
	/* Select this FLASH part */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);

	/* Send "Write Disable" command */
	SPI_SEND(priv->spi, W25N01_WRITE_DISABLE);

	/* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
}

/****************************************************************************
 * Name: w25n01_unprotect
 ****************************************************************************/
#ifndef CONFIG_W25N01_READONLY
static void w25n01_unprotect(FAR struct w25n01_dev_s *priv)
{
	// set SRP1 to 0 (hardware protection mode: /WP pin tied to 3.3V):
	w25n01_write_status(priv, PROTECT_REG_ADDR, 0x00);
}
#endif

/****************************************************************************
 * Name:  w25n01_read_bbm_lut
 *
 * (p. 33) LUT: 20 Logical-Physical memory block links (from LBA0/PBA0 to
 * LBA19/PBA19).
 *
 * LBA[9:0] & PBA[9:0] are effective Block Addresses. LBA[15:14] is used for
 * additional information.
 *
 * Used to check the existing address links stored inside the LUT. If link
 * exists there is bad block and LBA[15:14] give additional information:
 * LBA[15]	LBA[14]		Description
 * (Enable)	(Invalid)
 *   0	   	0		This link is available to use.
 *   1	   	0		This link is enabled and it is a valid link.
 *   1	   	1		This link was enabled but it is an invalid link.
 *   0	   	1		Reserved for future use.
 *
 * This command stores 20 entries of bad block mapping in piv->bbm[20]:
 * - bm[i] = { bad_block = LBAi, good_block = PBAi }
 * If invalid link (LBA[15:14] = 11) you must avoid the LBA (lba & 0x03FF)
 *
 * The command also updates priv->nbadblocks with the number of bad blocks found.
 *
 ****************************************************************************/
static int w25n01_read_bbm_lut(FAR struct w25n01_dev_s *priv)
{
	uint8_t lut_entry[W25N01_BBM_MAX_ENTRIES * 4];	/* 4 bytes per entry */
	uint16_t lba, pba;
	/* Clear current list */
	priv->nbadblocks = 0;
	memset(priv->bbm, 0, sizeof(priv->bbm));


	finfo("Reading BBM Look Up Table...\n");
	/* Read all BBM LUT entries */
	/* Select this FLASH part */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	/* Send "Read BBM Look Up Table" command */
	SPI_SEND(priv->spi, W25N01_READ_BBM_LUT);
	/* Send dummy byte */
	SPI_SEND(priv->spi, W25N01_DUMMY);
	/* Receive 4 bytes: 2 bytes LBA + 2 bytes PBA */
	// SPI_RECVBLOCK(priv->spi, lut_entry, W25N01_BBM_MAX_ENTRIES * 4);
	for (int i = 0; i < W25N01_BBM_MAX_ENTRIES * 4; i++)
	{
		lut_entry[i] = (uint8_t)SPI_SEND(priv->spi, W25N01_DUMMY);
	}
	/* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);

	/* Copy LBA and PBA to bbm array */
	for (int i = 0; i < W25N01_BBM_MAX_ENTRIES; i++)
	{
		uint8_t *entry = &lut_entry[i * 4];
		lba = (uint16_t)(entry[0] << 8) | entry[1];
		pba = (uint16_t)(entry[2] << 8) | entry[3];

		priv->bbm[i].bad_block = lba;
		priv->bbm[i].good_block = pba;

		/* Check if entry is valid */
		uint8_t link_status = (lba >> 14) & 0x03; // extract bits 15 and 14

		if (link_status > 1) // bits[15:14] = 10 or 11
		{
			priv->nbadblocks++;
		}
	}

	return OK;
}

/****************************************************************************
* Name:  w25n01_bbm
*
* (p. 32) Swap Blocks: The logical block address is the address for the “bad”
* block that will be replaced by the “good” block indicated by the physical
* block address.
*
 * (p. 32) 20 bad blocks (W25N01_BBM_MAX_ENTRIES).
 * A “Bad Block Marker” is a non-FFh data byte stored at Byte 0 of Page 0 for
 * each bad block. An additional marker is also stored in the first byte of the
 * 64-Byte spare area (see w25n01_read_bbm_lut()).
 *
 * A Write Enable instruction must be executed before the device will accept the
 * Bad Block Management Instructions. The logical block address is the address
 * for the “bad” block that will be replaced by the “good” block
 * indicated by the physical block address.
 *
 * Prior to issuing the Bad Block Management command, the LUT-F bit value can be
 * checked or a “Read BBM Look Up Table” command can be issued to confirm if
 * spare links are still available in the LUT.
 *

****************************************************************************/
static int w25n01_bbm(FAR struct w25n01_dev_s *priv, uint16_t lba, uint16_t *pba)
{

	uint16_t good_block;
	// uint16_t bad_block = lba;
	// uint32_t lut_f;
	int ret;

	finfo("lba: %d\n", lba);

	// Check LUT-F bit6 in SR-3 if BBM LUT is full
	priv->readbuf[0] = w25n01_read_status(priv, CONFIG_REG_ADDR);
	if (priv->readbuf[0] & STATUS3_LUTF_MASK)
	{
		ferr("ERROR: BBM LUT is full, cannot add new entry\n");
		*pba = W25N01_INVALID_BLOCK;
        return -ENOSPC;
    }

	/* Find a free good physical block from the end of the flash  to replace
	 * bad block */
	good_block = W25N01_BLOCKS - 1;
	while (good_block > 0)
	{
		if (!w25n01_is_bad_block(priv, good_block))
		{
			break;
		}
		good_block--;
	}
	if (good_block == 0)
	{
		ferr("ERROR: No good blocks available for remapping\n");
		*pba = W25N01_INVALID_BLOCK;
		return -ENOSPC;
	}
	/* Issue Swap Blocks command */
	finfo("Swapping bad block %d with good block %d\n", lba, good_block);
	/* Enable write */
	w25n01_write_enable(priv);
	/* Send Swap Blocks command */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	SPI_SEND(priv->spi, W25N01_BB_MANAGEMENT);
	/* Send 16-bit logical block address (bad block) */
	SPI_SEND(priv->spi, (lba >> 8) & 0xFF);
	SPI_SEND(priv->spi, lba & 0xFF);
	/* Send 16-bit physical block address (good block) */
	SPI_SEND(priv->spi, (good_block >> 8) & 0xFF);
	SPI_SEND(priv->spi, good_block & 0xFF);
	/* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
	/* Wait for completion */
	ret = w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
	if (ret < 0)
	{
		*pba = W25N01_INVALID_BLOCK;
		return ret;
	}
	/* Return the physical block address */
	*pba = good_block;
	return OK;
}

/****************************************************************************
 * Name:  w25n01_last_ecc_failure_page
 *
 * (page 34) By default ECC-E bit is set to 1.
 * Upon finishing the read operation, the ECC status bits should be check to v
 * erify if there’s any ECC correction or un-correctable errors existed in the
 * read out data. If ECC-1 & ECC-0 equal to (1, 0) or (1, 1), the previous read
 * out data contain one or more pages that contain ECC un-correctable errors.
 * The failure page address (or the last page address if it’s multiple pages)
 * can be obtained by issuing the “Last ECC failure Page Address” command.
 *
 * During a “Program Execute” command for a specific page, the ECC algorithm
 * will calculate the ECC information based on the data inside the 2K-Byte
 * data buffer and write the ECC data into the extra 64-Byte ECC area in the
 * same physical memory page.
 *
 * During the Read operations, ECC information will be used to verify the data
 * read out from the physical memory array and possible corrections can be
 * made to limited amount of data bits that contain errors. The ECC Status Bits
 * (ECC-1 & ECC-0) will also be set indicating the result of ECC calculation.
 * ECC status		Description
 * ECC-1	ECC-0
 * 0		0		Entire data output is successful, without any ECC correction.
 * 0		1		Entire data output is successful, with 1~4 bit/page ECC
 * 					corrections in either a single page or multiple pages.
 * 1		0		Entire data output contains more than 4 bits errors only in
 * 					a single page which cannot be repaired by ECC.
 * 					In the Continuous Read Mode, an additional command can be
 * 					used to read out the Page Address (PA) which had the errors.
 * 1		1		Entire data output contains more than 4 bits errors in
 * 					multiple pages.
 * 					In the Continuous Read Mode, an additional command can only
 * 					provided the last Page Address (PA) that had failure, the
 * 					user cannot obtain the PAs for other failure pages.
 * 					Data is not suitable to use.
 *
 *
 ****************************************************************************/
static uint16_t w25n01_last_ecc_failure_page(FAR struct w25n01_dev_s *priv)
{
	uint16_t page;
	uint8_t ecc_status;
	/* Send Read Status Register command */
	priv->readbuf[0] = w25n01_read_status(priv, STATUS_REG_ADDR);
	/* Extract bits 5-4: ECC status Bit */
	ecc_status = (uint8_t)((priv->readbuf[0] & STATUS3_ECC_MASK) >> 4);

	if (ecc_status > 1) // 10 and 11: un-correctable errors
	{
		/* Send "Last ECC Failure Page Address" command */
		SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
		SPI_SEND(priv->spi, W25N01_LAST_EEC_FAIL_PAGE_ADDR);
		SPI_SEND(priv->spi, W25N01_DUMMY);
		/* Receive 2 bytes: 16-bit page address */
		// SPI_RECVBLOCK(priv->spi, priv->readbuf, 2);
		priv->readbuf[0] = (uint8_t)SPI_SEND(priv->spi, W25N01_DUMMY);
		priv->readbuf[1] = (uint8_t)SPI_SEND(priv->spi, W25N01_DUMMY);

		/* Deselect the FLASH */
		SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);

		page = (uint16_t)(priv->readbuf[0] << 8) | priv->readbuf[1];
	}
	else
	{
		/* No un-correctable errors */
		return W25N01_INVALID_PAGE;
	}

	return page;
}

/****************************************************************************
 * Name:  w25n01_blockerase
 ****************************************************************************/
static int w25n01_block_erase(FAR struct w25n01_dev_s *priv, uint16_t block)
{
	int ret;
	// uint16_t page_addr = (block << W25N01_BLOCK_SHIFT) / W25N01_PAGE_SIZE;
	uint16_t page_addr = (uint16_t)(block << W25N01_BLOCK2PAGE_SHIFT);
	off_t address = (off_t)block << W25N01_BLOCK_SHIFT;

	finfo("sector: %08lx\n", (long)block);

	/* Check if block is bad */
	if (w25n01_is_bad_block(priv, block))
	{
		fwarn("Block %d is marked bad, skipping erase\n", block);
		return OK;
	}

	/* Wait for device ready */
	ret = w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
	if (ret < 0)
	{
		return ret;
	}

	/* Check if block is already erased. */
	if (w25n01_is_erased(priv, address, W25N01_BLOCK_SIZE))
	{
		finfo("Block %d is already erased, skipping erase\n", block);
		return OK;
	}

	/* Enable write */
	w25n01_write_enable(priv);

	/* Send block erase command */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	/* Send "Block Erase" instruction */
	SPI_SEND(priv->spi, W25N01_BLOCK_ERASE);
	/* Send 8 dummy cycles → 1 byte of 0x00 */
	SPI_SEND(priv->spi, W25N01_DUMMY);
	/* Send 16-bit address */
	SPI_SEND(priv->spi, (page_addr >> 8) & 0xFF);	/* PA[15:8] */
	SPI_SEND(priv->spi, page_addr & 0xFF);		/* PA[7:0] */
	/* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);

	/* Wait for the erase operation to complete. */
	return w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
}

/****************************************************************************
 * Name:  w25n01_program_data_load
 *
 * (p. 36, 38) The Program Data Load instruction allows from one byte to 2,112
 * bytes (a page) of data to be programmed at previously erased (FFh) memory
 * locations. A Program operation involves two steps: 1. Load the program data
 * into the Data Buffer. 2. Issue “Program Execute” command to transfer the
 * data from Data Buffer to the specified memory page.
 *
  * “Load Program Data” instruction will reset the unused the data bytes in the
 * Data Buffer to FFh value, while “Random Load Program Data” instruction will
 * only update the data bytes that are specified by the command input sequence,
 * the rest of the Data Buffer will remain unchanged.
 *
 * When random = true (default), use W25N01_RAND_PROGRAM_DATA_LOAD
 ****************************************************************************/
static void w25n01_program_data_load(FAR struct w25n01_dev_s *priv,
									uint16_t column_addr,
									FAR const uint8_t *buffer, size_t buflen,
									bool random)
{
	finfo("column_addr: %04x datalen: %d\n", column_addr, (int)buflen);

	// assert(datalen <= W25N01_PAGE_SIZE);
	// assert(column_addr + datalen <= W25N01_PAGE_SIZE);

	/* Wait for device ready */
	int ret = w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
	if (ret < 0)
	{
		return;
	}

	/* Enable write before Load Program Data Instructions */
	w25n01_write_enable(priv);

	/* Load data into buffer */
	/* Send "Program Data Load" command p. 25/36 */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	if (random)
	{
		SPI_SEND(priv->spi, W25N01_RAND_PROGRAM_DATA_LOAD);
	}
	else
	{
		SPI_SEND(priv->spi, W25N01_PROGRAM_DATA_LOAD);
	}
	/* Send 8 dummy cycles → 1 byte of 0x00 */
	SPI_SEND(priv->spi, W25N01_DUMMY);
	/* Send 2 x 8-bit column address */
	SPI_SEND(priv->spi, (column_addr >> 8) & 0xFF);	/* CA[15:8], CA[15:12] are considered as dummy bits. */
	SPI_SEND(priv->spi, column_addr & 0xFF);		/* CA[7:0] */
	/* Write data */
	SPI_SNDBLOCK(priv->spi, buffer, buflen);

	/* data have been sent, deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
}

/****************************************************************************
 * Name:  w25n01_program_execute
 ****************************************************************************/
static void w25n01_program_execute(FAR struct w25n01_dev_s *priv, uint16_t page)
{
	finfo("page_addr: %04x\n", page);

	/* Send "Program Execute" command p. 38 */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	SPI_SEND(priv->spi, W25N01_PROGRAM_EXECUTE);
	/* Send 8 dummy cycles → 1 byte of 0x00 */
	SPI_SEND(priv->spi, W25N01_DUMMY);
	/* Send 16-bit address */
	SPI_SEND(priv->spi, (page >> 8) & 0xFF);	/* PA[15:8] */
	SPI_SEND(priv->spi, page & 0xFF);		/* PA[7:0] */
	/* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);

	/* Wait for the program operation to complete. */
	w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
}

/****************************************************************************
 * Name:  w25n01_page_data_read
 *
 * The Page Data Read instruction transfer data of the specified memory page
 * into the 2,112-Byte Data Buffer.
 * After the 2,112 bytes of page data are loaded into the Data Buffer, several
 * Read instructions can be issued to access the Data Buffer and read out the
 * data. Depending on the BUF bit setting in the Status Register,either
 * “Buffer Read Mode” or “Continuous Read Mode” may be used to accomplish the
 * read operations.
 *
 ****************************************************************************/
static void w25n01_page_data_read(FAR struct w25n01_dev_s *priv,
								uint16_t page)
{
	finfo("page_addr: %04x\n", page);

	int ret = w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
	if (ret < 0)
	{
		return;
	}

	/* Send "Page Data Read" command */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	SPI_SEND(priv->spi, W25N01_PAGE_DATA_READ);
	/* Send 8 dummy cycles → 1 byte of 0x00 */
	SPI_SEND(priv->spi, W25N01_DUMMY);
	/* Send 16-bit address */
	SPI_SEND(priv->spi, (page >> 8) & 0xFF);	/* PA[15:8] */
	SPI_SEND(priv->spi, page & 0xFF);		/* PA[7:0] */
	/* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
}

/****************************************************************************
 * Name:  w25n01_read_data
 *
 * (page 40) The Read Data instruction allows one or more data bytes to be
 * sequentially read from the Data Buffer after executing the Read Page Data
 * instruction. The Read Data instruction is initiated by driving the /CS pin
 * low and then shifting the instruction code “03h” followed by the 16-bit
 * Column Address and 8-bit dummy clocks or a 24-bit dummy clocks into the DI
 * pin. After the address is received, the data byte of the addressed Data
 * Buffer location will be shifted out on the DO pin at the falling edge of
 * CLK with most significant bit (MSB) first. The address is automatically
 * incremented to the next higher address after each byte of data is shifted
 * out allowing for a continuous stream of data. The instruction is completed
 * by driving /CS high.
 *
 * When BUF=1, the device is in the Buffer Read Mode. The data output sequence
 * will start from the Data Buffer location specified by the 16-bit Column
 * Address and continue to the end of the Data Buffer. Once the last byte of
 * data is output, the output pin will become Hi-Z state. When BUF=0, the
 * device is in the Continuous Read Mode, the data output sequence will start
 * from the first byte of the Data Buffer and increment to the next higher
 * address. When the end of the Data Buffer is reached, the data of the first
 * byte of next memory page will be following and continues through the entire
 * memory array. This allows using a single Read instruction to read out the
 * entire memory array and is also compatible to Winbond’s SpiFlash NOR flash
 * memory command sequence.
 *
 ****************************************************************************/
static void w25n01_read_data(FAR struct w25n01_dev_s *priv, uint16_t column_addr,
							FAR uint8_t *buffer, size_t nbytes)
{
	finfo("nbytes: %d\n", (int)nbytes);

	/* check buffer read mode p. 19. BUF=1 (default) */
	priv->readbuf[0] = w25n01_read_status(priv, CONFIG_REG_ADDR);
	w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);

	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	// /* using standard Read (0x03) p. 25/40 */
	SPI_SEND(priv->spi, W25N01_READ_DATA);
	if ((priv->readbuf[0] & STATUS2_BUF_MASK) != 0)
	{
		/* Buffer Read Mode BUF=1 → Table 2 page 25 */
		/* Send 2 x 8-bit column address */
		SPI_SEND(priv->spi, (column_addr >> 8) & 0xFF);	/* CA[15:8], CA[15:12] are considered as dummy bits. */
		SPI_SEND(priv->spi, column_addr & 0xFF);		/* CA[7:0] */
	}
	else
	{
		/* Continuous Read Mode → Table 1 page 24 */
		/* send 2 dummy bytes  */
		SPI_SEND(priv->spi, W25N01_DUMMY);
		SPI_SEND(priv->spi, W25N01_DUMMY);
	}
	SPI_SEND(priv->spi, W25N01_DUMMY);

	/* read out data */
	// SPI_RECVBLOCK(priv->spi, &buffer, nbytes);


	/* complete the instruction */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
}

/****************************************************************************
 * Name:  w25n01_fast_read
 ****************************************************************************/
static void w25n01_fast_read(FAR struct w25n01_dev_s *priv, uint16_t column_addr,
							FAR uint8_t *buffer, size_t nbytes)
{
	finfo("nbytes: %d\n", (int)nbytes);

	/* check buffer read mode p. 19. BUF=1 (default) */
	priv->readbuf[0] = w25n01_read_status(priv, CONFIG_REG_ADDR);
	w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);

	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	// /* using Fast Read (0x0B) */
	SPI_SEND(priv->spi, W25N01_FAST_READ);
	if ((priv->readbuf[0] & STATUS2_BUF_MASK) != 0)
	{
		/* Buffer Read Mode BUF=1 → Table 2 page 25 */
		/* Send 2 x 8-bit column address */
		SPI_SEND(priv->spi, (column_addr >> 8) & 0xFF);	/* CA[15:8], CA[15:12] are considered as dummy bits. */
		SPI_SEND(priv->spi, column_addr & 0xFF);		/* CA[7:0] */
	}
	else
	{
		/* Continuous Read Mode → Table 1 page 24 */
		/* send 2 dummy bytes  */
		SPI_SEND(priv->spi, W25N01_DUMMY);
		SPI_SEND(priv->spi, W25N01_DUMMY);
	}
	SPI_SEND(priv->spi, W25N01_DUMMY);
	SPI_SEND(priv->spi, W25N01_DUMMY);

	/* read out data */
	// SPI_RECVBLOCK(priv->spi, &buffer, nbytes);
	for (size_t i = 0; i < nbytes; i++)
	{
		buffer[i] = (uint8_t)SPI_SEND(priv->spi, W25N01_DUMMY);
	}

	/* complete the instruction */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
}

/****************************************************************************
 * Name:  w25n01_fast_read_4b
 ****************************************************************************/
static void w25n01_fast_read_4b(FAR struct w25n01_dev_s *priv, uint16_t column_addr,
							FAR uint8_t *buffer, size_t nbytes)
{
	finfo("nbytes: %d\n", (int)nbytes);

	/* check buffer read mode p. 19. BUF=1 (default) */
	priv->readbuf[0] = w25n01_read_status(priv, CONFIG_REG_ADDR);
	w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);

	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	/* using Fast Read with 4-Byte Address (0x0C) */
	SPI_SEND(priv->spi, W25N01_FAST_READ_4B);
	if ((priv->readbuf[0] & STATUS2_BUF_MASK) != 0)
	{
		/* Buffer Read Mode BUF=1 → Table 2 page 25 */
		/* Send 2 x 8-bit column address */
		SPI_SEND(priv->spi, (column_addr >> 8) & 0xFF);	/* CA[15:8], CA[15:12] are considered as dummy bits. */
		SPI_SEND(priv->spi, column_addr & 0xFF);		/* CA[7:0] */
	}
	else
	{
		/* Continuous Read Mode → Table 1 page 24 */
		/* send 2 dummy bytes  */
		SPI_SEND(priv->spi, W25N01_DUMMY);
		SPI_SEND(priv->spi, W25N01_DUMMY);
	}
	SPI_SEND(priv->spi, W25N01_DUMMY);
	SPI_SEND(priv->spi, W25N01_DUMMY);
	SPI_SEND(priv->spi, W25N01_DUMMY);

	/* read out data */
	// SPI_RECVBLOCK(priv->spi, &buffer, nbytes);
	for (size_t i = 0; i < nbytes; i++)
	{
		buffer[i] = (uint8_t)SPI_SEND(priv->spi, W25N01_DUMMY);
	}

	/* complete the instruction */
	SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), false);
}

/****************************************************************************
 * Name: w25n01_scan_bad_blocks.
 *
 * Same as w25n01_read_bbm_lut but reads physical bad block markers instead of
 * LUT table. This implementation uses the legacy ONFI-compatibility legacy
 * marker, it scans all blocks by reading Page 0 Column 0 for bad block markers
 * and populates the bad block table: priv->bbm_table.
 *
 * Notes:
 * - The true, official bad-block marker location for the W25N01GV is: "Byte 0
 * of Spare Area of Page 0 in each block" (and optionally the first byte of the
 * main array for legacy ONFI compatibility). Location: Page 0, Spare[0] :
 * Official Winbond bad-block marker; Page 0, Main[0]: ONFI-compatibility
 * legacy marker.
 * When ECC=1 (default), Spare[0] is Bad block marker; Sapre[1] is reserved;
 * Spare[15:2] is ECC parity.
 *
 * - "Bad block marker = non-FFh byte stored at Byte 0 of Page 0 for each bad
 * block." this refers to physical NAND storage content. Location: Page 0;
 * Column 0 (or spare[0], depending on ECC mode). Value: 0xFF -> good block,
 * anything else -> bad block. This is not an address, it is data stored inside
 * the NAND.
 * - 0xFFFF is a software-level sentinel used inside this driver. This has
 * nothing to do with the NAND marker byte. Its meaning “There is no valid
 * block number here.”
 *
 ****************************************************************************/
static int w25n01_scan_bad_blocks(FAR struct w25n01_dev_s *priv)
{
	uint8_t marker; // first byte of Page 0 Column 0
	uint16_t block;
	int ret;
	uint8_t bad_count = 0; // TODO replaced by dev->nbadblocks

	finfo("Scanning for bad blocks...\n");

	/* Allocate bad block table if not already allocated */
	// TODO replace with w25n01_bbm_s */
	if (!priv->bbm_table)
	{
		priv->bbm_table = (uint8_t *)kmm_zalloc(W25N01_BLOCKS);
		if (!priv->bbm_table)
		{
			ferr("ERROR: Failed to allocate BBM table\n");
			return -ENOMEM;
		}
	}

	/* Scan all blocks */
	for (block = 0; block < W25N01_BLOCKS; block++)
	{
		/* Read Page 0 Column 0 */
		uint16_t page = block * W25N01_PAGES_PER_BLOCK;

		ret = w25n01_page_read(priv, page, 0, &marker, 1, 0);
		if (ret < 0)
		{
			/* Read error, mark as bad */
			priv->bbm_table[block] = W25N01_FACTORY_BAD_BLOCK;
			bad_count++;
			finfo("Block %ld marked bad (read error)\n", (long)block);
			continue;
		}

		/* Check for factory bad block marker */
		if (marker == W25N01_FACTORY_BAD_BLOCK)
		{
			priv->bbm_table[block] = W25N01_FACTORY_BAD_BLOCK;
			bad_count++;
			finfo("Block %ld is factory marked bad\n", (long)block);
		}
		else
		{
			priv->bbm_table[block] = W25N01_BAD_BLOCK_MARKER; // good block!
		}
	}

	if (bad_count > W25N01_BBM_MAX_ENTRIES)
	{
		ferr("ERROR: Too many bad blocks: %u (max %u)\n. Check BBM LUT!!!\n",
			bad_count, W25N01_BBM_MAX_ENTRIES);
		return -EIO;
	}

	priv->nbadblocks = bad_count;
	finfo("Found %u bad blocks out of %u total blocks\n",
		bad_count, W25N01_BLOCKS);

	// priv->nbadblocks = 0;		// reset bad block count

	// finfo("Scanning for bad blocks...\n");

	// /* Allocate bad block table if not already allocated */
	// // TODO replace with w25n01_bbm_s */
	// if (!priv->bbm_table)
	// {
	// 	priv->bbm_table = (uint8_t *)kmm_zalloc(W25N01_BLOCKS);
	// 	if (!priv->bbm_table)
	// 	{
	// 		ferr("ERROR: Failed to allocate BBM table\n");
	// 		return -ENOMEM;
	// 	}
	// }

	// /* Scan all blocks */
	// for (block = 0; block < W25N01_BLOCKS; block++)
	// {
	// 	/* Read Page 0 Column 0 */
	// 	uint16_t page = block * W25N01_PAGES_PER_BLOCK;

	// 	ret = w25n01_page_read(priv, page, 0, &marker, 1, 0);
	// 	if (ret < 0)
	// 	{
	// 		/* Read error, mark as bad */
	// 		priv->bbm_table[block] = W25N01_FACTORY_BAD_BLOCK;
	// 		priv->nbadblocks++;
	// 		finfo("Block %ld marked bad (read error)\n", (long)block);
	// 		continue;
	// 	}

	// 	/* Check for factory bad block marker */
	// 	if (marker == W25N01_FACTORY_BAD_BLOCK)
	// 	{
	// 		priv->bbm_table[block] = W25N01_FACTORY_BAD_BLOCK;
	// 		priv->nbadblocks++;
	// 		finfo("Block %ld is factory marked bad\n", (long)block);
	// 	}
	// 	else
	// 	{
	// 		priv->bbm_table[block] = W25N01_BAD_BLOCK_MARKER; // good block!
	// 	}
	// }

	// if (priv->nbadblocks > W25N01_BBM_MAX_ENTRIES)
	// {
	// 	ferr("ERROR: Too many bad blocks: %u (max %u)\n. Check BBM LUT!!!\n",
	// 		priv->nbadblocks, W25N01_BBM_MAX_ENTRIES);
	// 	return -EIO;
	// }

	// finfo("Found %u bad blocks out of %u total blocks\n",
	// 	priv->nbadblocks, W25N01_BLOCKS);

	return OK;
}

/****************************************************************************
 * Name: w25n01_is_bad_block
 ****************************************************************************/
static bool w25n01_is_bad_block(FAR struct w25n01_dev_s *priv, uint16_t block)
{
	if (!priv->bbm_table || block >= W25N01_BLOCKS)
	{
		return true;  /* Consider out of range blocks as bad */
	}

	return (priv->bbm_table[block] == W25N01_FACTORY_BAD_BLOCK);
}

/****************************************************************************
 * Name: w25n01_mark_bad_block
 ****************************************************************************/
static int w25n01_mark_bad_block(FAR struct w25n01_dev_s *priv, uint16_t block)
{
	uint8_t marker = W25N01_FACTORY_BAD_BLOCK;
	int ret;

	if (!priv->bbm_table || block >= W25N01_BLOCKS)
	{
		return -EINVAL;
	}

	/* Update BBM table in memory */
	if (priv->bbm_table[block] != W25N01_FACTORY_BAD_BLOCK)
	{
		priv->bbm_table[block] = W25N01_FACTORY_BAD_BLOCK;
		priv->nbadblocks++;
		finfo("Marked block %ld as bad (runtime failure)\n", (long)block);
	}

	/* Write marker to flash OOB area */
	off_t page = block * W25N01_PAGES_PER_BLOCK;

	/* Write to first page's OOB area */
	ret = w25n01_page_write(priv, page, &marker, 1, true);
	if (ret < 0)
	{
		ferr("ERROR: Failed to write bad block marker for block %ld\n", (long)block);
	}

	return ret;
}

/****************************************************************************
 * Name:  w25n01_chip_erase
 ****************************************************************************/
static inline int w25n01_chip_erase(FAR struct w25n01_dev_s *priv)
{
	finfo("priv: %p\n", priv);
	int ret = OK;

	/* There are 1024 blocks in W25N01GV */
	for (uint16_t block = 0; block < W25N01_BLOCKS; block++)
	{
		if (w25n01_is_bad_block(priv, block))
    		continue;

		ret = w25n01_block_erase(priv, block);

		/* You may want to add yield or progress here */
		if (ret < 0) break;
	}

	return ret;
}

/****************************************************************************
 * Name:  w25n01_is_erased (bytes but aligned to page (2048 bytes))
 *
 * Needs:
 * - address is page aligned (n * 2048)
 * - nbytes is multiple of bytes (n * 2048)
 *
 * TODO: if byte offset
 ****************************************************************************/
static bool w25n01_is_erased(struct w25n01_dev_s *priv, uint16_t page,
							size_t nbytes)
{
	DEBUGASSERT((page & W25N01_PAGE_MASK) == 0);
	DEBUGASSERT((nbytes & W25N01_PAGE_MASK) == 0);

	uint16_t npages = nbytes >> W25N01_PAGE_SHIFT;
	unsigned int i;
	uint8_t *buf;


	buf = (uint8_t *)kmm_malloc(W25N01_PAGE_SIZE); // 1 page
	if (!buf)
	{
		return false;
	}

	// memset(&erased_8, W25N01_ERASED_STATE, sizeof(erased_8));

	while (npages--)
	{
		/* Check if all bytes of page is in erased state.*/
		w25n01_page_read(priv, page, 0, buf, W25N01_PAGE_SIZE, 0);

		for (i = 0; i < W25N01_PAGE_SIZE; i++)
		{
			if (buf[i] != W25N01_ERASED_STATE)
			{
				/* Page not in erased state! */
				kmm_free(buf);
				return false;
			}
		}
		page++;
	}
	kmm_free(buf);
	return true;
}

/****************************************************************************
 * Name: w25n01_page_read
 *
 * This function implements the Page Data Read (13h) and the Fast
 * Read (0Bh) instructions to store data into *buffer. It supports buffer read
 * and continuous read modes.
 *
 * The Page Data Read instruction transfer data of the specified memory page
 * into the 2,112-Byte Data Buffer.
 * After the 2,112 bytes of page data are loaded into the Data Buffer, several
 * Read instructions can be issued to access the Data Buffer and read out the
 * data. Depending on the BUF bit setting in the Status Register,either
 * “Buffer Read Mode” or “Continuous Read Mode” may be used to accomplish the
 * read operations.
 *
 * Needs:
 * - page number (address) 0..65,535
 * - if ECC enables (default) buffer size = aligned to page size (n * 2,048 bytes)
 * -mode:
 *   0 = standard Read Data (0x03)
 *   1 = Fast Read (0x0B)
 *   2 = Fast Read with 4-Byte Address (0x0C)
 ****************************************************************************/
static int w25n01_page_read(FAR struct w25n01_dev_s *priv, uint16_t page,
						   uint16_t column_addr, FAR uint8_t *buffer,
						   size_t buflen, uint8_t mode)
{
	int ret;
	off_t block;

	DEBUGASSERT(buflen % (W25N01_PAGE_MASK) == 0); // n*2048

	finfo("page: %08lx buflen: %d\n", (long)page, (int)buflen);

	/* Calculate block number */
	block = page / W25N01_PAGES_PER_BLOCK;

	/* Check if block is bad */
	if (w25n01_is_bad_block(priv, (uint16_t)block))
	{
		fwarn("Block %ld is marked bad, cannot read page %ld\n", (long)block,
			(long)page);
		return -EIO;
	}

	/* Wait for device ready */
	ret = w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
	if (ret < 0)
	{
		return ret;
	}

	/* 1) transfert data from specified page addr to data buffer */
	/* Send "Page Data Read" command */
	w25n01_page_data_read(priv, page);

	/* 2) Access data buffer and read out the data */
	w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
	if (mode == 0)
	{
		/* standard Read Data */
		w25n01_read_data(priv, column_addr, buffer, buflen);
	}
	else if (mode == 1)
	{
		/* Fast Read */
		w25n01_fast_read(priv, column_addr, buffer, buflen);
	}
	else if (mode == 2)
	{
		/* Fast Read with 4-Byte Address */
		w25n01_fast_read_4b(priv, column_addr, buffer, buflen);
	}
	else
	{
		ferr("ERROR: Invalid read mode %d\n", mode);
		return -EINVAL;
	}

	return OK;
}

/****************************************************************************
 * Name: w25n01_page_write
 *
 * (p. 36, 38) The Program Data Load instruction allows from one byte to 2,112 bytes
 * (a page) of data to be programmed at previously erased (FFh) memory
 * locations. A Program operation involves two steps: 1. Load the program data
 * into the Data Buffer. 2. Issue “Program Execute” command to transfer the
 * data from Data Buffer to the specified memory page.
 *
  * “Load Program Data” instruction will reset the unused the data bytes in the
 * Data Buffer to FFh value, while “Random Load Program Data” instruction will
 * only update the data bytes that are specified by the command input sequence,
 * the rest of the Data Buffer will remain unchanged.
 *
 ****************************************************************************/
#ifndef CONFIG_W25N01_READONLY
static int w25n01_page_write(FAR struct w25n01_dev_s *priv, uint16_t page,
							FAR const uint8_t *data, size_t datalen,
							bool random)
{
	int ret;
	off_t block;

	/* in each page (2048 + 64 B) there are 4 sectors of 512 Bytes and 4
	 * associated spares of 16 Bytes.
	 * the coulumn address CA[11:0]is the address of the sector/spare within
	 * the page.
	 *
	 * sector 0 : CA[11:0] = 000h - 1FFh
	 * ...
	 * sector 3 : CA[11:0] = 600h - 7FFh
	 * spare 0  : CA[11:0] = 800h - 80Fh
	 * ...
	 * spare 3  : CA[11:0] = 830h - 83Fh
	 */
	uint16_t column_addr=0; // write full page starting at CA[0] = 000h

	finfo("page: %08lx datalen: %d\n", (long)page, (int)datalen);

	/* Calculate block number */
	block = page / W25N01_PAGES_PER_BLOCK;
	// column_addr = (uint16_t)(block << W25N01_BLOCK2PAGE_SHIFT);

	/* Check if block is bad */
	if (w25n01_is_bad_block(priv, (uint16_t)block))
	{
		fwarn("Block %ld is marked bad, cannot write page %ld\n", (long)block,
			(long)page);
		return -EIO;
	}

	/* Wait for device ready */
	ret = w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
	if (ret < 0)
	{
		return ret;
	}

	/* Enable write before Load Program Data Instructions */
	w25n01_write_enable(priv);

	/* Load data into buffer */
	/* Send "Program Data Load" command p. 25/36 */
	// SPI_SELECT(priv->spi, SPIDEV_FLASH(priv->devid), true);
	// if (random)
	// {
	// 	// SPI_SEND(priv->spi, W25N01_RAND_PROGRAM_DATA_LOAD);
	// 	w25n01_rand_program_data_load(priv, column_addr, data, datalen);
	// }
	// else
	// {
		// SPI_SEND(priv->spi, W25N01_PROGRAM_DATA_LOAD);
		w25n01_program_data_load(priv, column_addr, data, datalen, true);
	// }

	/* Send "Program Execute" command p. 38 */
	w25n01_program_execute(priv, page);
	return datalen;
}
#endif

/****************************************************************************
 * Name: w25n01_byteread (deprecated) use instead w25n01_page_read
 *
 * need:
 * - byte address
 ****************************************************************************/
static void w25n01_byteread(FAR struct w25n01_dev_s *priv, FAR uint8_t *buffer,
							uint16_t address, size_t nbytes)
{
	int ret;
	off_t block;
	uint16_t page_addr;
	uint8_t column_addr = address & W25N01_PAGE_MASK;

	finfo("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);

	/* Calculate block number */
	block = address / W25N01_BLOCK_SIZE;
	page_addr = (uint16_t)(block << W25N01_BLOCK2PAGE_SHIFT);

	/* Check if block is bad */
	if (w25n01_is_bad_block(priv, (uint16_t)block))
	{
		fwarn("Block %ld is marked bad, cannot read address %08lx\n",
			(long)block, (long)address);
		return;
	}

	/* Wait for device ready */
	ret = w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
	if (ret < 0)
	{
		return;
	}

	/* Send Page Data Read then Read commands */
	w25n01_page_data_read(priv, page_addr);
	w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
	/* Send Read Data command */
	w25n01_read_data(priv, column_addr, buffer, nbytes);
}

#if defined(CONFIG_MTD_BYTE_WRITE) && !defined(CONFIG_W25N01_READONLY)
/****************************************************************************
 * Name: w25n01_bytewrite (deprecated) use instead w25n01_page_write
 *
 * need:
 * - byte address
 ****************************************************************************/
static void w25n01_bytewrite(FAR struct w25n01_dev_s *priv,
							FAR const uint8_t *buffer,
							uint32_t address, size_t nbytes)
{
	int ret;
	off_t block;
	uint16_t page_addr;
	uint16_t column_addr;

	finfo("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);

	/* Calculate block number */
	block = address / W25N01_BLOCK_SIZE;
	page_addr = (uint16_t)(block << W25N01_BLOCK2PAGE_SHIFT);
	column_addr = address & W25N01_PAGE_MASK;

	/* Check if block is bad */
	if (w25n01_is_bad_block(priv, (uint16_t)block))
	{
		fwarn("Block %ld is marked bad, cannot write address %08lx\n",
			(long)block, (long)address);
		return;
	}

	/* Wait for device ready */
	ret = w25n01_wait_ready(priv, W25N01_DEFAULT_TIMEOUT_MS);
	if (ret < 0)
	{
		return;
	}
	/* Write enable */
	w25n01_write_enable(priv);
	/* Load data into buffer */
	w25n01_program_data_load(priv, column_addr, buffer, nbytes, true);
	/* Send "Program Execute" command p. 38 */
	w25n01_program_execute(priv, page_addr);
}
#endif

/* MTD driver methods */

/****************************************************************************
 * Name: w25n01_erase
 ****************************************************************************/
static int w25n01_erase(FAR struct mtd_dev_s *dev, off_t startblock,
						size_t nblocks)
{
#ifdef CONFIG_W25N01_READONLY
	return -EACESS
#else
	FAR struct w25n01_dev_s *priv = (FAR struct w25n01_dev_s *)dev;
	// uint16_t blockleft = nblocks;
	uint16_t blk;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	/* Lock the SPI bus until we complete the erase*/
	w25n01_lock(priv->spi);
	for (int blockleft = nblocks; blockleft > 0; blockleft--)
	{
		blk = (uint16_t)(startblock + (nblocks - blockleft));
		/* Check if block is bad */
		if (w25n01_is_bad_block(priv, blk))
		{
			fwarn("Skipping bad block %d\n", blk);
			continue;
		}
		/* Erase each block */
		w25n01_block_erase(priv, blk);
	}
	w25n01_unlock(priv->spi);
	return (int)nblocks;
#endif /* CONFIG_W25N01_READONLY */
}

/****************************************************************************
 * Name: w25n01_bread
 ****************************************************************************/
static ssize_t w25n01_bread(FAR struct mtd_dev_s *dev,
							 off_t startblock,
							 size_t nblocks,
							 FAR uint8_t *buffer)
{
	ssize_t nbytes;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	/* On this device, we can handle the block read just like the byte-oriented
	 * read
	 */

	nbytes = w25n01_read(dev, startblock << W25N01_BLOCK_SHIFT,
						nblocks << W25N01_BLOCK_SHIFT, buffer);
	if (nbytes > 0)
	{
		nbytes >>= W25N01_BLOCK_SHIFT;
	}
	return nbytes;
}

/****************************************************************************
 * Name: w25n01_bwrite
 ****************************************************************************/
static ssize_t w25n01_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
							size_t nblocks, FAR const uint8_t *buffer)
{
#ifdef CONFIG_W25N01_READONLY
	return -EROFS;
#else
	FAR struct w25n01_dev_s *priv = (FAR struct w25n01_dev_s *)dev;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
	int page = startblock << W25N01_BLOCK2PAGE_SHIFT;
	// int block;
	// int pagesperblock = W25N01_PAGES_PER_BLOCK;
	// int ret;

	/* Lock the SPI bus and write all of the pages to FLASH */
	w25n01_lock(priv->spi);

	w25n01_page_write(priv, page, buffer,
					  nblocks << W25N01_BLOCK2PAGE_SHIFT, true);
	w25n01_unlock(priv->spi);

	return nblocks;
#endif /* CONFIG_W25N01_READONLY */
}

/****************************************************************************
 * Name: w25n01_read
 ****************************************************************************/
static ssize_t w25n01_read(FAR struct mtd_dev_s *dev, off_t offset,
						 size_t nbytes, FAR uint8_t *buffer)
{
	FAR struct w25n01_dev_s *priv = (FAR struct w25n01_dev_s *)dev;

	finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

	/* Lock the SPI bus and select this FLASH part */

	w25n01_lock(priv->spi);
	w25n01_byteread(priv, buffer, offset, nbytes);
	w25n01_unlock(priv->spi);

	finfo("return nbytes: %d\n", (int)nbytes);
	return nbytes;
}


/****************************************************************************
 * Name: w25n01_write
 ****************************************************************************/
#if defined(CONFIG_MTD_BYTE_WRITE) && !defined(CONFIG_W25N01_READONLY)
static ssize_t w25n01_write(FAR struct mtd_dev_s *dev, off_t offset,
							size_t nbytes, FAR const uint8_t *buffer)
{
	FAR struct w25n01_dev_s *priv = (FAR struct w25n01_dev_s *)dev;
	int startpage;
	int endpage;
	int count;
	int index;
	int bytestowrite;

	finfo("offset: %08lx nbytes: %u\n", (long)offset, (int)nbytes);

	/* We must test if the offset + count crosses one or more pages
	* and perform individual writes.  The devices can only write in
	* page increments.
	*/

	startpage = offset / W25N01_PAGE_SIZE;
	endpage = (offset + nbytes) / W25N01_PAGE_SIZE;

	w25n01_lock(priv->spi);
	if (startpage == endpage)
	{
		/* All bytes within one programmable page.  Just do the write. */

		w25n01_bytewrite(priv, buffer, offset, nbytes);
	}
	else
	{
		/* Write the 1st partial-page */

		count = nbytes;
		bytestowrite = W25N01_PAGE_SIZE - (offset & (W25N01_PAGE_SIZE - 1));
		w25n01_bytewrite(priv, buffer, offset, bytestowrite);

		/* Update offset and count */

		offset += bytestowrite;
		count -=  bytestowrite;
		index = bytestowrite;

		/* Write full pages */

		while (count >= W25N01_PAGE_SIZE)
		{
			w25n01_bytewrite(priv, &buffer[index], offset, W25N01_PAGE_SIZE);

			/* Update offset and count */

			offset += W25N01_PAGE_SIZE;
			count -= W25N01_PAGE_SIZE;
			index += W25N01_PAGE_SIZE;
		}

		/* Now write any partial page at the end */

		if (count > 0)
		{
			w25n01_bytewrite(priv, &buffer[index], offset, count);
		}
	}

	w25n01_unlock(priv->spi);
	return nbytes;
}
#endif /* defined(CONFIG_MTD_BYTE_WRITE) && !defined(CONFIG_W25_READONLY) */


/****************************************************************************
 * Name: w25n01_ioctl
 ****************************************************************************/
static int w25n01_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
	FAR struct w25n01_dev_s *priv = (FAR struct w25n01_dev_s *)dev;
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
				* of fixed size blocks. That is most likely not true, but the
				* client will expect the device logic to do whatever is
				* necessary to make it appear so.
				*/

				geo->blocksize    = W25N01_PAGE_SIZE;	/* smallest r/w unit */
				geo->erasesize    = W25N01_BLOCK_SIZE; /* smallest erassable unit */
				geo->neraseblocks = W25N01_BLOCKS;
				geo->nbadblocks  = priv->nbadblocks * W25N01_PAGES_PER_BLOCK;
				ret               = OK;

				finfo("blocksize: %" PRIu32 " erasesize: %" PRIu32
					" neraseblocks: %" PRIu32 "\n",
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
				info->numsectors  = W25N01_BLOCKS * W25N01_PAGES_PER_BLOCK;
				info->sectorsize  = W25N01_PAGE_SIZE;
				info->startsector = 0;
				info->parent[0]   = '\0';
				ret               = OK;
			}
		}
		break;

		case MTDIOC_BULKERASE:
		{
			/* Erase the entire device */

			w25n01_lock(priv->spi);
			ret = w25n01_chip_erase(priv);
			w25n01_unlock(priv->spi);
		}
		break;

		case MTDIOC_ERASESTATE:
		{
			FAR uint8_t *result = (FAR uint8_t *)arg;
			*result = W25N01_ERASED_STATE;

			ret = OK;
		}
		break;

		default:
		ret = -ENOTTY; /* Bad command */
		break;
	}

	finfo("return %d\n", ret);
	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: w25n01_initialize
 *
 * Description:
 *   Create an initialize MTD device instance. MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ****************************************************************************/
FAR struct mtd_dev_s *w25n01_initialize(FAR struct spi_dev_s *dev,
                                      uint32_t spi_devid)
{
	FAR struct w25n01_dev_s *priv;
	int ret;

	finfo("spi: %p spi_devid: %lu\n", dev, (unsigned long)spi_devid);

	/* Allocate a state structure (we allocate the structure instead of using
	* a fixed, static allocation so that we can handle multiple FLASH devices.
	* The current implementation would handle only one FLASH part per SPI
	* device (only because of the SPIDEV_FLASH(priv->devid) definition) and so would
	* have to be extended to handle multiple FLASH parts on the same SPI bus.
	*/

	priv = (FAR struct w25n01_dev_s *)kmm_zalloc(sizeof(struct w25n01_dev_s));
	if (!priv)
    {
		ferr("ERROR: Failed to allocate device structure\n");
		return NULL;
    }

	/* Initialize device structure */
	priv->mtd.name			= "w25n01";
	priv->spi				= dev;
	priv->devid				= spi_devid;
	// pageshift, blockshift and nsectrors are set in w25n01_readid() for W25N01
	// priv->geom.pageshift	= W25N01_PAGE_SHIFT;  /* 2048 = 2^11 */
	// priv->geom.blockshift	= W25N01_BLOCK_SHIFT; /* 128KB = 2^17 (64 pages * 2048 bytes) */
	// priv->nsectors			= W25N01_BLOCKS; /* Number of erasable sectors */
	priv->initialized		= false;

	/* Allocate a one-byte buffer to support DMA-able status read data */
	priv->readbuf = (FAR uint8_t *)kmm_malloc(2);
	if (!priv->readbuf)
	{
		ferr("ERROR: Failed to allocate read buffer\n");
		kmm_free(priv);
		return NULL;
	}

	/* Set up MTD interface */
	priv->mtd.erase  = w25n01_erase;
	priv->mtd.bread  = w25n01_bread;
	priv->mtd.bwrite = w25n01_bwrite;
	priv->mtd.ioctl  = w25n01_ioctl;
	priv->mtd.read   = w25n01_read;
#if defined(CONFIG_MTD_BYTE_WRITE) && !defined(CONFIG_W25N01_READONLY)
	priv->mtd.write  = w25n01_write;
#endif

	// SPI_SELECT(priv->spi, SPIDEV_FLASH(spi_devid), false);

	/* Reset device */
	w25n01_reset(priv); //lock + config + deselect
	w25n01_unlock(priv->spi);

	/* Identify the FLASH chip and get its capacity */
	ret = w25n01_readid(priv);
	if (ret != OK)
	{
		/* Unrecognized! Discard all of that work we just did and
		* return NULL
		*/
		w25n01_unlock(priv->spi);
		ferr("ERROR: Unrecognized\n");
		kmm_free(priv);
		return NULL;
	}

	/* Make sure that the FLASH is unprotected so that we can write
	 * into it.
	 */
#ifndef CONFIG_W25N01_READONLY
		w25n01_unprotect(priv);
#endif

	/* Scan for bad blocks */
	ret = w25n01_scan_bad_blocks(priv);
	if (ret < 0)
	{
		ferr("ERROR: Failed to scan bad blocks\n");
		w25n01_unlock(priv->spi);
		if (priv->bbm)
		{
			kmm_free(priv->bbm);
		}
		kmm_free(priv);
		return NULL;
	}

	w25n01_unlock(priv->spi);

	priv->initialized = true;

	/* Return the implementation-specific state structure as the MTD device */

	finfo("W25N01 initialized successfully\n");
	finfo("Total blocks: %u, Bad blocks: %u\n",
		W25N01_BLOCKS, priv->nbadblocks);

	// UNUSED(ret);
	return &priv->mtd;
}

