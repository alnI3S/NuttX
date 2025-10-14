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

#define W25N01GV_DEFAULT_TIMEOUT_MS         5000  // wait ready timeout
#define W25N01GV_TIMEOUT_PAGE_READ_US        60   // tREmax = 60us (ECC enabled)
#define W25N01GV_TIMEOUT_PAGE_PROGRAM_US     700  // tPPmax = 700us
#define W25N01GV_TIMEOUT_BLOCK_ERASE_MS      10   // tBEmax = 10ms
#define W25N01GV_TIMEOUT_RESET_MS            500  // tRSTmax = 500ms


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
#define W25N01GV_LAST_EEC_FAIL_PAGE_ADDR       	0xA9	/* Last ECC failure page address	*/
#define W25N01GV_BLOCK_ERASE 		0xD8	/* Block Erase (64 KB)    */

#define W25N01GV_PROGRAM_DATA_LOAD   		0x02	/* reset buffer	*/
#define W25N01GV_RAND_PROGRAM_DATA_LOAD 	0x84 	/* Random Program Data Load */
#define W25N01GV_PROGRAM_DATA_LOAD_QUAD 	0x32  	/* Quad reset buffer             */
#define W25N01GV_RAND_PROGRAM_DATA_LOAD_QUAD 	0x34  	/* quad Random Program Data Load           */

#define W25N01GV_PROGRAM_EXECUTE 	0x10	/* Program Execute	*/

#define W25N01GV_PAGE_DATA_READ    	0x13	/* Read Page Data	*/
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
#define W25N01GV_PROT_ADDR          0xA0 	/* Protection SR-1, r/w	*/
#define W25N01GV_CONF_ADDR          0xB0	/* Configure SR-2, r/w	*/
#define W25N01GV_STATUS_ADDR		0xC0	/* Status SR-3, read only	*/

/* Status register 1 bit definitions                                      */
/* see p. 16-17
* Bit  |   7  |  6  |  5  |  4  |  3  |  2  |   1  |   0  |
*	--------------------------------------------------------------
*	   | SRP0 | BP3 | BP2 | BP1 | BP0 | TB  | WP-E | SRP1 |
*/
#define STATUS_SRP1_MASK     		(1 << 0) /* Bit 0: Status register protect-1  */
#define STATUS_SRP1_UNLOCKED  		(0 << 0) /*   see blow for details           */
#define STATUS_SRP1_LOCKED    		(1 << 0) /*   see blow for details           */
#define STATUS_WPE_MASK      		(1 << 1) /* Bit 1: /WP enable bit */
#define STATUS_WPE_UNLOCKED 		(0 << 1) /* 0 + SRP1 + SRP0 = Software protection         */
#define STATUS_WPE_LOCKED   		(1 << 1) /* 1 + SRP1 + SRP0 = Hardware protection              */
#define STATUS_TB_MASK       		(1 << 2) /* Bit 2: Top / Bottom Protect      */
#define STATUS_TB_TOP        		(0 << 2) /*   0 = BP3-BP0 protect Top down   */
#define STATUS_TB_BOTTOM     		(1 << 2) /*   1 = BP3-BP0 protect Bottom up  */
#define STATUS_BP_SHIFT      		(3)      /* Bits 3-6: 4 Block protect bits     */
#define STATUS_BP_4_MASK     		(15 << STATUS_BP_SHIFT)	/* all 1111 for BP0-3 */
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
*    0      X      1      VCC  	Hardware Protected    	SR-1 can be changed
*
*    1      0      1      VCC              			Power Lock-Down  SR-1
*
*    1      1      1      VCC               			Enter OTP mode to protect SR-1 (allow SR1-L=1)
*
*    X      X      1      GND  				All "Write/Program/Erase" commands are blocked
* 								Entire device (SRs, Array, OTP area) is read-only
*/
#define STATUS_BP_NONE       		(0 << STATUS_BP_SHIFT)	/* all 0000 for BP0-3 */
#define STATUS_BP_ALL        		(31 << STATUS_TB_MASK)	/* all 11111 for TB-BP3 */
#define STATUS_SRP0_MASK      		(1 << 7) /* Bit 7: Status register protect-0 */
#define STATUS_SRP0_UNLOCKED  		(0 << 7) /*   see above for details           */
#define STATUS_SRP0_LOCKED    		(1 << 7) /*   see above for details           */

/* Status register 2 bit definitions                                      */
/* see p. 18-19
* Bit  |   7   |   6   |   5   |   4   |  3  |  2  |  1  |  0  |
* 	--------------------------------------------------------------
* 	   | OTP-L | OTP-E | SR1-L | ECC-E | BUF | (R) | (R) | (R) |
*/
#define STATUS2_READ_MODE_MASK      (3 << 3) /* Bit 3-4: Read Mode          */
#define STATUS2_CONTINUOUS_READ  	(0 << 3) /*  00 = Continuous read, Output 2048 */
#define STATUS2_BUFFER_READ   		(1 << 3) /*  01 = Buffer read, Output 2048 + 64 */
#define STATUS2_CONTINUOUS_READ_OP  (2 << 3) /*  10 = Continuous read, Operation based, Output 2048 + 64 */
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
* 	   | (R) | LUT-F | ECC-1 | ECC-0 | P-FAIL | E-FAIL | WEL | BUSY |
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
#define STATUS3_LUTF_MASK 		(1 << 6) /* Bit 6: LUT (failure) Full flag      */
#define STATUS3_LUTF       		(0 << 6) /*   0 = default, no full              */
#define STATUS3_LUTF_FULL  		(1 << 6) /*   1 = no more memory block links may be established. */

/* Chip Geometries **********************************************************/
/* W25N01 (128 MB (1 Gb)) memory capacity */

#define W25N01GV_BLOCKS          1024     /* 1024 * 128KiB = 128MiB */
#define W25N01GV_PAGES_PER_BLK   64
#define W25N01GV_PAGE_SIZE       2048
#define W25N01GV_OOB_SIZE        64
#define W25N01GV_CACHE_READ_DUMMY 8       /* per datasheet for 6Bh (x4) */ /* TODO: confirm */

/* Bad Blocks ***************************************************************/
#define W25N01GV_BBM_MAX_ENTRIES   20   /* max entries in LUT per datasheet */
#define W25N01GV_BLOCK_ADDR_MASK   0x03FF  /* 10-bit block address field, LBA/PBA[9:0]*/

/* Chip IDs *****************************************************************/
#define JEDEC_WINBOND_ID   	    0xEF  /* Winbond W25n01GV Serial Flash */
#define W25N01GV_DEVICE_ID	    0x21
#define W25N01GV_MEMORY_TYPE	0xAA

/* NAND erase sets all cells in the block to 1s
*   A freshly erased page (2,048 bytes + 64 spare) will read back as all 0xFF bytes.
*   Programming changes bits from 1 → 0, but you can’t flip them back to 1 without an erase.
*/
#define W25N01GV_ERASED_STATE   0xFF

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
struct w25n01gv_geometry_s {
	uint32_t page_size_bytes;       // 2048 (data) + 64 (spare)
	uint32_t spare_size_bytes;      // 64
	uint32_t block_size;            // 64 pages per block
	uint32_t block_size_bytes;      // 131072 (64*2048)
	uint16_t total_blocks;          // 1024
	uint8_t blockshift;             /* Log2 of block size */
	uint8_t pageshift;              /* Log2 of page size */
};

struct w25n01gv_bbm_entry_s {
	uint16_t bad_block;   /* original bad block */
	uint16_t good_block;  /* replacement block */
};


/* This type represents the state of the MTD device. The struct mtd_dev_s
* must appear at the beginning of the definition so that you can freely
* cast between pointers to struct mtd_dev_s and struct w25n01gv_dev_s.
*/
struct w25n01gv_dev_s
{
	struct mtd_dev_s       	mtd;         /* MTD interface */
	FAR struct qspi_dev_s 	*qspi;       /* Saved QuadSPI interface instance */
	struct w25n01gv_geometry_s 	geom;         /* Geometry of the flash */
	struct w25n01gv_bbm_entry_s 	bbm[W25N01GV_BBM_MAX_ENTRIES]; /* Bad block table */
	int 					bbm_count;   /* Number of bad blocks */
	// uint16_t               	nsectors;    /* Number of erase sectors */
	uint8_t                	protectmask; /* Mask for protect bits in status register */
	uint8_t                	tbmask;      /* Mask for top/bottom bit in status register */
	FAR uint8_t           	*cmdbuf;      /* Allocated command buffer */
	FAR uint8_t           	*readbuf;     /* Allocated status read buffer */
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

/* Locking */

static void w25n01gv_lock(FAR struct qspi_dev_s *qspi);
static inline void w25n01gv_unlock(FAR struct qspi_dev_s *qspi);

/* Low-level message helpers */

// QSPI helpers
static int  w25n01gv_command(FAR struct qspi_dev_s *qspi, uint8_t cmd);
static int  w25n01gv_command_address(FAR struct qspi_dev_s *qspi, uint8_t cmd,
									uint32_t addr, uint8_t addrlen,
									FAR const void *buffer, size_t buflen);
static int  w25n01gv_command_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
									FAR const void *buffer, size_t buflen);
static int w25n01gv_command_memory(FAR struct qspi_dev_s *qspi,
                                    uint8_t cmd, uint8_t flag, uint32_t addr,
                                    uint8_t dummy_cycles,
                                    FAR uint8_t *buffer, size_t buflen);
// W25N01GV helpers
static int  w25n01gv_command_data_load(FAR struct w25n01gv_dev_s *priv,
                                    uint8_t cmd, uint8_t flag, uint32_t addr,
                                    FAR const uint8_t *buffer, size_t buflen);
static uint8_t w25n01gv_get_read_mode(FAR struct w25n01gv_dev_s *priv);

static int w25n01gv_wait_ready(struct w25n01gv_dev_s *priv, uint32_t timeout_ms);

// TODO : add check LUT-F is_bad_blocks_full()
static int  w25n01gv_protect(FAR struct w25n01gv_dev_s *priv,
							off_t startblock, size_t nblocks);
static int  w25n01gv_unprotect(FAR struct w25n01gv_dev_s *priv);
static bool w25n01gv_isprotected(FAR struct w25n01gv_dev_s *priv,
							uint8_t status, off_t blockaddr);
static int  w25n01gv_erase_block(FAR struct w25n01gv_dev_s *priv, off_t block);
static int  w25n01gv_erase_chip(FAR struct w25n01gv_dev_s *priv);
static int  w25n01gv_read_byte(FAR struct w25n01gv_dev_s *priv,
							FAR uint8_t *buffer, off_t address, size_t nbytes);
static int  w25n01gv_write_page(FAR struct w25n01gv_dev_s *priv,
							FAR const uint8_t *buffer,
							off_t address,
							size_t nbytes);
// TODO check if needed
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

/* w25n01gv commands	*/
// TODO void vs int if return OK or error
static void w25n01gv_reset(FAR struct w25n01gv_dev_s *priv);
static int  w25n01gv_readid(FAR struct w25n01gv_dev_s *priv);
static uint8_t w25n01gv_read_status(FAR struct w25n01gv_dev_s *priv,
									uint32_t status_addr);
static void w25n01gv_write_status(FAR struct w25n01gv_dev_s *priv,
									uint32_t status_addr, uint8_t reg);
static void w25n01gv_write_enable(FAR struct w25n01gv_dev_s *priv);
static void w25n01gv_write_disable(FAR struct w25n01gv_dev_s *priv);
static void w25n01gv_bad_block_management(FAR struct w25n01gv_dev_s *priv,
									uint16_t lba, uint16_t pba);
static int w25n01gv_read_bbm_lut(FAR struct w25n01gv_dev_s *priv);
static uint16_t w25n01gv_last_ecc_failure_page_addr(FAR struct w25n01gv_dev_s *priv);
static int w25n01gv_block_erase(FAR struct w25n01gv_dev_s *priv, uint32_t block);
static int w25n01gv_program_data_load(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR const uint8_t *buffer, size_t buflen);
static int w25n01gv_rand_program_data_load(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR const uint8_t *buffer, size_t buflen);
static int w25n01gv_program_data_load_quad(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR const uint8_t *buffer, size_t buflen);
static int w25n01gv_rand_program_data_load_quad(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR const uint8_t *buffer, size_t buflen);
static int w25n01gv_program_execute(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr);
static int w25n01gv_page_data_read(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr);
static int w25n01gv_read_data(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer, size_t buflen);
static int w25n01gv_fast_read(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer, size_t buflen);
static int w25n01gv_fast_read_4b(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer,
                                    size_t buflen);
static int w25n01gv_fast_read_dual(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer,
                                    size_t buflen);
static int w25n01gv_fast_read_dual_4b(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer,
                                    size_t buflen);
static int w25n01gv_fast_read_quad(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer,
                                    size_t buflen);
static int w25n01gv_fast_read_quad_4b(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer,
                                    size_t buflen);
static int w25n01gv_fast_read_dualio(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer,
                                    size_t buflen);
static int w25n01gv_fast_read_dualio_4b(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer,
                                    size_t buflen);
static int w25n01gv_fast_read_quadio(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer,
                                    size_t buflen);
static int w25n01gv_fast_read_quadio_4b(FAR struct w25n01gv_dev_s *priv,
                                    uint32_t addr,
                                    FAR uint8_t *buffer,
                                    size_t buflen);
/* bad locks management */
uint16_t w25n_remap_block(FAR struct w25n01gv_dev_s *priv, uint16_t lba)
{
    for (int i = 0; i < priv->bbm_count; i++)
    {
        if (priv->bbm[i].bad_block == lba) return priv->bbm[i].good_block;
    }
    return lba;
}

/* Very simple: scan PBAs from high blocks downwards and skip those already used */
int find_free_pba(FAR struct w25n01gv_dev_s *priv, uint16_t *out_pba)
{
    for (uint16_t p = priv->geom.total_blocks - 1; p >= 0; p--)
    {
        bool used = false;
        for (int i = 0; i < priv->bbm_count; i++)
        {
            if (priv->bbm[i].good_block == p || priv->bbm[i].bad_block == p) { used = true; break; }
        }
        if (!used)
        {
            *out_pba = p;
            return 0;
        }
        if (p == 0) break;
    }
    return -ENOSPC;
}

/* Called when an erase/program failed for logical block `lba` */
int handle_bad_block(FAR struct w25n01gv_dev_s *priv, uint16_t lba)
{
    uint16_t new_pba;
    int ret = find_free_pba(priv, &new_pba);
    if (ret < 0) return ret; /* no spare left */

    /* Optionally copy valid pages from old block to new block:
     * for each page in block:
     *   read page from old physical block (may fail)
     *   write to new physical block
     */

    /* Swap Blocks A1h: LBA -> new_pba */
    w25n01gv_bad_block_management(priv, lba, new_pba);

    /* Update in-memory LUT */
    priv->bbm[priv->bbm_count].bad_block = lba;
    priv->bbm[priv->bbm_count].good_block = new_pba;
    priv->bbm_count++;

    /* Re-try the erase/program on the new_pba */
    return OK;
}

/* MTD driver methods */

static int  w25n01gv_erase(FAR struct mtd_dev_s *dev,
							off_t startblock, size_t nblocks);
static ssize_t w25n01gv_bread(FAR struct mtd_dev_s *dev,
							off_t startblock, size_t nblocks, FAR uint8_t *buf);
static ssize_t w25n01gv_bwrite(FAR struct mtd_dev_s *dev,
							off_t startblock, size_t nblocks,
							FAR const uint8_t *buf);
static ssize_t w25n01gv_read(FAR struct mtd_dev_s *dev,
							off_t offset, size_t nbytes, FAR uint8_t *buffer);
static int  w25n01gv_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

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
	uint32_t addr, uint8_t addrlen,
	FAR const void *buffer, size_t buflen)
{
	struct qspi_cmdinfo_s cmdinfo;

	finfo("CMD: %02x Address: %04lx addrlen=%d\n",
		cmd,
		(unsigned long)addr,
		addrlen);

	cmdinfo.flags   = QSPICMD_ADDRESS;
	cmdinfo.addrlen = addrlen;
	cmdinfo.cmd     = cmd;
	cmdinfo.buflen  = buflen;
	cmdinfo.addr    = addr;
	cmdinfo.buffer  = (FAR void *)buffer;

	return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
* Name: w25n01gv_command_write
* TODO check if needed
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
* Name: w25n01gv_command_memory_read
* helper for memory read commands (03h, 0Bh, 0Ch, 3Bh, 3Ch, 6Bh, 6Ch, BBh, BCh,
*   EBh, ECh)
****************************************************************************/
static int w25n01gv_command_memory(FAR struct qspi_dev_s *qspi,
    uint8_t cmd, uint8_t flag, uint32_t addr, uint8_t dummycycles,
    FAR uint8_t *buffer, size_t buflen)
{
    struct qspi_meminfo_s meminfo;

    finfo("CMD: %02x Address: %04lx buflen: %lu\n",
        cmd,
        (unsigned long)addr,
        (unsigned long)buflen);

    meminfo.flags    = flag;
    meminfo.addrlen  = 2;
    meminfo.dummies  = dummycycles;
    meminfo.cmd      = cmd;
    meminfo.buflen   = buflen;
    meminfo.addr     = addr;
    meminfo.key      = 0;
    meminfo.buffer   = buffer;

    return QSPI_MEMORY(qspi, &meminfo);
}

/****************************************************************************
* Name: w25n01gv_command_data_load
* helper for program data load commands (02h, 84h, 32h, 34h)
****************************************************************************/
static int w25n01gv_command_data_load(FAR struct w25n01gv_dev_s *priv,
    uint8_t cmd, uint8_t flag, uint32_t addr,
    FAR const uint8_t *buffer, size_t buflen)
{
    w25n01gv_write_enable(priv);
    struct qspi_cmdinfo_s cmdinfo;

    finfo("CMD: %02x Address: %04lx buflen: %lu\n",
        cmd,
        (unsigned long)addr,
        (unsigned long)buflen);

    cmdinfo.flags   =  QSPICMD_ADDRESS | QSPICMD_WRITEDATA | flag;
    cmdinfo.addrlen = 2; /* always 3 bytes address */
    cmdinfo.cmd     = cmd;
    cmdinfo.buflen  = buflen;
    cmdinfo.addr    = addr;
    cmdinfo.buffer  = (FAR void *)buffer;

    return QSPI_COMMAND(priv->qspi, &cmdinfo);
    w25n01gv_write_disable(priv);
}

/****************************************************************************
* Name: w25n01gv_get_read_mode
****************************************************************************/
static uint8_t w25n01gv_get_read_mode(FAR struct w25n01gv_dev_s *priv)
{
    uint8_t sr2 = w25n01gv_read_status(priv, W25N01GV_CONF_ADDR);
    uint8_t mode = sr2 & STATUS2_READ_MODE_MASK;

    return mode;
}

/****************************************************************************
* Name: w25n01gv_wait_ready
****************************************************************************/
static int w25n01gv_wait_ready(struct w25n01gv_dev_s *priv, uint32_t timeout_ms)
{
	uint8_t status;
	uint32_t elapsed = 0;

	do
	{
		status = w25n01gv_read_status(priv, W25N01GV_STATUS_ADDR);
		if ((status & STATUS3_BUSY_MASK) == STATUS3_READY)
		{
			return OK;
		}

		up_mdelay(100);
		elapsed += 100;
	} while (elapsed < timeout_ms);

	ferr("ERROR: Timeout waiting for ready\n");
	return -ETIMEDOUT;
}
/****************************************************************************
* Name: w25n01gv_protect
*   See datasheet p. 22 section 7.4 W25N01GV Status Register Memory Protection
*   for details.
*
****************************************************************************/
static int w25n01gv_protect(FAR struct w25n01gv_dev_s *priv,
	off_t startblock, size_t nblocks)
{
    uint8_t newstatus;
    uint8_t bp_bits = 0;
    uint8_t tb_bit  = 0;

    /* Get the SR-1 value to check the current protection */
    priv->cmdbuf[0] = w25n01gv_read_status(priv, W25N01GV_PROT_ADDR);

    /* Already full protection? */
    if ((priv->cmdbuf[0] & priv->protectmask) ==
    (STATUS_BP_ALL & priv->protectmask)) {
        return 0;
    }

    /* Protection must be aligned to power-of-two number of blocks */
    if ((nblocks & (nblocks - 1)) != 0){
        ferr("ERROR: Protection range must be power-of-two blocks\n");
        return -EINVAL;
    }

    /* Determine TB (top/bottom) */
    if (startblock == 0) {
        /* Protect bottom N blocks */
        tb_bit = STATUS_TB_BOTTOM;
    }
    else if (startblock + nblocks == W25N01GV_BLOCKS) {
        /* Protect top N blocks */
        tb_bit = STATUS_TB_TOP;
    }
    else {
        ferr("ERROR: Only top or bottom aligned protection supported\n");
        return -EINVAL;
    }

    /* Determine BP bits (log2 of nblocks) */
    switch (nblocks)
    {
        case 2:   bp_bits = (0x1 << STATUS_BP_SHIFT); break; /* 1/512 KiB */
        case 4:   bp_bits = (0x2 << STATUS_BP_SHIFT); break; /* 1/256 KiB */
        case 8:   bp_bits = (0x3 << STATUS_BP_SHIFT); break; /* 1/128 KiB */
        case 16:   bp_bits = (0x4 << STATUS_BP_SHIFT); break; /* 1/64 KiB */
        case 32:  bp_bits = (0x5 << STATUS_BP_SHIFT); break; /* 1/32 KiB */
        case 64:  bp_bits = (0x6 << STATUS_BP_SHIFT); break; /* 1/16 KiB */
        case 128:  bp_bits = (0x7 << STATUS_BP_SHIFT); break; /* 1/8 KiB */
        case 256: bp_bits = (0x8 << STATUS_BP_SHIFT); break; /* 1/4 KiB */
        case 512: bp_bits = (0x9 << STATUS_BP_SHIFT); break; /* 1/2 KiB */
        case 1024: bp_bits = (0xA << STATUS_BP_SHIFT); break; /* ALL */
        default:
            ferr("ERROR: Unsupported block count: %lu\n", (unsigned long)nblocks);
            return -EINVAL;
    }

    /* Update SR-1 */
    newstatus = (priv->cmdbuf[0] & ~(STATUS_BP_4_MASK | STATUS_TB_MASK)) |
                bp_bits | tb_bit;

    /* Write new SR-1 */
    w25n01gv_write_enable(priv);
    priv->cmdbuf[0] = newstatus;
    w25n01gv_command_write(priv->qspi, W25N01GV_WRITE_STATUS,
                            (FAR const void *)priv->cmdbuf, 1);
    w25n01gv_write_disable(priv);

    /* Verify */
    priv->cmdbuf[0] = w25n01gv_read_status(priv, W25N01GV_PROT_ADDR);
    if ((priv->cmdbuf[0] & (STATUS_BP_4_MASK | STATUS_TB_MASK)) !=
        (bp_bits | tb_bit))
    {
        ferr("ERROR: Protection not set correctly\n");
        return -EIO;
    }

    return OK;
}

/****************************************************************************
* Name: w25n01gv_unprotect
****************************************************************************/

static int w25n01gv_unprotect(FAR struct w25n01gv_dev_s *priv)
{
    /* Get the SR-1 value to check the current protection */
    priv->cmdbuf[0] = w25n01gv_read_status(priv, W25N01GV_PROT_ADDR);

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
    w25n01gv_write_status(priv, W25N01GV_PROT_ADDR, priv->cmdbuf[0]);

    /* Check the new status */
    priv->cmdbuf[0] = w25n01gv_read_status(priv, W25N01GV_PROT_ADDR);
    if ((priv->cmdbuf[0] & (STATUS_SRP1_MASK | priv->protectmask)) != 0)
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
	off_t blockaddr)
{
    uint8_t bp  = (status & STATUS_BP_4_MASK) >> STATUS_BP_SHIFT; /* Extract BP[3:0] */
    bool tb     = (status & STATUS_TB_MASK) != 0;     /* Top/Bottom select */
    off_t protect_start = 0;
    off_t protect_end   = priv->geom.total_blocks - 1;

    /* No protection (BP = 0000) */
    if (bp == 0) {
        return false;
    }

    /* Full protection (BP = 1111) */
    if (w25n01gv_bp_divisor[bp] == 1) {
        return true;
    }

    /* Partial protection */
    uint16_t divisor = w25n01gv_bp_divisor[bp];
    off_t nprotect   = priv->geom.total_blocks / divisor;

    if (tb)
    {
        /* Protect TOP region */
        protect_start = priv->geom.total_blocks - nprotect;
    }
    else
    {
        /* Protect BOTTOM region */
        protect_end = nprotect - 1;
    }

    return (blockaddr >= protect_start && blockaddr <= protect_end);
}

/****************************************************************************
* Name:  w25n01gv_erase_block
*
* TODO: so confuse with the native w25n01gv_block_erase() function (D8h).
****************************************************************************/
static int w25n01gv_erase_block(FAR struct w25n01gv_dev_s *priv,
	off_t block)
{
    off_t blockaddr;
    uint8_t status;

    finfo("block: %08lx\n", (unsigned long)block);

    /* Check that the flash is ready and unprotected */
    status = w25n01gv_read_status(priv, W25N01GV_STATUS_ADDR);
    if ((status & STATUS3_BUSY_MASK) != STATUS3_READY) {
        ferr("ERROR: Flash busy: %02x", status);
        return -EBUSY;
    }

    /* Get the address associated with the block */
    blockaddr = (off_t)block << priv->geom.blockshift;

    if ((status & priv->protectmask) != 0 &&
    w25n01gv_isprotected(priv, status, blockaddr))
    {
        ferr("ERROR: Flash protected: %02x", status);
        return -EACCES;
    }

    /* Send the block erase command */
    w25n01gv_block_erase(priv, blockaddr);

    /* Wait for erasure to finish */
    while ((w25n01gv_read_status(priv, W25N01GV_STATUS_ADDR) & STATUS3_BUSY_MASK) != 0);

    return OK;
}

/****************************************************************************
* Name:  w25n01gv_erase_chip
****************************************************************************/
static int w25n01gv_erase_chip(FAR struct w25n01gv_dev_s *priv)
{
    int ret = OK;

    finfo("Chip erase: %u blocks\n", priv->geom.total_blocks);

    for (uint32_t blk = 0; blk < priv->geom.total_blocks; blk++) {
        ret = w25n01gv_erase_block(priv, blk);
        if (ret < 0)
        {
            ferr("ERROR: Chip erase stopped at block %lu\n", (unsigned long)blk);
            return ret;
        }
    }

    return OK;
}

/****************************************************************************
* Name: w25n01gv_read_byte
****************************************************************************/
static int w25n01gv_read_byte(FAR struct w25n01gv_dev_s *priv,
	FAR uint8_t *buffer, off_t address, size_t buflen)
{
    return w25n01gv_fast_read_quadio(priv, address, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_write_page
****************************************************************************/
static int w25n01gv_write_page(struct w25n01gv_dev_s *priv,
	FAR const uint8_t *buffer,
	off_t address, size_t buflen)
{
    unsigned int pagesize;
    unsigned int npages;
    int ret;
    int i;

    finfo("address: %08lx buflen: %u\n",
        (unsigned long)address,
        (unsigned)buflen);

    npages   = (buflen >> priv->geom.pageshift);
    pagesize = (1 << priv->geom.pageshift);

    /* Write each page */
    for (i = 0; i < npages; i++)
    {
        /* Write one page */
        ret = w25n01gv_rand_program_data_load_quad(priv, address, buffer, pagesize);
        if (ret < 0)
        {
            ferr("ERROR: QSPI_MEMORY failed data load address=%06"PRIxOFF"\n",
                address);
            return ret;
        }

        ret = w25n01gv_program_execute(priv, address);
        if (ret < 0)
        {
            ferr("ERROR: QSPI_MEMORY failed program execute address=%06"PRIxOFF"\n",
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
// W25N01GV commands

/****************************************************************************
* Name: w25n01gv_reset
****************************************************************************/
static void w25n01gv_reset(FAR struct w25n01gv_dev_s *priv)
{
    /* Check the BUSY bit before issuing the reset command */
    DEBUGVERIFY(w25n01gv_wait_ready(priv, W25N01GV_DEFAULT_TIMEOUT_MS));

    /* Send the reset command */
	w25n01gv_command(priv->qspi, W25N01GV_DEVICE_RESET);

	/* Wait 500 us for the flash to complete the reset */
	nxsig_usleep(W25N01GV_TIMEOUT_RESET_MS);

	/* could check SR-2 and SR-3 bits here for successful reset */
}

/****************************************************************************
* Name: w25n01gv_readid
****************************************************************************/
static inline int w25n01gv_readid(struct w25n01gv_dev_s *priv)
{

	/* Lock the QuadSPI bus and configure the bus. */

	w25n01gv_lock(priv->qspi);

	/* Read the JEDEC ID */
	//   need 8 dummies clocks after 0x9F command
    memset(priv->cmdbuf, 0, 3); // clear buffer. Must <= 4 bytes as par
                                // QSPI_ALLOC in *w25n01gv_initialize()

    // DEBUGVERIFY(w25n01gv_wait_ready(priv, W25N01GV_DEFAULT_TIMEOUT_MS));

    w25n01gv_command_memory(priv->qspi, W25N01GV_JEDEC_ID,
                                       QSPIMEM_READ, 0, 8, priv->cmdbuf, 3);

	/* Unlock the bus */
	w25n01gv_unlock(priv->qspi);

	finfo("Manufacturer: %02x Device Type %02x, Capacity: %02x\n",
		priv->cmdbuf[0], priv->cmdbuf[1], priv->cmdbuf[2]);

	/* Check Manufacturer ID */
	if (priv->cmdbuf[0] != JEDEC_WINBOND_ID)
	{
		ferr("ERROR: Unrecognized manufacturer ID: 0x%02x\n",
			priv->cmdbuf[0]);
		return -ENODEV;
	}

	/* Check for a recognized memory device type */
	if (priv->cmdbuf[1] != W25N01GV_MEMORY_TYPE &&
		priv->cmdbuf[2] != W25N01GV_DEVICE_ID)
	{
		uint16_t devId = ((uint16_t)priv->cmdbuf[1] << 8) | priv->cmdbuf[2];
		ferr("ERROR: Unrecognized device type: 0x%04x\n", devId);
		return -ENODEV;
	}

    /* Report supported geometry */
    priv->geom.page_size_bytes  = W25N01GV_PAGE_SIZE + W25N01GV_OOB_SIZE;
    priv->geom.spare_size_bytes = W25N01GV_OOB_SIZE;
    priv->geom.block_size       = W25N01GV_PAGES_PER_BLK;
    priv->geom.block_size_bytes = W25N01GV_PAGE_SIZE * W25N01GV_PAGES_PER_BLK;
    priv->geom.total_blocks     = W25N01GV_BLOCKS;
    priv->protectmask          = STATUS_BP_4_MASK;
    priv->tbmask               = STATUS_TB_MASK;

    priv->geom.blockshift = 17; /* in NAND context, it really means erase block size.
                            log2(131072 bytes per sector) = 17 */
    priv->geom.pageshift   = 11; /* log2(2048 bytes per page) = 11 */
    // priv->nsectors    = 1024; /* 1024 erase blocks */
    // priv->addresslen  = 2;
	return OK;
}

/****************************************************************************
* Name: w25n01gv_read_status
****************************************************************************/
static uint8_t w25n01gv_read_status(FAR struct w25n01gv_dev_s *priv, uint32_t status_addr)
{
	DEBUGVERIFY(w25n01gv_command_address(priv->qspi, W25N01GV_READ_STATUS,
		status_addr, 1, (FAR void *)&priv->cmdbuf[0], 1));
	return priv->cmdbuf[0];
}

/****************************************************************************
* Name:  w25n01gv_write_status
****************************************************************************/
static void w25n01gv_write_status(FAR struct w25n01gv_dev_s *priv, uint32_t status_addr, uint8_t reg)
{
    DEBUGVERIFY(w25n01gv_wait_ready(priv, W25N01GV_DEFAULT_TIMEOUT_MS));

	w25n01gv_write_enable(priv);

	/* Keep in Software Protection */
	priv->cmdbuf[0] = reg;
	priv->cmdbuf[0] &= ~STATUS_WPE_MASK;

	w25n01gv_command_address(priv->qspi, W25N01GV_WRITE_STATUS,
		status_addr,
		1,
		(FAR const void *)priv->cmdbuf, 1);
	w25n01gv_write_disable(priv);
}

// TODO: solve blocking when WEL not set
/****************************************************************************
* Name:  w25n01gv_write_enable
****************************************************************************/
static void w25n01gv_write_enable(FAR struct w25n01gv_dev_s *priv)
{
    uint8_t status;
    DEBUGVERIFY(w25n01gv_wait_ready(priv, W25N01GV_DEFAULT_TIMEOUT_MS));

    do
    {
        w25n01gv_command(priv->qspi, W25N01GV_WRITE_ENABLE);
        status = w25n01gv_read_status(priv, W25N01GV_STATUS_ADDR); // SR-3
    }
    while ( (status & STATUS3_WEL_MASK) != STATUS3_WEL_ENABLED );
}

/****************************************************************************
* Name:  w25n01gv_write_disable
****************************************************************************/
static void w25n01gv_write_disable(FAR struct w25n01gv_dev_s *priv)
{
	// w25n01gv_command(priv->qspi, W25N01GV_WRITE_DISABLE);
    uint8_t status;
    DEBUGVERIFY(w25n01gv_wait_ready(priv, W25N01GV_DEFAULT_TIMEOUT_MS));

    do
    {
        w25n01gv_command(priv->qspi, W25N01GV_WRITE_DISABLE);
        status = w25n01gv_read_status(priv, W25N01GV_STATUS_ADDR);
    }
    while ((status & STATUS3_WEL_MASK) != STATUS3_WEL_DISABLED);
}

/****************************************************************************
* Name:  w25n01gv_bad_block_management
* p. 32
* Swap Blocks: The logical block address is the address for the “bad” block
* that will be replaced by the “good” block indicated by the physical block address.
****************************************************************************/
static void w25n01gv_bad_block_management(FAR struct w25n01gv_dev_s *priv,
	uint16_t lba, uint16_t pba)
{
    // Check LUT-F bit6 in SR-3 before issuing the command
    priv->readbuf[0] = w25n01gv_read_status(priv, W25N01GV_STATUS_ADDR);
    if ((priv->readbuf[0] & STATUS3_LUTF_MASK) == STATUS3_LUTF_FULL)
    {
        ferr("ERROR: BBM LUT is full, cannot add new entry\n");
        return;
    }

	priv->cmdbuf[0] = (uint8_t)(lba >> 8);
	priv->cmdbuf[1] = (uint8_t)(lba & 0xff);
	priv->cmdbuf[2] = (uint8_t)(pba >> 8);
	priv->cmdbuf[3] = (uint8_t)(pba & 0xff);

	w25n01gv_write_enable(priv);

	w25n01gv_command_write(priv->qspi, W25N01GV_BB_MANAGEMENT,
		(FAR const void *)priv->cmdbuf, 4);

	w25n01gv_write_disable(priv);
}

//TODO  add to init : scan all blocks and keep a table of all manufacturer
//bad blocks prior to first erase/program operation.
/****************************************************************************
* Name:  w25n01gv_read_bbm_lut
* LUT: 20 Logical-Physical memory block links (from LBA0/PBA0 to LBA19/PBA19)
* used to check the existing address links stored inside the LUT. If link
* exists there is bad block and LBA[15:14] give additional information:
* LBA[15]	LBA[14]		Description
* (Enable)	(Invalid)
*   0	   	0		This link is available to use.
*   1	   	0		This link is enabled and it is a valid link.
*   1	   	1		This link was enabled but it is an invalid link.
*   0	   	1		Reserved for future use.
*
*  command uses 8 dummycycles,
****************************************************************************/
static int w25n01gv_read_bbm_lut(FAR struct w25n01gv_dev_s *priv)
{
	uint8_t raw[W25N01GV_BBM_MAX_ENTRIES * 4]; /* 4 bytes per entry */

	/* Clear current list */
	priv->bbm_count = 0;
	memset(priv->bbm, 0, sizeof(priv->bbm));

	/* Read BBM LUT), expect 80 bytes max */
    int ret = w25n01gv_command_memory(priv->qspi, W25N01GV_READ_BBM,
                                       QSPIMEM_READ, 0, 8, raw, sizeof(raw));

	if (ret < 0)
	{
		ferr("ERROR: Failed to read BBM LUT: %d\n", ret);
		return ret;
	}

	/* Parse each entry */
	for (int i = 0; i < W25N01GV_BBM_MAX_ENTRIES; i++) {
		uint16_t bad  = ((uint16_t)raw[i*4 + 0] << 8) | raw[i*4 + 1];
		uint16_t good = ((uint16_t)raw[i*4 + 2] << 8) | raw[i*4 + 3];

		if (bad == 0xFFFF && good == 0xFFFF) {
			/* End of table */
			break;
		}

		priv->bbm[priv->bbm_count].bad_block  = bad & W25N01GV_BLOCK_ADDR_MASK;
		priv->bbm[priv->bbm_count].good_block = good & W25N01GV_BLOCK_ADDR_MASK;
		priv->bbm_count++;

		finfo("BBM[%d]: bad=0x%04X -> good=0x%04X\n",
			priv->bbm_count, bad, good);

		if (priv->bbm_count >= W25N01GV_BBM_MAX_ENTRIES) {
			break;
		}
	}

	return OK;
}

/****************************************************************************
* Name:  w25n01gv_last_ecc_failure_page_addr
*
* Description:
* For the “Continuous Read Mode (BUF=0)” operation, multiple pages of main
* array data can be read out continuously by issuing a single read command.
* Upon finishing the read operation, the ECC status bits should be check to
* verify if there’s any ECC correction or un-correctable errors existed in the
* read out data. If ECC-1 & ECC-0 equal to (1, 0) or (1, 1), the previous read
* out data contain one or more pages that contain ECC un-correctable errors.
* The failure page address (or the last page address if it’s multiple pages)
* can be obtained by issuing the “Last ECC failure Page Address” command as
* illustrated in Figure 13 (p. 34). The 16-bit Page Address that contains
* un-correctable ECC errors will be presented on the DO pin following the
* instruction code “A9h” and 8-bit dummy clocks on the DI pin.
*
* returns: PA15-8 | PA7-0
* Page Address (PA) requires 16 bits. PA[15:6] is the address for 128KB blocks
* (total 1,024 blocks), PA[5:0] is the address for 2KB pages (total 64 pages
* for each block).
****************************************************************************/
static uint16_t w25n01gv_last_ecc_failure_page_addr(FAR struct w25n01gv_dev_s *priv)
{
	uint16_t page_addr;

    memset(priv->cmdbuf, 0, 2);
    int ret = w25n01gv_command_memory(priv->qspi,
                                           W25N01GV_LAST_EEC_FAIL_PAGE_ADDR,
                                           QSPIMEM_READ, 0, 8, (FAR void *)priv->cmdbuf, 2);

	if (ret < 0)
	{
		ferr("ERROR: Failed to read last ECC failure page address: %d\n", ret);
		return ret;
	}
	page_addr = priv->cmdbuf[0] << 8 | priv->cmdbuf[1];
	finfo("Last ECC failure page address: 0x%04X\n", page_addr);
	return page_addr;
}

/****************************************************************************
* Name:  w25n01gv_block_erase
*
* Description:
*   The 128KB Block Erase instruction sets all memory within a specified block
*   (64-Pages, 128K-Bytes) to the erased state of all 1s (FFh). A Write Enable
*   instruction must be executed before the device will accept the Block Erase
*   Instruction (Status Register bit WEL must equal 1).
*
*   While the Block Erase cycle is in progress, the Read Status Register
*   instruction may still be accessed for checking the status of the BUSYbit.
*   The BUSY bit is a 1 during the Block Erase cycle and becomes a 0 when the
*   cycle is finished and the device is ready to accept other instructions
*   again. After the Block Erase cycle has finished the Write Enable Latch
*   (WEL) bit in the Status Register is cleared to 0. The Block Erase
*   instruction will not be executed if the addressed block is protected by the
*   Block Protect (TB, BP2, BP1, and BP0) bits.
*
* Input Parameters:
*   priv - Device-specific state data
*   block - The block index to erase (0-1023 for W25N01GV)
*
* Returned Value: OK on success; a negated errno value on failure
*
****************************************************************************/
static int w25n01gv_block_erase(FAR struct w25n01gv_dev_s *priv,
    uint32_t block)
{
    /* Convert block → page address (use first page of the block) */
    uint32_t page_addr = block * W25N01GV_PAGES_PER_BLK;

    /* Enable erase */
    w25n01gv_write_enable(priv);

    int ret = w25n01gv_command_memory(priv->qspi, W25N01GV_BLOCK_ERASE,
                                       QSPIMEM_WRITE, page_addr, 8,
                                       NULL, 0);
    if (ret < 0) {
        ferr("ERROR: Failed to erase block %lu, err=%d\n",
            (unsigned long)block, ret);
        return ret;
    }

    /* Wait for erase completion */
    ret = w25n01gv_wait_ready(priv, W25N01GV_TIMEOUT_BLOCK_ERASE_MS);

    w25n01gv_write_disable(priv);

    return ret;
}

/****************************************************************************
* Name:  w25n01gv_program_data_load (Reset Buffer)
*   Description:
*   The Program operation allows from one byte to 2,112 bytes (a page) of data
*   to be programmed at previously erased (FFh) memory locations. A Program
*   operation involves two steps:
*       1. Load the program data into the Data Buffer.
*       2. Issue “Program Execute” command to transfer the data from Data Buffer
*           to the specified memory page.
*   A Write Enable instruction must be executed before the device will accept
*   the Load Program Data Instructions (Status Register bit WEL= 1).
*
*   Both “Load Program Data” and “Random Load Program Data” instructions share
*   the same command sequence. The difference is that “Load Program Data”
*   instruction will reset the unused the data bytes in the Data Buffer to FFh
*   value, while “Random Load Program Data” instruction will only update the
*   data bytes that are specified by the command input sequence, the rest of
*   the Data Buffer will remain unchanged.
*
*   If internal ECC algorithm is enabled, all 2,112 bytes of data will be
*   accepted, but the bytes designated for ECC parity bits in the extra 64 bytes
*   section will be overwritten by the ECC calculation. If the ECC-E bit is set
*   to a 0 to disable the internal ECC, the extra 64 bytes section can be used
*   for external ECC purpose or other usage.
*
* Input Parameters:
*   priv - Device-specific state data
*   Column addr [15:0] (only CA[11:0] is effective)
*   Data: at least one byte of data. Maximum 2,112 bytes (a page)
*   Data length.
*
* Returned Value: OK on success; a negated errno value on failure
*
****************************************************************************/
static int w25n01gv_program_data_load(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR const uint8_t *buffer,
    size_t buflen)
{
    return w25n01gv_command_data_load(priv,
                                    W25N01GV_PROGRAM_DATA_LOAD,
                                    0, addr, buffer, buflen);
}

static int w25n01gv_rand_program_data_load(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR const uint8_t *buffer,
    size_t buflen)
{
     return w25n01gv_command_data_load(priv,
                                       W25N01GV_RAND_PROGRAM_DATA_LOAD,
                                       0, addr, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_program_dat_load_quad (Reset Buffer)
*
* Description:
*   The “Quad Load Program Data” and “Quad Random Load Program Data”
*   instructions are identical to the “Load Program Data” and “Random Load
*   Program Data” in terms of operation sequence and functionality. The only
*   difference is that “Quad Load” instructions will input the data bytes from
*   all four IO pins instead of the single DI pin. This method will
*   significantly shorten the data input time when a large amount of data needs
*   to be loaded into the Data Buffer.
*
*   Both “Quad Load Program Data” and “Quad Random Load Program Data”
*   instructions share the same command sequence. The difference is that “Quad
*   Load Program Data” instruction will reset the unused the data bytes in the
*   Data Buffer to FFh value, while “Quad Random Load Program Data” instruction
*   will only update the data bytes that are specified by the command input
*   sequence, the rest of the Data Buffer will remain unchanged.
*
****************************************************************************/
static int w25n01gv_program_data_load_quad(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR const uint8_t *buffer,
    size_t buflen)
{
    return w25n01gv_command_data_load(priv,
                                       W25N01GV_RAND_PROGRAM_DATA_LOAD,
                                       QSPICMD_IQUAD, addr, buffer, buflen);
}

static int w25n01gv_rand_program_data_load_quad(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR const uint8_t *buffer,
    size_t buflen)
{
     return w25n01gv_command_data_load(priv,
                                       W25N01GV_RAND_PROGRAM_DATA_LOAD,
                                       QSPICMD_IQUAD, addr, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_program_execute
****************************************************************************/
static int w25n01gv_program_execute(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr)
{
    return w25n01gv_command_memory(priv->qspi, W25N01GV_PROGRAM_EXECUTE,
                                    QSPIMEM_WRITE, addr, 8, NULL, 0);

}

/****************************************************************************
* Name:  w25n01gv_page_data_read
* Description:
*   The Page Data Read instruction will transfer the data of the specified
*   memory page into the 2,112-Byte Data Buffer.
*
*   After the 2,112 bytes of page data are loaded into the Data Buffer, several
*   Read instructions can be issued to access the Data Buffer and read out the
*   data. Depending on the BUF bit setting in the Status Register, either
*   “Buffer Read Mode” or “Continuous Read Mode” may be used to accomplish the
*   read operations.
*
****************************************************************************/
static int w25n01gv_page_data_read(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr)
{
    return w25n01gv_command_memory(priv->qspi, W25N01GV_PAGE_DATA_READ,
                                    QSPIMEM_READ, addr, 1, NULL, 0);
}

/****************************************************************************
* Name:  w25n01gv_read_data
* Description:
*   The Read Data instruction allows one or more data bytes to be sequentially
*   read from the Data Buffer after executing the Read Page Data instruction.
*
*   After the instruction code "03h" is followed by the 16-bit Column Address
*   and 8-bit dummy clocks or a 24-bit dummy clocks into the DI pin.
*
*   When BUF=1, the device is in the Buffer Read Mode. The data output sequence
*   will start from the Data Buffer location specified by the 16-bit Column
*   Address and continue to the end of the Data Buffer. Once the last byte of
*   data is output, the output pin will become Hi-Z state.
*
*   When BUF=0, the device is in the Continuous Read Mode, the data output
*   sequence will start from the first byte of the Data Buffer and increment to
*   the next higher address. When the end of the Data Buffer is reached, the
*   data of the first byte of next memory page will be following and continues
*   through the entire memory array. This allows using a single Read instruction
*   to read out the entire memory array and is also compatible to Winbond’s
*   SpiFlash NOR flash memory command sequence.
*
****************************************************************************/
static int w25n01gv_read_data(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR uint8_t *buffer,
    size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummycycles = 8; // default 8 dummy clocks for Buffer Read Mode (BUF=1)

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummycycles = 24;
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_READ_DATA, QSPIMEM_READ,
                                    addr, dummycycles, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_fast_read
* Description:
*   The Fast Read instruction allows one or more data bytes to be sequentially
*   read from the Data Buffer after executing the Read Page Data instruction.
*
*   After the instruction code "0Bh" is followed by the 16-bit Column Address
*   and 8-bit dummy clocks or a 32-bit dummy clocks (BUF=0) into the DI pin
*
*   When BUF=1, the device is in the Buffer Read Mode. The data output sequence
*   will start from the Data Buffer location specified by the 16-bit Column
*   Address and continue to the end of the Data Buffer.
*
*   When BUF=0, the device is in the Continuous Read Mode, the data output
*   sequence will start from the first byte of the Data Buffer and increment to
*   the next higher address. When the end of the Data Buffer is reached, the
*   data of the first byte of next memory page will be following and continues
*   through the entire memory array. This allows using a single Read instruction
*   to read out the entire memory array and is also compatible to Winbond’s
*   SpiFlash NOR flash memory command sequence.
*
****************************************************************************/
static int w25n01gv_fast_read(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR uint8_t *buffer,
    size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummy_cycles = 8; // default 8 dummy clocks for Buffer Read Mode (BUF=1)

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummy_cycles = 32;
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_FAST_READ, QSPIMEM_READ,
                                    addr, dummy_cycles, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_fast_read_4b
*
* Description:
*   The Fast Read 4-byte Address instruction allows one or more data bytes to
*   be sequentially read from the Data Buffer after executing the Read Page
*   Data instruction.
*
*   After the instruction code "0Ch" is followed by the 16-bit Column Address
*   and 24-bit dummy clocks or a 40-bit dummy clocks (BUF=0) into the DI pin.
*
****************************************************************************/
static int w25n01gv_fast_read_4b(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR uint8_t *buffer,
    size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummy_cycles = 24; // default 24 dummy clocks for Buffer Read Mode (BUF=1)

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummy_cycles = 40;
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_FAST_READ_4B, QSPIMEM_READ,
                                    addr, dummy_cycles, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_fast_read_dual
*
* Description:
*   The Fast Read Dual Output (3Bh) instruction is similar to the standard Fast
*   Read (0Bh) instruction except that data is output on two pins; IO0 and IO1.
*   This allows data to be transferred at twice the rate of standard SPI devices.
*
*   After the instruction code "3Bh" is followed by the 16-bit Column Address
*   and 8-bit dummy clocks or a 32-bit dummy clocks (BUF=0) into the DI pin.
*
*   When BUF=1, the device is in the Buffer Read Mode. The data output sequence
*   will start from the Data Buffer location specified by the 16-bit Column
*   Address and continue to the end of the Data Buffer.
*
*   When BUF=0, the device is in the Continuous Read Mode, the data output
*   sequence will start from the first byte of the Data Buffer and increment
*   to the next higher address. When the end of the Data Buffer is reached,
*   the data of the first byte of next memory page will be following and
*   continues through the entire memory array. This allows using a single Read
*   instruction to read out the entire memory array and is also compatible to
*   Winbond’s SpiFlash NOR flash memory command sequence.
*
****************************************************************************/
static int w25n01gv_fast_read_dual(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR uint8_t *buffer,
    size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummy_cycles = 8; // default 8 dummy clocks for Buffer Read Mode (BUF=1)

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummy_cycles = 32;
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_FAST_READ_DUAL,
                                    QSPIMEM_READ | QSPIMEM_IDUAL,
                                    addr, dummy_cycles, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_fast_read_dual_4b
****************************************************************************/
static int w25n01gv_fast_read_dual_4b(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR uint8_t *buffer,
    size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummy_cycles = 24; // default 24 dummy clocks for Buffer Read Mode (BUF=1)

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummy_cycles = 40;
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_FAST_READ_DUAL_4B,
                                    QSPIMEM_READ | QSPIMEM_IDUAL,
                                    addr, dummy_cycles, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_fast_read_quad
****************************************************************************/
static int w25n01gv_fast_read_quad(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR uint8_t *buffer,
    size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummy_cycles = 8; // default 8 dummy clocks for Buffer Read Mode (BUF=1)

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummy_cycles = 32;
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_FAST_READ_QUAD,
                                    QSPIMEM_READ | QSPIMEM_IQUAD,
                                    addr, dummy_cycles, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_fast_read_quad_4b
****************************************************************************/
static int w25n01gv_fast_read_quad_4b(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR uint8_t *buffer,
    size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummy_cycles = 24; // default 24 dummy clocks for Buffer Read Mode (BUF=1)

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummy_cycles = 40;
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_FAST_READ_QUAD_4B,
                                    QSPIMEM_READ | QSPIMEM_IQUAD,
                                    addr, dummy_cycles, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_fast_read_dualio
****************************************************************************/
static int w25n01gv_fast_read_dualio(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR uint8_t *buffer,
    size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummycycles = 4; // 4 dummy clocks for Buffer Read Mode (BUF=1)

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummycycles = 8;
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_FAST_READ_DUALIO,
                                    QSPIMEM_READ | QSPIMEM_DUALIO,
                                    addr, dummycycles, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_fast_read_dualio_4b
****************************************************************************/
static int w25n01gv_fast_read_dualio_4b(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR uint8_t *buffer,
    size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummycycles = 12; // 12 dummy clocks for Buffer Read Mode (BUF=1)

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummycycles = 20;
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_FAST_READ_DUALIO_4B,
                                    QSPIMEM_READ | QSPIMEM_DUALIO,
                                    addr, dummycycles, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_fast_read_quadio
****************************************************************************/
static int w25n01gv_fast_read_quadio(FAR struct w25n01gv_dev_s *priv,
                                      uint32_t addr, FAR uint8_t *buffer,
                                      size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummycycles = 4; // 4 dummy clocks for Buffer Read Mode (BUF=1)

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummycycles = 12;
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_FAST_READ_QUADIO,
                                    QSPIMEM_READ | QSPIMEM_QUADIO,
                                    addr, dummycycles, buffer, buflen);
}

/****************************************************************************
* Name:  w25n01gv_fast_read_quadio_4b
****************************************************************************/
static int w25n01gv_fast_read_quadio_4b(FAR struct w25n01gv_dev_s *priv,
    uint32_t addr,
    FAR uint8_t *buffer,
    size_t buflen)
{
    w25n01gv_page_data_read(priv, addr); // store start page in data buffer

    // check read mode
    uint8_t mode = w25n01gv_get_read_mode(priv);
    uint8_t dummycycles = 10;

    switch (mode) {
    case STATUS2_CONTINUOUS_READ:
    case STATUS2_CONTINUOUS_READ_OP:
        finfo("Read Mode: Continuous read (2048)\n");
        dummycycles = 14;  // 10 dummy clocks for Buffer Read Mode (BUF=1)
        break;
    case STATUS2_BUFFER_READ:
    case STATUS2_PAGE_READ:
        finfo("Read Mode: Buffer read (2048 + 64)\n");
        break;
    default:
        ferr("ERROR: Unrecognized read mode: %02x\n", mode);
        return -EINVAL;
    }
    // if Continuous Read Mode (BUF=0), read until end of entire memory array.
    return w25n01gv_command_memory(priv->qspi,
                                    W25N01GV_FAST_READ_QUADIO_4B,
                                    QSPIMEM_READ | QSPIMEM_QUADIO,
                                    addr, dummycycles, buffer, buflen);
}

// **************************************************************************
/****************************************************************************
* MTD private methods below
****************************************************************************/

/****************************************************************************
* Name: w25n01gv_erase
****************************************************************************/
static int w25n01gv_erase(FAR struct mtd_dev_s *dev, off_t startblock,
	size_t nblocks)
{
    FAR struct w25n01gv_dev_s *priv = (FAR struct w25n01gv_dev_s *)dev;
    size_t blocksleft = nblocks;

    finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

    /* Lock access to the SPI bus until we complete the erase */
    w25n01gv_lock(priv->qspi);

    while (blocksleft-- > 0)
    {
        /* Erase each block */
        w25n01gv_erase_block(priv, startblock);
        startblock++;
    }

    w25n01gv_unlock(priv->qspi);

    return (int)nblocks;
}

/****************************************************************************
* Name: w25n01gv_bread
****************************************************************************/
static ssize_t w25n01gv_bread(FAR struct mtd_dev_s *dev, off_t startblock,
	size_t nblocks, FAR uint8_t *buffer)
{
    FAR struct w25n01gv_dev_s *priv = (FAR struct w25n01gv_dev_s *)dev;
    ssize_t nbytes;

    finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

    /* On this device, we can handle the block read just like the byte-oriented
    * read
    */

    nbytes = w25n01gv_read(dev, startblock << priv->geom.pageshift,
                            nblocks << priv->geom.pageshift, buffer);
    if (nbytes > 0)
    {
        nbytes >>= priv->geom.pageshift;
    }

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

    ret = w25n01gv_write_page(priv, buffer, startblock << priv->geom.pageshift,
        nblocks << priv->geom.pageshift);
    if (ret < 0)
    {
        ferr("ERROR: w25n01gv_write_page failed: %d\n", ret);
    }

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

                geo->blocksize    = (1 << priv->geom.pageshift); // yes, set to page size, the smallest writable unit
                geo->erasesize    = (1 << priv->geom.blockshift);
                geo->neraseblocks = priv->geom.total_blocks;
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
                info->numsectors  = priv->geom.total_blocks <<
                (priv->geom.blockshift - priv->geom.pageshift);
                info->sectorsize  = 1 << priv->geom.pageshift;
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
            ret = w25n01gv_unprotect(priv);
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

        // Reset the device to make sure we are in a known state
        w25n01gv_reset(priv);
        if (ret != OK)
        {
            ferr("ERROR: w25n01gv_reset failed: %d\n", ret);
            goto errout_with_readbuf;
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

        // TODO enable ECC and buffer mode

        // read BBM LUT
        ret = w25n01gv_read_bbm_lut(priv);
        if (ret != OK)
        {
            ferr("ERROR: qspi_bbm_read_lut failed: %d\n", ret);
            return NULL;
        }
        // TODO get the physical block for any logical block


        //
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

