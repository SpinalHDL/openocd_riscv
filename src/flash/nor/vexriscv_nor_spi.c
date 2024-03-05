/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   LPC1700 support Copyright (C) 2009 by Audrius Urmanavicius            *
 *   didele.deze@gmail.com                                                 *
 *                                                                         *
 *   LPC1100 variant and auto-probing support Copyright (C) 2014           *
 *   by Cosmin Gorgovan cosmin [at] linux-geek [dot] org                   *
 *                                                                         *
 *   LPC800/LPC1500/LPC54100 support Copyright (C) 2013/2014               *
 *   by Nemui Trinomius                                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/target.h>
#include <target/target_type.h>
#include <helper/command.h>
#include <math.h>
#include <string.h>

typedef struct  {
	uint32_t ctrlAddress;
	bool probed;
} vexriscv_nor_spi_priv;


static int get_vexriscv_nor_spi_info(struct flash_bank *bank, struct command_invocation *cmd)
{

	return ERROR_OK;
}

#define priv ((vexriscv_nor_spi_priv*)bank->driver_priv)

#define CTRL_DATA 0x00
#define CTRL_STATUS 0x04
#define CTRL_MODE 0x08
#define CTRL_RATE 0x20
#define CTRL_SS_SETUP 0x24
#define CTRL_SS_HOLD 0x28
#define CTRL_SS_DISABLE 0x2C
#define CTRL_XIP_CONFIG 0x40
#define CTRL_XIP_MODE 0x44


static uint32_t vexriscv_nor_spi_readCtrlU32(struct flash_bank *bank, uint32_t addr)
{
	uint32_t ret;
	target_read_memory(bank->target, priv->ctrlAddress + addr, 4, 1, (uint8_t *)&ret);
	return ret;
}

static void vexriscv_nor_spi_writeCtrlU32(struct flash_bank *bank, uint32_t addr, uint32_t data)
{
	target_write_memory(bank->target, priv->ctrlAddress + addr, 4, 1, (uint8_t *)&data);
}

static void vexriscv_nor_spi_spiNotFull(struct flash_bank *bank)
{
	while((vexriscv_nor_spi_readCtrlU32(bank, CTRL_STATUS) & 0xFFFF) == 0);
}


static void vexriscv_nor_spi_spiStart(struct flash_bank *bank)
{
	vexriscv_nor_spi_spiNotFull(bank);
	vexriscv_nor_spi_writeCtrlU32(bank, CTRL_DATA, 0x880);
}


static void vexriscv_nor_spi_spiStop(struct flash_bank *bank)
{
	vexriscv_nor_spi_spiNotFull(bank);
	vexriscv_nor_spi_writeCtrlU32(bank, CTRL_DATA, 0x800);
}

static void vexriscv_nor_spi_spiWrite(struct flash_bank *bank, uint8_t data)
{
	vexriscv_nor_spi_spiNotFull(bank);
	vexriscv_nor_spi_writeCtrlU32(bank, CTRL_DATA, 0x100 | data);
}

static uint8_t vexriscv_nor_spi_spiRead(struct flash_bank *bank)
{
	vexriscv_nor_spi_spiNotFull(bank);
	vexriscv_nor_spi_writeCtrlU32(bank, CTRL_DATA, 0x200);
	while(1){
		uint32_t ret = vexriscv_nor_spi_readCtrlU32(bank, CTRL_DATA);
		if((ret & 0x80000000) == 0) return ret;
	}
}

static uint8_t vexriscv_nor_spi_spiReadRegister(struct flash_bank *bank, uint8_t address)
{
	vexriscv_nor_spi_spiStart(bank);
	vexriscv_nor_spi_spiWrite(bank, address);
	uint8_t ret = vexriscv_nor_spi_spiRead(bank);
	vexriscv_nor_spi_spiStop(bank);
	return ret;
}

static uint8_t vexriscv_nor_spi_spiReadStatus(struct flash_bank *bank)
{
	return vexriscv_nor_spi_spiReadRegister(bank, 0x05);
}



static void vexriscv_nor_spi_spiWriteLock(struct flash_bank *bank, uint32_t address, uint8_t value)
{
	vexriscv_nor_spi_spiStart(bank);
	vexriscv_nor_spi_spiWrite(bank, 0xE5);
	vexriscv_nor_spi_spiWrite(bank, (address >> 16) & 0xFF);
	vexriscv_nor_spi_spiWrite(bank, (address >>  8) & 0xFF);
	vexriscv_nor_spi_spiWrite(bank, (address >>  0) & 0xFF);
	vexriscv_nor_spi_spiWrite(bank, value);
	vexriscv_nor_spi_spiStop(bank);
}
/*
static uint8_t vexriscv_nor_spi_spiReadLock(struct flash_bank *bank, uint32_t address)
{
	vexriscv_nor_spi_spiStart(bank);
	vexriscv_nor_spi_spiWrite(bank, 0xE8);
	vexriscv_nor_spi_spiWrite(bank, (address >> 16) & 0xFF);
	vexriscv_nor_spi_spiWrite(bank, (address >>  8) & 0xFF);
	vexriscv_nor_spi_spiWrite(bank, (address >>  0) & 0xFF);
	uint8_t ret = vexriscv_nor_spi_spiRead(bank);
	vexriscv_nor_spi_spiStop(bank);
	return ret;
}
static uint8_t vexriscv_nor_spi_spiReadFlag(struct flash_bank *bank)
{
	return vexriscv_nor_spi_spiReadRegister(bank, 0x70);
}

static uint16_t vexriscv_nor_spi_spiReadNvConfig(struct flash_bank *bank)
{
	vexriscv_nor_spi_spiStart(bank);
	vexriscv_nor_spi_spiWrite(bank, 0xB5);
	uint16_t ret = vexriscv_nor_spi_spiRead(bank) + (vexriscv_nor_spi_spiRead(bank) << 8);
	vexriscv_nor_spi_spiStop(bank);
	return ret;
}

static uint8_t vexriscv_nor_spi_spiReadVConfig(struct flash_bank *bank)
{
	return vexriscv_nor_spi_spiReadRegister(bank, 0x85);
}*/

static void vexriscv_nor_spi_spiWaitNotBusy(struct flash_bank *bank){
	while(vexriscv_nor_spi_spiReadStatus(bank) & 1);
}



/*
static uint16_t vexriscv_nor_spi_spiReadNvConfig(struct flash_bank *bank)
{
	vexriscv_nor_spi_spiStart(bank);
	vexriscv_nor_spi_spiWrite(bank, 0xB1);
	uint16_t ret = vexriscv_nor_spi_spiRead(bank) + (vexriscv_nor_spi_spiRead(bank) << 8);
	vexriscv_nor_spi_spiStop(bank);
	return ret;
}

static uint16_t vexriscv_nor_spi_spiReadVConfig(struct flash_bank *bank)
{
	vexriscv_nor_spi_spiStart(bank);
	vexriscv_nor_spi_spiWrite(bank, 0x81);
	uint16_t ret = vexriscv_nor_spi_spiRead(bank) + (vexriscv_nor_spi_spiRead(bank) << 8);
	vexriscv_nor_spi_spiStop(bank);
	return ret;
}
*/


static void vexriscv_nor_spi_spiWriteEnable(struct flash_bank *bank){
	vexriscv_nor_spi_spiStart(bank);
	vexriscv_nor_spi_spiWrite(bank, 0x06);
	vexriscv_nor_spi_spiStop(bank);
}

static void vexriscv_nor_spi_spiWriteVolatileConfig(struct flash_bank *bank, uint8_t config){
	vexriscv_nor_spi_spiStart(bank);
	vexriscv_nor_spi_spiWrite(bank, 0x81);
	vexriscv_nor_spi_spiWrite(bank, config);
	vexriscv_nor_spi_spiStop(bank);
}


static void vexriscv_nor_spi_init(struct flash_bank *bank){
	vexriscv_nor_spi_writeCtrlU32(bank, CTRL_XIP_CONFIG, 0x0);
	vexriscv_nor_spi_writeCtrlU32(bank, CTRL_MODE, 0x0);
	vexriscv_nor_spi_writeCtrlU32(bank, CTRL_RATE, 0x2);
	vexriscv_nor_spi_writeCtrlU32(bank, CTRL_SS_SETUP, 0x4);
	vexriscv_nor_spi_writeCtrlU32(bank, CTRL_SS_HOLD, 0x4);
	vexriscv_nor_spi_writeCtrlU32(bank, CTRL_SS_DISABLE, 0x4);
}

static int vexriscv_nor_spi_probe(struct flash_bank *bank)
{
	if(!priv->probed){
		vexriscv_nor_spi_init(bank);

        vexriscv_nor_spi_spiStart(bank);
        vexriscv_nor_spi_spiWrite(bank, 0xAB);
        vexriscv_nor_spi_spiStop(bank);

        sleep(1);

		vexriscv_nor_spi_spiStart(bank);
		vexriscv_nor_spi_spiWrite(bank, 0x9F);
		uint8_t a = vexriscv_nor_spi_spiRead(bank);
		uint8_t b = vexriscv_nor_spi_spiRead(bank);
		uint8_t c = vexriscv_nor_spi_spiRead(bank);
		vexriscv_nor_spi_spiStop(bank);
		printf("%d %d %d\n", a, b, c);


		vexriscv_nor_spi_spiWriteVolatileConfig(bank,0x83);


		bank->num_sectors = bank->size/(64*1024);
		uint32_t offset = 0;
		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = offset;
			bank->sectors[i].size = 64 * 1024;
			offset += bank->sectors[i].size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 1;//vexriscv_nor_spi_spiReadLock(bank, offset);
			bank->size += bank->sectors[i].size;
		}
		priv->probed = 1;
	}

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(vexriscv_nor_spi_flash_bank_command)
{
	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	bank->driver_priv = malloc(sizeof(vexriscv_nor_spi_priv));
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], priv->ctrlAddress);
	priv->probed = 0;

	return ERROR_OK;
}


int  vexriscv_nor_spi_erase(struct flash_bank *bank, unsigned int first, unsigned int last){
	LOG_DEBUG("vexriscv_nor_spi_erase %d %d", first, last);
	vexriscv_nor_spi_init(bank);
	for(unsigned int sector = first;sector <= last;sector++){
		uint32_t addr = sector << 16;
		vexriscv_nor_spi_spiWriteEnable(bank);
		vexriscv_nor_spi_spiStart(bank);
		vexriscv_nor_spi_spiWrite(bank, 0xD8);
		vexriscv_nor_spi_spiWrite(bank, (addr >> 16) & 0xFF);
		vexriscv_nor_spi_spiWrite(bank, (addr >>  8) & 0xFF);
		vexriscv_nor_spi_spiWrite(bank, (addr >>  0) & 0xFF);
		vexriscv_nor_spi_spiStop(bank);
		vexriscv_nor_spi_spiWaitNotBusy(bank);

	}

	return ERROR_OK;
}


int vexriscv_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last){
	vexriscv_nor_spi_init(bank);
	for(unsigned int sector = first;sector <= last;sector++){
		vexriscv_nor_spi_spiWriteLock(bank, bank->sectors[sector].offset, set ? 1 : 0);
	}
	return ERROR_OK;
}

static void vexriscv_nor_spi_spiClearStatus(struct flash_bank *bank){
	vexriscv_nor_spi_spiStart(bank);
	vexriscv_nor_spi_spiWrite(bank, 0x50);
	vexriscv_nor_spi_spiStop(bank);
}


static char* vexriscv_args[] = {"x10","x11","x12","x13"};
static char* riscv_args[]    = {"a0","a1","a2","a3"};

#include "vexriscv_algo/vexriscv_nor_spi_write.h"
int vexriscv_nor_spi_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count){
	uint32_t end = offset + count;
	uint32_t burstMax = 256;
	uint32_t burstMask = ~(burstMax-1);
	struct target *target = bank->target;
	struct working_area * workArea;
	if(target_alloc_working_area(target, 512 + burstMax, &workArea) != ERROR_OK){
		LOG_ERROR("vexriscv_nor_spi can't allocate a working area in that target");
		return ERROR_BUF_TOO_SMALL;
	}
	uint32_t ramAddress = workArea->address;
	uint32_t codeAddress = ramAddress + burstMax;
	char** rpNames = strcmp(bank->target->type->name, "riscv") == 0 ? riscv_args : vexriscv_args;

	target_write_buffer(bank->target, codeAddress, vexriscv_nor_spi_write_bin_len, vexriscv_nor_spi_write_bin);

	while(offset < end){
		uint32_t offsetNext = MIN((offset & burstMask) + burstMax, end);
		uint32_t burstSize = offsetNext - offset;
		struct mem_param mp[1];
		mp[0].address = ramAddress;
		mp[0].size = burstSize;
		mp[0].value = (uint8_t*)buffer;
		mp[0].direction = PARAM_OUT;
		struct reg_param rp[4];
		uint32_t rpValues[] = {priv->ctrlAddress, offset, burstSize, ramAddress};
		for(int i = 0;i < 4;i ++){
			rp[i].reg_name = rpNames[i];
			rp[i].value = (uint8_t*)&rpValues[i];
			rp[i].size = 32;
			rp[i].direction = PARAM_OUT;
		}
		target_run_algorithm(bank->target, 1,mp,4,rp, codeAddress, 0, 2000, NULL);
		buffer += burstSize;
		offset = offsetNext;
	}

	vexriscv_nor_spi_spiWaitNotBusy(bank);
	vexriscv_nor_spi_spiClearStatus(bank);


	target_free_working_area(target, workArea);

	return ERROR_OK;
}

#include "vexriscv_algo/vexriscv_nor_spi_read.h"
int vexriscv_nor_spi_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count){
	vexriscv_nor_spi_init(bank);
	uint32_t end = offset + count;
	uint32_t burstMax = 256;
	uint32_t burstMask = ~(burstMax-1);
	struct target *target = bank->target;
	struct working_area * workArea;
	if(target_alloc_working_area(target, 512 + burstMax, &workArea) != ERROR_OK){
		LOG_ERROR("vexriscv_nor_spi can't allocate a working area in that target");
		return ERROR_BUF_TOO_SMALL;
	}
	uint32_t ramAddress = workArea->address;
	uint32_t codeAddress = ramAddress + burstMax;
	char** rpNames = strcmp(bank->target->type->name, "riscv") == 0 ? riscv_args : vexriscv_args;

	target_write_buffer(bank->target, codeAddress, vexriscv_nor_spi_read_bin_len, vexriscv_nor_spi_read_bin);

	while(offset < end){
		uint32_t offsetNext = MIN((offset & burstMask) + burstMax, end);
		uint32_t burstSize = offsetNext - offset;
		struct mem_param mp[1];
		mp[0].address = ramAddress;
		mp[0].size = burstSize;
		mp[0].value = (uint8_t*)buffer;
		mp[0].direction = PARAM_IN;
		struct reg_param rp[4];
		uint32_t rpValues[] = {priv->ctrlAddress, offset, burstSize,  ramAddress};
		for(int i = 0;i < 4;i ++){
			rp[i].reg_name = rpNames[i];
			rp[i].value = (uint8_t*)&rpValues[i];
			rp[i].size = 32;
			rp[i].direction = PARAM_OUT;
		}

		target_run_algorithm(bank->target, 1,mp,4,rp, codeAddress, 0, 2000, NULL);
		buffer += burstSize;
		offset = offsetNext;
	}

	target_free_working_area(target, workArea);
	return ERROR_OK;
}

COMMAND_HANDLER(vexriscv_nor_spi_reset_driver_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	struct flash_bank* bank = get_flash_bank_by_name_noprobe(cmd->argv[0]);
	if(bank == NULL) return ERROR_COMMAND_SYNTAX_ERROR;
	priv->probed = 0;
	return ERROR_OK;
}

static const struct command_registration vexriscv_nor_spi_exec_command_handlers[] = {
	{
		.name = "reset_driver",
		.handler = vexriscv_nor_spi_reset_driver_command,
		.mode = COMMAND_EXEC,
		.help = "reset driver <bank_name>",
		.usage = "<bank_name>",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration vexriscv_nor_spi_command_handlers[] = {
	{
		.name = "vexriscv_nor_spi",
		.mode = COMMAND_ANY,
		.help = "vexriscv nor spi command group",
		.usage = "",
		.chain = vexriscv_nor_spi_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


const struct flash_driver vexriscv_nor_spi = {
	.name = "vexriscv_nor_spi",
	.commands = vexriscv_nor_spi_command_handlers,
	.flash_bank_command = vexriscv_nor_spi_flash_bank_command,
	.erase = vexriscv_nor_spi_erase,
	.protect = vexriscv_protect,
	.write = vexriscv_nor_spi_write,
	.read = vexriscv_nor_spi_read,
	.probe = vexriscv_nor_spi_probe,
	.auto_probe = vexriscv_nor_spi_probe,
//	.erase_check = lpc2000_erase_check,
//	.protect_check = lpc2000_protect_check,
	.info = get_vexriscv_nor_spi_info,
};

