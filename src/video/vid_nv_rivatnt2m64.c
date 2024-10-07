/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          nVIDIA Riva TNT2 Model 64 emulation.
 *
 *
 *
 * Authors: aquaboxs
 *
 *
 *          Copyright 2024 aquaboxs.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <wchar.h>
#include <86box/86box.h>
#include "../cpu/cpu.h"
#include <86box/dma.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/pci.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/video.h>
#include <86box/i2c.h>
#include <86box/vid_ddc.h>
#include <86box/vid_svga.h>
#include <86box/vid_svga_render.h>

typedef struct rivatnt2m64_t
{
  mem_mapping_t   linear_mapping;
  mem_mapping_t     mmio_mapping;

  rom_t bios_rom;

  svga_t svga;

  uint32_t linear_base, linear_size;

  uint16_t rma_addr;

  uint8_t pci_regs[256];

  int memory_size;

  uint8_t ext_regs_locked;

  uint8_t read_bank;
  uint8_t write_bank;

  struct
  {
	uint32_t intr;
	uint32_t intr_en;
	uint32_t intr_line;
	uint32_t enable;
  } pmc;
  
  struct
  {
	uint32_t intr;
	uint32_t intr_en;
  } pbus;
  
  struct
  {
	uint32_t intr;
	uint32_t intr_en;
	
	uint32_t ramht;
	uint32_t ramht_addr;
	uint32_t ramht_size;
	
	uint32_t ramfc;
	uint32_t ramfc_addr;
	
	uint32_t ramro;
	uint32_t ramro_addr;
	uint32_t ramro_size;
	
	uint16_t chan_mode;
	uint16_t chan_dma;
	uint16_t chan_size; //0 = 1024, 1 = 512
	
	struct
	{
		int chanid;
	} caches[2];
	
	struct
	{
		int subchan;
		uint16_t method;
		uint32_t param;
	} cache0, cache1[64];
  } pfifo;
  
  struct
  {
    uint32_t addr;
    uint32_t data;
    uint8_t access_reg[4];
    uint8_t mode;
  } rma;
  
  struct
  {
	uint32_t time;
  } ptimer;
  
  struct
  {
    int width;
    int bpp;
    uint32_t config_0;
  } pfb;

  struct
  {
	uint32_t obj_handle[16][8];
	uint8_t obj_class[16][8];
	
	uint32_t intr;
  } pgraph;
  
  struct
  {
    uint32_t nvpll;
    uint32_t nv_m,nv_n,nv_p;
    
    uint32_t mpll;
    uint32_t m_m,m_n,m_p;
  
    uint32_t vpll;
    uint32_t v_m,v_n,v_p;
  
    uint32_t pll_ctrl;
  
    uint32_t gen_ctrl;
  } pramdac;
  
  uint32_t channels[16][8][0x2000];
  
  int coretime;

  void *i2c, *ddc;
} rivatnt2m64_t;

const char* rivatnt2m64_pmc_interrupts[32] =
{
	"","","","","PMEDIA","","","","PFIFO","","","","PGRAPH","","","","PRAMDAC.VIDEO","","","","PTIMER","","","","PCRTC","","","","PBUS","","",""
};

const char* rivatnt2m64_pbus_interrupts[32] =
{
	"BUS_ERROR","","","","","","","","","","","","","","","","","","","","","","","","","","","","","","",""	
};

const char* rivatnt2m64_pfifo_interrupts[32] =
{
	"CACHE_ERROR","","","","RUNOUT","","","","RUNOUT_OVERFLOW","","","","DMA_PUSHER","","","","DMA_PTE","","","","","","","","","","","","","","",""	
};

static video_timings_t timing_rivatnt2m64 = { .type = VIDEO_AGP, .write_b = 2, .write_w = 2, .write_l = 3, .read_b = 24, .read_w = 24, .read_l = 36 };

static uint8_t rivatnt2m64_pci_read(int func, int addr, void *p);
static void rivatnt2m64_pci_write(int func, int addr, uint8_t val, void *p);

static uint8_t rivatnt2m64_in(uint16_t addr, void *p);
static void rivatnt2m64_out(uint16_t addr, uint8_t val, void *p);

static void rivatnt2m64_mmio_write_l(uint32_t addr, uint32_t val, void *p);

static uint8_t rivatnt2m64_pmc_read(uint32_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  //pclog("RIVA TNT PMC read %08X %04X:%08X\n", addr, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x000000: ret = 0x00; break;
  case 0x000001: ret = 0x40; break;
  case 0x000002: ret = 0x15; break;
  case 0x000003: ret = 0x20; break;
  case 0x000100: ret = rivatnt2m64->pmc.intr & 0xff; break;
  case 0x000101: ret = (rivatnt2m64->pmc.intr >> 8) & 0xff; break;
  case 0x000102: ret = (rivatnt2m64->pmc.intr >> 16) & 0xff; break;
  case 0x000103: ret = (rivatnt2m64->pmc.intr >> 24) & 0xff; break;
  case 0x000140: ret = rivatnt2m64->pmc.intr & 0xff; break;
  case 0x000141: ret = (rivatnt2m64->pmc.intr_en  >> 8) & 0xff; break;
  case 0x000142: ret = (rivatnt2m64->pmc.intr_en >> 16) & 0xff; break;
  case 0x000143: ret = (rivatnt2m64->pmc.intr_en >> 24) & 0xff; break;
  case 0x000160: ret = rivatnt2m64->pmc.intr_line & 0xff; break;
  case 0x000161: ret = (rivatnt2m64->pmc.intr_line >> 8) & 0xff; break;
  case 0x000162: ret = (rivatnt2m64->pmc.intr_line >> 16) & 0xff; break;
  case 0x000163: ret = (rivatnt2m64->pmc.intr_line >> 24) & 0xff; break;
  case 0x000200: ret = rivatnt2m64->pmc.enable & 0xff; break;
  case 0x000201: ret = (rivatnt2m64->pmc.enable >> 8) & 0xff; break;
  case 0x000202: ret = (rivatnt2m64->pmc.enable >> 16) & 0xff; break;
  case 0x000203: ret = (rivatnt2m64->pmc.enable >> 24) & 0xff; break;
  }

  return ret;
}

static void rivatnt2m64_pmc_write(uint32_t addr, uint32_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  //pclog("RIVA TNT PMC write %08X %08X %04X:%08X\n", addr, val, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x000100:
  rivatnt2m64->pmc.intr &= ~val;
  //if((val & 0x80000000) && (rivatnt2m64->pmc.intr_en & 2)) pci_interrupt(0);
  break;
  case 0x000140:
  rivatnt2m64->pmc.intr_en = val & 3;
  break;
  case 0x000200:
  rivatnt2m64->pmc.enable = val;
  //if(val & 0x80000000) pci_interrupt(0);
  break;
  }
}

static uint8_t rivatnt2m64_pbus_read(uint32_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  //pclog("RIVA TNT PBUS read %08X %04X:%08X\n", addr, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x001100: ret = rivatnt2m64->pbus.intr & 0xff; break;
  case 0x001101: ret = (rivatnt2m64->pbus.intr >> 8) & 0xff; break;
  case 0x001102: ret = (rivatnt2m64->pbus.intr >> 16) & 0xff; break;
  case 0x001103: ret = (rivatnt2m64->pbus.intr >> 24) & 0xff; break;
  case 0x001140: ret = rivatnt2m64->pbus.intr & 0xff; break;
  case 0x001141: ret = (rivatnt2m64->pbus.intr_en  >> 8) & 0xff; break;
  case 0x001142: ret = (rivatnt2m64->pbus.intr_en >> 16) & 0xff; break;
  case 0x001143: ret = (rivatnt2m64->pbus.intr_en >> 24) & 0xff; break;
  case 0x001800 ... 0x0018ff: ret = rivatnt2m64_pci_read(0, addr - 0x1800, rivatnt2m64); break;
  }

  return ret;
}

static void rivatnt2m64_pbus_write(uint32_t addr, uint32_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  //pclog("RIVA TNT PBUS write %08X %08X %04X:%08X\n", addr, val, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x001100:
  rivatnt2m64->pbus.intr &= ~val;
  break;
  case 0x001140:
  rivatnt2m64->pbus.intr_en = val;
  break;
  case 0x001800 ... 0x0018ff:
  rivatnt2m64_pci_write(0, (addr & 0xfc) + 0, (val >> 0) & 0xff, rivatnt2m64);
  rivatnt2m64_pci_write(0, (addr & 0xfc) + 1, (val >> 8) & 0xff, rivatnt2m64);
  rivatnt2m64_pci_write(0, (addr & 0xfc) + 2, (val >> 16) & 0xff, rivatnt2m64);
  rivatnt2m64_pci_write(0, (addr & 0xfc) + 3, (val >> 24) & 0xff, rivatnt2m64);
  break;
  }
}

static uint8_t rivatnt2m64_pfifo_read(uint32_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  pclog("RIVA TNT PFIFO read %08X %04X:%08X\n", addr, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x002100: ret = rivatnt2m64->pfifo.intr & 0xff; break;
  case 0x002101: ret = (rivatnt2m64->pfifo.intr >> 8) & 0xff; break;
  case 0x002102: ret = (rivatnt2m64->pfifo.intr >> 16) & 0xff; break;
  case 0x002103: ret = (rivatnt2m64->pfifo.intr >> 24) & 0xff; break;
  case 0x002140: ret = rivatnt2m64->pfifo.intr_en & 0xff; break;
  case 0x002141: ret = (rivatnt2m64->pfifo.intr_en >> 8) & 0xff; break;
  case 0x002142: ret = (rivatnt2m64->pfifo.intr_en >> 16) & 0xff; break;
  case 0x002143: ret = (rivatnt2m64->pfifo.intr_en >> 24) & 0xff; break;
  case 0x002210: ret = rivatnt2m64->pfifo.ramht & 0xff; break;
  case 0x002211: ret = (rivatnt2m64->pfifo.ramht >> 8) & 0xff; break;
  case 0x002212: ret = (rivatnt2m64->pfifo.ramht >> 16) & 0xff; break;
  case 0x002213: ret = (rivatnt2m64->pfifo.ramht >> 24) & 0xff; break;
  case 0x002214: ret = rivatnt2m64->pfifo.ramfc & 0xff; break;
  case 0x002215: ret = (rivatnt2m64->pfifo.ramfc >> 8) & 0xff; break;
  case 0x002216: ret = (rivatnt2m64->pfifo.ramfc >> 16) & 0xff; break;
  case 0x002217: ret = (rivatnt2m64->pfifo.ramfc >> 24) & 0xff; break;
  case 0x002218: ret = rivatnt2m64->pfifo.ramro & 0xff; break;
  case 0x002219: ret = (rivatnt2m64->pfifo.ramro >> 8) & 0xff; break;
  case 0x00221a: ret = (rivatnt2m64->pfifo.ramro >> 16) & 0xff; break;
  case 0x00221b: ret = (rivatnt2m64->pfifo.ramro >> 24) & 0xff; break;
  case 0x002504: ret = rivatnt2m64->pfifo.chan_mode & 0xff; break;
  case 0x002505: ret = (rivatnt2m64->pfifo.chan_mode >> 8) & 0xff; break;
  case 0x002506: ret = (rivatnt2m64->pfifo.chan_mode >> 16) & 0xff; break;
  case 0x002507: ret = (rivatnt2m64->pfifo.chan_mode >> 24) & 0xff; break;
  case 0x002508: ret = rivatnt2m64->pfifo.chan_dma & 0xff; break;
  case 0x002509: ret = (rivatnt2m64->pfifo.chan_dma >> 8) & 0xff; break;
  case 0x00250a: ret = (rivatnt2m64->pfifo.chan_dma >> 16) & 0xff; break;
  case 0x00250b: ret = (rivatnt2m64->pfifo.chan_dma >> 24) & 0xff; break;
  case 0x00250c: ret = rivatnt2m64->pfifo.chan_size & 0xff; break;
  case 0x00250d: ret = (rivatnt2m64->pfifo.chan_size >> 8) & 0xff; break;
  case 0x00250e: ret = (rivatnt2m64->pfifo.chan_size >> 16) & 0xff; break;
  case 0x00250f: ret = (rivatnt2m64->pfifo.chan_size >> 24) & 0xff; break;
  //HACK
  case 0x002400: ret = 0x10; break;
  case 0x002401: ret = 0x00; break;
  case 0x003204: ret = rivatnt2m64->pfifo.caches[1].chanid; break;
  case 0x003214: ret = 0x10; break;
  case 0x003215: ret = 0x00; break;
  case 0x003220: ret = 0x01; break;
  }

  return ret;
}

static void rivatnt2m64_pfifo_write(uint32_t addr, uint32_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  pclog("RIVA TNT PFIFO write %08X %08X %04X:%08X\n", addr, val, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x002100:
  rivatnt2m64->pfifo.intr &= ~val;
  break;
  case 0x002140:
  rivatnt2m64->pfifo.intr_en = val;
  break;
  case 0x002210:
  rivatnt2m64->pfifo.ramht = val;
  rivatnt2m64->pfifo.ramht_addr = (val & 0x1f0) << 8;
  switch(val & 0x30000)
  {
	case 0x00000:
	rivatnt2m64->pfifo.ramht_size = 4 * 1024;
	break;
	case 0x10000:
	rivatnt2m64->pfifo.ramht_size = 8 * 1024;
	break;
	case 0x20000:
	rivatnt2m64->pfifo.ramht_size = 16 * 1024;
	break;
	case 0x30000:
	rivatnt2m64->pfifo.ramht_size = 32 * 1024;
	break;
  }
  break;
  case 0x002214:
  rivatnt2m64->pfifo.ramfc = val;
  rivatnt2m64->pfifo.ramfc_addr = (val & 0x1fe) << 4;
  break;
  case 0x002218:
  rivatnt2m64->pfifo.ramro = val;
  rivatnt2m64->pfifo.ramro_addr = (val & 0x1fe) << 4;
  if(val & 0x10000) rivatnt2m64->pfifo.ramro_size = 8192;
  else rivatnt2m64->pfifo.ramro_size = 512;
  break;
  case 0x002504:
  rivatnt2m64->pfifo.chan_mode = val;
  break;
  case 0x002508:
  rivatnt2m64->pfifo.chan_dma = val;
  break;
  case 0x00250c:
  rivatnt2m64->pfifo.chan_size = val;
  break;
  case 0x003204:
  rivatnt2m64->pfifo.caches[1].chanid = val;
  break;
  }
}

static uint8_t rivatnt2m64_ptimer_read(uint32_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  //pclog("RIVA TNT PTIMER read %08X %04X:%08X\n", addr, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x009400: ret = rivatnt2m64->ptimer.time & 0xff; break;
  case 0x009401: ret = (rivatnt2m64->ptimer.time >> 8) & 0xff; break;
  case 0x009402: ret = (rivatnt2m64->ptimer.time >> 16) & 0xff; break;
  case 0x009403: ret = (rivatnt2m64->ptimer.time >> 24) & 0xff; break;
  }

  //TODO: gross hack to make NT4 happy for the time being.
  rivatnt2m64->ptimer.time += 0x10000;
  
  return ret;
}

static uint8_t rivatnt2m64_pfb_read(uint32_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  //pclog("RIVA TNT PFB read %08X %04X:%08X\n", addr, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x100000:
  {
    switch(rivatnt2m64->memory_size)
    {
    case 4: ret = 1; break;
    case 8: ret = 2; break;
    case 16: ret = 3; break;
    case 32: ret = 0; break;
    }
    ret |= 0x14;
    break;
  }
  case 0x100200: ret = rivatnt2m64->pfb.config_0 & 0xff; break;
  case 0x100201: ret = (rivatnt2m64->pfb.config_0 >> 8) & 0xff; break;
  case 0x100202: ret = (rivatnt2m64->pfb.config_0 >> 16) & 0xff; break;
  case 0x100203: ret = (rivatnt2m64->pfb.config_0 >> 24) & 0xff; break;
  }

  return ret;
}

static void rivatnt2m64_pfb_write(uint32_t addr, uint32_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  //pclog("RIVA TNT PFB write %08X %08X %04X:%08X\n", addr, val, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x100200:
  rivatnt2m64->pfb.config_0 = val;
  rivatnt2m64->pfb.width = (val & 0x3f) << 5;
  switch((val >> 8) & 3)
  {
  case 1: rivatnt2m64->pfb.bpp = 8; break;
  case 2: rivatnt2m64->pfb.bpp = 16; break;
  case 3: rivatnt2m64->pfb.bpp = 32; break;
  }
  break;
  }
}

static uint8_t rivatnt2m64_pextdev_read(uint32_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  //pclog("RIVA TNT PEXTDEV read %08X %04X:%08X\n", addr, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x101000: ret = 0x9e; break;
  case 0x101001: ret = 0x01; break;
  }

  return ret;
}

static uint8_t rivatnt2m64_pgraph_read(uint32_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  pclog("RIVA TNT PGRAPH read %08X %04X:%08X\n", addr, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x400100: ret = rivatnt2m64->pgraph.intr & 0xff; break;
  case 0x400101: ret = (rivatnt2m64->pgraph.intr >> 8) & 0xff; break;
  case 0x400102: ret = (rivatnt2m64->pgraph.intr >> 16) & 0xff; break;
  case 0x400103: ret = (rivatnt2m64->pgraph.intr >> 24) & 0xff; break;
  }

  return ret;
}

static uint8_t rivatnt2m64_pramdac_read(uint32_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  //pclog("RIVA TNT PRAMDAC read %08X %04X:%08X\n", addr, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x680500: ret = rivatnt2m64->pramdac.nvpll & 0xff; break;
  case 0x680501: ret = (rivatnt2m64->pramdac.nvpll >> 8) & 0xff; break;
  case 0x680502: ret = (rivatnt2m64->pramdac.nvpll >> 16) & 0xff; break;
  case 0x680503: ret = (rivatnt2m64->pramdac.nvpll >> 24) & 0xff; break;
  case 0x680504: ret = rivatnt2m64->pramdac.mpll & 0xff; break;
  case 0x680505: ret = (rivatnt2m64->pramdac.mpll >> 8) & 0xff; break;
  case 0x680506: ret = (rivatnt2m64->pramdac.mpll >> 16) & 0xff; break;
  case 0x680507: ret = (rivatnt2m64->pramdac.mpll >> 24) & 0xff; break;
  case 0x680508: ret = rivatnt2m64->pramdac.vpll & 0xff; break;
  case 0x680509: ret = (rivatnt2m64->pramdac.vpll >> 8) & 0xff; break;
  case 0x68050a: ret = (rivatnt2m64->pramdac.vpll >> 16) & 0xff; break;
  case 0x68050b: ret = (rivatnt2m64->pramdac.vpll >> 24) & 0xff; break;
  case 0x68050c: ret = rivatnt2m64->pramdac.pll_ctrl & 0xff; break;
  case 0x68050d: ret = (rivatnt2m64->pramdac.pll_ctrl >> 8) & 0xff; break;
  case 0x68050e: ret = (rivatnt2m64->pramdac.pll_ctrl >> 16) & 0xff; break;
  case 0x68050f: ret = (rivatnt2m64->pramdac.pll_ctrl >> 24) & 0xff; break;
  case 0x680600: ret = rivatnt2m64->pramdac.gen_ctrl & 0xff; break;
  case 0x680601: ret = (rivatnt2m64->pramdac.gen_ctrl >> 8) & 0xff; break;
  case 0x680602: ret = (rivatnt2m64->pramdac.gen_ctrl >> 16) & 0xff; break;
  case 0x680603: ret = (rivatnt2m64->pramdac.gen_ctrl >> 24) & 0xff; break;
  }

  return ret;
}

static void rivatnt2m64_pramdac_write(uint32_t addr, uint32_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  //pclog("RIVA TNT PRAMDAC write %08X %08X %04X:%08X\n", addr, val, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x680500:
  rivatnt2m64->pramdac.nvpll = val;
  rivatnt2m64->pramdac.nv_m = val & 0xff;
  rivatnt2m64->pramdac.nv_n = (val >> 8) & 0xff;
  rivatnt2m64->pramdac.nv_p = (val >> 16) & 3;
  break;
  case 0x680504:
  rivatnt2m64->pramdac.mpll = val;
  rivatnt2m64->pramdac.m_m = val & 0xff;
  rivatnt2m64->pramdac.m_n = (val >> 8) & 0xff;
  rivatnt2m64->pramdac.m_p = (val >> 16) & 3;
  break;
  case 0x680508:
  rivatnt2m64->pramdac.vpll = val;
  rivatnt2m64->pramdac.v_m = val & 0xff;
  rivatnt2m64->pramdac.v_n = (val >> 8) & 0xff;
  rivatnt2m64->pramdac.v_p = (val >> 16) & 3;
  svga_recalctimings(svga);
  break;
  case 0x68050c:
  rivatnt2m64->pramdac.pll_ctrl = val;
  break;
  case 0x680600:
  rivatnt2m64->pramdac.gen_ctrl = val;
  break;
  }
}

static uint8_t rivatnt2m64_ramht_lookup(uint32_t handle, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  pclog("RIVA TNT RAMHT lookup with handle %08X %04X:%08X\n", handle, CS, cpu_state.pc);
  
  uint8_t objclass;
  
  uint32_t ramht_base = rivatnt2m64->pfifo.ramht_addr;
  
  uint32_t tmp = handle;
  uint32_t hash = 0;
  
  int bits;
  
  switch(rivatnt2m64->pfifo.ramht_size)
  {
	case 4096: bits = 12;
	case 8192: bits = 13;
	case 16384: bits = 14;
	case 32768: bits = 15;
  }
  
  while(handle)
  {
	hash ^= (tmp & (rivatnt2m64->pfifo.ramht_size - 1));
	tmp = handle >> 1;
  }
  
  hash ^= rivatnt2m64->pfifo.caches[1].chanid << (bits - 4);
  
  objclass = svga_readl_linear((svga->vram_max - (1 * 1024 * 1024)) + ramht_base + (hash * 8), svga);
  objclass &= 0xff;
  return objclass;
}

static void rivatnt2m64_pgraph_exec_method(int chanid, int subchanid, int offset, uint32_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  pclog("RIVA TNT PGRAPH executing method %04X on channel %01X %04X:%08X\n", offset, chanid, val, CS, cpu_state.pc);
}

static void rivatnt2m64_puller_exec_method(int chanid, int subchanid, int offset, uint32_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  pclog("RIVA TNT Puller executing method %04X on channel %01X[%01X] %04X:%08X\n", offset, chanid, subchanid, val, CS, cpu_state.pc);
  
  if(offset < 0x100)
  {
	//These methods are executed by the puller itself.
	if(offset == 0)
	{
		rivatnt2m64->pgraph.obj_handle[chanid][subchanid] = val;
		rivatnt2m64->pgraph.obj_class[chanid][subchanid] = rivatnt2m64_ramht_lookup(val, rivatnt2m64);
	}
  }
  else
  {
	rivatnt2m64_pgraph_exec_method(chanid, subchanid, offset, val, rivatnt2m64);
  }
}

static void rivatnt2m64_user_write(uint32_t addr, uint32_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  pclog("RIVA TNT USER write %08X %08X %04X:%08X\n", addr, val, CS, cpu_state.pc);
  
  addr -= 0x800000;
  
  int chanid = (addr >> 16) & 0xf;
  int subchanid = (addr >> 13) & 0x7;
  int offset = addr & 0x1fff;
  
  rivatnt2m64->channels[chanid][subchanid][offset] = val;
  //TODO: make this async
  rivatnt2m64_puller_exec_method(chanid, subchanid, offset, val, rivatnt2m64);
}

static uint8_t rivatnt2m64_mmio_read(uint32_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  addr &= 0xffffff;

  pclog("RIVA TNT MMIO read %08X %04X:%08X\n", addr, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x000000 ... 0x000fff:
  ret = rivatnt2m64_pmc_read(addr, rivatnt2m64);
  break;
  case 0x001000 ... 0x001fff:
  ret = rivatnt2m64_pbus_read(addr, rivatnt2m64);
  break;
  case 0x002000 ... 0x002fff:
  ret = rivatnt2m64_pfifo_read(addr, rivatnt2m64);
  break;
  case 0x009000 ... 0x009fff:
  ret = rivatnt2m64_ptimer_read(addr, rivatnt2m64);
  break;
  case 0x100000 ... 0x100fff:
  ret = rivatnt2m64_pfb_read(addr, rivatnt2m64);
  break;
  case 0x101000 ... 0x101fff:
  ret = rivatnt2m64_pextdev_read(addr, rivatnt2m64);
  break;
  case 0x6013b4 ... 0x6013b5: case 0x6013d4 ... 0x6013d5: case 0x0c03c2 ... 0x0c03c5: case 0x0c03cc ... 0x0c03cf:
  ret = rivatnt2m64_in(addr & 0xfff, rivatnt2m64);
  break;
  case 0x680000 ... 0x680fff:
  ret = rivatnt2m64_pramdac_read(addr, rivatnt2m64);
  break;
  }
  return ret;
}

static uint16_t rivatnt2m64_mmio_read_w(uint32_t addr, void *p)
{
  addr &= 0xffffff;
  //pclog("RIVA TNT MMIO read %08X %04X:%08X\n", addr, CS, cpu_state.pc);
  return (rivatnt2m64_mmio_read(addr+0,p) << 0) | (rivatnt2m64_mmio_read(addr+1,p) << 8);
}

static uint32_t rivatnt2m64_mmio_read_l(uint32_t addr, void *p)
{
  addr &= 0xffffff;
  //pclog("RIVA TNT MMIO read %08X %04X:%08X\n", addr, CS, cpu_state.pc);
  return (rivatnt2m64_mmio_read(addr+0,p) << 0) | (rivatnt2m64_mmio_read(addr+1,p) << 8) | (rivatnt2m64_mmio_read(addr+2,p) << 16) | (rivatnt2m64_mmio_read(addr+3,p) << 24);
}

static void rivatnt2m64_mmio_write(uint32_t addr, uint8_t val, void *p)
{
  addr &= 0xffffff;
  //pclog("RIVA TNT MMIO write %08X %02X %04X:%08X\n", addr, val, CS, cpu_state.pc);
  if(addr != 0x6013d4 && addr != 0x6013d5 && addr != 0x6013b4 && addr != 0x6013b5)
  {
    uint32_t tmp = rivatnt2m64_mmio_read_l(addr,p);
    tmp &= ~(0xff << ((addr & 3) << 3));
    tmp |= val << ((addr & 3) << 3);
    rivatnt2m64_mmio_write_l(addr, tmp, p);
  }
  else
  {
    rivatnt2m64_out(addr & 0xfff, val & 0xff, p);
  }
}

static void rivatnt2m64_mmio_write_w(uint32_t addr, uint16_t val, void *p)
{
  addr &= 0xffffff;
  //pclog("RIVA TNT MMIO write %08X %04X %04X:%08X\n", addr, val, CS, cpu_state.pc);
  uint32_t tmp = rivatnt2m64_mmio_read_l(addr,p);
  tmp &= ~(0xffff << ((addr & 2) << 4));
  tmp |= val << ((addr & 2) << 4);
  rivatnt2m64_mmio_write_l(addr, tmp, p);
}

static void rivatnt2m64_mmio_write_l(uint32_t addr, uint32_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;

  addr &= 0xffffff;

  pclog("RIVA TNT MMIO write %08X %08X %04X:%08X\n", addr, val, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x000000 ... 0x000fff:
  rivatnt2m64_pmc_write(addr, val, rivatnt2m64);
  break;
  case 0x001000 ... 0x001fff:
  rivatnt2m64_pbus_write(addr, val, rivatnt2m64);
  break;
  case 0x002000 ... 0x002fff:
  rivatnt2m64_pfifo_write(addr, val, rivatnt2m64);
  break;
  case 0x100000 ... 0x100fff:
  rivatnt2m64_pfb_write(addr, val, rivatnt2m64);
  break;
  case 0x680000 ... 0x680fff:
  rivatnt2m64_pramdac_write(addr, val, rivatnt2m64);
  break;
  case 0x800000 ... 0xffffff:
  rivatnt2m64_user_write(addr, val, rivatnt2m64);
  break;
  }
}

static void rivatnt2m64_poll(void *p)
{
	rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
	svga_t *svga = &rivatnt2m64->svga;
}

static uint8_t rivatnt2m64_rma_in(uint16_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  addr &= 0xff;

  //pclog("RIVA TNT RMA read %04X %04X:%08X\n", addr, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x00: ret = 0x65; break;
  case 0x01: ret = 0xd0; break;
  case 0x02: ret = 0x16; break;
  case 0x03: ret = 0x2b; break;
  case 0x08: case 0x09: case 0x0a: case 0x0b: ret = rivatnt2m64_mmio_read(rivatnt2m64->rma.addr + (addr & 3), rivatnt2m64); break;
  }

  return ret;
}

static void rivatnt2m64_rma_out(uint16_t addr, uint8_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;

  addr &= 0xff;

  //pclog("RIVA TNT RMA write %04X %02X %04X:%08X\n", addr, val, CS, cpu_state.pc);

  switch(addr)
  {
  case 0x04:
  rivatnt2m64->rma.addr &= ~0xff;
  rivatnt2m64->rma.addr |= val;
  break;
  case 0x05:
  rivatnt2m64->rma.addr &= ~0xff00;
  rivatnt2m64->rma.addr |= (val << 8);
  break;
  case 0x06:
  rivatnt2m64->rma.addr &= ~0xff0000;
  rivatnt2m64->rma.addr |= (val << 16);
  break;
  case 0x07:
  rivatnt2m64->rma.addr &= ~0xff000000;
  rivatnt2m64->rma.addr |= (val << 24);
  break;
  case 0x08: case 0x0c: case 0x10: case 0x14:
  rivatnt2m64->rma.data &= ~0xff;
  rivatnt2m64->rma.data |= val;
  break;
  case 0x09: case 0x0d: case 0x11: case 0x15:
  rivatnt2m64->rma.data &= ~0xff00;
  rivatnt2m64->rma.data |= (val << 8);
  break;
  case 0x0a: case 0x0e: case 0x12: case 0x16:
  rivatnt2m64->rma.data &= ~0xff0000;
  rivatnt2m64->rma.data |= (val << 16);
  break;
  case 0x0b: case 0x0f: case 0x13: case 0x17:
  rivatnt2m64->rma.data &= ~0xff000000;
  rivatnt2m64->rma.data |= (val << 24);
  if(rivatnt2m64->rma.addr < 0x1000000) rivatnt2m64_mmio_write_l(rivatnt2m64->rma.addr & 0xffffff, rivatnt2m64->rma.data, rivatnt2m64);
  else svga_writel_linear((rivatnt2m64->rma.addr - 0x1000000) & 0xffffff, rivatnt2m64->rma.data, svga);
  break;
  }

  if(addr & 0x10) rivatnt2m64->rma.addr+=4;
}

static uint8_t rivatnt2m64_in(uint16_t addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;

  switch (addr)
  {
  case 0x3D0 ... 0x3D3:
  //pclog("RIVA TNT RMA BAR Register read %04X %04X:%08X\n", addr, CS, cpu_state.pc);
  if(!(rivatnt2m64->rma.mode & 1)) return ret;
  ret = rivatnt2m64_rma_in(rivatnt2m64->rma_addr + ((rivatnt2m64->rma.mode & 0xe) << 1) + (addr & 3), rivatnt2m64);
  return ret;
  }
  
  if (((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0) && !(svga->miscout & 1))
    addr ^= 0x60;

    //        if (addr != 0x3da) pclog("S3 in %04X %04X:%08X  ", addr, CS, cpu_state.pc);
  switch (addr)
  {
  case 0x3D4:
  ret = svga->crtcreg;
  break;
  case 0x3D5:
  switch(svga->crtcreg)
  {
  case 0x3e:
  ret = (i2c_gpio_get_sda(rivatnt2m64->i2c) << 3) | (i2c_gpio_get_scl(rivatnt2m64->i2c) << 2);
  break;
  default:
  ret = svga->crtc[svga->crtcreg];
  break;
  }
  //if(svga->crtcreg > 0x18)
  //  pclog("RIVA TNT Extended CRTC read %02X %04X:%08X\n", svga->crtcreg, CS, cpu_state.pc);
  break;
  default:
  ret = svga_in(addr, svga);
  break;
  }
  //        if (addr != 0x3da) pclog("%02X\n", ret);
  return ret;
}

static void rivatnt2m64_out(uint16_t addr, uint8_t val, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;

  uint8_t old;
  
  switch(addr)
  {
  case 0x3D0 ... 0x3D3:
  //pclog("RIVA TNT RMA BAR Register write %04X %02x %04X:%08X\n", addr, val, CS, cpu_state.pc);
  rivatnt2m64->rma.access_reg[addr & 3] = val;
  if(!(rivatnt2m64->rma.mode & 1)) return;
  rivatnt2m64_rma_out(rivatnt2m64->rma_addr + ((rivatnt2m64->rma.mode & 0xe) << 1) + (addr & 3), rivatnt2m64->rma.access_reg[addr & 3], rivatnt2m64);
  return;
  }

  if (((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0) && !(svga->miscout & 1))
    addr ^= 0x60;

  switch(addr)
  {
  case 0x3D4:
  svga->crtcreg = val;
  return;
  case 0x3D5:
  if ((svga->crtcreg < 7) && (svga->crtc[0x11] & 0x80))
    return;
  if ((svga->crtcreg == 7) && (svga->crtc[0x11] & 0x80))
    val = (svga->crtc[7] & ~0x10) | (val & 0x10);
  old = svga->crtc[svga->crtcreg];
  svga->crtc[svga->crtcreg] = val;
  switch(svga->crtcreg)
  {
  case 0x1a:
  //if(val & 2) svga->dac8bit = 1;
  //else svga->dac8bit = 0;
  svga_recalctimings(svga);
  break;
  case 0x1e:
  rivatnt2m64->read_bank = val;
  if (svga->chain4) svga->read_bank = rivatnt2m64->read_bank << 15;
  else              svga->read_bank = rivatnt2m64->read_bank << 13;
  break;
  case 0x1d:
  rivatnt2m64->write_bank = val;
  if (svga->chain4) svga->write_bank = rivatnt2m64->write_bank << 15;
  else              svga->write_bank = rivatnt2m64->write_bank << 13;
  break;
  case 0x26:
  if (!svga->attrff)
    svga->attraddr = val & 31;
  break;
  case 0x19:
  case 0x25:
  case 0x28:
  case 0x2d:
  svga_recalctimings(svga);
  break;
  case 0x38:
  rivatnt2m64->rma.mode = val & 0xf;
  break;
  case 0x3f:
  i2c_gpio_set(rivatnt2m64->i2c, !!(val & 0x20), !!(val & 0x10));
  break;
  }
  //if(svga->crtcreg > 0x18)
  //  pclog("RIVA TNT Extended CRTC write %02X %02x %04X:%08X\n", svga->crtcreg, val, CS, cpu_state.pc);
  if (old != val)
  {
    if (svga->crtcreg < 0xE || svga->crtcreg > 0x10)
    {
      svga->fullchange = changeframecount;
      svga_recalctimings(svga);
    }
  }
  return;
  case 0x3C5:
  if(svga->seqaddr == 6) rivatnt2m64->ext_regs_locked = val;
  break;
  }

  svga_out(addr, val, svga);
}

static uint8_t rivatnt2m64_pci_read(int func, int addr, void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  uint8_t ret = 0;
  //pclog("RIVA TNT PCI read %02X %04X:%08X\n", addr, CS, cpu_state.pc);
  switch (addr)
  {
    case 0x00: ret = 0xde; break; /*'nVidia'*/
    case 0x01: ret = 0x10; break;

    case 0x02: ret = 0x2d; break; /*'RIVA TNT2 Model 64'*/
    case 0x03: ret = 0x00; break;

    case 0x04: ret = rivatnt2m64->pci_regs[0x04] & 0x37; break;
    case 0x05: ret = rivatnt2m64->pci_regs[0x05]; break;

    case 0x06: ret = 0xb0; break;
    case 0x07: ret = 0x02; break;

    case 0x08: ret = 0x15; break; /*Revision ID*/
    case 0x09: ret = 0; break; /*Programming interface*/

    case 0x0a: ret = 0x00; break; /*Supports VGA interface*/
    case 0x0b: ret = 0x03; /*output = 3; */break;

    case 0x0d: ret = 0xf8; break;

    case 0x0e: ret = 0x00; break; /*Header type*/

    case 0x13:
    case 0x17:
    ret = rivatnt2m64->pci_regs[addr];
    break;

    case 0x2c: case 0x2d: case 0x2e: case 0x2f:
    ret = rivatnt2m64->pci_regs[addr];
    //if(CS == 0x0028) output = 3;
    break;

    case 0x34: ret = 0x60; break;

    case 0x3c: ret = rivatnt2m64->pci_regs[0x3c]; break;

    case 0x3d: ret = 0x01; break; /*INTA*/

    case 0x3e: ret = 0x03; break;
    case 0x3f: ret = 0x01; break;

    case 0x44: ret = 0x02; break;
    case 0x45: ret = 0x00; break;
    case 0x46: ret = 0x20; break;
    case 0x47: ret = 0x00; break;

    case 0x48: ret = 0x03; break;
    case 0x49: ret = 0x02; break;
    case 0x4a: ret = 0x00; break;
    case 0x4b: ret = 0x1f; break;

    case 0x4c: ret = rivatnt2m64->pci_regs[0x4c] & 0x7; break;
    case 0x4d: ret = rivatnt2m64->pci_regs[0x4d] & 0x3; break;
    case 0x4e: ret = rivatnt2m64->pci_regs[0x4e]; break;
    case 0x4f: ret = rivatnt2m64->pci_regs[0x4f]; break;

    case 0x60: ret = 0x01; break;
    case 0x61: ret = 0x44; break;
    case 0x62: ret = 0x02; break;
    case 0x63: ret = 0x00; break;
  }
  //        pclog("%02X\n", ret);
  return ret;
}

static void rivatnt2m64_pci_write(int func, int addr, uint8_t val, void *p)
{
  //pclog("RIVA TNT PCI write %02X %02X %04X:%08X\n", addr, val, CS, cpu_state.pc);
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
  svga_t *svga = &rivatnt2m64->svga;
  switch (addr)
  {
    case 0x00: case 0x01: case 0x02: case 0x03:
    case 0x08: case 0x09: case 0x0a: case 0x0b:
    case 0x3d: case 0x3e: case 0x3f:
    return;

    case PCI_REG_COMMAND:
    if (val & PCI_COMMAND_IO)
    {
      io_removehandler(0x03c0, 0x0020, rivatnt2m64_in, NULL, NULL, rivatnt2m64_out, NULL, NULL, rivatnt2m64);
      io_sethandler(0x03c0, 0x0020, rivatnt2m64_in, NULL, NULL, rivatnt2m64_out, NULL, NULL, rivatnt2m64);
    }
    else
      io_removehandler(0x03c0, 0x0020, rivatnt2m64_in, NULL, NULL, rivatnt2m64_out, NULL, NULL, rivatnt2m64);
    rivatnt2m64->pci_regs[PCI_REG_COMMAND] = val & 0x37;
    return;

    case 0x05:
    rivatnt2m64->pci_regs[0x05] = val;
    return;

    case 0x07:
    rivatnt2m64->pci_regs[0x07] = (rivatnt2m64->pci_regs[0x07] & 0x8f) | (val & 0x70);
    return;

    case 0x13:
    {
      rivatnt2m64->pci_regs[addr] = val;
      uint32_t mmio_addr = val << 24;
      mem_mapping_set_addr(&rivatnt2m64->mmio_mapping, mmio_addr, 0x1000000);
      return;
    }

    case 0x17:
    {
      rivatnt2m64->pci_regs[addr] = val;
      uint32_t linear_addr = (val << 24);
      mem_mapping_set_addr(&rivatnt2m64->linear_mapping, linear_addr, 0x2000000);
      return;
    }

    case 0x30: case 0x32: case 0x33:
    rivatnt2m64->pci_regs[addr] = val;
    if (rivatnt2m64->pci_regs[0x30] & 0x01)
    {
      uint32_t addr = (rivatnt2m64->pci_regs[0x32] << 16) | (rivatnt2m64->pci_regs[0x33] << 24);
      //                        pclog("RIVA TNT bios_rom enabled at %08x\n", addr);
      mem_mapping_set_addr(&rivatnt2m64->bios_rom.mapping, addr, 0x10000);
      mem_mapping_enable(&rivatnt2m64->bios_rom.mapping);
    }
    else
    {
      //                        pclog("RIVA TNT bios_rom disabled\n");
      mem_mapping_disable(&rivatnt2m64->bios_rom.mapping);
    }
    return;

    case 0x40: case 0x41: case 0x42: case 0x43:
    rivatnt2m64->pci_regs[addr - 0x14] = val; //0x40-0x43 are ways to write to 0x2c-0x2f
    return;

    case 0x4c:
    rivatnt2m64->pci_regs[0x4c] = val & 0x7;
    return;

    case 0x4d:
    rivatnt2m64->pci_regs[0x4d] = val & 0x3;
    return;

    case 0x4e:
    rivatnt2m64->pci_regs[0x4e] = val;
    return;

    case 0x4f:
    rivatnt2m64->pci_regs[0x4f] = val;
    return;
  }
}

static void rivatnt2m64_recalctimings(svga_t *svga)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)svga->priv;

  svga->ma_latch += (svga->crtc[0x19] & 0x1f) << 16;
  svga->rowoffset += (svga->crtc[0x19] & 0xe0) << 3;
  if (svga->crtc[0x25] & 0x01) svga->vtotal      += 0x400;
  if (svga->crtc[0x25] & 0x02) svga->dispend     += 0x400;
  if (svga->crtc[0x25] & 0x04) svga->vblankstart += 0x400;
  if (svga->crtc[0x25] & 0x08) svga->vsyncstart  += 0x400;
  if (svga->crtc[0x25] & 0x10) svga->htotal      += 0x100;
  if (svga->crtc[0x2d] & 0x01) svga->hdisp       += 0x100;
  //The effects of the large screen bit seem to just be doubling the row offset.
  //However, these large modes still don't work. Possibly core SVGA bug? It does report 640x2 res after all.
  if (!(svga->crtc[0x1a] & 0x04)) svga->rowoffset <<= 1;
  switch(svga->crtc[0x28] & 3)
  {
    case 1:
    svga->bpp = 8;
    svga->lowres = 0;
    svga->render = svga_render_8bpp_highres;
    break;
    case 2:
    svga->bpp = 16;
    svga->lowres = 0;
    svga->render = svga_render_16bpp_highres;
    break;
    case 3:
    svga->bpp = 32;
    svga->lowres = 0;
    svga->render = svga_render_32bpp_highres;
    break;
  }
  
  if (((svga->miscout >> 2) & 2) == 2)
  {
	double freq = 13500000.0;

	if(rivatnt2m64->pramdac.v_m == 0) freq = 0;
	else
	{
		freq = (freq * rivatnt2m64->pramdac.v_n) / (1 << rivatnt2m64->pramdac.v_p) / rivatnt2m64->pramdac.v_m;
		//pclog("RIVA TNT Pixel clock is %f Hz\n", freq);
	}
	
        svga->clock = cpuclock / freq;
  }
}

static void *rivatnt2m64_init()
{
  rivatnt2m64_t *rivatnt2m64 = malloc(sizeof(rivatnt2m64_t));
  memset(rivatnt2m64, 0, sizeof(rivatnt2m64_t));

  rivatnt2m64->memory_size = device_get_config_int("memory");

  svga_init(&rivatnt2m64->svga, rivatnt2m64, rivatnt2m64->memory_size << 20,
  rivatnt2m64_recalctimings,
  rivatnt2m64_in, rivatnt2m64_out,
  NULL, NULL);

  rivatnt2m64->svga.decode_mask = 0x3fffff;
  rivatnt2m64->svga.force_old_addr = 1;

  rom_init(&rivatnt2m64->bios_rom, "roms/video/nvidia/w2137.rom", 0xc0000, 0x10000, 0xffff, 0, MEM_MAPPING_EXTERNAL);
  mem_mapping_disable(&rivatnt2m64->bios_rom.mapping);

  mem_mapping_add(&rivatnt2m64->mmio_mapping,     0, 0,
    rivatnt2m64_mmio_read,
    rivatnt2m64_mmio_read_w,
    rivatnt2m64_mmio_read_l,
    rivatnt2m64_mmio_write,
    rivatnt2m64_mmio_write_w,
    rivatnt2m64_mmio_write_l,
    NULL,
    0,
    rivatnt2m64);
  mem_mapping_add(&rivatnt2m64->linear_mapping,   0, 0,
    svga_read_linear,
    svga_readw_linear,
    svga_readl_linear,
    svga_write_linear,
    svga_writew_linear,
    svga_writel_linear,
    NULL,
    0,
    &rivatnt2m64->svga);

  rivatnt2m64->pci_regs[0x04] = 0x07;
  rivatnt2m64->pci_regs[0x05] = 0x00;
  rivatnt2m64->pci_regs[0x07] = 0x02;
  
  rivatnt2m64->pmc.intr = 0;
  rivatnt2m64->pbus.intr = 0;
  rivatnt2m64->pfifo.intr = 0;
  rivatnt2m64->pgraph.intr = 0;

  rivatnt2m64->pramdac.mpll = 0x03c20d;
  rivatnt2m64->pramdac.nvpll = 0x03c20d;
  rivatnt2m64->pramdac.vpll = 0x03c20d;
  
  pci_add_card(PCI_ADD_AGP, rivatnt2m64_pci_read, rivatnt2m64_pci_write, rivatnt2m64, &rivatnt2m64->pci_slot);

  rivatnt2m64->i2c = i2c_gpio_init("ddc_rivatnt2m64");
  rivatnt2m64->ddc = ddc_init(i2c_gpio_get_bus(rivatnt2m64->i2c));

  return rivatnt2m64;
}

static void rivatnt2m64_close(void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

  svga_close(&rivatnt2m64->svga);

  ddc_close(rivatnt2m64->ddc);
  i2c_gpio_close(rivatnt2m64->i2c);

  free(rivatnt2m64);
}

static int rivatnt2m64_available(void)
{
  return rom_present("roms/video/nvidia/w2137.rom");
}

static void rivatnt2m64_speed_changed(void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

  svga_recalctimings(&rivatnt2m64->svga);
}

static void rivatnt2m64_force_redraw(void *p)
{
  rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

  rivatnt2m64->svga.fullchange = changeframecount;
}

static const device_config_t rivatnt2m64_config[] = {
  // clang-format off
    {
        .name = "memory",
        .description = "Memory size",
        .type = CONFIG_SELECTION,
        .default_int = 32,
        .selection = {
            {
                .description = "4 MB",
                .value = 4
            },
            {
                .description = "8 MB",
                .value = 8
            },
            {
                .description = "16 MB",
                .value = 16
            },
            {
                .description = "32 MB",
                .value = 32
            },
            {
                .description = ""
            }
        }
    },
    {
        .type = CONFIG_END
    }
  // clang-format on
};

const device_t rivatnt2m64_device = {
    .name = "nVidia RIVA TNT2 Model 64",
    .internal_name = "rivatnt2m64",
    .flags = DEVICE_AGP,
    .local = 0,
    .init = rivatnt2m64_init,
    .close = rivatnt2m64_close, 
    .reset = NULL,
    { .available = rivatnt2m64_available },
    .speed_changed = rivatnt2m64_speed_changed,
    .force_redraw = rivatnt2m64_force_redraw,
    .config = rivatnt2m64_config
};
