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

#define BIOS_RIVATNT2M64_PATH   "roms/video/nvidia/w2137.rom"

#define RIVATNT2M64_VENDOR_ID 0x10de
#define RIVATNT2M64_DEVICE_ID 0x002d

typedef struct rivatnt2m64_t
{
    mem_mapping_t mmio_mapping;
    mem_mapping_t   linear_mapping;

    svga_t    svga;

    rom_t   bios_rom;

    uint32_t    vram_size, vram_mask,
            mmio_base, lfb_base;

    uint8_t   read_bank, write_bank;

    uint8_t   pci_regs[256];
    uint8_t     pci_slot;
    uint8_t     irq_state;
    uint8_t   int_line;

    int     card;

    struct
    {
        uint8_t rma_access_reg[4];
        uint8_t rma_mode;
        uint32_t rma_dst_addr;
        uint32_t rma_data;
    } rma;

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
        uint32_t cache_error;

        struct
        {
            uint32_t push_enabled, pull_enabled;
            uint32_t status0, status1;
            uint32_t put, get;
        } caches[2];
    } pfifo;

    struct
    {
        uint32_t intr, intr_en;

        uint64_t time;
        uint32_t alarm;

        uint16_t clock_mul, clock_div;
    } ptimer;

    struct
    {
        uint16_t width;
        int bpp;

        uint32_t config_0;
    } pfb;
    
    struct
    {
        uint32_t intr, intr_en;
    } pcrtc;

    struct
    {
        uint32_t fifo_enable;
    } pgraph;
    

    struct
    {
        uint32_t nvpll, mpll, vpll;
    } pramdac;

    uint32_t ramin[0x100000/4];

    pc_timer_t nvtimer;
    pc_timer_t mtimer;

    double nvtime;
    double mtime;

    void *i2c, *ddc;
} rivatnt2m64_t;

static video_timings_t timing_rivatnt2m64 = { .type = VIDEO_AGP, .write_b = 2, .write_w = 2, .write_l = 3, .read_b = 24, .read_w = 24, .read_l = 36 };

static uint8_t rivatnt2m64_in(uint16_t addr, void *p);
static void rivatnt2m64_out(uint16_t addr, uint8_t val, void *p);

static uint8_t 
rivatnt2m64_pci_read(int func, int addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    // svga_t *svga = &rivatnt2m64->svga;

    //pclog("RIVA TNT PCI read %02x\n", addr);

    switch (addr) {
    case 0x00: return 0xde; /*nVidia*/
    case 0x01: return 0x10;

    case 0x02: return 0x2d;
    case 0x03: return 0x00;
    
    case 0x04: return rivatnt2m64->pci_regs[0x04] & 0x37; /*Respond to IO and memory accesses*/
    case 0x05: return rivatnt2m64->pci_regs[0x05];

    case 0x06: return 0xb0;
    case 0x07: return 0x02;

    case 0x08: return 0x15; /*Revision ID*/
    case 0x09: return 0x00; /*Programming interface*/

    case 0x0a: return 0x00; /*Supports VGA interface*/
    case 0x0b: return 0x03;

    case 0x10: return 0x00;
    case 0x11: return 0x00;
    case 0x12: return 0x00;
    case 0x13: return rivatnt2m64->mmio_base >> 24;

    case 0x14: return 0x08;
    case 0x15: return 0x00;
    case 0x16: return 0x00;
    case 0x17: return rivatnt2m64->lfb_base >> 24;

    case 0x2c: case 0x2d: case 0x2e: case 0x2f:
        return rivatnt2m64->pci_regs[addr];

    case 0x30: return (rivatnt2m64->pci_regs[0x30] & 0x01); /*BIOS ROM address*/
    case 0x31: return 0x00;
    case 0x32: return rivatnt2m64->pci_regs[0x32];
    case 0x33: return rivatnt2m64->pci_regs[0x33];

    case 0x34: return 0x60;

    case 0x3c: return rivatnt2m64->int_line;
    case 0x3d: return PCI_INTA;

    case 0x3e: return 0x05;
    case 0x3f: return 0x01;

    case 0x44: return 0x02;
    case 0x45: return 0x00;
    case 0x46: return 0x20;
    case 0x47: return 0x00;

    case 0x48: return 0x03;
    case 0x49: return 0x02;
    case 0x4a: return 0x00;
    case 0x4b: return 0x1f;

    case 0x4c: return rivatnt2m64->pci_regs[0x4c] & 0x7;
    case 0x4d: return rivatnt2m64->pci_regs[0x4d] & 0x3;
    case 0x4e: return rivatnt2m64->pci_regs[0x4e];
    case 0x4f: return rivatnt2m64->pci_regs[0x4f];

    case 0x60: return 0x01;
    case 0x61: return 0x44;
    case 0x62: return 0x02;
    case 0x63: return 0x00;
    }

    return 0x00;
}


static void 
rivatnt2m64_recalc_mapping(rivatnt2m64_t *rivatnt2m64)
{
    svga_t *svga = &rivatnt2m64->svga;
        
    if (!(rivatnt2m64->pci_regs[PCI_REG_COMMAND] & PCI_COMMAND_MEM)) {
    //pclog("PCI mem off\n");
        mem_mapping_disable(&svga->mapping);
        mem_mapping_disable(&rivatnt2m64->mmio_mapping);
        mem_mapping_disable(&rivatnt2m64->linear_mapping);
    return;
    }

    //pclog("PCI mem on\n");
    //pclog("rivatnt2m64->mmio_base = %08X\n", rivatnt2m64->mmio_base);
    if (rivatnt2m64->mmio_base)
        mem_mapping_set_addr(&rivatnt2m64->mmio_mapping, rivatnt2m64->mmio_base, 0x1000000);
    else
        mem_mapping_disable(&rivatnt2m64->mmio_mapping);

    //pclog("rivatnt2m64->lfb_base = %08X\n", rivatnt2m64->lfb_base);
    if (rivatnt2m64->lfb_base) {
    mem_mapping_set_addr(&rivatnt2m64->linear_mapping, rivatnt2m64->lfb_base, 0x1000000);
    } else {
        mem_mapping_disable(&rivatnt2m64->linear_mapping);
    }

    switch (svga->gdcreg[6] & 0x0c) {
    case 0x0: /*128k at A0000*/
        mem_mapping_set_addr(&svga->mapping, 0xa0000, 0x20000);
        svga->banked_mask = 0xffff;
        break;
    case 0x4: /*64k at A0000*/
        mem_mapping_set_addr(&svga->mapping, 0xa0000, 0x10000);
        svga->banked_mask = 0xffff;
        break;
    case 0x8: /*32k at B0000*/
        mem_mapping_set_addr(&svga->mapping, 0xb0000, 0x08000);
        svga->banked_mask = 0x7fff;
        break;
    case 0xC: /*32k at B8000*/
        mem_mapping_set_addr(&svga->mapping, 0xb8000, 0x08000);
        svga->banked_mask = 0x7fff;
        break;
    }
}


static void 
rivatnt2m64_pci_write(int func, int addr, uint8_t val, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    //pclog("RIVA TNT PCI write %02x %02x\n", addr, val);

    switch (addr) {
    case PCI_REG_COMMAND:
        rivatnt2m64->pci_regs[PCI_REG_COMMAND] = val & 0x37;
        io_removehandler(0x03c0, 0x0020, rivatnt2m64_in, NULL, NULL, rivatnt2m64_out, NULL, NULL, rivatnt2m64);
        if (val & PCI_COMMAND_IO)
            io_sethandler(0x03c0, 0x0020, rivatnt2m64_in, NULL, NULL, rivatnt2m64_out, NULL, NULL, rivatnt2m64);
        rivatnt2m64_recalc_mapping(rivatnt2m64);
        break;

    case 0x05:
        rivatnt2m64->pci_regs[0x05] = val;
        break;

    case 0x13:
        rivatnt2m64->mmio_base = val << 24;
        rivatnt2m64_recalc_mapping(rivatnt2m64);
        break;

    case 0x17: 
        rivatnt2m64->lfb_base = val << 24;
        rivatnt2m64_recalc_mapping(rivatnt2m64);
        break;

    case 0x30: case 0x32: case 0x33:
        rivatnt2m64->pci_regs[addr] = val;
        if (rivatnt2m64->pci_regs[0x30] & 0x01) {
            uint32_t addr = (rivatnt2m64->pci_regs[0x32] << 16) | (rivatnt2m64->pci_regs[0x33] << 24);
            mem_mapping_set_addr(&rivatnt2m64->bios_rom.mapping, addr, 0x10000);
        } else
            mem_mapping_disable(&rivatnt2m64->bios_rom.mapping);
        break;

    case 0x3c:
        rivatnt2m64->int_line = val;
        break;

    case 0x40: case 0x41: case 0x42: case 0x43:
        /* 0x40-0x43 are ways to write to 0x2c-0x2f */
        rivatnt2m64->pci_regs[0x2c + (addr & 0x03)] = val;
        break;

    case 0x4c:
        rivatnt2m64->pci_regs[0x4c] = val & 0x7;
        break;

    case 0x4d:
        rivatnt2m64->pci_regs[0x4d] = val & 0x3;
        break;

    case 0x4e:
        rivatnt2m64->pci_regs[0x4e] = val;
        break;

    case 0x4f:
        rivatnt2m64->pci_regs[0x4f] = val;
        break;
    }
}

uint32_t
rivatnt2m64_pmc_recompute_intr(void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    uint32_t intr = 0;
    if(rivatnt2m64->pfifo.intr & rivatnt2m64->pfifo.intr_en) intr |= (1 << 8);
    if(rivatnt2m64->ptimer.intr & rivatnt2m64->ptimer.intr_en) intr |= (1 << 20);
    if(rivatnt2m64->pcrtc.intr & rivatnt2m64->pcrtc.intr_en) intr |= (1 << 24);
    if(rivatnt2m64->pmc.intr & (1u << 31)) intr |= (1u << 31);
    
    if((intr & 0x7fffffff) && (rivatnt2m64->pmc.intr_en & 1)) pci_set_irq(rivatnt2m64->pci_slot, PCI_INTA, &rivatnt2m64->irq_state);
    else if((intr & (1 << 31)) && (rivatnt2m64->pmc.intr_en & 2)) pci_set_irq(rivatnt2m64->pci_slot, PCI_INTA, &rivatnt2m64->irq_state);
    //else pci_clear_irq(rivatnt2m64->pci_slot, PCI_INTA, &rivatnt2m64->irq_state);
    return intr;
}

uint32_t
rivatnt2m64_pmc_read(uint32_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    switch(addr)
    {
    case 0x000000:
        return 0x20154000; //ID register.
    case 0x000100:
        return rivatnt2m64_pmc_recompute_intr(rivatnt2m64);
    case 0x000140:
        return rivatnt2m64->pmc.intr_en;
    case 0x000200:
        return rivatnt2m64->pmc.enable;
    }
    return 0;
}

void
rivatnt2m64_pmc_write(uint32_t addr, uint32_t val, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    switch(addr)
    {
    case 0x000100:
        rivatnt2m64->pmc.intr = val & (1u << 31);
        rivatnt2m64_pmc_recompute_intr(rivatnt2m64);
        break;
    case 0x000140:
        rivatnt2m64->pmc.intr_en = val & 3;
        rivatnt2m64_pmc_recompute_intr(rivatnt2m64);
        break;
    case 0x000200:
        rivatnt2m64->pmc.enable = val;
        break;
    }
}

uint32_t
rivatnt2m64_pfifo_read(uint32_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    switch(addr)
    {
    case 0x002080:
        return rivatnt2m64->pfifo.cache_error;
    case 0x002100:
        return rivatnt2m64->pfifo.intr;
    case 0x002140:
        return rivatnt2m64->pfifo.intr_en;
    }
    return 0;
}

void
rivatnt2m64_pfifo_write(uint32_t addr, uint32_t val, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    switch(addr)
    {
    case 0x002100:
    {
        uint32_t tmp = rivatnt2m64->pfifo.intr & ~val;
        rivatnt2m64->pfifo.intr = tmp;
        pci_clear_irq(rivatnt2m64->pci_slot, PCI_INTA, &rivatnt2m64->irq_state);
        if(!(rivatnt2m64->pfifo.intr & 1)) rivatnt2m64->pfifo.cache_error = 0;
        break;
    }
    case 0x002140:
        rivatnt2m64->pfifo.intr_en = val & 0x11111;
        rivatnt2m64_pmc_recompute_intr(rivatnt2m64);
        break;
    case 0x003000:
        rivatnt2m64->pfifo.caches[0].push_enabled = val & 1;
        break;
    case 0x003010:
        rivatnt2m64->pfifo.caches[0].put = val;
        break;
    case 0x003050:
        rivatnt2m64->pfifo.caches[0].pull_enabled = val & 1;
        break;
    case 0x003070:
        rivatnt2m64->pfifo.caches[0].get = val;
        break;
    case 0x003200:
        rivatnt2m64->pfifo.caches[1].push_enabled = val & 1;
        break;
    case 0x003210:
        rivatnt2m64->pfifo.caches[1].put = val;
        break;
    case 0x003250:
        rivatnt2m64->pfifo.caches[1].pull_enabled = val & 1;
        break;
    case 0x003270:
        rivatnt2m64->pfifo.caches[1].get = val;
        break;
    }
}

void
rivatnt2m64_ptimer_interrupt(int num, void *p)
{
    //nv_riva_log("RIVA TNT PTIMER interrupt #%d fired!\n", num);
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    rivatnt2m64->ptimer.intr |= (1 << num);

    rivatnt2m64_pmc_recompute_intr(rivatnt2m64);
}

uint32_t
rivatnt2m64_ptimer_read(uint32_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    switch(addr)
    {
    case 0x009100:
        return rivatnt2m64->ptimer.intr;
    case 0x009140:
        return rivatnt2m64->ptimer.intr_en;
    case 0x009200:
        return rivatnt2m64->ptimer.clock_div;
    case 0x009210:
        return rivatnt2m64->ptimer.clock_mul;
    case 0x009400:
        return rivatnt2m64->ptimer.time & 0xffffffffULL;
    case 0x009410:
        return rivatnt2m64->ptimer.time >> 32;
    case 0x009420:
        return rivatnt2m64->ptimer.alarm;
    }
    return 0;
}

void
rivatnt2m64_ptimer_write(uint32_t addr, uint32_t val, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    switch(addr)
    {
    case 0x009100:
        rivatnt2m64->ptimer.intr &= ~val;
        rivatnt2m64_pmc_recompute_intr(rivatnt2m64);
        break;
    case 0x009140:
        rivatnt2m64->ptimer.intr_en = val & 1;
        rivatnt2m64_pmc_recompute_intr(rivatnt2m64);
        break;
    case 0x009200:
        if(!(uint16_t)val) val = 1;
        rivatnt2m64->ptimer.clock_div = (uint16_t)val;
        break;
    case 0x009210:
        rivatnt2m64->ptimer.clock_mul = (uint16_t)val;
        break;
    case 0x009400:
        rivatnt2m64->ptimer.time &= 0x0fffffff00000000ULL;
        rivatnt2m64->ptimer.time |= val & 0xffffffe0;
        break;
    case 0x009410:
        rivatnt2m64->ptimer.time &= 0xffffffe0;
        rivatnt2m64->ptimer.time |= (uint64_t)(val & 0x0fffffff) << 32;
        break;
    case 0x009420:
        rivatnt2m64->ptimer.alarm = val & 0xffffffe0;
        //HACK to make wfw3.11 not take forever to start
        if(val == 0xffffffff)
        {
            rivatnt2m64_ptimer_interrupt(0, rivatnt2m64);
        }
        break;
    }
}

uint32_t
rivatnt2m64_pfb_read(uint32_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    uint8_t ret = 0;

    switch(addr)
    {
        case 0x100000:
            switch(rivatnt2m64->vram_size)
            {
                case 4 << 20: ret = 1; break;
                case 8 << 20: ret = 2; break;
                case 16 << 20: ret = 3; break;
                case 32 << 20: ret = 0; break;
            }
            ret |= 0x14;
            break;
    }

    return ret;
}

uint32_t
rivatnt2m64_pextdev_read(uint32_t addr, void *p)
{
    //rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    switch(addr)
    {
        case 0x101000:
            return 0x0000019e;
    }

    return 0;
}

uint32_t
rivatnt2m64_pcrtc_read(uint32_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    switch(addr)
    {
        case 0x600100:
            return rivatnt2m64->pcrtc.intr;
        case 0x600140:
            return rivatnt2m64->pcrtc.intr_en;
    }
    return 0;
}

void
rivatnt2m64_pcrtc_write(uint32_t addr, uint32_t val, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    switch(addr)
    {
        case 0x600100:
            rivatnt2m64->pcrtc.intr &= ~val;
            rivatnt2m64_pmc_recompute_intr(rivatnt2m64);
            break;
        case 0x600140:
            rivatnt2m64->pcrtc.intr_en = val & 1;
            rivatnt2m64_pmc_recompute_intr(rivatnt2m64);
            break;
    }
}

uint32_t
rivatnt2m64_pramdac_read(uint32_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    switch(addr)
    {
        case 0x680500:
            return rivatnt2m64->pramdac.nvpll;
        case 0x680504:
            return rivatnt2m64->pramdac.mpll;
        case 0x680508:
            return rivatnt2m64->pramdac.vpll;
    }
    return 0;
}

void
rivatnt2m64_pramdac_write(uint32_t addr, uint32_t val, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    switch(addr)
    {
        case 0x680500:
            rivatnt2m64->pramdac.nvpll = val;
            break;
        case 0x680504:
            rivatnt2m64->pramdac.mpll = val;
            break;
        case 0x680508:
            rivatnt2m64->pramdac.vpll = val;
            break;
    }
    svga_recalctimings(&rivatnt2m64->svga);
}

void
rivatnt2m64_ptimer_tick(void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    //pclog("[RIVA TNT] PTIMER tick! mul %04x div %04x\n", rivatnt2m64->ptimer.clock_mul, rivatnt2m64->ptimer.clock_div);

    double time = ((double)rivatnt2m64->ptimer.clock_mul * 10.0) / (double)rivatnt2m64->ptimer.clock_div; //Multiply by 10 to avoid timer system limitations.
    //uint32_t tmp;
    int alarm_check;

    //if(cs == 0x0008 && !rivatnt2m64->pgraph.beta) nv_riva_log("RIVA TNT PTIMER time elapsed %f alarm %08x, time_low %08x\n", time, rivatnt2m64->ptimer.alarm, rivatnt2m64->ptimer.time & 0xffffffff);

    //tmp = rivatnt2m64->ptimer.time;
    rivatnt2m64->ptimer.time += (uint64_t)time;

    alarm_check = (uint32_t)(rivatnt2m64->ptimer.time - rivatnt2m64->ptimer.alarm) & 0x80000000;

    //alarm_check = ((uint32_t)rivatnt2m64->ptimer.time >= (uint32_t)rivatnt2m64->ptimer.alarm);

    //pclog("[RIVA TNT] Timer %08x %016llx %08x %d\n", rivatnt2m64->ptimer.alarm, rivatnt2m64->ptimer.time, tmp, alarm_check);

    if(alarm_check)
    {
        pclog("[RIVA TNT] PTIMER ALARM interrupt fired!\n");
        rivatnt2m64_ptimer_interrupt(0, rivatnt2m64);
    }
}

void
rivatnt2m64_nvclk_poll(void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    rivatnt2m64_ptimer_tick(rivatnt2m64);
    timer_on_auto(&rivatnt2m64->nvtimer, rivatnt2m64->nvtime);
}

void
rivatnt2m64_mclk_poll(void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    timer_on_auto(&rivatnt2m64->mtimer, rivatnt2m64->mtime);
}

uint32_t
rivatnt2m64_mmio_read_l(uint32_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    addr &= 0xffffff;

    uint32_t ret = 0;

    switch(addr) {
    case 0x6013b4: case 0x6013b5:
    case 0x6013d4: case 0x6013d5:
    case 0x6013da:
    case 0x6013c0:
    case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
        ret = (rivatnt2m64_in((addr+0) & 0x3ff,p) << 0) | (rivatnt2m64_in((addr+1) & 0x3ff,p) << 8) | (rivatnt2m64_in((addr+2) & 0x3ff,p) << 16) | (rivatnt2m64_in((addr+3) & 0x3ff,p) << 24);
        break;
    }

    addr &= 0xfffffc;

    if ((addr >= 0x000000) && (addr <= 0x000fff)) ret = rivatnt2m64_pmc_read(addr, rivatnt2m64);
    if ((addr >= 0x002000) && (addr <= 0x003fff)) ret = rivatnt2m64_pfifo_read(addr, rivatnt2m64);
    if ((addr >= 0x009000) && (addr <= 0x009fff)) ret = rivatnt2m64_ptimer_read(addr, rivatnt2m64);
    if ((addr >= 0x100000) && (addr <= 0x100fff)) ret = rivatnt2m64_pfb_read(addr, rivatnt2m64);
    if ((addr >= 0x101000) && (addr <= 0x101fff)) ret = rivatnt2m64_pextdev_read(addr, rivatnt2m64);
    if ((addr >= 0x600000) && (addr <= 0x600fff)) ret = rivatnt2m64_pcrtc_read(addr, rivatnt2m64);
    if ((addr >= 0x680000) && (addr <= 0x680fff)) ret = rivatnt2m64_pramdac_read(addr, rivatnt2m64);
    if ((addr >= 0x700000) && (addr <= 0x7fffff)) ret = rivatnt2m64->ramin[(addr & 0xfffff) >> 2];
    if ((addr >= 0x300000) && (addr <= 0x30ffff)) ret = ((uint32_t *) rivatnt2m64->bios_rom.rom)[(addr & rivatnt2m64->bios_rom.mask) >> 2];

    if ((addr >= 0x1800) && (addr <= 0x18ff))
        ret = (rivatnt2m64_pci_read(0,(addr+0) & 0xff,p) << 0) | (rivatnt2m64_pci_read(0,(addr+1) & 0xff,p) << 8) | (rivatnt2m64_pci_read(0,(addr+2) & 0xff,p) << 16) | (rivatnt2m64_pci_read(0,(addr+3) & 0xff,p) << 24);

    if(addr != 0x9400) pclog("[RIVA TNT] MMIO read %08x returns value %08x\n", addr, ret);

    return ret;
}


uint8_t
rivatnt2m64_mmio_read(uint32_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    addr &= 0xffffff;

    if ((addr >= 0x300000) && (addr <= 0x30ffff)) return rivatnt2m64->bios_rom.rom[addr & rivatnt2m64->bios_rom.mask];

    if ((addr >= 0x1800) && (addr <= 0x18ff))
    return rivatnt2m64_pci_read(0,addr & 0xff,p);

    switch(addr) {
    case 0x6013b4: case 0x6013b5:
    case 0x6013d4: case 0x6013d5:
    case 0x6013da:
    case 0x6013c0:
    case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
        return rivatnt2m64_in(addr & 0x3ff,p);
        break;
    }

    return (rivatnt2m64_mmio_read_l(addr & 0xffffff, rivatnt2m64) >> ((addr & 3) << 3)) & 0xff;
}


uint16_t
rivatnt2m64_mmio_read_w(uint32_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    addr &= 0xffffff;

    if ((addr >= 0x300000) && (addr <= 0x30ffff)) return ((uint16_t *) rivatnt2m64->bios_rom.rom)[(addr & rivatnt2m64->bios_rom.mask) >> 1];

    if ((addr >= 0x1800) && (addr <= 0x18ff))
    return (rivatnt2m64_pci_read(0,(addr+0) & 0xff,p) << 0) | (rivatnt2m64_pci_read(0,(addr+1) & 0xff,p) << 8);

    switch(addr) {
    case 0x6013b4: case 0x6013b5:
    case 0x6013d4: case 0x6013d5:
    case 0x6013da:
    case 0x6013c0:
    case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
        return (rivatnt2m64_in((addr+0) & 0x3ff,p) << 0) | (rivatnt2m64_in((addr+1) & 0x3ff,p) << 8);
        break;
    }

   return (rivatnt2m64_mmio_read_l(addr & 0xffffff, rivatnt2m64) >> ((addr & 3) << 3)) & 0xffff;
}


void
rivatnt2m64_mmio_write_l(uint32_t addr, uint32_t val, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;

    addr &= 0xffffff;

    pclog("[RIVA TNT] MMIO write %08x %08x\n", addr, val);

    if ((addr >= 0x1800) && (addr <= 0x18ff)) {
    rivatnt2m64_pci_write(0, addr & 0xff, val & 0xff, p);
    rivatnt2m64_pci_write(0, (addr+1) & 0xff, (val>>8) & 0xff, p);
    rivatnt2m64_pci_write(0, (addr+2) & 0xff, (val>>16) & 0xff, p);
    rivatnt2m64_pci_write(0, (addr+3) & 0xff, (val>>24) & 0xff, p);
    return;
    }

    if((addr >= 0x000000) && (addr <= 0x000fff)) rivatnt2m64_pmc_write(addr, val, rivatnt2m64);
    if((addr >= 0x002000) && (addr <= 0x003fff)) rivatnt2m64_pfifo_write(addr, val, rivatnt2m64);
    if((addr >= 0x009000) && (addr <= 0x009fff)) rivatnt2m64_ptimer_write(addr, val, rivatnt2m64);
    if((addr >= 0x600000) && (addr <= 0x600fff)) rivatnt2m64_pcrtc_write(addr, val, rivatnt2m64);
    if((addr >= 0x680000) && (addr <= 0x680fff)) rivatnt2m64_pramdac_write(addr, val, rivatnt2m64);
    if((addr >= 0x700000) && (addr <= 0x7fffff)) rivatnt2m64->ramin[(addr & 0xfffff) >> 2] = val;

    switch(addr) {
    case 0x6013b4: case 0x6013b5:
    case 0x6013d4: case 0x6013d5:
    case 0x6013da:
    case 0x6013c0:
    case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
        rivatnt2m64_out(addr & 0xfff, val & 0xff, p);
        rivatnt2m64_out((addr+1) & 0xfff, (val>>8) & 0xff, p);
        rivatnt2m64_out((addr+2) & 0xfff, (val>>16) & 0xff, p);
        rivatnt2m64_out((addr+3) & 0xfff, (val>>24) & 0xff, p);
        break;
    }
}


void
rivatnt2m64_mmio_write(uint32_t addr, uint8_t val, void *p)
{
    uint32_t tmp;

    addr &= 0xffffff;

    if ((addr >= 0x1800) && (addr <= 0x18ff)) {
    rivatnt2m64_pci_write(0, addr & 0xff, val & 0xff, p);
    return;
    }

    switch(addr) {
    case 0x6013b4: case 0x6013b5:
    case 0x6013d4: case 0x6013d5:
    case 0x6013da:
    case 0x6013c0:
    case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
        rivatnt2m64_out(addr & 0xfff, val & 0xff, p);
        return;
    }

    tmp = rivatnt2m64_mmio_read_l(addr,p);
    tmp &= ~(0xff << ((addr & 3) << 3));
    tmp |= val << ((addr & 3) << 3);
    rivatnt2m64_mmio_write_l(addr, tmp, p);
    if ((addr >= 0x1800) && (addr <= 0x18ff)) rivatnt2m64_pci_write(0, addr & 0xff, val, p);
}


void
rivatnt2m64_mmio_write_w(uint32_t addr, uint16_t val, void *p)
{
    uint32_t tmp;

    addr &= 0xffffff;

    if ((addr >= 0x1800) && (addr <= 0x18ff)) {
    rivatnt2m64_pci_write(0, addr & 0xff, val & 0xff, p);
    rivatnt2m64_pci_write(0, (addr+1) & 0xff, (val>>8) & 0xff, p);
    return;
    }

    switch(addr) {
    case 0x6013b4: case 0x6013b5:
    case 0x6013d4: case 0x6013d5:
    case 0x6013da:
    case 0x6013c0:
    case 0x0c03c2: case 0x0c03c3: case 0x0c03c4: case 0x0c03c5: case 0x0c03cc:
        rivatnt2m64_out(addr & 0xfff, val & 0xff, p);
        rivatnt2m64_out(addr & 0xfff, val >> 8, p);
        return;
    }

    tmp = rivatnt2m64_mmio_read_l(addr,p);
    tmp &= ~(0xffff << ((addr & 3) << 3));
    tmp |= val << ((addr & 3) << 3);
    rivatnt2m64_mmio_write_l(addr, tmp, p);
}

uint8_t
rivatnt2m64_rma_in(uint16_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    svga_t *svga = &rivatnt2m64->svga;
    uint8_t ret = 0;

    addr &= 0xff;

    pclog("RIVA TNT RMA read %04X %04X:%08X\n", addr, CS, cpu_state.pc);

    switch(addr) {
    case 0x00:
        ret = 0x65;
        break;
    case 0x01:
        ret = 0xd0;
        break;
    case 0x02:
        ret = 0x16;
        break;
    case 0x03:
        ret = 0x2b;
        break;
    case 0x08:
    case 0x09:
    case 0x0a:
    case 0x0b:
        if (rivatnt2m64->rma.rma_dst_addr < 0x1000000)
            ret = rivatnt2m64_mmio_read((rivatnt2m64->rma.rma_dst_addr + (addr & 3)) & 0xffffff, rivatnt2m64);
        else
            ret = svga_read_linear((rivatnt2m64->rma.rma_dst_addr - 0x1000000) & 0xffffff, svga);
        break;
    }

    return ret;
}


void
rivatnt2m64_rma_out(uint16_t addr, uint8_t val, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    svga_t* svga = &rivatnt2m64->svga;

    addr &= 0xff;

    pclog("RIVA TNT RMA write %04X %02X %04X:%08X\n", addr, val, CS, cpu_state.pc);

    switch(addr) {
    case 0x04:
        rivatnt2m64->rma.rma_dst_addr &= ~0xff;
        rivatnt2m64->rma.rma_dst_addr |= val;
        break;
    case 0x05:
        rivatnt2m64->rma.rma_dst_addr &= ~0xff00;
        rivatnt2m64->rma.rma_dst_addr |= (val << 8);
        break;
    case 0x06:
        rivatnt2m64->rma.rma_dst_addr &= ~0xff0000;
        rivatnt2m64->rma.rma_dst_addr |= (val << 16);
        break;
    case 0x07:
        rivatnt2m64->rma.rma_dst_addr &= ~0xff000000;
        rivatnt2m64->rma.rma_dst_addr |= (val << 24);
        break;
    case 0x08:
    case 0x0c:
    case 0x10:
    case 0x14:
        rivatnt2m64->rma.rma_data &= ~0xff;
        rivatnt2m64->rma.rma_data |= val;
        break;
    case 0x09:
    case 0x0d:
    case 0x11:
    case 0x15:
        rivatnt2m64->rma.rma_data &= ~0xff00;
        rivatnt2m64->rma.rma_data |= (val << 8);
        break;
    case 0x0a:
    case 0x0e:
    case 0x12:
    case 0x16:
        rivatnt2m64->rma.rma_data &= ~0xff0000;
        rivatnt2m64->rma.rma_data |= (val << 16);
        break;
    case 0x0b:
    case 0x0f:
    case 0x13:
    case 0x17:
        rivatnt2m64->rma.rma_data &= ~0xff000000;
        rivatnt2m64->rma.rma_data |= (val << 24);
        if (rivatnt2m64->rma.rma_dst_addr < 0x1000000)
            rivatnt2m64_mmio_write_l(rivatnt2m64->rma.rma_dst_addr & 0xffffff, rivatnt2m64->rma.rma_data, rivatnt2m64);
        else
            svga_writel_linear((rivatnt2m64->rma.rma_dst_addr - 0x1000000) & 0xffffff, rivatnt2m64->rma.rma_data, svga);
        break;
    }

    if (addr & 0x10)
    rivatnt2m64->rma.rma_dst_addr+=4;
}


static void
rivatnt2m64_out(uint16_t addr, uint8_t val, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    svga_t *svga = &rivatnt2m64->svga;
    uint8_t old;

    if ((addr >= 0x3d0) && (addr <= 0x3d3)) {
    rivatnt2m64->rma.rma_access_reg[addr & 3] = val;
    if(!(rivatnt2m64->rma.rma_mode & 1))
        return;
    rivatnt2m64_rma_out(((rivatnt2m64->rma.rma_mode & 0xe) << 1) + (addr & 3), rivatnt2m64->rma.rma_access_reg[addr & 3], rivatnt2m64);
    }

    if (((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0) && !(svga->miscout & 1))
    addr ^= 0x60;

    switch (addr) {
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
            switch(svga->crtcreg) {
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
                case 0x19: case 0x1a: case 0x25: case 0x28:
                case 0x2d:
                    svga_recalctimings(svga);
                    break;
                case 0x38:
                    rivatnt2m64->rma.rma_mode = val & 0xf;
                    break;
                case 0x3f:
                    i2c_gpio_set(rivatnt2m64->i2c, !!(val & 0x20), !!(val & 0x10));
                    break;
            }
        //if (svga->crtcreg > 0x18 && svga->crtcreg != 0x38 && svga->crtcreg != 0x1d && svga->crtcreg != 0x1e && svga->crtcreg != 0x19 && svga->crtcreg != 0x1a && svga->crtcreg != 0x25 && svga->crtcreg != 0x28)
             //pclog("RIVA TNT Extended CRTC write %02X %02x\n", svga->crtcreg, val);
        if (old != val) {
            if ((svga->crtcreg < 0xe) || (svga->crtcreg > 0x10)) {
                svga->fullchange = changeframecount;
                svga_recalctimings(svga);
            }
        }
        break;
    }

    svga_out(addr, val, svga);
}


static uint8_t
rivatnt2m64_in(uint16_t addr, void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    svga_t *svga = &rivatnt2m64->svga;
    uint8_t temp;

    if ((addr >= 0x3d0) && (addr <= 0x3d3)) {
    if (!(rivatnt2m64->rma.rma_mode & 1))
        return 0x00;
    return rivatnt2m64_rma_in(((rivatnt2m64->rma.rma_mode & 0xe) << 1) + (addr & 3), rivatnt2m64);
    }

    if (((addr&0xFFF0) == 0x3D0 || (addr&0xFFF0) == 0x3B0) && !(svga->miscout&1)) addr ^= 0x60;

    switch (addr) {
    case 0x3D4:
        temp = svga->crtcreg;
        break;
    case 0x3D5:
        switch(svga->crtcreg) {
            case 0x3e:
                    /* DDC status register */
                temp = (i2c_gpio_get_sda(rivatnt2m64->i2c) << 3) | (i2c_gpio_get_scl(rivatnt2m64->i2c) << 2);
                break;
            default:
                temp = svga->crtc[svga->crtcreg];
                break;
        }
        break;
    default:
        temp = svga_in(addr, svga);
        break;
    }

    return temp;
}

static void
rivatnt2m64_recalctimings(svga_t *svga)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)svga->priv;

    if (svga->crtc[0x25] & 0x01) svga->vtotal      += 0x400;
    if (svga->crtc[0x25] & 0x02) svga->dispend     += 0x400;
    if (svga->crtc[0x25] & 0x04) svga->vblankstart += 0x400;
    if (svga->crtc[0x25] & 0x08) svga->vsyncstart  += 0x400;
    if (svga->crtc[0x25] & 0x10) svga->htotal      += 0x100;
    if (svga->crtc[0x2d] & 0x01) svga->hdisp       += 0x100;  

    switch(svga->crtc[0x28] & 3) {
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

    double freq = 13500000.0;
    int m_m = rivatnt2m64->pramdac.mpll & 0xff;
    int m_n = (rivatnt2m64->pramdac.mpll >> 8) & 0xff;
    int m_p = (rivatnt2m64->pramdac.mpll >> 16) & 7;

    if(m_n == 0) m_n = 1;
    if(m_m == 0) m_m = 1;

    freq = (freq * m_n) / (m_m << m_p);
    rivatnt2m64->mtime = 10000000.0 / freq; //Multiply period by 10 to work around timer system limitations.
    timer_on_auto(&rivatnt2m64->mtimer, rivatnt2m64->mtime);

    freq = 13500000;
    int nv_m = rivatnt2m64->pramdac.nvpll & 0xff;
    int nv_n = (rivatnt2m64->pramdac.nvpll >> 8) & 0xff;
    int nv_p = (rivatnt2m64->pramdac.nvpll >> 16) & 7;

    if(nv_n == 0) nv_n = 1;
    if(nv_m == 0) nv_m = 1;

    freq = (freq * nv_n) / (nv_m << nv_p);
    rivatnt2m64->nvtime = 10000000.0 / freq; //Multiply period by 10 to work around timer system limitations.
    timer_on_auto(&rivatnt2m64->nvtimer, rivatnt2m64->nvtime);

    freq = 13500000;
    int v_m = rivatnt2m64->pramdac.vpll & 0xff;
    int v_n = (rivatnt2m64->pramdac.vpll >> 8) & 0xff;
    int v_p = (rivatnt2m64->pramdac.vpll >> 16) & 7;

    if(v_n == 0) v_n = 1;
    if(v_m == 0) v_m = 1;

    freq = (freq * v_n) / (v_m << v_p);
    svga->clock = (cpuclock * (double)(1ull << 32)) / freq;
}

void
rivatnt2m64_vblank_start(svga_t *svga)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)svga->priv;

    rivatnt2m64->pcrtc.intr |= 1;

    rivatnt2m64_pmc_recompute_intr(rivatnt2m64);
}

static void
*rivatnt2m64_init(const device_t *info)
{
    rivatnt2m64_t *rivatnt2m64 = malloc(sizeof(rivatnt2m64_t));
    svga_t *svga;
    char *romfn = BIOS_RIVATNT2M64_PATH;
    memset(rivatnt2m64, 0, sizeof(rivatnt2m64_t));
    svga = &rivatnt2m64->svga;

    rivatnt2m64->vram_size = device_get_config_int("memory") << 20;
    rivatnt2m64->vram_mask = rivatnt2m64->vram_size - 1;

    svga_init(info, &rivatnt2m64->svga, rivatnt2m64, rivatnt2m64->vram_size,
          rivatnt2m64_recalctimings, rivatnt2m64_in, rivatnt2m64_out,
          NULL, NULL);

    svga->decode_mask = 0x3fffff;
    svga->force_old_addr = 1;

    rom_init(&rivatnt2m64->bios_rom, romfn, 0xc0000, 0x10000, 0xffff, 0, MEM_MAPPING_EXTERNAL);
    mem_mapping_disable(&rivatnt2m64->bios_rom.mapping);

    mem_mapping_add(&rivatnt2m64->mmio_mapping, 0, 0, rivatnt2m64_mmio_read, rivatnt2m64_mmio_read_w, rivatnt2m64_mmio_read_l, rivatnt2m64_mmio_write, rivatnt2m64_mmio_write_w, rivatnt2m64_mmio_write_l,  NULL, MEM_MAPPING_EXTERNAL, rivatnt2m64);
    mem_mapping_disable(&rivatnt2m64->mmio_mapping);
    mem_mapping_add(&rivatnt2m64->linear_mapping, 0, 0, svga_read_linear, svga_readw_linear, svga_readl_linear, svga_write_linear, svga_writew_linear, svga_writel_linear,  NULL, MEM_MAPPING_EXTERNAL, &rivatnt2m64->svga);
    mem_mapping_disable(&rivatnt2m64->linear_mapping);

    svga->vblank_start = rivatnt2m64_vblank_start;

    pci_add_card(PCI_ADD_AGP, rivatnt2m64_pci_read, rivatnt2m64_pci_write, rivatnt2m64, &rivatnt2m64->pci_slot);

    rivatnt2m64->pci_regs[0x04] = 0x07;
    rivatnt2m64->pci_regs[0x05] = 0x00;
    rivatnt2m64->pci_regs[0x07] = 0x02;

    rivatnt2m64->pci_regs[0x30] = 0x00;
    rivatnt2m64->pci_regs[0x32] = 0x0c;
    rivatnt2m64->pci_regs[0x33] = 0x00;

    rivatnt2m64->pmc.intr_en = 1;

    //Default values for the RAMDAC PLLs
    rivatnt2m64->pramdac.mpll = 0x03c20d;
    rivatnt2m64->pramdac.nvpll = 0x03c20d;
    rivatnt2m64->pramdac.vpll = 0x03c20d;

    timer_add(&rivatnt2m64->nvtimer, rivatnt2m64_nvclk_poll, rivatnt2m64, 0);
    timer_add(&rivatnt2m64->mtimer, rivatnt2m64_mclk_poll, rivatnt2m64, 0);

    video_inform(VIDEO_FLAG_TYPE_SPECIAL, &timing_rivatnt2m64);

    rivatnt2m64->i2c = i2c_gpio_init("ddc_rivatnt2m64");
    rivatnt2m64->ddc = ddc_init(i2c_gpio_get_bus(rivatnt2m64->i2c));

    return rivatnt2m64;
}


static int
rivatnt2m64_available(void)
{
    return rom_present(BIOS_RIVATNT2M64_PATH);
}


void
rivatnt2m64_close(void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    
    svga_close(&rivatnt2m64->svga);
    
    free(rivatnt2m64);
}


void
rivatnt2m64_speed_changed(void *p)
{
    rivatnt2m64_t *rivatnt2m64 = (rivatnt2m64_t *)p;
    
    svga_recalctimings(&rivatnt2m64->svga);
}


void
rivatnt2m64_force_redraw(void *p)
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