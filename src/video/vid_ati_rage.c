/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          ATI Rage series emulation.
 *
 *
 *
 * Authors: aquaboxs
 *
 *
 *          Copyright 2024 aquaboxs.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#include <stdatomic.h>
#include <stdbool.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/timer.h>
#include <86box/pci.h>
#include <86box/rom.h>
#include <86box/plat.h>
#include <86box/thread.h>
#include <86box/video.h>
#include <86box/i2c.h>
#include <86box/vid_ddc.h>
#include <86box/vid_svga.h>
#include <86box/vid_svga_render.h>
#include <86box/bswap.h>

static video_timings_t timing_atirage     = { .type = VIDEO_AGP, .write_b = 2, .write_w = 2, .write_l = 1, .read_b = 20, .read_w = 20, .read_l = 21 };

typedef struct atirage_t
{
    svga_t  svga;
    uint8_t pci_regs[256];

    mem_mapping_t linear_mapping;
    mem_mapping_t reg_mapping;
    mem_mapping_t reg1_mapping;
    rom_t bios_rom;
    uint32_t linear_base, reg_base, reg1_base;
    uint32_t io_base;
    uint32_t vram_size, vram_mask;

	uint32_t subsys;

    uint8_t slot;

    uint8_t regs[256];
    int     index;

    int type;
    int pci;

    uint8_t irq_state;

    int bank_r[2];
    int bank_w[2];

    uint32_t config_cntl;

    uint32_t context_load_cntl;
    uint32_t context_mask;

    uint32_t crtc_gen_cntl;
    uint8_t  crtc_int_cntl;
    uint32_t crtc_h_sync_strt_wid;
    uint32_t crtc_h_total_disp;
    uint32_t crtc_v_sync_strt_wid;
    uint32_t crtc_v_total_disp;
    uint32_t crtc_off_pitch;

    uint32_t clock_cntl;

    uint32_t clr_cmp_clr;
    uint32_t clr_cmp_cntl;
    uint32_t clr_cmp_mask;

    uint32_t cur_horz_vert_off;
    uint32_t cur_horz_vert_posn;
    uint32_t cur_offset;

    uint32_t dac_cntl;

    uint32_t dp_bkgd_clr;
    uint32_t dp_frgd_clr;
    uint32_t dp_mix;
    uint32_t dp_pix_width;
    uint32_t dp_src;

    uint32_t dst_bres_lnth;
    uint32_t dst_bres_dec;
    uint32_t dst_bres_err;
    uint32_t dst_bres_inc;

    uint32_t dst_cntl;
    uint32_t dst_height_width;
    uint32_t dst_off_pitch;
    uint32_t dst_y_x;

    uint32_t gen_test_cntl;

    uint32_t gui_traj_cntl;

    uint32_t host_cntl;

    uint32_t mem_cntl;

    uint32_t ovr_clr;
    uint32_t ovr_wid_left_right;
    uint32_t ovr_wid_top_bottom;

    uint32_t pat_cntl;
    uint32_t pat_reg0;
    uint32_t pat_reg1;

    uint32_t sc_left_right;
    uint32_t sc_top_bottom;

    uint32_t scratch_reg0;
    uint32_t scratch_reg1;

    uint32_t src_cntl;
    uint32_t src_off_pitch;
    uint32_t src_y_x;
    uint32_t src_y_x_start;
    uint32_t src_height1_width1;
    uint32_t src_height2_width2;

    uint32_t write_mask;
    uint32_t chain_mask;

    struct {
        int op;

        int      dst_x;
        int      dst_y;
        int      dst_x_start;
        int      dst_y_start;
        int      src_x;
        int      src_y;
        int      src_x_start;
        int      src_y_start;
        int      xinc;
        int      yinc;
        int      x_count;
        int      y_count;
        int      xx_count;
        int      src_x_count;
        int      src_y_count;
        int      src_width1;
        int      src_height1;
        int      src_width2;
        int      src_height2;
        uint32_t src_offset;
        uint32_t src_pitch;
        uint32_t dst_offset;
        uint32_t dst_pitch;
        int      mix_bg;
        int      mix_fg;
        int      source_bg;
        int      source_fg;
        int      source_mix;
        int      source_host;
        int      dst_width;
        int      dst_height;
        int      busy;
        int      pattern[8][8];
        uint8_t  pattern_clr4x2[2][4];
        uint8_t  pattern_clr8x1[8];
        int      sc_left;
        int      sc_right;
        int      sc_top;
        int      sc_bottom;
        int      dst_pix_width;
        int      src_pix_width;
        int      host_pix_width;
        int      dst_size;
        int      src_size;
        int      host_size;
        int      temp_cnt;

        uint32_t dp_bkgd_clr;
        uint32_t dp_frgd_clr;
        uint32_t write_mask;

        uint32_t clr_cmp_clr;
        uint32_t clr_cmp_mask;
        int      clr_cmp_fn;
        int      clr_cmp_src;

        int err;
        int poly_draw;
    } accel;

    int      blitter_busy;
    uint64_t blitter_time;
    uint64_t status_time;

    uint16_t pci_id;
    uint32_t config_chip_id;
    uint32_t block_decoded_io;
    int      use_block_decoded_io;

    int     pll_addr;
    uint8_t pll_regs[16];
    double  pll_freq[4];

    uint32_t config_stat0;

    uint32_t cur_clr0;
    uint32_t cur_clr1;

    uint32_t overlay_dat[1024];
    uint32_t overlay_graphics_key_clr;
    uint32_t overlay_graphics_key_msk;
    uint32_t overlay_video_key_clr;
    uint32_t overlay_video_key_msk;
    uint32_t overlay_key_cntl;
    uint32_t overlay_scale_inc;
    uint32_t overlay_scale_cntl;
    uint32_t overlay_y_x_start;
    uint32_t overlay_y_x_end;

    uint32_t scaler_height_width;
    int      scaler_format;
    int      scaler_update;

    uint32_t buf_offset[2];
    uint32_t buf_pitch[2];

    int overlay_v_acc;

    uint8_t thread_run;
} atirage_t;

static inline bool bit_changed(uint64_t old_val, uint64_t new_val, int bit_num) {
    return (old_val ^ new_val) & (1ULL << bit_num);
}

static inline bool bit_set(const uint64_t val, const int bit_num) {
    return !!(val & (1ULL << bit_num));
}


/** atirage register offsets. */
enum {
    ATI_CRTC_H_TOTAL_DISP     = 0x000, // 0x0000
        ATI_CRTC_H_TOTAL    =  0, ATI_CRTC_H_TOTAL_size = 9,
        ATI_CRTC_H_DISP     = 16, ATI_CRTC_H_DISP_size = 8,

    ATI_CRTC_H_SYNC_STRT_WID  = 0x001, // 0x0004
        ATI_CRTC_H_SYNC_STRT    =  0, ATI_CRTC_H_SYNC_STRT_size = 8,
        ATI_CRTC_H_SYNC_DLY     =  8, ATI_CRTC_H_SYNC_DLY_size = 3,
        ATI_CRTC_H_SYNC_STRT_HI = 12,
        ATI_CRTC_H_SYNC_WID     = 16, ATI_CRTC_H_SYNC_WID_size = 5,
        ATI_CRTC_H_SYNC_POL     = 21,

    ATI_CRTC_V_TOTAL_DISP     = 0x002, // 0x0008
        ATI_CRTC_V_TOTAL    =  0, ATI_CRTC_V_TOTAL_size = 11,
        ATI_CRTC_V_DISP     = 16, ATI_CRTC_V_DISP_size = 11,

    ATI_CRTC_V_SYNC_STRT_WID  = 0x003, // 0x000C
        ATI_CRTC_V_SYNC_STRT    =  0, ATI_CRTC_V_SYNC_STRT_size = 11,
        ATI_CRTC_V_SYNC_WID     = 16, ATI_CRTC_V_SYNC_WID_size = 5,
        ATI_CRTC_V_SYNC_POL     = 21,

    ATI_CRTC_VLINE_CRNT_VLINE = 0x004, // 0x0010
        ATI_CRTC_VLINE      =  0, ATI_CRTC_VLINE_size      = 11,
        ATI_CRTC_CRNT_VLINE = 16, ATI_CRTC_CRNT_VLINE_size = 11,

    ATI_CRTC_OFF_PITCH        = 0x005, // 0x0014
        ATI_CRTC_OFFSET         =  0, ATI_CRTC_OFFSET_size = 20,
        ATI_CRTC_OFFSET_LOCK    = 20,
        ATI_CRTC_PITCH          = 22, ATI_CRTC_PITCH_size = 10,

    ATI_CRTC_INT_CNTL         = 0x006, // 0x0018
        ATI_CRTC_VBLANK             =  0,
        ATI_CRTC_VBLANK_INT_EN      =  1,
        ATI_CRTC_VBLANK_INT         =  2,
        ATI_CRTC_VBLANK_INT_AK      =  2,
        ATI_CRTC_VLINE_INT_EN       =  3,
        ATI_CRTC_VLINE_INT          =  4,
        ATI_CRTC_VLINE_INT_AK       =  4,
        ATI_CRTC_VLINE_SYNC         =  5,
        ATI_CRTC_FRAME              =  6,

        ATI_SNAPSHOT_INT_EN         =  7, // not VT,GT
        ATI_SNAPSHOT_INT            =  8, // not VT,GT
        ATI_SNAPSHOT_INT_AK         =  8, // not VT,GT
        ATI_I2C_INT_EN              =  9, // not VT,GT
        ATI_I2C_INT                 = 10, // not VT,GT
        ATI_I2C_INT_AK              = 10, // not VT,GT
        ATI_CRTC2_VBLANK            = 11, // not VT,GT
        ATI_CRTC2_VBLANK_INT_EN     = 12, // not VT,GT
        ATI_CRTC2_VBLANK_INT        = 13, // not VT,GT
        ATI_CRTC2_VBLANK_INT_AK     = 13, // not VT,GT
        ATI_CRTC2_VLINE_INT_EN      = 14, // not VT,GT
        ATI_CRTC2_VLINE_INT         = 15, // not VT,GT
        ATI_CRTC2_VLINE_INT_AK      = 15, // not VT,GT

        ATI_VIDEOIN_EVEN_INT_EN     = 16, // VT,GT
        ATI_VIDEOIN_EVEN_INT        = 17, // VT,GT
        ATI_VIDEOIN_EVEN_INT_AK     = 17, // VT,GT
        ATI_VIDEOIN_ODD_INT_EN      = 18, // VT,GT
        ATI_VIDEOIN_ODD_INT         = 19, // VT,GT
        ATI_VIDEOIN_ODD_INT_AK      = 19, // VT,GT

        ATI_CUPBUF0_INT_EN          = 16, // not VT,GT
        ATI_CUPBUF0_INT             = 17, // not VT,GT
        ATI_CUPBUF0_INT_AK          = 17, // not VT,GT
        ATI_CUPBUF1_INT_EN          = 18, // not VT,GT
        ATI_CUPBUF1_INT             = 19, // not VT,GT
        ATI_CUPBUF1_INT_AK          = 19, // not VT,GT

        ATI_OVERLAY_EOF_INT_EN      = 20,
        ATI_OVERLAY_EOF_INT         = 21,
        ATI_OVERLAY_EOF_INT_AK      = 21,

        ATI_VMC_EC_INT_EN           = 22, // VT,GT
        ATI_VMC_EC_INT              = 23, // VT,GT
        ATI_VMC_EC_INT_AK           = 23, // VT,GT

        ATI_ONESHOT_CAP_INT_EN      = 22, // not VT,GT
        ATI_ONESHOT_CAP_INT         = 23, // not VT,GT
        ATI_ONESHOT_CAP_INT_AK      = 23, // not VT,GT

        ATI_BUSMASTER_EOL_INT_EN    = 24, // not VT,GT
        ATI_BUSMASTER_EOL_INT       = 25, // not VT,GT
        ATI_BUSMASTER_EOL_INT_AK    = 25, // not VT,GT
        ATI_GP_INT_EN               = 26, // not VT,GT
        ATI_GP_INT                  = 27, // not VT,GT
        ATI_GP_INT_AK               = 27, // not VT,GT
        ATI_CRTC2_VLINE_SYNC        = 28, // not VT,GT
        ATI_SNAPSHOT2_INT_EN        = 29, // not VT,GT
        ATI_SNAPSHOT2_INT           = 30, // not VT,GT
        ATI_SNAPSHOT2_INT_AK        = 30, // not VT,GT
        ATI_VBLANK_BIT2_INT         = 31, // not VT,GT
        ATI_VBLANK_BIT2_INT_AK      = 31, // not VT,GT

    ATI_CRTC_GEN_CNTL         = 0x007, // 0x001C
        ATI_CRTC_DBL_SCAN_EN        =  0,
        ATI_CRTC_INTERLACE_EN       =  1,
        ATI_CRTC_HSYNC_DIS          =  2,
        ATI_CRTC_VSYNC_DIS          =  3,
        ATI_CRTC_CSYNC_EN           =  4,
        ATI_CRTC2_DBL_SCAN_EN       =  5, // not VT,GT
        ATI_CRTC_DISPLAY_DIS        =  6,
        ATI_CRTC_VGA_XOVERSCAN      =  7,
        ATI_CRTC_PIX_WIDTH          =  8, ATI_CRTC_PIX_WIDTH_size = 3,
        ATI_CRTC_BYTE_PIX_ORDER     = 11,

        ATI_CRTC_VSYNC_INT_EN       = 12,                               // not VT,GT
        ATI_CRTC_VSYNC_INT          = 13,                               // not VT,GT
        ATI_CRTC_VSYNC_INT_AK       = 13,                               // not VT,GT
        ATI_CRTC2_VSYNC_INT_EN      = 14,                               // not VT,GT
        ATI_CRTC2_VSYNC_INT         = 15,                               // not VT,GT
        ATI_CRTC2_VSYNC_INT_AK      = 15,                               // not VT,GT
        ATI_HVSYNC_IO_DRIVE         = 16,                               // not VT,GT
        ATI_CRTC2_PIX_WIDTH         = 17, ATI_CRTC2_PIX_WIDTH_size = 3, // not VT,GT

        ATI_CRTC_FIFO_OVERFILL      = 14, ATI_CRTC_FIFO_OVERFILL_size = 2, // VT,GT
        ATI_CRTC_FIFO_LWM           = 16, ATI_CRTC_FIFO_LWM_size = 4,      // VT,GT

        ATI_VGA_128KAP_PAGING       = 20,

        ATI_CRTC_DISPREQ_ONLY       = 21, // not VT,GT

        ATI_CRTC2_ENABLE            = 21, // not VT,GT

        ATI_CRTC_LOCK_REGS          = 22,
        ATI_CRTC_SYNC_TRISTATE      = 23,
        ATI_CRTC_EXT_DISP_EN        = 24,
        ATI_CRTC_ENABLE             = 25,
        ATI_CRTC_DISP_REQ_ENB       = 26,
        ATI_VGA_ATI_LINEAR          = 27,
        ATI_CRTC_VSYNC_FALL_EDGE    = 28,
        ATI_VGA_TEXT_132            = 29,
        ATI_VGA_XCRT_CNT_EN         = 30,
        ATI_VGA_CUR_B_TEST          = 31,

    ATI_DSP_CONFIG            = 0x008, // 0x0020
        ATI_DSP_XCLKS_PER_QW    =  0, ATI_DSP_XCLKS_PER_QW_size = 14,   // not VT,GT
        ATI_DSP_FLUSH_WB        = 15,                                   // not VT,GT
        ATI_DSP_LOOP_LATENCY    = 16, ATI_DSP_LOOP_LATENCY_size =  4,   // not VT,GT
        ATI_DSP_PRECISION       = 20, ATI_DSP_PRECISION_size    =  3,   // not VT,GT

    ATI_DSP_ON_OFF            = 0x009, // 0x0024
        ATI_DSP_OFF =  0, ATI_DSP_OFF_size = 11,    // not VT,GT
        ATI_DSP_ON  = 16, ATI_DSP_ON_size = 11,     // not VT,GT

    ATI_TIMER_CFG             = 0x00A, // 0x0028
    ATI_MEM_BUF_CNTL          = 0x00B, // 0x002C
    ATI_MEM_ADDR_CFG          = 0x00D, // 0x0034
    ATI_CRT_TRAP              = 0x00E, // 0x0038
    ATI_I2C_CNTL_0            = 0x00F, // 0x003C
    ATI_OVR_CLR               = 0x010, // 0x0040
    ATI_OVR_WID_LEFT_RIGHT    = 0x011, // 0x0044
    ATI_OVR_WID_TOP_BOTTOM    = 0x012, // 0x0048
    ATI_VGA_DSP_CFG           = 0x013, // 0x004C
    ATI_VGA_DSP_ON_OFF        = 0x014, // 0x0050
    ATI_DSP2_CONFIG           = 0x015, // 0x0054 <-- LT specific
    ATI_DSP2_ON_OFF           = 0x016, // 0x0058 <-- LT specific
    ATI_CRTC2_OFF_PITCH       = 0x017, // 0x005C <-- LT specific
    ATI_CUR_CLR0              = 0x018, // 0x0060
    ATI_CUR_CLR1              = 0x019, // 0x0064
    ATI_CUR_OFFSET            = 0x01A, // 0x0068
        ATI_CUR_OFFSET_size = 20,

    ATI_CUR_HORZ_VERT_POSN    = 0x01B, // 0x006C
        ATI_CUR_HORZ_POSN   =  0, ATI_CUR_HORZ_POSN_size = 11,
        ATI_CUR_VERT_POSN   = 16, ATI_CUR_VERT_POSN_size = 11,

    ATI_CUR_HORZ_VERT_OFF     = 0x01C, // 0x0070
        ATI_CUR_HORZ_OFF    =  0, ATI_CUR_HORZ_OFF_size = 6,
        ATI_CUR_VERT_OFF    = 16, ATI_CUR_VERT_OFF_size = 6,

    ATI_GP_IO                 = 0x01E, // 0x0078

    ATI_HW_DEBUG              = 0x01F, // 0x007C
    ATI_SCRATCH_REG0          = 0x020, // 0x0080
    ATI_SCRATCH_REG1          = 0x021, // 0x0084
    ATI_SCRATCH_REG2          = 0x022, // 0x0088
    ATI_SCRATCH_REG3          = 0x023, // 0x008C

    ATI_CLOCK_CNTL            = 0x024, // 0x0090
        ATI_CLOCK_SEL           =  0, ATI_CLOCK_SEL_size = 2,          // CT
        ATI_CLOCK_SEL_INTERNAL  =  0, ATI_CLOCK_SEL_INTERNAL_size = 2, // CT
        ATI_CLOCK_SEL_EXTERNAL  =  2, ATI_CLOCK_SEL_EXTERNAL_size = 2, // CT
        ATI_CLOCK_BIT           =  2,                                  // GX ; For ICS2595
        ATI_CLOCK_PULSE         =  3,                                  // GX ; For ICS2595
        ATI_CLOCK_DIV           =  4, ATI_CLOCK_DIV_size          = 2, // CT ; 0=DIV1, 1=DIV2, 2=DIV4
        ATI_CLOCK_STROBE        =  6,                                  // CT/GX
        ATI_CLOCK_DATA          =  7,                                  // GX
        ATI_PLL_WR_EN           =  9,                                  // CT    ; internal PLL
        ATI_PLL_ADDR            = 10,                                  // CT    ; internal PLL
            //ATI_PLL_ADDR_size  = 4,                                  // VT/GT ; internal PLL
            ATI_PLL_ADDR_size  = 6,                                    // CT    ; internal PLL
        ATI_PLL_DATA            = 16, ATI_PLL_DATA_size  = 8,          // CT    ; internal PLL

    ATI_CONFIG_STAT1          = 0x025, // 0x0094
    ATI_CONFIG_STAT2          = 0x026, // 0x0098
    ATI_BUS_CNTL              = 0x028, // 0x00A0
        ATI_BUS_WS                  =  0, ATI_BUS_WS_size = 4,          // VT/GT
        ATI_BUS_ROM_WS              =  4, ATI_BUS_ROM_WS_size = 4,      // VT/GT
        ATI_BUS_ROM_PAGE            =  8, ATI_BUS_ROM_PAGE_size = 4,    // VT/GT
        ATI_BUS_ROM_DIS             = 12,                               // VT/GT
        ATI_BUS_DAC_SNOOP_EN        = 14,                               // VT/GT
        ATI_BUS_PCI_RETRY_EN        = 15,                               // VT/GT
        ATI_BUS_FIFO_WS             = 16, ATI_BUS_FIFO_WS_size = 4,     // VT/GT
        ATI_BUS_FIFO_ERR_INT_EN     = 20,                               // VT/GT
        ATI_BUS_FIFO_ERR_INT        = 21,                               // VT/GT
        ATI_BUS_FIFO_ERR_AK         = 21,                               // VT/GT
        ATI_BUS_HOST_ERR_INT_EN     = 22,                               // VT/GT
        ATI_BUS_HOST_ERR_INT        = 23,                               // VT/GT
        ATI_BUS_HOST_ERR_AK         = 23,                               // VT/GT
        ATI_BUS_EXT_REG_EN          = 27,                               // VT, not GT
        ATI_BUS_PCI_MEMW_WS         = 28,                               // VT/GT
        ATI_BUS_BURST               = 29,                               // VT/GT
        ATI_BUS_RDY_READ_DLY        = 30, ATI_BUS_PCI_MEMW_WS_size = 2, // VT/GT

        ATI_BUS_DBL_RESYNC          =  0,                               // not VT/GT
        ATI_BUS_MSTR_RESET          =  1,                               // not VT/GT
        ATI_BUS_FLUSH_BUF           =  2,                               // not VT/GT
        ATI_BUS_STOP_REQ_DIS        =  3,                               // not VT/GT
        ATI_BUS_APER_REG_DIS        =  4,                               // not VT/GT
        ATI_BUS_EXTRA_PIPE_DIS      =  5,                               // not VT/GT
        ATI_BUS_MASTER_DIS          =  6,                               // not VT/GT
        ATI_ROM_WRT_EN              =  7,                               // not VT/GT
        ATI_MINOR_REV_ID            =  8, ATI_MINOR_REV_ID_size = 3,    // not VT/GT
        ATI_BUS_PCI_READ_RETRY_EN   = 13,                               // not VT/GT
        ATI_BUS_PCI_WRT_RETRY_EN    = 15,                               // not VT/GT
        ATI_BUS_RETRY_WS            = 16, ATI_BUS_RETRY_WS_size = 4,    // not VT/GT
        ATI_BUS_MSTR_RD_MULT        = 20,                               // not VT/GT
        ATI_BUS_MSTR_RD_LINE        = 21,                               // not VT/GT
        ATI_BUS_SUSPEND             = 22,                               // not VT/GT
        ATI_LAT16X                  = 23,                               // not VT/GT
        ATI_BUS_RD_DISCARD_EN       = 24,                               // not VT/GT
        ATI_BUS_RD_ABORT_EN         = 25,                               // not VT/GT
        ATI_BUS_MSTR_WS             = 26,                               // not VT/GT
        //ATI_BUS_EXT_REG_EN        = 27,                               // VT, not GT
        ATI_BUS_MSTR_DISCONNECT_EN  = 28,                               // not VT/GT
        ATI_BUS_WRT_BURST           = 29,                               // not VT/GT
        ATI_BUS_READ_BURST          = 30,                               // not VT/GT
        //ATI_BUS_RDY_READ_DLY      = 31,                               // not VT/GT

    ATI_EXT_MEM_CNTL          = 0x02B, // 0x00AC
        ATI_MEM_TBWC            =  0,                                   // not VT/GT
        ATI_MEM_SDRAM_RESET_ext =  1,                                   // not VT/GT
        ATI_MEM_CYC_TEST        =  2, ATI_MEM_CYC_TEST_size = 2,        // not VT/GT
        ATI_MEM_MA_YCLK         =  4,                                   // not VT/GT
        ATI_MEM_CNTL_YCLK       =  5,                                   // not VT/GT
        ATI_MEM_CS_YCLK         =  6,                                   // not VT/GT
        ATI_MEM_EXTND_ERST      =  7,                                   // not VT/GT
        ATI_MEM_ERST_CNTL       =  8, ATI_MEM_ERST_CNTL_size = 2,       // not VT/GT
        ATI_MEM_CAS_LATENCY     = 10, ATI_MEM_CAS_LATENCY_size = 2,     // not VT/GT
        ATI_MEM_SSTL_EN         = 12,                                   // not VT/GT
        ATI_MEM_MD_REC          = 13,                                   // not VT/GT
        ATI_MEM_HCLK_DRIVE      = 14,                                   // not VT/GT
        ATI_MEM_CS_DRIVE        = 15,                                   // not VT/GT
        ATI_MEM_MDA_DRIVE       = 16,                                   // not VT/GT
        ATI_MEM_MDB_DRIVE       = 17,                                   // not VT/GT
        ATI_MEM_MDL_DRIVE       = 18,                                   // not VT/GT
        ATI_MEM_GROUP_CHANGE_EN = 19,                                   // not VT/GT
        ATI_MEM_MA_DRIVE        = 20,                                   // not VT/GT
        ATI_MEM_CNTL_DRIVE      = 22,                                   // not VT/GT
        ATI_MEM_DQM_DRIVE       = 23,                                   // not VT/GT
        ATI_MEM_GCMRS           = 24, ATI_MEM_GCMRS_size = 4,           // not VT/GT
        ATI_MEM_CS_STRAP        = 28,                                   // not VT/GT
        ATI_SDRAM_MEM_CFG       = 29,                                   // not VT/GT
        ATI_MEM_ALL_PAGE_DIS    = 30,                                   // not VT/GT
        ATI_MEM_GROUP_FAULT_EN  = 31,                                   // not VT/GT

    ATI_MEM_CNTL              = 0x02C, // 0x00B0
        ATI_MEM_SIZE            =  0, ATI_MEM_SIZE_size = 3,            // VT/GT    ; 0=512K, 1=1MB, 2=2MB, 3=4MB, 4=6MB
        ATI_MEM_REFRESH         =  3, ATI_MEM_REFRESH_size = 4,         // VT/GT
        ATI_MEM_CYC_LNTH_AUX    =  7, ATI_MEM_CYC_LNTH_AUX_size = 2,    // VT/GT
        ATI_MEM_CYC_LNTH        =  9, ATI_MEM_CYC_LNTH_size = 2,        // VT/GT
        ATI_MEM_REFRESH_RATE    = 11, ATI_MEM_REFRESH_RATE_size = 2,    // VT/GT
        ATI_DLL_RESET           = 13,                                   // VT/GT
        ATI_MEM_ACTV_PRE        = 14, ATI_MEM_ACTV_PRE_size = 2,        // VT/GT
        ATI_DLL_GAIN_CNTL       = 16, ATI_DLL_GAIN_CNTL_size = 2,       // VT/GT
        ATI_MEM_SDRAM_RESET     = 18,                                   // VT/GT
        ATI_MEM_TILE_SELECT     = 19, ATI_MEM_TILE_SELECT_size = 2,     // VT/GT
        ATI_LOW_LATENCY_MODE    = 21,                                   // VT/GT
        ATI_CDE_PULLBACK        = 22,                                   // VT/GT
        ATI_MEM_PIX_WIDTH       = 24, ATI_MEM_PIX_WIDTH_size = 3,       // VT/GT
        ATI_MEM_OE_SELECT       = 27, ATI_MEM_OE_SELECT_size = 2,       // VT/GT

    ATI_MEM_VGA_WP_SEL        = 0x02D, // 0x00B4
    ATI_MEM_VGA_RP_SEL        = 0x02E, // 0x00B8
    ATI_I2C_CNTL_1            = 0x02F, // 0x00BC
    ATI_DAC_REGS              = 0x030, // 0x00C0

    // byte offsets for the built-in DAC registers
    ATI_DAC_W_INDEX           = 0x00C0,
    ATI_DAC_DATA              = 0x00C1,
    ATI_DAC_MASK              = 0x00C2,
    ATI_DAC_R_INDEX           = 0x00C3,

    ATI_DAC_CNTL              = 0x031, // 0x00C4
        ATI_DAC_RANGE_CNTL      =  0, ATI_DAC_RANGE_CNTL_size = 2,  // not VT/GT
        ATI_DAC_BLANKING        =  2,                               //
        ATI_DAC_CMP_DISABLE     =  3,                               //
        ATI_DAC1_CLK_SEL        =  4,                               // not VT/GT
        ATI_PALETTE_ACCESS_CNTL =  5,                               // not VT/GT
        ATI_PALETTE2_SNOOP_EN   =  6,                               // not VT/GT
        ATI_DAC_CMP_OUTPUT      =  7,                               //
        ATI_DAC_8BIT_EN         =  8,                               //
        ATI_DAC_PIX_DLY_2NS     =  9,                               // not VT/GT
        ATI_DAC_DIRECT          = 10, // ATI_DAC_PIX_DLY_4NS        // not VT/GT
        ATI_CRT_SENSE           = 11,                               // not VT/GT
        ATI_CRT_DETECTION_ON    = 12,                               // not VT/GT
        ATI_DAC_VGA_ADR_EN      = 13,                               //
        ATI_DAC_FEA_CON_EN      = 14,                               //
        ATI_DAC_PDWN            = 15,                               //
        ATI_DAC_TYPE            = 16, ATI_DAC_TYPE_size = 3,        //
        ATI_DAC_GIO_STATE       = 24, ATI_DAC_GIO_STATE_size = 3,
        ATI_DAC_GIO_STATE_1     = 24,                               // VT/GT
        ATI_DAC_GIO_STATE_0     = 25,                               // VT/GT
        ATI_DAC_GIO_STATE_4     = 26,                               // VT/GT
        ATI_DAC_GIO_DIR         = 27, ATI_DAC_GIO_DIR_size = 3,
        ATI_DAC_GIO_DIR_1       = 27,                               // VT/GT
        ATI_DAC_GIO_DIR_0       = 28,                               // VT/GT
        ATI_DAC_GIO_DIR_4       = 29,                               // VT/GT
        ATI_DAC_RW_WS           = 31,                               // VT/GT

    ATI_GEN_TEST_CNTL         = 0x034, // 0x00D0
        ATI_GEN_GIO2_DATA_OUT   =  0,                                   // VT/GT
        ATI_GEN_GIO3_DATA_OUT   =  2,                                   // VT/GT
        ATI_GEN_GIO2_DATA_IN    =  3,                                   // VT/GT

        ATI_GEN_GIO2_EN         =  4,                                   // VT/GT
        ATI_GEN_GIO2_WRITE      =  5,                                   // VT/GT

        ATI_GEN_ICON2_ENABLE    =  4,                                   // not VT/GT
        ATI_GEN_CUR2_ENABLE     =  5,                                   // not VT/GT
        ATI_GEN_ICON_ENABLE     =  6,                                   // not VT/GT

        ATI_GEN_CUR_ENABLE      =  7,
        ATI_GEN_GUI_RESETB      =  8,
        ATI_GEN_SOFT_RESET      =  9,                                   // not VT/GT
        ATI_GEN_MEM_TRISTATE    = 10,                                   // not VT/GT
        ATI_GEN_TEST_VECT_MODE  = 12, ATI_GEN_TEST_VECT_MODE_size = 2,
        ATI_GEN_TEST_MODE       = 16, ATI_GEN_TEST_MODE_size = 4,
        ATI_GEN_TEST_CNT_EN     = 20,                                   // VT/GT
        ATI_GEN_CRC_EN          = 21,
        ATI_GEN_DELAY_MUX       = 22, ATI_GEN_DELAY_MUX_size = 2,       // VT A4S ; not VT-A3/A4N or GT
        ATI_GEN_TEST_CNT_VALUE  = 24, ATI_GEN_TEST_CNT_VALUE_size = 6,  // VT/GT

        ATI_GEN_DEBUG_MODE      = 24, ATI_GEN_DEBUG_MODE_size = 8,      // not VT/GT

    ATI_CUSTOM_MACRO_CNTL     = 0x035, // 0x00D4
    ATI_CONFIG_CNTL           = 0x037, // 0x00DC
        ATI_CFG_MEM_AP_SIZE     =  0, ATI_CFG_MEM_AP_SIZE_size  =  2,   // VT/GT
        ATI_CFG_MEM_VGA_AP_EN   =  2,                                   // VT/GT
        ATI_CFG_MEM_AP_LOC      =  4, ATI_CFG_MEM_AP_LOC_size   = 10,   // VT/GT
        ATI_CFG_VGA_DIS         = 19,                                   // VT/GT
        ATI_CDE_WINDOW          = 24, ATI_CDE_WINDOW_size       =  6,   // VT

    ATI_CONFIG_CHIP_ID        = 0x038, // 0x00E0
        ATI_CFG_CHIP_TYPE   =  0, ATI_CFG_CHIP_TYPE_size   = 16,
        ATI_CFG_CHIP_CLASS  = 16, ATI_CFG_CHIP_CLASS_size  =  8,
        ATI_CFG_CHIP_MAJOR  = 24, ATI_CFG_CHIP_MAJOR_size  =  3,
        ATI_CFG_CHIP_FND_ID = 27, ATI_CFG_CHIP_FND_ID_size =  3,
        ATI_CFG_CHIP_MINOR  = 30, ATI_CFG_CHIP_MINOR_size  =  2,

    ATI_CONFIG_STAT0          = 0x039, // 0x00E4
        ATI_CFG_MEM_TYPE    = 0, ATI_CFG_MEM_TYPE_size = 3,
        ATI_CFG_DUAL_CAS_EN = 3,                        // VT/GT
        ATI_CFG_VGA_EN      = 4,                        // VT/GT
        ATI_CFG_CLOCK_EN    = 5,                        // VT/GT
        ATI_VMC_SENSE       = 6,                        // VT/GT
        ATI_VFC_SENSE       = 7,                        // VT/GT
        ATI_BOARD_ID        = 8, ATI_BOARD_ID_size = 8, // VT/GT

    ATI_GX_CONFIG_STAT1       = 0x03A, // 0x00E8 <-- GX/CT specific
    ATI_CRC_SIG               = 0x03A, // 0x00E8

    ATI_DST_OFF_PITCH         = 0x040, // 0x0100
        ATI_DST_OFFSET  =  0, ATI_DST_OFFSET_size = 20,
        ATI_DST_PITCH   = 22, ATI_DST_PITCH_size  = 10,

    ATI_DST_X                 = 0x041, // 0x0104
        ATI_DST_X_pos = 0, ATI_DST_X_size = 13,

    ATI_DST_Y                 = 0x042, // 0x0108
        ATI_DST_Y_pos = 0, ATI_DST_Y_size = 15,

    ATI_DST_Y_X               = 0x043, // 0x010C

    ATI_DST_WIDTH             = 0x044, // 0x0110
        ATI_DST_WIDTH_pos               =  0, ATI_DST_WIDTH_size = 13, // VT/GT
        ATI_DST_BRES_LENGTH_ALIAS_pos   = 13, ATI_DST_BRES_LENGTH_ALIAS_size = 3, // GT
        DST_WIDTH_FILL_DIS  = 31,

    ATI_DST_HEIGHT            = 0x045, // 0x0114
        ATI_DST_HEIGHT_pos  = 0, ATI_DST_HEIGHT_size = 15,

    ATI_DST_HEIGHT_WIDTH      = 0x046, // 0x0118
    ATI_DST_X_WIDTH           = 0x047, // 0x011C
    ATI_DST_BRES_LNTH         = 0x048, // 0x0120
    ATI_DST_BRES_ERR          = 0x049, // 0x0124
    ATI_DST_BRES_INC          = 0x04A, // 0x0128
    ATI_DST_BRES_DEC          = 0x04B, // 0x012C
    ATI_DST_CNTL              = 0x04C, // 0x0130
    ATI_DST_Y_X_ALIAS1        = 0x04D, // 0x0134
    ATI_TRAIL_BRES_ERR        = 0x04E, // 0x0138
    ATI_TRAIL_BRES_INC        = 0x04F, // 0x013C
    ATI_TRAIL_BRES_DEC        = 0x050, // 0x0140
    ATI_DST_BRES_LNTH_ALIAS1  = 0x051, // 0x0144
    ATI_Z_OFF_PITCH           = 0x052, // 0x0148
    ATI_Z_CNTL                = 0x053, // 0x014C
    ATI_ALPHA_TST_CNTL        = 0x054, // 0x0150
    ATI_SRC_OFF_PITCH         = 0x060, // 0x0180
    ATI_SRC_X                 = 0x061, // 0x0184
    ATI_SRC_Y                 = 0x062, // 0x0188
    ATI_SRC_Y_X               = 0x063, // 0x018C
    ATI_SRC_WIDTH1            = 0x064, // 0x0190
    ATI_SRC_HEIGHT1           = 0x065, // 0x0194
    ATI_SRC_HEIGHT1_WIDTH1    = 0x066, // 0x0198
    ATI_SRC_X_START           = 0x067, // 0x019C
    ATI_SRC_Y_START           = 0x068, // 0x01A0
    ATI_SRC_Y_X_START         = 0x069, // 0x01A4
    ATI_SRC_WIDTH2            = 0x06A, // 0x01A8
    ATI_SRC_HEIGHT2           = 0x06B, // 0x01AC
    ATI_SRC_HEIGHT2_WIDTH2    = 0x06C, // 0x01B0
    ATI_SRC_CNTL              = 0x06D, // 0x01B4
    ATI_SCALE_OFF             = 0x070, // 0x01C0
    ATI_SCALE_WIDTH           = 0x077, // 0x01DC
    ATI_SCALE_HEIGHT          = 0x078, // 0x01E0
    ATI_SCALE_PITCH           = 0x07B, // 0x01EC
    ATI_SCALE_X_INC           = 0x07C, // 0x01F0
    ATI_SCALE_Y_INC           = 0x07D, // 0x01F4
    ATI_SCALE_VACC            = 0x07E, // 0x01F8
    ATI_SCALE_3D_CNTL         = 0x07F, // 0x01FC
    ATI_HOST_CNTL             = 0x090, // 0x0240

    ATI_PAT_REG0              = 0x0A0, // 0x0280
    ATI_PAT_REG1              = 0x0A1, // 0x0284
    ATI_PAT_CNTL              = 0x0A2, // 0x0288

    ATI_SC_LEFT               = 0x0A8, // 0x02A0
        ATI_SC_LEFT_pos = 0, ATI_SC_LEFT_size = 13,

    ATI_SC_RIGHT              = 0x0A9, // 0x02A4
        ATI_SC_RIGHT_pos = 0, ATI_SC_RIGHT_size = 13,

    ATI_SC_LEFT_RIGHT         = 0x0AA, // 0x02A8

    ATI_SC_TOP                = 0x0AB, // 0x02AC
        ATI_SC_TOP_pos = 0, ATI_SC_TOP_size = 15,

    ATI_SC_BOTTOM             = 0x0AC, // 0x02B0
        ATI_SC_BOTTOM_pos = 0, ATI_SC_BOTTOM_size = 15,

    ATI_SC_TOP_BOTTOM         = 0x0AD, // 0x02B4

    ATI_DP_BKGD_CLR           = 0x0B0, // 0x02C0
    ATI_DP_FRGD_CLR           = 0x0B1, // 0x02C4
    ATI_DP_FOG_CLR            = 0x0B1, // 0x02C4
    ATI_DP_WRITE_MSK          = 0x0B2, // 0x02C8
    ATI_DP_CHAIN_MSK          = 0x0B3, // 0x02CC

    ATI_DP_PIX_WIDTH          = 0x0B4, // 0x02D0
        ATI_DP_DST_PIX_WIDTH        =  0, ATI_DP_DST_PIX_WIDTH_size  = 4,  // VT/GT ; 3 for VT
        ATI_DP_SRC_PIX_WIDTH        =  8, ATI_DP_SRC_PIX_WIDTH_size  = 4,  // VT/GT ; 3 for VT
        ATI_DP_HOST_PIX_WIDTH       = 16, ATI_DP_HOST_PIX_WIDTH_size = 4,  // VT/GT ; 3 for VT
        ATI_DP_CI4_RGB_INDEX        = 20, ATI_DP_CI4_RGB_INDEX_size  = 4,  // GT
        ATI_DP_BYTE_PIX_ORDER       = 24,  // VT/GT
        ATI_DP_CONVERSION_TEMP      = 25,  // GT
        ATI_DP_CI4_RGB_LOW_NIBBLE   = 26,  // GT
        ATI_DP_CI4_RGB_HIGH_NIBBLE  = 27,  // GT
        DP_SCALE_PIX_WIDTH          = 28, DP_SCALE_PIX_WIDTH_size   = 4,  // GT

    ATI_DP_MIX                = 0x0B5, // 0x02D4
        ATI_DP_BKGD_MIX =  0, ATI_DP_BKGD_MIX_size = 5,
        ATI_DP_FRGD_MIX = 16, ATI_DP_FRGD_MIX_size = 5,

    ATI_DP_SRC                = 0x0B6, // 0x02D8
        ATI_DP_BKGD_SRC =  0, ATI_DP_BKGD_SRC_size = 3,
        ATI_DP_FRGD_SRC =  8, ATI_DP_FRGD_SRC_size = 3,
        ATI_DP_MONO_SRC = 16, ATI_DP_MONO_SRC_size = 2,

    ATI_DP_FRGD_CLR_MIX       = 0x0B7, // 0x02DC
    ATI_DP_FRGD_BKGD_CLR      = 0x0B8, // 0x02E0
    ATI_DST_X_Y               = 0x0BA, // 0x02E8
    ATI_DST_WIDTH_HEIGHT      = 0x0BB, // 0x02EC
    ATI_USR_DST_PITCH         = 0x0BC, // 0x02F0
    ATI_DP_SET_GUI_ENGINE2    = 0x0BE, // 0x02F8
    ATI_DP_SET_GUI_ENGINE     = 0x0BF, // 0x02FC
    ATI_CLR_CMP_CLR           = 0x0C0, // 0x0300
    ATI_CLR_CMP_MSK           = 0x0C1, // 0x0304

    ATI_CLR_CMP_CNTL          = 0x0C2, // 0x0308
        ATI_CLR_CMP_FCN =  0, ATI_CLR_CMP_FCN_size = 3, // VT/GT
        ATI_CLR_CMP_SRC = 24, ATI_CLR_CMP_SRC_size = 2, // VT/GT ; 1 for VT

    ATI_FIFO_STAT             = 0x0C4, // 0x0310
        ATI_FIFO_STAT_pos =  0, ATI_FIFO_STAT_size = 16,
        ATI_FIFO_ERR      = 31,

    ATI_CONTEXT_MASK          = 0x0C8, // 0x0320 <-- 264VT specific

    ATI_GUI_TRAJ_CNTL         = 0x0CC, // 0x0330
        ATI_DST_X_DIR              =  0,                             // VT/GT
        ATI_DST_Y_DIR              =  1,                             // VT/GT
        ATI_DST_Y_MAJOR            =  2,                             // VT/GT
        ATI_DST_X_TILE             =  3,                             // VT/GT
        ATI_DST_Y_TILE             =  4,                             // VT/GT
        ATI_DST_LAST_PEL           =  5,                             // VT/GT
        ATI_DST_POLYGON_EN         =  6,                             // VT/GT
        ATI_DST_24_ROT_EN          =  7,                             // VT/GT
        ATI_DST_24_ROT             =  8, ATI_DST_24_ROT_size = 3,    // VT/GT
        ATI_DST_BRES_SIGN          = 11,                             // VT/GT
        ATI_DST_POLYGON_RTEDGE_DIS = 12,                             // VT/GT
        ATI_TRAIL_X_DIR            = 13,                             // GT
        ATI_TRAP_FILL_DIR          = 14,                             // GT
        ATI_TRAIL_BRES_SIGN        = 15,                             // GT
        ATI_SRC_PATT_EN            = 16,                             // VT/GT
        ATI_SRC_PATT_ROT_EN        = 17,                             // VT/GT
        ATI_SRC_LINEAR_EN          = 18,                             // VT/GT
        ATI_SRC_LINE_X_DIR         = 19,                             // VT/GT
        ATI_SRC_TRACK_DST          = 20,                             // VT/GT
        ATI_PAT_MONO_EN            = 23,                             // GT
        ATI_PAT_CLR_4x2_EN         = 24,                             // VT/GT
        ATI_PAT_CLR_8x1_EN         = 25,                             // VT/GT
        ATI_HOST_BYTE_ALIGN        = 26,                             // VT/GT
        ATI_HOST_BIG_ENDIAN_EN     = 28,                             // VT/GT
        ATI_SRC_BYTE_ALIGN         = 29,                             // VT/GT

    ATI_GUI_STAT              = 0x0CE, // 0x0338
        ATI_GUI_ACTIVE             =  0,
        ATI_DSTX_LT_SCISSOR_LEFT   =  8,
        ATI_DSTX_GT_SICISSOR_RIGHT =  9,
        ATI_DSTY_LT_SCISSOR_TOP    = 10,
        ATI_DSTY_GT_SCISSOR_BOTTOM = 11,
        ATI_FIFO_CNT               = 16, ATI_FIFO_CNT_size = 8,

    ATI_MPP_CONFIG            = 0x130, // 0x04C0
    ATI_MPP_STROBE_SEQ        = 0x131, // 0x04C4
    ATI_MPP_ADDR              = 0x132, // 0x04C8
    ATI_MPP_DATA              = 0x133, // 0x04CC
    ATI_TVO_CNTL              = 0x140, // 0x0500
    ATI_SETUP_CNTL            = 0x1C1, // 0x0704
    ATI_INVALID               = 0xFFFF
};

static int atirage_reg_names = {
    #define one_reg_name(x) {ATI_ ## x, #x}
    one_reg_name(CRTC_H_TOTAL_DISP),
    one_reg_name(CRTC_H_SYNC_STRT_WID),
    one_reg_name(CRTC_V_TOTAL_DISP),
    one_reg_name(CRTC_V_SYNC_STRT_WID),
    one_reg_name(CRTC_VLINE_CRNT_VLINE),
    one_reg_name(CRTC_OFF_PITCH),
    one_reg_name(CRTC_INT_CNTL),
    one_reg_name(CRTC_GEN_CNTL),
    one_reg_name(DSP_CONFIG),
    one_reg_name(DSP_ON_OFF),
    one_reg_name(MEM_BUF_CNTL),
    one_reg_name(MEM_ADDR_CFG),
    one_reg_name(OVR_CLR),
    one_reg_name(OVR_WID_LEFT_RIGHT),
    one_reg_name(OVR_WID_TOP_BOTTOM),
    one_reg_name(CUR_CLR0),
    one_reg_name(CUR_CLR1),
    one_reg_name(CUR_OFFSET),
    one_reg_name(CUR_HORZ_VERT_POSN),
    one_reg_name(CUR_HORZ_VERT_OFF),
    one_reg_name(GP_IO),
    one_reg_name(HW_DEBUG),
    one_reg_name(SCRATCH_REG0),
    one_reg_name(SCRATCH_REG1),
    one_reg_name(SCRATCH_REG2),
    one_reg_name(SCRATCH_REG3),
    one_reg_name(CLOCK_CNTL),
    one_reg_name(BUS_CNTL),
    one_reg_name(EXT_MEM_CNTL),
    one_reg_name(MEM_CNTL),
    one_reg_name(DAC_REGS),
    one_reg_name(DAC_CNTL),
    one_reg_name(GEN_TEST_CNTL),
    one_reg_name(CUSTOM_MACRO_CNTL),
    one_reg_name(CONFIG_CHIP_ID),
    one_reg_name(CONFIG_STAT0),
    one_reg_name(SRC_CNTL),
    one_reg_name(SCALE_3D_CNTL),
    one_reg_name(FIFO_STAT),
    one_reg_name(GUI_STAT),
    one_reg_name(MPP_CONFIG),
    one_reg_name(MPP_STROBE_SEQ),
    one_reg_name(MPP_ADDR),
    one_reg_name(MPP_DATA),
    one_reg_name(TVO_CNTL),
    one_reg_name(SETUP_CNTL),
};

void
atirage_out(uint16_t addr, uint8_t val, void *priv)
{
    atirage_t  *atirage  = (atirage_t *) priv;
    svga_t *svga = &atirage->svga;
    uint8_t old;

    if ((((addr & 0xFFF0) == 0x3D0 || (addr & 0xFFF0) == 0x3B0) && addr < 0x3de) && !(svga->miscout & 1))
        addr ^= 0x60;

    switch (addr) {
        case 0x3D4:
            svga->crtcreg = val & 0x3f;
            return;
        case 0x3D5:
            if (svga->crtcreg & 0x20)
                return;
            if ((svga->crtcreg < 7) && (svga->crtc[0x11] & 0x80))
                return;
            if ((svga->crtcreg == 7) && (svga->crtc[0x11] & 0x80))
                val = (svga->crtc[7] & ~0x10) | (val & 0x10);
            old                       = svga->crtc[svga->crtcreg];
            svga->crtc[svga->crtcreg] = val;
            if (old != val) {
                if (svga->crtcreg < 0xe || svga->crtcreg > 0x10) {
                    if ((svga->crtcreg == 0xc) || (svga->crtcreg == 0xd)) {
                        svga->fullchange = 3;
                        svga->ma_latch   = ((svga->crtc[0xc] << 8) | svga->crtc[0xd]) + ((svga->crtc[8] & 0x60) >> 5);
                    } else {
                        svga->fullchange = changeframecount;
                        svga_recalctimings(svga);
                    }
                }
            }
            break;

        default:
            break;
    }
    svga_out(addr, val, svga);
}

uint8_t
atirage_in(uint16_t addr, void *priv)
{
    atirage_t  *atirage  = (atirage_t *) priv;
    svga_t *svga = &atirage->svga;
    uint8_t temp;

    if ((((addr & 0xFFF0) == 0x3D0 || (addr & 0xFFF0) == 0x3B0) && addr < 0x3de) && !(svga->miscout & 1))
        addr ^= 0x60;

    switch (addr) {
        case 0x3D4:
            temp = svga->crtcreg;
            break;
        case 0x3D5:
            if (svga->crtcreg & 0x20)
                temp = 0xff;
            else
                temp = svga->crtc[svga->crtcreg];
            break;
		case 0x3DE:
			temp = (atirage->reg1_base >> 24) & 0xFF;
			break;		
		case 0x3DF:
			temp = (atirage->io_base >> 8) & 0xFF;
			break;
        default:
            temp = svga_in(addr, svga);
            break;
    }
    return temp;
}

void
atirage_recalcmapping(atirage_t *atirage)
{
    svga_t *svga = &atirage->svga;
    int     map;

    if (!(atirage->pci_regs[PCI_REG_COMMAND] & PCI_COMMAND_MEM)) {
        mem_mapping_disable(&svga->mapping);
        mem_mapping_disable(&atirage->reg1_mapping);
        mem_mapping_disable(&atirage->reg_mapping);
        return;
    }
		switch (svga->gdcreg[6] & 0xc) { /*Banked framebuffer*/
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

			default:
				break;
		}
    mem_mapping_set_addr(&atirage->reg1_mapping, atirage->reg1_base & 0xD6000000, 0x1000000);
    mem_mapping_set_addr(&atirage->reg_mapping, atirage->reg_base & 0xD5000000, 0x1000);
}

#define READ8(addr, var)                \
    switch ((addr) &3) {                \
        case 0:                         \
            ret = (var) &0xff;          \
            break;                      \
        case 1:                         \
            ret = ((var) >> 8) & 0xff;  \
            break;                      \
        case 2:                         \
            ret = ((var) >> 16) & 0xff; \
            break;                      \
        case 3:                         \
            ret = ((var) >> 24) & 0xff; \
            break;                      \
    }

#define WRITE8(addr, var, val)                        \
    switch ((addr) &3) {                              \
        case 0:                                       \
            var = (var & 0xffffff00) | (val);         \
            break;                                    \
        case 1:                                       \
            var = (var & 0xffff00ff) | ((val) << 8);  \
            break;                                    \
        case 2:                                       \
            var = (var & 0xff00ffff) | ((val) << 16); \
            break;                                    \
        case 3:                                       \
            var = (var & 0x00ffffff) | ((val) << 24); \
            break;                                    \
    }

uint8_t
atirage_reg_in(uint32_t addr, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
    svga_t *svga = &atirage->svga;
	uint8_t ret = 0x00;
	uint32_t reg_addr = addr & 3;
	switch (addr) {
    case ATI_CLOCK_CNTL:
       	svga_recalctimings(&atirage->svga);
        break;
    case ATI_DAC_REGS:
        switch (reg_addr) {
        case ATI_DAC_W_INDEX:
            svga_recalctimings(&atirage->svga);
            break;
        case ATI_DAC_MASK:
            svga_recalctimings(&atirage->svga);
            break;
        case ATI_DAC_R_INDEX:
            svga_recalctimings(&atirage->svga);
            break;
        case ATI_DAC_DATA:
            svga_recalctimings(&atirage->svga);
        }
        break;
    }
    return ret;
}

uint16_t
atirage_reg_inw(uint32_t addr, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
    //fatal("%s, 0x%08X\n", __func__, addr);
}

uint32_t
atirage_reg_inl(uint32_t addr, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
	uint32_t ret;
	switch (addr)
	{
		default:
    		ret = (atirage_reg_in(addr, atirage)) |
					(atirage_reg_in(addr + 1, atirage) << 8) |
					(atirage_reg_in(addr + 2, atirage) << 16) |
					(atirage_reg_in(addr + 3, atirage) << 24);
			break;
	}
	return ret;
}

void
atirage_reg_out(uint32_t addr, uint8_t val, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
    svga_t *svga = &atirage->svga;
	uint32_t new_value;
	switch (addr)
	{
		case ATI_CRTC_H_TOTAL_DISP:
			WRITE8(addr, atirage->crtc_h_total_disp, val);
			svga_recalctimings(&atirage->svga);
			break;
    	case ATI_CRTC_VLINE_CRNT_VLINE:
        	new_value = val;
        	svga_recalctimings(&atirage->svga);
        	break;
    	case ATI_CRTC_OFF_PITCH:
        	WRITE8(addr, atirage->crtc_off_pitch, val);
        	svga_recalctimings(&atirage->svga);
        	break;
		case ATI_CRTC_INT_CNTL: {
        uint32_t bits_read_only =
            (1 << ATI_CRTC_VBLANK) |
            (1 << ATI_CRTC_VLINE_SYNC) |
            (1 << ATI_CRTC_FRAME) |
#if 1
#else
            (1 << ATI_CRTC2_VBLANK) |
            (1 << ATI_CRTC2_VLINE_SYNC) |
#endif
            0;

        uint32_t bits_AK =
            (1 << ATI_CRTC_VBLANK_INT_AK) |
            (1 << ATI_CRTC_VLINE_INT_AK) |
#if 1
            (1 << ATI_VIDEOIN_EVEN_INT_AK) |
            (1 << ATI_VIDEOIN_ODD_INT_AK) |
            (1 << ATI_OVERLAY_EOF_INT_AK) |
            (1 << ATI_VMC_EC_INT_AK) |
#else
            (1 << ATI_SNAPSHOT_INT_AK) |
            (1 << ATI_I2C_INT_AK) |
            (1 << ATI_CRTC2_VBLANK_INT_AK) |
            (1 << ATI_CRTC2_VLINE_INT_AK) |
            (1 << ATI_CUPBUF0_INT_AK) |
            (1 << ATI_CUPBUF1_INT_AK) |
            (1 << ATI_OVERLAY_EOF_INT_AK) |
            (1 << ATI_ONESHOT_CAP_INT_AK) |
            (1 << ATI_BUSMASTER_EOL_INT_AK) |
            (1 << ATI_GP_INT_AK) |
            (1 << ATI_SNAPSHOT2_INT_AK) |
            (1 << ATI_VBLANK_BIT2_INT_AK) |
#endif
            0;
/*
        uint32_t bits_EN =
            (1 << ATI_CRTC_VBLANK_INT_EN) |
            (1 << ATI_CRTC_VLINE_INT_EN) |
#if 1
            (1 << ATI_VIDEOIN_EVEN_INT_EN) |
            (1 << ATI_VIDEOIN_ODD_INT_EN) |
            (1 << ATI_OVERLAY_EOF_INT_EN) |
            (1 << ATI_VMC_EC_INT_EN) |
#else
            (1 << ATI_SNAPSHOT_INT_EN) |
            (1 << ATI_I2C_INT_EN) |
            (1 << ATI_CRTC2_VBLANK_INT_EN) |
            (1 << ATI_CRTC2_VLINE_INT_EN) |
            (1 << ATI_CUPBUF0_INT_EN) |
            (1 << ATI_CUPBUF1_INT_EN) |
            (1 << ATI_OVERLAY_EOF_INT_EN) |
            (1 << ATI_ONESHOT_CAP_INT_EN) |
            (1 << ATI_BUSMASTER_EOL_INT_EN) |
            (1 << ATI_GP_INT_EN) |
            (1 << ATI_SNAPSHOT2_INT_EN) |
#endif
            0;
*/
        svga_recalctimings(&atirage->svga);
        break;
    }
    case ATI_CONFIG_CHIP_ID:
        WRITE8(addr, atirage->config_chip_id, val);
        break;
    case ATI_CRTC_GEN_CNTL:
        new_value = val;
        WRITE8(addr, atirage->crtc_gen_cntl, val);
        if (bit_changed(val, new_value, ATI_CRTC_DISPLAY_DIS)) {
            if (bit_set(new_value, ATI_CRTC_DISPLAY_DIS)) {
                svga->fb_only = 1;
                svga_recalctimings(&atirage->svga);
            } else {
                svga->fb_only = 0;
            }
        }
        break;

		default:
			//fatal("%s, 0x%08X, 0x%02X\n", __func__, addr, val);
			break;
	}
}

void
atirage_reg_outw(uint32_t addr, uint16_t val, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
	switch (addr)
	{
		default:
    		//fatal("%s, 0x%08X, 0x%04X\n", __func__, addr, val);
			break;
	}
}

void
atirage_reg_outl(uint32_t addr, uint32_t val, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
    svga_t *svga = &atirage->svga;

	uint32_t new_value;
	switch (addr)
	{
		case ATI_CRTC_H_TOTAL_DISP:
			WRITE8(addr, atirage->crtc_h_total_disp, val);
			svga_recalctimings(&atirage->svga);
			break;
    	case ATI_CRTC_VLINE_CRNT_VLINE:
        	new_value = val;
        	svga_recalctimings(&atirage->svga);
        	break;
    	case ATI_CRTC_OFF_PITCH:
        	WRITE8(addr, atirage->crtc_off_pitch, val);
        	svga_recalctimings(&atirage->svga);
        	break;
		case ATI_CRTC_INT_CNTL: {
        uint32_t bits_read_only =
            (1 << ATI_CRTC_VBLANK) |
            (1 << ATI_CRTC_VLINE_SYNC) |
            (1 << ATI_CRTC_FRAME) |
#if 1
#else
            (1 << ATI_CRTC2_VBLANK) |
            (1 << ATI_CRTC2_VLINE_SYNC) |
#endif
            0;

        uint32_t bits_AK =
            (1 << ATI_CRTC_VBLANK_INT_AK) |
            (1 << ATI_CRTC_VLINE_INT_AK) |
#if 1
            (1 << ATI_VIDEOIN_EVEN_INT_AK) |
            (1 << ATI_VIDEOIN_ODD_INT_AK) |
            (1 << ATI_OVERLAY_EOF_INT_AK) |
            (1 << ATI_VMC_EC_INT_AK) |
#else
            (1 << ATI_SNAPSHOT_INT_AK) |
            (1 << ATI_I2C_INT_AK) |
            (1 << ATI_CRTC2_VBLANK_INT_AK) |
            (1 << ATI_CRTC2_VLINE_INT_AK) |
            (1 << ATI_CUPBUF0_INT_AK) |
            (1 << ATI_CUPBUF1_INT_AK) |
            (1 << ATI_OVERLAY_EOF_INT_AK) |
            (1 << ATI_ONESHOT_CAP_INT_AK) |
            (1 << ATI_BUSMASTER_EOL_INT_AK) |
            (1 << ATI_GP_INT_AK) |
            (1 << ATI_SNAPSHOT2_INT_AK) |
            (1 << ATI_VBLANK_BIT2_INT_AK) |
#endif
            0;
/*
        uint32_t bits_EN =
            (1 << ATI_CRTC_VBLANK_INT_EN) |
            (1 << ATI_CRTC_VLINE_INT_EN) |
#if 1
            (1 << ATI_VIDEOIN_EVEN_INT_EN) |
            (1 << ATI_VIDEOIN_ODD_INT_EN) |
            (1 << ATI_OVERLAY_EOF_INT_EN) |
            (1 << ATI_VMC_EC_INT_EN) |
#else
            (1 << ATI_SNAPSHOT_INT_EN) |
            (1 << ATI_I2C_INT_EN) |
            (1 << ATI_CRTC2_VBLANK_INT_EN) |
            (1 << ATI_CRTC2_VLINE_INT_EN) |
            (1 << ATI_CUPBUF0_INT_EN) |
            (1 << ATI_CUPBUF1_INT_EN) |
            (1 << ATI_OVERLAY_EOF_INT_EN) |
            (1 << ATI_ONESHOT_CAP_INT_EN) |
            (1 << ATI_BUSMASTER_EOL_INT_EN) |
            (1 << ATI_GP_INT_EN) |
            (1 << ATI_SNAPSHOT2_INT_EN) |
#endif
            0;
*/
        svga_recalctimings(&atirage->svga);
        break;
    }
    case ATI_CONFIG_CHIP_ID:
        WRITE8(addr, atirage->config_chip_id, val);
        break;
    case ATI_CRTC_GEN_CNTL:
        new_value = val;
        WRITE8(addr, atirage->crtc_gen_cntl, val);
        if (bit_changed(val, new_value, ATI_CRTC_DISPLAY_DIS)) {
            if (bit_set(new_value, ATI_CRTC_DISPLAY_DIS)) {
                svga->fb_only = 1;
            } else {
                svga->fb_only = 0;
            }
            svga_recalctimings(&atirage->svga);
        }
        break;
		default:
			//fatal("%s, 0x%08X, 0x%08X\n", __func__, addr, val);
			break;
	}
    
}

uint8_t
atirage_ext_in(uint16_t addr, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
    return atirage_reg_in(addr, priv);
}

uint16_t
atirage_ext_inw(uint16_t addr, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
    return atirage_reg_inw(addr, priv);
}

uint32_t
atirage_ext_inl(uint16_t addr, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
    return atirage_reg_inl(addr, priv);
}

void
atirage_ext_out(uint16_t addr, uint8_t val, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;

	atirage_reg_out(addr & 0xFF, val, priv);
    //fatal("%s, 0x%04X, 0x%02X\n", __func__, addr, val);
}

void
atirage_ext_outw(uint16_t addr, uint16_t val, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
	atirage_reg_outw(addr & 0xFF, val, priv);
    //fatal("%s, 0x%04X, 0x%04X\n", __func__, addr, val);
}

void
atirage_ext_outl(uint16_t addr, uint32_t val, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
	atirage_reg_outl(addr & 0xFF, val, priv);
    //fatal("%s, 0x%04X, 0x%08X\n", __func__, addr, val);
}

uint16_t
atirage_reg1_in(uint32_t addr, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
    //fatal("%s, 0x%08X\n", __func__, addr);
}

void
atirage_reg1_out(uint16_t addr, uint32_t val, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
    //fatal("%s, 0x%04X, 0x%08X\n", __func__, addr, val);
}

static void
atirage_io_remove(atirage_t *atirage)
{
    io_removehandler(0x03c0, 0x0020, atirage_in, NULL, NULL, atirage_out, NULL, NULL, atirage);

    io_removehandler(atirage->io_base, 0x100, atirage_ext_in, atirage_ext_inw, atirage_ext_inl, atirage_ext_out, atirage_ext_outw, atirage_ext_outl, atirage);
}

static void
atirage_io_set(atirage_t *atirage)
{
    atirage_io_remove(atirage);
    io_sethandler(0x03c0, 0x0020, atirage_in, NULL, NULL, atirage_out, NULL, NULL, atirage);

    io_sethandler(atirage->io_base, 0x100, atirage_ext_in, atirage_ext_inw, atirage_ext_inl, atirage_ext_out, atirage_ext_outw, atirage_ext_outl, atirage);
}

uint8_t
atirage_pci_read(UNUSED(int func), int addr, void *priv)
{
    const atirage_t *atirage = (atirage_t *) priv;

    addr &= 0xff;

    switch (addr) {
        case 0x00:
            return 0x02; /* ATi */
        case 0x01:
            return 0x10;

        case 0x02:
            return 0x5a;
        case 0x03:
            return 0x47;

   		case 0x04:
        	return atirage->pci_regs[0x04] & 0x37;
    	case 0x05:
        	return atirage->pci_regs[0x05];

		case 0x06:
			return 0x90;

        case 0x07:
            return 0x02;

        case 0x08:
            return 0x7a; /* Revision ID */
        case 0x09:
            return 0; /* Programming interface */

        case 0x0a:
            return 0x00; /* Supports VGA interface */
        case 0x0b:
            return 0x03;

        case 0x10:
            return 0x08
        case 0x11:
            return 0x00;
        case 0x12:
            return 0x00;
        case 0x13:
            return (atirage->reg1_base >> 24);

        case 0x14:
            return 0x01;

        case 0x15:
        case 0x16:
        case 0x17:
            return atirage->pci_regs[addr & 0xFF];
        
        case 0x18:
        case 0x19:
            return 0x00;
        case 0x1a:
            return (atirage->reg_base >> 16) & 0xfc;
        case 0x1b:
            return (atirage->reg_base >> 24);

        case 0x2c: case 0x2d: case 0x2e: case 0x2f:
            return atirage->pci_regs[addr];

        case 0x30:
            return atirage->pci_regs[0x30] & 0x01; /* BIOS ROM address */
        case 0x31:
            return 0x00;
        case 0x32:
            return atirage->pci_regs[0x32];
        case 0x33:
            return atirage->pci_regs[0x33];

    	case 0x34:
        	return 0x5c;

		case 0x3c:
			return atirage->pci_regs[0x3c];
		
		case 0x3d:
			return PCI_INTA;

    	case 0x5c:
        	return 0x1;
    
    	case 0x5d:
        	return 0x0;
        
    	case 0x5e:
        	return 0x1;

    	case 0x5f: /* Don't indicate any D1/D2 support at the moment. */
        	return 0x0;

    	case 0x60:
        	return 0x0;

        default:
            break;
    }

    return 0;
}

void
atirage_pci_write(UNUSED(int func), int addr, uint8_t val, void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;
    svga_t       *svga   = &atirage->svga;

    addr &= 0xff;
    switch (addr)
    {
    case PCI_REG_COMMAND:
        atirage->pci_regs[PCI_REG_COMMAND] = (val & 0x27);
        atirage_io_remove(atirage);
        if (val & PCI_COMMAND_IO)
            atirage_io_set(atirage);
        else
            atirage_io_remove(atirage);
        atirage_recalcmapping(atirage);
        break;
    
    case 0x13:
        atirage->pci_regs[addr] = val;
        atirage->reg1_base = (atirage->pci_regs[0x13] << 24);
        atirage_recalcmapping(atirage);
        break;

    case 0x15:
    case 0x16:
    case 0x17:
        atirage_io_remove(atirage);
        atirage->pci_regs[addr] = val;
        atirage->io_base = (atirage->pci_regs[0x15] << 8) | (atirage->pci_regs[0x16] << 16) | (atirage->pci_regs[0x17] << 24);
        if (atirage->pci_regs[PCI_REG_COMMAND] & PCI_COMMAND_IO) {
            atirage_io_set(atirage);
        }

    case 0x1a:
        atirage->pci_regs[addr] = val & 0xfc;
        atirage->reg_base = (atirage->pci_regs[0x1a] << 16) | (atirage->pci_regs[0x1b] << 24);
        atirage_recalcmapping(atirage);
        break;

    case 0x1b:
        atirage->pci_regs[addr] = val;
        atirage->reg_base = (atirage->pci_regs[0x1a] << 16) | (atirage->pci_regs[0x1b] << 24);
        atirage_recalcmapping(atirage);
        break;

    case 0x30: case 0x32:
    case 0x33:
        atirage->pci_regs[addr] = val;
        if (atirage->pci_regs[0x30] & 0x01) {
            uint32_t addr = (atirage->pci_regs[0x32] << 16)
                    | (atirage->pci_regs[0x33] << 24);
            mem_mapping_set_addr(&atirage->bios_rom.mapping, addr,
                    0x8000);
        } else
            mem_mapping_disable(&atirage->bios_rom.mapping);
        break;

	case 0x3c:
		atirage->pci_regs[addr] = val;
		break;

    default:
        break;
    }
}

void
atirage_recalctimings(svga_t* svga)
{
    const atirage_t *atirage = (atirage_t *) svga->priv;

    if (((atirage->crtc_gen_cntl >> 24) & 3) == 3) {
        svga->vtotal     = (atirage->crtc_v_total_disp & 2047) + 1;
        svga->dispend    = ((atirage->crtc_v_total_disp >> 16) & 2047) + 1;
        svga->htotal     = ((atirage->crtc_h_total_disp & 255) + 1) * 8;
        svga->hdisp_time = svga->hdisp = ((atirage->crtc_h_total_disp >> 16) & 255) + 1;
        svga->rowoffset                = (atirage->crtc_off_pitch >> 22);
        svga->ma_latch                 = (atirage->crtc_off_pitch & 0x1fffff) * 2;
        svga->linedbl = svga->rowcount = 0;
        svga->split                    = 0xffffff;
        svga->vblankstart              = svga->dispend;
        svga->rowcount                 = atirage->crtc_gen_cntl & 1;
        svga->rowoffset <<= 1;
                switch ((atirage->crtc_gen_cntl >> 8) & 7) {
                case 1:
                        svga->render = svga_render_4bpp_highres;
                        svga->hdisp *= 8;
                        break;
                case 2:
                        svga->render = svga_render_8bpp_highres;
                        svga->hdisp *= 8;
                        svga->rowoffset /= 2;
                        break;
                case 3:
                        svga->render = svga_render_15bpp_highres;
                        svga->hdisp *= 8;
                        // svga_rowoffset *= 2;
                        break;
                case 4:
                        svga->render = svga_render_16bpp_highres;
                        svga->hdisp *= 8;
                        // svga_rowoffset *= 2;
                        break;
                case 5:
                        svga->render = svga_render_24bpp_highres;
                        svga->hdisp *= 8;
                        svga->rowoffset = (svga->rowoffset * 3) / 2;
                        break;
                case 6:
                        svga->render = svga_render_32bpp_highres;
                        svga->hdisp *= 8;
                        svga->rowoffset *= 2;
                        break;
                }
    }
}

static void *
atirage_init(const device_t *info)
{
    atirage_t *atirage = calloc(1, sizeof(atirage_t));

    rom_init(&atirage->bios_rom, "roms/video/atirage/4mbsgram.vbi", 0xc0000, 0x8000, 0x7fff, 0x0000, MEM_MAPPING_EXTERNAL);
    mem_mapping_disable(&atirage->bios_rom.mapping);

    atirage->vram_size = device_get_config_int("memory") << 20;
    atirage->vram_mask = atirage->vram_size - 1;

    mem_mapping_add(&atirage->reg_mapping, 0, 0, atirage_reg_in, atirage_reg_inw, atirage_reg_inl, atirage_reg_out, atirage_reg_outw, atirage_reg_outl, NULL, MEM_MAPPING_EXTERNAL, atirage);
    mem_mapping_add(&atirage->reg1_mapping, 0, 0, atirage_reg1_in, NULL, NULL, atirage_reg1_out, NULL, NULL, NULL, MEM_MAPPING_EXTERNAL, atirage);

    svga_init(info, &atirage->svga, atirage, atirage->vram_size,
              atirage_recalctimings,
              atirage_in, atirage_out,
              NULL,
              NULL);

    video_inform(VIDEO_FLAG_TYPE_SPECIAL, &timing_atirage);

    io_sethandler(0x03c0, 0x0020, atirage_in, NULL, NULL, atirage_out, NULL, NULL, atirage);

    pci_add_card(PCI_ADD_AGP, atirage_pci_read, atirage_pci_write, atirage, &atirage->slot);

    atirage->pci_regs[0x04] = 0x87;
    atirage->pci_regs[0x07] = 0x02;

    atirage->pci_regs[0x30] = 0x00;
    atirage->pci_regs[0x32] = 0x0c;
    atirage->pci_regs[0x33] = 0x00;

    atirage->config_chip_id = 0x7a00475a;

    atirage->svga.force_old_addr = 1;
    atirage->svga.decode_mask = 0xffffff;

    return atirage;
}

void
atirage_close(void *priv)
{
    atirage_t *atirage = (atirage_t *) priv;

    svga_close(&atirage->svga);

    free(atirage);
}

int
atirage_available(void)
{
    return rom_present("roms/video/atirage/4mbsgram.vbi");
}

void
atirage_speed_changed(void *p)
{
    atirage_t *atirage = (atirage_t *)p;
    
    svga_recalctimings(&atirage->svga);
}


void
atirage_force_redraw(void *p)
{
    atirage_t *atirage = (atirage_t *)p;

    atirage->svga.fullchange = changeframecount;
}

static const device_config_t atirage_config[] = {
  // clang-format off
    {
        .name = "memory",
        .description = "Memory size",
        .type = CONFIG_SELECTION,
        .default_int = 4,
        .selection = {
            {
                .description = "2 MB",
                .value = 2
            },
            {
                .description = "4 MB",
                .value = 4
            },
            {
                .description = "8 MB",
                .value = 8
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

const device_t atirageiic_device = {
    .name          = "ATI Rage IIC",
    .internal_name = "atirage",
    .flags         = DEVICE_AGP,
    .local         = 0,
    .init          = atirage_init,
    .close         = atirage_close,
    .reset         = NULL,
    { .available = atirage_available },
    .speed_changed = atirage_speed_changed,
    .force_redraw  = atirage_force_redraw,
    .config        = atirage_config
};