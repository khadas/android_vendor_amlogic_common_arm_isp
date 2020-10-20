/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#include "acamera_command_api.h"
#include "acamera_firmware_settings.h"
// ------------ 3A & iridix
static uint8_t _calibration_evtolux_probability_enable[] = {0};

static uint8_t _calibration_awb_avg_coef[] = {15};

static uint8_t _calibration_iridix_avg_coef[] = {15};

static uint16_t _calibration_ccm_one_gain_threshold[] = {2048};

static uint8_t _calibration_iridix_strength_maximum[] = {255};

static uint16_t _calibration_iridix_min_max_str[] = {0};

static uint32_t _calibration_iridix_ev_lim_full_str[] = {1300000};

static uint32_t _calibration_iridix_ev_lim_no_str[] = {5180000, 3116186};

static uint8_t _calibration_ae_correction[] = {128, 128, 128, 128, 128, 108, 98, 98, 98, 90, 90, 90};

static uint32_t _calibration_ae_exposure_correction[] = {6710, 15739, 15778, 23282, 56186, 500325, 632161, 1190074, 1406400, 2382765, 3295034, 5491142}; //500,157778,500325,632161,1406400,6046465 //23282 - Max Lab Exposure

// ------------Noise reduction ----------------------//
static uint16_t _calibration_sinter_strength[][2] = {
    {0 * 256, 65},
    {1 * 256, 68},
    {2 * 256, 73},
    {3 * 256, 80},
    {4 * 256, 80},
    {5 * 256, 80},
    {6 * 256, 80},
    {7 * 256, 80}};

// ------------Noise reduction ----------------------//
static uint16_t _calibration_sinter_strength_MC_contrast[][2] = {
    {0 * 256, 0}};

static uint16_t _calibration_sinter_strength1[][2] = {
    {0 * 256, 50},
    {1 * 256, 90},
    {2 * 256, 90},
    {3 * 256, 95},
    {4 * 256, 100},
    {5 * 256, 110},
    {6 * 256, 130},
    {7 * 256, 150}};

static uint16_t _calibration_sinter_thresh1[][2] = {
    {0 * 256, 1},
    {1 * 256, 3},
    {2 * 256, 5},
    {3 * 256, 9},
    {4 * 256, 64},
    {5 * 256, 64},
    {6 * 256, 64},
    {7 * 256, 64},
    {8 * 256, 64}};

static uint16_t _calibration_sinter_thresh4[][2] = {
    {0 * 256, 1},
    {1 * 256, 3},
    {2 * 256, 5},
    {3 * 256, 9},
    {4 * 256, 64},
    {5 * 256, 64},
    {6 * 256, 64}};

static uint16_t _calibration_sinter_intConfig[][2] = {
    {0 * 256, 10},
    {1 * 256, 10},
    {2 * 256, 8},
    {3 * 256, 8},
    {4 * 256, 7},
    {5 * 256, 5},
    {6 * 256, 5},
    {7 * 256, 5},
    {8 * 256, 4}};

static uint8_t _calibration_sinter_radial_lut[] = {0, 0, 0, 0, 0, 0, 1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 18, 19, 21, 22, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24};

static uint16_t _calibration_sinter_radial_params[] = {
    0,        // rm_enable
    1920 / 2, // rm_centre_x
    1080 / 2, // rm_centre_y
    1770      // rm_off_centre_mult: round((2^31)/((rm_centre_x^2)+(rm_centre_y^2)))
};

static uint16_t _calibration_sinter_sad[][2] = {
    {0, 8},
    {1 * 256, 8},
    {2 * 256, 8},
    {3 * 256, 9},
    {4 * 256, 10},
    {5 * 256, 11},
    {6 * 256, 11},
    {7 * 256, 11},
    {8 * 256, 13}};

// ------------ Sharpening and demosaic
static uint16_t _calibration_sharp_alt_d[][2] = {
    {0 * 256, 25},
    {1 * 256, 25},
    {2 * 256, 25},
    {3 * 256, 25},
    {4 * 256, 20},
    {5 * 256, 20},
    {6 * 256, 20},
    {7 * 256, 20}};

static uint16_t _calibration_sharp_alt_ud[][2] = {
    {0 * 256, 15},
    {1 * 256, 15},
    {2 * 256, 15},
    {3 * 256, 13},
    {4 * 256, 10},
    {5 * 256, 10},
    {6 * 256, 10},
    {7 * 256, 10}};

static uint16_t _calibration_sharp_alt_du[][2] = {
    {0 * 256, 75},
    {1 * 256, 70},
    {2 * 256, 60},
    {3 * 256, 40},
    {4 * 256, 40},
    {5 * 256, 30},
    {6 * 256, 30},
    {7 * 256, 30}};

static uint16_t _calibration_sharpen_fr[][2] = {
    {0 * 256, 80},
    {1 * 256, 75},
    {2 * 256, 70},
    {3 * 256, 70},
    {4 * 256, 65},
    {5 * 256, 60},
    {6 * 256, 60},
    {7 * 256, 60},
    {8 * 256, 50},};

static uint16_t _calibration_demosaic_np_offset[][2] = {
    {0 * 256, 1},
    {1 * 256, 1},
    {2 * 256, 1},
    {3 * 256, 3},
    {4 * 256, 18},
    {5 * 256, 18},
    {6 * 256, 18},
    {7 * 256, 18},
    {8 * 256, 15}};

static uint16_t _calibration_mesh_shading_strength[][2] = {
    {0 * 256, 4096},
    {1 * 256, 4096},
    {2 * 256, 4096},
    {3 * 256, 4096},
    {4 * 256, 2048},
    {5 * 256, 512},
    {6 * 256, 512},
    {7 * 256, 512},
    {8 * 256, 128}};

static uint16_t _calibration_saturation_strength[][2] = {
    {0 * 256, 128},
    {1 * 256, 128},
    {2 * 256, 128},
    {3 * 256, 128},
    {4 * 256, 110},
    {5 * 256, 80},
    {6 * 256, 40},
    {7 * 256, 40},
    {8 * 256, 40}};

// ----------- Frame stitching motion
static uint16_t _calibration_stitching_lm_np[][2] = {
    {0, 540},
    {3 * 256, 1458},
    {4 * 256, 1458},
    {5 * 256, 3000}};

static uint16_t _calibration_stitching_lm_mov_mult[][2] = {
    {0, 128},
    {2 * 256 - 128, 20},
    {5 * 256, 8}};

static uint16_t _calibration_stitching_lm_med_noise_intensity_thresh[][2] = {
    {0, 32},
    {6 * 256, 32},
    {8 * 256, 4095}};

static uint16_t _calibration_stitching_ms_np[][2] = {
    {0, 3680},
    {1 * 256, 3680},
    {2 * 256, 2680}};

static uint16_t _calibration_stitching_ms_mov_mult[][2] = {
    {0, 128},
    {1 * 256, 128},
    {2 * 256, 100}};

static uint16_t _calibration_dp_slope[][2] = {
    {0 * 256, 170},
    {1 * 256, 170},
    {2 * 256, 170},
    {3 * 256, 200},
    {4 * 256, 400},
    {5 * 256, 600},
    {6 * 256, 800},
    {7 * 256, 1000},
    {8 * 256, 1800}};

static uint16_t _calibration_dp_threshold[][2] = {
    {0 * 256, 4095},
    {1 * 256, 3000},
    {2 * 256, 2000},
    {3 * 256, 1000},
    {4 * 256, 500},
    {5 * 256, 250},
    {6 * 256, 100},
    {7 * 256, 85},
    {8 * 256, 70}};

static uint16_t _calibration_AWB_bg_max_gain[][2] = {
    {0 * 256, 100},
    {1 * 256, 100},
    {7 * 256, 200}};

// *** NOTE: to add/remove items in partition luts, please also update SYSTEM_EXPOSURE_PARTITION_VALUE_COUNT.
static uint16_t _calibration_cmos_exposure_partition_luts[][10] = {
    // {integration time, gain }
    // value: for integration time - milliseconds, for gains - multiplier.
    //		  Zero value means maximum.
    // lut partitions_balanced
    {
        10, 2,
        30, 4,
        60, 6,
        100, 8,
        0, 0,
    },

    // lut partition_int_priority
    {
        0, 0,
        0, 0,
        0, 0,
        0, 0,
        0, 0,
    },
};

static uint32_t _calibration_cmos_control[] = {
    0,   // enable antiflicker
    50,  // antiflicker frequency
    0,   // manual integration time
    0,   // manual sensor analog gain
    0,   // manual sensor digital gain
    0,   // manual isp digital gain
    0,   // manual max integration time
    0,   // max integration time
    150, // max sensor AG
    0,   // max sensor DG
    112, // max isp DG
    255, // max exposure ratio
    0,   // integration time
    0,   // sensor analog gain. log2 fixed - 5 bits
    0,   // sensor digital gain. log2 fixed - 5 bits
    0,   // isp digital gain. log2 fixed - 5 bits
    0,   // analog_gain_last_priority
    4    // analog_gain_reserve
};


static uint32_t _calibration_status_info[] = {
    0xFFFFFFFF, // sys.total_gain_log2
    0xFFFFFFFF, // sys.expsoure_log2
    0xFFFFFFFF, // awb.mix_light_contrast
    0xFFFFFFFF, // af.cur_lens_pos
    0xFFFFFFFF  // af.cur_focus_value
};

static uint32_t _calibration_iridix8_strength_dk_enh_control[] = {
    25,      // dark_prc
    95,      // bright_prc
    450,     // min_dk: minimum dark enhancement
    1000,    // max_dk: maximum dark enhancement
    4,       // pD_cut_min: minimum intensity cut for dark regions in which dk_enh will be applied
    12,      // pD_cut_max: maximum intensity cut for dark regions in which dk_enh will be applied
    30 << 8, // dark contrast min
    50 << 8, // dark contrast max
    0,       // min_str: iridix strength in percentage
    20,      // max_str: iridix strength in percentage: 50 = 1x gain. 100 = 2x gain
    20,      // dark_prc_gain_target: target in histogram (percentage) for dark_prc after iridix is applied
    30 << 8, // contrast_min: clip factor of strength for LDR scenes.
    40 << 8, // contrast_max: clip factor of strength for HDR scenes.
    10,      // max iridix gain
    0        // print debug
};

static uint32_t _calibration_ae_control[] = {
    15,  // AE convergance
    210, // LDR AE target -> this should match the 18% grey of teh output gamma
    0,  // AE tail weight
    77,  // WDR mode only: Max percentage of clipped pixels for long exposure: WDR mode only: 256 = 100% clipped pixels
    15,  // WDR mode only: Time filter for exposure ratio
    100, // control for clipping: bright percentage of pixels that should be below hi_target_prc
    99,  // control for clipping: highlights percentage (hi_target_prc): target for tail of histogram
    0,   // 1:0 enable | disable iridix global gain.
    10,  // AE tolerance
};

static uint16_t _calibration_ae_control_HDR_target[][2] = {
    {0 * 256, 210}, // HDR AE target should not be higher than LDR target
    {1 * 256, 210},
    {2 * 256, 210},
    {3 * 256, 210},
    {4 * 256, 210},
    {5 * 256, 210},
    {6 * 256, 210},
    {7 * 256, 210},
    {8 * 256, 210}};

static uint8_t _calibration_pf_radial_lut[] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};

static uint16_t _calibration_pf_radial_params[] = {
    1920 / 2, // rm_centre_x
    1080 / 2, // rm_centre_y
    1770      // rm_off_centre_mult: round((2^31)/((540^2)+(960^2)))
};

static uint32_t _calibration_auto_level_control[] = {
    1,  // black_percentage
    99, // white_percentage
    20,  // auto_black_min
    50, // auto_black_max
    90, // auto_white_prc
    15, // avg_coeff
    1   // enable_auto_level
};


static uint16_t _calibration_exposure_ratio_adjustment[][2] = {
    //contrast u8.8, adjustment u8.8
    {1 * 256, 256},
    {16 * 256, 256},
    {32 * 256, 256},
    {64 * 256, 256}};


static uint16_t _calibration_cnr_uv_delta12_slope[][2] = {
    {0 * 256, 10000},
    {1 * 256, 15000},
    {2 * 256, 20000},
    {3 * 256, 30000},
    {4 * 256, 40000},
    {5 * 256, 60000},
    {6 * 256, 65000},
    {7 * 256, 65000},
    {8 * 256, 65000}};


static uint16_t _calibration_fs_mc_off[] = {
    // gain_log2 threshold. if gain is higher than the current gain_log2. mc off mode will be enabed.
    8 * 256,
};

static int16_t _AWB_colour_preference[] = {7500, 6000, 4700, 2800};

static uint32_t _calibration_awb_mix_light_parameters[] = {
    1,    // 1 = enable, 0 = disable
    10,  //lux low boundary for mix light lux range : range = {500: inf}
    3000, // lux high boundary for mix light range : range = {500: inf}
    1000, // contrast threshold for mix light: range = {200:2000}
    500,  //BG threshold {255:400}
    5,    // BG weight
    180,  // rgHigh_LUT_max
    440,  // rgHigh_LUT_min
    0     // print debug
};

static uint16_t _calibration_rgb2yuv_conversion[] = {77, 150, 29, 0x802b, 0x8055, 128, 128, 0x806b, 0x8015, 0, 512, 512};

static uint16_t _calibration_ae_zone_wght_hor[] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
static uint16_t _calibration_ae_zone_wght_ver[] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};

static uint16_t _calibration_awb_zone_wght_hor[] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
static uint16_t _calibration_awb_zone_wght_ver[] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};

static uint32_t _scaler_h_filter[] = {
    0x27f70200, 0x0002f727, 0x29f70200, 0x0002f824, 0x2cf70200, 0x0002f821, 0x2ef70200, 0x0002f91e, 0x30f70200, 0x0002f91c, 0x33f70200, 0x0001fa19, 0x35f70200, 0x0001fa17, 0x37f70200, 0x0001fb14, 0x39f70200, 0x0001fc11, 0x3af80200, 0x0001fc0f, 0x3bf90200, 0x0001fd0c, 0x3cfa0200, 0x0001fd0a, 0x3efb0100, 0x0000fe08, 0x3efc0100, 0x0000ff06, 0x3ffd0100, 0x0000ff04, 0x40fe0000, 0x00000002, 0x40000000, 0x00000000, 0x40020000, 0x000000fe, 0x3f04ff00, 0x000001fd, 0x3e06ff00, 0x000001fc, 0x3e08fe00, 0x000001fb, 0x3c0afd01, 0x000002fa, 0x3b0cfd01, 0x000002f9, 0x3a0ffc01, 0x000002f8, 0x3911fc01, 0x000002f7, 0x3714fb01, 0x000002f7, 0x3517fa01, 0x000002f7, 0x3319fa01, 0x000002f7, 0x301cf902, 0x000002f7, 0x2e1ef902, 0x000002f7, 0x2c21f802, 0x000002f7, 0x2924f802, 0x000002f7,
    0x25fbfd05, 0x00fdfb26, 0x27fcfc05, 0x00fefa24, 0x28fdfc05, 0x00fef923, 0x29fefb05, 0x00fff921, 0x29fffb06, 0x00fff820, 0x2a00fa06, 0x0000f81e, 0x2b01fa06, 0x0000f71d, 0x2b02f906, 0x0001f71c, 0x2c03f906, 0x0001f71a, 0x2d04f806, 0x0002f619, 0x2d06f806, 0x0002f617, 0x2d07f806, 0x0003f615, 0x2f08f705, 0x0003f614, 0x2f09f705, 0x0004f612, 0x2f0bf705, 0x0004f610, 0x2f0df605, 0x0004f60f, 0x2f0df605, 0x0005f60e, 0x2f0ff604, 0x0005f60d, 0x2f10f604, 0x0005f70b, 0x2f12f604, 0x0005f709, 0x2f14f603, 0x0005f708, 0x2d15f603, 0x0006f807, 0x2d17f602, 0x0006f806, 0x2d19f602, 0x0006f804, 0x2c1af701, 0x0006f903, 0x2b1cf701, 0x0006f902, 0x2b1df700, 0x0006fa01, 0x2a1ef800, 0x0006fa00, 0x2920f8ff, 0x0006fbff, 0x2921f9ff, 0x0005fbfe, 0x2823f9fe, 0x0005fcfd, 0x2724fafe, 0x0005fcfc,
    0x1e0afafc, 0x00fa0a1e, 0x1e0bfafc, 0x00fa091e, 0x1e0cfbfb, 0x00fa091d, 0x1f0cfbfb, 0x00fa081d, 0x200dfbfb, 0x00f9071d, 0x200efbfb, 0x00f9071c, 0x200efcfb, 0x00f9061c, 0x210ffcfa, 0x00f9051c, 0x2110fcfa, 0x00f9051b, 0x2111fdfa, 0x00f9041a, 0x2211fdfa, 0x00f9031a, 0x2212fefa, 0x00f90318, 0x2213fefa, 0x00f90218, 0x2213fff9, 0x00f90218, 0x2215fff9, 0x00f90117, 0x2215fff9, 0x00f90117, 0x221600f9, 0x00f90016, 0x221701f9, 0x00f9ff15, 0x221701f9, 0x00f9ff15, 0x221802f9, 0x00f9ff13, 0x221802f9, 0x00fafe13, 0x221803f9, 0x00fafe12, 0x221a03f9, 0x00fafd11, 0x211a04f9, 0x00fafd11, 0x211b05f9, 0x00fafc10, 0x211c05f9, 0x00fafc0f, 0x201c06f9, 0x00fbfc0e, 0x201c07f9, 0x00fbfb0e, 0x201d07f9, 0x00fbfb0d, 0x1f1d08fa, 0x00fbfb0c, 0x1e1d09fa, 0x00fbfb0c, 0x1e1e09fa, 0x00fcfa0b,
    0x0e0b0602, 0x00060b0e, 0x0e0b0702, 0x00060b0d, 0x0e0b0702, 0x00060b0d, 0x0e0b0702, 0x00060b0d, 0x0e0c0702, 0x00060a0d, 0x0e0c0702, 0x00060a0d, 0x0e0c0703, 0x00050a0d, 0x0e0c0703, 0x00050a0d, 0x0e0c0703, 0x00050a0d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0804, 0x0004090d, 0x0e0c0804, 0x0004090d, 0x0e0c0904, 0x0004090c, 0x0e0c0904, 0x0004090c, 0x0e0c0904, 0x0004090c, 0x0e0d0904, 0x0004080c, 0x0e0d0904, 0x0004080c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0a05, 0x0003070c, 0x0e0d0a05, 0x0003070c, 0x0e0d0a05, 0x0003070c, 0x0e0d0a06, 0x0002070c, 0x0e0d0a06, 0x0002070c, 0x0e0d0b06, 0x0002070b, 0x0e0d0b06, 0x0002070b, 0x0e0d0b06, 0x0002070b};

static uint32_t _scaler_v_filter[] = {
    0x00400000, 0x00000000, 0x0240fe00, 0x00000000, 0x0340fd01, 0x000000ff, 0x053ffc01, 0x000000ff, 0x073ffb01, 0x000000fe, 0x093efa01, 0x000000fe, 0x0c3cf901, 0x000000fe, 0x0e3bf901, 0x000000fd, 0x1138f801, 0x000001fd, 0x1337f801, 0x000001fc, 0x1635f801, 0x000001fb, 0x1932f801, 0x000001fb, 0x1b31f801, 0x000001fa, 0x1e2ef801, 0x000001fa, 0x212cf801, 0x000001f9, 0x2429f801, 0x000001f9, 0x2626f901, 0x000001f9, 0x2924f901, 0x000001f8, 0x2c21f901, 0x000001f8, 0x2e1efa01, 0x000001f8, 0x311bfa01, 0x000001f8, 0x3219fb01, 0x000001f8, 0x3516fb01, 0x000001f8, 0x3713fc01, 0x000001f8, 0x3811fd01, 0x000001f8, 0x3b0efd00, 0x000001f9, 0x3c0cfe00, 0x000001f9, 0x3e09fe00, 0x000001fa, 0x3f07fe00, 0x000001fb, 0x3f05ff00, 0x000001fc, 0x4003ff00, 0x000001fd, 0x40020000, 0x000000fe,
    0x2526fbfd, 0x0005fdfb, 0x2724fafe, 0x0005fcfc, 0x2823f9fe, 0x0005fcfd, 0x2921f9ff, 0x0005fbfe, 0x2920f8ff, 0x0006fbff, 0x2a1ef800, 0x0006fa00, 0x2b1df700, 0x0006fa01, 0x2b1cf701, 0x0006f902, 0x2c1af701, 0x0006f903, 0x2d19f602, 0x0006f804, 0x2d17f602, 0x0006f806, 0x2d15f603, 0x0006f807, 0x2f14f603, 0x0005f708, 0x2f12f604, 0x0005f709, 0x2f10f604, 0x0005f70b, 0x2f0ff604, 0x0005f60d, 0x2f0ef605, 0x0005f60d, 0x2f0df605, 0x0004f60f, 0x2f0bf705, 0x0004f610, 0x2f09f705, 0x0004f612, 0x2f08f705, 0x0003f614, 0x2d07f806, 0x0003f615, 0x2d06f806, 0x0002f617, 0x2d04f806, 0x0002f619, 0x2c03f906, 0x0001f71a, 0x2b02f906, 0x0001f71c, 0x2b01fa06, 0x0000f71d, 0x2a00fa06, 0x0000f81e, 0x29fffb06, 0x00fff820, 0x29fefb05, 0x00fff921, 0x28fdfc05, 0x00fef923, 0x27fcfc05, 0x00fefa24,
    0x1e1e0afa, 0x00fcfa0a, 0x1e1e09fa, 0x00fcfa0b, 0x1e1d09fa, 0x00fbfb0c, 0x1f1d08fa, 0x00fbfb0c, 0x201d07f9, 0x00fbfb0d, 0x201c07f9, 0x00fbfb0e, 0x201c06f9, 0x00fbfc0e, 0x211c05f9, 0x00fafc0f, 0x211b05f9, 0x00fafc10, 0x211a04f9, 0x00fafd11, 0x221a03f9, 0x00fafd11, 0x221803f9, 0x00fafe12, 0x221802f9, 0x00fafe13, 0x221802f9, 0x00f9ff13, 0x221701f9, 0x00f9ff15, 0x221701f9, 0x00f9ff15, 0x221600f9, 0x00f90016, 0x2215fff9, 0x00f90117, 0x2215fff9, 0x00f90117, 0x2213fff9, 0x00f90218, 0x2213fefa, 0x00f90218, 0x2212fefa, 0x00f90318, 0x2211fdfa, 0x00f9031a, 0x2111fdfa, 0x00f9041a, 0x2110fcfa, 0x00f9051b, 0x210ffcfa, 0x00f9051c, 0x200efcfb, 0x00f9061c, 0x200efbfb, 0x00f9071c, 0x200dfbfb, 0x00f9071d, 0x1f0cfbfb, 0x00fa081d, 0x1e0cfbfb, 0x00fa091d, 0x1e0bfafc, 0x00fa091e,
    0x0e0e0b06, 0x0002060b, 0x0e0d0b06, 0x0002070b, 0x0e0d0b06, 0x0002070b, 0x0e0d0b06, 0x0002070b, 0x0e0d0a06, 0x0002070c, 0x0e0d0a06, 0x0002070c, 0x0e0d0a05, 0x0003070c, 0x0e0d0a05, 0x0003070c, 0x0e0d0a05, 0x0003070c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0904, 0x0004080c, 0x0e0d0904, 0x0004080c, 0x0e0c0904, 0x0004090c, 0x0e0c0904, 0x0004090c, 0x0e0c0904, 0x0004090c, 0x0e0c0804, 0x0004090d, 0x0e0c0804, 0x0004090d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0703, 0x00050a0d, 0x0e0c0703, 0x00050a0d, 0x0e0c0703, 0x00050a0d, 0x0e0c0702, 0x00060a0d, 0x0e0c0702, 0x00060a0d, 0x0e0b0702, 0x00060b0d, 0x0e0b0702, 0x00060b0d, 0x0e0b0702, 0x00060b0d};

static uint16_t _calibration_sharpen_ds1[][2] = {
    {0 * 256, 70},
    {1 * 256, 70},
    {2 * 256, 70},
    {3 * 256, 70},
    {4 * 256, 70},
    {5 * 256, 50},
    {6 * 256, 40},
    {7 * 256, 25},
    {8 * 256, 10}};

static uint16_t _calibration_temper_strength[][2] = {
    {0 * 256, 115},
    {1 * 256, 120},
    {2 * 256, 130},
    {3 * 256, 145},
    {4 * 256, 155},
    {5 * 256, 160},
    {6 * 256, 160},
    {7 * 256, 160}};

static uint32_t _calibration_af_lms[] = {
    70 << 6,  // Down_FarEnd
    70 << 6,  // Hor_FarEnd
    70 << 6,  // Up_FarEnd
    112 << 6, // Down_Infinity
    112 << 6, // Hor_Infinity
    112 << 6, // Up_Infinity
    832 << 6,                      // Down_Macro
    832 << 6,                      // Hor_Macro
    832 << 6,                      // Up_Macro
    915 << 6,                      // Down_NearEnd
    915 << 6,                      // Hor_NearEnd
    915 << 6,                      // Up_NearEnd
    11,                            // step_num
    6,                             // skip_frames_init
    2,                             // skip_frames_move
    30,                            // dynamic_range_th
    2 << ( LOG2_GAIN_SHIFT - 2 ),  // spot_tolerance
    1 << ( LOG2_GAIN_SHIFT - 1 ),  // exit_th
    16 << ( LOG2_GAIN_SHIFT - 4 ), // caf_trigger_th
    4 << ( LOG2_GAIN_SHIFT - 4 ),  // caf_stable_th
    0,                             // print_debug
};

static uint16_t _calibration_af_zone_wght_hor[] = {0, 0, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0};

static uint16_t _calibration_af_zone_wght_ver[] = {0, 0, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0};


static int32_t _calibration_gamma_threshold[] = {0, 3357408, 3680000};

// CALIBRATION_GAMMA_EV1
static uint16_t _calibration_gamma_ev1[] =
    /*sRGB highcontrast{0, 150, 261, 359, 452, 541, 623, 702, 781, 859, 937, 1014, 1087, 1158, 1224, 1288, 1348, 1407, 1464, 1519, 1572, 1625, 1676, 1727, 1775, 1823, 1869, 1913, 1956, 1999, 2041, 2082, 2123, 2162, 2201, 2238, 2276, 2312, 2348, 2383, 2417, 2451, 2485, 2516, 2549, 2580, 2611, 2641, 2671, 2701, 2730, 2759, 2787, 2816, 2843, 2871, 2897, 2923, 2950, 2975, 3000, 3025, 3048, 3071, 3095, 3118, 3139, 3161, 3182, 3204, 3224, 3244, 3263, 3283, 3302, 3322, 3340, 3358, 3377, 3394, 3412, 3429, 3447, 3464, 3481, 3497, 3514, 3530, 3546, 3562, 3579, 3594, 3610, 3625, 3641, 3656, 3671, 3686, 3701, 3716, 3731, 3745, 3759, 3774, 3788, 3802, 3816, 3830, 3843, 3857, 3871, 3884, 3898, 3911, 3924, 3936, 3949, 3962, 3974, 3987, 4000, 4011, 4024, 4036, 4048, 4060, 4072, 4083, 4095}; */
    /*sRGB 65{0,192,318,419,511,596,675,749,820,887,950,1012,1070,1126,1180,1231,1282,1332,1380,1428,1475,1521,1568,1614,1660,1706,1751,1796,1842,1890,1938,1988,2037,2085,2133,2180,2228,2273,2319,2363,2406,2447,2489,2528,2566,2603,2638,2671,2703,2734,2762,2790,2818,2845,2871,2897,2921,2946,2970,2993,3016,3038,3060,3081,3103,3123,3143,3163,3183,3203,3222,3241,3259,3278,3296,3315,3333,3351,3369,3386,3403,3420,3438,3455,3472,3489,3506,3522,3539,3555,3572,3588,3604,3620,3635,3651,3666,3681,3696,3712,3726,3741,3755,3770,3784,3798,3813,3827,3840,3854,3868,3881,3895,3908,3921,3934,3947,3960,3972,3985,3998,4010,4023,4035,4048,4060,4071,4083,4095}; */
    {0,101,217,347,537,738,925,1087,1222,1323,1407,1482,1550,1612,1670,1723,1773,1822,1867,1910,1952,1993,2033,2071,2110,2147,2184,2220,2255,2289,2322,2355,2387,2418,2448,2478,2507,2535,2564,2591,2618,2644,2671,2696,2722,2747,2772,2796,2821,2845,2868,2891,2914,2938,2960,2984,3005,3028,3051,3073,3094,3116,3137,3158,3180,3201,3220,3241,3261,3281,3301,3320,3339,3358,3376,3395,3413,3431,3449,3465,3483,3499,3516,3532,3548,3564,3579,3594,3609,3624,3639,3653,3668,3682,3696,3710,3723,3737,3750,3764,3777,3790,3803,3816,3828,3841,3854,3866,3878,3890,3902,3913,3925,3937,3948,3959,3970,3982,3992,4003,4014,4024,4035,4045,4056,4066,4075,4085,4095};


// CALIBRATION_GAMMA_EV2
static uint16_t _calibration_gamma_ev2[] =
    /*sRGB highcontrast{0, 150, 261, 359, 452, 541, 623, 702, 781, 859, 937, 1014, 1087, 1158, 1224, 1288, 1348, 1407, 1464, 1519, 1572, 1625, 1676, 1727, 1775, 1823, 1869, 1913, 1956, 1999, 2041, 2082, 2123, 2162, 2201, 2238, 2276, 2312, 2348, 2383, 2417, 2451, 2485, 2516, 2549, 2580, 2611, 2641, 2671, 2701, 2730, 2759, 2787, 2816, 2843, 2871, 2897, 2923, 2950, 2975, 3000, 3025, 3048, 3071, 3095, 3118, 3139, 3161, 3182, 3204, 3224, 3244, 3263, 3283, 3302, 3322, 3340, 3358, 3377, 3394, 3412, 3429, 3447, 3464, 3481, 3497, 3514, 3530, 3546, 3562, 3579, 3594, 3610, 3625, 3641, 3656, 3671, 3686, 3701, 3716, 3731, 3745, 3759, 3774, 3788, 3802, 3816, 3830, 3843, 3857, 3871, 3884, 3898, 3911, 3924, 3936, 3949, 3962, 3974, 3987, 4000, 4011, 4024, 4036, 4048, 4060, 4072, 4083, 4095}; */
    /*sRGB 65{0,192,318,419,511,596,675,749,820,887,950,1012,1070,1126,1180,1231,1282,1332,1380,1428,1475,1521,1568,1614,1660,1706,1751,1796,1842,1890,1938,1988,2037,2085,2133,2180,2228,2273,2319,2363,2406,2447,2489,2528,2566,2603,2638,2671,2703,2734,2762,2790,2818,2845,2871,2897,2921,2946,2970,2993,3016,3038,3060,3081,3103,3123,3143,3163,3183,3203,3222,3241,3259,3278,3296,3315,3333,3351,3369,3386,3403,3420,3438,3455,3472,3489,3506,3522,3539,3555,3572,3588,3604,3620,3635,3651,3666,3681,3696,3712,3726,3741,3755,3770,3784,3798,3813,3827,3840,3854,3868,3881,3895,3908,3921,3934,3947,3960,3972,3985,3998,4010,4023,4035,4048,4060,4071,4083,4095}; */
    //sRGB {0, 347, 539, 679, 794, 894, 982, 1062, 1136, 1204, 1268, 1329, 1386, 1441, 1493, 1543, 1591, 1638, 1683, 1726, 1768, 1809, 1849, 1888, 1926, 1963, 1999, 2034, 2068, 2102, 2135, 2168, 2200, 2231, 2262, 2292, 2322, 2351, 2380, 2408, 2436, 2463, 2491, 2517, 2544, 2570, 2596, 2621, 2646, 2671, 2695, 2719, 2743, 2767, 2790, 2814, 2836, 2859, 2882, 2904, 2926, 2948, 2969, 2990, 3012, 3033, 3053, 3074, 3094, 3115, 3135, 3155, 3174, 3194, 3213, 3233, 3252, 3271, 3290, 3308, 3327, 3345, 3364, 3382, 3400, 3418, 3436, 3453, 3471, 3488, 3506, 3523, 3540, 3557, 3574, 3591, 3607, 3624, 3640, 3657, 3673, 3689, 3705, 3721, 3737, 3753, 3769, 3785, 3800, 3816, 3831, 3846, 3862, 3877, 3892, 3907, 3922, 3937, 3951, 3966, 3981, 3995, 4010, 4024, 4039, 4053, 4067, 4081, 4095};
    {0,137,273,435,588,714,820,917,1007,1089,1166,1239,1308,1374,1437,1497,1554,1610,1664,1715,1764,1812,1858,1903,1947,1990,2031,2072,2111,2149,2187,2224,2261,2295,2330,2364,2398,2430,2462,2493,2524,2554,2585,2614,2643,2671,2700,2727,2753,2780,2806,2832,2857,2883,2907,2933,2956,2981,3005,3028,3052,3076,3098,3121,3146,3169,3191,3214,3236,3260,3282,3304,3324,3346,3367,3388,3408,3428,3448,3467,3486,3504,3523,3540,3557,3574,3590,3605,3621,3636,3652,3667,3682,3696,3711,3725,3738,3753,3766,3780,3793,3806,3819,3831,3844,3857,3869,3881,3893,3905,3916,3927,3939,3950,3961,3971,3982,3992,4002,4012,4022,4032,4041,4051,4060,4069,4078,4086,4095};

static uint32_t _calibration_custom_settings_context[][4] = {
    //CNR
    { 0x1b0f8, 0x0L, 0xfff,2 }, //uv_seg1_threshold
    { 0x1b0fc, 0x1c2L, 0xfff,2 }, //uv_seg1_offset
    { 0x1b100, 0xee48L, 0x0,2 }, //uv_seg1_slope
    //{ 0x1b14c, 0x1010L, 0x3f3f,2 }, //uv_var1_scale, uv_var2_scale
    //demosaic
    { 0x1ae7c, 0x91b4b4c8L, 0xffffffff,4 },// UU/VA/AA/VH slope
    { 0x1ae7c, 0xb4c3d2dcL, 0xffffffff,4 }, // UU Slope/VA Slope/AA Slope/VH Slope
    { 0x1ae84, 0x00f00c8L, 0xfff0fff,4 }, //AA Thresh/VH Thresh
    { 0x1ae88, 0x0000040L, 0xfff0fff,4 }, //UU Thresh/VA Thresh
    { 0x1aec0, 0x808c8L, 0xffffff,4 }, //grey det thresh/lg det thresh/UU SH Slope
    { 0x1aec8, 0x2af80708L, 0xffffffff,4 }, //grey det slope/lg det slope
    { 0x1aeb0, 0x800f6432, 0x0,4 }, //sad_amp/sharp_alt_lu/sharp_alt_ldu/sharp_alt_ld
    { 0x19368, 0x1e03L, 0xff07,2 }, //0x19368[0]: use LUT, 1=use LUT data, 0 = use exposure mask provided by Frame stitching or threshold
    //stop sequence - address is 0x0000
    {0x0000, 0x0000, 0x0000, 0x0000}};

static uint32_t _calibration_defog_control[] = {
    0, //defog enable
    3, //defog detect mode
    600000, //acc fog value threshhold
    80, //hist fog idx threshhold
    10, //hist fog pec threshhold_1
    20, //hist fog pec threshhold_2
    1024, //ratio delta
    950, //max rng
    50, //min rng
    10, //black percentage
    995, //white percentage
    15, //avg_coeff
    0, //reserved
    0, //reserved
    0, //reserved
    0, //reserved
};

// { GAIN_LOG2_ID*256, lum_thresh, sad_amp, uu_sh_slope, uu_sh_thresh, luma_thresh_low_d,
// luma_thresh_low_ud, luma_slope_low_d, luma_slope_low_ud, luma_thresh_high_d,
// luma_thresh_high_ud, luma_slope_high_d, luma_slope_high_ud }
static uint32_t _calibration_demosaic_rgb_extension_control[][13] = {
    {0 * 256, 150, 128, 200, 150, 75, 100, 10000, 10000, 4000, 4000, 8000, 8000},
    {1 * 256, 150, 128, 200, 150, 75, 100, 10000, 10000, 4000, 4000, 8000, 8000},
    {2 * 256, 150, 128, 200, 150, 75, 100, 10000, 10000, 4000, 4000, 8000, 8000},
    {3 * 256, 150, 128, 200, 150, 75, 100, 10000, 10000, 4000, 4000, 8000, 8000},
    {4 * 256, 150, 128, 200, 150, 75, 100, 10000, 10000, 4000, 4000, 8000, 8000},
    {5 * 256, 150, 128, 200, 150, 75, 100, 10000, 10000, 4000, 4000, 8000, 8000},
    {6 * 256, 150, 128, 200, 150, 75, 100, 10000, 10000, 4000, 4000, 8000, 8000},
    {7 * 256, 150, 128, 200, 150, 75, 100, 10000, 10000, 4000, 4000, 8000, 8000},
};

// { GAIN_LOG2_ID*256, alpha_undershoot, luma_thresh_low, luma_slope_low, luma_thresh_high, luma_slope_high }
static uint32_t _calibration_fr_sharpen_extension_control[][6] = {
    {0 * 256, 10, 300, 1000, 1000, 1700},
    {1 * 256, 10, 300, 1000, 1000, 1700},
    {2 * 256, 10, 300, 1000, 1000, 1700},
    {3 * 256, 10, 300, 1000, 1000, 1700},
    {4 * 256, 10, 300, 1000, 1000, 1700},
    {5 * 256, 10, 300, 1000, 1000, 1700},
    {6 * 256, 10, 300, 1000, 1000, 1700},
    {7 * 256, 10, 300, 1000, 1000, 1700},
};

// { GAIN_LOG2_ID*256, alpha_undershoot, luma_thresh_low, luma_slope_low, luma_thresh_high, luma_slope_high }
static uint32_t _calibration_ds_sharpen_extension_control[][6] = {
    {0 * 256, 19, 0, 1020, 818, 1700},
    {1 * 256, 19, 0, 1020, 818, 1700},
    {2 * 256, 19, 0, 1020, 818, 1700},
    {3 * 256, 19, 0, 1020, 818, 1700},
    {4 * 256, 19, 0, 1020, 818, 1700},
    {5 * 256, 19, 0, 1020, 818, 1700},
    {6 * 256, 19, 0, 1020, 818, 1700},
    {7 * 256, 19, 0, 1020, 818, 1700},
};

// { GAIN_LOG2_ID*256, delta_factor, umean1_thd, umean1_offset, umean1_slope, umean2_thd, umean2_offset, umean2_slope,
// vmean1_thd, vmean1_offset, vmean1_slope, vmean2_thd, vmean2_offset, vmean2_slope, uv_delta1_thd, uv_delta1_offset,
// uv_delta1_slope, uv_delta2_thd, uv_delta2_offset, uv_delta2_slope }
static uint32_t _calibration_cnr_extension_control[][20] = {
    {0 * 256, 150, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 100, 10000, 0, 100, 10000},
    {1 * 256, 150, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 100, 10000, 0, 100, 15000},
    {2 * 256, 150, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 100, 10000, 0, 100, 20000},
    {3 * 256, 150, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 100, 10000, 0, 100, 30000},
    {4 * 256, 150, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 100, 10000, 0, 100, 40000},
    {5 * 256, 150, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 100, 10000, 0, 100, 60000},
    {6 * 256, 150, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 100, 10000, 0, 100, 65000},
    {7 * 256, 150, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 200, 60500, 0, 100, 10000, 0, 100, 65000},
};

// { GAIN_LOG2_ID*256, svariance, bright_pr, contrast, white_level }
static uint32_t _calibration_iridix_extension_control[][5] = {
    {0 * 256, 10, 220, 180, 809600},
    {1 * 256, 10, 220, 180, 809600},
    {2 * 256, 10, 220, 180, 809600},
    {3 * 256, 10, 220, 180, 809600},
    {4 * 256, 10, 220, 180, 809600},
    {5 * 256, 10, 220, 180, 809600},
    {6 * 256, 10, 220, 180, 809600},
    {7 * 256, 10, 220, 180, 809600},
};

// { GAIN_LOG2_ID*256, black_level_in, black_level_out }
static uint32_t _calibration_sqrt_extension_control[][3] = {
    {0 * 256, 17152, 1072},
    {1 * 256, 17152, 1072},
    {2 * 256, 17152, 1072},
    {3 * 256, 17152, 1072},
    {4 * 256, 17152, 1072},
    {5 * 256, 17152, 1072},
    {6 * 256, 17152, 1072},
    {7 * 256, 17152, 1072},
};

// { GAIN_LOG2_ID*256, black_level_in, black_level_out }
static uint32_t _calibration_square_be_extension_control[][3] = {
    {0 * 256, 1072, 17152},
    {1 * 256, 1072, 17152},
    {2 * 256, 1072, 17152},
    {3 * 256, 1072, 17152},
    {4 * 256, 1072, 17152},
    {5 * 256, 1072, 17152},
    {6 * 256, 1072, 17152},
    {7 * 256, 1072, 17152},
};

// { GAIN_LOG2_ID*256, dpdev threshold }
static uint16_t _calibration_dp_devthreshold[][2] = {
    {0 * 256, 32768},
    {1 * 256, 32768},
    {2 * 256, 32768},
    {3 * 256, 32768},
    {4 * 256, 32768},
    {5 * 256, 32768},
    {6 * 256, 32768},
    {7 * 256, 32768},
};

// { GAIN_LOG2_ID*256, hue strength,luma strength,sat strength,saturation strength,purple strength }
static uint16_t _calibration_pf_correction[][6] = {
    {0 * 256, 1024, 1024, 100, 10, 2048},
    {1 * 256, 1024, 1024, 100, 10, 2048},
    {2 * 256, 1024, 1024, 100, 10, 2048},
    {3 * 256, 1024, 1024, 100, 10, 2048},
    {4 * 256, 1024, 1024, 100, 10, 2048},
    {5 * 256, 1024, 1024, 100, 10, 2048},
    {6 * 256, 1024, 1024, 100, 10, 2048},
    {7 * 256, 1024, 1024, 100, 10, 2048},
};

// { GAIN_LOG2_ID*256, fc slope, alias slop, alias thresh }
static uint16_t _calibration_fc_correction[][4] = {
    {0 * 256, 150, 85, 0},
    {1 * 256, 150, 85, 0},
    {2 * 256, 150, 85, 0},
    {3 * 256, 150, 85, 0},
    {4 * 256, 150, 85, 0},
    {5 * 256, 150, 85, 0},
    {6 * 256, 150, 85, 0},
    {7 * 256, 150, 85, 0},
};

static LookupTable calibration_gamma_threshold = {.ptr = _calibration_gamma_threshold, .rows = 1, .cols = sizeof( _calibration_gamma_threshold ) / sizeof( _calibration_gamma_threshold[0] ), .width = sizeof( _calibration_gamma_threshold[0] )};
static LookupTable calibration_gamma_ev1 = {.ptr = _calibration_gamma_ev1, .rows = 1, .cols = sizeof( _calibration_gamma_ev1 ) / sizeof( _calibration_gamma_ev1[0] ), .width = sizeof( _calibration_gamma_ev1[0] )};
static LookupTable calibration_gamma_ev2 = {.ptr = _calibration_gamma_ev2, .rows = 1, .cols = sizeof( _calibration_gamma_ev2 ) / sizeof( _calibration_gamma_ev2[0] ), .width = sizeof( _calibration_gamma_ev2[0] )};

static LookupTable calibration_fs_mc_off = {.ptr = _calibration_fs_mc_off, .rows = 1, .cols = sizeof( _calibration_fs_mc_off ) / sizeof( _calibration_fs_mc_off[0] ), .width = sizeof( _calibration_fs_mc_off[0] )};
static LookupTable calibration_exposure_ratio_adjustment = {.ptr = _calibration_exposure_ratio_adjustment, .rows = sizeof( _calibration_exposure_ratio_adjustment ) / sizeof( _calibration_exposure_ratio_adjustment[0] ), .cols = 2, .width = sizeof( _calibration_exposure_ratio_adjustment[0][0] )};
static LookupTable AWB_colour_preference = {.ptr = _AWB_colour_preference, .rows = 1, .cols = sizeof( _AWB_colour_preference ) / sizeof( _AWB_colour_preference[0] ), .width = sizeof( _AWB_colour_preference[0] )};
static LookupTable calibration_awb_mix_light_parameters = {.ptr = _calibration_awb_mix_light_parameters, .rows = 1, .cols = sizeof( _calibration_awb_mix_light_parameters ) / sizeof( _calibration_awb_mix_light_parameters[0] ), .width = sizeof( _calibration_awb_mix_light_parameters[0] )};
static LookupTable calibration_sinter_strength_MC_contrast = {.ptr = _calibration_sinter_strength_MC_contrast, .rows = sizeof( _calibration_sinter_strength_MC_contrast ) / sizeof( _calibration_sinter_strength_MC_contrast[0] ), .cols = 2, .width = sizeof( _calibration_sinter_strength_MC_contrast[0][0] )};
static LookupTable calibration_pf_radial_lut = {.ptr = _calibration_pf_radial_lut, .rows = 1, .cols = sizeof( _calibration_pf_radial_lut ) / sizeof( _calibration_pf_radial_lut[0] ), .width = sizeof( _calibration_pf_radial_lut[0] )};
static LookupTable calibration_pf_radial_params = {.ptr = _calibration_pf_radial_params, .rows = 1, .cols = sizeof( _calibration_pf_radial_params ) / sizeof( _calibration_pf_radial_params[0] ), .width = sizeof( _calibration_pf_radial_params[0] )};
static LookupTable calibration_sinter_radial_lut = {.ptr = _calibration_sinter_radial_lut, .rows = 1, .cols = sizeof( _calibration_sinter_radial_lut ) / sizeof( _calibration_sinter_radial_lut[0] ), .width = sizeof( _calibration_sinter_radial_lut[0] )};
static LookupTable calibration_sinter_radial_params = {.ptr = _calibration_sinter_radial_params, .rows = 1, .cols = sizeof( _calibration_sinter_radial_params ) / sizeof( _calibration_sinter_radial_params[0] ), .width = sizeof( _calibration_sinter_radial_params[0] )};
static LookupTable calibration_AWB_bg_max_gain = {.ptr = _calibration_AWB_bg_max_gain, .rows = sizeof( _calibration_AWB_bg_max_gain ) / sizeof( _calibration_AWB_bg_max_gain[0] ), .cols = 2, .width = sizeof( _calibration_AWB_bg_max_gain[0][0] )};
static LookupTable calibration_iridix8_strength_dk_enh_control = {.ptr = _calibration_iridix8_strength_dk_enh_control, .rows = 1, .cols = sizeof( _calibration_iridix8_strength_dk_enh_control ) / sizeof( _calibration_iridix8_strength_dk_enh_control[0] ), .width = sizeof( _calibration_iridix8_strength_dk_enh_control[0] )};
static LookupTable calibration_auto_level_control = {.ptr = _calibration_auto_level_control, .rows = 1, .cols = sizeof( _calibration_auto_level_control ) / sizeof( _calibration_auto_level_control[0] ), .width = sizeof( _calibration_auto_level_control[0] )};
static LookupTable calibration_dp_threshold = {.ptr = _calibration_dp_threshold, .rows = sizeof( _calibration_dp_threshold ) / sizeof( _calibration_dp_threshold[0] ), .cols = 2, .width = sizeof( _calibration_dp_threshold[0][0] )};
static LookupTable calibration_stitching_lm_np = {.ptr = _calibration_stitching_lm_np, .rows = sizeof( _calibration_stitching_lm_np ) / sizeof( _calibration_stitching_lm_np[0] ), .cols = 2, .width = sizeof( _calibration_stitching_lm_np[0][0] )};
static LookupTable calibration_stitching_lm_med_noise_intensity_thresh = {.ptr = _calibration_stitching_lm_med_noise_intensity_thresh, .rows = sizeof( _calibration_stitching_lm_med_noise_intensity_thresh ) / sizeof( _calibration_stitching_lm_med_noise_intensity_thresh[0] ), .cols = 2, .width = sizeof( _calibration_stitching_lm_med_noise_intensity_thresh[0][0] )};
static LookupTable calibration_stitching_lm_mov_mult = {.ptr = _calibration_stitching_lm_mov_mult, .rows = sizeof( _calibration_stitching_lm_mov_mult ) / sizeof( _calibration_stitching_lm_mov_mult[0] ), .cols = 2, .width = sizeof( _calibration_stitching_lm_mov_mult[0][0] )};
static LookupTable calibration_stitching_ms_np = {.ptr = _calibration_stitching_ms_np, .rows = sizeof( _calibration_stitching_ms_np ) / sizeof( _calibration_stitching_ms_np[0] ), .cols = 2, .width = sizeof( _calibration_stitching_ms_np[0][0] )};
static LookupTable calibration_stitching_ms_mov_mult = {.ptr = _calibration_stitching_ms_mov_mult, .rows = sizeof( _calibration_stitching_ms_mov_mult ) / sizeof( _calibration_stitching_ms_mov_mult[0] ), .cols = 2, .width = sizeof( _calibration_stitching_ms_mov_mult[0][0] )};
static LookupTable calibration_evtolux_probability_enable = {.ptr = _calibration_evtolux_probability_enable, .rows = 1, .cols = sizeof( _calibration_evtolux_probability_enable ) / sizeof( _calibration_evtolux_probability_enable[0] ), .width = sizeof( _calibration_evtolux_probability_enable[0] )};
static LookupTable calibration_awb_avg_coef = {.ptr = _calibration_awb_avg_coef, .rows = 1, .cols = sizeof( _calibration_awb_avg_coef ) / sizeof( _calibration_awb_avg_coef[0] ), .width = sizeof( _calibration_awb_avg_coef[0] )};
static LookupTable calibration_iridix_avg_coef = {.ptr = _calibration_iridix_avg_coef, .rows = 1, .cols = sizeof( _calibration_iridix_avg_coef ) / sizeof( _calibration_iridix_avg_coef[0] ), .width = sizeof( _calibration_iridix_avg_coef[0] )};
static LookupTable calibration_iridix_strength_maximum = {.ptr = _calibration_iridix_strength_maximum, .rows = 1, .cols = sizeof( _calibration_iridix_strength_maximum ) / sizeof( _calibration_iridix_strength_maximum[0] ), .width = sizeof( _calibration_iridix_strength_maximum[0] )};
static LookupTable calibration_iridix_min_max_str = {.ptr = _calibration_iridix_min_max_str, .rows = 1, .cols = sizeof( _calibration_iridix_min_max_str ) / sizeof( _calibration_iridix_min_max_str[0] ), .width = sizeof( _calibration_iridix_min_max_str[0] )};
static LookupTable calibration_iridix_ev_lim_full_str = {.ptr = _calibration_iridix_ev_lim_full_str, .rows = 1, .cols = sizeof( _calibration_iridix_ev_lim_full_str ) / sizeof( _calibration_iridix_ev_lim_full_str[0] ), .width = sizeof( _calibration_iridix_ev_lim_full_str[0] )};
static LookupTable calibration_iridix_ev_lim_no_str = {.ptr = _calibration_iridix_ev_lim_no_str, .rows = 1, .cols = sizeof( _calibration_iridix_ev_lim_no_str ) / sizeof( _calibration_iridix_ev_lim_no_str[0] ), .width = sizeof( _calibration_iridix_ev_lim_no_str[0] )};
static LookupTable calibration_ae_correction = {.ptr = _calibration_ae_correction, .rows = 1, .cols = sizeof( _calibration_ae_correction ) / sizeof( _calibration_ae_correction[0] ), .width = sizeof( _calibration_ae_correction[0] )};
static LookupTable calibration_ae_exposure_correction = {.ptr = _calibration_ae_exposure_correction, .rows = 1, .cols = sizeof( _calibration_ae_exposure_correction ) / sizeof( _calibration_ae_exposure_correction[0] ), .width = sizeof( _calibration_ae_exposure_correction[0] )};
static LookupTable calibration_sinter_strength = {.ptr = _calibration_sinter_strength, .rows = sizeof( _calibration_sinter_strength ) / sizeof( _calibration_sinter_strength[0] ), .cols = 2, .width = sizeof( _calibration_sinter_strength[0][0] )};
static LookupTable calibration_sinter_strength1 = {.ptr = _calibration_sinter_strength1, .rows = sizeof( _calibration_sinter_strength1 ) / sizeof( _calibration_sinter_strength1[0] ), .cols = 2, .width = sizeof( _calibration_sinter_strength1[0][0] )};
static LookupTable calibration_sinter_thresh1 = {.ptr = _calibration_sinter_thresh1, .rows = sizeof( _calibration_sinter_thresh1 ) / sizeof( _calibration_sinter_thresh1[0] ), .cols = 2, .width = sizeof( _calibration_sinter_thresh1[0][0] )};
static LookupTable calibration_sinter_thresh4 = {.ptr = _calibration_sinter_thresh4, .rows = sizeof( _calibration_sinter_thresh4 ) / sizeof( _calibration_sinter_thresh4[0] ), .cols = 2, .width = sizeof( _calibration_sinter_thresh4[0][0] )};
static LookupTable calibration_sinter_intConfig = {.ptr = _calibration_sinter_intConfig, .rows = sizeof( _calibration_sinter_intConfig ) / sizeof( _calibration_sinter_intConfig[0] ), .cols = 2, .width = sizeof( _calibration_sinter_intConfig[0][0] )};
static LookupTable calibration_sharp_alt_d = {.ptr = _calibration_sharp_alt_d, .rows = sizeof( _calibration_sharp_alt_d ) / sizeof( _calibration_sharp_alt_d[0] ), .cols = 2, .width = sizeof( _calibration_sharp_alt_d[0][0] )};
static LookupTable calibration_sharp_alt_ud = {.ptr = _calibration_sharp_alt_ud, .rows = sizeof( _calibration_sharp_alt_ud ) / sizeof( _calibration_sharp_alt_ud[0] ), .cols = 2, .width = sizeof( _calibration_sharp_alt_ud[0][0] )};
static LookupTable calibration_sharp_alt_du = {.ptr = _calibration_sharp_alt_du, .rows = sizeof( _calibration_sharp_alt_du ) / sizeof( _calibration_sharp_alt_du[0] ), .cols = 2, .width = sizeof( _calibration_sharp_alt_du[0][0] )};
static LookupTable calibration_sharpen_fr = {.ptr = _calibration_sharpen_fr, .rows = sizeof( _calibration_sharpen_fr ) / sizeof( _calibration_sharpen_fr[0] ), .cols = 2, .width = sizeof( _calibration_sharpen_fr[0][0] )};
static LookupTable calibration_demosaic_np_offset = {.ptr = _calibration_demosaic_np_offset, .rows = sizeof( _calibration_demosaic_np_offset ) / sizeof( _calibration_demosaic_np_offset[0] ), .cols = 2, .width = sizeof( _calibration_demosaic_np_offset[0][0] )};
static LookupTable calibration_mesh_shading_strength = {.ptr = _calibration_mesh_shading_strength, .rows = sizeof( _calibration_mesh_shading_strength ) / sizeof( _calibration_mesh_shading_strength[0] ), .cols = 2, .width = sizeof( _calibration_mesh_shading_strength[0][0] )};
static LookupTable calibration_saturation_strength = {.ptr = _calibration_saturation_strength, .rows = sizeof( _calibration_saturation_strength ) / sizeof( _calibration_saturation_strength[0] ), .cols = 2, .width = sizeof( _calibration_saturation_strength[0][0] )};
static LookupTable calibration_ccm_one_gain_threshold = {.ptr = _calibration_ccm_one_gain_threshold, .cols = sizeof( _calibration_ccm_one_gain_threshold ) / sizeof( _calibration_ccm_one_gain_threshold[0] ), .rows = 1, .width = sizeof( _calibration_ccm_one_gain_threshold[0] )};
static LookupTable calibration_cmos_exposure_partition_luts = {.ptr = _calibration_cmos_exposure_partition_luts, .rows = sizeof( _calibration_cmos_exposure_partition_luts ) / sizeof( _calibration_cmos_exposure_partition_luts[0] ), .cols = 10, .width = sizeof( _calibration_cmos_exposure_partition_luts[0][0] )};
static LookupTable calibration_cmos_control = {.ptr = _calibration_cmos_control, .rows = 1, .cols = sizeof( _calibration_cmos_control ) / sizeof( _calibration_cmos_control[0] ), .width = sizeof( _calibration_cmos_control[0] )};
static LookupTable calibration_status_info = {.ptr = _calibration_status_info, .rows = 1, .cols = sizeof( _calibration_status_info ) / sizeof( _calibration_status_info[0] ), .width = sizeof( _calibration_status_info[0] )};
static LookupTable calibration_ae_control = {.ptr = _calibration_ae_control, .rows = 1, .cols = sizeof( _calibration_ae_control ) / sizeof( _calibration_ae_control[0] ), .width = sizeof( _calibration_ae_control[0] )};
static LookupTable calibration_ae_control_HDR_target = {.ptr = _calibration_ae_control_HDR_target, .rows = sizeof( _calibration_ae_control_HDR_target ) / sizeof( _calibration_ae_control_HDR_target[0] ), .cols = 2, .width = sizeof( _calibration_ae_control_HDR_target[0][0] )};
static LookupTable calibration_rgb2yuv_conversion = {.ptr = _calibration_rgb2yuv_conversion, .rows = 1, .cols = sizeof( _calibration_rgb2yuv_conversion ) / sizeof( _calibration_rgb2yuv_conversion[0] ), .width = sizeof( _calibration_rgb2yuv_conversion[0] )};
static LookupTable calibration_calibration_af_lms = {.ptr = _calibration_af_lms, .rows = 1, .cols = sizeof( _calibration_af_lms ) / sizeof( _calibration_af_lms[0] ), .width = sizeof( _calibration_af_lms[0] )};
static LookupTable calibration_calibration_af_zone_wght_hor = {.ptr = _calibration_af_zone_wght_hor, .rows = 1, .cols = sizeof( _calibration_af_zone_wght_hor ) / sizeof( _calibration_af_zone_wght_hor[0] ), .width = sizeof( _calibration_af_zone_wght_hor[0] )};
static LookupTable calibration_calibration_af_zone_wght_ver = {.ptr = _calibration_af_zone_wght_ver, .rows = 1, .cols = sizeof( _calibration_af_zone_wght_ver ) / sizeof( _calibration_af_zone_wght_ver[0] ), .width = sizeof( _calibration_af_zone_wght_ver[0] )};
static LookupTable calibration_calibration_ae_zone_wght_hor = {.ptr = _calibration_ae_zone_wght_hor, .rows = 1, .cols = sizeof( _calibration_ae_zone_wght_hor ) / sizeof( _calibration_ae_zone_wght_hor[0] ), .width = sizeof( _calibration_ae_zone_wght_hor[0] )};
static LookupTable calibration_calibration_ae_zone_wght_ver = {.ptr = _calibration_ae_zone_wght_ver, .rows = 1, .cols = sizeof( _calibration_ae_zone_wght_ver ) / sizeof( _calibration_ae_zone_wght_ver[0] ), .width = sizeof( _calibration_ae_zone_wght_ver[0] )};
static LookupTable calibration_calibration_awb_zone_wght_hor = {.ptr = _calibration_awb_zone_wght_hor, .rows = 1, .cols = sizeof( _calibration_awb_zone_wght_hor ) / sizeof( _calibration_awb_zone_wght_hor[0] ), .width = sizeof( _calibration_awb_zone_wght_hor[0] )};
static LookupTable calibration_calibration_awb_zone_wght_ver = {.ptr = _calibration_awb_zone_wght_ver, .rows = 1, .cols = sizeof( _calibration_awb_zone_wght_ver ) / sizeof( _calibration_awb_zone_wght_ver[0] ), .width = sizeof( _calibration_awb_zone_wght_ver[0] )};
static LookupTable calibration_dp_slope = {.ptr = _calibration_dp_slope, .rows = sizeof( _calibration_dp_slope ) / sizeof( _calibration_dp_slope[0] ), .cols = 2, .width = sizeof( _calibration_dp_slope[0][0] )};
static LookupTable calibration_cnr_uv_delta12_slope = {.ptr = _calibration_cnr_uv_delta12_slope, .rows = sizeof( _calibration_cnr_uv_delta12_slope ) / sizeof( _calibration_cnr_uv_delta12_slope[0] ), .cols = 2, .width = sizeof( _calibration_cnr_uv_delta12_slope[0][0] )};
static LookupTable calibration_sinter_sad = {.ptr = _calibration_sinter_sad, .rows = sizeof( _calibration_sinter_sad ) / sizeof( _calibration_sinter_sad[0] ), .cols = 2, .width = sizeof( _calibration_sinter_sad[0][0] )};
static LookupTable calibration_scaler_h_filter = {.ptr = _scaler_h_filter, .rows = 1, .cols = sizeof( _scaler_h_filter ) / sizeof( _scaler_h_filter[0] ), .width = sizeof( _scaler_h_filter[0] )};
static LookupTable calibration_scaler_v_filter = {.ptr = _scaler_v_filter, .rows = 1, .cols = sizeof( _scaler_v_filter ) / sizeof( _scaler_v_filter[0] ), .width = sizeof( _scaler_v_filter[0] )};
static LookupTable calibration_sharpen_ds1 = {.ptr = _calibration_sharpen_ds1, .rows = sizeof( _calibration_sharpen_ds1 ) / sizeof( _calibration_sharpen_ds1[0] ), .cols = 2, .width = sizeof( _calibration_sharpen_ds1[0][0] )};
static LookupTable calibration_temper_strength = {.ptr = _calibration_temper_strength, .rows = sizeof( _calibration_temper_strength ) / sizeof( _calibration_temper_strength[0] ), .cols = 2, .width = sizeof( _calibration_temper_strength[0][0] )};
static LookupTable calibration_custom_settings_context = {.ptr = _calibration_custom_settings_context, .rows = sizeof( _calibration_custom_settings_context ) / sizeof( _calibration_custom_settings_context[0] ), .cols = 4, .width = sizeof( _calibration_custom_settings_context[0][0] )};
static LookupTable calibration_defog_control = {.ptr = _calibration_defog_control, .rows = 1, .cols = sizeof(_calibration_defog_control) / sizeof(_calibration_defog_control[0]), .width = sizeof(_calibration_defog_control[0])};
static LookupTable calibration_demosaic_rgb_extension_control = {.ptr = _calibration_demosaic_rgb_extension_control, .rows = sizeof(_calibration_demosaic_rgb_extension_control) / sizeof(_calibration_demosaic_rgb_extension_control[0]), .cols = 13, .width = sizeof(_calibration_demosaic_rgb_extension_control[0][0])};
static LookupTable calibration_fr_sharpen_extension_control = {.ptr = _calibration_fr_sharpen_extension_control, .rows = sizeof(_calibration_fr_sharpen_extension_control) / sizeof(_calibration_fr_sharpen_extension_control[0]), .cols = 6, .width = sizeof(_calibration_fr_sharpen_extension_control[0][0])};
static LookupTable calibration_ds_sharpen_extension_control = {.ptr = _calibration_ds_sharpen_extension_control, .rows = sizeof(_calibration_ds_sharpen_extension_control) / sizeof(_calibration_ds_sharpen_extension_control[0]), .cols = 6, .width = sizeof(_calibration_ds_sharpen_extension_control[0][0])};
static LookupTable calibration_cnr_extension_control = {.ptr = _calibration_cnr_extension_control, .rows = sizeof(_calibration_cnr_extension_control) / sizeof(_calibration_cnr_extension_control[0]), .cols = 20, .width = sizeof(_calibration_cnr_extension_control[0][0])};
static LookupTable calibration_iridix_extension_control = {.ptr = _calibration_iridix_extension_control, .rows = sizeof(_calibration_iridix_extension_control) / sizeof(_calibration_iridix_extension_control[0]), .cols = 5, .width = sizeof(_calibration_iridix_extension_control[0][0])};
static LookupTable calibration_sqrt_extension_control = {.ptr = _calibration_sqrt_extension_control, .rows = sizeof(_calibration_sqrt_extension_control) / sizeof(_calibration_sqrt_extension_control[0]), .cols = 3, .width = sizeof(_calibration_sqrt_extension_control[0][0])};
static LookupTable calibration_square_be_extension_control = {.ptr = _calibration_square_be_extension_control, .rows = sizeof(_calibration_square_be_extension_control) / sizeof(_calibration_square_be_extension_control[0]), .cols = 3, .width = sizeof(_calibration_square_be_extension_control[0][0])};
static LookupTable calibration_dp_devthreshold = {.ptr = _calibration_dp_devthreshold, .rows = sizeof(_calibration_dp_devthreshold) / sizeof(_calibration_dp_devthreshold[0]), .cols = 2, .width = sizeof(_calibration_dp_devthreshold[0][0])};
static LookupTable calibration_pf_correction = {.ptr = _calibration_pf_correction, .rows = sizeof(_calibration_pf_correction) / sizeof(_calibration_pf_correction[0]), .cols = 6, .width = sizeof(_calibration_pf_correction[0][0])};
static LookupTable calibration_fc_correction = {.ptr = _calibration_fc_correction, .rows = sizeof(_calibration_fc_correction) / sizeof(_calibration_fc_correction[0]), .cols = 4, .width = sizeof(_calibration_fc_correction[0][0])};

uint32_t get_calibrations_dynamic_linear_sc2232h( ACameraCalibrations *c )
{
    uint32_t result = 0;
    if ( c != 0 ) {
        c->calibrations[CALIBRATION_STITCHING_LM_MED_NOISE_INTENSITY] = &calibration_stitching_lm_med_noise_intensity_thresh;
        c->calibrations[CALIBRATION_EXPOSURE_RATIO_ADJUSTMENT] = &calibration_exposure_ratio_adjustment;
        c->calibrations[CALIBRATION_SINTER_STRENGTH_MC_CONTRAST] = &calibration_sinter_strength_MC_contrast;
        c->calibrations[AWB_COLOUR_PREFERENCE] = &AWB_colour_preference;
        c->calibrations[CALIBRATION_AWB_MIX_LIGHT_PARAMETERS] = &calibration_awb_mix_light_parameters;
        c->calibrations[CALIBRATION_PF_RADIAL_LUT] = &calibration_pf_radial_lut;
        c->calibrations[CALIBRATION_PF_RADIAL_PARAMS] = &calibration_pf_radial_params;
        c->calibrations[CALIBRATION_SINTER_RADIAL_LUT] = &calibration_sinter_radial_lut;
        c->calibrations[CALIBRATION_SINTER_RADIAL_PARAMS] = &calibration_sinter_radial_params;
        c->calibrations[CALIBRATION_AWB_BG_MAX_GAIN] = &calibration_AWB_bg_max_gain;
        c->calibrations[CALIBRATION_IRIDIX8_STRENGTH_DK_ENH_CONTROL] = &calibration_iridix8_strength_dk_enh_control;
        c->calibrations[CALIBRATION_CMOS_EXPOSURE_PARTITION_LUTS] = &calibration_cmos_exposure_partition_luts;
        c->calibrations[CALIBRATION_CMOS_CONTROL] = &calibration_cmos_control;
        c->calibrations[CALIBRATION_STATUS_INFO] = &calibration_status_info;
        c->calibrations[CALIBRATION_AUTO_LEVEL_CONTROL] = &calibration_auto_level_control;
        c->calibrations[CALIBRATION_DP_SLOPE] = &calibration_dp_slope;
        c->calibrations[CALIBRATION_DP_THRESHOLD] = &calibration_dp_threshold;
        c->calibrations[CALIBRATION_STITCHING_LM_MOV_MULT] = &calibration_stitching_lm_mov_mult;
        c->calibrations[CALIBRATION_STITCHING_LM_NP] = &calibration_stitching_lm_np;
        c->calibrations[CALIBRATION_STITCHING_MS_MOV_MULT] = &calibration_stitching_ms_mov_mult;
        c->calibrations[CALIBRATION_STITCHING_MS_NP] = &calibration_stitching_ms_np;
        c->calibrations[CALIBRATION_EVTOLUX_PROBABILITY_ENABLE] = &calibration_evtolux_probability_enable;
        c->calibrations[CALIBRATION_AWB_AVG_COEF] = &calibration_awb_avg_coef;
        c->calibrations[CALIBRATION_IRIDIX_AVG_COEF] = &calibration_iridix_avg_coef;
        c->calibrations[CALIBRATION_IRIDIX_STRENGTH_MAXIMUM] = &calibration_iridix_strength_maximum;
        c->calibrations[CALIBRATION_IRIDIX_MIN_MAX_STR] = &calibration_iridix_min_max_str;
        c->calibrations[CALIBRATION_IRIDIX_EV_LIM_FULL_STR] = &calibration_iridix_ev_lim_full_str;
        c->calibrations[CALIBRATION_IRIDIX_EV_LIM_NO_STR] = &calibration_iridix_ev_lim_no_str;
        c->calibrations[CALIBRATION_AE_CORRECTION] = &calibration_ae_correction;
        c->calibrations[CALIBRATION_AE_EXPOSURE_CORRECTION] = &calibration_ae_exposure_correction;
        c->calibrations[CALIBRATION_SINTER_STRENGTH] = &calibration_sinter_strength;
        c->calibrations[CALIBRATION_SINTER_STRENGTH1] = &calibration_sinter_strength1;
        c->calibrations[CALIBRATION_SINTER_THRESH1] = &calibration_sinter_thresh1;
        c->calibrations[CALIBRATION_SINTER_THRESH4] = &calibration_sinter_thresh4;
        c->calibrations[CALIBRATION_SINTER_INTCONFIG] = &calibration_sinter_intConfig;
        c->calibrations[CALIBRATION_SHARP_ALT_D] = &calibration_sharp_alt_d;
        c->calibrations[CALIBRATION_SHARP_ALT_UD] = &calibration_sharp_alt_ud;
        c->calibrations[CALIBRATION_SHARP_ALT_DU] = &calibration_sharp_alt_du;
        c->calibrations[CALIBRATION_SHARPEN_FR] = &calibration_sharpen_fr;
        c->calibrations[CALIBRATION_DEMOSAIC_NP_OFFSET] = &calibration_demosaic_np_offset;
        c->calibrations[CALIBRATION_MESH_SHADING_STRENGTH] = &calibration_mesh_shading_strength;
        c->calibrations[CALIBRATION_SATURATION_STRENGTH] = &calibration_saturation_strength;
        c->calibrations[CALIBRATION_CCM_ONE_GAIN_THRESHOLD] = &calibration_ccm_one_gain_threshold;
        c->calibrations[CALIBRATION_AE_CONTROL] = &calibration_ae_control;
        c->calibrations[CALIBRATION_AE_CONTROL_HDR_TARGET] = &calibration_ae_control_HDR_target;
        c->calibrations[CALIBRATION_RGB2YUV_CONVERSION] = &calibration_rgb2yuv_conversion;
        c->calibrations[CALIBRATION_AF_LMS] = &calibration_calibration_af_lms;
        c->calibrations[CALIBRATION_AF_ZONE_WGHT_HOR] = &calibration_calibration_af_zone_wght_hor;
        c->calibrations[CALIBRATION_AF_ZONE_WGHT_VER] = &calibration_calibration_af_zone_wght_ver;
        c->calibrations[CALIBRATION_AE_ZONE_WGHT_HOR] = &calibration_calibration_ae_zone_wght_hor;
        c->calibrations[CALIBRATION_AE_ZONE_WGHT_VER] = &calibration_calibration_ae_zone_wght_ver;
        c->calibrations[CALIBRATION_AWB_ZONE_WGHT_HOR] = &calibration_calibration_awb_zone_wght_hor;
        c->calibrations[CALIBRATION_AWB_ZONE_WGHT_VER] = &calibration_calibration_awb_zone_wght_ver;
        c->calibrations[CALIBRATION_CNR_UV_DELTA12_SLOPE] = &calibration_cnr_uv_delta12_slope;
        c->calibrations[CALIBRATION_FS_MC_OFF] = &calibration_fs_mc_off;
        c->calibrations[CALIBRATION_SINTER_SAD] = &calibration_sinter_sad;
        c->calibrations[CALIBRATION_SCALER_H_FILTER] = &calibration_scaler_h_filter;
        c->calibrations[CALIBRATION_SCALER_V_FILTER] = &calibration_scaler_v_filter;
        c->calibrations[CALIBRATION_SHARPEN_DS1] = &calibration_sharpen_ds1;
        c->calibrations[CALIBRATION_TEMPER_STRENGTH] = &calibration_temper_strength;
        c->calibrations[CALIBRATION_GAMMA_EV1] = &calibration_gamma_ev1;
        c->calibrations[CALIBRATION_GAMMA_EV2] = &calibration_gamma_ev2;
        c->calibrations[CALIBRATION_GAMMA_THRESHOLD] = &calibration_gamma_threshold;
        c->calibrations[CALIBRATION_CUSTOM_SETTINGS_CONTEXT] = &calibration_custom_settings_context;
        c->calibrations[CALIBRATION_DEFOG_CONTROL] = &calibration_defog_control;
        c->calibrations[CALIBRATION_DEMOSAIC_RGB_EXT_CONTROL] = &calibration_demosaic_rgb_extension_control;
        c->calibrations[CALIBRATION_FR_SHARPEN_EXT_CONTROL] = &calibration_fr_sharpen_extension_control;
        c->calibrations[CALIBRATION_DS_SHARPEN_EXT_CONTROL] = &calibration_ds_sharpen_extension_control;
        c->calibrations[CALIBRATION_CNR_EXT_CONTROL] = &calibration_cnr_extension_control;
        c->calibrations[CALIBRATION_IRIDIX_EXT_CONTROL] = &calibration_iridix_extension_control;
        c->calibrations[CALIBRATION_SQRT_EXT_CONTROL] = &calibration_sqrt_extension_control;
        c->calibrations[CALIBRATION_SQUARE_BE_EXT_CONTROL] = &calibration_square_be_extension_control;
        c->calibrations[CALIBRATION_DP_DEVTHRESHOLD] = &calibration_dp_devthreshold;
        c->calibrations[CALIBRATION_PF_CORRECTION] = &calibration_pf_correction;
        c->calibrations[CALIBRATION_FC_CORRECTION] = &calibration_fc_correction;
    } else {
        result = -1;
    }
    return result;
}
