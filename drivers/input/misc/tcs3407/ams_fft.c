/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   AS7000 heartrate application
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file fft.c
 *
 *  \brief FFT-related functions
 *
 */

#include <linux/types.h>
#include "ams_fft.h"

  

/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */


/*
 *****************************************************************************
 * VARIABLES
 *****************************************************************************
 */

/* #pragma arm section zidata = "prm_algorithm" */
/* #pragma arm section rwdata = "prm_algorithm" */


// Pseudo code for generation of the Twiddle Coefficients...
// for (i=0; i<(3*FFT_N/4); i++)
// {
//     twiddleCoef[2*i] = cos(i * 2 * PI / (double)FFT_N);
//     twiddleCoef[2*i+1] = sin(i * 2 * PI / (double)FFT_N);
// }

// Twiddle Coefficients for 16-bit 256-bin FFT
static const int16_t twiddleCoef_int16_256[384] = {
    0x7fff, 0x0000,  0x7ff6, 0x0324,  0x7fd9, 0x0648,  0x7fa7, 0x096b,
    0x7f62, 0x0c8c,  0x7f0a, 0x0fab,  0x7e9d, 0x12c8,  0x7e1e, 0x15e2,
    0x7d8a, 0x18f9,  0x7ce4, 0x1c0c,  0x7c2a, 0x1f1a,  0x7b5d, 0x2224,
    0x7a7d, 0x2528,  0x798a, 0x2827,  0x7885, 0x2b1f,  0x776c, 0x2e11,
    0x7642, 0x30fc,  0x7505, 0x33df,  0x73b6, 0x36ba,  0x7255, 0x398d,
    0x70e3, 0x3c57,  0x6f5f, 0x3f17,  0x6dca, 0x41ce,  0x6c24, 0x447b,
    0x6a6e, 0x471d,  0x68a7, 0x49b4,  0x66d0, 0x4c40,  0x64e9, 0x4ec0,
    0x62f2, 0x5134,  0x60ec, 0x539b,  0x5ed7, 0x55f6,  0x5cb4, 0x5843,
    0x5a82, 0x5a82,  0x5843, 0x5cb4,  0x55f6, 0x5ed7,  0x539b, 0x60ec,
    0x5134, 0x62f2,  0x4ec0, 0x64e9,  0x4c40, 0x66d0,  0x49b4, 0x68a7,
    0x471d, 0x6a6e,  0x447b, 0x6c24,  0x41ce, 0x6dca,  0x3f17, 0x6f5f,
    0x3c57, 0x70e3,  0x398d, 0x7255,  0x36ba, 0x73b6,  0x33df, 0x7505,
    0x30fc, 0x7642,  0x2e11, 0x776c,  0x2b1f, 0x7885,  0x2827, 0x798a,
    0x2528, 0x7a7d,  0x2224, 0x7b5d,  0x1f1a, 0x7c2a,  0x1c0c, 0x7ce4,
    0x18f9, 0x7d8a,  0x15e2, 0x7e1e,  0x12c8, 0x7e9d,  0x0fab, 0x7f0a,
    0x0c8c, 0x7f62,  0x096b, 0x7fa7,  0x0648, 0x7fd9,  0x0324, 0x7ff6,
    0x0000, 0x7fff,  0xfcdc, 0x7ff6,  0xf9b8, 0x7fd9,  0xf695, 0x7fa7,
    0xf374, 0x7f62,  0xf055, 0x7f0a,  0xed38, 0x7e9d,  0xea1e, 0x7e1e,
    0xe707, 0x7d8a,  0xe3f4, 0x7ce4,  0xe0e6, 0x7c2a,  0xdddc, 0x7b5d,
    0xdad8, 0x7a7d,  0xd7d9, 0x798a,  0xd4e1, 0x7885,  0xd1ef, 0x776c,
    0xcf04, 0x7642,  0xcc21, 0x7505,  0xc946, 0x73b6,  0xc673, 0x7255,
    0xc3a9, 0x70e3,  0xc0e9, 0x6f5f,  0xbe32, 0x6dca,  0xbb85, 0x6c24,
    0xb8e3, 0x6a6e,  0xb64c, 0x68a7,  0xb3c0, 0x66d0,  0xb140, 0x64e9,
    0xaecc, 0x62f2,  0xac65, 0x60ec,  0xaa0a, 0x5ed7,  0xa7bd, 0x5cb4,
    0xa57e, 0x5a82,  0xa34c, 0x5843,  0xa129, 0x55f6,  0x9f14, 0x539b,
    0x9d0e, 0x5134,  0x9b17, 0x4ec0,  0x9930, 0x4c40,  0x9759, 0x49b4,
    0x9592, 0x471d,  0x93dc, 0x447b,  0x9236, 0x41ce,  0x90a1, 0x3f17,
    0x8f1d, 0x3c57,  0x8dab, 0x398d,  0x8c4a, 0x36ba,  0x8afb, 0x33df,
    0x89be, 0x30fc,  0x8894, 0x2e11,  0x877b, 0x2b1f,  0x8676, 0x2827,
    0x8583, 0x2528,  0x84a3, 0x2224,  0x83d6, 0x1f1a,  0x831c, 0x1c0c,
    0x8276, 0x18f9,  0x81e2, 0x15e2,  0x8163, 0x12c8,  0x80f6, 0x0fab,
    0x809e, 0x0c8c,  0x8059, 0x096b,  0x8027, 0x0648,  0x800a, 0x0324,
    0x8000, 0x0000,  0x800a, 0xfcdc,  0x8027, 0xf9b8,  0x8059, 0xf695,
    0x809e, 0xf374,  0x80f6, 0xf055,  0x8163, 0xed38,  0x81e2, 0xea1e,
    0x8276, 0xe707,  0x831c, 0xe3f4,  0x83d6, 0xe0e6,  0x84a3, 0xdddc,
    0x8583, 0xdad8,  0x8676, 0xd7d9,  0x877b, 0xd4e1,  0x8894, 0xd1ef,
    0x89be, 0xcf04,  0x8afb, 0xcc21,  0x8c4a, 0xc946,  0x8dab, 0xc673,
    0x8f1d, 0xc3a9,  0x90a1, 0xc0e9,  0x9236, 0xbe32,  0x93dc, 0xbb85,
    0x9592, 0xb8e3,  0x9759, 0xb64c,  0x9930, 0xb3c0,  0x9b17, 0xb140,
    0x9d0e, 0xaecc,  0x9f14, 0xac65,  0xa129, 0xaa0a,  0xa34c, 0xa7bd,
    0xa57e, 0xa57e,  0xa7bd, 0xa34c,  0xaa0a, 0xa129,  0xac65, 0x9f14,
    0xaecc, 0x9d0e,  0xb140, 0x9b17,  0xb3c0, 0x9930,  0xb64c, 0x9759,
    0xb8e3, 0x9592,  0xbb85, 0x93dc,  0xbe32, 0x9236,  0xc0e9, 0x90a1,
    0xc3a9, 0x8f1d,  0xc673, 0x8dab,  0xc946, 0x8c4a,  0xcc21, 0x8afb,
    0xcf04, 0x89be,  0xd1ef, 0x8894,  0xd4e1, 0x877b,  0xd7d9, 0x8676,
    0xdad8, 0x8583,  0xdddc, 0x84a3,  0xe0e6, 0x83d6,  0xe3f4, 0x831c,
    0xe707, 0x8276,  0xea1e, 0x81e2,  0xed38, 0x8163,  0xf055, 0x80f6,
    0xf374, 0x809e,  0xf695, 0x8059,  0xf9b8, 0x8027,  0xfcdc, 0x800a,
};

// Pseudo code for generation of the Twiddle A and B coefficients...
// for (i=0; i<FFT_N; i++)
// {
//     twiddleCoef_A[2*i]   = 0.5 * (1.0 - sin (2 * PI / (double)(2 * FFT_N) * (double)i));
//     twiddleCoef_A[2*i+1] = 0.5 * (-1.0 * cos (2 * PI / (double)(2 * FFT_N) * (double)i));
//     twiddleCoef_B[2*i]   = 0.5 * (1.0 + sin (2 * PI / (double)(2 * FFT_N) * (double)i));
//     twiddleCoef_B[2*i+1] = 0.5 * (1.0 * cos (2 * PI / (double)(2 * FFT_N) * (double)i));
// }

// Twiddle A and B coefficients for 16-bit 256-bin FFT (used for 512-bin result)
const int16_t twiddleCoef_A_Q15_256[512] = {
    0x4000, 0xc000,  0x3f37, 0xc001,  0x3e6e, 0xc005,  0x3da5, 0xc00b,
    0x3cdc, 0xc014,  0x3c13, 0xc01f,  0x3b4b, 0xc02c,  0x3a82, 0xc03c,
    0x39ba, 0xc04f,  0x38f2, 0xc064,  0x382a, 0xc07b,  0x3763, 0xc095,
    0x369c, 0xc0b1,  0x35d5, 0xc0d0,  0x350f, 0xc0f1,  0x3449, 0xc115,
    0x3384, 0xc13b,  0x32bf, 0xc163,  0x31fa, 0xc18e,  0x3136, 0xc1bb,
    0x3073, 0xc1eb,  0x2fb0, 0xc21d,  0x2eee, 0xc251,  0x2e2d, 0xc288,
    0x2d6c, 0xc2c1,  0x2cac, 0xc2fd,  0x2bed, 0xc33b,  0x2b2e, 0xc37b,
    0x2a70, 0xc3be,  0x29b4, 0xc403,  0x28f7, 0xc44a,  0x283c, 0xc493,
    0x2782, 0xc4df,  0x26c9, 0xc52d,  0x2611, 0xc57e,  0x2559, 0xc5d0,
    0x24a3, 0xc625,  0x23ee, 0xc67c,  0x233a, 0xc6d5,  0x2287, 0xc731,
    0x21d5, 0xc78f,  0x2124, 0xc7ee,  0x2074, 0xc850,  0x1fc6, 0xc8b5,
    0x1f19, 0xc91b,  0x1e6d, 0xc983,  0x1dc3, 0xc9ee,  0x1d19, 0xca5b,
    0x1c72, 0xcac9,  0x1bcb, 0xcb3a,  0x1b26, 0xcbad,  0x1a82, 0xcc21,
    0x19e0, 0xcc98,  0x193f, 0xcd11,  0x18a0, 0xcd8c,  0x1802, 0xce08,
    0x1766, 0xce87,  0x16cb, 0xcf07,  0x1632, 0xcf8a,  0x159b, 0xd00e,
    0x1505, 0xd094,  0x1471, 0xd11c,  0x13df, 0xd1a6,  0x134e, 0xd231,
    0x12bf, 0xd2bf,  0x1231, 0xd34e,  0x11a6, 0xd3df,  0x111c, 0xd471,
    0x1094, 0xd505,  0x100e, 0xd59b,  0x0f8a, 0xd632,  0x0f07, 0xd6cb,
    0x0e87, 0xd766,  0x0e08, 0xd802,  0x0d8c, 0xd8a0,  0x0d11, 0xd93f,
    0x0c98, 0xd9e0,  0x0c21, 0xda82,  0x0bad, 0xdb26,  0x0b3a, 0xdbcb,
    0x0ac9, 0xdc72,  0x0a5b, 0xdd19,  0x09ee, 0xddc3,  0x0983, 0xde6d,
    0x091b, 0xdf19,  0x08b5, 0xdfc6,  0x0850, 0xe074,  0x07ee, 0xe124,
    0x078f, 0xe1d5,  0x0731, 0xe287,  0x06d5, 0xe33a,  0x067c, 0xe3ee,
    0x0625, 0xe4a3,  0x05d0, 0xe559,  0x057e, 0xe611,  0x052d, 0xe6c9,
    0x04df, 0xe782,  0x0493, 0xe83c,  0x044a, 0xe8f7,  0x0403, 0xe9b4,
    0x03be, 0xea70,  0x037b, 0xeb2e,  0x033b, 0xebed,  0x02fd, 0xecac,
    0x02c1, 0xed6c,  0x0288, 0xee2d,  0x0251, 0xeeee,  0x021d, 0xefb0,
    0x01eb, 0xf073,  0x01bb, 0xf136,  0x018e, 0xf1fa,  0x0163, 0xf2bf,
    0x013b, 0xf384,  0x0115, 0xf449,  0x00f1, 0xf50f,  0x00d0, 0xf5d5,
    0x00b1, 0xf69c,  0x0095, 0xf763,  0x007b, 0xf82a,  0x0064, 0xf8f2,
    0x004f, 0xf9ba,  0x003c, 0xfa82,  0x002c, 0xfb4b,  0x001f, 0xfc13,
    0x0014, 0xfcdc,  0x000b, 0xfda5,  0x0005, 0xfe6e,  0x0001, 0xff37,
    0x0000, 0x0000,  0x0001, 0x00c9,  0x0005, 0x0192,  0x000b, 0x025b,
    0x0014, 0x0324,  0x001f, 0x03ed,  0x002c, 0x04b5,  0x003c, 0x057e,
    0x004f, 0x0646,  0x0064, 0x070e,  0x007b, 0x07d6,  0x0095, 0x089d,
    0x00b1, 0x0964,  0x00d0, 0x0a2b,  0x00f1, 0x0af1,  0x0115, 0x0bb7,
    0x013b, 0x0c7c,  0x0163, 0x0d41,  0x018e, 0x0e06,  0x01bb, 0x0eca,
    0x01eb, 0x0f8d,  0x021d, 0x1050,  0x0251, 0x1112,  0x0288, 0x11d3,
    0x02c1, 0x1294,  0x02fd, 0x1354,  0x033b, 0x1413,  0x037b, 0x14d2,
    0x03be, 0x1590,  0x0403, 0x164c,  0x044a, 0x1709,  0x0493, 0x17c4,
    0x04df, 0x187e,  0x052d, 0x1937,  0x057e, 0x19ef,  0x05d0, 0x1aa7,
    0x0625, 0x1b5d,  0x067c, 0x1c12,  0x06d5, 0x1cc6,  0x0731, 0x1d79,
    0x078f, 0x1e2b,  0x07ee, 0x1edc,  0x0850, 0x1f8c,  0x08b5, 0x203a,
    0x091b, 0x20e7,  0x0983, 0x2193,  0x09ee, 0x223d,  0x0a5b, 0x22e7,
    0x0ac9, 0x238e,  0x0b3a, 0x2435,  0x0bad, 0x24da,  0x0c21, 0x257e,
    0x0c98, 0x2620,  0x0d11, 0x26c1,  0x0d8c, 0x2760,  0x0e08, 0x27fe,
    0x0e87, 0x289a,  0x0f07, 0x2935,  0x0f8a, 0x29ce,  0x100e, 0x2a65,
    0x1094, 0x2afb,  0x111c, 0x2b8f,  0x11a6, 0x2c21,  0x1231, 0x2cb2,
    0x12bf, 0x2d41,  0x134e, 0x2dcf,  0x13df, 0x2e5a,  0x1471, 0x2ee4,
    0x1505, 0x2f6c,  0x159b, 0x2ff2,  0x1632, 0x3076,  0x16cb, 0x30f9,
    0x1766, 0x3179,  0x1802, 0x31f8,  0x18a0, 0x3274,  0x193f, 0x32ef,
    0x19e0, 0x3368,  0x1a82, 0x33df,  0x1b26, 0x3453,  0x1bcb, 0x34c6,
    0x1c72, 0x3537,  0x1d19, 0x35a5,  0x1dc3, 0x3612,  0x1e6d, 0x367d,
    0x1f19, 0x36e5,  0x1fc6, 0x374b,  0x2074, 0x37b0,  0x2124, 0x3812,
    0x21d5, 0x3871,  0x2287, 0x38cf,  0x233a, 0x392b,  0x23ee, 0x3984,
    0x24a3, 0x39db,  0x2559, 0x3a30,  0x2611, 0x3a82,  0x26c9, 0x3ad3,
    0x2782, 0x3b21,  0x283c, 0x3b6d,  0x28f7, 0x3bb6,  0x29b4, 0x3bfd,
    0x2a70, 0x3c42,  0x2b2e, 0x3c85,  0x2bed, 0x3cc5,  0x2cac, 0x3d03,
    0x2d6c, 0x3d3f,  0x2e2d, 0x3d78,  0x2eee, 0x3daf,  0x2fb0, 0x3de3,
    0x3073, 0x3e15,  0x3136, 0x3e45,  0x31fa, 0x3e72,  0x32bf, 0x3e9d,
    0x3384, 0x3ec5,  0x3449, 0x3eeb,  0x350f, 0x3f0f,  0x35d5, 0x3f30,
    0x369c, 0x3f4f,  0x3763, 0x3f6b,  0x382a, 0x3f85,  0x38f2, 0x3f9c,
    0x39ba, 0x3fb1,  0x3a82, 0x3fc4,  0x3b4b, 0x3fd4,  0x3c13, 0x3fe1,
    0x3cdc, 0x3fec,  0x3da5, 0x3ff5,  0x3e6e, 0x3ffb,  0x3f37, 0x3fff,
};

const int16_t twiddleCoef_B_Q15_256[512] = {
    0x4000, 0x4000,  0x40c9, 0x3fff,  0x4192, 0x3ffb,  0x425b, 0x3ff5,
    0x4324, 0x3fec,  0x43ed, 0x3fe1,  0x44b5, 0x3fd4,  0x457e, 0x3fc4,
    0x4646, 0x3fb1,  0x470e, 0x3f9c,  0x47d6, 0x3f85,  0x489d, 0x3f6b,
    0x4964, 0x3f4f,  0x4a2b, 0x3f30,  0x4af1, 0x3f0f,  0x4bb7, 0x3eeb,
    0x4c7c, 0x3ec5,  0x4d41, 0x3e9d,  0x4e06, 0x3e72,  0x4eca, 0x3e45,
    0x4f8d, 0x3e15,  0x5050, 0x3de3,  0x5112, 0x3daf,  0x51d3, 0x3d78,
    0x5294, 0x3d3f,  0x5354, 0x3d03,  0x5413, 0x3cc5,  0x54d2, 0x3c85,
    0x5590, 0x3c42,  0x564c, 0x3bfd,  0x5709, 0x3bb6,  0x57c4, 0x3b6d,
    0x587e, 0x3b21,  0x5937, 0x3ad3,  0x59ef, 0x3a82,  0x5aa7, 0x3a30,
    0x5b5d, 0x39db,  0x5c12, 0x3984,  0x5cc6, 0x392b,  0x5d79, 0x38cf,
    0x5e2b, 0x3871,  0x5edc, 0x3812,  0x5f8c, 0x37b0,  0x603a, 0x374b,
    0x60e7, 0x36e5,  0x6193, 0x367d,  0x623d, 0x3612,  0x62e7, 0x35a5,
    0x638e, 0x3537,  0x6435, 0x34c6,  0x64da, 0x3453,  0x657e, 0x33df,
    0x6620, 0x3368,  0x66c1, 0x32ef,  0x6760, 0x3274,  0x67fe, 0x31f8,
    0x689a, 0x3179,  0x6935, 0x30f9,  0x69ce, 0x3076,  0x6a65, 0x2ff2,
    0x6afb, 0x2f6c,  0x6b8f, 0x2ee4,  0x6c21, 0x2e5a,  0x6cb2, 0x2dcf,
    0x6d41, 0x2d41,  0x6dcf, 0x2cb2,  0x6e5a, 0x2c21,  0x6ee4, 0x2b8f,
    0x6f6c, 0x2afb,  0x6ff2, 0x2a65,  0x7076, 0x29ce,  0x70f9, 0x2935,
    0x7179, 0x289a,  0x71f8, 0x27fe,  0x7274, 0x2760,  0x72ef, 0x26c1,
    0x7368, 0x2620,  0x73df, 0x257e,  0x7453, 0x24da,  0x74c6, 0x2435,
    0x7537, 0x238e,  0x75a5, 0x22e7,  0x7612, 0x223d,  0x767d, 0x2193,
    0x76e5, 0x20e7,  0x774b, 0x203a,  0x77b0, 0x1f8c,  0x7812, 0x1edc,
    0x7871, 0x1e2b,  0x78cf, 0x1d79,  0x792b, 0x1cc6,  0x7984, 0x1c12,
    0x79db, 0x1b5d,  0x7a30, 0x1aa7,  0x7a82, 0x19ef,  0x7ad3, 0x1937,
    0x7b21, 0x187e,  0x7b6d, 0x17c4,  0x7bb6, 0x1709,  0x7bfd, 0x164c,
    0x7c42, 0x1590,  0x7c85, 0x14d2,  0x7cc5, 0x1413,  0x7d03, 0x1354,
    0x7d3f, 0x1294,  0x7d78, 0x11d3,  0x7daf, 0x1112,  0x7de3, 0x1050,
    0x7e15, 0x0f8d,  0x7e45, 0x0eca,  0x7e72, 0x0e06,  0x7e9d, 0x0d41,
    0x7ec5, 0x0c7c,  0x7eeb, 0x0bb7,  0x7f0f, 0x0af1,  0x7f30, 0x0a2b,
    0x7f4f, 0x0964,  0x7f6b, 0x089d,  0x7f85, 0x07d6,  0x7f9c, 0x070e,
    0x7fb1, 0x0646,  0x7fc4, 0x057e,  0x7fd4, 0x04b5,  0x7fe1, 0x03ed,
    0x7fec, 0x0324,  0x7ff5, 0x025b,  0x7ffb, 0x0192,  0x7fff, 0x00c9,
    0x7fff, 0x0000,  0x7fff, 0xff37,  0x7ffb, 0xfe6e,  0x7ff5, 0xfda5,
    0x7fec, 0xfcdc,  0x7fe1, 0xfc13,  0x7fd4, 0xfb4b,  0x7fc4, 0xfa82,
    0x7fb1, 0xf9ba,  0x7f9c, 0xf8f2,  0x7f85, 0xf82a,  0x7f6b, 0xf763,
    0x7f4f, 0xf69c,  0x7f30, 0xf5d5,  0x7f0f, 0xf50f,  0x7eeb, 0xf449,
    0x7ec5, 0xf384,  0x7e9d, 0xf2bf,  0x7e72, 0xf1fa,  0x7e45, 0xf136,
    0x7e15, 0xf073,  0x7de3, 0xefb0,  0x7daf, 0xeeee,  0x7d78, 0xee2d,
    0x7d3f, 0xed6c,  0x7d03, 0xecac,  0x7cc5, 0xebed,  0x7c85, 0xeb2e,
    0x7c42, 0xea70,  0x7bfd, 0xe9b4,  0x7bb6, 0xe8f7,  0x7b6d, 0xe83c,
    0x7b21, 0xe782,  0x7ad3, 0xe6c9,  0x7a82, 0xe611,  0x7a30, 0xe559,
    0x79db, 0xe4a3,  0x7984, 0xe3ee,  0x792b, 0xe33a,  0x78cf, 0xe287,
    0x7871, 0xe1d5,  0x7812, 0xe124,  0x77b0, 0xe074,  0x774b, 0xdfc6,
    0x76e5, 0xdf19,  0x767d, 0xde6d,  0x7612, 0xddc3,  0x75a5, 0xdd19,
    0x7537, 0xdc72,  0x74c6, 0xdbcb,  0x7453, 0xdb26,  0x73df, 0xda82,
    0x7368, 0xd9e0,  0x72ef, 0xd93f,  0x7274, 0xd8a0,  0x71f8, 0xd802,
    0x7179, 0xd766,  0x70f9, 0xd6cb,  0x7076, 0xd632,  0x6ff2, 0xd59b,
    0x6f6c, 0xd505,  0x6ee4, 0xd471,  0x6e5a, 0xd3df,  0x6dcf, 0xd34e,
    0x6d41, 0xd2bf,  0x6cb2, 0xd231,  0x6c21, 0xd1a6,  0x6b8f, 0xd11c,
    0x6afb, 0xd094,  0x6a65, 0xd00e,  0x69ce, 0xcf8a,  0x6935, 0xcf07,
    0x689a, 0xce87,  0x67fe, 0xce08,  0x6760, 0xcd8c,  0x66c1, 0xcd11,
    0x6620, 0xcc98,  0x657e, 0xcc21,  0x64da, 0xcbad,  0x6435, 0xcb3a,
    0x638e, 0xcac9,  0x62e7, 0xca5b,  0x623d, 0xc9ee,  0x6193, 0xc983,
    0x60e7, 0xc91b,  0x603a, 0xc8b5,  0x5f8c, 0xc850,  0x5edc, 0xc7ee,
    0x5e2b, 0xc78f,  0x5d79, 0xc731,  0x5cc6, 0xc6d5,  0x5c12, 0xc67c,
    0x5b5d, 0xc625,  0x5aa7, 0xc5d0,  0x59ef, 0xc57e,  0x5937, 0xc52d,
    0x587e, 0xc4df,  0x57c4, 0xc493,  0x5709, 0xc44a,  0x564c, 0xc403,
    0x5590, 0xc3be,  0x54d2, 0xc37b,  0x5413, 0xc33b,  0x5354, 0xc2fd,
    0x5294, 0xc2c1,  0x51d3, 0xc288,  0x5112, 0xc251,  0x5050, 0xc21d,
    0x4f8d, 0xc1eb,  0x4eca, 0xc1bb,  0x4e06, 0xc18e,  0x4d41, 0xc163,
    0x4c7c, 0xc13b,  0x4bb7, 0xc115,  0x4af1, 0xc0f1,  0x4a2b, 0xc0d0,
    0x4964, 0xc0b1,  0x489d, 0xc095,  0x47d6, 0xc07b,  0x470e, 0xc064,
    0x4646, 0xc04f,  0x457e, 0xc03c,  0x44b5, 0xc02c,  0x43ed, 0xc01f,
    0x4324, 0xc014,  0x425b, 0xc00b,  0x4192, 0xc005,  0x40c9, 0xc001,
};

// Pseudo code for generation of the bit-reversal-table...
// for (k=1; k<=FFT_N/4; k++)
// {
//     for (i=0; i<log2(FFT_N); i++)
//     {
//         a[i] = k & (1 << i);
//     }
//     for (j=0; j<log2(FFT_N); j++)
//     {
//         if (a[j] != 0)
//             y[k] += (1 << ((log2(FFT_N)-1)-j));
//     }
//     y[k] = y[k] >> 1;
// }
// FFT_N=256 (number of FFT-bins) ==> log2(FFT_N) = 8

// Table for bit-reversal process for 256-bin FFT
static const uint16_t armBitRevTable256[64] = {
    0x40, 0x20, 0x60, 0x10, 0x50, 0x30, 0x70, 0x08,
    0x48, 0x28, 0x68, 0x18, 0x58, 0x38, 0x78, 0x04,
    0x44, 0x24, 0x64, 0x14, 0x54, 0x34, 0x74, 0x0c,
    0x4c, 0x2c, 0x6c, 0x1c, 0x5c, 0x3c, 0x7c, 0x02,
    0x42, 0x22, 0x62, 0x12, 0x52, 0x32, 0x72, 0x0a,
    0x4a, 0x2a, 0x6a, 0x1a, 0x5a, 0x3a, 0x7a, 0x06,
    0x46, 0x26, 0x66, 0x16, 0x56, 0x36, 0x76, 0x0e,
    0x4e, 0x2e, 0x6e, 0x1e, 0x5e, 0x3e, 0x7e, 0x01 };


/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */


/*!
 *****************************************************************************
 * \brief In-place bit reversal function.
 *
 * \param *pSrc - points to the complex input vector (processing occurs in-place)
 * \param fftLen - length of the FFT.
 * \param *pBitRevTab - points to bit reversal table.
 *****************************************************************************
 */
static void ams_bitreversal_int16(int16_t *pSrc16, uint16_t fftLen, uint16_t *pBitRevTab)
{
    int32_t *pSrc = (int32_t *)pSrc16;
    int32_t tmp;
    uint16_t fftLenBy2, fftLenBy2p1;
    uint16_t i, j;

    // Initialization
    j = 0;
    fftLenBy2 = fftLen / 2;
    fftLenBy2p1 = (fftLen / 2) + 1;

    // Bit-reversal
    for (i=0; i<=(fftLenBy2 - 2); i+=2)
    {
        if (i < j)
        {
            // pSrc[i] <-> pSrc[j];
            tmp = pSrc[i];
            pSrc[i] = pSrc[j];
            pSrc[j] = tmp;

            // pSrc[i + fftLenBy2p1] <-> pSrc[j + fftLenBy2p1];
            tmp = pSrc[i + fftLenBy2p1];
            pSrc[i + fftLenBy2p1] = pSrc[j + fftLenBy2p1];
            pSrc[j + fftLenBy2p1] = tmp;
        }

        // pSrc[i+1] <-> pSrc[j+fftLenBy2];
        tmp = pSrc[i + 1];
        pSrc[i + 1] = pSrc[j + fftLenBy2];
        pSrc[j + fftLenBy2] = tmp;

        // Reading the index for the bit reversal
        j = *pBitRevTab;

        // Updating the bit-reversal index depending on the fft length
        pBitRevTab += 1;
    }
}


/*!
 *****************************************************************************
 * \brief Inline 16-bit saturation
 *
 * \param x - the 32-bit value to saturate to 16-bits
 * \returns - the 16-bit saturated result
 *****************************************************************************
 */
/* __STATIC_INLINE */ int32_t __SSAT_16(int32_t x)
{
    if (x > 32767)
    {
        x = 32767;
    }
    else if (x < -32768)
    {
        x = -32768;
    }
    return x;
}


// Radix-4 FFT algorithm...
//
// Input real and imaginary data:
//   x(n) = xa + j * ya
//   x(n+N/4) = xb + j * yb
//   x(n+N/2) = xc + j * yc
//   x(n+3N/4) = xd + j * yd
//
// Output real and imaginary data:
//   x(4r) = xa'+ j * ya'
//   x(4r+1) = xb'+ j * yb'
//   x(4r+2) = xc'+ j * yc'
//   x(4r+3) = xd'+ j * yd'
//
// Twiddle factors for radix-4 FFT:
//   Wn = cos1 + j * (-sin1)
//   W2n = cos2 + j * (-sin2)
//   W3n = cos3 + j * (-sin3)
//
// The real and imaginary output values for the radix-4 butterfly are
//   xa' = xa + xb + xc + xd
//   ya' = ya + yb + yc + yd
//   xb' = (xa+yb-xc-yd)*cos1 + (ya-xb-yc+xd)*sin1
//   yb' = (ya-xb-yc+xd)*cos1 - (xa+yb-xc-yd)*sin1
//   xc' = (xa-xb+xc-xd)*cos2 + (ya-yb+yc-yd)*sin2
//   yc' = (ya-yb+yc-yd)*cos2 - (xa-xb+xc-xd)*sin2
//   xd' = (xa-yb-xc+yd)*cos3 + (ya+xb-yc-xd)*sin3
//   yd' = (ya+xb-yc-xd)*cos3 - (xa-yb-xc+yd)*sin3
//

/*!
 *****************************************************************************
 * \brief Core function for the 16-bit CFFT butterfly process.
 * Internally the input is downscaled by 2 for every stage to avoid saturations
 * during CFFT processing.
 *
 * \param *pSrc - points to the complex input vector (processing occurs in-place)
 * \param fftLen - length of the FFT.
 * \param *pCoef16 - points to twiddle coefficient buffer.
 * \param twidCoefModifier - twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table.
 *****************************************************************************
 */
static void ams_radix4_butterfly_int16(int16_t *pSrc16, uint16_t fftLen, int16_t *pCoef16, uint16_t twidCoefModifier)
{
    int16_t R0, R1, S0, S1, T0, T1, U0, U1;
    int16_t cos1, sin1, cos2, sin2, cos3, sin3, out1, out2;
    uint32_t n1, n2, ic, i0, i1, i2, i3, j, k;

    // Total processing is divided into three stages

    // Initialization for the first stage
    n2 = fftLen;
    n1 = n2;

    // n2 = fftLen/4
    n2 >>= 2;

    // Index for twiddle coefficient
    ic = 0;

    // Index for input read and output write
    i0 = 0;
    j = n2;

    // Input is in 1.15(q15) format

    // First stage processing...
    do
    {
        /* Butterfly implementation */

        /* index calculation for the inputs... */
        /* pSrc16[i0 + 0], pSrc16[i0 + fftLen/4], pSrc16[i0 + fftLen/2], pSrc16[i0 + 3fftLen/4] */
        i1 = i0 + n2;
        i2 = i1 + n2;
        i3 = i2 + n2;

        /* Reading i0, i0+fftLen/2 inputs */

        /* input is down scale by 4 to avoid overflow */
        /* Read ya(real), xa(imag) input */
        T0 = pSrc16[i0 * 2] >> 2;  // --1.13
        T1 = pSrc16[(i0 * 2) + 1] >> 2;  // --1.13

        /* input is down scale by 4 to avoid overflow */
        /* Read yc(real), xc(imag) input */
        S0 = pSrc16[i2 * 2] >> 2;  // --1.13
        S1 = pSrc16[(i2 * 2) + 1] >> 2;  // --1.13

        /* R0 = (ya + yc), R1 = (xa + xc) */
        R0 = (T0 + S0);  // -2.13
        R1 = (T1 + S1);  // -2.13

        /* S0 = (ya - yc), S1 = (xa - xc) */
        S0 = (T0 - S0);  // -2.13
        S1 = (T1 - S1);  // -2.13

        /* Reading i0+fftLen/4, i0+3fftLen/4 inputs */
        /* input is down scale by 4 to avoid overflow */
        /* Read yb(real), xb(imag) input */
        T0 = pSrc16[i1 * 2] >> 2;  // --1.13
        T1 = pSrc16[(i1 * 2) + 1] >> 2;  // --1.13

        /* input is down scale by 4 to avoid overflow */
        /* Read yd(real), xd(imag) input */
        U0 = pSrc16[i3 * 2] >> 2;  // --1.13
        U1 = pSrc16[(i3 * 2) + 1] >> 2;  // --1.13

        /* T0 = (yb + yd), T1 = (xb + xd) */
        T0 = (T0 + U0);  // -2.13
        T1 = (T1 + U1);  // -2.13

        /* writing the butterfly processed i0 sample */
        /* ya' = ya + yb + yc + yd */
        /* xa' = xa + xb + xc + xd */
        pSrc16[i0 * 2] = (R0 >> 1) + (T0 >> 1); // (--2.12) + (--2.12) ==> (-3.12)
        pSrc16[(i0 * 2) + 1] = (R1 >> 1) + (T1 >> 1); // (--2.12) + (--2.12) ==> (-3.12)

        /* R0 = (ya + yc) - (yb + yd) */
        /* R1 = (xa + xc) - (xb + xd) */
        R0 = (R0 - T0);  // 3.13
        R1 = (R1 - T1);  // 3.13

        /* cos2 & sin2 are read from Coefficient pointer */
        cos2 = pCoef16[2 * (ic * 2)]; // 1.15
        sin2 = pCoef16[(2 * (ic * 2)) + 1]; // 1.15

        /* xc' = (xa-xb+xc-xd) * cos2 + (ya-yb+yc-yd) * sin2 */
        /* yc' = (ya-yb+yc-yd) * cos2 - (xa-xb+xc-xd) * sin2 */
        out1 = (int16_t)((cos2 * R0 + sin2 * R1) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12
        out2 = (int16_t)((-sin2 * R0 + cos2 * R1) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12

        /* Reading i0+fftLen/4 */
        /* input is down scale by 4 to avoid overflow */
        /* T0 = yb, T1 = xb */
        T0 = pSrc16[i1 * 2] >> 2;  // --1.13
        T1 = pSrc16[(i1 * 2) + 1] >> 2;  // --1.13

        /* writing the butterfly processed i0 + fftLen/4 sample */
        /* writing output(xc', yc') in little endian cos1format */
        pSrc16[i1 * 2] = out1;  // 4.12
        pSrc16[(i1 * 2) + 1] = out2;  // 4.12

        /* Butterfly calculations */
        /* input is down scale by 4 to avoid overflow */
        /* U0 = yd, U1 = xd */
        U0 = pSrc16[i3 * 2] >> 2;  // --1.13
        U1 = pSrc16[(i3 * 2) + 1] >> 2;  // --1.13

        /* T0 = yb-yd, T1 = xb-xd */
        T0 = (T0 - U0);  // -2.13
        T1 = (T1 - U1);  // -2.13

        /* R1 = (ya-yc) + (xb- xd),  R0 = (xa-xc) - (yb-yd) */
        R0 = (S0 - T1);  // 3.13
        R1 = (S1 + T0);  // 3.13

        /* S1 = (ya-yc) - (xb- xd), S0 = (xa-xc) + (yb-yd) */
        S0 = (S0 + T1);  // 3.13
        S1 = (S1 - T0);  // 3.13

        /* cos1 & sin1 are read from Coefficient pointer */
        cos1 = pCoef16[ic * 2]; // 1.15
        sin1 = pCoef16[(ic * 2) + 1]; // 1.15

        /* Butterfly process for the i0+fftLen/2 sample */
        /* xb' = (xa+yb-xc-yd) * cos1 + (ya-xb-yc+xd) * sin1 */
        /* yb' = (ya-xb-yc+xd) * cos1 - (xa+yb-xc-yd) * sin1 */
        out1 = (int16_t)((sin1 * S1 + cos1 * S0) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12
        out2 = (int16_t)((-sin1 * S0 + cos1 * S1) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12
        /* writing output(xb', yb') in little endian format */
        pSrc16[i2 * 2] = out1;  // 4.12
        pSrc16[(i2 * 2) + 1] = out2;  // 4.12

        /* cos3 & sin3 are read from Coefficient pointer */
        cos3 = pCoef16[3 * (ic * 2)]; // 1.15
        sin3 = pCoef16[(3 * (ic * 2)) + 1]; // 1.15

        /* Butterfly process for the i0+3fftLen/4 sample */
        /* xd' = (xa-yb-xc+yd) * cos3 + (ya+xb-yc-xd) * sin3 */
        /* yd' = (ya+xb-yc-xd) * cos3 - (xa-yb-xc+yd) * sin3 */
        out1 = (int16_t)((sin3 * R1 + cos3 * R0) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12
        out2 = (int16_t)((-sin3 * R0 + cos3 * R1) >> 16); // (-3.28) + (-3.28) = 4.28 ==> 4.12
        /* writing output(xd', yd') in little endian format */
        pSrc16[i3 * 2] = out1;  // 4.12
        pSrc16[(i3 * 2) + 1] = out2;  // 4.12

        /* Twiddle coefficients index modifier */
        ic = ic + twidCoefModifier;

        /* Updating input index */
        i0 = i0 + 1;

    } while(--j);
    // data is in 4.12(q12) format
    // end of first stage processing


    // Start of middle stage processing

    // Twiddle coefficients index modifier
    twidCoefModifier <<= 2;

    // Middle stage processing...
    for (k = fftLen / 4; k > 4; k >>= 2)
    {
        /* Initializations for the middle stage */
        n1 = n2;
        n2 >>= 2;
        ic = 0;

        for (j = 0; j <= (n2 - 1); j++)
        {
            /* index calculation for the coefficients */
            cos1 = pCoef16[ic * 2]; // 1.15
            sin1 = pCoef16[(ic * 2) + 1];
            cos2 = pCoef16[2 * (ic * 2)];
            sin2 = pCoef16[(2 * (ic * 2)) + 1];
            cos3 = pCoef16[3 * (ic * 2)];
            sin3 = pCoef16[(3 * (ic * 2)) + 1];

            /* Twiddle coefficients index modifier */
            ic = ic + twidCoefModifier;

            /* Butterfly implementation */
            for (i0 = j; i0 < fftLen; i0 += n1)
            {
                /* index calculation for the inputs... */
                /* pSrc16[i0 + 0], pSrc16[i0 + fftLen/4], pSrc16[i0 + fftLen/2], pSrc16[i0 + 3fftLen/4] */
                i1 = i0 + n2;
                i2 = i1 + n2;
                i3 = i2 + n2;

                /* Reading i0, i0+fftLen/2 inputs */
                /* Read ya(real), xa(imag) input */
                T0 = pSrc16[i0 * 2];  // 4.12
                T1 = pSrc16[(i0 * 2) + 1];  // 4.12

                /* Read yc(real), xc(imag) input */
                S0 = pSrc16[i2 * 2];  // 4.12
                S1 = pSrc16[(i2 * 2) + 1];  // 4.12

                /* R0 = (ya + yc), R1 = (xa + xc) */
                R0 = __SSAT_16((int32_t)T0 + S0);  // 4.12
                R1 = __SSAT_16((int32_t)T1 + S1);  // 4.12

                /* S0 = (ya - yc), S1 =(xa - xc) */
                S0 = __SSAT_16((int32_t)T0 - S0);  // 4.12
                S1 = __SSAT_16((int32_t)T1 - S1);  // 4.12

                /* Reading i0+fftLen/4, i0+3fftLen/4 inputs */
                /* Read yb(real), xb(imag) input */
                T0 = pSrc16[i1 * 2];  // 4.12
                T1 = pSrc16[(i1 * 2) + 1];  // 4.12

                /* Read yd(real), xd(imag) input */
                U0 = pSrc16[i3 * 2];  // 4.12
                U1 = pSrc16[(i3 * 2) + 1];  // 4.12

                /* T0 = (yb + yd), T1 = (xb + xd) */
                T0 = __SSAT_16((int32_t)T0 + U0);  // 4.12
                T1 = __SSAT_16((int32_t)T1 + U1);  // 4.12

                /* writing the butterfly processed i0 sample */

                /* xa' = xa + xb + xc + xd */
                /* ya' = ya + yb + yc + yd */
                out1 = ((R0 >> 1) + (T0 >> 1)) >> 1;  // 6.10
                out2 = ((R1 >> 1) + (T1 >> 1)) >> 1;  // 6.10

                pSrc16[i0 * 2] = out1;  // 6.10
                pSrc16[(i0 * 2) + 1] = out2;  // 6.10

                /* R0 = (ya + yc) - (yb + yd), R1 = (xa + xc) - (xb + xd) */
                R0 = (R0 >> 1) - (T0 >> 1);  // 5.11
                R1 = (R1 >> 1) - (T1 >> 1);  // 5.11

                /* (ya-yb+yc-yd) * sin2 + (xa-xb+xc-xd) * cos2 */
                /* (ya-yb+yc-yd) * cos2 - (xa-xb+xc-xd) * sin2 */
                out1 = (int16_t)((cos2 * R0 + sin2 * R1) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10
                out2 = (int16_t)((-sin2 * R0 + cos2 * R1) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10

                /* Reading i0+3fftLen/4 */
                /* Read yb(real), xb(imag) input */
                T0 = pSrc16[i1 * 2];  // 4.12
                T1 = pSrc16[(i1 * 2) + 1];  // 4.12

                /* writing the butterfly processed i0 + fftLen/4 sample */
                /* xc' = (xa-xb+xc-xd) * cos2 + (ya-yb+yc-yd) * sin2 */
                /* yc' = (ya-yb+yc-yd) * cos2 - (xa-xb+xc-xd) * sin2 */
                pSrc16[i1 * 2] = out1;  // 6.10
                pSrc16[(i1 * 2) + 1] = out2;  // 6.10

                /* Butterfly calculations */

                /* Read yd(real), xd(imag) input */
                U0 = pSrc16[i3 * 2];  // 4.12
                U1 = pSrc16[(i3 * 2) + 1];  // 4.12

                /* T0 = yb-yd, T1 = xb-xd */
                T0 = __SSAT_16((int32_t)T0 - U0);  // 4.12
                T1 = __SSAT_16((int32_t)T1 - U1);  // 4.12

                /* R0 = (ya-yc) + (xb- xd), R1 = (xa-xc) - (yb-yd) */
                R0 = (S0 >> 1) - (T1 >> 1);  // 5.11
                R1 = (S1 >> 1) + (T0 >> 1);  // 5.11

                /* S0 = (ya-yc) - (xb- xd), S1 = (xa-xc) + (yb-yd) */
                S0 = (S0 >> 1) + (T1 >> 1);  // 5.11
                S1 = (S1 >> 1) - (T0 >> 1);  // 5.11

                /* Butterfly process for the i0+fftLen/2 sample */
                out1 = (int16_t)((cos1 * S0 + sin1 * S1) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10
                out2 = (int16_t)((-sin1 * S0 + cos1 * S1) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10

                /* xb' = (xa+yb-xc-yd) * cos1 + (ya-xb-yc+xd) * sin1 */
                /* yb' = (ya-xb-yc+xd) * cos1 - (xa+yb-xc-yd) * sin1 */
                pSrc16[i2 * 2] = out1;  // 6.10
                pSrc16[(i2 * 2) + 1] = out2;  // 6.10

                /* Butterfly process for the i0+3fftLen/4 sample */
                out1 = (int16_t)((sin3 * R1 + cos3 * R0) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10
                out2 = (int16_t)((-sin3 * R0 + cos3 * R1) >> 16);  // (-5.26) + (-5.26) = 6.26 ==> 6.10

                /* xd' = (xa-yb-xc+yd) * cos3 + (ya+xb-yc-xd) * sin3 */
                /* yd' = (ya+xb-yc-xd) * cos3 - (xa-yb-xc+yd) * sin3 */
                pSrc16[i3 * 2] = out1;  // 6.10
                pSrc16[(i3 * 2) + 1] = out2;  // 6.10
            }
        }
        /* Twiddle coefficients index modifier */
        twidCoefModifier <<= 2;
    }
    // end of middle stage processing


    // data is in 10.6(q6) format for the 1024 point
    // data is in 8.8(q8) format for the 256 point
    // data is in 6.10(q10) format for the 64 point
    // data is in 4.12(q12) format for the 16 point

    // Initialization for the last stage
    n1 = n2;
    n2 >>= 2;

    // Last stage processing...

    /* Butterfly implementation */
    for (i0 = 0; i0 <= (fftLen - n1); i0 += n1)
    {
        /* index calculation for the inputs... */
        /* pSrc16[i0 + 0], pSrc16[i0 + fftLen/4], pSrc16[i0 + fftLen/2], pSrc16[i0 + 3fftLen/4] */
        i1 = i0 + n2;
        i2 = i1 + n2;
        i3 = i2 + n2;

        /* Reading i0, i0+fftLen/2 inputs */
        /* Read ya (real), xa(imag) input */
        T0 = pSrc16[i0 * 2];  // 8.8
        T1 = pSrc16[(i0 * 2) + 1];  // 8.8

        /* Read yc(real), xc(imag) input */
        S0 = pSrc16[i2 * 2];  // 8.8
        S1 = pSrc16[(i2 * 2) + 1];  // 8.8

        /* R0 = (ya + yc), R1 = (xa + xc) */
        R0 = __SSAT_16((int32_t)T0 + S0);  // 8.8
        R1 = __SSAT_16((int32_t)T1 + S1);  // 8.8

        /* S0 = (ya - yc), S1 = (xa - xc) */
        S0 = __SSAT_16((int32_t)T0 - S0);  // 8.8
        S1 = __SSAT_16((int32_t)T1 - S1);  // 8.8

        /* Reading i0+fftLen/4, i0+3fftLen/4 inputs */
        /* Read yb (real), xb(imag) input */
        T0 = pSrc16[i1 * 2];  // 8.8
        T1 = pSrc16[(i1 * 2) + 1];  // 8.8

        /* Read yd (real), xd(imag) input */
        U0 = pSrc16[i3 * 2];  // 8.8
        U1 = pSrc16[(i3 * 2) + 1];  // 8.8

        /* T0 = (yb + yd), T1 = (xb + xd) */
        T0 = __SSAT_16((int32_t)T0 + U0);  // 8.8
        T1 = __SSAT_16((int32_t)T1 + U1);  // 8.8

        /* writing the butterfly processed i0 sample */
        /* xa' = (xa+xb+xc+xd) */
        /* ya' = (ya+yb+yc+yd) */
        pSrc16[i0 * 2] = (R0 >> 1) + (T0 >> 1);  // 9.7
        pSrc16[(i0 * 2) + 1] = (R1 >> 1) + (T1 >> 1);  // 9.7

        /* R0 = (ya + yc) - (yb + yd), R1 = (xa + xc) - (xb + xd) */
        R0 = (R0 >> 1) - (T0 >> 1);  // 9.7
        R1 = (R1 >> 1) - (T1 >> 1);  // 9.7

        /* Read yb(real), xb(imag) input */
        T0 = pSrc16[i1 * 2];  // 8.8
        T1 = pSrc16[(i1 * 2) + 1];  // 8.8

        /* writing the butterfly processed i0 + fftLen/4 sample */
        /* xc' = (xa-xb+xc-xd) */
        /* yc' = (ya-yb+yc-yd) */
        pSrc16[i1 * 2] = R0;  // 9.7
        pSrc16[(i1 * 2) + 1] = R1;  // 9.7

        /* Read yd(real), xd(imag) input */
        U0 = pSrc16[i3 * 2];  // 8.8
        U1 = pSrc16[(i3 * 2) + 1];  // 8.8

        /* T0 = (yb - yd), T1 = (xb - xd)  */
        T0 = __SSAT_16((int32_t)T0 - U0);  // 8.8
        T1 = __SSAT_16((int32_t)T1 - U1);  // 8.8

        /* writing the butterfly processed i0 + fftLen/2 sample */
        /* xb' = (xa+yb-xc-yd) */
        /* yb' = (ya-xb-yc+xd) */
        pSrc16[i2 * 2] = (S0 >> 1) + (T1 >> 1);  // 9.7
        pSrc16[(i2 * 2) + 1] = (S1 >> 1) - (T0 >> 1);  // 9.7

        /* writing the butterfly processed i0 + 3fftLen/4 sample */
        /* xd' = (xa-yb-xc+yd) */
        /* yd' = (ya+xb-yc-xd) */
        pSrc16[i3 * 2] = (S0 >> 1) - (T1 >> 1);  // 9.7
        pSrc16[(i3 * 2) + 1] = (S1 >> 1) + (T0 >> 1);  // 9.7
    }

    // end of last stage processing

    // output is in 11.5(q5) format for the 1024 point
    // output is in 9.7(q7) format for the 256 point
    // output is in 7.9(q9) format for the 64 point
    // output is in 5.11(q11) format for the 16 point
}


/*!
 *****************************************************************************
 * \brief Processing function for the 16-bit 256-bin CFFT
 * Internally the input is downscaled by 2 for every stage to avoid saturations
 * during CFFT processing.
 *
 * \param *pSrc - points to the complex input vector (processing occurs in-place)
 *****************************************************************************
 */
void ams_cfft_radix4_int16_256(int16_t *pSrc)
{
    // Complex-FFT radix-4
    ams_radix4_butterfly_int16(pSrc, 256, (int16_t *)twiddleCoef_int16_256, 1);

    // Bit Reversal
    ams_bitreversal_int16(pSrc, 256, (uint16_t *)armBitRevTable256);
}


/*!
 *****************************************************************************
 * \brief Core Real-FFT process
 * Computes the first N/2 bins of N-bin FFT using the N/2 bin result of complex-FFT.
 * Ar=[0..1], Ai=[-0.5..0.5], Br=[0..1], Bi=[-0.5..0.5]
 *
 * \param *pSrc - array of N/2 complex-values (input)
 * \param fftLen - length of the FFT (this is N/2).
 * \param *pATable - A twiddle coefficient buffer.
 * \param *pBTable - B twiddle coefficient buffer.
 * \param modifier - A/B twiddle coefficient modifier that supports different size FFTs with the same twiddle factor tables.
 *****************************************************************************
 */
void ams_split_in_place_rfft_int16(int16_t *pSrc, uint16_t fftLen, int16_t *pATable, int16_t *pBTable, uint16_t modifier)
{
    uint16_t i;
    int32_t outR1, outI1, outR2, outI2;
    int16_t *pCoefA, *pCoefB;
    int16_t *pSrc1, *pSrc2;

    // handle all cases except "i=N/4" and "i=0"
    for (i=1; i<(fftLen>>1); i++)
    {
        // setup data-pointers for "i" and "N/2-i"
        pSrc1 = &pSrc[i * 2];
        pSrc2 = &pSrc[(fftLen - i) * 2];

        // setup pointers to twiddle-coefficients for "i"
        pCoefA = &pATable[i * modifier * 2];
        pCoefB = &pBTable[i * modifier * 2];

        // outR[i] = (pSrc[2i] * pATable[2i] - pSrc[2i + 1] * pATable[2i + 1] +
        //            pSrc[2(N-i)] * pBTable[2i] + pSrc[2(N-i) + 1] * pBTable[2i + 1]);
        outR1 = (*pSrc1 * *pCoefA) - (*(pSrc1 + 1) * *(pCoefA + 1));
        outR1 += (*pSrc2 * *pCoefB) + (*(pSrc2 + 1) * *(pCoefB + 1));

        // outI[i] = (pIn[2i + 1] * pATable[2i] + pIn[2i] * pATable[2i + 1] +
        //            pIn[2(N-i)] * pBTable[2i + 1] - pIn[2(N-i) + 1] * pBTable[2i]);
        outI1 = (*(pSrc1 + 1) * *pCoefA) + (*pSrc1 * *(pCoefA + 1));
        outI1 += (*pSrc2 * *(pCoefB + 1)) - (*(pSrc2 + 1) * *pCoefB);

        // setup pointers to twiddle-coefficients for "N/2-i"
        pCoefA = &pATable[(fftLen - i) * modifier * 2];
        pCoefB = &pBTable[(fftLen - i) * modifier * 2];

        // outR[N-i] = (pSrc[2(N-i)] * pATable[2(N-i)] - pSrc[2(N-i) + 1] * pATable[2(N-i) + 1] +
        //              pSrc[2i] * pBTable[2(N-i)] + pSrc[2i + 1] * pBTable[2(N-i) + 1]);
        outR2 = (*pSrc2 * *pCoefA) - (*(pSrc2 + 1) * *(pCoefA + 1));
        outR2 += (*pSrc1 * *pCoefB) + (*(pSrc1 + 1) * *(pCoefB + 1));

        // outI[N-i] = (pIn[2(N-i) + 1] * pATable[2(N-i)] + pIn[2(N-i)] * pATable[2(N-i) + 1] +
        //              pIn[2i] * pBTable[2(N-i) + 1] - pIn[2i + 1] * pBTable[2(N-i)]);
        outI2 = (*(pSrc2 + 1) * *pCoefA) + (*pSrc2 * *(pCoefA + 1));
        outI2 += (*pSrc1 * *(pCoefB + 1)) - (*(pSrc1 + 1) * *pCoefB);

        // write output for "i" and "N/2-i"
        *pSrc1 = (int16_t)(outR1 >> 15);
        *(pSrc1+1) = (int16_t)(outI1 >> 15);

        *pSrc2 = (int16_t)(outR2 >> 15);
        *(pSrc2+1) = (int16_t)(outI2 >> 15);
    }

    // handle "i=N/4"
    // setup data-pointers for "i = N/4"
    pSrc1 = &pSrc[i * 2]; // note:  N/4 == N/2 - N/4
    // setup pointers to twiddle-coefficients for "i"
    pCoefA = &pATable[i * modifier * 2];
    pCoefB = &pBTable[i * modifier * 2];
    // outR[i] = (pSrc[2i] * pATable[2i] - pSrc[2i + 1] * pATable[2i + 1] +
    //            pSrc[2(N-i)] * pBTable[2i] + pSrc[2(N-i) + 1] * pBTable[2i + 1]);
    outR1 = (*pSrc1 * *pCoefA) - (*(pSrc1 + 1) * *(pCoefA + 1));
    outR1 += (*pSrc1 * *pCoefB) + (*(pSrc1 + 1) * *(pCoefB + 1));
    // outI[i] = (pIn[2i + 1] * pATable[2i] + pIn[2i] * pATable[2i + 1] +
    //            pIn[2(N-i)] * pBTable[2i + 1] - pIn[2(N-i) + 1] * pBTable[2i]);
    outI1 = (*(pSrc1 + 1) * *pCoefA) + (*pSrc1 * *(pCoefA + 1));
    outI1 += (*pSrc1 * *(pCoefB + 1)) - (*(pSrc1 + 1) * *pCoefB);
    // write output for "i = N/4"
    *pSrc1 = (int16_t)(outR1 >> 15);
    *(pSrc1+1) = (int16_t)(outI1 >> 15);

    // handle "i=0"
    pSrc[0] = pSrc[0] + pSrc[1];
    pSrc[1] = 0;
}


/*!
 *****************************************************************************
 * \brief Computes the 512-bin FFT for the 512-value real-input-array
 *
 * \param *pSrc - 512-value real-input-array (processing occurs in-place)
 *****************************************************************************
 */
void ams_rfft_int16_512(int16_t *pSrc)
{
    // Perform a 256-bin complex FFT using the real-data as input
    ams_radix4_butterfly_int16(pSrc, 256, (int16_t *)twiddleCoef_int16_256, 1);
    ams_bitreversal_int16(pSrc, 256, (uint16_t *)armBitRevTable256);

    // Compute the first 256-bins of the 512-bin result
    ams_split_in_place_rfft_int16(pSrc, 256, (int16_t *)twiddleCoef_A_Q15_256, (int16_t *)twiddleCoef_B_Q15_256, 1);
}


/*!
 *****************************************************************************
 * \brief Computes the complex magnitude for the complex-array
 *
 * \param *pSrc - points to the complex input vector
 * \param *pDst - points to the real output vector [magnitude]
 * \param numSamples - number of complex samples in the input vector
 *****************************************************************************
 */
void complexMagnitude(int16_t *pSrc, uint16_t *pDst, uint16_t numSamples)
{
    int16_t real, imag;
    uint32_t magSq;

    while (numSamples > 0)
    {
        // result = sqrt((real * real) + (imag * imag))
        real = *pSrc++;
        imag = *pSrc++;
        magSq = (uint32_t)(((int32_t)real * real) + ((int32_t)imag * imag));
        *pDst++ = (uint16_t)ams_isqrt(magSq);
        /* *pDst++ = (uint16_t)sqrtf((float)magSq); */

        // decrement the loop counter
        numSamples--;
    }
}


uint32_t ams_isqrt(uint32_t x)
{
    register uint32_t op, res, one;
    op = x;
    res = 0;

    // "one" starts at the highest power of four <= than the argument.
    one = 1 << 30;  // second-to-top bit set
    while (one > op) one >>= 2;

    while (one != 0)
    {
        if (op >= res + one)
	{
            op -= res + one;
            res += one << 1;  // <-- faster than 2 * one
        }
        res >>= 1;
        one >>= 2;
    }
    return res;
}



