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
 *      PROJECT:   AS7000 firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file fft.h
 *
 *  \brief header for the FFT-related functions
 *
 */

#ifndef FFT_H
#define FFT_H

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
//#include "heartrate.h"


/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */


/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */

// Computes the 16-bit-integer 256-bin complex-FFT for the complex-input-data (result replaces input)
void ams_cfft_radix4_int16_256(int16_t *pSrc);

// Computes the 16-bit-integer 512-bin FFT for the real-input-data returning 256 complex-bins (result replaces input)
void ams_rfft_int16_512(int16_t *pSrc);

// Computes the complex magnitude for the complex-array
void complexMagnitude(int16_t *pSrc, uint16_t *pDst, uint16_t numSamples);

// Compute integer square root
uint32_t ams_isqrt(uint32_t x);


#endif /* FFT_H */
