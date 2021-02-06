/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
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
 * AMS generic device driver
 */

/*
 * @@AMS_REVISION_Id: 969322ab678073812e97aab66a03df1d04e97836
 */

#ifndef __OSAL_LINUX_INPUT_ALS_H
#define __OSAL_LINUX_INPUT_ALS_H

#if defined(CONFIG_AMS_OPTICAL_SENSOR_ALS)
#include "osal_linux_input.h"

extern struct device_attribute osal_als_attrs []; 
extern int osal_als_attrs_size;

extern void osal_report_als(struct amsDriver_chip *chip);
extern void osal_report_flicker(struct amsDriver_chip *chip);
extern void osal_report_sw_flicker(struct amsDriver_chip *chip);
extern ssize_t osal_flicker_enable_set(struct amsDriver_chip *chip, uint8_t valueToSet);
extern ssize_t osal_als_enable_set(struct amsDriver_chip *chip, uint8_t valueToSet);
extern int osal_configure_als_mode(struct amsDriver_chip *chip, u8 state);
extern int osal_get_lux(struct amsDriver_chip *chip);
extern int osal_read_als(struct amsDriver_chip *chip);
extern void osal_report_als(struct amsDriver_chip *chip);
extern int osal_set_segment_tables(struct amsDriver_chip *chip);
extern ssize_t osal_sw_flicker_enable_set(struct amsDriver_chip *chip, uint8_t valueToSet);

#endif

#endif /*__OSAL_LINUX_INPUT_ALS_H */
