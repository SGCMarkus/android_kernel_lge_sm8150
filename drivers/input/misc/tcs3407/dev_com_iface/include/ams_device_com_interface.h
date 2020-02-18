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
 * Device Communication Interface
 */

/*
 * @@AMS_REVISION_Id: cbb588206a899acbdaf1264bdec007f27e4aab1a
 */

#ifndef __AMS_DEVICE_COMM_INTERFACE_H__
#define __AMS_DEVICE_COMM_INTERFACE_H__

#ifdef  __cplusplus
extern "C" {
#endif

extern uint8_t ams_getByte(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t * readData);
extern uint8_t ams_getWord(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint16_t * readData);
extern uint8_t ams_getBuf(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t * readData, uint8_t length);
extern uint8_t ams_getField(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t * data, ams_regMask_t mask);
extern uint8_t ams_setByte(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t data);
extern uint8_t ams_setWord(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint16_t data);
extern uint8_t ams_setBuf(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t * data, uint8_t length);
extern uint8_t ams_setField(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t data, ams_regMask_t mask);

#ifdef  __cplusplus
}
#endif

#endif  /* __AMS_DEVICE_COMM_INTERFACE_H__ */

