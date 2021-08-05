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

#include "ams_port_platform.h"
#include "ams_device_control_block.h"

uint8_t ams_getByte(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t * readData)
{
    uint8_t read_count = 0;
    uint8_t length = 1;

    /* Sanity check input param */
    if(reg > DEVREG_REG_MAX)
    {
        return 0;
    }

    read_count = AMS_PORT_getByte(portHndl,
                                deviceRegisterDefinition[reg].address,
                                readData,
                                length);

    return read_count;
}

uint8_t ams_setByte(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t data)
{
    uint8_t write_count = 0;
    uint8_t length = 1;

    /* Sanity check input param */
    if(reg > DEVREG_REG_MAX)
    {
        return 0;
    }

    write_count = AMS_PORT_setByte(portHndl,
                                deviceRegisterDefinition[reg].address,
                                &data,
                                length);

    return write_count;
}

uint8_t ams_getBuf(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t * readData, uint8_t length)
{
    uint8_t read_count = 0;

    /* Sanity check input param */
    if(reg > DEVREG_REG_MAX)
    {
        return 0;
    }

    read_count = AMS_PORT_getByte(portHndl,
                                deviceRegisterDefinition[reg].address,
                                readData,
                                length);

    return read_count;
}

uint8_t ams_setBuf(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t * data, uint8_t length)
{
    uint8_t write_count = 0;

    /* Sanity check input param */
    if(reg > DEVREG_REG_MAX)
    {
        return 0;
    }

    write_count = AMS_PORT_setByte(portHndl,
                                deviceRegisterDefinition[reg].address,
                                data,
                                length);

    return write_count;
}

uint8_t ams_getWord(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint16_t * readData)
{
    uint8_t read_count = 0;
    uint8_t length = sizeof(uint16_t);
    uint8_t buffer[sizeof(uint16_t)];

    /* Sanity check input param */
    if(reg > DEVREG_REG_MAX)
    {
        return 0;
    }

    read_count = AMS_PORT_getByte(portHndl,
                                deviceRegisterDefinition[reg].address,
                                buffer,
                                length);

    *readData = ((buffer[0] << AMS_ENDIAN_1) + (buffer[1] << AMS_ENDIAN_2));

    return read_count;
}

uint8_t ams_setWord(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint16_t data)
{
    uint8_t write_count = 0;
    uint8_t length = sizeof(uint16_t);
    uint8_t buffer[sizeof(uint16_t)];

    /* Sanity check input param */
    if(reg > DEVREG_REG_MAX)
    {
        return 0;
    }

    buffer[0] = ((data >> AMS_ENDIAN_1) & 0xff);
    buffer[1] = ((data >> AMS_ENDIAN_2) & 0xff);

    write_count = AMS_PORT_setByte(portHndl,
                                deviceRegisterDefinition[reg].address,
                                &buffer[0],
                                length);
    return write_count;
}

uint8_t ams_getField(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t * data, ams_regMask_t mask)
{
    uint8_t read_count = 0;
    uint8_t length = 1;

    /* Sanity check input param */
    if(reg > DEVREG_REG_MAX)
    {
        return 0;
    }

    read_count = AMS_PORT_getByte(portHndl,
                                deviceRegisterDefinition[reg].address,
                                data,
                                length);

    *data &= mask;

    return read_count;
}

uint8_t ams_setField(AMS_PORT_portHndl * portHndl, ams_deviceRegister_t reg, uint8_t data, ams_regMask_t mask)
{
    uint8_t write_count = 1;
    uint8_t length = 1;
    uint8_t original_data;
    uint8_t new_data;

    AMS_PORT_getByte(portHndl,
                        deviceRegisterDefinition[reg].address,
                        &original_data,
                        length);

    new_data = original_data & ~mask;
    new_data |= (data & mask);

    if (new_data != original_data){
        write_count = AMS_PORT_setByte(portHndl,
                        deviceRegisterDefinition[reg].address,
                        &new_data,
                        length);
    }

    return write_count;
}
