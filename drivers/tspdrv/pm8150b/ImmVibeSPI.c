/*
** =============================================================================
**
** File: ImmVibeSPI.c
**
** Description:
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
**
** Copyright (c) 2012-2017 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
**
** =============================================================================
*/

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>

/*
** This SPI supports only one actuator.
*/
#define DEVICE_BUS  6
#define DEVICE_ADDR 0x59
#define NUM_ACTUATORS       1

/*
** Called to disable amp (disable output force)
*/
#define DEVICE_NAME "LG JUDY"
#define NUM_EXTRA_BUFFERS  0

extern int qti_haptic_timed_vmax(int value);

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
    DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_AmpDisable.\n"));

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_AmpEnable.\n"));

    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
    /* Remove driver */

    /* Set PWM frequency */
    /* To be implemented with appropriate hardware access macros */

    /* Set duty cycle to 50% */
    /* To be implemented with appropriate hardware access macros */

    DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_Terminate.\n"));
    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/
int g_value=0;
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{

	//DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_SetSamples: nOutputSignalBitDepth %d, nBufferSizeInBytes %d, pForceOutputBuffer[0] %d\n",
	//	nOutputSignalBitDepth, nBufferSizeInBytes, pForceOutputBuffer[0]));

	//DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_SetSamples: pForceOutputBuffer[0] %d\n", pForceOutputBuffer[0]));

	/*
	   how to process negative value
	   1) ignore
	   2) do like zero
	   3) make positive
	*/
	if( pForceOutputBuffer[0] == -128 ) {
		//DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_SetSamples: negative pForceOutputBuffer[0] %d, change to 127.\n", pForceOutputBuffer[0]));
		pForceOutputBuffer[0] = 127;
	}else if( pForceOutputBuffer[0] < 0 ) {
		//DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_SetSamples: negative pForceOutputBuffer[0] %d\n", pForceOutputBuffer[0]));
		// 1) ignore
		//return VIBE_S_SUCCESS;
		// 2) do like zero
		//pForceOutputBuffer[0] = 0;
		// 3) make positive
		pForceOutputBuffer[0] *= -1;
	}

	// check if new value is the same with a previous one.
	if( g_value == pForceOutputBuffer[0] ) {
		//DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_SetSamples: the same with previous value. nOutputSignalBitDepth %d, nBufferSizeInBytes %d, pForceOutputBuffer[0] %d\n", 
		//	nOutputSignalBitDepth, nBufferSizeInBytes, pForceOutputBuffer[0]));
		return VIBE_S_SUCCESS;
	}

	g_value = pForceOutputBuffer[0];

    qti_haptic_timed_vmax(pForceOutputBuffer[0]);	

    return VIBE_S_SUCCESS;
}



/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(
        VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID,
        VibeUInt32 nFrequencyParameterValue) {
    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    return VIBE_S_SUCCESS;
}



