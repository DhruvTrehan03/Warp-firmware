/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
	TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
	PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
	BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"
#include "math.h"
#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t			gWarpI2cBaudRateKbps;
extern volatile uint32_t			gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t			gWarpSupplySettlingDelayMilliseconds;



void
initMMA8451Q(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceMMA8451QState.i2cAddress					= i2cAddress;
	deviceMMA8451QState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterMMA8451Q(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		1,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorMMA8451Q(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;


	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	i2cWriteStatus1 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QF_SETUP /* register address F_SETUP */,
												  payloadF_SETUP /* payload: Disable FIFO */
	);

	i2cWriteStatus2 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1 /* register address CTRL_REG1 */,
												  payloadCTRL_REG1 /* payload */
	);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceMMA8451QState.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

#define BUFFER_WINDOW_SIZE 30 // Change this value to adjust the buffer size

void
printSensorDataMMA8451Q(bool hexModeFlag, uint32_t timeAtStart)
{
    uint16_t readSensorRegisterValueLSB;
    uint16_t readSensorRegisterValueMSB;
    int16_t readSensorRegisterValueCombined;
    WarpStatus i2cReadStatus;

    static float smoothedMagnitudeBuffer[BUFFER_WINDOW_SIZE] = {0.0f}; // Circular buffer for smoothed magnitudes
    static int smoothedMagnitudeIndex = 0;                            // Index for the circular buffer
    static int knockBuffer[BUFFER_WINDOW_SIZE] = {0};                 // Circular buffer for knock detection
    static int knockBufferIndex = 0;
    static int knockCount = 0;
    static bool doorKnockActive = false;
    static float knockConfidenceBuffer[BUFFER_WINDOW_SIZE] = {0.0f};
    static int confidenceBufferIndex = 0;
    static int initializationCounter = 0;                             // Counter for initialization phase

    const int threshold = 800;
    const float smoothingFactor = 0.5f;
    const int changeWindow = 2;

    warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

    // Read X, Y, Z data
    i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
    readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
    float x = (float)readSensorRegisterValueCombined;

    i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
    readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
    float y = (float)readSensorRegisterValueCombined;

    i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
    readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
    float z = (float)readSensorRegisterValueCombined;

    // Normalize the data
    float controlMean[3] = {34.508f, -22.628f, 4214.858f};
    float controlCovInv[3][3] = {
        {6.22339275e-03f, -4.08358881e-04f, 4.45779715e-05f},
        {-4.08358881e-04f, 7.56096923e-03f, 1.22078095e-05f},
        {4.45779715e-05f, 1.22078095e-05f, 2.24688965e-03f}};
    float centeredData[3] = {x - controlMean[0], y - controlMean[1], z - controlMean[2]};
    float normalizedData[3] = {
        centeredData[0] * controlCovInv[0][0] + centeredData[1] * controlCovInv[0][1] + centeredData[2] * controlCovInv[0][2],
        centeredData[0] * controlCovInv[1][0] + centeredData[1] * controlCovInv[1][1] + centeredData[2] * controlCovInv[1][2],
        centeredData[0] * controlCovInv[2][0] + centeredData[1] * controlCovInv[2][1] + centeredData[2] * controlCovInv[2][2]};
    float normalizedMagnitude = sqrtf(normalizedData[0] * normalizedData[0] +
                                       normalizedData[1] * normalizedData[1] +
                                       normalizedData[2] * normalizedData[2]);

    // Apply exponential smoothing and store in the circular buffer
    smoothedMagnitudeBuffer[smoothedMagnitudeIndex] =
        1000 * smoothingFactor * normalizedMagnitude +
        (1 - smoothingFactor) * smoothedMagnitudeBuffer[(smoothedMagnitudeIndex - 1 + BUFFER_WINDOW_SIZE) % BUFFER_WINDOW_SIZE];
    smoothedMagnitudeIndex = (smoothedMagnitudeIndex + 1) % BUFFER_WINDOW_SIZE;

    // Initialization phase: Populate the buffer during the first BUFFER_WINDOW_SIZE iterations
    if (initializationCounter < BUFFER_WINDOW_SIZE)
    {
        initializationCounter++;
        return; // Skip further processing during initialization
    }

    // Calculate maxDifference using the smoothedMagnitudeBuffer
    float maxDifference = 0.0f;
    for (int i = 1; i <= changeWindow; i++)
    {
        float difference = fabsf(smoothedMagnitudeBuffer[smoothedMagnitudeIndex] -
                                 smoothedMagnitudeBuffer[(smoothedMagnitudeIndex - i + BUFFER_WINDOW_SIZE) % BUFFER_WINDOW_SIZE]);
        if (difference > maxDifference)
        {
            maxDifference = difference;
        }
    }

    // Detect knocks
    bool knockDetected = maxDifference > threshold;
	//print Knocks

    // Add a static variable to track the knock state
    static bool knockInProgress = false;

    if (knockDetected)
    {
        if (!knockInProgress)
        {
            // Register the knock only if no knock is currently in progress
            knockBuffer[knockBufferIndex] = 1;
            knockConfidenceBuffer[confidenceBufferIndex] = 1.0f / (1.0f + expf(-maxDifference/1000.0f)); // Sigmoid scaling
            knockInProgress = true; // Mark that a knock is in progress
        }
        else
        {
            // Do not register additional knocks while the knock is in progress
            knockBuffer[knockBufferIndex] = 0;
            knockConfidenceBuffer[confidenceBufferIndex] = 0.0f;
        }
    }
    else
    {
        // Reset the knockInProgress flag when the knock condition is no longer met
        knockInProgress = false;
        knockBuffer[knockBufferIndex] = 0;
        knockConfidenceBuffer[confidenceBufferIndex] = 0.0f;
    }

    knockBufferIndex = (knockBufferIndex + 1) % BUFFER_WINDOW_SIZE;
    confidenceBufferIndex = (confidenceBufferIndex + 1) % BUFFER_WINDOW_SIZE;
    // Check for door knock condition
    knockCount = 0;
    for (int i = 0; i < BUFFER_WINDOW_SIZE; i++)
    {
        knockCount += knockBuffer[i];
    }

    if (knockCount >= 3)
    {
        if (!doorKnockActive)
        {
            doorKnockActive = true;
            float avgConfidence = 0.0f;
            for (int i = 0; i < BUFFER_WINDOW_SIZE; i++)
            {
                avgConfidence += knockConfidenceBuffer[i];
            }
            avgConfidence /= BUFFER_WINDOW_SIZE; // Calculate the average confidence

            // Scale avgConfidence to an integer (e.g., multiply by 100 for percentage-like output)
            int scaledConfidence = (int)(avgConfidence * 100);

            // Print the scaled confidence as an integer
            warpPrint(" Door knock detected! Confidence: %d%% \n", (int)scaledConfidence*10);
        }
    }
    else
    {
        // Reset the doorKnockActive flag when the knock count drops below the threshold
        doorKnockActive = false;
    }
}

uint8_t
appendSensorDataMMA8451Q(uint8_t* buf)
{
	uint8_t index = 0;
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	return index;
}