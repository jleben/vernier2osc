/*********************************************************************************

Copyright (c) 2010, Vernier Software & Technology
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Vernier Software & Technology nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL VERNIER SOFTWARE & TECHNOLOGY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************************/

// NGIO_DeviceCheck.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <assert.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#pragma warning(disable: 4996)
#include <windows.h>
#endif

#ifdef UNIX
#include <time.h>
#endif

/*
#ifdef TARGET_OS_MAC
#include <Carbon/Carbon.h>
#endif
*/

#include "NGIO_lib_interface.h"

#include <lo/lo.h>

#include <string>

#define MAX_NUM_MEASUREMENTS 40000
#define MEASUREMENT_PERIOD 0.001
#define OSC_MESSAGE_PERIOD 0.005

static void sleep_ms(unsigned int msToSleep)
{
#if defined(_WIN32)
    ::Sleep(msToSleep);
#elif defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
    struct timespec tv;
    tv.tv_sec = msToSleep/1000;
    tv.tv_nsec = (msToSleep % 1000) * 1000000;
    nanosleep(&tv, NULL);
#else
    bool OS_CAN_SLEEP = false;
    assert(OS_CAN_SLEEP);
#endif
}

NGIO_LIBRARY_HANDLE g_hNGIOlib = NULL;
NGIO_DEVICE_HANDLE g_hDevice = NULL;

static bool send_measurements(int channel_count, float *channel_data, lo_address address)
{
    //string msg_format(channel_count, 'i');

    lo_message msg = lo_message_new();

    //printf("sending message:\n");
    for (int ch = 0; ch < channel_count; ++ch)
    {
        lo_message_add( msg, "f", channel_data[ch] );
        //printf("%f\n", channel_data[ch]);
    }
#if 0
    lo_message msg_gyro = lo_message_new();
    lo_message_add( msg_gyro, "ii", (int)frame.gyroX, (int)frame.gyroY );

    lo_bundle bundle = lo_bundle_new( LO_TT_IMMEDIATE );
    lo_bundle_add_message( bundle, osc_eeg_addr.c_str(), msg_channels );
    lo_bundle_add_message( bundle, osc_gyro_addr.c_str(), msg_gyro );

    int res = lo_send_bundle( addr, bundle );
#endif
    int res = lo_send_message( address, "/sensor", msg );

    lo_message_free(msg);

    if( res == -1 ) {
        printf("Sending OSC failed!\n");
        return false;
    }

    return true;
}

int main(int argc, char* argv[])
{
    gtype_uint16 majorVersion, minorVersion;
    gtype_int32 status = 0;
    gtype_uint32 sig, mask, deviceType;
    gtype_uint32 numDevices;
    NGIO_DEVICE_LIST_HANDLE hDeviceList;
    char deviceName[NGIO_MAX_SIZE_DEVICE_NAME];
    NGIOGetStatusCmdResponsePayload getStatusResponse;
    NGIO_NVMEM_CHANNEL_ID1_rec getNVMemResponse;
    gtype_uint32 nRespBytes;
    signed char channel;
    char units[20];

    gtype_int32 rawMeasurements[MAX_NUM_MEASUREMENTS];
    gtype_real32 volts[MAX_NUM_MEASUREMENTS];
    gtype_real32 calbMeasurements[MAX_NUM_MEASUREMENTS];
    gtype_int32 numMeasurements, i;
    gtype_real32 averageCalbMeasurement;

    float *osc_data = 0;
    int channel_count = 0;

    // OSC
    lo_address osc_addr = lo_address_new( "localhost", "57120" );

    //////

    g_hNGIOlib = NGIO_Init();
    NGIO_GetDLLVersion(g_hNGIOlib, &majorVersion, &minorVersion);

    printf("NGIO_DeviceCheck version '3.2'.\n");
    printf("NGIO_DeviceCheck is linked to NGIO library version '%02d.%02d'.\n", majorVersion, minorVersion);


#ifdef _WIN32
    sleep_ms(500); //Give Jungo device driver time to find the LabQuest.
#endif

    if (g_hNGIOlib)
    {
        deviceType = NGIO_DEVTYPE_LABQUEST;
        NGIO_SearchForDevices(g_hNGIOlib, deviceType, NGIO_COMM_TRANSPORT_USB, NULL, &sig);

        hDeviceList = NGIO_OpenDeviceListSnapshot(g_hNGIOlib, deviceType, &numDevices, &sig);
        status = NGIO_DeviceListSnapshot_GetNthEntry(hDeviceList, 0, deviceName, sizeof(deviceName), &mask);
        NGIO_CloseDeviceListSnapshot(hDeviceList);

        if (0 != status)
        {
            //Couldn't find a LabQuest, so look for a LabQuest Mini.
            deviceType = NGIO_DEVTYPE_LABQUEST_MINI;
            NGIO_SearchForDevices(g_hNGIOlib, deviceType, NGIO_COMM_TRANSPORT_USB, NULL, &sig);

            hDeviceList = NGIO_OpenDeviceListSnapshot(g_hNGIOlib, deviceType, &numDevices, &sig);
            status = NGIO_DeviceListSnapshot_GetNthEntry(hDeviceList, 0, deviceName, sizeof(deviceName), &mask);
            NGIO_CloseDeviceListSnapshot(hDeviceList);
        }

        if (0 != status)
        {
            //Now look for LabQuest2.
            deviceType = NGIO_DEVTYPE_LABQUEST2;
            NGIO_SearchForDevices(g_hNGIOlib, deviceType, NGIO_COMM_TRANSPORT_USB, NULL, &sig);

            hDeviceList = NGIO_OpenDeviceListSnapshot(g_hNGIOlib, deviceType, &numDevices, &sig);
            status = NGIO_DeviceListSnapshot_GetNthEntry(hDeviceList, 0, deviceName, sizeof(deviceName), &mask);
            NGIO_CloseDeviceListSnapshot(hDeviceList);
        }

        if (0 != status)
        {
            printf("NGIO_DeviceCheck cannot find a LabQuest or a LabQuest Mini.\n");
        }
        else
        {
            char deviceDesc[50];
            if (NGIO_DEVTYPE_LABQUEST == deviceType)
                strcpy(deviceDesc, "LabQuest");
            else if (NGIO_DEVTYPE_LABQUEST2 == deviceType)
                strcpy(deviceDesc, "LabQuest2");
            else
                strcpy(deviceDesc, "LabQuest Mini");
            g_hDevice = NGIO_Device_Open(g_hNGIOlib, deviceName, 0);
            if (!g_hDevice)
                printf("Failed to open %s device %s \n", deviceDesc, deviceName);
            else
                printf("Successfully opened %s device %s \n", deviceDesc, deviceName);

            if (g_hDevice)
            {
                if ((NGIO_DEVTYPE_LABQUEST == deviceType) || (NGIO_DEVTYPE_LABQUEST2 == deviceType))
                {
#if !(defined(TARGET_PLATFORM_LABQUEST) || defined(TARGET_PLATFORM_LABQUEST2))
                    //Wrest control of the LabQuest data acquisition subsystem(the DAQ) away from the GUI app running
                    //down on the LabQuest.
                    status = NGIO_Device_AcquireExclusiveOwnership(g_hDevice, NGIO_GRAB_DAQ_TIMEOUT);
                    if (0 != status)
                        printf("NGIO_Device_AcquireExclusiveOwnership() failed!\n");
#endif
                }

                if (0 == status)
                {
                    memset(&getStatusResponse, 0, sizeof(getStatusResponse));
                    nRespBytes = sizeof(getStatusResponse);
                    status = NGIO_Device_SendCmdAndGetResponse(g_hDevice, NGIO_CMD_ID_GET_STATUS, NULL, 0, &getStatusResponse,
                                                               &nRespBytes, NGIO_TIMEOUT_MS_DEFAULT);
                }

                if (0 == status)
                {
                    printf("DAQ firmware version is %x.%02x .\n", (gtype_uint16) getStatusResponse.majorVersionMasterCPU,
                           (gtype_uint16) getStatusResponse.minorVersionMasterCPU);

                    memset(&getNVMemResponse, 0, sizeof(getNVMemResponse));
                    status = NGIO_Device_NVMemBlk_Read(g_hDevice, NGIO_NVMEM_CHANNEL_ID1, &getNVMemResponse, 0,
                                                       sizeof(getNVMemResponse) - 1, NGIO_TIMEOUT_MS_DEFAULT);
                }

                if (0 == status)
                {
                    unsigned int serialNum = getNVMemResponse.serialNumber.msbyteMswordSerialCounter;
                    serialNum = (serialNum << 8) + getNVMemResponse.serialNumber.lsbyteMswordSerialCounter;
                    serialNum = (serialNum << 8) + getNVMemResponse.serialNumber.msbyteLswordSerialCounter;
                    serialNum = (serialNum << 8) + getNVMemResponse.serialNumber.lsbyteLswordSerialCounter;
                    printf("LabQuest serial number(yy ww nnnnnnnn) is %02x %02x %08d \n",
                           (gtype_uint16) getNVMemResponse.serialNumber.yy,
                           (gtype_uint16) getNVMemResponse.serialNumber.ww, serialNum);
                }

                if (0 == status)
                {
                    NGIOSetSensorChannelEnableMaskParams maskParams;
                    NGIOSetAnalogInputParams analogInputParams;
                    unsigned char sensorId = 0;
                    unsigned char channelMask = NGIO_CHANNEL_MASK_ANALOG1;
                    gtype_uint32 sig;
                    memset(&maskParams, 0, sizeof(maskParams));
                    for (channel = NGIO_CHANNEL_ID_ANALOG1; channel <= NGIO_CHANNEL_ID_ANALOG4; channel++)
                    {
                        NGIO_Device_DDSMem_GetSensorNumber(g_hDevice, channel, &sensorId, 1, &sig, NGIO_TIMEOUT_MS_DEFAULT);
                        if (sensorId != 0)
                        {
                            maskParams.lsbyteLsword_EnableSensorChannels = maskParams.lsbyteLsword_EnableSensorChannels | channelMask;
                            if (sensorId >= kSensorIdNumber_FirstSmartSensor)
                                NGIO_Device_DDSMem_ReadRecord(g_hDevice, channel, 0, NGIO_TIMEOUT_MS_READ_DDSMEMBLOCK);

                            if (kProbeTypeAnalog10V == NGIO_Device_GetProbeType(g_hDevice, channel))
                                analogInputParams.analogInput = NGIO_ANALOG_INPUT_PM10V_BUILTIN_12BIT_ADC;
                            else
                                analogInputParams.analogInput = NGIO_ANALOG_INPUT_5V_BUILTIN_12BIT_ADC;
                            analogInputParams.channel = channel;
                            NGIO_Device_SendCmdAndGetResponse(g_hDevice, NGIO_CMD_ID_SET_ANALOG_INPUT, &analogInputParams,
                                                              sizeof(analogInputParams), NULL, NULL, NGIO_TIMEOUT_MS_DEFAULT);

                            ++channel_count;
                        }
                        channelMask = channelMask << 1;
                    }

                    if (0 == maskParams.lsbyteLsword_EnableSensorChannels)
                        printf("No analog sensors found.\n");
                    else
                    {
                        maskParams.lsbyteLsword_EnableSensorChannels = maskParams.lsbyteLsword_EnableSensorChannels & 14;//spam ignore analog4

                        NGIO_Device_SendCmdAndGetResponse(g_hDevice, NGIO_CMD_ID_SET_SENSOR_CHANNEL_ENABLE_MASK, &maskParams,
                                                          sizeof(maskParams), NULL, NULL, NGIO_TIMEOUT_MS_DEFAULT);

                        NGIO_Device_SetMeasurementPeriod(g_hDevice, -1, MEASUREMENT_PERIOD, NGIO_TIMEOUT_MS_DEFAULT);// 1000 hz.

                        NGIO_Device_SendCmdAndGetResponse(g_hDevice, NGIO_CMD_ID_START_MEASUREMENTS, NULL, 0, NULL, NULL, NGIO_TIMEOUT_MS_DEFAULT);

                        osc_data = new float[channel_count];

                        while(1)
                        {
                            sleep_ms(OSC_MESSAGE_PERIOD * 1000); //Wait 1 second.
                            int osc_channel_idx = 0;

                            for (channel = NGIO_CHANNEL_ID_ANALOG1; channel <= NGIO_CHANNEL_ID_ANALOG4; channel++)
                            {
                                NGIO_Device_DDSMem_GetSensorNumber(g_hDevice, channel, &sensorId, 0, &sig, 0);
                                if (sensorId != 0)
                                {
                                    //do {
#if 0
                                    char longname[30];
                                    longname[0] = 0;
                                    printf("Sensor id in channel ANALOG%d = %d", channel, sensorId);
                                    NGIO_Device_DDSMem_GetLongName(g_hDevice, channel, longname, sizeof(longname));
                                    if (strlen(longname) != 0)
                                        printf("(%s)", longname);
#endif
                                    int probeType = NGIO_Device_GetProbeType(g_hDevice, channel);
                                    numMeasurements = NGIO_Device_ReadRawMeasurements(g_hDevice, channel, rawMeasurements, NULL, MAX_NUM_MEASUREMENTS);

                                    if (numMeasurements > 0)
                                    {
                                        averageCalbMeasurement = 0.0;
                                        for (i = 0; i < numMeasurements; i++)
                                        {
                                            volts[i] = NGIO_Device_ConvertToVoltage(g_hDevice, channel, rawMeasurements[i], probeType);
                                            calbMeasurements[i] = NGIO_Device_CalibrateData(g_hDevice, channel, volts[i]);
                                            averageCalbMeasurement += calbMeasurements[i];
                                        }
                                        if (numMeasurements > 1)
                                            averageCalbMeasurement = averageCalbMeasurement/numMeasurements;

                                        gtype_real32 a, b, c;
                                        unsigned char activeCalPage = 0;
                                        NGIO_Device_DDSMem_GetActiveCalPage(g_hDevice, channel, &activeCalPage);
                                        NGIO_Device_DDSMem_GetCalPage(g_hDevice, channel, activeCalPage, &a, &b, &c, units, sizeof(units));

                                        //printf("; average of %d measurements = %8.3f %s .", numMeasurements, averageCalbMeasurement, units);
                                        osc_data[osc_channel_idx] = averageCalbMeasurement;
                                    }
                                    //printf("\n");
                                    //} while (numMeasurements > 0);

                                    ++osc_channel_idx;
                                } //if (sensorId != 0)
                            }

                            send_measurements(channel_count, osc_data, osc_addr);
                        }

                        NGIO_Device_SendCmdAndGetResponse(g_hDevice, NGIO_CMD_ID_STOP_MEASUREMENTS, NULL, 0, NULL, NULL, NGIO_TIMEOUT_MS_DEFAULT);

                        memset(&getStatusResponse, 0, sizeof(getStatusResponse));
                        nRespBytes = sizeof(getStatusResponse);
                        status = NGIO_Device_SendCmdAndGetResponse(g_hDevice, NGIO_CMD_ID_GET_STATUS, NULL, 0, &getStatusResponse,
                                                                   &nRespBytes, NGIO_TIMEOUT_MS_DEFAULT);
                        if (0 == status)
                        {
                            printf("DAQ reports status byte of %xh\n", getStatusResponse.status);
                        }

                    }
                }

                NGIO_Device_Close(g_hDevice);
                g_hDevice = NULL;
            }
        }

        NGIO_Uninit(g_hNGIOlib);
    }
    g_hNGIOlib = NULL;

    return 0;
}
