/*
Sensor Model 	All
Firmware Version 	3.x
SDK Version 	All

This example demonstrates the use of the Go2 API to connect to a single Gocator at address 192.168.1.10 to read a number of profiles. The profiles are being copied to a memory array for further processing.

Each received data message contains a header with information about the current timestamp, encoder position, and frame index. Each message can contain several data items which are identified by their type. Profiles are identified by the type GO2_TYPE_PROFILE_DATA.

Note: When used in a production environment, error handling functionality should be added to the code.
*/


#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <Go2.h>

#define NUMPROFILES 10

void main(int argc, char **argv)
{
    Go2UInt32 i;
    Go2UInt32 j;
    Go2System system = GO2_NULL;
    Go2Data data = GO2_NULL;
    Go2Int16* memory = GO2_NULL;

    //Initialize the Go2 API.
    Go2Api_Initialize();

    //Construct a Gocator system object.
    Go2System_Construct(&system);

    //Connect to default sensor IP address (192.168.1.10) as user Admin with default password (blank).
    Go2System_Connect(system, GO2_DEFAULT_IP_ADDRESS);

    //Connect to the sensor data channel.
    Go2System_ConnectData(system, GO2_NULL, GO2_NULL);

    //Start the system.
    Go2System_Start(system);

    //loop forever until 10 results are received
    i = 0;
    while (i<NUMPROFILES)
    {
        if (Go2System_ReceiveData(system, 20000, &data) > 0)
        {
            printf("Data message received:\n");
            printf("  Timestamp: %llu\n", Go2Data_Timestamp(data));
            printf("  Encoder: %lld\n", Go2Data_Encoder(data));
            printf("  Frame index: %llu\n", Go2Data_FrameIndex(data));
            printf("  Item count: %u\n", Go2Data_ItemCount(data));

            //each result can have multiple data items
            for (j = 0; j<Go2Data_ItemCount(data); ++j)
            {
                Go2Object dataItem = Go2Data_ItemAt(data, j);
                if (Go2Object_Type(dataItem) == GO2_TYPE_PROFILE_DATA)
                {
                    Go2UInt32 profilePointCount = Go2ProfileData_Width(dataItem);
                    Go2UInt32 profileSizeBytes = profilePointCount * sizeof(Go2UInt16);
                    printf("    Item[%u]: Profile data (%u points)\n", j, profilePointCount);
                    //allocate memory
                    if (memory == GO2_NULL)
                    {
                        memory = malloc(NUMPROFILES * profileSizeBytes);
                    }

                    //copy profiles to memory array
                    memcpy(&memory[i * profilePointCount], Go2ProfileData_Ranges(dataItem), profileSizeBytes);
                }
            }

            Go2Data_Destroy(data);
            i++;
        }
    }

    //free memory array
    free(memory);

    //Stop the system
    Go2System_Stop(system);

    //Free the system object.
    Go2System_Destroy(system);

    //Terminate the Go2 library.
    Go2Api_Terminate();
}
