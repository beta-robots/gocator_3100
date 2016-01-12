#include <GoSdk/GoSdk.h>

kStatus ExampleMain()
{
    kAssembly api = 0; 
    GoSystem system = 0; 

    kCheck(GoSdk_Construct(&api)); 
    kCheck(GoSystem_Construct(&system, kNULL)); 

    kCheck(kObject_Destroy(system)); 
    kCheck(kObject_Destroy(api));

    return kOK; 
}

int main()
{
    return kSuccess(ExampleMain()) ? 0 : -1; 
}