/** 
 * @file    kAssembly.h
 * @brief   Declares the kAssembly class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>   //--inclusion order controlled by kApiDef

#ifndef K_API_ASSEMBLY_H
#define K_API_ASSEMBLY_H

kBeginHeader()

/**
 * @class   kAssembly
 * @extends kObject
 * @ingroup kApi
 * @brief   Represents a library of types.  
 * 
 * An assembly represents a collection of types. Typically, one assembly is defined per library or application. 
 * 
 * The kAssembly_Enumerate method can be used to get a list of all loaded assemblies. The kAssemblyOf macro can be 
 * used to obtain a handle to a specific type assembly using the compile-time assembly symbol. 
 * 
 * @code {.c}
 * 
 * //prints a list of supplemental assemblies that have been constructed (i.e., assemblies other than kApiLib)
 * kStatus PrintOtherAssemblies()
 * {
 *     kAssembly coreAssembly = kAssemblyOf(kApiLib); 
 *     kArrayList assemblyList = kNULL; 
 *     kSize i; 
 * 
 *     kTry
 *     {
 *          //get a list of the loaded assemblies
 *          kTest(kArrayList_Construct(&assemblyList, kTypeOf(kAssembly), 0, kNULL)); 
 *          kTest(kAssembly_Enumerate(assemblyList)); 
 *         
 *          //print information for all assemblies except kApiLib
 *          for (i = 0; i < kArrayList_Count(assemblyList); ++i)
 *          {
 *              kAssembly assembly = kArrayList_As_(assemblyList, i, kAssembly); 
 *              
 *              if (assembly != coreAssembly)
 *              {
 *                  printf("%s (%u types)\n", kAssembly_Name(assembly), (k32u)kAssembly_TypeCount(assembly)); 
 *              }
 *          }
 *      }
 *      kFinally
 *      {
 *          //dispose the list of assemblies; each assembly handle returned by kAssembly_Enumerate
 *          //is a reference-counted instance that must be destroyed
 *          kObject_Dispose(assemblyList); 
 *    
 *          kEndFinally(); 
 *      } 
 *     
 *      return kOK; 
 * } 
 * 
 * @endcode
 * 
 */
//typedef kObject kAssembly;   --forward-declared in kApiDef.x.h

/** 
 * Gets a list of the currently-loaded assemblies.  
 * 
 * Each assembly returned by this function is a reference-counted instance that should be destroyed 
 * when no longer needed. 
 *
 * @public              @memberof kAssembly
 * @param   assemblies  Receives references to loaded assemblies. 
 * @return              Operation status. 
 */
kFx(kStatus) kAssembly_Enumerate(kArrayList assemblies); 

/** 
 * Adds a callback to be notified when a new assembly is loaded. 
 * 
 * The callback will be notified immediately after the assembly is loaded. The callback 'sender' 
 * argument will be the handle of the loaded assembly.
 * 
 * The static assembly lock (an internal, recursive, mutual exclusion structure) is held for the duration 
 * of callback invocation. Callback functions should be structured accordingly to avoid the potential 
 * for deadlock.
 *
 * @public              @memberof kAssembly
 * @param   function    Callback function.
 * @param   receiver    Context pointer for callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kAssembly_AddLoadHandler(kCallbackFx function, kPointer receiver); 

/** 
 * Removes a callback that was registered with the kAssembly_AddLoadHandler function.
 * 
 * @public              @memberof kAssembly
 * @param   function    Callback function.
 * @param   receiver    Context pointer for callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kAssembly_RemoveLoadHandler(kCallbackFx function, kPointer receiver); 

/** 
 * Adds a callback to be notified just prior to unloading an assembly. 
 * 
 * The callback will be notified just prior to unloading the assembly. The callback 'sender' 
 * argument will be the handle of the assembly that is being unloaded.
 *
 * The static assembly lock (an internal, recursive, mutual exclusion structure) is held for the duration 
 * of callback invocation. Callback functions should be structured accordingly to avoid the potential 
 * for deadlock.
 *
 * @public              @memberof kAssembly
 * @param   function    Callback function.
 * @param   receiver    Context pointer for callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kAssembly_AddUnloadHandler(kCallbackFx function, kPointer receiver); 

/** 
 * Removes a callback that was registered with the kAssembly_AddUnloadHandler function.
 * 
 * @public              @memberof kAssembly
 * @param   function    Callback function.
 * @param   receiver    Context pointer for callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kAssembly_RemoveUnloadHandler(kCallbackFx function, kPointer receiver); 

/** 
 * Adds a callback to be notified after unloading an assembly.
 * 
 * The callback will be notified just after the release of the assembly's static types, and just 
 * prior to destroying the assembly handle. The callback 'sender' argument will be the handle of 
 * the assembly that is being unloaded.
 *
 * The static assembly lock (an internal, recursive, mutual exclusion structure) is held for the duration 
 * of callback invocation. Callback functions should be structured accordingly to avoid the potential 
 * for deadlock.
 *
 * @public              @memberof kAssembly
 * @param   function    Callback function.
 * @param   receiver    Context pointer for callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kAssembly_AddUnloadedHandler(kCallbackFx function, kPointer receiver); 

/** 
 * Removes a callback that was registered with the kAssembly_AddUnloadedHandler function.
 * 
 * @public              @memberof kAssembly
 * @param   function    Callback function.
 * @param   receiver    Context pointer for callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kAssembly_RemoveUnloadedHandler(kCallbackFx function, kPointer receiver); 

/** 
 * Gets the assembly version.
 *
 * @public              @memberof kAssembly
 * @param   assembly    Assembly. 
 * @return              Version number. 
 */
kFx(kVersion) kAssembly_Version(kAssembly assembly);

/** 
 * Gets the assembly name.
 *
 * @public              @memberof kAssembly
 * @param   assembly    Assembly. 
 * @return              Assembly name. 
 */
kFx(const kChar*) kAssembly_Name(kAssembly assembly);

/** 
 * Gets the number of types in an assembly. 
 *
 * @public              @memberof kAssembly
 * @param   assembly    Assembly. 
 * @return              Type count.
 */
kFx(kSize) kAssembly_TypeCount(kAssembly assembly); 

/** 
 * Gets the type at a particular index within an assembly. 
 *
 * @public              @memberof kAssembly
 * @param   assembly    Assembly. 
 * @param   index       Type index. 
 * @return              Type at the specified index.  
 */
kFx(kType) kAssembly_TypeAt(kAssembly assembly, kSize index); 

/** 
 * Finds a type by name. 
 *
 * @public              @memberof kAssembly
 * @param   assembly    Assembly to search. 
 * @param   name        Type name. 
 * @param   type        Receives the type.  
 * @return              Operation status (kERROR_NOT_FOUND on lookup failure). 
 */
kFx(kStatus) kAssembly_FindType(kAssembly assembly, const kChar* name, kType* type); 

/** 
 * Gets the count of assembly dependencies. 
 *
 * @public              @memberof kAssembly
 * @param   assembly    Assembly. 
 * @return              Operation status. 
 */
kFx(kSize) kAssembly_DependencyCount(kAssembly assembly); 

/** 
 * Gets the assembly dependency at the specified index. 
 *
 * @public              @memberof kAssembly
 * @param   assembly    Assembly. 
 * @param   index       Dependency index. 
 * @return              Assembly dependency.   
 */
kFx(kAssembly) kAssembly_DependencyAt(kAssembly assembly, kSize index); 

kEndHeader()

#include <kApi/kAssembly.x.h>

#endif
