/** 
 * @file    kCollection.c
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kCollection.h>

kBeginInterface(k, kCollection, kNull) 
    //interface methods
    kAddVMethod(kCollection, kCollection, VGetIterator)
    kAddVMethod(kCollection, kCollection, VItemType)
    kAddVMethod(kCollection, kCollection, VCount)
    kAddVMethod(kCollection, kCollection, VHasNext)
    kAddVMethod(kCollection, kCollection, VNext)
kEndInterface() 

kFx(kIterator) kCollection_GetIterator(kCollection collection)
{
    kAssertType(collection, kCollection); 

    return kCollection_GetIterator_(collection); 
}

kFx(kIterator) kCollection_VGetIterator(kCollection collection)
{
    return kNULL; 
}

kFx(kType) kCollection_ItemType(kCollection collection)
{
    kAssertType(collection, kCollection); 
    
    return kCollection_ItemType_(collection); 
}

kFx(kType) kCollection_VItemType(kCollection collection)
{
    return kNULL; 
}

kFx(kSize) kCollection_Count(kCollection collection)
{
    kAssertType(collection, kCollection); 

    return kCollection_Count_(collection); 
}

kFx(kSize) kCollection_VCount(kCollection collection)
{
    return 0;
}

kFx(kBool) kCollection_HasNext(kCollection collection, kIterator iterator)
{
    kAssertType(collection, kCollection); 

    return kCollection_HasNext_(collection, iterator); 
}

kFx(kBool) kCollection_VHasNext(kCollection collection, kIterator iterator)
{
    return kFALSE; 
}

kFx(void*) kCollection_Next(kCollection collection, kIterator* iterator)
{
    kAssertType(collection, kCollection); 

    return kCollection_Next_(collection, iterator); 
}

kFx(void*) kCollection_VNext(kCollection collection, kIterator* iterator)
{
    return kNULL; 
}

