/** 
 * @file    GoMeasurements.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoMeasurements.h>
#include <GoSdk/Tools/GoExtParam.x.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

/* Range tool measurements */

kBeginClass(Go, GoRangePositionZ, GoMeasurement)
    kAddVMethod(GoRangePositionZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoRangePositionZ_VInit(GoRangePositionZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoRangePositionZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_RANGE_POSITION_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoRangeThicknessThickness, GoMeasurement)
    kAddVMethod(GoRangeThicknessThickness, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoRangeThicknessThickness_VInit(GoRangeThicknessThickness measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoRangeThicknessThicknessClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_RANGE_THICKNESS_THICKNESS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


/* Profile tool measurements */

kBeginClass(Go, GoProfileAreaArea, GoMeasurement)
    kAddVMethod(GoProfileAreaArea, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileAreaArea_VInit(GoProfileAreaArea measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileAreaAreaClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_AREA_AREA, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileAreaCentroidX, GoMeasurement)
    kAddVMethod(GoProfileAreaCentroidX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileAreaCentroidX_VInit(GoProfileAreaCentroidX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileAreaCentroidXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_AREA_CENTROID_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileAreaCentroidZ, GoMeasurement)
    kAddVMethod(GoProfileAreaCentroidZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileAreaCentroidZ_VInit(GoProfileAreaCentroidZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileAreaCentroidZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_AREA_CENTROID_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileBoxX, GoMeasurement)
kAddVMethod(GoProfileBoxX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileBoxX_VInit(GoProfileBoxX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileBoxXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileBoxZ, GoMeasurement)
kAddVMethod(GoProfileBoxZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileBoxZ_VInit(GoProfileBoxZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileBoxZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileBoxWidth, GoMeasurement)
kAddVMethod(GoProfileBoxWidth, GoMeasurement, VInit)
kEndClass()


GoFx(kStatus) GoProfileBoxWidth_VInit(GoProfileBoxWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileBoxWidthClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_WIDTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileBoxHeight, GoMeasurement)
kAddVMethod(GoProfileBoxHeight, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileBoxHeight_VInit(GoProfileBoxHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileBoxHeightClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_HEIGHT, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileBoxGlobalX, GoMeasurement)
kAddVMethod(GoProfileBoxGlobalX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileBoxGlobalX_VInit(GoProfileBoxGlobalX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileBoxGlobalXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileCircleX, GoMeasurement)
    kAddVMethod(GoProfileCircleX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileCircleX_VInit(GoProfileCircleX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileCircleXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileCircleZ, GoMeasurement)
    kAddVMethod(GoProfileCircleZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileCircleZ_VInit(GoProfileCircleZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileCircleZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileCircleRadius, GoMeasurement)
    kAddVMethod(GoProfileCircleRadius, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileCircleRadius_VInit(GoProfileCircleRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileCircleRadiusClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_RADIUS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileDimWidth, GoMeasurement)
    kAddVMethod(GoProfileDimWidth, GoMeasurement, VInit)
    kAddVMethod(GoProfileDimWidth, GoMeasurement, VRead)
    kAddVMethod(GoProfileDimWidth, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfileDimWidth_VInit(GoProfileDimWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileDimWidthClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_DIMENSION_WIDTH, sensor, srcTool, kTRUE, alloc)); 

    obj->absolute = kFALSE;

    return kOK; 
}

GoFx(kStatus) GoProfileDimWidth_VRead(GoProfileDimWidth measurement, kXml xml, kXmlItem item)
{
    GoProfileDimWidthClass* obj = GoProfileDimWidth_Cast_(measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->absolute));

    return kOK; 
}

GoFx(kStatus) GoProfileDimWidth_VWrite(GoProfileDimWidth measurement, kXml xml, kXmlItem item)
{
    GoProfileDimWidthClass* obj = GoProfileDimWidth_Cast_(measurement);

    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->absolute));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(kBool) GoProfileDimWidth_AbsoluteEnabled(GoProfileDimWidth measurement)
{
    GoProfileDimWidthClass* obj = GoProfileDimWidth_Cast_(measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->absolute;
}

GoFx(kStatus) GoProfileDimWidth_EnableAbsolute(GoProfileDimWidth measurement, kBool absolute)
{
    GoProfileDimWidthClass* obj = GoProfileDimWidth_Cast_(measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));    
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->absolute = absolute;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfileDimHeight, GoMeasurement)
    kAddVMethod(GoProfileDimHeight, GoMeasurement, VInit)
    kAddVMethod(GoProfileDimHeight, GoMeasurement, VRead)
    kAddVMethod(GoProfileDimHeight, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfileDimHeight_VInit(GoProfileDimHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileDimHeightClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_DIMENSION_HEIGHT, sensor, srcTool, kTRUE, alloc)); 

    obj->absolute = kFALSE;

    return kOK; 
}

GoFx(kStatus) GoProfileDimHeight_VRead(GoProfileDimHeight measurement, kXml xml, kXmlItem item)
{
    GoProfileDimHeightClass* obj = GoProfileDimHeight_Cast_(measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->absolute));

    return kOK; 
}

GoFx(kStatus) GoProfileDimHeight_VWrite(GoProfileDimHeight measurement, kXml xml, kXmlItem item)
{
    GoProfileDimHeightClass* obj = GoProfileDimHeight_Cast_(measurement);

    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->absolute));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(kBool) GoProfileDimHeight_AbsoluteEnabled(GoProfileDimHeight measurement)
{
    GoProfileDimHeightClass* obj = GoProfileDimHeight_Cast_(measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->absolute;
}

GoFx(kStatus) GoProfileDimHeight_EnableAbsolute(GoProfileDimHeight measurement, kBool absolute)
{
    GoProfileDimHeightClass* obj = GoProfileDimHeight_Cast_(measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->absolute = absolute;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfileDimDistance, GoMeasurement)
    kAddVMethod(GoProfileDimDistance, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileDimDistance_VInit(GoProfileDimDistance measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileDimDistanceClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_DIMENSION_DISTANCE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileDimCenterX, GoMeasurement)
    kAddVMethod(GoProfileDimCenterX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileDimCenterX_VInit(GoProfileDimCenterX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileDimCenterXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileDimCenterZ, GoMeasurement)
    kAddVMethod(GoProfileDimCenterZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileDimCenterZ_VInit(GoProfileDimCenterZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileDimCenterZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileGrooveX, GoMeasurement)
    kAddVMethod(GoProfileGrooveX, GoMeasurement, VInit)
    kAddVMethod(GoProfileGrooveX, GoMeasurement, VRead)
    kAddVMethod(GoProfileGrooveX, GoMeasurement, VWrite)    
kEndClass()


GoFx(kStatus) GoProfileGrooveX_VInit(GoProfileGrooveX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileGrooveXClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_GROOVE_X, sensor, srcTool, kTRUE, alloc)); 

    obj->location = GO_PROFILE_GROOVE_LOCATION_BOTTOM;
    obj->selectType = GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH;
    obj->selectIndex = 0;

    return kOK; 
}

GoFx(kStatus) GoProfileGrooveX_VRead(GoProfileGrooveX measurement, kXml xml, kXmlItem item)
{
    GoProfileGrooveXClass* obj = GoProfileGrooveX_Cast_(measurement); 

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));

    return kOK; 
}

GoFx(kStatus) GoProfileGrooveX_VWrite(GoProfileGrooveX measurement, kXml xml, kXmlItem item)
{
    GoProfileGrooveXClass* obj = GoProfileGrooveX_Cast_(measurement);

    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));

    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(GoProfileGrooveLocation) GoProfileGrooveX_Location(GoProfileGrooveX measurement)
{
    GoProfileGrooveXClass* obj = GoProfileGrooveX_Cast_(measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoProfileGrooveX_SetLocation(GoProfileGrooveX measurement, GoProfileGrooveLocation location)
{
    GoProfileGrooveXClass* obj = GoProfileGrooveX_Cast_(measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileGrooveX_SelectType(GoProfileGrooveX measurement)
{
    GoProfileGrooveXClass* obj = GoProfileGrooveX_Cast_(measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileGrooveX_SetSelectType(GoProfileGrooveX measurement, GoProfileGrooveSelectType selectType)
{
    GoProfileGrooveXClass* obj = GoProfileGrooveX_Cast_(measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileGrooveX_SelectIndex(GoProfileGrooveX measurement)
{
    GoProfileGrooveXClass* obj = GoProfileGrooveX_Cast_(measurement);
    return obj->selectIndex;
}

GoFx(kStatus) GoProfileGrooveX_SetSelectIndex(GoProfileGrooveX measurement, k32u selectIndex)
{
    GoProfileGrooveXClass* obj = GoProfileGrooveX_Cast_(measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfileGrooveZ, GoMeasurement)
    kAddVMethod(GoProfileGrooveZ, GoMeasurement, VInit)
    kAddVMethod(GoProfileGrooveZ, GoMeasurement, VRead)
    kAddVMethod(GoProfileGrooveZ, GoMeasurement, VWrite)
kEndClass()


GoFx(kStatus) GoProfileGrooveZ_VInit(GoProfileGrooveZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileGrooveZClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_GROOVE_Z, sensor, srcTool, kTRUE, alloc)); 

    obj->location = GO_PROFILE_GROOVE_LOCATION_BOTTOM;
    obj->selectType = GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH;
    obj->selectIndex = 0;

    return kOK; 
}

GoFx(kStatus) GoProfileGrooveZ_VRead(GoProfileGrooveZ measurement, kXml xml, kXmlItem item)
{
    GoProfileGrooveZClass* obj = GoProfileGrooveZ_Cast_(measurement); 

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));

    return kOK; 
}

GoFx(kStatus) GoProfileGrooveZ_VWrite(GoProfileGrooveZ measurement, kXml xml, kXmlItem item)
{
    GoProfileGrooveZClass* obj = GoProfileGrooveZ_Cast_(measurement); 

    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(GoProfileGrooveLocation) GoProfileGrooveZ_Location(GoProfileGrooveZ measurement)
{
    GoProfileGrooveZClass* obj = GoProfileGrooveZ_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoProfileGrooveZ_SetLocation(GoProfileGrooveZ measurement, GoProfileGrooveLocation location)
{
    GoProfileGrooveZClass* obj = GoProfileGrooveZ_Cast_(measurement); 

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


GoFx(GoProfileGrooveSelectType) GoProfileGrooveZ_SelectType(GoProfileGrooveZ measurement)
{
    GoProfileGrooveZClass* obj = GoProfileGrooveZ_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileGrooveZ_SetSelectType(GoProfileGrooveZ measurement, GoProfileGrooveSelectType selectType)
{
    GoProfileGrooveZClass* obj = GoProfileGrooveZ_Cast_(measurement); 

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileGrooveZ_SelectIndex(GoProfileGrooveZ measurement)
{
    GoProfileGrooveZClass* obj = GoProfileGrooveZ_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileGrooveZ_SetSelectIndex(GoProfileGrooveZ measurement, k32u selectIndex)
{
    GoProfileGrooveZClass* obj = GoProfileGrooveZ_Cast_(measurement); 

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfileGrooveWidth, GoMeasurement)
    kAddVMethod(GoProfileGrooveWidth, GoMeasurement, VInit)
    kAddVMethod(GoProfileGrooveWidth, GoMeasurement, VRead)
    kAddVMethod(GoProfileGrooveWidth, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfileGrooveWidth_VInit(GoProfileGrooveWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileGrooveWidthClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_GROOVE_WIDTH, sensor, srcTool, kTRUE, alloc)); 

    obj->selectType = GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH;
    obj->selectIndex = 0;

    return kOK; 
}

GoFx(kStatus) GoProfileGrooveWidth_VRead(GoProfileGrooveWidth measurement, kXml xml, kXmlItem item)
{
    GoProfileGrooveWidthClass* obj = GoProfileGrooveWidth_Cast_(measurement); 

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));    

    return kOK; 
}

GoFx(kStatus) GoProfileGrooveWidth_VWrite(GoProfileGrooveWidth measurement, kXml xml, kXmlItem item)
{
    GoProfileGrooveWidthClass* obj = GoProfileGrooveWidth_Cast_(measurement); 

    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));    
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(GoProfileGrooveSelectType) GoProfileGrooveWidth_SelectType(GoProfileGrooveWidth measurement)
{
    GoProfileGrooveWidthClass* obj = GoProfileGrooveWidth_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileGrooveWidth_SetSelectType(GoProfileGrooveWidth measurement, GoProfileGrooveSelectType selectType)
{
    GoProfileGrooveWidthClass* obj = GoProfileGrooveWidth_Cast_(measurement); 

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileGrooveWidth_SelectIndex(GoProfileGrooveWidth measurement)
{
    GoProfileGrooveWidthClass* obj = GoProfileGrooveWidth_Cast_(measurement);    return obj->selectIndex;
}

GoFx(kStatus) GoProfileGrooveWidth_SetSelectIndex(GoProfileGrooveWidth measurement, k32u selectIndex)
{
    GoProfileGrooveWidthClass* obj = GoProfileGrooveWidth_Cast_(measurement); 

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfileGrooveDepth, GoMeasurement)
    kAddVMethod(GoProfileGrooveDepth, GoMeasurement, VInit)
    kAddVMethod(GoProfileGrooveDepth, GoMeasurement, VRead)
    kAddVMethod(GoProfileGrooveDepth, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfileGrooveDepth_VInit(GoProfileGrooveDepth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileGrooveDepthClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_GROOVE_DEPTH, sensor, srcTool, kTRUE, alloc)); 

    obj->selectType = GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH;
    obj->selectIndex = 0;

    return kOK; 
}

GoFx(kStatus) GoProfileGrooveDepth_VRead(GoProfileGrooveDepth measurement, kXml xml, kXmlItem item)
{
    GoProfileGrooveDepthClass* obj = GoProfileGrooveDepth_Cast_(measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));    

    return kOK; 
}

GoFx(kStatus) GoProfileGrooveDepth_VWrite(GoProfileGrooveDepth measurement, kXml xml, kXmlItem item)
{
    GoProfileGrooveDepthClass* obj = GoProfileGrooveDepth_Cast_(measurement);

    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));    
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(GoProfileGrooveSelectType) GoProfileGrooveDepth_SelectType(GoProfileGrooveDepth measurement)
{
    GoProfileGrooveDepthClass* obj = GoProfileGrooveDepth_Cast_(measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileGrooveDepth_SetSelectType(GoProfileGrooveDepth measurement, GoProfileGrooveSelectType selectType)
{
    GoProfileGrooveDepthClass* obj = GoProfileGrooveDepth_Cast_(measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileGrooveDepth_SelectIndex(GoProfileGrooveDepth measurement)
{
    GoProfileGrooveDepthClass* obj = GoProfileGrooveDepth_Cast_(measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileGrooveDepth_SetSelectIndex(GoProfileGrooveDepth measurement, k32u selectIndex)
{
    GoProfileGrooveDepthClass* obj = GoProfileGrooveDepth_Cast_(measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfileIntersectX, GoMeasurement)
    kAddVMethod(GoProfileIntersectX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileIntersectX_VInit(GoProfileIntersectX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileIntersectXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_INTERSECT_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileIntersectZ, GoMeasurement)
    kAddVMethod(GoProfileIntersectZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileIntersectZ_VInit(GoProfileIntersectZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileIntersectZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_INTERSECT_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileIntersectAngle, GoMeasurement)
    kAddVMethod(GoProfileIntersectAngle, GoMeasurement, VInit)
    kAddVMethod(GoProfileIntersectAngle, GoMeasurement, VRead)
    kAddVMethod(GoProfileIntersectAngle, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfileIntersectAngle_VInit(GoProfileIntersectAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileIntersectAngleClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_INTERSECT_ANGLE, sensor, srcTool, kTRUE, alloc)); 

    obj->absolute = kFALSE;

    return kOK; 
}

GoFx(kStatus) GoProfileIntersectAngle_VRead(GoProfileIntersectAngle measurement, kXml xml, kXmlItem item)
{
    GoProfileIntersectAngleClass* obj = GoProfileIntersectAngle_Cast_(measurement); 

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->absolute));

    return kOK;  
}

GoFx(kStatus) GoProfileIntersectAngle_VWrite(GoProfileIntersectAngle measurement, kXml xml, kXmlItem item)
{
    GoProfileIntersectAngleClass* obj = GoProfileIntersectAngle_Cast_(measurement); 

    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->absolute));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(kBool) GoProfileIntersectAngle_AbsoluteEnabled(GoProfileIntersectAngle measurement)
{
    GoProfileIntersectAngleClass* obj = GoProfileIntersectAngle_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->absolute;
}

GoFx(kStatus) GoProfileIntersectAngle_EnableAbsolute(GoProfileIntersectAngle measurement, kBool absolute)
{
    GoProfileIntersectAngleClass* obj = GoProfileIntersectAngle_Cast_(measurement); 

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->absolute = absolute;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

kBeginClass(Go, GoProfileLineStdDev, GoMeasurement)
    kAddVMethod(GoProfileLineStdDev, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileLineStdDev_VInit(GoProfileLineStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileLineStdDevClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_STDDEV, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileLineMinError, GoMeasurement)
    kAddVMethod(GoProfileLineMinError, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileLineMinError_VInit(GoProfileLineMinError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileLineMinErrorClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileLineMaxError, GoMeasurement)
    kAddVMethod(GoProfileLineMaxError, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfileLineMaxError_VInit(GoProfileLineMaxError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileLineMaxErrorClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileLinePercentile, GoMeasurement)
    kAddVMethod(GoProfileLinePercentile, GoMeasurement, VInit)
    kAddVMethod(GoProfileLinePercentile, GoMeasurement, VRead)
    kAddVMethod(GoProfileLinePercentile, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfileLinePercentile_VInit(GoProfileLinePercentile measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileLinePercentileClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_PERCENTILE, sensor, srcTool, kTRUE, alloc)); 

    obj->percent = 95.0;

    return kOK; 
}

GoFx(kStatus) GoProfileLinePercentile_VRead(GoProfileLinePercentile measurement, kXml xml, kXmlItem item)
{
    GoProfileLinePercentileClass* obj = GoProfileLinePercentile_Cast_(measurement); 

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "Percent", &obj->percent));

    return kOK; 
}

GoFx(kStatus) GoProfileLinePercentile_VWrite(GoProfileLinePercentile measurement, kXml xml, kXmlItem item)
{
    GoProfileLinePercentileClass* obj = GoProfileLinePercentile_Cast_(measurement); 

    kCheck(kXml_SetChild64f(xml, item, "Percent", obj->percent));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(k64f) GoProfileLinePercentile_Percent(GoProfileLinePercentile measurement)
{
    GoProfileLinePercentileClass* obj = GoProfileLinePercentile_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->percent;
}

GoFx(kStatus) GoProfileLinePercentile_SetPercent(GoProfileLinePercentile measurement, k64f percent)
{
    GoProfileLinePercentileClass* obj = GoProfileLinePercentile_Cast_(measurement); 

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->percent = percent;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfilePanelGap, GoMeasurement)
    kAddVMethod(GoProfilePanelGap, GoMeasurement, VInit)
    kAddVMethod(GoProfilePanelGap, GoMeasurement, VRead)
    kAddVMethod(GoProfilePanelGap, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfilePanelGap_VInit(GoProfilePanelGap measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfilePanelGapClass* obj = measurement; 

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_GAP, sensor, srcTool, kTRUE, alloc)); 

    obj->axis = GO_PROFILE_GAP_AXIS_EDGE;

    return kOK; 
}

GoFx(kStatus) GoProfilePanelGap_VRead(GoProfilePanelGap measurement, kXml xml, kXmlItem item)
{
    GoProfilePanelGapClass* obj = GoProfilePanelGap_Cast_(measurement); 

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Axis", &obj->axis));

    return kOK; 
}

GoFx(kStatus) GoProfilePanelGap_VWrite(GoProfilePanelGap measurement, kXml xml, kXmlItem item)
{
    GoProfilePanelGapClass* obj = GoProfilePanelGap_Cast_(measurement); 

    kCheck(kXml_SetChild32s(xml, item, "Axis", obj->axis));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(GoProfileGapAxis) GoProfilePanelGap_Axis(GoProfilePanelGap measurement)
{
    GoProfilePanelGapClass* obj = GoProfilePanelGap_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->axis;
}

GoFx(kStatus) GoProfilePanelGap_SetAxis(GoProfilePanelGap measurement, GoProfileGapAxis axis)
{
    GoProfilePanelGapClass* obj = GoProfilePanelGap_Cast_(measurement); 

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->axis = axis;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfilePanelFlush, GoMeasurement)
    kAddVMethod(GoProfilePanelFlush, GoMeasurement, VInit)
    kAddVMethod(GoProfilePanelFlush, GoMeasurement, VRead)
    kAddVMethod(GoProfilePanelFlush, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfilePanelFlush_VInit(GoProfilePanelFlush measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfilePanelFlushClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_FLUSH, sensor, srcTool, kTRUE, alloc)); 

    obj->absolute = kFALSE;

    return kOK; 
}

GoFx(kStatus) GoProfilePanelFlush_VRead(GoProfilePanelFlush measurement, kXml xml, kXmlItem item)
{
    GoProfilePanelFlushClass* obj = GoProfilePanelFlush_Cast_(measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->absolute));

    return kOK; 
}

GoFx(kStatus) GoProfilePanelFlush_VWrite(GoProfilePanelFlush measurement, kXml xml, kXmlItem item)
{
    GoProfilePanelFlushClass* obj = GoProfilePanelFlush_Cast_(measurement);
    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->absolute));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(kBool) GoProfilePanelFlush_AbsoluteEnabled(GoProfilePanelFlush measurement)
{
    GoProfilePanelFlushClass* obj = GoProfilePanelFlush_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->absolute;
}

GoFx(kStatus) GoProfilePanelFlush_EnableAbsolute(GoProfilePanelFlush measurement, kBool absolute)
{
    GoProfilePanelFlushClass* obj = GoProfilePanelFlush_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->absolute = absolute;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfilePositionX, GoMeasurement)
    kAddVMethod(GoProfilePositionX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfilePositionX_VInit(GoProfilePositionX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfilePositionXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_POSITION_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfilePositionZ, GoMeasurement)
    kAddVMethod(GoProfilePositionZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoProfilePositionZ_VInit(GoProfilePositionZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfilePositionZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_POSITION_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoProfileStripX, GoMeasurement)
    kAddVMethod(GoProfileStripX, GoMeasurement, VInit)
    kAddVMethod(GoProfileStripX, GoMeasurement, VRead)
    kAddVMethod(GoProfileStripX, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfileStripX_VInit(GoProfileStripX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileStripXClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_STRIP_POSITION_X, sensor, srcTool, kTRUE, alloc)); 

    obj->location = GO_PROFILE_STRIP_LOCATION_LEFT;
    obj->selectType = GO_PROFILE_STRIP_SELECT_TYPE_BEST;
    obj->selectIndex = 0;

    return kOK; 
}

GoFx(kStatus) GoProfileStripX_VRead(GoProfileStripX measurement, kXml xml, kXmlItem item)
{
    GoProfileStripXClass* obj = GoProfileStripX_Cast_(measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));    

    return kOK; 
}

GoFx(kStatus) GoProfileStripX_VWrite(GoProfileStripX measurement, kXml xml, kXmlItem item)
{
    GoProfileStripXClass* obj = GoProfileStripX_Cast_(measurement);
    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));    
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(GoProfileGrooveLocation) GoProfileStripX_Location(GoProfileStripX measurement)
{
    GoProfileStripXClass* obj = GoProfileStripX_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoProfileStripX_SetLocation(GoProfileStripX measurement, GoProfileGrooveLocation location)
{
    GoProfileStripXClass* obj = GoProfileStripX_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileStripX_SelectType(GoProfileStripX measurement)
{
    GoProfileStripXClass* obj = GoProfileStripX_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileStripX_SetSelectType(GoProfileStripX measurement, GoProfileGrooveSelectType selectType)
{
    GoProfileStripXClass* obj = GoProfileStripX_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileStripX_SelectIndex(GoProfileStripX measurement)
{
    GoProfileStripXClass* obj = GoProfileStripX_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileStripX_SetSelectIndex(GoProfileStripX measurement, k32u selectIndex)
{
    GoProfileStripXClass* obj = GoProfileStripX_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfileStripZ, GoMeasurement)
    kAddVMethod(GoProfileStripZ, GoMeasurement, VInit)
    kAddVMethod(GoProfileStripZ, GoMeasurement, VRead)
    kAddVMethod(GoProfileStripZ, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfileStripZ_VInit(GoProfileStripZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileStripZClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z, sensor, srcTool, kTRUE, alloc)); 

    obj->location = GO_PROFILE_STRIP_LOCATION_LEFT;
    obj->selectType = GO_PROFILE_STRIP_SELECT_TYPE_BEST;
    obj->selectIndex = 0;

    return kOK; 
}

GoFx(kStatus) GoProfileStripZ_VRead(GoProfileStripZ measurement, kXml xml, kXmlItem item)
{
    GoProfileStripZClass* obj = GoProfileStripZ_Cast_(measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));    

    return kOK; 
}

GoFx(kStatus) GoProfileStripZ_VWrite(GoProfileStripZ measurement, kXml xml, kXmlItem item)
{
    GoProfileStripZClass* obj = GoProfileStripZ_Cast_(measurement);
    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));    
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(GoProfileGrooveLocation) GoProfileStripZ_Location(GoProfileStripZ measurement)
{
    GoProfileStripZClass* obj = GoProfileStripZ_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoProfileStripZ_SetLocation(GoProfileStripZ measurement, GoProfileGrooveLocation location)
{
    GoProfileStripZClass* obj = GoProfileStripZ_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileStripZ_SelectType(GoProfileStripZ measurement)
{
    GoProfileStripZClass* obj = GoProfileStripZ_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileStripZ_SetSelectType(GoProfileStripZ measurement, GoProfileGrooveSelectType selectType)
{
    GoProfileStripZClass* obj = GoProfileStripZ_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileStripZ_SelectIndex(GoProfileStripZ measurement)
{
    GoProfileStripZClass* obj = GoProfileStripZ_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileStripZ_SetSelectIndex(GoProfileStripZ measurement, k32u selectIndex)
{
    GoProfileStripZClass* obj = GoProfileStripZ_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfileStripWidth, GoMeasurement)
    kAddVMethod(GoProfileStripWidth, GoMeasurement, VInit)
    kAddVMethod(GoProfileStripWidth, GoMeasurement, VRead)
    kAddVMethod(GoProfileStripWidth, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfileStripWidth_VInit(GoProfileStripWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileStripWidthClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_STRIP_WIDTH, sensor, srcTool, kTRUE, alloc)); 

    obj->selectType = GO_PROFILE_STRIP_SELECT_TYPE_BEST;
    obj->selectIndex = 0;

    return kOK; 
}

GoFx(kStatus) GoProfileStripWidth_VRead(GoProfileStripWidth measurement, kXml xml, kXmlItem item)
{
    GoProfileStripWidthClass* obj = GoProfileStripWidth_Cast_(measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));    

    return kOK; 
}

GoFx(kStatus) GoProfileStripWidth_VWrite(GoProfileStripWidth measurement, kXml xml, kXmlItem item)
{
    GoProfileStripWidthClass* obj = GoProfileStripWidth_Cast_(measurement);
    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));    
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(GoProfileGrooveSelectType) GoProfileStripWidth_SelectType(GoProfileStripWidth measurement)
{
    GoProfileStripWidthClass* obj = GoProfileStripWidth_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileStripWidth_SetSelectType(GoProfileStripWidth measurement, GoProfileGrooveSelectType selectType)
{
    GoProfileStripWidthClass* obj = GoProfileStripWidth_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileStripWidth_SelectIndex(GoProfileStripWidth measurement)
{
    GoProfileStripWidthClass* obj = GoProfileStripWidth_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileStripWidth_SetSelectIndex(GoProfileStripWidth measurement, k32u selectIndex)
{
    GoProfileStripWidthClass* obj = GoProfileStripWidth_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClass(Go, GoProfileStripHeight, GoMeasurement)
    kAddVMethod(GoProfileStripHeight, GoMeasurement, VInit)
    kAddVMethod(GoProfileStripHeight, GoMeasurement, VRead)
    kAddVMethod(GoProfileStripHeight, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoProfileStripHeight_VInit(GoProfileStripHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoProfileStripHeightClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_STRIP_HEIGHT, sensor, srcTool, kTRUE, alloc)); 

    obj->location = GO_PROFILE_STRIP_LOCATION_LEFT;
    obj->selectType = GO_PROFILE_STRIP_SELECT_TYPE_BEST;
    obj->selectIndex = 0;

    return kOK; 
}

GoFx(kStatus) GoProfileStripHeight_VRead(GoProfileStripHeight measurement, kXml xml, kXmlItem item)
{
    GoProfileStripHeightClass* obj = GoProfileStripHeight_Cast_(measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));    

    return kOK; 
}

GoFx(kStatus) GoProfileStripHeight_VWrite(GoProfileStripHeight measurement, kXml xml, kXmlItem item)
{
    GoProfileStripHeightClass* obj = GoProfileStripHeight_Cast_(measurement);
    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));    
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(GoProfileGrooveLocation) GoProfileStripHeight_Location(GoProfileStripHeight measurement)
{
    GoProfileStripHeightClass* obj = GoProfileStripHeight_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoProfileStripHeight_SetLocation(GoProfileStripHeight measurement, GoProfileGrooveLocation location)
{
    GoProfileStripHeightClass* obj = GoProfileStripHeight_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileStripHeight_SelectType(GoProfileStripHeight measurement)
{
    GoProfileStripHeightClass* obj = GoProfileStripHeight_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileStripHeight_SetSelectType(GoProfileStripHeight measurement, GoProfileGrooveSelectType selectType)
{
    GoProfileStripHeightClass* obj = GoProfileStripHeight_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileStripHeight_SelectIndex(GoProfileStripHeight measurement)
{
    GoProfileStripHeightClass* obj = GoProfileStripHeight_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileStripHeight_SetSelectIndex(GoProfileStripHeight measurement, k32u selectIndex)
{
    GoProfileStripHeightClass* obj = GoProfileStripHeight_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


/* Surface tool measurements */

kBeginClass(Go, GoSurfaceBoxX, GoMeasurement)
    kAddVMethod(GoSurfaceBoxX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceBoxX_VInit(GoSurfaceBoxX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceBoxXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceBoxY, GoMeasurement)
    kAddVMethod(GoSurfaceBoxY, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceBoxY_VInit(GoSurfaceBoxY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceBoxYClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceBoxZ, GoMeasurement)
    kAddVMethod(GoSurfaceBoxZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceBoxZ_VInit(GoSurfaceBoxZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceBoxZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceBoxWidth, GoMeasurement)
    kAddVMethod(GoSurfaceBoxWidth, GoMeasurement, VInit)
kEndClass()


GoFx(kStatus) GoSurfaceBoxWidth_VInit(GoSurfaceBoxWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceBoxWidthClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_WIDTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceBoxLength, GoMeasurement)
    kAddVMethod(GoSurfaceBoxLength, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceBoxLength_VInit(GoSurfaceBoxLength measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceBoxLengthClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_LENGTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceBoxHeight, GoMeasurement)
    kAddVMethod(GoSurfaceBoxHeight, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceBoxHeight_VInit(GoSurfaceBoxHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceBoxHeightClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_HEIGHT, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceBoxZAngle, GoMeasurement)
    kAddVMethod(GoSurfaceBoxZAngle, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceBoxZAngle_VInit(GoSurfaceBoxZAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceBoxZAngleClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_ZANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClass(Go, GoSurfaceBoxGlobalX, GoMeasurement)
    kAddVMethod(GoSurfaceBoxGlobalX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceBoxGlobalX_VInit(GoSurfaceBoxGlobalX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceBoxGlobalXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceBoxGlobalY, GoMeasurement)
    kAddVMethod(GoSurfaceBoxGlobalY, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceBoxGlobalY_VInit(GoSurfaceBoxGlobalY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceBoxGlobalYClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClass(Go, GoSurfaceBoxGlobalZAngle, GoMeasurement)
    kAddVMethod(GoSurfaceBoxGlobalZAngle, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceBoxGlobalZAngle_VInit(GoSurfaceBoxGlobalY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceBoxGlobalYClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Z_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceCountersunkHoleX, GoMeasurement)
kAddVMethod(GoSurfaceCountersunkHoleX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceCountersunkHoleX_VInit(GoSurfaceCountersunkHoleX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceCountersunkHoleXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClass(Go, GoSurfaceCountersunkHoleXAngle, GoMeasurement)
kAddVMethod(GoSurfaceCountersunkHoleXAngle, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceCountersunkHoleXAngle_VInit(GoSurfaceCountersunkHoleXAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceCountersunkHoleXAngleClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClass(Go, GoSurfaceCountersunkHoleY, GoMeasurement)
kAddVMethod(GoSurfaceCountersunkHoleY, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceCountersunkHoleY_VInit(GoSurfaceCountersunkHoleY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceCountersunkHoleYClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClass(Go, GoSurfaceCountersunkHoleYAngle, GoMeasurement)
kAddVMethod(GoSurfaceCountersunkHoleYAngle, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceCountersunkHoleYAngle_VInit(GoSurfaceCountersunkHoleYAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceCountersunkHoleYAngleClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceCountersunkHoleZ, GoMeasurement)
kAddVMethod(GoSurfaceCountersunkHoleZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceCountersunkHoleZ_VInit(GoSurfaceCountersunkHoleZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceCountersunkHoleZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClass(Go, GoSurfaceCountersunkHoleBevelAngle, GoMeasurement)
kAddVMethod(GoSurfaceCountersunkHoleBevelAngle, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceCountersunkHoleBevelAngle_VInit(GoSurfaceCountersunkHoleBevelAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceCountersunkHoleBevelAngleClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceCountersunkHoleBevelRadius, GoMeasurement)
kAddVMethod(GoSurfaceCountersunkHoleBevelRadius, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceCountersunkHoleBevelRadius_VInit(GoSurfaceCountersunkHoleBevelRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceCountersunkHoleBevelRadiusClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_RADIUS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceCountersunkHoleDepth, GoMeasurement)
kAddVMethod(GoSurfaceCountersunkHoleDepth, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceCountersunkHoleDepth_VInit(GoSurfaceCountersunkHoleDepth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceCountersunkHoleDepthClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_DEPTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClass(Go, GoSurfaceCountersunkHoleOuterRadius, GoMeasurement)
kAddVMethod(GoSurfaceCountersunkHoleOuterRadius, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceCountersunkHoleOuterRadius_VInit(GoSurfaceCountersunkHoleOuterRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceCountersunkHoleOuterRadiusClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_OUTER_RADIUS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClass(Go, GoSurfaceEllipseMajor, GoMeasurement)
    kAddVMethod(GoSurfaceEllipseMajor, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceEllipseMajor_VInit(GoSurfaceEllipseMajor measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceEllipseMajorClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_ELLIPSE_MINOR, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceEllipseMinor, GoMeasurement)
    kAddVMethod(GoSurfaceEllipseMinor, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceEllipseMinor_VInit(GoSurfaceEllipseMinor measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceEllipseMinorClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_ELLIPSE_MAJOR, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceEllipseRatio, GoMeasurement)
    kAddVMethod(GoSurfaceEllipseRatio, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceEllipseRatio_VInit(GoSurfaceEllipseRatio measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceEllipseRatioClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_ELLIPSE_RATIO, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceEllipseZAngle, GoMeasurement)
    kAddVMethod(GoSurfaceEllipseZAngle, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceEllipseZAngle_VInit(GoSurfaceEllipseZAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceEllipseZAngleClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_ELLIPSE_ZANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceHoleX, GoMeasurement)
    kAddVMethod(GoSurfaceHoleX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceHoleX_VInit(GoSurfaceHoleX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceHoleXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_HOLE_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceHoleY, GoMeasurement)
    kAddVMethod(GoSurfaceHoleY, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceHoleY_VInit(GoSurfaceHoleY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceHoleYClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_HOLE_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceHoleZ, GoMeasurement)
    kAddVMethod(GoSurfaceHoleZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceHoleZ_VInit(GoSurfaceHoleZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceHoleZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_HOLE_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceHoleRadius, GoMeasurement)
    kAddVMethod(GoSurfaceHoleRadius, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceHoleRadius_VInit(GoSurfaceHoleRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceHoleRadiusClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_HOLE_RADIUS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceOpeningX, GoMeasurement)
    kAddVMethod(GoSurfaceOpeningX, GoMeasurement, VInit)   
kEndClass()

GoFx(kStatus) GoSurfaceOpeningX_VInit(GoSurfaceOpeningX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceOpeningXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceOpeningY, GoMeasurement)
    kAddVMethod(GoSurfaceOpeningY, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceOpeningY_VInit(GoSurfaceOpeningY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceOpeningYClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceOpeningZ, GoMeasurement)
    kAddVMethod(GoSurfaceOpeningZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceOpeningZ_VInit(GoSurfaceOpeningZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceOpeningZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceOpeningWidth, GoMeasurement)
    kAddVMethod(GoSurfaceOpeningWidth, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceOpeningWidth_VInit(GoSurfaceOpeningWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceOpeningWidthClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_WIDTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceOpeningLength, GoMeasurement)
    kAddVMethod(GoSurfaceOpeningLength, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceOpeningLength_VInit(GoSurfaceOpeningLength measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceOpeningLengthClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_LENGTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceOpeningAngle, GoMeasurement)
    kAddVMethod(GoSurfaceOpeningAngle, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceOpeningAngle_VInit(GoSurfaceOpeningAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceOpeningAngleClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfacePlaneXAngle, GoMeasurement) 
    kAddVMethod(GoSurfacePlaneXAngle, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfacePlaneXAngle_VInit(GoSurfacePlaneXAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfacePlaneXAngleClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_X_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfacePlaneYAngle, GoMeasurement)
    kAddVMethod(GoSurfacePlaneYAngle, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfacePlaneYAngle_VInit(GoSurfacePlaneYAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfacePlaneYAngleClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_Y_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfacePlaneZOffset, GoMeasurement)
    kAddVMethod(GoSurfacePlaneZOffset, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfacePlaneZOffset_VInit(GoSurfacePlaneZOffset measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfacePlaneZOffsetClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_Z_OFFSET, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfacePositionX, GoMeasurement)
    kAddVMethod(GoSurfacePositionX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfacePositionX_VInit(GoSurfacePositionX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfacePositionXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_POSITION_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfacePositionY, GoMeasurement)
    kAddVMethod(GoSurfacePositionY, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfacePositionY_VInit(GoSurfacePositionY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfacePositionYClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_POSITION_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfacePositionZ, GoMeasurement)
    kAddVMethod(GoSurfacePositionZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfacePositionZ_VInit(GoSurfacePositionZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfacePositionZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_POSITION_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceStudBaseX, GoMeasurement)
    kAddVMethod(GoSurfaceStudBaseX, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceStudBaseX_VInit(GoSurfaceStudBaseX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceStudBaseXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_BASE_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceStudBaseY, GoMeasurement)
    kAddVMethod(GoSurfaceStudBaseY, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceStudBaseY_VInit(GoSurfaceStudBaseY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceStudBaseYClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_BASE_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceStudBaseZ, GoMeasurement)
    kAddVMethod(GoSurfaceStudBaseZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceStudBaseZ_VInit(GoSurfaceStudBaseZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceStudBaseZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_BASE_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceStudTipX, GoMeasurement)
    kAddVMethod(GoSurfaceStudTipX, GoMeasurement, VInit)
kEndClass()


GoFx(kStatus) GoSurfaceStudTipX_VInit(GoSurfaceStudTipX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceStudTipXClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_TIP_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceStudTipY, GoMeasurement)
    kAddVMethod(GoSurfaceStudTipY, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceStudTipY_VInit(GoSurfaceStudTipY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceStudTipYClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_TIP_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceStudTipZ, GoMeasurement)
    kAddVMethod(GoSurfaceStudTipZ, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceStudTipZ_VInit(GoSurfaceStudTipZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceStudTipZClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_TIP_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceStudRadius, GoMeasurement)
    kAddVMethod(GoSurfaceStudRadius, GoMeasurement, VInit)
    kAddVMethod(GoSurfaceStudRadius, GoMeasurement, VRead)
    kAddVMethod(GoSurfaceStudRadius, GoMeasurement, VWrite)
kEndClass()


GoFx(kStatus) GoSurfaceStudRadius_VInit(GoSurfaceStudRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceStudRadiusClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_RADIUS, sensor, srcTool, kTRUE, alloc)); 

    obj->radiusOffset = 0.0;

    return kOK; 
}

GoFx(kStatus) GoSurfaceStudRadius_VRead(GoSurfaceStudRadius measurement, kXml xml, kXmlItem item)
{
    GoSurfaceStudRadiusClass* obj = GoSurfaceStudRadius_Cast_(measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "RadiusOffset", &obj->radiusOffset));

    return kOK; 
}

GoFx(kStatus) GoSurfaceStudRadius_VWrite(GoSurfaceStudRadius measurement, kXml xml, kXmlItem item)
{
    GoSurfaceStudRadiusClass* obj = GoSurfaceStudRadius_Cast_(measurement);
    kCheck(kXml_SetChild64f(xml, item, "RadiusOffset", obj->radiusOffset));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

GoFx(k64f) GoSurfaceStudRadius_RadiusOffset(GoSurfaceStudRadius measurement)
{
    GoSurfaceStudRadiusClass* obj = GoSurfaceStudRadius_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->radiusOffset;
}

GoFx(kStatus) GoSurfaceStudRadius_SetRadiusOffset(GoSurfaceStudRadius measurement, k64f offset)
{
    GoSurfaceStudRadiusClass* obj = GoSurfaceStudRadius_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->radiusOffset = offset;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

kBeginClass(Go, GoSurfaceVolumeVolume, GoMeasurement)
    kAddVMethod(GoSurfaceVolumeVolume, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceVolumeVolume_VInit(GoSurfaceVolumeVolume measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceVolumeVolumeClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type,  GO_MEASUREMENT_SURFACE_VOLUME_VOLUME, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceVolumeArea, GoMeasurement)
    kAddVMethod(GoSurfaceVolumeArea, GoMeasurement, VInit)
kEndClass()

GoFx(kStatus) GoSurfaceVolumeArea_VInit(GoSurfaceVolumeArea measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceVolumeAreaClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_VOLUME_AREA, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClass(Go, GoSurfaceVolumeThickness, GoMeasurement)
    kAddVMethod(GoSurfaceVolumeThickness, GoMeasurement, VInit)
    kAddVMethod(GoSurfaceVolumeThickness, GoMeasurement, VRead)
    kAddVMethod(GoSurfaceVolumeThickness, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoSurfaceVolumeThickness_VInit(GoSurfaceVolumeThickness measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoSurfaceVolumeThicknessClass* obj = measurement;
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_VOLUME_THICKNESS, sensor, srcTool, kTRUE, alloc)); 

    obj->location = GO_SURFACE_LOCATION_TYPE_MAX;

    return kOK; 
}

GoFx(kStatus) GoSurfaceVolumeThickness_VRead(GoSurfaceVolumeThickness measurement, kXml xml, kXmlItem item)
{
    GoSurfaceVolumeThicknessClass* obj = GoSurfaceVolumeThickness_Cast_(measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));

    return kOK; 
}

GoFx(kStatus) GoSurfaceVolumeThickness_VWrite(GoSurfaceVolumeThickness measurement, kXml xml, kXmlItem item)
{
    GoSurfaceVolumeThicknessClass* obj = GoSurfaceVolumeThickness_Cast_(measurement);
    
    kCheck(GoMeasurement_VWrite(measurement, xml, item));
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));
    
    return kOK; 
}

GoFx(GoSurfaceLocation) GoSurfaceVolumeThickness_Location(GoSurfaceVolumeThickness measurement)
{
    GoSurfaceVolumeThicknessClass* obj = GoSurfaceVolumeThickness_Cast_(measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoSurfaceVolumeThickness_SetLocation(GoSurfaceVolumeThickness measurement, GoSurfaceLocation location)
{
    GoSurfaceVolumeThicknessClass* obj = GoSurfaceVolumeThickness_Cast_(measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


/* Script tool measurement */

kBeginClass(Go, GoScriptOutput, GoMeasurement)
    kAddVMethod(GoScriptOutput, GoMeasurement, VInit)
    kAddVMethod(GoScriptOutput, GoMeasurement, VRead)
    kAddVMethod(GoScriptOutput, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoScriptOutput_VInit(GoScriptOutput measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoScriptOutputClass* obj = measurement;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SCRIPT_OUTPUT, sensor, srcTool, kFALSE, alloc)); 
    kInitFields_(GoScriptOutput, measurement);

    return kOK; 
}

GoFx(kStatus) GoScriptOutput_VRead(GoScriptOutput measurement, kXml xml, kXmlItem item)
{
    GoScriptOutputClass* obj = GoScriptOutput_Cast_(measurement);
    
    kCheck(GoMeasurement_VRead(measurement, xml, item));

    return kOK; 
}

GoFx(kStatus) GoScriptOutput_VWrite(GoScriptOutput measurement, kXml xml, kXmlItem item)
{
    GoScriptOutputClass* obj = GoScriptOutput_Cast_(measurement);

    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK; 
}

kBeginClass(Go, GoExtMeasurement, GoMeasurement)
    kAddVMethod(GoExtMeasurement, GoMeasurement, VInit)
    kAddVMethod(GoExtMeasurement, GoMeasurement, VRead)
    kAddVMethod(GoExtMeasurement, GoMeasurement, VWrite)
    kAddVMethod(GoExtMeasurement, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoExtMeasurement_VInit(GoExtMeasurement measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoExtMeasurementClass* obj = measurement;
    kStatus exception;

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_EXTENSIBLE, sensor, srcTool, kTRUE, alloc)); 
    kCheck(kStrCopy(obj->type, 64, ""));
    
    kTry
    {
        kTest(kArrayList_Construct(&obj->customParameters, kTypeOf(GoExtParam), 0, alloc));
    }
    kCatch(&exception)
    {
        GoExtMeasurement_VRelease(measurement);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoExtMeasurement_VRead(GoExtMeasurement measurement, kXml xml, kXmlItem item)
{
    GoExtMeasurementClass* obj = GoExtMeasurement_Cast_(measurement);
    kSize i;
    kXmlItem paramsItem = kNULL;
    kStatus exception;
    kType paramType;
    k32s typeVal;

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    
    kCheck(kXml_AttrText(xml, item, "type", obj->type, 64));
    kCheck(!kIsNull(paramsItem = kXml_Child(xml, item, "Parameters"))); 
    kCheck(kArrayList_Purge(obj->customParameters));
    
    for (i = 0; i < kXml_ChildCount(xml, paramsItem); i++)
    {
        GoExtParam param = kNULL;
        kXmlItem paramItem = kXml_ChildAt(xml, paramsItem, i);

        kCheck(kXml_Attr32s(xml, paramItem, "type", &typeVal));
        paramType = GoExtParam_GetKType(typeVal);

        kTry
        {
            kTest(GoExtParam_Construct(&param, paramType, obj->base.sensor, kObject_Alloc(measurement)));
            kTest(GoExtParam_Read(param, xml, paramItem));
            kTest(kArrayList_Add(obj->customParameters, &param));
        }
        kCatch(&exception)
        {
            kDestroyRef(&param);
            kEndCatch(exception);
        }
    }

    return kOK; 
}

GoFx(kStatus) GoExtMeasurement_VWrite(GoExtMeasurement measurement, kXml xml, kXmlItem item)
{
    GoExtMeasurementClass* obj = GoExtMeasurement_Cast_(measurement);
    kSize i;
    kXmlItem paramsItem = kNULL;

    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    kCheck(kXml_SetAttrText(xml, item, "type", obj->type));
    kCheck(kXml_AddItem(xml, item, "Parameters", &paramsItem));

    for (i = 0; i < kArrayList_Count(obj->customParameters); i++)
    {
        kCheck(GoExtParam_Write(*(GoExtParam*)kArrayList_At(obj->customParameters, i), xml, paramsItem));
    }

    return kOK; 
}

GoFx(kStatus) GoExtMeasurement_VRelease(GoExtMeasurement measurement)
{
    GoExtMeasurementClass* obj = GoExtMeasurement_Cast_(measurement);
    
    kCheck(kDisposeRef(&obj->customParameters));

    return GoMeasurement_VRelease(measurement);
}

GoFx(const kChar*) GoExtMeasurement_Type(GoExtMeasurement measurement)
{
    GoExtMeasurementClass* obj = GoExtMeasurement_Cast_(measurement);

    return obj->type;
}

GoFx(kSize) GoExtMeasurement_CustomParameterCount(GoExtMeasurement measurement)
{
    GoExtMeasurementClass* obj = GoExtMeasurement_Cast_(measurement);

    return kArrayList_Count(obj->customParameters);
}

GoFx(GoExtParam) GoExtMeasurement_CustomParameterAt(GoExtMeasurement measurement, kSize index)
{
    GoExtMeasurementClass* obj = GoExtMeasurement_Cast_(measurement);

    if (index >= kArrayList_Count(obj->customParameters))
    {
        return kNULL;
    }

    return *(GoExtParam*)kArrayList_At(obj->customParameters, index);
}

GoFx(kStatus) GoMeasurements_ParseType(const kChar* toolName, const kChar* measurementName, kType* type)
{
    kChar* name = kNULL;
    kSize capacity = kStrLength(toolName) + kStrLength(measurementName) + 1;

    kCheck(kMemAllocZero(sizeof(kChar) * capacity, &name));
    kCheck(kStrCopy(name, capacity, toolName));
    kCheck(kStrCat(name, capacity, measurementName));

    if           (strcmp(name, "RangePositionZ") == 0)                   *type = kTypeOf(GoRangePositionZ); 
    else if      (strcmp(name, "RangeThicknessThickness") == 0)          *type = kTypeOf(GoRangeThicknessThickness); 

    else if      (strcmp(name, "ProfileAreaArea") == 0)                  *type = kTypeOf(GoProfileAreaArea); 
    else if      (strcmp(name, "ProfileAreaCentroidX") == 0)             *type = kTypeOf(GoProfileAreaCentroidX); 
    else if      (strcmp(name, "ProfileAreaCentroidZ") == 0)             *type = kTypeOf(GoProfileAreaCentroidZ); 

    else if      (strcmp(name, "ProfileBoundingBoxX") == 0)              *type = kTypeOf(GoProfileBoxX); 
    else if      (strcmp(name, "ProfileBoundingBoxZ") == 0)              *type = kTypeOf(GoProfileBoxZ); 
    else if      (strcmp(name, "ProfileBoundingBoxWidth") == 0)          *type = kTypeOf(GoProfileBoxWidth); 
    else if      (strcmp(name, "ProfileBoundingBoxHeight") == 0)         *type = kTypeOf(GoProfileBoxHeight); 
    else if      (strcmp(name, "ProfileBoundingBoxGlobalX") == 0)        *type = kTypeOf(GoProfileBoxGlobalX); 

    else if      (strcmp(name, "ProfileCircleX") == 0)                   *type = kTypeOf(GoProfileCircleX); 
    else if      (strcmp(name, "ProfileCircleZ") == 0)                   *type = kTypeOf(GoProfileCircleZ); 
    else if      (strcmp(name, "ProfileCircleRadius") == 0)              *type = kTypeOf(GoProfileCircleRadius); 

    else if      (strcmp(name, "ProfileDimensionWidth") == 0)            *type = kTypeOf(GoProfileDimWidth);    
    else if      (strcmp(name, "ProfileDimensionHeight") == 0)           *type = kTypeOf(GoProfileDimHeight); 
    else if      (strcmp(name, "ProfileDimensionDistance") == 0)         *type = kTypeOf(GoProfileDimDistance); 
    else if      (strcmp(name, "ProfileDimensionCenterX") == 0)          *type = kTypeOf(GoProfileDimCenterX); 
    else if      (strcmp(name, "ProfileDimensionCenterZ") == 0)          *type = kTypeOf(GoProfileDimCenterZ); 

    else if      (strcmp(name, "ProfileGrooveX") == 0)                   *type = kTypeOf(GoProfileGrooveX); 
    else if      (strcmp(name, "ProfileGrooveZ") == 0)                   *type = kTypeOf(GoProfileGrooveZ); 
    else if      (strcmp(name, "ProfileGrooveWidth") == 0)               *type = kTypeOf(GoProfileGrooveWidth); 
    else if      (strcmp(name, "ProfileGrooveDepth") == 0)               *type = kTypeOf(GoProfileGrooveDepth); 

    else if      (strcmp(name, "ProfileIntersectX") == 0)                *type = kTypeOf(GoProfileIntersectX); 
    else if      (strcmp(name, "ProfileIntersectZ") == 0)                *type = kTypeOf(GoProfileIntersectZ); 
    else if      (strcmp(name, "ProfileIntersectAngle") == 0)            *type = kTypeOf(GoProfileIntersectAngle); 

    else if      (strcmp(name, "ProfileLineStdDev") == 0)                *type = kTypeOf(GoProfileLineStdDev); 
    else if      (strcmp(name, "ProfileLineMaxError") == 0)              *type = kTypeOf(GoProfileLineMaxError); 
    else if      (strcmp(name, "ProfileLineMinError") == 0)              *type = kTypeOf(GoProfileLineMinError); 
    else if      (strcmp(name, "ProfileLinePercentile") == 0)            *type = kTypeOf(GoProfileLinePercentile); 

    else if      (strcmp(name, "ProfilePanelGap") == 0)                  *type = kTypeOf(GoProfilePanelGap); 
    else if      (strcmp(name, "ProfilePanelFlush") == 0)                *type = kTypeOf(GoProfilePanelFlush); 

    else if      (strcmp(name, "ProfilePositionX") == 0)                 *type = kTypeOf(GoProfilePositionX); 
    else if      (strcmp(name, "ProfilePositionZ") == 0)                 *type = kTypeOf(GoProfilePositionZ); 

    else if      (strcmp(name, "ProfileStripX") == 0)                    *type = kTypeOf(GoProfileStripX); 
    else if      (strcmp(name, "ProfileStripZ") == 0)                    *type = kTypeOf(GoProfileStripZ); 
    else if      (strcmp(name, "ProfileStripWidth") == 0)                *type = kTypeOf(GoProfileStripWidth); 
    else if      (strcmp(name, "ProfileStripHeight") == 0)               *type = kTypeOf(GoProfileStripHeight); 

    else if      (strcmp(name, "SurfaceBoundingBoxX") == 0)              *type = kTypeOf(GoSurfaceBoxX); 
    else if      (strcmp(name, "SurfaceBoundingBoxY") == 0)              *type = kTypeOf(GoSurfaceBoxY); 
    else if      (strcmp(name, "SurfaceBoundingBoxZ") == 0)              *type = kTypeOf(GoSurfaceBoxZ); 
    else if      (strcmp(name, "SurfaceBoundingBoxWidth") == 0)          *type = kTypeOf(GoSurfaceBoxWidth); 
    else if      (strcmp(name, "SurfaceBoundingBoxLength") == 0)         *type = kTypeOf(GoSurfaceBoxLength); 
    else if      (strcmp(name, "SurfaceBoundingBoxHeight") == 0)         *type = kTypeOf(GoSurfaceBoxHeight); 
    else if      (strcmp(name, "SurfaceBoundingBoxZAngle") == 0)         *type = kTypeOf(GoSurfaceBoxZAngle); 
    else if      (strcmp(name, "SurfaceBoundingBoxGlobalX") == 0)        *type = kTypeOf(GoSurfaceBoxGlobalX); 
    else if      (strcmp(name, "SurfaceBoundingBoxGlobalY") == 0)        *type = kTypeOf(GoSurfaceBoxGlobalY); 
    else if      (strcmp(name, "SurfaceBoundingBoxGlobalZAngle") == 0)   *type = kTypeOf(GoSurfaceBoxGlobalZAngle); 

    else if      (strcmp(name, "SurfaceCsHoleX") == 0)                   *type = kTypeOf(GoSurfaceCountersunkHoleX);
    else if      (strcmp(name, "SurfaceCsHoleY") == 0)                   *type = kTypeOf(GoSurfaceCountersunkHoleY);
    else if      (strcmp(name, "SurfaceCsHoleZ") == 0)                   *type = kTypeOf(GoSurfaceCountersunkHoleZ);
    else if      (strcmp(name, "SurfaceCsHoleOuterRadius") == 0)         *type = kTypeOf(GoSurfaceCountersunkHoleOuterRadius);
    else if      (strcmp(name, "SurfaceCsHoleDepth") == 0)               *type = kTypeOf(GoSurfaceCountersunkHoleDepth);
    else if      (strcmp(name, "SurfaceCsHoleBevelAngle") == 0)          *type = kTypeOf(GoSurfaceCountersunkHoleBevelAngle);
    else if      (strcmp(name, "SurfaceCsHoleBevelRadius") == 0)         *type = kTypeOf(GoSurfaceCountersunkHoleBevelRadius);
    else if      (strcmp(name, "SurfaceCsHoleXAngle") == 0)              *type = kTypeOf(GoSurfaceCountersunkHoleXAngle);
    else if      (strcmp(name, "SurfaceCsHoleYAngle") == 0)              *type = kTypeOf(GoSurfaceCountersunkHoleYAngle);   

    else if      (strcmp(name, "SurfaceEllipseMajor") == 0)              *type = kTypeOf(GoSurfaceEllipseMajor);
    else if      (strcmp(name, "SurfaceEllipseMinor") == 0)              *type = kTypeOf(GoSurfaceEllipseMinor); 
    else if      (strcmp(name, "SurfaceEllipseRatio") == 0)              *type = kTypeOf(GoSurfaceEllipseRatio); 
    else if      (strcmp(name, "SurfaceEllipseZAngle") == 0)             *type = kTypeOf(GoSurfaceEllipseZAngle); 

    else if      (strcmp(name, "SurfaceHoleX") == 0)                     *type = kTypeOf(GoSurfaceHoleX); 
    else if      (strcmp(name, "SurfaceHoleY") == 0)                     *type = kTypeOf(GoSurfaceHoleY); 
    else if      (strcmp(name, "SurfaceHoleZ") == 0)                     *type = kTypeOf(GoSurfaceHoleZ); 
    else if      (strcmp(name, "SurfaceHoleRadius") == 0)                *type = kTypeOf(GoSurfaceHoleRadius); 

    else if      (strcmp(name, "SurfaceOpeningX") == 0)                  *type = kTypeOf(GoSurfaceOpeningX); 
    else if      (strcmp(name, "SurfaceOpeningY") == 0)                  *type = kTypeOf(GoSurfaceOpeningY); 
    else if      (strcmp(name, "SurfaceOpeningZ") == 0)                  *type = kTypeOf(GoSurfaceOpeningZ); 
    else if      (strcmp(name, "SurfaceOpeningWidth") == 0)              *type = kTypeOf(GoSurfaceOpeningWidth); 
    else if      (strcmp(name, "SurfaceOpeningLength") == 0)             *type = kTypeOf(GoSurfaceOpeningLength); 
    else if      (strcmp(name, "SurfaceOpeningAngle") == 0)              *type = kTypeOf(GoSurfaceOpeningAngle); 

    else if      (strcmp(name, "SurfacePlaneXAngle") == 0)               *type = kTypeOf(GoSurfacePlaneXAngle); 
    else if      (strcmp(name, "SurfacePlaneYAngle") == 0)               *type = kTypeOf(GoSurfacePlaneYAngle); 
    else if      (strcmp(name, "SurfacePlaneZOffset") == 0)              *type = kTypeOf(GoSurfacePlaneZOffset); 

    else if      (strcmp(name, "SurfacePositionX") == 0)                 *type = kTypeOf(GoSurfacePositionX); 
    else if      (strcmp(name, "SurfacePositionY") == 0)                 *type = kTypeOf(GoSurfacePositionY); 
    else if      (strcmp(name, "SurfacePositionZ") == 0)                 *type = kTypeOf(GoSurfacePositionZ); 

    else if      (strcmp(name, "SurfaceStudBaseX") == 0)                 *type = kTypeOf(GoSurfaceStudBaseX); 
    else if      (strcmp(name, "SurfaceStudBaseY") == 0)                 *type = kTypeOf(GoSurfaceStudBaseY); 
    else if      (strcmp(name, "SurfaceStudBaseZ") == 0)                 *type = kTypeOf(GoSurfaceStudBaseZ); 
    else if      (strcmp(name, "SurfaceStudTipX") == 0)                  *type = kTypeOf(GoSurfaceStudTipX); 
    else if      (strcmp(name, "SurfaceStudTipY") == 0)                  *type = kTypeOf(GoSurfaceStudTipY); 
    else if      (strcmp(name, "SurfaceStudTipZ") == 0)                  *type = kTypeOf(GoSurfaceStudTipZ); 
    else if      (strcmp(name, "SurfaceStudRadius") == 0)                *type = kTypeOf(GoSurfaceStudRadius); 

    else if      (strcmp(name, "SurfaceVolumeVolume") == 0)              *type = kTypeOf(GoSurfaceVolumeVolume); 
    else if      (strcmp(name, "SurfaceVolumeArea") == 0)                *type = kTypeOf(GoSurfaceVolumeArea); 
    else if      (strcmp(name, "SurfaceVolumeThickness") == 0)           *type = kTypeOf(GoSurfaceVolumeThickness); 

    else if      (strcmp(name, "ScriptOutput") == 0)                     *type = kTypeOf(GoScriptOutput);

    else if      (strcmp(name, "CustomCustom") == 0)                     *type = kTypeOf(GoExtMeasurement);

    else                                                                 return kERROR_PARAMETER; 

    if(name)    kCheck(kMemFree(name));

    return kOK; 
}

GoFx(kStatus) GoMeasurements_FormatType(GoMeasurement measurement, kChar* measurementName, kSize capacity)
{
    const kChar* output = kNULL; 

    if           (kObject_Is(measurement, kTypeOf(GoRangePositionZ)))             output = "Z";

    else if      (kObject_Is(measurement, kTypeOf(GoRangeThicknessThickness)))    output = "Thickness";

    else if      (kObject_Is(measurement, kTypeOf(GoProfileAreaArea)))            output = "Area";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileAreaCentroidX)))       output = "CentroidX";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileAreaCentroidZ)))       output = "CentroidZ";

    else if      (kObject_Is(measurement, kTypeOf(GoProfileBoxX)))                output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileBoxZ)))                output = "Z";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileBoxWidth)))            output = "Width";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileBoxHeight)))           output = "Height";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileBoxGlobalX)))          output = "GlobalX";

    else if      (kObject_Is(measurement, kTypeOf(GoProfileCircleX)))             output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileCircleZ)))             output = "Z";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileCircleRadius)))        output = "Radius";

    else if      (kObject_Is(measurement, kTypeOf(GoProfileDimWidth)))            output = "Width";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileDimHeight)))           output = "Height";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileDimDistance)))         output = "Distance";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileDimCenterX)))          output = "CenterX";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileDimCenterZ)))          output = "CenterZ";

    else if      (kObject_Is(measurement, kTypeOf(GoProfileGrooveX)))             output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileGrooveZ)))             output = "Z";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileGrooveWidth)))         output = "Width";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileGrooveDepth)))         output = "Depth";

    else if      (kObject_Is(measurement, kTypeOf(GoProfileIntersectX)))          output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileIntersectZ)))          output = "Z";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileIntersectAngle)))      output = "Angle";

    else if      (kObject_Is(measurement, kTypeOf(GoProfileLineStdDev)))          output = "StdDev";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileLineMaxError)))        output = "MaxError";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileLineMinError)))        output = "MinError";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileLinePercentile)))      output = "Percentile";

    else if      (kObject_Is(measurement, kTypeOf(GoProfilePanelGap)))            output = "Gap";
    else if      (kObject_Is(measurement, kTypeOf(GoProfilePanelFlush)))          output = "Flush";

    else if      (kObject_Is(measurement, kTypeOf(GoProfilePositionX)))           output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoProfilePositionZ)))           output = "Z";

    else if      (kObject_Is(measurement, kTypeOf(GoProfileStripX)))              output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileStripZ)))              output = "Z";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileStripWidth)))          output = "Width";
    else if      (kObject_Is(measurement, kTypeOf(GoProfileStripHeight)))         output = "Height";

    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceBoxX)))                output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceBoxY)))                output = "Y";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceBoxZ)))                output = "Z";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceBoxWidth)))            output = "Width";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceBoxLength)))           output = "Length";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceBoxHeight)))           output = "Height";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceBoxZAngle)))           output = "ZAngle";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceBoxGlobalX)))          output = "GlobalX";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceBoxGlobalY)))          output = "GlobalY";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceBoxGlobalZAngle)))     output = "GlobalZAngle";

    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleX)))               output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleY)))               output = "Y";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleZ)))               output = "Z";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleOuterRadius)))     output = "OuterRadius";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleDepth)))           output = "Depth";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleBevelAngle)))      output = "BevelAngle";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleBevelRadius)))     output = "BevelRadius";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleXAngle)))          output = "XAngle";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleYAngle)))          output = "YAngle";

    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceEllipseMajor)))        output = "Major";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceEllipseMinor)))        output = "Minor";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceEllipseRatio)))        output = "Ratio";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceEllipseZAngle)))       output = "ZAngle";

    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceHoleX)))               output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceHoleY)))               output = "Y";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceHoleZ)))               output = "Z";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceHoleRadius)))          output = "Radius";

    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningX)))            output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningY)))            output = "Y";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningZ)))            output = "Z";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningWidth)))        output = "Width";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningLength)))       output = "Length";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningAngle)))        output = "Angle";

    else if      (kObject_Is(measurement, kTypeOf(GoSurfacePlaneXAngle)))         output = "XAngle";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfacePlaneYAngle)))         output = "YAngle";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfacePlaneZOffset)))        output = "ZOffset";

    else if      (kObject_Is(measurement, kTypeOf(GoSurfacePositionX)))           output = "X";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfacePositionY)))           output = "Y";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfacePositionZ)))           output = "Z";

    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceStudBaseX)))           output = "BaseX";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceStudBaseY)))           output = "BaseY";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceStudBaseZ)))           output = "BaseZ";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceStudTipX)))            output = "TipX";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceStudTipY)))            output = "TipY";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceStudTipZ)))            output = "TipZ";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceStudRadius)))          output = "Radius";

    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceVolumeVolume)))        output = "Volume";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceVolumeArea)))          output = "Area";
    else if      (kObject_Is(measurement, kTypeOf(GoSurfaceVolumeThickness)))     output = "Thickness";

    else if      (kObject_Is(measurement, kTypeOf(GoScriptOutput)))               output = "Output";

    else if      (kObject_Is(measurement, kTypeOf(GoExtMeasurement)))             output = "Custom";
    
    else                                                         return kERROR_PARAMETER;

    strncpy(measurementName, output, capacity-1);
    measurementName[capacity-1] = 0; 

    return kOK; 
}
