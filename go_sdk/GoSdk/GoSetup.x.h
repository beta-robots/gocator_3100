/** 
 * @file    GoSetup.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SETUP_X_H
#define GO_SDK_SETUP_X_H

#include <GoSdk/GoSetup.h>
#include <kApi/Data/kXml.h>
kBeginHeader()

#define GO_MAX_RESOLUTION_COUNT         (3)
#define GO_MAX_EXPOSURE_COUNT           (5)

typedef struct GoTriggerClass
{
    kObjectClass base;

    GoTriggerSource source;
    GoTriggerUnits unit;

    GoElement64f frameRate;
    kBool maxFrameRateEnabled;
    GoFrameRateMaxSource frameRateMaxSource;

    GoElement64f encoderSpacing;
    GoEncoderSpacingMinSource encoderSpacingMinSource;
    GoEncoderTriggerMode encoderTriggerMode;

    GoElement64f delay;
    kBool gateEnabled;
    kBool gateEnabledSystemValue;
    kBool gateEnabledUsed;
} GoTriggerClass;

typedef struct GoFiltersClass
{
    kObjectClass base;

    GoElement64f xGapFilling;
    GoElement64f yGapFilling;
    GoElement64f xMedian;
    GoElement64f yMedian;
    GoElement64f xSmoothing;
    GoElement64f ySmoothing;
    GoElement64f xDecimation;
    GoElement64f yDecimation;
} GoFiltersClass;

typedef struct GoAlignmentClass
{
    kObjectClass base;

    GoAlignmentType type;
    kArrayList typeOptions;
    GoAlignmentTarget stationaryTarget;
    kArrayList stationaryTargetOptions;
    GoAlignmentTarget movingTarget;
    kArrayList movingTargetOptions;
    kBool inputTriggerEnabled;
    kBool encoderCalibrateEnabled;
    k64f diskDiameter;
    k64f diskHeight;
    k64f barWidth;
    k64f barHeight;
    kSize barHoleCount;
    k64f barHoleDistance;
    k64f barHoleDiameter;
    k64f plateHeight;
    kSize plateHoleCount;
    k64f plateRefHoleDiameter;
    k64f plateSecHoleDiameter;
} GoAlignmentClass;

typedef struct GoCameraConfig
{
    k32u x; 
    k32u y; 
    k32u width; 
    k32u height; 
} GoCameraConfig;

typedef struct GoTrackingConfig
{
    kBool used;
    kBool enabled;
    GoElement64f trackingAreaHeight;
    k64f searchThreshold;
} GoTrackingConfig;

typedef struct GoTransformConfig
{
    k64f xOffset;
    k64f yOffset;
    k64f zOffset;
    k64f xAngle;
    k64f yAngle;
    k64f zAngle;
} GoTransformConfig;

/* 
 * GoSetupNode
 */

typedef kObject GoSetupNode; 

typedef struct GoSetupNodeClass
{
    kObjectClass base;   

    kXml configXml;
    kXmlItem configXmlItem;
    kXml transformXml;
    kXmlItem transformXmlItem;
    
    GoRole role;
    kObject setup;          //setup object (parent)
    kBool hasConfig;
    kBool hasTransform;

    //Configuration elements
    GoActiveAreaConfig activeArea;
    GoTransformedDataRegion transformedDataRegion;

    GoCameraConfig frontCamera;
    GoCameraConfig backCamera;
    kBool backCameraUsed;

    GoExposureMode exposureMode;
    kArrayList exposureModeOptions;
    GoElement64f exposure;
    k64f dynamicExposureMax; 
    k64f dynamicExposureMin; 
    k64f exposureSteps[GO_MAX_EXPOSURE_COUNT]; 
    kSize exposureStepCount; 
    kSize intensityStepIndex;
    GoPatternSequenceType patternSequenceType;
    kArrayList patternSequenceTypeOptions;
    kBool patternSequenceTypeUsed;
    kSize patternSequenceCount;

    GoElement64f spacingInterval;
    GoElement32u spacingIntervalType;
    k32u xSpacingCount;
    k32u ySpacingCount;

    k32u zSubsamplingOptions[GO_MAX_RESOLUTION_COUNT]; 
    kSize zSubsamplingOptionCount; 
    k32u zSubsampling; 
    k32u xSubsamplingOptions[GO_MAX_RESOLUTION_COUNT]; 
    kSize xSubsamplingOptionCount; 
    k32u xSubsampling; 

    GoTrackingConfig tracking;
    GoMaterial material;
} GoSetupNodeClass; 

kDeclareClass(Go, GoSetupNode, kObject)

#define GoSetupNode_Cast_(CONTEXT)    kCastClass_(GoSetupNode, CONTEXT)

GoFx(kStatus) GoSetupNode_Construct(GoSetupNode* node, kObject setup, GoRole role, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoSetupNode_Init(GoSetupNode node, kType type, kObject setup, GoRole role, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSetupNode_VRelease(GoSetupNode node);

GoFx(GoRole) GoSetupNode_Role(GoSetupNode node);
GoFx(kStatus) GoSetupNode_ReadConfig(GoSetupNode node, kXml xml, kXmlItem item);
GoFx(kStatus) GoSetupNode_WriteConfig(GoSetupNode node, kXml xml, kXmlItem item); 
GoFx(kStatus) GoSetupNode_ClearConfig(GoSetupNode node); 
GoFx(kStatus) GoSetupNode_ReadTransform(GoSetupNode node, kXml xml, kXmlItem item); 
GoFx(kStatus) GoSetupNode_WriteTransform(GoSetupNode node, kXml xml, kXmlItem item); 
GoFx(kStatus) GoSetupNode_ClearTransform(GoSetupNode node); 


/* 
 * GoSetup
 */

typedef struct GoSetupClass
{
    kObjectClass base; 
    kObject sensor;             // sensor (parent object)
    kArrayList nodes;           // device-specific setup (kArrayList<GoSetupNode>

    kXml configXml;
    kXmlItem configXmlItem;
    kXml transformXml;
    kXmlItem transformXmlItem;

    //configuration elements
    kBool autoStartEnabled;

    kBool temperatureSafetyEnabled;
    kBool temperatureSafetyUsed;

    GoMode scanMode;
    kArrayList scanModeOptions;
    
    kBool intensityEnabled;
    kBool intensityAvail;
    kBool intensityEnabledSystemValue;

    kBool occlusionReductionEnabled;

    kBool uniformSpacingEnabled;
    kBool uniformSpacingEnabledSystemValue;
    kBool uniformSpacingAvail;

    GoTriggerClass trigger;
    GoFiltersClass filters;
    GoLayout layout;
    GoAlignmentClass alignment;

    GoProfileGeneration profileGeneration;      //profile generation module
    GoSurfaceGeneration surfaceGeneration;      //surface generation module
    GoPartDetection partDetection;              //part detection module
    GoPartMatching partMatching;              

    kBool externalInputZPulseEnabled;
} GoSetupClass; 

kDeclareClass(Go, GoSetup, kObject)

#define GoSetup_Cast_(CONTEXT)    kCastClass_(GoSetup, CONTEXT)

GoFx(kStatus) GoSetup_Construct(GoSetup* setup, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoSetup_Init(GoSetup setup, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSetup_VRelease(GoSetup setup);

GoFx(GoSetupNode) GoSetup_FindNode(GoSetup setup, GoRole role);

GoFx(kStatus) GoSetup_ReadConfig(GoSetup setup, kXml xml, kXmlItem item);
GoFx(kStatus) GoSetup_WriteConfig(GoSetup setup, kXml xml, kXmlItem item); 
GoFx(kStatus) GoSetup_ReadTransform(GoSetup setup, kXml xml, kXmlItem item); 
GoFx(kStatus) GoSetup_WriteTransform(GoSetup setup, kXml xml, kXmlItem item); 

GoFx(kStatus) GoSetup_EnableTemperatureSafety( GoSetup setup, kBool enable);
GoFx(kBool) GoSetup_TemperatureSafetyEnabled(GoSetup setup);
GoFx(kBool) GoSetup_TemperatureSafetyValueUsed(GoSetup setup);

kEndHeader()

#endif
