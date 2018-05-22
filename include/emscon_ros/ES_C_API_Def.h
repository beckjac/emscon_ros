
/******************************************************************************

Copyright (C) Leica Geosystems AG, 2001..2011

Filename: ES_C_API_Def.h 

Description: C- Application Programming Interface for Leica Embedded Systems

Notes: 
This file comprises the interface for all Leica Tracker- types (3D, 6DoF).     
If addressing AT4xx Trackers exclusively, it is recommended rather using the   
related include file (of same name) that is delivered with the AT4xx SDK).     

******************************************************************************/ 

#ifndef ES_C_API_DEF_H
#define ES_C_API_DEF_H

// These symbols allow checking against correct include file versions.
// Note: TPI/SDK Version not necessarily matches EmScon server version!
//
// EmScon TPI/SDK Version V3.6
//
#define ES_API_MAJOR_VERSION  3
#define ES_API_MINOR_VERSION  6

// An application can define this symbol if interested in  
// version definitions only (mainly for Leica internal use)
#ifndef ES_API_VERSION_INC_ONLY

#include "Enum.h" /* include 'Enum.h' PAST version symbol definition */

// For managed CPP applications, enum definitions require 
// to be prefixed with a 'public __value' directive. 
// To achieve this, define preprocessor symbol 'ES_MCPP_SUPPORT'
// prior to inclusion of file 'es_c_api_def.h'.
// 
#undef ES_API
#ifdef ES_MCPP_SUPPORT
   #define ES_API public __value
#else
   #define ES_API
#endif

/////////////////////////////////////////////////////////////////////////////

// new byte alignment == 4, save old value on stack
#pragma pack (push, 4)

// No boolean data type is available in C-language. By convention, 'int' is used
// in C for boolean variables. For convenience, the ES API defines an 'ES_BOOL'.
// In order to remain platform-independent, it must be of type 'int' (4 Bytes)'.
// Do not use 'BOOL' for not to conflict with Microsoft- specific type (2 Bytes!)!
//
typedef int ES_BOOL;

// New additional range for error numbers
#define ES_API_ERROR_OFFSET 10000

/////////////////////////////////////////////////////////////////////////////

// Enumeration Types:

/**
Identify the data used in the API calls over the TCP/IP communication
**/
ES_API enum ES_DataType
{
    ES_DT_Command = 0,
    ES_DT_Error = 1,
    ES_DT_SingleMeasResult = 2,
    ES_DT_MultiMeasResult = 3,                     
    ES_DT_StationaryProbeMeasResult = 4,           
    ES_DT_ContinuousProbeMeasResult = 5,           
    ES_DT_NivelResult = 6,
    ES_DT_ReflectorPosResult = 7,
    ES_DT_SystemStatusChange = 8,
    ES_DT_SingleMeasResult2 = 9,
    ES_DT_MultiMeasResult2 = 10,                   
    ES_DT_ProbePosResult = 11,                     
};

/**
The embedded system supports these commands
**/
ES_API enum ES_Command
{
    ES_C_ExitApplication = 0,                      // stop (exit) the embedded system
    ES_C_GetSystemStatus = 1,                      // frequently used information
    ES_C_GetTrackerStatus = 2,                     // seldom used information
    ES_C_SetTemperatureRange = 3,                  // set the temperature range for the tracker
    ES_C_GetTemperatureRange = 4,                  // get the temperature range for the tracker
    ES_C_SetUnits = 5,
    ES_C_GetUnits = 6,
    ES_C_Initialize = 7,
    ES_C_ReleaseMotors = 8,                        
    ES_C_ActivateCameraView = 9,
    ES_C_Park = 10,
    ES_C_SwitchLaser = 11,                         
    ES_C_SetStationOrientationParams = 12,
    ES_C_GetStationOrientationParams = 13,
    ES_C_SetTransformationParams = 14,
    ES_C_GetTransformationParams = 15,
    ES_C_SetBoxRegionParams = 16,                  
    ES_C_GetBoxRegionParams = 17,                  
    ES_C_SetSphereRegionParams = 18,               
    ES_C_GetSphereRegionParams = 19,               
    ES_C_SetEnvironmentParams = 20,
    ES_C_GetEnvironmentParams = 21,
    ES_C_SetRefractionParams = 22,
    ES_C_GetRefractionParams = 23,
    ES_C_SetMeasurementMode = 24,
    ES_C_GetMeasurementMode = 25,
    ES_C_SetCoordinateSystemType = 26,
    ES_C_GetCoordinateSystemType = 27,
    ES_C_SetStationaryModeParams = 28,
    ES_C_GetStationaryModeParams = 29,
    ES_C_SetContinuousTimeModeParams = 30,         
    ES_C_GetContinuousTimeModeParams = 31,         
    ES_C_SetContinuousDistanceModeParams = 32,     
    ES_C_GetContinuousDistanceModeParams = 33,     
    ES_C_SetSphereCenterModeParams = 34,           
    ES_C_GetSphereCenterModeParams = 35,           
    ES_C_SetCircleCenterModeParams = 36,           
    ES_C_GetCircleCenterModeParams = 37,           
    ES_C_SetGridModeParams = 38,                   
    ES_C_GetGridModeParams = 39,                   
    ES_C_SetReflector = 40,
    ES_C_GetReflector = 41,
    ES_C_GetReflectors = 42,
    ES_C_SetSearchParams = 43,
    ES_C_GetSearchParams = 44,
    ES_C_SetAdmParams = 45,                        
    ES_C_GetAdmParams = 46,                        
    ES_C_SetSystemSettings = 47,
    ES_C_GetSystemSettings = 48,
    ES_C_StartMeasurement = 49,
    ES_C_StartNivelMeasurement = 51,
    ES_C_StopMeasurement = 52,
    ES_C_ChangeFace = 53,
    ES_C_GoBirdBath = 54,
    ES_C_GoPosition = 55,
    ES_C_GoPositionHVD = 56,
    ES_C_PositionRelativeHV = 57,
    ES_C_PointLaser = 58,
    ES_C_PointLaserHVD = 59,
    ES_C_MoveHV = 60,
    ES_C_GoNivelPosition = 61,
    ES_C_GoLastMeasuredPoint = 62,
    ES_C_FindReflector = 63,                       // searches a reflector at the given position
    ES_C_Unknown = 64,                              
    
    // New commands added for release V1.1 / V1.2
    ES_C_LookForTarget = 65,                       // looks for a reflector at the given position and returns Hz V values
    ES_C_GetDirection = 66,                        // get direction even without reflector locked on
    ES_C_CallOrientToGravity = 67,                 // starts the orient to gravity process
    ES_C_ClearTransformationNominalPointList = 68, // clears the nominal point list
    ES_C_ClearTransformationActualPointList = 69,  // clears the actual point list
    ES_C_AddTransformationNominalPoint = 70,       // adds a point to the nominal point list
    ES_C_AddTransformationActualPoint = 71,        // adds a point to the nominal point list
    ES_C_SetTransformationInputParams = 72,        // set the input params for the transformation
    ES_C_GetTransformationInputParams = 73,        // get the input params for the transformation
    ES_C_CallTransformation = 74,                  // starts the transformation process
    ES_C_GetTransformedPoints = 75,                // each record in this list is sent via the network
    ES_C_ClearDrivePointList = 76,                 // clears the drive point list
    ES_C_AddDrivePoint = 77,                       // adds a point to the nominal point list
    ES_C_CallIntermediateCompensation = 78,        // starts the intermediate compensation process
    ES_C_SetCompensation = 79,                     // set's the last calculated compensation as the active one
    ES_C_SetStatisticMode = 80,                    // set's the amount of statistical information returned by the system
    ES_C_GetStatisticMode = 81,                    // get's the statistical setting
    ES_C_GetStillImage = 82,                       // get a still image
    ES_C_SetCameraParams = 83,                     // adjust video camera parameters
    ES_C_GetCameraParams = 84,                     // read the current video camera parameters            
    
    // New commands added for release V1.3 / V1.4 
    ES_C_GetCompensation = 85,                     // read the currently active compensation ID
    ES_C_GetCompensations = 86,                    // read all compensations stored in the database
    ES_C_CheckBirdBath = 87,                       // bird bath check routine
    ES_C_GetTrackerDiagnostics = 88,                              
    ES_C_GetADMInfo = 89,                           
    ES_C_GetTPInfo = 90,
    ES_C_GetNivelInfo = 91,
    ES_C_SetLaserOnTimer = 92,                     // switch the laser on in ... time
    ES_C_GetLaserOnTimer = 93,                     // read the remining time until it is switched on
    ES_C_ConvertDisplayCoordinates = 94,           // convert display coordinate triples from base to acutal and back
    ES_C_GoBirdBath2 = 95,                         // GoBirdBath with selection (clockwise / counter clockwise)
    ES_C_SetTriggerSource = 96,                    
    ES_C_GetTriggerSource = 97,                    
    ES_C_GetFace = 98,                             // returns the current face (Face1 / Face2)

    // New commands added for release V2.0 / V2.1
    ES_C_GetCameras = 99,                          
    ES_C_GetCamera = 100,                          
    ES_C_SetMeasurementCameraMode = 101,           
    ES_C_GetMeasurementCameraMode = 102,           
    ES_C_GetProbes = 103,                          
    ES_C_GetProbe = 104,                           
    ES_C_GetTipAdapters = 105,                     
    ES_C_GetTipAdapter = 106,                      
    ES_C_GetTCamToTrackerCompensations = 107,      
    ES_C_GetTCamToTrackerCompensation = 108,       
    ES_C_SetTCamToTrackerCompensation = 109,       
    ES_C_GetProbeCompensations = 110,              
    ES_C_GetProbeCompensation = 111,               
    ES_C_SetProbeCompensation = 112,               
    ES_C_GetTipToProbeCompensations = 113,         // kept for compatibility reasons only. Rather use ES_C_GetTipToProbeCompensations2
    ES_C_GetTipToProbeCompensation = 114,          
    ES_C_SetExternTriggerParams = 115,             
    ES_C_GetExternTriggerParams = 116,             
    ES_C_GetErrorEllipsoid = 117,                  
    ES_C_GetMeasurementCameraInfo = 118,           
    ES_C_GetMeasurementProbeInfo = 119,            
    ES_C_SetLongSystemParameter = 120,
    ES_C_GetLongSystemParameter = 121,
    ES_C_GetMeasurementStatusInfo = 122,
    ES_C_GetCompensations2 = 123,                  // enhanced read all compensations stored in the database
    ES_C_GetCurrentPrismPosition = 124,            // read the current prism position 3D and 6D
    
    // New commands added for release V2.3 / V2.4
    ES_C_SetDoubleSystemParameter = 125,
    ES_C_GetDoubleSystemParameter = 126,
    ES_C_GetObjectTemperature = 127,               // read the object temperature
    ES_C_GetTriggerBoardInfo = 128,                
    ES_C_GetOverviewCameraInfo = 129,                   
    ES_C_ClearCommandQueue = 130,                  // clear the command queue
    ES_C_GetADMInfo2 = 131, 
    ES_C_GetTrackerInfo = 132,
    ES_C_GetNivelInfo2 = 133,
    ES_C_RestoreStartupConditions = 134,
    ES_C_GoAndMeasure = 135,                       // AutoInspect type of operation
    ES_C_GetTipToProbeCompensations2 = 136,        // extended information (versions V2.4 and up only)

    // New commands added for release V3.5 
    ES_C_SetTipAdapter = 137,                      // Set a "virtual" tip 
    ES_C_GetATRInfo = 138,
    ES_C_GetMeteoStationInfo = 139,    
    ES_C_GetAT4xxInfo = 140,
    
    // New commands added for release V3.6
    ES_C_GetSystemSoftwareVersion = 142,           
};

/**
The currently implemented measurement modes
**/
ES_API enum ES_MeasMode
{
    ES_MM_Stationary = 0,
    ES_MM_ContinuousTime = 1,                      
    ES_MM_ContinuousDistance = 2,                  
    ES_MM_Grid = 3,                                
    ES_MM_SphereCenter = 4,                        
    ES_MM_CircleCenter = 5,                        
    ES_MM_6DStationary = 6,                        
    ES_MM_6DContinuousTime = 7,                    
    ES_MM_6DContinuousDistance = 8,                
    ES_MM_6DGrid = 9,                              
    ES_MM_6DSphereCenter = 10,                     
    ES_MM_6DCircleCenter = 11,                     
};

/**
The known taraget types (prism types)
**/
ES_API enum ES_TargetType
{
    ES_TT_Unknown = 0,
    ES_TT_CornerCube = 1,
    ES_TT_CatsEye = 2,
    ES_TT_GlassPrism = 3,
    ES_TT_RFIPrism = 4,
	ES_TT_RRR15 = 5,
	ES_TT_RRR05 = 6,
	ES_TT_BRR15 = 7,
	ES_TT_BRR05 = 8,
	ES_TT_TBR05 = 9,
};

/**
Set the temperature range for the laser tracker
ES_TR_Low is for ambient temperatures between 5 and 20 C
ES_TR_Medium is for ambient temperatures between 10 and 30 C
ES_TR_High is for ambient temperatures between 20 and 40 C
ES_TR_Automatic is for AT sensors only
**/
ES_API enum ES_TrackerTemperatureRange
{
    ES_TR_Low = 0,
    ES_TR_Medium = 1,
    ES_TR_High = 2,
    ES_TR_Automatic = 3,
};

/**
Ready status of laser tracker. This information is typically used to display the
status to the user (red green yellow) 
                                                                         
Version 2.0: (ES_TS_6DInvalid) pink indicates the 6D status is not valid 
**/
ES_API enum ES_TrackerStatus
{
    ES_TS_NotReady = 0,
    ES_TS_Busy = 1,
    ES_TS_Ready = 2,
    ES_TS_6DStatusInvalid = 3, 
};

/**
Result status values. Generated as answer to function calls via API
**/
ES_API enum ES_ResultStatus
{
    ES_RS_AllOK = 0,
    ES_RS_ServerBusy = 1,                          // a pending command
    ES_RS_NotImplemented = 2,
    ES_RS_WrongParameter = 3,
    ES_RS_WrongParameter1 = 4,
    ES_RS_WrongParameter2 = 5,
    ES_RS_WrongParameter3 = 6,
    ES_RS_WrongParameter4 = 7,
    ES_RS_WrongParameter5 = 8,
    ES_RS_WrongParameter6 = 9,
    ES_RS_WrongParameter7 = 10,
    ES_RS_Parameter1OutOfRangeOK = 11,
    ES_RS_Parameter1OutOfRangeNOK = 12,
    ES_RS_Parameter2OutOfRangeOK = 13,
    ES_RS_Parameter2OutOfRangeNOK = 14,
    ES_RS_Parameter3OutOfRangeOK = 15,
    ES_RS_Parameter3OutOfRangeNOK = 16,
    ES_RS_Parameter4OutOfRangeOK = 17,
    ES_RS_Parameter4OutOfRangeNOK = 18,
    ES_RS_Parameter5OutOfRangeOK = 19,
    ES_RS_Parameter5OutOfRangeNOK = 20,
    ES_RS_Parameter6OutOfRangeOK = 21,
    ES_RS_Parameter6OutOfRangeNOK = 22,
    ES_RS_WrongCurrentReflector = 23,
    ES_RS_NoCircleCenterFound = 24,                
    ES_RS_NoSphereCenterFound = 25,                
    ES_RS_NoTPFound = 26,
    ES_RS_NoWeathermonitorFound = 27,
    ES_RS_NoLastMeasuredPoint = 28,
    ES_RS_NoVideoCamera = 29,
    ES_RS_NoAdm = 30,
    ES_RS_NoNivel = 31,
    ES_RS_WrongTPFirmware = 32,
    ES_RS_DataBaseNotFound = 33,                   
    ES_RS_LicenseExpired = 34,                     
    ES_RS_UsageConflict = 35,
    ES_RS_Unknown = 36,

    // New status values added for V1.1 / V1.2
    ES_RS_NoDistanceSet = 37,
    ES_RS_NoTrackerConnected = 38,
    ES_RS_TrackerNotInitialized = 39,              // sensor is not initialized
    ES_RS_ModuleNotStarted = 40,                   // an internal client (module) could not be started
    ES_RS_ModuleTimedOut = 41,                     // an internal client (module) did not finish in time
    ES_RS_ErrorReadingModuleDb = 42,               // the module data base could not be read
    ES_RS_ErrorWritingModuleDb = 43,               // the data base could not be written
    ES_RS_NotInCameraPosition = 44,                // Camera can not be used in this position
    ES_RS_TPHasServiceFirmware = 45,               // Service Firmware loaded
    ES_RS_TPExternalControl = 46,                  // TP runs under external (AXYZ?) control
    ES_RS_WrongParameter8 = 47,
    ES_RS_WrongParameter9 = 48,
    ES_RS_WrongParameter10 = 49,
    ES_RS_WrongParameter11 = 50,
    ES_RS_WrongParameter12 = 51,
    ES_RS_WrongParameter13 = 52,
    ES_RS_WrongParameter14 = 53,
    ES_RS_WrongParameter15 = 54,
    ES_RS_WrongParameter16 = 55,

    // New status values added for V1.3 / V1.4
    ES_RS_NoSuchCompensation = 56,                 // the selected compensation does not exist
    ES_RS_MeteoDataOutOfRange = 57,
    ES_RS_InCompensationMode = 58,                 
    ES_RS_InternalProcessActive = 59,              
    ES_RS_NoCopyProtectionDongleFound = 60,        
    ES_RS_ModuleNotActivated = 61,                 
    ES_RS_ModuleWrongVersion = 62,                 
    ES_RS_DemoDongleExpired = 63,                  

    // New status values added for V2.0 / V2.1
    ES_RS_ParameterImportFromProbeFailed = 64,     
    ES_RS_ParameterExportToProbeFailed = 65,       
    ES_RS_TrkCompMeasCameraMismatch = 66,          
    ES_RS_NoMeasurementCamera = 67,                
    ES_RS_NoActiveMeasurementCamera = 68,          
    ES_RS_NoMeasurementCamerasInDb = 69,           
    ES_RS_NoCameraToTrackerCompSet = 70,           
    ES_RS_NoCameraToTrackerCompInDb = 71,          
    ES_RS_ProblemStoringCameraToTrackerFactorySet = 72, 
    ES_RS_ProblemWithCameraInternalCalibration = 73, 
    ES_RS_CommunicationWithMeasurementCameraFailed = 74, 
    ES_RS_NoMeasurementProbe = 75,                 
    ES_RS_NoActiveMeasurementProbe = 76,           
    ES_RS_NoMeasurementProbesInDb = 77,            
    ES_RS_NoMeasurementProbeCompSet = 78,          
    ES_RS_NoMeasurementProbeCompInDb = 79,         
    ES_RS_ProblemStoringProbeFactorySet = 80,      
    ES_RS_WrongActiveMeasurementProbeCompInDb = 81, 
    ES_RS_CommunicationWithMeasurementProbeFailed = 82, 
    ES_RS_NoMeasurementTip = 83,                   
    ES_RS_NoActiveMeasurementTip = 84,             
    ES_RS_NoMeasurementTipsInDb = 85,              
    ES_RS_NoMeasurementTipCompInDb = 86,           
    ES_RS_NoMeasurementTipCompSet = 87,            
    ES_RS_ProblemStoringTipAssembly = 88,          
    ES_RS_ProblemReadingCompensationDb = 89,       
    ES_RS_NoDataToImport = 90,
    ES_RS_ProblemSettingTriggerSource = 91,        
    ES_RS_6DModeNotAllowed = 92,                   
    ES_RS_Bad6DResult = 93,                        
    ES_RS_NoTemperatureFromWM = 94,
    ES_RS_NoPressureFromWM = 95,
    ES_RS_NoHumidityFromWM = 96,
    ES_RS_6DMeasurementFace2NotAllowed = 97,       

    // New status values added for V2.3 / V2.4
    ES_RS_InvalidInputData = 98,
    ES_RS_NoTriggerBoard = 99,                     
    ES_RS_NoMeasurementShankCompSet = ES_API_ERROR_OFFSET + 1, 

    // New status values added for V3.0
    ES_RS_NoValidADMCompensation = ES_API_ERROR_OFFSET + 2,
    ES_RS_PressureSensorProblem = ES_API_ERROR_OFFSET + 3,
    ES_RS_MeasurementStatusNotReady = ES_API_ERROR_OFFSET + 4,
    ES_RS_AIFMStartUpBusy = ES_API_ERROR_OFFSET + 5,   // Intentionally repeated on next line with different name (alias)! 
    ES_RS_ADMStartUpBusy = ES_API_ERROR_OFFSET + 5,    // for Sensors w/o Interferometer - alias to AIFMStartUpBusy (from V3.6 and up)

    // New status values added for V3.5
    ES_RS_InvalidTipAdapter = ES_API_ERROR_OFFSET + 6, 
    ES_RS_NoAtr = ES_API_ERROR_OFFSET + 7,
    ES_RS_NoOVC = ES_API_ERROR_OFFSET + 8,
    ES_RS_NoStationaryResult = ES_API_ERROR_OFFSET + 9,
    ES_RS_SensorNotLeveled = ES_API_ERROR_OFFSET + 10,
    ES_RS_MultiConnectionsNotAllowed = ES_API_ERROR_OFFSET + 11,

    // New status values added for V3.6
    ES_RS_SensorNotWarmedUp = ES_API_ERROR_OFFSET + 12, 
    ES_RS_SensorNotStable = ES_API_ERROR_OFFSET + 13,
    ES_RS_SystemNotReadyForMeasurement = ES_API_ERROR_OFFSET + 14,
    ES_RS_CommunicationWithSensorFailed = ES_API_ERROR_OFFSET + 15,
    ES_RS_No_Sensor_Battery = ES_API_ERROR_OFFSET + 16,
    ES_RS_CompensatorNotAllowed = ES_API_ERROR_OFFSET + 17,
    ES_RS_WarmedUpStateReached = ES_API_ERROR_OFFSET + 18,
    ES_RS_NotLeveledForInitialization = ES_API_ERROR_OFFSET + 19,
    ES_RS_ADMHardwareProblem = ES_API_ERROR_OFFSET + 20,
    ES_RS_ATRHardwareProblem = ES_API_ERROR_OFFSET + 21,
};

/**
Additional info for each measurement in the continuous measurement modes
measurements wit a status other than ES_MS_AllOK should be treated with care
**/
ES_API enum ES_MeasurementStatus
{
    ES_MS_AllOK = 0,
    ES_MS_SpeedWarning = 1,
    ES_MS_SpeedExeeded = 2,
    ES_MS_PrismError = 3,
    ES_MS_TriggerTimeViolation = 4,
};

/**
Additional info for each 6DOF measurement in the continuous measurement mode
gives information about the trigger button on the probe
**/
ES_API enum ES_TriggerStatus
{
    ES_TS_TriggerNotPressed = 0,
    ES_TS_TriggerPressed = 1,
};

/**
Additional info for each 6DOF measurement in the continuous measurement mode
gives information about the measurement tip
**/
ES_API enum ES_MeasurementTipStatus
{
    ES_PTS_TipOK = 0,
    ES_PTS_UnknownTip = 1,
    ES_PTS_MultipleTipsAttached = 2,
};

/**
Information about the laser tracker during startup. 

  The sequence of this enum is important. It shows the state of the laser tracker processor during 
  the startup of the embedded system.
  
    The tracker is only ready if it was initialized
    it is only initialized if it has a valid compensation
    it can only have a valid compensation if it was booted
    it can only be booted if there was a connection between 
    the embedded system and the laser tracker and this 
    connection is only possible if there is a laser tracker
                                                   
    The status shows the sequence in the hardware detection. 
    - a camera was found                           
    - a probe was found                            
    - a tip was found (if applicable)              
**/
ES_API enum ES_TrackerProcessorStatus
{
    ES_TPS_NoTPFound = 0,
    ES_TPS_TPFound = 1,
    ES_TPS_NBOpen = 2,                             
    ES_TPS_Booted = 3,
    ES_TPS_CompensationSet = 4,
    ES_TPS_Initialized = 5,
};

/**
Additional information about the laser processor in the laser tracker
**/
ES_API enum ES_LaserProcessorStatus
{
    ES_LPS_LCPCommFailed = 0,                      
    ES_LPS_LCPNotAvail = 1,                        
    ES_LPS_LaserHeatingUp = 2,                     
    ES_LPS_LaserReady = 3,
    ES_LPS_UnableToStabilize = 4,                  
    ES_LPS_LaserOff = 5,                           
};

/**
Additional information about the absolute distance meter in the laser tracker
**/
ES_API enum ES_ADMStatus
{
    ES_AS_NoADM = 0,                               
    ES_AS_ADMCommFailed = 1,                       
    ES_AS_ADMReady = 2,
    ES_AS_ADMBusy = 3,                             
    ES_AS_HWError = 4,                             
    ES_AS_SecurityLockActive = 5,                  
    ES_AS_NotCompensated = 6,                      
};

/**
Additional information about the nivel sensor connected to the laser tracker
**/
ES_API enum ES_NivelStatus
{
    ES_NS_AllOK = 0,
    ES_NS_OutOfRangeOK = 1,
    ES_NS_OutOfRangeNOK = 2,
    ES_NS_NoNivel = 3,
};

/**
Additional information about the weather monitor connected to the embedded system
**/
ES_API enum ES_WeatherMonitorStatus
{
    ES_WMS_NotConnected = 0,
    ES_WMS_ReadOnly = 1,
    ES_WMS_ReadAndCalculateRefractions = 2,
};

/**
System changes originated by the embedded system
**/
ES_API enum ES_SystemStatusChange
{
    ES_SSC_DistanceSet = 0,                        
    ES_SSC_LaserWarmedUp = 1,                      
                                                   
    // New events added for releases V1.3 .. V2.3
    ES_SSC_EnvironmentParamsChanged = 2,
    ES_SSC_RefractionParamsChanged = 3,
    ES_SSC_SearchParamsChanged = 4,
    ES_SSC_AdmParamsChanged = 5,
    ES_SSC_UnitsChanged = 6,
    ES_SSC_ReflectorChanged = 7,
    ES_SSC_SystemSettingsChanged = 8,
    ES_SSC_TemperatureRangeChanged = 9,
    ES_SSC_CameraParamsChanged = 10,
    ES_SSC_CompensationChanged = 11,
    ES_SSC_CoordinateSystemTypeChanged = 12,
    ES_SSC_BoxRegionParamsChanged = 13,            
    ES_SSC_SphereRegionParamsChanged = 14,         
    ES_SSC_StationOrientationParamsChanged = 15,
    ES_SSC_TransformationParamsChanged = 16,
    ES_SSC_MeasurementModeChanged = 17,
    ES_SSC_StationaryModeParamsChanged = 18,
    ES_SSC_ContinuousTimeModeParamsChanged = 19,   
    ES_SSC_ContinuousDistanceModeParamsChanged = 20, 
    ES_SSC_GridModeParamsChanged = 21,             
    ES_SSC_CircleCenterModeParamsChanged = 22,     
    ES_SSC_SphereCenterModeParamsChanged = 23,     
    ES_SSC_StatisticModeChanged = 24,
    ES_SSC_MeasStatus_NotReady = 25,
    ES_SSC_MeasStatus_Busy = 26,
    ES_SSC_MeasStatus_Ready = 27,
    ES_SSC_MeasurementCountReached = 28,           
    ES_SSC_TriggerSourceChanged = 29,              
    ES_SSC_IsFace1 = 30,
    ES_SSC_IsFace2 = 31,
    ES_SSC_ExternalControlActive = 32,             
    ES_SSC_ServiceSoftwareActive = 33,             
    ES_SSC_MeasurementCameraChanged = 34,          
    ES_SSC_MeasurementCameraModeChanged = 35,      
    ES_SSC_ProbeChanged = 36,                      
    ES_SSC_TipChanged = 37,                        
    ES_SSC_TCamToTrackerCompensationChanged = 38,  
    ES_SSC_ProbeCompensationChanged = 39,          
    ES_SSC_TipToProbeCompensationChanged = 40,     
    ES_SSC_ExternTriggerParamsChanged = 41,        
    ES_SSC_TCamToTrackerCompensationDeleted = 42,  
    ES_SSC_MeasurementProbeCompensationDeleted = 43, 
    ES_SSC_MeasurementTipCompensationDeleted = 44, 
    ES_SSC_ManyMechanicalCompensationsInDB = 45,   
    ES_SSC_MeasStatus_6DStatusInvalid = 99,        
    ES_SSC_MeasurementProbeButtonDown = 100,       
    ES_SSC_MeasurementProbeButtonUp = 101,         
    ES_SSC_ExternalTriggerEvent = 102,             
    ES_SSC_ExternalTriggerStartEvent = 103,        
    ES_SSC_ExternalTriggerStopEvent = 104,         
    ES_SSC_ObjectTemperatureChanged = 105,               
    ES_SSC_OverviewCameraChanged = 106,            
    ES_SSC_NivelSensorChanged = 107,               
    ES_SSC_ProbeButton1Down = 110,
    ES_SSC_ProbeButton1Up = 111,
    ES_SSC_ProbeButton1DoubleClick = 112,          // for future use
    ES_SSC_ProbeButton2Down = 120,
    ES_SSC_ProbeButton2Up = 121,
    ES_SSC_ProbeButton2DoubleClick = 122,          // for future use
    ES_SSC_ProbeButton3Down = 130,
    ES_SSC_ProbeButton3Up = 131,
    ES_SSC_ProbeButton3DoubleClick = 132,          // for future use
    ES_SSC_ProbeButton4Down = 140,
    ES_SSC_ProbeButton4Up = 141,
    ES_SSC_ProbeButton4DoubleClick = 142,          // for future use
                                                   
    // New events added for release V3.0           
    ES_SSC_QuickReleaseOpend = 143,                
    ES_SSC_QuickReleaseClosed = 144,               
    ES_SSC_LaserReachingLimit = 145,               
    ES_SSC_LaserNotStabilized = 146,               
                                                   
    // New event added for release V3.5            
    ES_SSC_MultipleTipAdapterConnected = 150,      
    
    // New events added for release V3.6
    ES_SCC_InitializationStatusChanged = 151,      // Note the typing error: this should read ES_SSC instead of ES_SCC...
    ES_SCC_TiltSensorStatusChanged = 152,          // ...nevertheless, for compatibility reasons, leave this unchanged. 
    
    // New events added for release V2.3           // Note that items of this enum are ordered by theîr values, not by their release- time)
    ES_SSC_CompensationModeStart = 800,            
    ES_SSC_CompensationModeEnd = 801,              
    ES_SSC_EmsysFilesImported = 820,     

    // New events added for release V3.5           // Note that items of this enum are ordered by theîr values, not by their release- time)
    ES_SSC_SensorDetected = 850,     
    ES_SSC_SensorDisconnected = 851,     

    // New events added for release V3.6
    ES_SSC_CompensatorStatusChanged = 852,         // compensator was switched ON / OFF
    ES_SSC_BatteryStatusChanged = 853,             // battery capacity has changed
    
    ES_SSC_CopyProtectionRemoved = 996,            
    ES_SSC_TPConnectionClosing = 997,
    ES_SSC_ServerClosing = 998,
    ES_SSC_ServerStarted = 999,
};

/**
Positions used to drive to during a orient to gravity procedure
**/
ES_API enum ES_NivelPosition
{
    ES_NP_Pos1 = 0,
    ES_NP_Pos2 = 1,
    ES_NP_Pos3 = 2,
    ES_NP_Pos4 = 3,
};

/**
Region types supported 
**/
ES_API enum ES_RegionType
{
    ES_RT_Sphere = 0,
    ES_RT_Box = 1,
};

/**
The possible statistics mode selections
applicable to stationary and continuous mode measurements
**/
ES_API enum ES_StatisticMode
{
    ES_SM_Standard = 0,
    ES_SM_Extended = 1,
};

/**
The possible formats for the still image function
**/
ES_API enum ES_StillImageFileType
{
    ES_SI_Bitmap = 0,
    ES_SI_Jpeg = 1,
};

/**
The implemented result types for the transformation functionality
**/
ES_API enum ES_TransResultType
{
    ES_TR_AsTransformation = 0,
    ES_TR_AsOrientation = 1,
};

/**
The different tracker processor controller types
**/
ES_API enum ES_TrackerProcessorType
{
    ES_TT_Undefined = 0,
    ES_TT_SMART310 = 1,                            
    ES_TT_LT_Controller = 2,                       
    ES_TT_EmbeddedController = 3,                  
    ES_TT_EmbeddedController600 = 4,               
    ES_TT_ATC900 = 6,                              
    ES_TT_ATC400 = 20,
};

/**
The possible tracker controller micro processor types
**/
ES_API enum ES_TPMicroProcessorType
{
    ES_TPM_Undefined = 0,
    ES_TPM_i486 = 1,                               
    ES_TPM_686 = 2,                                
    ES_TPM_PXA250 = 20,
};

/**
The implemented sensor types
**/
ES_API enum ES_LTSensorType
{
    ES_LTS_Undefined = 0,
    ES_LTS_SMARTOptodyne = 1,                      
    ES_LTS_SMARTLeica = 2,                         
    ES_LTS_LT_D_500 = 3,                           // LT500 or LTD500
    ES_LTS_LT300 = 4,                              
    ES_LTS_LT301 = 5,                              
    ES_LTS_LT_D_800 = 6,                           // LT800 or LTD800
    ES_LTS_LT_D_700 = 7,                           // LT700 or LTD700
    ES_LTS_LT_D_600 = 8,                           // LT600 or LTD600
    ES_LTS_LT_D_640 = 9,                           
    ES_LTS_LT_D_706 = 10,                          
    ES_LTS_LT_D_709 = 11,                          
    ES_LTS_LT_D_840 = 12,                          
    ES_LTS_AT901_B  = 13,                          
    ES_LTS_AT901_MR = 14,                          
    ES_LTS_AT901_LR = 15,                          
    ES_LTS_AT401 = 70,
    ES_LTS_NoSensor = 99,
};

/**
The implemented trigger sources
**/
ES_API enum ES_TriggerSource
{
    ES_TS_Undefined = 0,
    ES_TS_Internal_Application = 1,                // default
    ES_TS_External = 2,
    ES_TS_External_EventMessage = 3,
};

/**
The allowed display coordinate conversion types
**/
ES_API enum ES_DisplayCoordinateConversionType
{
    ES_DCC_BaseToCurrent = 0,
    ES_DCC_CurrentToBase = 1,
};

/**
The GetFace command returns one of the following values
**/
ES_API enum ES_TrackerFace
{
    ES_TF_Unknown = 0,
    ES_TF_Face1 = 1,
    ES_TF_Face2 = 2,
};

/**
The Set/GetCameraMode command takes/returns one of the following values
**/
ES_API enum ES_MeasurementCameraMode
{  
   ES_MCM_Measure = 0,  
   ES_MCM_Overview = 1,
};

/**
Used for GetCameras command parameter
**/
ES_API enum ES_MeasurementCameraType
{  
   ES_MC_None = 0,
   ES_MC_TCam700 = 1,
   ES_MC_TCam800 = 2,
   ES_MC_TCam706 = 3,
   ES_MC_TCam709 = 4,
   ES_MC_TCam_LR = 5,
   ES_MC_TCam_MR = 6,
};

/**
Used for GetProbes/ GetMeasurementProbeInfoRT command parameter
**/
ES_API enum ES_ProbeType
{  
   ES_PT_None = 0,
   ES_PT_Reflector = 1,
   ES_PT_TProbe = 2,
   ES_PT_TScan = 3,
   ES_PT_MachineControlProbe = 4,
   ES_PT_MachineControlProbeMultiSide = 5,
   ES_PT_TCamToTrackerTool = 100,
   ES_PT_ZoomArtifactTool = 200,
};

/**
Used for GetMeasurementProbeInfoRT command parameter
**/
ES_API enum ES_ProbeConnectionType
{  
   ES_PCT_None = 0,
   ES_PCT_CableController = 1,
   ES_PCT_CableSensor = 2,
   ES_PCT_IRLaser = 3,
   ES_PCT_IRWideAngle = 4,
};

ES_API enum ES_ProbeButtonType
{
   ES_PBT_None = 0,
   ES_PBT_Measurement = 1,
   ES_PBT_4Button = 2,
};

/**
Used for GetTipAdapters command parameter
**/
ES_API enum ES_SocketType
{
    ES_ST_Accurate = 0,                            // Has ID
    ES_ST_ThreadWithID = 100,                      // Has ID
    ES_ST_Thread = 200,                            // Does not have ID, detects presence
    ES_ST_Virtual = 999,                         
};

ES_API enum ES_TipType
{  
   ES_TT_None = 0,
   ES_TT_Fixed = 1,                                // Tip connected to ES_ST_Accurate socket type
   ES_TT_Scanner = 2,                              // Tip connected to ES_ST_Accurate socket type  
   ES_TT_TouchTrigger = 3,                         // Tip connected to ES_ST_Accurate socket type
   ES_TT_OpticalTrigger = 4,                       // Tip connected to ES_ST_Accurate socket type
   ES_TT_ThreadWithID_Fixed = ES_ST_ThreadWithID + ES_TT_Fixed,
   ES_TT_ThreadWithID_Scanner = ES_ST_ThreadWithID + ES_TT_Scanner,
   ES_TT_ThreadWithID_TouchTrigger = ES_ST_ThreadWithID + ES_TT_TouchTrigger,
   ES_TT_ThreadWithID_OpticalTrigger = ES_ST_ThreadWithID + ES_TT_OpticalTrigger,
   ES_TT_Thread = ES_ST_Thread,
   ES_TT_Virtual = ES_ST_Virtual,
};

/**
The Set/GetTriggerParams command take/returns of the following values
**/
ES_API enum ES_ClockTransition
{
   ES_CT_Positive = 1,
   ES_CT_Negative = 0,
};

ES_API enum ES_TriggerMode
{
   ES_TM_EventTrigger = 0,
   ES_TM_ContinuousExternalClockWithStartStop = 1,
   ES_TM_InternalClockWithExternalStartStop = 2,
};

ES_API enum ES_TriggerStartSignal
{
   ES_TSS_High = 1,
   ES_TSS_Low = 0,
};

/**
The currently implemented system parameter settings
**/
ES_API enum ES_ProbeConfigButton
{
    ES_PCB_SingleClick = 0,
    ES_PCB_StartStop = 1,    
    ES_PCB_4ButtonMode = 2,                        // each button own event
};

ES_API enum ES_ProbeButtonEvent
{
    ES_PBE_DisableEvents = 0,                      // no button events are send
    ES_PBE_EnableEvents = 1,                       // server sends button events
};

ES_API enum ES_ProbeConfigTip
{
    ES_PCT_OnlyWithTip = 0,
    ES_PCT_NoTipAllowed = 1,
    ES_PCT_OnlyWithShankCompensation = 2,          // tip MUST have valid shank compensation
};

ES_API enum ES_QuickReleaseStatus
{
    ES_QRS_NotExisting = -1,                       // older sensor type, does not have quick release
    ES_QRS_Closed = 0,
    ES_QRS_Open = 1,
};


ES_API enum ES_PowerLockMode
{
    ES_PLM_InDoor = 0,
    ES_PLM_OutDoor = 1,
    ES_PLM_OutDoor_LongRange = 2,
};

ES_API enum ES_SystemParameter
{
    ES_SP_KeepLastPositionFlag = 0,                // 0 = OFF; 1 = ON
    ES_SP_WeatherMonitorSetting = 1,               // see ES_WeatherMonitorStatus
    ES_SP_ShowAll6DMeasurements = 2,               // 0 = Only show data if 6D status is OK (default)
    ES_SP_LaserPointerCaptureBeam = 3,             // 0 = Beam catch OFF; 1 = Beam catch ON (default
    ES_SP_DisplayReflectorPosition = 10,           // 0 = OFF; 1 = ON
    ES_SP_ProbeConfig_Button = 50,                 // see enum ES_ProbeConfigButton
    ES_SP_ProbeConfig_ButtonEvent = 51,            // see enum ES_ProbeButtonEvent
    ES_SP_ProbeConfig_Tip = 52,                    // see enum ES_ProbeConfigTip
    ES_SP_ProbeConfig_SoundVolume = 53,            // 0 = OFF; 1..7 soft .. loud
    ES_SP_ProbeConfig_PowerOffTime = 54,           // 2..255 minutes until power off
    ES_SP_QuickReleaseStatus = 60,                 // Get only : see enum ES_QuickReleaseStatus
    ES_SP_TcpCommandQueueSize = 200,               // 0 = OFF 1..10 queue size
    ES_SP_SystemMax6DDataRate = 300,               // Get only! Maximal 6D data rate (interpolated)
    ES_SP_TcpDataPacketRate = 400,                 // Between 3 and 10 packets per second (LTD706 and later!)
    ES_SP_PowerLockFunctionAvailable = 410,        // 0 = NO ; 1 = YES, Power Lock functionality is available
    ES_SP_PowerLockFunctionActive = 411,           // 0 = NO ; 1 = YES Power Lock functionality is active
    ES_SP_PowerLockMode = 450,                     // see enum ES_PowerLockMode
    ES_SP_D_TemperatureThreshold = 1000,           // causes an event to be sent 
    ES_SP_D_PressureThreshold = 1001,              // causes an event to be sent 
    ES_SP_D_HumidityThreshold = 1002,              // causes an event to be sent 
    ES_SP_D_SystemLongest3DDistanceIFM = 1100,     // Get only! Longest distance system can measure
    ES_SP_D_SystemLongest3DDistanceADM = 1101,     // Get only! Longest distance system can measure
    ES_SP_D_SystemLongest6DDistance = 1102,        // Get only! Longest distance system can measure
    ES_SP_AT4xxControllerBatteryStatus = 5000,     // Get only; 0 .. 100%; 110% ==> external power; 120% ==> PoE (power over Ethernet)
    ES_SP_AT4xxSensorBatteryStatus = 5001,         // Get only; 0 .. 100%; 110% ==> external power
    ES_SP_AT4xxInclinationSensorState = 5002,      // valid states see ES_InclinationSensorState
};

ES_API enum ES_MeasurementStatusInfo
{
    ES_MSI_Unknown = 0,
    ES_MSI_TrackerFound = 1,
    ES_MSI_TrackerCompensationFound = 2,
    ES_MSI_ADMFound = 4,
    ES_MSI_ADMCompensationFound = 8,
    ES_MSI_MeasurementCameraFound = 16,             
    ES_MSI_InternalCameraParamsOK = 32,             
    ES_MSI_CameraToTrackerParamsFound = 64,         
    ES_MSI_MeasurementProbeFound = 128,             
    ES_MSI_ProbeParamsFound = 256,                  
    ES_MSI_MeasurementTipFound = 512,               
    ES_MSI_TipParamsFound = 1024,                   
    ES_MSI_ReflectorFound = 2048,
    ES_MSI_InFace1 = 4096,
    ES_MSI_ShankParamsFound = 8192,                 
    ES_MSI_SensorBatteryMounted = 16384,
    ES_MSI_NivelInWorkingRange = 32768,
    ES_MSI_Initialized = 65536,
};

ES_API enum ES_ClearCommandQueueType
{
    ES_CCQ_ClearOwnOnly = 0,                       // clear only own commands from queue. all others are left
    ES_CCQ_ClearAll = 1,                           // clear all commands on queue
};

ES_API enum ES_OverviewCameraType
{
    ES_OCT_Unknown = 0,
    ES_OCT_Classic = 1,
    ES_OCT_TCam_Integrated = 2,                    // overview camera in TCam stand
    ES_OCT_AT4xx_Integrated = 20,                  
};

ES_API enum ES_TriggerCardType
{
    ES_TCT_None = 0,
    ES_TCT_SingleTracker = 1,
};

ES_API enum ES_ADMType
{
    ES_AMT_Unknown = 0,
    ES_AMT_LeicaADM = 1,                           // ADM
    ES_AMT_LeicaAIFM = 2,                          // AIFM
    ES_AMT_LeicaADM2 = 3,                          // ADM of AT401 tracker series
};

ES_API enum ES_ATRType
{
    ES_ATR_None = 0,
    ES_ATR_4 = 1,
    ES_ATR_5i = 2,                                 // ATR of AT401 tracker series
};

ES_API enum ES_TrkAccuracyModel
{
    ES_TAM_Unknown = 0,
    ES_TAM_2005 = 1,
};

ES_API enum ES_NivelType
{
    ES_NT_Unknown = 0,
    ES_NT_Nivel20 = 1,                              
    ES_NT_Nivel230 = 2,                            // name TBD !!!
    ES_NT_NivelAT4xx = 3,
};

ES_API enum ES_TipToProbeCompensationType
{
    ES_TCT_Unknown = 0,
    ES_TCT_TipOnly = 1,
    ES_TCT_ShankEnabled = 2,
};

ES_API enum ES_MeteoStationType
{
    ES_MST_None = 0,
    ES_MST_Thommen = 1,
    ES_MST_AT = 2,
};

ES_API enum ES_WLANType
{
    ES_WLAN_None = 0,
    ES_WLAN_OWL211 = 1,
    ES_WLAN_OWL221 = 2,
};

ES_API enum ES_InclinationSensorState
{
    ES_ISS_Off = 0,
    ES_ISS_ApplyCorrections = 2,
};

/////////////////////////////////////////////////////////////////////////////

// Data- types used as sub- and helper- structs for some command- structs:

// C- structures defined here are not suitable for managed C++ 
#ifndef ES_MCPP_SUPPORT

/**
Basic structure that is base of all data transmitted on the network
**/
struct PacketHeaderT
{
    long             lPacketSize;
    enum ES_DataType type;
};

/**
General data type used with all data packets on the network
**/
struct ReturnDataT
{
    struct PacketHeaderT packetHeader;
    enum ES_ResultStatus status;   
};

/**
General data type used with all commands that do not contain parameters
**/
struct BasicCommandCT
{
    struct PacketHeaderT packetHeader;
    enum ES_Command      command;
};

struct BasicCommandRT
{
    struct PacketHeaderT packetHeader;
    enum ES_Command      command;
    enum ES_ResultStatus status;
};

/**
Helper struct and union, to analyze 6D rotation status
**/
struct RotationStatus
{
    unsigned Status6D:1;                           // 0 => no rotation status; 1 => rotation status valid
    unsigned Error6D:1;                            // 1 => ERROR in rotation status
    unsigned NotEnoughLED:1;                       // 
    unsigned RMSToHigh:1;                          // 
    unsigned AngleOutOfRange:1;                    // Hz or Vt (see RotStatus values)
    unsigned Frozen6DValues:1;                     // 6D values are not updated !
    unsigned DistanceOutOfRange:1;                 // distance is too short or too long
    unsigned Reserved1:1;                          // always 0
    unsigned RotStatLeftRight:3;                   // see documentation
    unsigned RotStatUpDown:3;                      // see documentation
    unsigned GoodGauge:2;                          // 0 => All bad; 1 => 33% good; 2 => 66% good ...
    unsigned Face2:1;                              // 0 => Face1; 1 => Face2
    unsigned Reserved2:15;                         // always 0 
};

union URotationStatus
{
    long                  l;
    struct RotationStatus rotStat;
};

/////////////////////////////////////////////////////////////////////////////
/**
Result packet sent after a Nivel measurement was carried out
**/
struct NivelResultT
{
    struct ReturnDataT  packetInfo;
    enum ES_NivelStatus nivelStatus;
    double              dXTilt;
    double              dYTilt;
    double              dNivelTemperature;
};

/**
Result packet sent whenever the tracker is locked onto a reflector
this "measurements" can be switched on/off
**/
struct ReflectorPosResultT
{
    struct ReturnDataT packetInfo;
    double             dVal1;
    double             dVal2;
    double             dVal3;
};

// This structure serves for 6D probe position/orientation indication regardless
// whether RotationAngle or Quaternion rotation representation.
//
struct ProbePosResultT
{
    struct ReturnDataT           packetInfo;
    long                         lRotationStatus;
    enum ES_MeasurementTipStatus tipStatus;
    int                          iInternalTipAdapterId;
    int                          iTipAdapterInterface;
    double                       dPosition1;
    double                       dPosition2;
    double                       dPosition3;
    double                       dQuaternion0;
    double                       dQuaternion1;
    double                       dQuaternion2;
    double                       dQuaternion3;
    double                       dRotationAngleX;
    double                       dRotationAngleY;
    double                       dRotationAngleZ;
};

/**
Result packet sent after a Single Point measurement was carried out
**/
struct SingleMeasResultT
{
    struct ReturnDataT packetInfo;
    enum ES_MeasMode   measMode;
    ES_BOOL            bIsTryMode;
    double             dVal1;
    double             dVal2;
    double             dVal3;
    double             dStd1;
    double             dStd2;
    double             dStd3;
    double             dStdTotal;
    double             dPointingError1;
    double             dPointingError2;
    double             dPointingError3;
    double             dAprioriStd1;
    double             dAprioriStd2;
    double             dAprioriStd3;
    double             dAprioriStdTotal;
    double             dTemperature;
    double             dPressure;
    double             dHumidity;
};

/**
Result packet sent after a Single Point measurement was carried out
in addition to SingleMeasResultT it contains additional statiscical information
**/
struct SingleMeasResult2T
{
    struct ReturnDataT packetInfo;
    enum ES_MeasMode   measMode;
    ES_BOOL            bIsTryMode;
    double             dVal1;
    double             dVal2;
    double             dVal3;
    double             dStd1;
    double             dStd2;
    double             dStd3;
    double             dStdTotal;
    double             dCovar12;
    double             dCovar13;
    double             dCovar23;
    double             dPointingErrorH;
    double             dPointingErrorV;
    double             dPointingErrorD;
    double             dAprioriStd1;
    double             dAprioriStd2;
    double             dAprioriStd3;
    double             dAprioriStdTotal;
    double             dAprioriCovar12;
    double             dAprioriCovar13;
    double             dAprioriCovar23;
    double             dTemperature;
    double             dPressure;
    double             dHumidity;
};

/**
This struct contains a single measurement in a continuous measurement stream
**/
struct MeasValueT
{
    enum ES_MeasurementStatus status;
    long                      lTime1;
    long                      lTime2;
    double                    dVal1;
    double                    dVal2;
    double                    dVal3;
};

/**
This struct contains a single measurement in a continuous measurement stream
in addition to the MeasValueT record, it contains statistical information about the point
**/
struct MeasValue2T
{
    enum ES_MeasurementStatus status;
    long                      lTime1;
    long                      lTime2;
    double                    dVal1;
    double                    dVal2;
    double                    dVal3;
    double                    dAprioriStd1;
    double                    dAprioriStd2;
    double                    dAprioriStd3;
    double                    dAprioriStdTotal;
    double                    dAprioriCovar12;
    double                    dAprioriCovar13;
    double                    dAprioriCovar23;
};

/**
"header" part of a continuous measurement data packet
**/
struct MultiMeasResultT
{
    struct ReturnDataT packetInfo;
    long               lNumberOfResults;
    enum ES_MeasMode   measMode;
    ES_BOOL            bIsTryMode;
    double             dTemperature;
    double             dPressure;
    double             dHumidity;
    struct MeasValueT  data[1];                    // first element (exact number of elements is in lNumberOfResults)
};                         

/**
"header" part of a continuous measurement data packet
**/
struct MultiMeasResult2T
{
    struct ReturnDataT packetInfo;
    long               lNumberOfResults;
    enum ES_MeasMode   measMode;
    ES_BOOL            bIsTryMode;
    double             dTemperature;
    double             dPressure;
    double             dHumidity;
    struct MeasValue2T data[1];                    // first element (exact number of elements is in lNumberOfResults)
};                         

// This structure is used to transmit the result of a 6D stationary measurement. 
// The result depends on length and angle units, the coordinate system type, 
// orientation and transformation paramters selected.
// It contains
// - Status Information
// - Position of Tip including its accuracy
// - Probe orientation in two different representations: 
//     - Quaternion or 
//     - RotationAngle Angles including their accuraccy
//     - Environmental Data
//
struct ProbeStationaryResultT
{
    struct ReturnDataT           packetInfo;
    enum ES_MeasMode             measMode;
    ES_BOOL                      bIsTryMode;
    enum ES_TriggerStatus        triggerStatus;
    long                         lRotationStatus;
    int                          iInternalProbeId;        
    int                          iFieldNumber;        
    enum ES_MeasurementTipStatus tipStatus;
    int                          iInternalTipAdapterId;
    int                          iTipAdapterInterface;
    double                       dPosition1;
    double                       dPosition2;
    double                       dPosition3;
    double                       dStdDevPosition1;
    double                       dStdDevPosition2;
    double                       dStdDevPosition3;
    double                       dStdDevPositionTotal;
    double                       dCovarPosition12;
    double                       dCovarPosition13;
    double                       dCovarPosition23;
    double                       dAprioriStdDevPosition1;
    double                       dAprioriStdDevPosition2;
    double                       dAprioriStdDevPosition3;
    double                       dAprioriStdDevPositionTotal;
    double                       dAprioriCovarPosition12;
    double                       dAprioriCovarPosition13;
    double                       dAprioriCovarPosition23;
    double                       dQuaternion0;
    double                       dQuaternion1;
    double                       dQuaternion2;
    double                       dQuaternion3;
    double                       dRotationAngleX;
    double                       dRotationAngleY;
    double                       dRotationAngleZ;
    double                       dStdDevRotationAngleX;
    double                       dStdDevRotationAngleY;
    double                       dStdDevRotationAngleZ;
    double                       dStdDevRotationAngleTotal; 
    double                       dCovarRotationAngleXY;
    double                       dCovarRotationAngleXZ;
    double                       dCovarRotationAngleYZ;
    double                       dAprioriStdDevRotationAngleX;
    double                       dAprioriStdDevRotationAngleY;
    double                       dAprioriStdDevRotationAngleZ;
    double                       dAprioriStdDevRotationAngleTotal; 
    double                       dAprioriCovarRotationAngleXY;
    double                       dAprioriCovarRotationAngleXZ;
    double                       dAprioriCovarRotationAngleYZ;
    double                       dTemperature;
    double                       dPressure;
    double                       dHumidity;
}; 

// This struct contains a single measurement (6 degree of freedom) in a continuous 
// measurement stream 
//
struct ProbeMeasValueT
{
    enum ES_MeasurementStatus status;
    enum ES_TriggerStatus     triggerStatus;   
    long                      lRotationStatus;     // Yes, it is possible to change tips
    long                      lTime1;
    long                      lTime2;
    double                    dPosition1;
    double                    dPosition2;
    double                    dPosition3;
    double                    dStdDevPosition1;
    double                    dStdDevPosition2;
    double                    dStdDevPosition3;
    double                    dStdDevPositionTotal;
    double                    dCovarPosition12;
    double                    dCovarPosition13;
    double                    dCovarPosition23;
    double                    dQuaternion0;
    double                    dQuaternion1;
    double                    dQuaternion2;
    double                    dQuaternion3;
    double                    dRotationAngleX;
    double                    dRotationAngleY;
    double                    dRotationAngleZ;
    double                    dStdDevRotationAngleX;
    double                    dStdDevRotationAngleY;
    double                    dStdDevRotationAngleZ;
    double                    dStdDevRotationAngleTotal;
    double                    dCovarRotationAngleXY;
    double                    dCovarRotationAngleXZ;
    double                    dCovarRotationAngleYZ;
};

// "header" part of a continuous measurement (6 degree of freedom)
// data packet with full statistis 
//
struct ProbeContinuousResultT
{
    struct ReturnDataT           packetInfo;
    long                         lNumberOfResults;
    enum ES_MeasMode             measMode;
    ES_BOOL                      bIsTryMode;
    int                          iInternalProbeId;
    int                          iFieldNumber;        
    enum ES_MeasurementTipStatus tipStatus;
    int                          iInternalTipAdapterId;
    int                          iTipAdapterInterface;
    double                       dTemperature;
    double                       dPressure;
    double                       dHumidity;
    struct ProbeMeasValueT       data[1];          // first element (exact number of elements is in lNumberOfResults)
};

/**
Packet sent after a system status change
**/
struct SystemStatusChangeT
{
    struct PacketHeaderT       packetHeader;
    enum ES_SystemStatusChange systemStatusChange;
};

struct ErrorResponseT
{
    struct PacketHeaderT packetHeader;
    enum ES_Command      command;
    enum ES_ResultStatus status;
};

/////////////////////////////////////////////////////////////////////////////

// Command data- types (structs):

struct InitializeCT
{
    struct BasicCommandCT packetInfo;
};

struct InitializeRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
struct ReleaseMotorsCT
{
    struct BasicCommandCT packetInfo;
};

struct ReleaseMotorsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
struct ActivateCameraViewCT
{
    struct BasicCommandCT packetInfo;
};

struct ActivateCameraViewRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
struct ParkCT
{
    struct BasicCommandCT packetInfo;
};

struct ParkRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
struct GoBirdBathCT
{
    struct BasicCommandCT packetInfo;
};

struct GoBirdBathRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Drive to BirdBath clock- or counter clock wise
**/
struct GoBirdBath2CT
{
    struct BasicCommandCT packetInfo;
    ES_BOOL               bClockWise;
};

struct GoBirdBath2RT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
struct ChangeFaceCT
{
    struct BasicCommandCT packetInfo;
};

struct ChangeFaceRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
struct StartNivelMeasurementCT
{
    struct BasicCommandCT packetInfo;
};

struct StartNivelMeasurementRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
struct StartMeasurementCT
{
    struct BasicCommandCT packetInfo;
};

struct StartMeasurementRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
struct StopMeasurementCT
{
    struct BasicCommandCT packetInfo;
};

struct StopMeasurementRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
struct ExitApplicationCT
{
    struct BasicCommandCT packetInfo;
};

struct ExitApplicationRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Return to the last measured (single point mode) point
**/
struct GoLastMeasuredPointCT
{
    struct BasicCommandCT packetInfo;
};

struct GoLastMeasuredPointRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Switch the laser of the laser tracker ON and OFF
**/
struct SwitchLaserCT
{
    struct BasicCommandCT packetInfo;
    ES_BOOL               bIsOn;
};

struct SwitchLaserRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Call the FindReflector command
**/
struct FindReflectorCT
{
    struct BasicCommandCT packetInfo;
    double                dAproxDistance;
};

struct FindReflectorRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the active coordinate system type
**/
struct SetCoordinateSystemTypeCT
{
    struct BasicCommandCT        packetInfo;
    enum ES_CoordinateSystemType coordSysType;
};

struct SetCoordinateSystemTypeRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the active coordinate system type
**/
struct GetCoordinateSystemTypeCT
{
    struct BasicCommandCT packetInfo;
};

struct GetCoordinateSystemTypeRT
{
    struct BasicCommandRT        packetInfo;
    enum ES_CoordinateSystemType coordSysType;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the active laser tracker temperature range
**/
struct SetTemperatureRangeCT
{
    struct BasicCommandCT           packetInfo;
    enum ES_TrackerTemperatureRange temperatureRange;
};

struct SetTemperatureRangeRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the active laser tracker temperature range
**/
struct GetTemperatureRangeCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTemperatureRangeRT
{
    struct BasicCommandRT           packetInfo;
    enum ES_TrackerTemperatureRange temperatureRange;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the active measurement mode
**/
struct SetMeasurementModeCT
{
    struct BasicCommandCT packetInfo;
    enum ES_MeasMode      measMode;
};

struct SetMeasurementModeRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the active measurement mode
**/
struct GetMeasurementModeCT
{
    struct BasicCommandCT packetInfo;
};

struct GetMeasurementModeRT
{
    struct BasicCommandRT packetInfo;
    enum ES_MeasMode      measMode;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the parameters for the search functionality
**/
struct SearchParamsDataT
{
    double dSearchRadius;
    long   lTimeOut;                               // in milliseconds
};

struct SetSearchParamsCT
{
    struct BasicCommandCT    packetInfo;
    struct SearchParamsDataT searchParams;
};

struct SetSearchParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the parameters for the search functionality
**/
struct GetSearchParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetSearchParamsRT
{
    struct BasicCommandRT    packetInfo;
    struct SearchParamsDataT searchParams;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the parameters for the ADM
**/
struct AdmParamsDataT
{
    double dTargetStabilityTolerance;
    long   lRetryTimeFrame;                        // in milliseconds
    long   lNumberOfRetrys;
};

struct SetAdmParamsCT
{
    struct BasicCommandCT packetInfo;
    struct AdmParamsDataT admParams;
};

struct SetAdmParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the parameters for the ADM
**/
struct GetAdmParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetAdmParamsRT
{
    struct BasicCommandRT packetInfo;
    struct AdmParamsDataT admParams;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the parameters for the stationary measurement mode
**/
struct StationaryModeDataT
{
    long    lMeasTime;
    ES_BOOL bUseADM;                               // Caution: has no effect in 6D mode and for AT4xx trackers !
};

struct SetStationaryModeParamsCT
{
    struct BasicCommandCT      packetInfo;
    struct StationaryModeDataT stationaryModeData;
};

struct SetStationaryModeParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the parameters for the stationary measurement mode
**/
struct GetStationaryModeParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetStationaryModeParamsRT
{
    struct BasicCommandRT      packetInfo;
    struct StationaryModeDataT stationaryModeData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the parameters for the continuous time measurement mode
**/
struct ContinuousTimeModeDataT
{
    long               lTimeSeparation;
    long               lNumberOfPoints;            // ZERO means continuously
    ES_BOOL            bUseRegion;
    enum ES_RegionType regionType;    
};

struct SetContinuousTimeModeParamsCT
{
    struct BasicCommandCT          packetInfo;
    struct ContinuousTimeModeDataT continuousTimeModeData;
};

struct SetContinuousTimeModeParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the parameters for the continuous time measurement mode
**/
struct GetContinuousTimeModeParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetContinuousTimeModeParamsRT
{
    struct BasicCommandRT          packetInfo;
    struct ContinuousTimeModeDataT continuousTimeModeData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the parameters for the continuous distance measurement mode
**/
struct ContinuousDistanceModeDataT
{
    double             dSpatialDistance;
    long               lNumberOfPoints;            // ZERO means continuously
    ES_BOOL            bUseRegion;
    enum ES_RegionType regionType;    
};

struct SetContinuousDistanceModeParamsCT
{
    struct BasicCommandCT              packetInfo;
    struct ContinuousDistanceModeDataT continuousDistanceModeData;
};

struct SetContinuousDistanceModeParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the parameters for the continuous distance measurement mode
**/
struct GetContinuousDistanceModeParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetContinuousDistanceModeParamsRT
{
    struct BasicCommandRT              packetInfo;
    struct ContinuousDistanceModeDataT continuousDistanceModeData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the parameters for the sphere center measurement mode
**/
struct SphereCenterModeDataT
{
    double  dSpatialDistance;
    long    lNumberOfPoints;                       // ZERO means continuously
    ES_BOOL bFixRadius;
    double  dRadius;
};

struct SetSphereCenterModeParamsCT
{
    struct BasicCommandCT        packetInfo;
    struct SphereCenterModeDataT sphereCenterModeData;
};

struct SetSphereCenterModeParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the parameters for the sphere center measurement mode
**/
struct GetSphereCenterModeParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetSphereCenterModeParamsRT
{
    struct BasicCommandRT        packetInfo;
    struct SphereCenterModeDataT sphereCenterModeData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the parameters for the circle center measurement mode
**/
struct CircleCenterModeDataT
{
    double  dSpatialDistance;
    long    lNumberOfPoints;                       // ZERO means continuously
    ES_BOOL bFixRadius;
    double  dRadius;
};

struct SetCircleCenterModeParamsCT
{
    struct BasicCommandCT        packetInfo;
    struct CircleCenterModeDataT circleCenterModeData;
};

struct SetCircleCenterModeParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the parameters for the circle center measurement mode
**/
struct GetCircleCenterModeParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetCircleCenterModeParamsRT
{
    struct BasicCommandRT        packetInfo;
    struct CircleCenterModeDataT circleCenterModeData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the parameters for the grid measurement mode
**/
struct GridModeDataT
{
    double             dVal1;
    double             dVal2;
    double             dVal3;
    long               lNumberOfPoints;            // ZERO means continuously
    ES_BOOL            bUseRegion;
    enum ES_RegionType regionType;    
};

struct SetGridModeParamsCT
{
    struct BasicCommandCT packetInfo;
    struct GridModeDataT  gridModeData;
};

struct SetGridModeParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the parameters for the grid measurement mode
**/
struct GetGridModeParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetGridModeParamsRT
{
    struct BasicCommandRT packetInfo;
    struct GridModeDataT  gridModeData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set system specific settings

bApplyTransformationParams: 
if this flag TRUE, the embedded system tranforms the measurements into
a user specified coordinate system. If FALSE, the specified system is ignored.

bApplyStationOrientationParams:
if this flag TRUE, the embedded system uses the given orientation parameters. If  FALSE 
the station orientation will be 0.0, 0.0, 0.0, 0.0, 0.0, 0.0.

bKeepLastPosition:
If this flag is TRUE, the laser does not leave the current position after the laser
beam is broken. If the flag is FALSE, the laser moves to a save position or if a video camera
is installed, the sensor drives into the video position.

bSendUnsolicitedMessages:
If this flag is TRUE, the system sends error messages at all times (when they occur). 

bSendReflectorPositionData:
If this flag is TRUE, the system sends (at a rate of up to 10 measurements per second) the
current reflector position. This only happens, if a reflector is locked on by the tracker.

weatherMonitorStatus:
This parameter represents the possible modes for the weather monitor. 

bTryMeasurementMode:
With this flag set to TRUE, the system delivers all results in the try mode.

bHasNivel:
This flag tells the system that a nivel sensor is mounted. Measurements with this sensor are
now possible.

bHasVideoCamera:
This flag tells the system that a video camera is mounted.
**/
struct SystemSettingsDataT
{
    enum ES_WeatherMonitorStatus weatherMonitorStatus;
    ES_BOOL                      bApplyTransformationParams;
    ES_BOOL                      bApplyStationOrientationParams;
    ES_BOOL                      bKeepLastPosition;
    ES_BOOL                      bSendUnsolicitedMessages;
    ES_BOOL                      bSendReflectorPositionData;
    ES_BOOL                      bTryMeasurementMode;
    ES_BOOL                      bHasNivel;
    ES_BOOL                      bHasVideoCamera;
};                                  

struct SetSystemSettingsCT
{
    struct BasicCommandCT      packetInfo;
    struct SystemSettingsDataT systemSettings;
};

struct SetSystemSettingsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current system settings
**/
struct GetSystemSettingsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetSystemSettingsRT
{
    struct BasicCommandRT      packetInfo;
    struct SystemSettingsDataT systemSettings;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the current system units
**/
struct SystemUnitsDataT
{
    enum ES_LengthUnit      lenUnitType;
    enum ES_AngleUnit       angUnitType;
    enum ES_TemperatureUnit tempUnitType;
    enum ES_PressureUnit    pressUnitType;
    enum ES_HumidityUnit    humUnitType;
};

struct SetUnitsCT
{
    struct BasicCommandCT   packetInfo;
    struct SystemUnitsDataT unitsSettings;
};

struct SetUnitsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current system units
**/
struct GetUnitsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetUnitsRT
{
    struct BasicCommandRT   packetInfo;
    struct SystemUnitsDataT unitsSettings;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current system status
**/
struct ESVersionNumberT
{
    int iMajorVersionNumber;
    int iMinorVersionNumber;
    int iBuildNumber;
    
};

struct GetSystemStatusCT
{
    struct BasicCommandCT packetInfo;
};

struct GetSystemStatusRT
{
    struct BasicCommandRT          packetInfo;
    enum ES_ResultStatus           lastResultStatus;
    enum ES_TrackerProcessorStatus trackerProcessorStatus;
    enum ES_LaserProcessorStatus   laserStatus;
    enum ES_ADMStatus              admStatus;
    struct ESVersionNumberT        esVersionNumber;
    enum ES_WeatherMonitorStatus   weatherMonitorStatus;
    long                           lFlagsValue;                     // Always 0 (zero) in AT401 use GetMeasurementStatusInfo
    long                           lTrackerSerialNumber;
};                                

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current circle status
**/
struct GetMeasurementStatusInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetMeasurementStatusInfoRT
{
    struct BasicCommandRT packetInfo;
    enum ES_ResultStatus  lastResultStatus;
    long                  lMeasurementStatusInfo;
};                                

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current laser tracker status
**/
struct GetTrackerStatusCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTrackerStatusRT
{
    struct BasicCommandRT packetInfo;
    enum ES_TrackerStatus trackerStatus;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the current reflector
**/
struct SetReflectorCT
{
    struct BasicCommandCT packetInfo;
    int                   iInternalReflectorId;
};

struct SetReflectorRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current reflector
**/
struct GetReflectorsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetReflectorsRT
{
    struct BasicCommandRT packetInfo;
    int                   iTotalReflectors;
    int                   iInternalReflectorId;
    enum ES_TargetType    targetType;
    double                dSurfaceOffset;
    unsigned short        cReflectorName[32];      // reflector name as UNICODE string
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current reflector
**/
struct GetReflectorCT
{
    struct BasicCommandCT packetInfo;
};

struct GetReflectorRT
{
    struct BasicCommandRT packetInfo;
    int                   iInternalReflectorId;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the current temperature, pressure and humidity
**/
struct EnvironmentDataT
{
    double dTemperature;
    double dPressure;
    double dHumidity;
};

struct SetEnvironmentParamsCT
{
    struct BasicCommandCT   packetInfo;
    struct EnvironmentDataT environmentData;
};

struct SetEnvironmentParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current temperature, pressure and humidity.
If the weather monitor is active, the internal values are updated at a regular interval.
**/
struct GetEnvironmentParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetEnvironmentParamsRT
{
    struct BasicCommandRT   packetInfo;
    struct EnvironmentDataT environmentData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the current refraction data
**/
struct RefractionDataT
{
    double dIfmRefractionIndex;
    double dAdmRefractionIndex;
};

struct SetRefractionParamsCT
{
    struct BasicCommandCT  packetInfo;
    struct RefractionDataT refractionData;
};

struct SetRefractionParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current refraction values.
**/
struct GetRefractionParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetRefractionParamsRT
{
    struct BasicCommandRT  packetInfo;
    struct RefractionDataT refractionData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the current station orientation parameters
**/
struct StationOrientationDataT
{
    double dVal1;
    double dVal2;
    double dVal3;
    double dRot1;
    double dRot2;
    double dRot3;
};

struct SetStationOrientationParamsCT
{
    struct BasicCommandCT          packetInfo;
    struct StationOrientationDataT stationOrientation;
};

struct SetStationOrientationParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current station orientation parameters
**/
struct GetStationOrientationParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetStationOrientationParamsRT
{
    struct BasicCommandRT          packetInfo;
    struct StationOrientationDataT stationOrientation;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the transformation parameters
**/
struct TransformationDataT
{
    double dVal1;
    double dVal2;
    double dVal3;
    double dRot1;
    double dRot2;
    double dRot3;
    double dScale;
};

struct SetTransformationParamsCT
{
    struct BasicCommandCT      packetInfo;
    struct TransformationDataT transformationData;
};

struct SetTransformationParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the transformation parameters
**/
struct GetTransformationParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTransformationParamsRT
{
    struct BasicCommandRT      packetInfo;
    struct TransformationDataT transformationData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the box region parameters
**/
struct BoxRegionDataT
{
    double dP1Val1;
    double dP1Val2;
    double dP1Val3;
    double dP2Val1;
    double dP2Val2;
    double dP2Val3;
};

struct SetBoxRegionParamsCT
{
    struct BasicCommandCT packetInfo;
    struct BoxRegionDataT boxRegionData;
};

struct SetBoxRegionParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the box region parameters
**/
struct GetBoxRegionParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetBoxRegionParamsRT
{
    struct BasicCommandRT packetInfo;
    struct BoxRegionDataT boxRegionData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the sphere region parameters
**/
struct SphereRegionDataT
{
    double dVal1;
    double dVal2;
    double dVal3;
    double dRadius;
};

struct SetSphereRegionParamsCT
{
    struct BasicCommandCT    packetInfo;
    struct SphereRegionDataT sphereRegionData;
};

struct SetSphereRegionParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the sphere region parameters
**/
struct GetSphereRegionParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetSphereRegionParamsRT
{
    struct BasicCommandRT    packetInfo;
    struct SphereRegionDataT sphereRegionData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GoPosition command
The input parameter are according to the selected coordinate system type
**/
struct GoPositionCT
{
    struct BasicCommandCT packetInfo;
    double                dVal1;
    double                dVal2;
    double                dVal3;
    ES_BOOL               bUseADM;
};

struct GoPositionRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the LookForTarget command
The input parameter are according to the selected coordinate system type
**/
struct LookForTargetCT
{
    struct BasicCommandCT packetInfo;
    double                dVal1;
    double                dVal2;
    double                dVal3;
    double                dSearchRadius;
};

struct LookForTargetRT
{
    struct BasicCommandRT packetInfo;
    double                dHzAngle;
    double                dVtAngle;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetDirectionCT command
**/
struct GetDirectionCT
{
    struct BasicCommandCT packetInfo;
};

struct GetDirectionRT
{
    struct BasicCommandRT packetInfo;
    double                dHzAngle;
    double                dVtAngle;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GoPosition command
The input parameter are always in a spherical coordinate system type
**/
struct GoPositionHVDCT
{
    struct BasicCommandCT packetInfo;
    double                dHzAngle;
    double                dVtAngle;
    double                dDistance;
    ES_BOOL               bUseADM;
};

struct GoPositionHVDRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the Laser Pointer command
The input parameter are according to the selected coordinate system type
**/
struct PointLaserCT
{
    struct BasicCommandCT packetInfo;
    double                dVal1;
    double                dVal2;
    double                dVal3;
};

struct PointLaserRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the PositionRelative command
**/
struct PositionRelativeHVCT
{
    struct BasicCommandCT packetInfo;
    double                dHzVal;
    double                dVtVal;
};

struct PositionRelativeHVRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the LaserPointer command
The input parameter are alwasy in a spherical coordinate system type
**/
struct PointLaserHVDCT
{
    struct BasicCommandCT packetInfo;
    double                dHzAngle;
    double                dVtAngle;
    double                dDistance;
};

struct PointLaserHVDRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the MoveHV command
The input parameter are a vertical or horizontal speed value between
1% and 100% ot the total speed of the tracker.
**/
struct MoveHVCT
{
    struct BasicCommandCT packetInfo;
    int                   iHzSpeed;
    int                   iVtSpeed;
};

struct MoveHVRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the NivelMove command
The input parameter are the defined nivel positions (1 to 4). The laser tracker moves at a much 
slower speed. This is to avoid disturbing the nivel sensor.

  This command is used for the orient to gravity procedure.
**/
struct GoNivelPositionCT
{
    struct BasicCommandCT packetInfo;
    enum ES_NivelPosition nivelPosition;
};

struct GoNivelPositionRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the OrientToGravity process
**/
struct CallOrientToGravityCT
{
    struct BasicCommandCT packetInfo;
};

struct CallOrientToGravityRT
{
    struct BasicCommandRT packetInfo;
    double                dOmega;
    double                dPhi;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the IntermediateCompensation process
**/
struct CallIntermediateCompensationCT
{
    struct BasicCommandCT packetInfo;
};

struct CallIntermediateCompensationRT
{
    struct BasicCommandRT packetInfo;
    double                dTotalRMS;
    double                dMaxDev;
    long                  lWarningFlags;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the Transformation process
**/
struct CallTransformationCT
{
    struct BasicCommandCT packetInfo;
};

struct CallTransformationRT
{
    struct BasicCommandRT packetInfo;
    double                dTransVal1;
    double                dTransVal2;
    double                dTransVal3;
    double                dRotVal1;
    double                dRotVal2;
    double                dRotVal3;
    double                dScale;
    double                dTransStdVal1;
    double                dTransStdVal2;
    double                dTransStdVal3;
    double                dRotStdVal1;
    double                dRotStdVal2;
    double                dRotStdVal3;
    double                dScaleStd;
    double                dRMS;
    double                dMaxDev;
    double                dVarianceFactor;
};                            

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the input parameters for the
transformation process
**/
struct TransformationInputDataT
{
    enum ES_TransResultType resultType;  
    double                  dTransVal1;
    double                  dTransVal2;
    double                  dTransVal3;
    double                  dRotVal1;
    double                  dRotVal2;
    double                  dRotVal3;
    double                  dScale;
    double                  dTransStdVal1;
    double                  dTransStdVal2;
    double                  dTransStdVal3;
    double                  dRotStdVal1;
    double                  dRotStdVal2;
    double                  dRotStdVal3;
    double                  dScaleStd;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the SetTransformationInputParams command
**/
struct SetTransformationInputParamsCT
{
    struct BasicCommandCT           packetInfo;
    struct TransformationInputDataT transformationData;
};

struct SetTransformationInputParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetTransformationInputParams command
**/
struct GetTransformationInputParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTransformationInputParamsRT
{
    struct BasicCommandRT           packetInfo;
    struct TransformationInputDataT transformationData;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the ClearTransformationNominalPointList command
this command clears the nominal point list used for the transformation
**/
struct ClearTransformationNominalPointListCT
{
    struct BasicCommandCT packetInfo;
};

struct ClearTransformationNominalPointListRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the ClearTransformationActualPointList command
this command clears the actual point list used for the transformation
**/
struct ClearTransformationActualPointListCT
{
    struct BasicCommandCT packetInfo;
};

struct ClearTransformationActualPointListRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to add points to the point lists for the
transformation process
**/
struct TransformationPointT
{
    double dVal1;
    double dVal2;
    double dVal3;
    double dStd1;
    double dStd2;
    double dStd3;   
    double dCovar12;
    double dCovar13;
    double dCovar23;   
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the AddTransformationNominalPoint command
**/
struct AddTransformationNominalPointCT
{
    struct BasicCommandCT       packetInfo;
    struct TransformationPointT transformationPoint;
};

struct AddTransformationNominalPointRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the AddTransformationActualPoint command
**/
struct AddTransformationActualPointCT
{
    struct BasicCommandCT        packetInfo;
    struct TransformationPointT transformationPoint;
};

struct AddTransformationActualPointRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetTransformedPointss command
this command starts a transmission, where all records in the 
transformed point's and residual's
list will be sent to the caller
**/
struct GetTransformedPointsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTransformedPointsRT
{
    struct BasicCommandRT packetInfo;
    int                   iTotalPoints;
    double                dVal1;
    double                dVal2;
    double                dVal3;
    double                dStd1;
    double                dStd2;
    double                dStd3;
    double                dStdTotal;
    double                dCovar12;
    double                dCovar13;
    double                dCovar23;
    double                dResidualVal1;
    double                dResidualVal2;
    double                dResidualVal3;   
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the ClearDrivePointList command
this command clears the drive point list used for the intermediate compensation
**/
struct ClearDrivePointListCT
{
    struct BasicCommandCT packetInfo;
};

struct ClearDrivePointListRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the AddDrivePoint command
**/
struct AddDrivePointCT
{
    struct BasicCommandCT packetInfo;
    int                   iInternalReflectorId;
    double                dVal1;
    double                dVal2;
    double                dVal3;
};

struct AddDrivePointRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the SetCompensation command
this command clears the drive point list used for the intermediate compensation
**/
struct SetCompensationCT
{
    struct BasicCommandCT packetInfo;
    int                   iInternalCompensationId;
};

struct SetCompensationRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the currently active compensation ID
**/
struct GetCompensationCT
{
    struct BasicCommandCT packetInfo;
};

struct GetCompensationRT
{
    struct BasicCommandRT packetInfo;
    int                   iInternalCompensationId;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read all the compensations currently 
stored in the systems database
**/
struct GetCompensationsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetCompensationsRT
{
    struct BasicCommandRT packetInfo;
    int                   iTotalCompensations;
    int                   iInternalCompensationId;
    unsigned short        cTrackerCompensationName[32];      // tracker compensation name as UNICODE string
    unsigned short        cTrackerCompensationComment[128];  // comment in UNICODE
    unsigned short        cADMCompensationName[32];          // ADM compensation name as UNICODE string
    ES_BOOL               bHasMeasurementCameraMounted;      // Used for 6D systems
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read all the compensations currently 
stored in the systems database
**/
struct GetCompensations2CT
{
    struct BasicCommandCT packetInfo;
};

struct GetCompensations2RT
{
    struct BasicCommandRT packetInfo;
    int                   iTotalCompensations;
    int                   iInternalCompensationId;
    unsigned short        cTrackerCompensationName[32];      // tracker compensation name as UNICODE string
    unsigned short        cTrackerCompensationComment[128];  // comment in UNICODE
    unsigned short        cADMCompensationName[32];          // ADM compensation name as UNICODE string
    unsigned short        cADMCompensationComment[128];      // comment in UNICODE
    ES_BOOL               bHasMeasurementCameraMounted;      // Used for 6D systems
    ES_BOOL               bIsActive;                         // Will be used in this configuration
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the SetStatisticMode command
This command selects the amount of statistical information returned by the 
system for stationary and/or continuous measurement modes
**/
struct SetStatisticModeCT
{
    struct BasicCommandCT   packetInfo;
    enum   ES_StatisticMode stationaryMeasurements;
    enum   ES_StatisticMode continuousMeasurements;
};

struct SetStatisticModeRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetStatisticMode command
**/
struct GetStatisticModeCT
{
    struct BasicCommandCT packetInfo;
};

struct GetStatisticModeRT
{
    struct BasicCommandRT   packetInfo;
    enum   ES_StatisticMode stationaryMeasurements;
    enum   ES_StatisticMode continuousMeasurements;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the parameters for the video camera
**/
struct CameraParamsDataT
{
    int iContrast;
    int iBrightness;
    int iSaturation;
};

struct SetCameraParamsCT
{
    struct BasicCommandCT    packetInfo;
    struct CameraParamsDataT cameraParams;
};

struct SetCameraParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the parameters for the video camera
**/
struct GetCameraParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetCameraParamsRT
{
    struct BasicCommandRT    packetInfo;
    struct CameraParamsDataT cameraParams;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetStillImage command
**/
struct GetStillImageCT
{
    struct BasicCommandCT        packetInfo;
    enum   ES_StillImageFileType imageFileType;
};

struct GetStillImageRT
{
    struct BasicCommandRT        packetInfo;
    enum   ES_StillImageFileType imageFiletype;
    long                         lFileSize;
    char                         cFileStart;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to check the bird bath position of the 
currently selected reflector.
**/
struct CheckBirdBathCT
{
    struct BasicCommandCT packetInfo;
};

struct CheckBirdBathRT
{
    struct BasicCommandRT packetInfo;
    double                dInitialHzAngle;
    double                dInitialVtAngle;
    double                dInitialDistance;
    double                dHzAngleDiff;
    double                dVtAngleDiff;
    double                dDistanceDiff;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read laser tracker diagnostic data
**/
struct GetTrackerDiagnosticsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTrackerDiagnosticsRT
{
    struct BasicCommandRT packetInfo;
    double                dTrkPhotoSensorXVal;
    double                dTrkPhotoSensorYVal;
    double                dTrkPhotoSensorIVal;
    double                dRefPhotoSensorXVal;
    double                dRefPhotoSensorYVal;
    double                dRefPhotoSensorIVal;
    double                dADConverterRange;
    double                dServoControlPointX;
    double                dServoControlPointY;
    double                dLaserLightRatio;
    int                   iLaserControlMode;
    double                dSensorInsideTemperature;
    int                   iLCPRunTime;
    int                   iLaserTubeRunTime;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to get additional ADM diagnostic data
Only works with the device connected and selected
**/
struct GetADMInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetADMInfoRT
{
    struct BasicCommandRT packetInfo;
    int                   iFirmwareMajorVersionNumber;
    int                   iFirmwareMinorVersionNumber;
    long                  lSerialNumber;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to get additional ADM diagnostic data
Only works with the device connected and selected
**/
struct GetADMInfo2CT
{
    struct BasicCommandCT packetInfo;
};

struct GetADMInfo2RT
{
    struct BasicCommandRT packetInfo;
    enum ES_ADMType       admType;
    unsigned short        cADMName[32];                  // UNICODE string
    long                  lSerialNumber;
    int                   iFirmwareMajorVersionNumber;
    int                   iFirmwareMinorVersionNumber;
    double                dMaxDistance;                  // maximal ADM distance
    double                dMinDistance;                  // minimal ADM distance
    int                   iMaxDataRate;                  // measurements per second
    double                dAccuracyADMDistance;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to get additional Nivel diagnostic data
Only works with the device connected and selected
**/
struct GetNivelInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetNivelInfoRT
{
    struct BasicCommandRT packetInfo;
    int                   iFirmwareMajorVersionNumber;
    int                   iFirmwareMinorVersionNumber;
    long                  lSerialNumber;
};

struct GetNivelInfo2CT
{
    struct BasicCommandCT packetInfo;
};

struct GetNivelInfo2RT
{
    struct BasicCommandRT packetInfo;
    enum ES_NivelType     nivelType;
    unsigned short        cNivelName[32];                // UNICODE string
    long                  lSerialNumber;
    int                   iFirmwareMajorVersionNumber;
    int                   iFirmwareMinorVersionNumber;
    double                dMeasurementRange;             // +- in radians
    double                dMeasurementAccuracyOffset;    // Accuracy = Offset + (Facor * Reading)
    double                dMeasurementAccuracyFactor;                
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to get additional TP diagnostic data
**/
struct GetTPInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTPInfoRT
{
    struct BasicCommandRT        packetInfo;
    int                          iTPBootMajorVersionNumber;
    int                          iTPBootMinorVersionNumber;
    int                          iTPFirmwareMajorVersionNumber;
    int                          iTPFirmwareMinorVersionNumber;
    int                          iLCPFirmwareMajorVersionNumber;
    int                          iLCPFirmwareMinorVersionNumber;
    enum ES_TrackerProcessorType trackerprocessorType;
    enum ES_TPMicroProcessorType microProcessorType;
    int                          iMicroProcessorClockSpeed;
    enum ES_LTSensorType         laserTrackerSensorType;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to get Tracker features data
**/
struct GetTrackerInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTrackerInfoRT
{
    struct BasicCommandRT    packetInfo;
    enum ES_LTSensorType     trackerType;
    unsigned short           cTrackerName[32];           // UNICODE string
    long                     lSerialNumber;
    long                     lCompensationIdNumber;      // identifies the compensation
    ES_BOOL                  bHasADM;
    ES_BOOL                  bHasOverviewCamera;
    ES_BOOL                  bHasNivel;
    double                   dNivelMountOffset;
    double                   dMaxDistance;               // maximal distance for laser
    double                   dMinDistance;               // minimal distance for laser
    int                      iMaxDataRate;               // measurements per second
    int                      iNumberOfFaces;
    double                   dHzAngleRange;              // e.g.: +/- 240
    double                   dVtAngleRange;
    enum ES_TrkAccuracyModel accuracyModel;
    int                      iMajLCPFirmwareVersion;
    int                      iMinLCPFirmwareVersion;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to get ATR diagnostic data
**/
struct GetATRInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetATRInfoRT
{
    struct BasicCommandRT packetInfo;
    enum ES_ATRType       atrType;
    unsigned short        cATRName[32];                  // UNICODE string
    long                  lMajFirmwareVersion;
    long                  lMinFirmwareVersion;
    long                  lBuildFirmwareVersion;
    long                  lHardwareVersion;
    long                  lErrorcode;                    // 0 ==> All OK
    long                  lFPGAVersion;
    double                dMaxDistance;
    double                dMinDistance;
    double                dFieldOfView;
    double                dMaxTrackingSpeed;             // 0.0 ==> stationary only
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the time (hour, minute) when the laser will
be switched on by the system (must be running). 
The input is rounded to the nearest 15 minutes
**/
struct SetLaserOnTimerCT
{
    struct BasicCommandCT packetInfo;
    int                   iLaserOnTimeOffsetHour;
    int                   iLaserOnTimeOffsetMinute;
};

struct SetLaserOnTimerRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the time (hour, minute) until the laser will
be switched on. This value is ZERO after a system restart.
The value is rounded to the nearest 15 minutes
**/
struct GetLaserOnTimerCT
{
    struct BasicCommandCT packetInfo;
};

struct GetLaserOnTimerRT
{
    struct BasicCommandRT packetInfo;
    int                   iLaserOnTimeOffsetHour;
    int                   iLaserOnTimeOffsetMinute;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the DisplayCoordinateConversion function
**/
struct ConvertDisplayCoordinatesCT
{
    struct BasicCommandCT                   packetInfo;
    enum ES_DisplayCoordinateConversionType conversionType; 
    double                                  dVal1;
    double                                  dVal2;
    double                                  dVal3;
};

struct ConvertDisplayCoordinatesRT
{
    struct BasicCommandRT packetInfo;
    double                dVal1;
    double                dVal2;
    double                dVal3;
};                            

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the active trigger source
**/
struct SetTriggerSourceCT
{
    struct BasicCommandCT packetInfo;
    enum ES_TriggerSource triggerSource;
};

struct SetTriggerSourceRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the active trigger source
**/
struct GetTriggerSourceCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTriggerSourceRT
{
    struct BasicCommandRT packetInfo;
    enum ES_TriggerSource triggerSource;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current laser tracker face information
**/
struct GetFaceCT
{
    struct BasicCommandCT packetInfo;
};

struct GetFaceRT
{
    struct BasicCommandRT packetInfo;
    enum ES_TrackerFace   trackerFace;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the cameras known to the system
**/
struct GetCamerasCT
{
    struct BasicCommandCT packetInfo;
};

struct GetCamerasRT
{
    struct BasicCommandRT         packetInfo;
    int                           iTotalCameras;
    int                           iInternalCameraId;
    long                          lSerialNumber;
    enum ES_MeasurementCameraType cameraType;
    unsigned short                cName[32];       // UNICODE strings
    unsigned short                cComment[128];
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current camera
**/
struct GetCameraCT
{
    struct BasicCommandCT packetInfo;
};

struct GetCameraRT
{
    struct BasicCommandRT packetInfo;
    int                   iInternalCameraId;
    ES_BOOL               bMeasurementCameraIsMounted;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the current camera mode
**/
struct SetMeasurementCameraModeCT
{
    struct BasicCommandCT         packetInfo;
    enum ES_MeasurementCameraMode cameraMode;
};

struct SetMeasurementCameraModeRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set the current camera mode
**/
struct GetMeasurementCameraModeCT
{
    struct BasicCommandCT packetInfo;
};

struct GetMeasurementCameraModeRT
{
    struct BasicCommandRT         packetInfo;
    enum ES_MeasurementCameraMode cameraMode;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the probes known to the system
**/
struct GetProbesCT
{
    struct BasicCommandCT packetInfo;
};

struct GetProbesRT
{
    struct BasicCommandRT packetInfo;
    int                   iTotalProbes;
    int                   iInternalProbeId;
    long                  lSerialNumber; 
    enum ES_ProbeType     probeType;
    int                   iNumberOfFields;         // How many fields (faces) does the probe have
    unsigned short        cName[32];               // UNICODE strings
    unsigned short        cComment[128];
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current camera
**/
struct GetProbeCT
{
    struct BasicCommandCT packetInfo;
};

struct GetProbeRT
{
    struct BasicCommandRT packetInfo;
    int                   iInternalProbeId;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the tips known to the system
**/
struct GetTipAdaptersCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTipAdaptersRT
{
    struct BasicCommandRT packetInfo;
    int                   iTotalTipAdapters;
    int                   iInternalTipAdapterId;
    long                  lAssemblyId;
    long                  lSerialNumberLowPart;
    long                  lSerialNumberHighPart;
    enum ES_TipType       tipType;
    double                dRadius; 
    double                dLength;
    unsigned short        cName[32];               // UNICODE strings
    unsigned short        cComment[128];
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the current tip
**/
struct GetTipAdapterCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTipAdapterRT
{
    struct BasicCommandRT packetInfo;
    int                   iInternalTipAdapterId;
    int                   iTipAdapterInterface;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read all the TCam to Tracker compensations  
currently stored in the systems database
**/
struct GetTCamToTrackerCompensationsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTCamToTrackerCompensationsRT
{
    struct BasicCommandRT packetInfo;
    int                   iTotalCompensations;
    int                   iInternalTCamToTrackerCompensationId;
    int                   iInternalTrackerCompensationId;
    int                   iInternalCameraId;
    ES_BOOL               bIsActive;
    long                  lTrackerSerialNumber;
    unsigned short        cTCamToTrackerCompensationName[32];  // UNICODE strings
    unsigned short        cTCamToTrackerCompensationComment[128];
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type activate a TCam to Tracker compensation
**/
struct SetTCamToTrackerCompensationCT
{
    struct BasicCommandCT packetInfo;
    int                   iInternalTCamToTrackerCompensationId;
};

struct SetTCamToTrackerCompensationRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the currently active TCam to Tracker ID
**/
struct GetTCamToTrackerCompensationCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTCamToTrackerCompensationRT
{
    struct BasicCommandRT packetInfo;
    int                   iInternalTCamToTrackerCompensationId;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read all the Probe compensations  
currently stored in the systems database
**/
struct GetProbeCompensationsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetProbeCompensationsRT
{
    struct BasicCommandRT packetInfo;
    int                   iTotalCompensations;
    int                   iInternalProbeCompensationId;
    int                   iInternalProbeId;
    int                   iFieldNumber;
    ES_BOOL               bIsActive;
    ES_BOOL               bMarkedForExport;
    ES_BOOL               bPreliminary;
    unsigned short        cProbeCompensationName[32];        // UNICODE strings
    unsigned short        cProbeCompensationComment[128]; 
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type activate a Probe compensation
**/
struct SetProbeCompensationCT
{
    struct BasicCommandCT packetInfo;
    int                   iInternalProbeCompensationId;
};

struct SetProbeCompensationRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the currently active Probe compensation ID
**/
struct GetProbeCompensationCT
{
    struct BasicCommandCT packetInfo;
};

struct GetProbeCompensationRT
{
    struct BasicCommandRT packetInfo;
    int                   iInternalProbeCompensationId;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read all the Tip to probe compensations  
currently stored in the systems database
**/
struct GetTipToProbeCompensationsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTipToProbeCompensationsRT
{
    struct BasicCommandRT packetInfo;
    int                   iTotalCompensations;
    int                   iInternalTipToProbeCompensationId;
    int                   iInternalTipAdapterId;
    int                   iTipAdapterInterface;
    int                   iInternalProbeCompensationId;
    ES_BOOL               bMarkedForExport;
    unsigned short        cTipToProbeCompensationName[32];   // UNICODE strings
    unsigned short        cTipToProbeCompensationComment[128];
};

struct GetTipToProbeCompensations2CT
{
    struct BasicCommandCT packetInfo;
};

struct GetTipToProbeCompensations2RT
{
    struct BasicCommandRT              packetInfo;
    int                                iTotalCompensations;
    int                                iInternalTipToProbeCompensationId;
    int                                iInternalTipAdapterId;
    int                                iTipAdapterInterface;
    int                                iInternalProbeCompensationId;
    ES_BOOL                            bMarkedForExport;
    enum ES_TipToProbeCompensationType compensationType;
    unsigned short                     cTipToProbeCompensationName[32];  // UNICODE strings
    unsigned short                     cTipToProbeCompensationComment[128];
    unsigned short                     cShankCompensationName[32];
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the currently active TCam to Tracker ID
**/
struct GetTipToProbeCompensationCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTipToProbeCompensationRT
{
    struct BasicCommandRT packetInfo;
    int                   iInternalTipToProbeCompensationId;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use thes data types to set the trigger parameters
**/
struct ExternTriggerParamsT
{
   enum ES_ClockTransition    clockTransition;
   enum ES_TriggerMode        triggerMode;
   enum ES_TriggerStartSignal startSignal;
   long                       lMinimalTimeDelay;
};

struct SetExternTriggerParamsCT
{
    struct BasicCommandCT       packetInfo;
    struct ExternTriggerParamsT triggerParams;
};

struct SetExternTriggerParamsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read the  trigger parameters
**/
struct GetExternTriggerParamsCT
{
    struct BasicCommandCT packetInfo;
};

struct GetExternTriggerParamsRT
{
    struct BasicCommandRT       packetInfo;
    struct ExternTriggerParamsT triggerParams;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use these data types to calculate the error ellipsoid
**/
struct GetErrorEllipsoidCT
{
    struct BasicCommandCT packetInfo;
    double                dCoord1; 
    double                dCoord2; 
    double                dCoord3; 
    double                dStdDev1; 
    double                dStdDev2; 
    double                dStdDev3; 
    double                dCovar12; 
    double                dCovar13; 
    double                dCovar23;
};

struct GetErrorEllipsoidRT
{
    struct BasicCommandRT packetInfo;
    double                dStdDevX; 
    double                dStdDevY; 
    double                dStdDevZ; 
    double                dRotationAngleX; 
    double                dRotationAngleY; 
    double                dRotationAngleZ;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to get additional Measurement Camera diagnostic data
Only works with the device connected and selected
**/
struct GetMeasurementCameraInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetMeasurementCameraInfoRT
{
    struct BasicCommandRT         packetInfo;
    int                           iFirmwareMajorVersionNumber;
    int                           iFirmwareMinorVersionNumber;
    long                          lSerialNumber;
    enum ES_MeasurementCameraType cameraType;
    unsigned short                cName[32];                 // UNICODE strings
    long                          lCompensationIdNumber;     // identifies the compensation
    long                          lZoomSerialNumber;
    long                          lZoomAdjustmentIdNumber;
    long                          lZoom2DCompensationIdNumber;
    long                          lZoomProjCenterCompIdNumber;
    double                        dMaxDistance;
    double                        dMinDistance;
    long                          lNrOfPixelsX;
    long                          lNrOfPixelsY;
    double                        dPixelSizeX;
    double                        dPixelSizeY;
    long                          lMaxDataRate;              // measurements per second
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to get additional Measurement Probe diagnostic data
Only works with the device connected and selected
**/
struct GetMeasurementProbeInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetMeasurementProbeInfoRT
{
    struct BasicCommandRT       packetInfo;
    int                         iFirmwareMajorVersionNumber;
    int                         iFirmwareMinorVersionNumber;
    long                        lSerialNumber;
    enum ES_ProbeType           probeType;
    long                        lCompensationIdNumber;       // identifies the compensation
    long                        lActiveField;                // 0...
    enum ES_ProbeConnectionType connectionType;
    long                        lNumberOfTipAdapters;        // how many tip adapters
    enum ES_ProbeButtonType     probeButtonType;
    long                        lNumberOfFields;             // Number of probes in a probe!
    ES_BOOL                     bHasWideAngleReceiver;
    long                        lNumberOfTipDataSets;        // available space for iNumberOfTipDataSets TipDataSets
    long                        lNumberOfMelodies;           // available sounds on probe
    long                        lNumberOfLoudnesSteps;       // ZERO -> No sound available
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to set an individual system parameter
See ES_SystemParameter for possible parameters
**/
struct SetLongSystemParamCT
{
    struct BasicCommandCT   packetInfo;
    enum ES_SystemParameter systemParam;
    long                    lParameter;
};

struct SetLongSystemParamRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to read an individual system parameter
See ES_SystemParameter for possible parameters
**/
struct GetLongSystemParamCT
{
    struct BasicCommandCT   packetInfo;
    enum ES_SystemParameter systemParam;
};

struct GetLongSystemParamRT
{
    struct BasicCommandRT   packetInfo;
    enum ES_SystemParameter systemParam;
    long                    lParameter;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetCurrentPrismPosition command
**/
struct GetCurrentPrismPositionCT
{
    struct BasicCommandCT packetInfo;
};

struct GetCurrentPrismPositionRT
{
    struct BasicCommandRT packetInfo;
    double                dVal1;
    double                dVal2;
    double                dVal3;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetObjTemperature command
**/
struct GetObjectTemperatureCT
{
    struct BasicCommandCT packetInfo;
};

struct GetObjectTemperatureRT
{
    struct BasicCommandRT packetInfo;
    double                dObjectTemperature;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the ClearCommandQueue command
**/
struct ClearCommandQueueCT
{
    struct BasicCommandCT         packetInfo;
    enum ES_ClearCommandQueueType clearQueueType;
};

struct ClearCommandQueueRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetTriggerBoardInfo command
**/
struct GetTriggerBoardInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetTriggerBoardInfoRT
{
    struct BasicCommandRT   packetInfo;
    enum ES_TriggerCardType triggerCardType;                                    
    long                    lFPGAVersion;
    long                    lMaxTriggerFrequency;
    long                    lErrorCode;            // 0 ==> All OK
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetOverviewCameraInfo command
**/
struct GetOverviewCameraInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetOverviewCameraInfoRT
{
    struct BasicCommandRT      packetInfo;
    enum ES_OverviewCameraType cameraType;
    unsigned short             cCameraName[32];    // UNICODE strings
    ES_BOOL                    bIsColorCamera;
    double                     dFocalLength;
    double                     dHorizontalChipSize;
    double                     dVerticalChipSize;
    ES_BOOL                    bMirrorImageHz;
    ES_BOOL                    bMirrorImageVt;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetDoubleSystemParameter command
**/
struct GetDoubleSystemParamCT
{
    struct BasicCommandCT   packetInfo;
    enum ES_SystemParameter systemParam;
};

struct GetDoubleSystemParamRT
{
    struct BasicCommandRT   packetInfo;
    enum ES_SystemParameter systemParam;
    double                  dParameter;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the SetDoubleSystemParameter command
**/
struct SetDoubleSystemParamCT
{
    struct BasicCommandCT   packetInfo;
    enum ES_SystemParameter systemParam;
    double                  dParameter;
};

struct SetDoubleSystemParamRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the RestoreStartupConditions command
**/
struct RestoreStartupConditionsCT
{
    struct BasicCommandCT packetInfo;
};

struct RestoreStartupConditionsRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GoAndMeasure command
**/
struct GoAndMeasureCT
{
    struct BasicCommandCT packetInfo;
    double                dVal1;
    double                dVal2;
    double                dVal3;
};

struct GoAndMeasureRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to activate a "virtual" Tip
**/
struct SetTipAdapterCT
{
    struct BasicCommandCT packetInfo;
    int                   iInternalTipAdapterId;
};

struct SetTipAdapterRT
{
    struct BasicCommandRT packetInfo;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to get additional meteo station data
Only works with the device connected and switched on
**/
struct GetMeteoStationInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetMeteoStationInfoRT
{
    struct BasicCommandRT    packetInfo;
    enum ES_MeteoStationType meteoStationType;
    unsigned short           cIdentifier[32];      // UNICODE string
    int                      iFirmwareMajorVersionNumber;
    int                      iFirmwareMinorVersionNumber;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to get AT4xx sensor information
Only works with the device connected and switched on
**/
struct GetAT4xxInfoCT
{
    struct BasicCommandCT packetInfo;
};

struct GetAT4xxInfoRT
{
    struct BasicCommandRT        packetInfo;
    enum ES_LTSensorType         trackerType;
    unsigned short               cTrackerName[32]; // UNICODE string
    long                         lSerialNumber;
    long                         lMajorFirmwareVersion;
    long                         lMinorFirmwareVersion;
    long                         lProcessorBoardFWBuildNumber;
    long                         lSensorBoardFWBuildNumber;
    long                         lMajorOSVersion;
    long                         lMinorOSVersion;
    long                         lMajorServerSoftwareVersion;
    long                         lMinorServerSoftwareVersion;
    long                         lServerSoftwareBuildNumber;
    enum ES_WLANType             wlanType;
    enum ES_TPMicroProcessorType xscaleType;
    long                         lMinMeasureTime;
    double                       dMinDistance;
    double                       dMaxDistance;
    double                       dStdDevDistOffsetADM;
    double                       dStdDevAngleConst;
    double                       dStdDevAngleOffset;
    double                       dStdDevAngleFactor;
};

/////////////////////////////////////////////////////////////////////////////
/**
Use this data type to call the GetSystemSoftwareVersion command
**/
struct GetSystemSoftwareVersionCT
{
    struct BasicCommandCT packetInfo;
};

struct GetSystemSoftwareVersionRT
{
    struct BasicCommandRT packetInfo;
    unsigned short        cSoftwareVersion[32]; // UNICODE string
};


#endif /* ES_MCPP_SUPPORT */

/////////////////////////////////////////////////////////////////////////////

// restore old byte alignment from stack
#pragma pack (pop)

/////////////////////////////////////////////////////////////////////////////

#endif /* ES_API_VERSION_INC_ONLY */

#endif /* ES_C_API_DEF_H */


