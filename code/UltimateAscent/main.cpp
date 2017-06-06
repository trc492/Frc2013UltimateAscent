#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="main.cpp" />
///
/// <summary>
///     This main module contains the entry point of the program.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

//
// Program switches.
//
//#define _ENABLE_COMPETITION
#define _USE_TANK_DRIVE
#define _USE_DUAL_JOYSTICKS
#define _USE_COLORFONT
#define _USE_COMPETITION_ROBOT

#define _USE_VISION

#ifndef _ENABLE_COMPETITION
#define _DBGTRACE_ENABLED
//#define _USE_MECANUM

//#define _TUNE_TURN_PID
//#define _TUNE_SHOOTER_PID

//#define _DEBUG_DRIVEBASE
//#define _DEBUG_SHOOTER
//#define _DEBUG_TRCSOL
//#define _DEBUG_TARGET
//#define _DEBUG_PANTILT

//#define _LOGDATA_PIDCTRL
//#define _LOGDATA_SHOOTER
//#define _LOGDATA_DRIVEBASE
//#define _LOGDATA_JOYSTICK
//#define _LOGDATA_TILT
//#define _ENABLE_DATALOGGER

//#define _VISION_PERF
//#define _CANJAG_PERF
#ifdef _CANJAG_PERF
  #define _PERFDATA_LOOP
#endif
#endif

#define PROGRAM_NAME            "Ultimate Ascent"

//
// Library includes.
//
#include "WPILib.h"
#include "RobotInfo.h"          //Robot configurations
#include "..\frclib\TrcLib.h"

//
// Tracing info.
//
#define MOD_DRIVEBASE           TGenModId(1)
#define MOD_SHOOTER             TGenModId(2)
#define MOD_HANGER              TGenModId(3)
#define MOD_PANTILT             TGenModId(4)
#define MOD_TARGET              TGenModId(5)
#define MOD_FEEDER              TGenModId(6)

#define TRACE_MODULES           (MOD_MAIN)
#define TRACE_LEVEL             FUNC
#define MSG_LEVEL               INFO

//
// Project includes.
//
#include "DashboardDataFormat.h"

#include "VisionTarget.h"       //VisionTarget subsystem
#include "DriveBase.h"          //DriveBase subsystem
#include "PanTilt.h"            //PanTilt subsystem
#include "Feeder.h"             //Feeder subsystem
#include "Shooter.h"            //Shooter subsystem
#include "Hanger.h"             //Hanger subsystem

#include "TrcRobot.h"           //Main Robot module
#include "Disabled.h"           //Disabled mode
#include "Auto.h"               //Autonomous mode
#include "TeleOp.h"             //TeleOp mode

START_ROBOT_CLASS(TrcRobot);
