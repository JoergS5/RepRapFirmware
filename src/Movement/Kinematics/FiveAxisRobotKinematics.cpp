/*
 * FiveAxisRobotKinematics.cpp
 *
 *  Created on: 09 Aug 2020
 *      Author: JoergS5
 *
 *	documentation: https://duet3d.dozuki.com/Wiki/Configuring_RepRapFirmware_for_a_FiveAxisRobot?revisionid=HEAD
 */

#include "FiveAxisRobotKinematics.h"
#include "RepRap.h"
#include "Platform.h"
#include "Storage/MassStorage.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Movement/DDA.h"

#include <limits>

//#define debugPrintf if(0) debugPrintf

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(FiveAxisRobotKinematics, __VA_ARGS__)


constexpr ObjectModelTableEntry FiveAxisRobotKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "name",	OBJECT_MODEL_FUNC(self->GetName(true)), 	ObjectModelEntryFlags::none },
	//TODO lots more to be added here
};

constexpr uint8_t FiveAxisRobotKinematics::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE(FiveAxisRobotKinematics)

#endif

FiveAxisRobotKinematics::FiveAxisRobotKinematics() noexcept
	: Kinematics(KinematicsType::fiveAxisRobot, DefaultSegmentsPerSecond, DefaultMinSegmentSize, true)
{
	Recalc();
}

///////////////////////////// public functions /////////////////////////

// Return the name of the current kinematics
const char *FiveAxisRobotKinematics::GetName(bool forStatusReport) const noexcept
{
	return "FiveAxisRobot";
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool FiveAxisRobotKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) /*override*/
{
	if (mCode == 669)
	{
		float arr[10];		// return values, maximum for A
		size_t length;		// returns count of parameters
		float val;			// return value
		int32_t valInt;
		bool seen = false;			// return value
		bool seenNonGeometry = false;

		if (gb.Seen('X')) {
			gb.GetFloatArray(arr, length, false);
			if(length >= 2) {
				axis1coords[0] = arr[0];
				axis2coords[0] = arr[1];
			}
			if(length == 3) {
				axis6coords[0] = arr[2];
				useRail = true;
			}
			if(length == 2) {
				useRail = false;
			}
		}

		if (gb.Seen('Y')) {
			gb.GetFloatArray(arr, length, false);
			if(length == 2) {
				axis1coords[1] = arr[0];
				axis2coords[1] = arr[1];
			}
		}

		if (gb.Seen('Z')) {
			gb.GetFloatArray(arr, length, false);
			if(length == 1) {
				axis2coords[2] = arr[0];
			}
		}

		if (gb.Seen('P')) {
			gb.TryGetIValue('P', valInt, seen);
			if(seen) {
				currentPstrategy = val;
				previousPstrategy = 0;
			}
		}
		else {
			currentPstrategy = 0;
			previousPstrategy = 0;
		}

		if (gb.Seen('L')) {		// arm 2 to 5 lengths
			gb.GetFloatArray(arr, length, false);
			if(length == 4) {
				arm2length = arr[0];
				arm3length = arr[1];
				arm4length = arr[2];
				arm5length = arr[3];
			}
		}

		if (gb.Seen('A')) {		// angle restrictions
			gb.GetFloatArray(arr, length, false);
			if(length == 10) {
				angle1limits[0] = arr[0];
				angle1limits[1] = arr[1];
				angle2limits[0] = arr[2];
				angle2limits[1] = arr[3];
				angle3limits[0] = arr[4];
				angle3limits[1] = arr[5];
				angle4limits[0] = arr[6];
				angle4limits[1] = arr[7];
				angle5limits[0] = arr[8];
				angle5limits[1] = arr[9];
			}
		}
		else {		// set A defaults
			angle1limits[0] = -45.0;
			angle1limits[1] = 45.0;
			angle2limits[0] = 0.0;
			angle2limits[1] = 75.0;
			angle3limits[0] = -5.0;
			angle3limits[1] = -75.0;
			angle4limits[0] = -170.0;
			angle4limits[1] = 170.0;
			angle5limits[0] = -135.0;
			angle5limits[1] = 135.0;
		}

		if (gb.Seen('S')) {
			gb.TryGetFValue('S', val, seenNonGeometry);
			if(seenNonGeometry) {
				segmentsPerSecond = arr[0];
			}
			else {
				segmentsPerSecond = DefaultSegmentsPerSecond;
			}
		}

		if (gb.Seen('T')) {
			gb.TryGetFValue('T', val, seenNonGeometry);
			if(seenNonGeometry) {
				minSegmentLength = arr[0];
			}
			else {
				minSegmentLength = DefaultMinSegmentSize;
			}
		}

		if (gb.Seen('D')) {
			gb.TryGetIValue('D', valInt, seen);
			if(seen) {
				optimizeCode(valInt);
			}
		}

		if (gb.Seen('B')) {
			gb.GetFloatArray(arr, length, false);
			if(length == 3) {
				arm2bendingFactor = arr[0];
				arm3bendingFactor = arr[1];
				arm5bendingFactor = arr[2];
			}
		}

		if (seen)
		{
			Recalc();
		}
		else if (!seenNonGeometry && !gb.Seen('K'))
		{
			//TODO print all the parameters here
			reply.printf("Kinematics is FiveAxisRobot, documented in https://duet3d.dozuki.com/Wiki/Configuring_RepRapFirmware_for_a_FiveAxisRobot?revisionid=HEAD");
		}

		return seen;
	}
	else
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
}

bool FiveAxisRobotKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept {
	return false;
}

void FiveAxisRobotKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept {

}

bool FiveAxisRobotKinematics::DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) noexcept {
	return false;
}

void FiveAxisRobotKinematics::SetCalibrationDefaults() noexcept {

}

#if HAS_MASS_STORAGE
	bool FiveAxisRobotKinematics::WriteCalibrationParameters(FileStore *f) const noexcept {
		return true;
	}
#endif

float FiveAxisRobotKinematics::GetTiltCorrection(size_t axis) const noexcept {
	return 0.0;
}

bool FiveAxisRobotKinematics::IsReachable(float x, float y, bool isCoordinated) const noexcept {
	return true;
}

LimitPositionResult FiveAxisRobotKinematics::LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept {
	return LimitPositionResult::ok;
}

void FiveAxisRobotKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept {

}
AxesBitmap FiveAxisRobotKinematics::AxesToHomeBeforeProbing() const noexcept {
	//return AxesBitmap::MakeFromBits(Z_AXIS);
	return AxesBitmap::MakeLowestNBits(5);

}

MotionType FiveAxisRobotKinematics::GetMotionType(size_t axis) const noexcept {
	return MotionType::linear;
}

AxesBitmap FiveAxisRobotKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept {
	return g92Axes;
}

AxesBitmap FiveAxisRobotKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept {
	return axesMoving;
}

AxesBitmap FiveAxisRobotKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept {
	return Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
}

bool FiveAxisRobotKinematics::QueryTerminateHomingMove(size_t axis) const noexcept {
	return false;
}

void FiveAxisRobotKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept {
	switch(axis)  {
	case 0:
		dda.SetDriveCoordinate(0, axis);
		break;
	case 1:
		dda.SetDriveCoordinate(0, axis);
		break;
	case 2:
		dda.SetDriveCoordinate(0, axis);
		break;
	case 3:
		dda.SetDriveCoordinate(0, axis);
		break;
	case 4:
		dda.SetDriveCoordinate(0, axis);
		break;
	}
}

#if HAS_MASS_STORAGE
bool FiveAxisRobotKinematics::WriteResumeSettings(FileStore *f) const noexcept {
	return true;
}
#endif

void FiveAxisRobotKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept {

}

AxesBitmap FiveAxisRobotKinematics::GetLinearAxes() const noexcept {
	AxesBitmap bm = AxesBitmap::MakeFromBits(Z_AXIS);
	bm.Clear();
	return bm;
}

////////////////////////// private functions //////////////////////////////

void FiveAxisRobotKinematics::optimizeCode(int32_t valInt) {
	// todo implement
}

void FiveAxisRobotKinematics::Recalc() noexcept {
	// todo implement

}

