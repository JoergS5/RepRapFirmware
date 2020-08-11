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
#include <cmath>

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
	// 0. kinematics members angles
	{ "angle1limitsMax",	OBJECT_MODEL_FUNC(self->angle1limits[1], 3), ObjectModelEntryFlags::none },
	{ "angle1limitsMin",	OBJECT_MODEL_FUNC(self->angle1limits[0], 3), ObjectModelEntryFlags::none },
	{ "angle2limitsMax",	OBJECT_MODEL_FUNC(self->angle2limits[1], 3), ObjectModelEntryFlags::none },
	{ "angle2limitsMin",	OBJECT_MODEL_FUNC(self->angle2limits[0], 3), ObjectModelEntryFlags::none },
	{ "angle3limitsMax",	OBJECT_MODEL_FUNC(self->angle3limits[1], 3), ObjectModelEntryFlags::none },
	{ "angle3limitsMin",	OBJECT_MODEL_FUNC(self->angle3limits[0], 3), ObjectModelEntryFlags::none },
	{ "angle4limitsMax",	OBJECT_MODEL_FUNC(self->angle4limits[1], 3), ObjectModelEntryFlags::none },
	{ "angle4limitsMin",	OBJECT_MODEL_FUNC(self->angle4limits[0], 3), ObjectModelEntryFlags::none },
	{ "angle5limitsMax",	OBJECT_MODEL_FUNC(self->angle5limits[1], 3), ObjectModelEntryFlags::none },
	{ "angle5limitsMin",	OBJECT_MODEL_FUNC(self->angle5limits[0], 3), ObjectModelEntryFlags::none },
	{ "name", 				OBJECT_MODEL_FUNC(self->GetName(true)), ObjectModelEntryFlags::none },

	// 1. kinematics members lengths
	{ "arm2length",	OBJECT_MODEL_FUNC(self->arm2length, 3), ObjectModelEntryFlags::none },
	{ "arm3length",	OBJECT_MODEL_FUNC(self->arm3length, 3), ObjectModelEntryFlags::none },
	{ "arm4length",	OBJECT_MODEL_FUNC(self->arm4length, 3), ObjectModelEntryFlags::none },
	{ "arm5length",	OBJECT_MODEL_FUNC(self->arm5length, 3), ObjectModelEntryFlags::none },

	// 2. kinematics members special
	{ "arm2bendingFactor",	OBJECT_MODEL_FUNC(self->arm2bendingFactor, 3), ObjectModelEntryFlags::none },
	{ "arm3bendingFactor",	OBJECT_MODEL_FUNC(self->arm3bendingFactor, 3), ObjectModelEntryFlags::none },
	{ "arm5bendingFactor",	OBJECT_MODEL_FUNC(self->arm5bendingFactor, 3), ObjectModelEntryFlags::none },
	{ "currentP", 				OBJECT_MODEL_FUNC(self->currentPstrategy), ObjectModelEntryFlags::none },

	// 3. kinematics members axis
	{ "axis1coordsX",	OBJECT_MODEL_FUNC(self->axis1coords[0], 3), ObjectModelEntryFlags::none },
	{ "axis1coordsY",	OBJECT_MODEL_FUNC(self->axis1coords[1], 3), ObjectModelEntryFlags::none },
	{ "axis2coordsX",	OBJECT_MODEL_FUNC(self->axis2coords[0], 3), ObjectModelEntryFlags::none },
	{ "axis2coordsY",	OBJECT_MODEL_FUNC(self->axis2coords[1], 3), ObjectModelEntryFlags::none },
	{ "axis2coordsZ",	OBJECT_MODEL_FUNC(self->axis2coords[2], 3), ObjectModelEntryFlags::none },
};


// number of groups, number of entries for each group:
constexpr uint8_t FiveAxisRobotKinematics::objectModelTableDescriptor[] = { 4, 11, 4, 4, 5 };

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
				if(axis1coords[1] == axis2coords[1]) {
					axis2yis0 = true;
				}
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
			}
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

	 float x = machinePos[0];
	 float y = machinePos[1];
	 float z = machinePos[2];

	 float angle1;
	 float angles234[3];
	 float angle5;

	 float ax2Inv[3];
	 float ax3Inv[3];
	 float ax4Inv[3];
	 float ax5Inv[3];

	 if(currentPstrategy == 2) { // axis5 parallel to x
		 ax5Inv[0] = x - arm5length;
		 ax5Inv[1] = y;
		 ax5Inv[2] = z;
		 angle1 = getAngle1(ax5Inv[0], ax5Inv[1], ax5Inv[2]);
		 getAxis2Coords(angle1, ax2Inv);
		 getAxis4Coords(ax5Inv, ax4Inv);
		 getAxis3Coords(angle1, ax2Inv, ax4Inv, ax3Inv, angles234);
		 angle5 = - angle1;
	 }
	 else if(currentPstrategy == 3) { // axis5 parallel to y
		 ax5Inv[0] = x;
		 ax5Inv[1] = y - arm5length;
		 ax5Inv[2] = z;
		 angle1 = getAngle1(ax5Inv[0], ax5Inv[1], ax5Inv[2]);
		 getAxis2Coords(angle1, ax2Inv);
		 getAxis4Coords(ax5Inv, ax4Inv);
		 getAxis3Coords(angle1, ax2Inv, ax4Inv, ax3Inv, angles234);
		 angle5 = - angle1 + 90.0;
	 }
	 else if(currentPstrategy == 4) {
		 angle1 = getAngle1(x, y, z);
		 getAxis2Coords(angle1, ax2Inv);
		 getAxis5Coords(x, y, z, angle1, ax5Inv);
		 getAxis4Coords(ax5Inv, ax4Inv);
		 getAxis3Coords(angle1, ax2Inv, ax4Inv, ax3Inv, angles234);
		 angle5 = 0;
	 }
	 else {
		 angle1 = -999;
		 angle5 = -999;
		 // todo report error
	 }

	 motorPos[0] = int32_t(angle1 * stepsPerMm[0]);
	 motorPos[1] = int32_t(angles234[0] * stepsPerMm[1]);
	 motorPos[2] = int32_t(angles234[1] * stepsPerMm[2]);
	 motorPos[3] = int32_t(angles234[2] * stepsPerMm[3]);
	 motorPos[4] = int32_t(angle5 * stepsPerMm[4]);

	 return true;
}

void FiveAxisRobotKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept {
	 float angle1 = (float) motorPos[0] / stepsPerMm[0];
	 float angle2 = (float) motorPos[1] / stepsPerMm[1];
	 float angle3 = (float) motorPos[2] / stepsPerMm[2];
	 float angle4 = (float) motorPos[3] / stepsPerMm[3];
	 float angle5 = (float) motorPos[4] / stepsPerMm[4];

	 float x = axis2coords[0];
	 float y = axis2coords[1];
	 float z = axis2coords[2];

	 // position axis 3
	 float l2angledX = cos(angle2/360.0*2.0*Pi) * arm2length;
	 x += l2angledX;
	 float l2angledZ = sin(angle2/360.0*2.0*Pi) * arm2length;
	 z += l2angledZ;

	 // position axis 4
	 float l3angled = cos((angle2+angle3)/360.0*2.0*Pi) * arm3length;
	 x += l3angled;
	 float l3angledZ = sin((angle2+angle3)/360.0*2.0*Pi) * arm3length;
	 z += l3angledZ;

	 // position axis 5
	 float l4angled = cos((angle2+angle3+angle4)/360.0*2.0*Pi) * arm4length;
	 x += l4angled;
	 float l4angledZ = sin((angle2+angle3+angle4)/360.0*2.0*Pi) * arm4length;
	 z += l4angledZ;

	 // position nozzle
	 float l5angled = cos(angle5/360.0*2.0*Pi) * arm5length;
	 x += l5angled;
	 float l5angledY = sin(angle5/360.0*2.0*Pi) * arm5length;
	 y += l5angledY;

	 // rotate axis 1
	 // set rotation orig:
	 float origX = x - axis1coords[0];
	 float origY = y - axis1coords[1];

	 float sinangle = sin(angle1/360.0*2.0*Pi);
	 float cosangle = cos(angle1/360.0*2.0*Pi);
	 x = origX * cosangle - origY * sinangle;
	 y = origX * sinangle + origY * cosangle;

	 // correct back rotation orig
	 x += axis1coords[0];
	 y += axis1coords[1];

	 machinePos[0] = x;
	 machinePos[1] = y;
	 machinePos[2] = z;
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
	const Platform& platform = reprap.GetPlatform();
	bool reachable = true;

	// test x
	float xMin = platform.AxisMinimum(0) - AxisRoundingError;
	float xMax = platform.AxisMaximum(0) + AxisRoundingError;
	if(x < xMin || x > xMax) {
		reachable = false;
	}

	//test y
	float yMin = platform.AxisMinimum(1) - AxisRoundingError;
	float yMax = platform.AxisMaximum(1) + AxisRoundingError;
	if(y < yMin || y > yMax) {
		reachable = false;
	}
	return reachable;
}

LimitPositionResult FiveAxisRobotKinematics::LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept {

	// First limit all axes according to M208
	const bool m208Limited = applyM208Limits && Kinematics::LimitPositionFromAxis(finalCoords, 0, numVisibleAxes, axesHomed);

	return (m208Limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}

void FiveAxisRobotKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept {

}
AxesBitmap FiveAxisRobotKinematics::AxesToHomeBeforeProbing() const noexcept {
	if(useRail) {
		return AxesBitmap::MakeLowestNBits(6);
	}
	else {
		return AxesBitmap::MakeLowestNBits(5);
	}
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
	case 5:
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
	// not implemented yet
}

void FiveAxisRobotKinematics::Recalc() noexcept {
	// no cached values yet
}

bool FiveAxisRobotKinematics::getIntersectionUpper(float x1, float y1, float rad1, float x2, float y2,
		 	float rad2, float tangent[]) const {

	bool error = false;

	 // only one intersection point (rad1 + rad2 == distance)?
	 float distance = sqrt(fsquare(x1-x2) + fsquare(y1-y2));
	 float radsum = rad1 + rad2;

	 if(fabs(distance - radsum) < 0.00001f) {   // float tolerance
		 float fract = rad1 / (rad1 + rad2);
		 float diffx = fabs(x1-x2);
		 float diffy = fabs(y1-y2);
		 float xIntersec = x1 + fract * diffx;
		 float yIntersec = y1 + fract * diffy;

		 tangent[0] = xIntersec;
		 tangent[1] = yIntersec;
	 }
	 else if(distance > radsum) { // no intersection
		 error = true;
	 }
	 else {
		 float result[4];
		 getIntersec(result, rad1, rad2, x1, y1, x2, y2);
		 if(result[1] >= result[3]) {
			 tangent[0] = result[0];
			 tangent[1] = result[1];
		 }
		 else {
			 tangent[0] = result[2];
			 tangent[1] = result[3];
		 }
	 }
	 return error;
}

// first circle, second circle. Return the two intersection points
void FiveAxisRobotKinematics::getIntersec(float result[], float firstRadius, float secondRadius, float firstX, float firstY, float secondX, float secondY) const noexcept
{
	const float firstRadius2  = fsquare(firstRadius);
	const float secondRadius2 = fsquare(secondRadius);

	const float distance2 = fsquare(firstX - secondX) + fsquare(firstY - secondY);
	const float distance  = sqrtf(distance2);

	const float delta = 0.25 * sqrtf(
			(distance + firstRadius + secondRadius)
			* (distance + firstRadius - secondRadius)
			* (distance - firstRadius + secondRadius)
			* (-distance + firstRadius + secondRadius)
		);

	// calculate x
	const float term1x = (firstX + secondX) / 2;
	const float term2x = (secondX - firstX) * (firstRadius2 - secondRadius2) / (2 * distance2);
	const float term3x = 2 * (firstY - secondY) / (distance2) * delta;
	const float x1 = term1x + term2x + term3x;
	const float x2 = term1x + term2x - term3x;

	// calculate y
	const float term1y = (firstY + secondY) / 2;
	const float term2y = (secondY - firstY)*(firstRadius2 - secondRadius2) / (2 * distance2);
	const float term3y = 2 * (firstX - secondX) / distance2 * delta;
	const float y1 = term1y + term2y - term3y;
	const float y2 = term1y + term2y + term3y;

	result[0] = x1;
	result[1] = y1;
	result[2] = x2;
	result[3] = y2;
}

// get angle1 for axis1
// called by inverse kinematics
// find tangent https://silo.tips/download/geometrie-dossier-kreis-2 section Grundkonstruktion 2
// xyz is the nozzle endpoint (P4) or the axis5 coordinate (P2, P3)
float FiveAxisRobotKinematics::getAngle1(float x, float y, float z) const {
	 float angle1;	// angle of axis 1 CCW

	 if(axis2yis0) {		// arm 2 through axis 1
		 float xdiff = x - axis1coords[0];
		 float ydiff = y - axis1coords[1];
		 angle1 = atan(ydiff/xdiff) * 180.0 / Pi;
	 }
	 else {
		 if(y == axis2coords[1]) {
			 angle1 = 0.0;
		 }
		 else {
			 float tangent[2];
			 float xmid = (x + axis1coords[0]) / 2.0;
			 float ymid = (y + axis1coords[1]) / 2.0;
			 float radiusMid = sqrt(fsquare(x-axis1coords[0]) + fsquare(y-axis1coords[1])) / 2.0;
			 if(getIntersectionUpper(axis1coords[0], axis1coords[1], (axis2coords[1]-axis1coords[1]),
					 xmid, ymid, radiusMid, tangent)) {
				 float xdiff = x - tangent[0];
				 float ydiff = y - tangent[1];
				 angle1 = atan(ydiff/xdiff) * 180.0 / Pi;
			 }
			 else {
				 // todo report error (how?)
				 angle1 = 0.0;
			 }
		 }
	 }
	 return angle1;
}

void FiveAxisRobotKinematics::getAxis4Coords(const float axis5c[], float axis4c[]) const {
	 axis4c[0] = axis5c[0];
	 axis4c[1] = axis5c[1];
	 axis4c[2] = axis5c[2] + arm4length;	// axis4 is always vertical above axis5
}

// is only called with currentPstrategy == 4
void FiveAxisRobotKinematics::getAxis5Coords(float x, float y, float z, float angle1, float axis5c[]) const {
	 float sinangle = sin(angle1/360.0*2.0*Pi);
	 float cosangle = cos(angle1/360.0*2.0*Pi);

	 float ydiff = sinangle * arm5length;
	 float xdiff = cosangle * arm5length;

	 axis5c[0] = x - xdiff;
	 axis5c[1] = y - ydiff;
	 axis5c[2] = z;
}

void FiveAxisRobotKinematics::getAxis2Coords(float angle1, float axis2c[]) const {
	 float sinangle = sin(angle1/360.0*2.0*Pi);
	 float cosangle = cos(angle1/360.0*2.0*Pi);
	 float x2rotateby0 = axis2coords[0] - axis1coords[0];
	 float y2rotateby0 = axis2coords[1] - axis1coords[1];
	 float x2new = x2rotateby0 * cosangle - y2rotateby0 * sinangle;
	 float y2new = x2rotateby0 * sinangle + y2rotateby0 * cosangle;

	 axis2c[0] = x2new + axis1coords[0];
	 axis2c[1] = y2new + axis1coords[1];
	 axis2c[2] = axis2coords[2];
}

void FiveAxisRobotKinematics::getAxis3Coords(float angle1, const float axis2c[], const float axis4c[], float axis3c[],
				float angles234[]) const {
	 // first get distance between axis 2 and axis4, three dimensional
	 float dist2to4 = sqrt(fsquare(axis2c[0]-axis4c[0]) + fsquare(axis2c[1]-axis4c[1])
			 + fsquare(axis2c[2]-axis4c[2]));

	 // get triangle of arm2 and arm3 above the line
	 float temp3[2];
	 getIntersectionUpper(0, 0, arm2length, dist2to4, 0, arm3length, temp3);
	 // angle between horizontal line and axis3
	 float angle2uncorrected = asin(temp3[1]/arm2length) * 180.0 / Pi;

	 // get z-angle between axis 2 and axis 4
	 float zdiff2to4 = axis2c[2] - axis4c[2];
	 float distxy2to4 = sqrt(fsquare(axis2c[0]-axis4c[0]) + fsquare(axis2c[1]-axis4c[1]));
	 float zangle2to4 = - atan(zdiff2to4/distxy2to4) * 180.0 / Pi;

	 // angle2
	 float angle2 =  angle2uncorrected + zangle2to4;
	 angles234[0] = angle2;

	 // vector addition, get axis3
	 float x3 = cos(angle2/180.0*Pi) * cos(angle1/180.0*Pi) * arm2length + axis2c[0];
	 float y3 = cos(angle2/180.0*Pi) * sin(angle1/180.0*Pi) * arm2length + axis2c[1];
	 float z3 = sin(angle2/180.0*Pi) * arm2length + axis2c[2];

	 axis3c[0] = x3;
	 axis3c[1] = y3;
	 axis3c[2] = z3;

	 // angle from axis3 to axis4 (measured beginning from axis4)
	 float zdiff3to4 = axis4c[2] - axis3c[2];
	 float angle3 = asin(zdiff3to4/arm3length) * 180.0 / Pi - angle2;
	 angles234[1] = angle3;

	 // get angle 4
	 float angle4 = -90 - angle2 - angle3;
	 angles234[2] = angle4;
}

