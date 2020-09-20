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

#define ROUND_2_INT(f) ((int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5)))
#define FLOATERROR 0.001

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
	{ "arm4bendingFactor",	OBJECT_MODEL_FUNC(self->arm4bendingFactor, 3), ObjectModelEntryFlags::none },
	{ "arm4vertical",	OBJECT_MODEL_FUNC(self->arm4vertical, 3), ObjectModelEntryFlags::none },
	{ "arm5bendingFactor",	OBJECT_MODEL_FUNC(self->arm5bendingFactor, 3), ObjectModelEntryFlags::none },
	{ "armOrientation", 				OBJECT_MODEL_FUNC(self->armOrientation), ObjectModelEntryFlags::none },
	{ "p2Angle", 				OBJECT_MODEL_FUNC(self->p2Angle), ObjectModelEntryFlags::none },
	{ "pMode", 				OBJECT_MODEL_FUNC(self->pMode), ObjectModelEntryFlags::none },
	{ "rMode", 				OBJECT_MODEL_FUNC(self->rMode), ObjectModelEntryFlags::none },

	// 3. kinematics members axis
	{ "axis1coordsX",	OBJECT_MODEL_FUNC(self->axis1coords[0], 3), ObjectModelEntryFlags::none },
	{ "axis1coordsY",	OBJECT_MODEL_FUNC(self->axis1coords[1], 3), ObjectModelEntryFlags::none },
	{ "axis2coordsX",	OBJECT_MODEL_FUNC(self->axis2coords[0], 3), ObjectModelEntryFlags::none },
	{ "axis2coordsY",	OBJECT_MODEL_FUNC(self->axis2coords[1], 3), ObjectModelEntryFlags::none },
	{ "axis2coordsZ",	OBJECT_MODEL_FUNC(self->axis2coords[2], 3), ObjectModelEntryFlags::none },
	{ "axis5offsetX",	OBJECT_MODEL_FUNC(self->axis5offset[0], 3), ObjectModelEntryFlags::none },
	{ "axis5offsetY",	OBJECT_MODEL_FUNC(self->axis5offset[1], 3), ObjectModelEntryFlags::none },

	// 4. rail parameters
	{ "railMode", 				OBJECT_MODEL_FUNC(self->railMode), ObjectModelEntryFlags::none },
	{ "railX", 				OBJECT_MODEL_FUNC(self->railX), ObjectModelEntryFlags::none },
	{ "railY", 				OBJECT_MODEL_FUNC(self->railY), ObjectModelEntryFlags::none },
	{ "railZ", 				OBJECT_MODEL_FUNC(self->railZ), ObjectModelEntryFlags::none },
	{ "useRail", 				OBJECT_MODEL_FUNC(self->useRail), ObjectModelEntryFlags::none },
};


// number of groups, number of entries for each group:
constexpr uint8_t FiveAxisRobotKinematics::objectModelTableDescriptor[] = { 5, 11, 4, 9, 7, 5 };

DEFINE_GET_OBJECT_MODEL_TABLE(FiveAxisRobotKinematics)

#endif

FiveAxisRobotKinematics::FiveAxisRobotKinematics() noexcept
	: Kinematics(KinematicsType::robot5axis, DefaultSegmentsPerSecond, DefaultMinSegmentSize, true)
{
	Recalc();
}

///////////////////////////// public functions /////////////////////////

bool FiveAxisRobotKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept {

 	 float x = machinePos[0];
 	 float y = machinePos[1];
 	 float z = machinePos[2];

 	 float angles[5];

 	 float ax2Inv[3];
 	 float ax3Inv[3];
 	 float ax4Inv[3];
 	 float ax5Inv[3];

 	 if(pMode == 0) {
 		 angles[0] = getAngle1(x, y, z);
 		 getAxis2Coords(angles[0], ax2Inv);
 		 getAxis5Coords(x, y, z, angles[0], ax5Inv);
 		 getAxis4Coords(ax5Inv, ax4Inv, angles[0]);
 		 getAxis3Coords(angles[0], ax2Inv, ax4Inv, ax3Inv, angles);
 		angles[4] = 0;
 	 }
 	 else if(pMode == 2) { // axis5 p2Angle degree in respect to x axis
 		ax5Inv[0] = x - cos(p2Angle/180.0*Pi) * arm5length;
 		ax5Inv[1] = y - sin(p2Angle/180.0*Pi) * arm5length;
 		ax5Inv[2] = z;

 		angles[0] = getAngle1(ax5Inv[0], ax5Inv[1], ax5Inv[2]);

 		 getAxis2Coords(angles[0], ax2Inv);
 		 getAxis4Coords(ax5Inv, ax4Inv, angles[0]);
 		 getAxis3Coords(angles[0], ax2Inv, ax4Inv, ax3Inv, angles);
 		angles[4] = - angles[0] + p2Angle;
 	 }
 	  else {
 		  // todo report error
 		  return false;
 	 }

 	 if(!constraintsOk(angles)) {
 		 return false;
 	 }

 	 if(rMode == 0) {			// all 5 axis have actuators
 	 	 motorPos[0] = ROUND_2_INT(angles[0] * stepsPerMm[0]);
 	 	 motorPos[1] = ROUND_2_INT(angles[1] * stepsPerMm[1]);
 	 	 motorPos[2] = ROUND_2_INT(angles[2] * stepsPerMm[2]);
 	 	 motorPos[3] = ROUND_2_INT(angles[3] * stepsPerMm[3]);
 	 	 motorPos[4] = ROUND_2_INT(angles[4] * stepsPerMm[4]);
 	 }
 	 else if(rMode == 1) {		// no 4th actuator
 	 	 motorPos[0] = ROUND_2_INT(angles[0] * stepsPerMm[0]);
 	 	 motorPos[1] = ROUND_2_INT(angles[1] * stepsPerMm[1]);
 	 	 motorPos[2] = ROUND_2_INT(angles[2] * stepsPerMm[2]);
 	 	 motorPos[3] = ROUND_2_INT(angles[4] * stepsPerMm[4]);
 	 }
 	 else if(rMode == 2) {		// no 4th and 5th actuator
 	 	 motorPos[0] = ROUND_2_INT(angles[0] * stepsPerMm[0]);
 	 	 motorPos[1] = ROUND_2_INT(angles[1] * stepsPerMm[1]);
 	 	 motorPos[2] = ROUND_2_INT(angles[2] * stepsPerMm[2]);
 	 }

 	 return true;
 }


 void FiveAxisRobotKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept {
 	 float angle1 = (float) motorPos[0] / stepsPerMm[0];
 	 float angle2 = (float) motorPos[1] / stepsPerMm[1];
 	 float angle3 = (float) motorPos[2] / stepsPerMm[2];
 	 float angle4 = -90.0 - angle2 - angle3;
 	 float angle5 = 0.0;

	 if(rMode == 0) {
	 	 angle4 = (float) motorPos[3] / stepsPerMm[3];
	 	 angle5 = (float) motorPos[4] / stepsPerMm[4];
	 }
	 else if(rMode == 1) {		// angle4 is automatic
	 	 //angle4 = -90.0 - angle2 - angle3;
	 	 angle5 = (float) motorPos[3] / stepsPerMm[3];
	 }
	 else if(rMode == 2) {		// angle4 is automatic, angle5 is always 0
	 	 //angle4 = -90.0 - angle2 - angle3;
	 	 //angle5 = 0.0;
	 }

 	 float x = axis2coords[0];
 	 float y = axis2coords[1];
 	 float z = axis2coords[2];

 	 // first calculate nozzle endpoint with axis1 unrotated, then rotate by axis1

 	 // position axis 3
 	 float l2angledX = cos(angle2/180.0*Pi) * arm2length;
 	 x += l2angledX;
 	 float l2angledZ = sin(angle2/180.0*Pi) * arm2length;
 	 z += l2angledZ;

 	 // position axis 4
 	 float l3angled = cos((angle2+angle3)/180.0*Pi) * arm3length;
 	 x += l3angled;
 	 float l3angledZ = sin((angle2+angle3)/180.0*Pi) * arm3length;
 	 z += l3angledZ;

 	 // position axis 5 (endpoint arm 4)
 	 float l4angled = cos((angle2+angle3+angle4)/180.0*Pi) * arm4length;
 	 x += l4angled;
 	 float l4angledZ = sin((angle2+angle3+angle4)/180.0*Pi) * arm4length;
 	 z += l4angledZ;

 	 // axis 5 position
 	 x += axis5offset[0];
 	 y += axis5offset[1];

 	 // position nozzle
 	 float l5angled = cos(angle5/180.0*Pi) * arm5length;
 	 x += l5angled;
 	 float l5angledY = sin(angle5/180.0*Pi) * arm5length;
 	 y += l5angledY;

 	 // rotate by axis 1
 	 // set rotation orig:
 	 float origX = x - axis1coords[0];
 	 float origY = y - axis1coords[1];

 	 float sinangle = sin(angle1/180.0*Pi);
 	 float cosangle = cos(angle1/180.0*Pi);
 	 x = origX * cosangle - origY * sinangle;
 	 y = origX * sinangle + origY * cosangle;

 	 // correct back rotation orig
 	 x += axis1coords[0];
 	 y += axis1coords[1];

 	 machinePos[0] = x;
 	 machinePos[1] = y;
 	 machinePos[2] = z;
 }


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
		float val;			// return value
		int32_t valInt;
		bool seen = false;			// return value
		bool seenNonGeometry = false;

		if (gb.Seen('X')) {		// X of axis 1 and axis 2, optional axis 5 offset
			float arr[3];
			size_t length = 3;
			gb.GetFloatArray(arr, length, false);
			if(length >= 2) {
				axis1coords[0] = arr[0];
				axis2coords[0] = arr[1];
				seen = true;
				if(length == 3) {
					axis5offset[0] = arr[2];
				}
			}
		}

		if (gb.Seen('Y')) {		// Y of axis 1 and axis 2, optional axis 5 offset
			float arr[3];
			size_t length = 3;
			gb.GetFloatArray(arr, length, false);
			if(length >= 2) {
				axis1coords[1] = arr[0];
				axis2coords[1] = arr[1];
				seen = true;
				if(length == 3) {
					axis5offset[1] = arr[2];
				}
			}
		}

		if (gb.Seen('Z')) {		// Z of Axis 2
			gb.TryGetFValue('Z', val, seenNonGeometry);
			axis2coords[2] = val;
			seen = true;
		}

		if (gb.Seen('C')) {		// define rail parameters
			float arr[4];
			size_t length = 4;
			gb.GetFloatArray(arr, length, false);
			if(length == 1) {
				railMode = (int32_t) arr[0];
				useRail = false;
				railX = 0.0;
				railX = 0.0;
				railZ = 0.0;
				seen = true;
			}
			else if(length == 4) {
				railMode = (int32_t) arr[0];
				if(railMode > 0) {
					useRail = true;
					railX = arr[1];
					railY = arr[2];
					railZ = arr[3];
				}
				else if(railMode == 0) {
					useRail = false;
					railX = 0.0;
					railX = 0.0;
					railZ = 0.0;
				}
				seen = true;
			}
		}

		if (gb.Seen('P')) {		// mode for axis 5
			float arr[2];
			size_t length = 2;
			gb.GetFloatArray(arr, length, false);
			if(length == 1) {
				pMode = (int32_t) arr[0];
				arm4vertical = true;
				seen = true;
				p2Angle = 0.0; // set to clarify it's not used
			}
			else if(length == 2) {
				pMode = (int32_t) arr[0];	// todo report error if not 2
				p2Angle = arr[1];
				arm4vertical = true;
				seen = true;
			}
		}

		if (gb.Seen('R')) {			// configuration of actuators
			gb.TryGetIValue('R', valInt, seen);
			if(seen) {
				rMode = valInt;
			}
			seen = true;
		}

		if (gb.Seen('L')) {		// arm 2 to 5 lengths
			float arms[4];
			gb.TryGetFloatArray('L', 4, arms, reply, seen);
			arm2length = arms[0];
			arm3length = arms[1];
			arm4length = arms[2];
			arm5length = arms[3];
			seen = true;
		}

		if (gb.Seen('A')) {		// angle restrictions
			float limits[10];
			gb.TryGetFloatArray('A', 10, limits, reply, seen);
			angle1limits[0] = limits[0];
			angle1limits[1] = limits[1];
			angle2limits[0] = limits[2];
			angle2limits[1] = limits[3];
			angle3limits[0] = limits[4];
			angle3limits[1] = limits[5];
			angle4limits[0] = limits[6];
			angle4limits[1] = limits[7];
			angle5limits[0] = limits[8];
			angle5limits[1] = limits[9];
			seen = true;
		}

		if (gb.Seen('S')) {
			gb.TryGetFValue('S', val, seenNonGeometry);
			if(seenNonGeometry) {
				segmentsPerSecond = val;
			}
			else {
				segmentsPerSecond = DefaultSegmentsPerSecond;
			}
			seen = true;
		}

		if (gb.Seen('T')) {
			gb.TryGetFValue('T', val, seenNonGeometry);
			if(seenNonGeometry) {
				minSegmentLength = val;
			}
			else {
				minSegmentLength = DefaultMinSegmentSize;
			}
			seen = true;
		}

		if (gb.Seen('D')) {			// optimization
			gb.TryGetIValue('D', valInt, seen);
			if(seen) {
				optimizeCode(valInt);
			}
			seen = true;
		}

		if (gb.Seen('B')) {			// arm orientation and bendings
			float bending[5];
			gb.TryGetFloatArray('B', 5, bending, reply, seen);
			armOrientation = (int32_t) bending[0];
			arm2bendingFactor = bending[1];
			arm3bendingFactor = bending[2];
			arm4bendingFactor = bending[3];
			arm5bendingFactor = bending[4];
			seen = true;
		}

		if (seen)
		{
			Recalc();
		}

		reply.catf("Kinematics FiveAxisRobot");
		reply.catf(", axis1XY: %.1f/%.1f, axis2XYZ: %.1f/%.1f/%.1f",
				(double) axis1coords[0], (double) axis1coords[1],
				(double) axis2coords[0], (double) axis2coords[1], (double) axis2coords[2]);
		reply.catf(", L2/3/4/5: %.1f/%.1f/%.1f/%.1f",
				(double) arm2length, (double) arm3length, (double) arm4length, (double) arm5length);

		if(pMode == 0) {
			reply.catf(", P0 (arm 5 fixed 0 degree)");
		}
		else if(pMode == 1) {
			reply.catf(", P1 (arm 5 free rotation)");
		}
		else if(pMode == 2) {
			reply.catf(", P2 (arm 5 fixed angle: %.1f)", (double) p2Angle);
		}
		else if(pMode == 3) {
			reply.catf(", P3 (arm 5 movement direction)");
		}

		if(useRail) {
			if(rMode == 0) {
				reply.catf(", R0 (5+rail actuators XYZUVW)");
			}
			else if(rMode == 1) {
				reply.catf(", R1 (4+rail actuators XYZUV)");
			}
			else if(rMode == 2) {
				reply.catf(", R2 (3+rail actuators XYZU)");
			}
		}
		else {
			if(rMode == 0) {
				reply.catf(", R0 (5 actuators XYZUV)");
			}
			else if(rMode == 1) {
				reply.catf(", R1 (4 actuators XYZU)");
			}
			else if(rMode == 2) {
				reply.catf(", R2 (3 actuators XYZ)");
			}
		}

		if(useRail) {
			if(railMode == 1) {
				reply.catf(", C%i (rail parallel to X, XYZ: %.1f/%.1f/%.1f)",
						(int) railMode, (double) railX, (double) railY, (double) railZ);
			}
			else if(railMode == 2) {
				reply.catf(", C%i (rail parallel to Y, XYZ: %.1f/%.1f/%.1f)",
						(int) railMode, (double) railX, (double) railY, (double) railZ);
			}
			else if(railMode == 3) {
				reply.catf(", C%i (rail parallel to Z, XYZ: %.1f/%.1f/%.1f)",
						(int) railMode, (double) railX, (double) railY, (double) railZ);
			}
		}
		else {
			reply.catf(", C0 (no rail)");
		}

		reply.catf(", A%.1f/%.1f/%.1f/%.1f/%.1f/%.1f",
				(double) angle1limits[0], (double) angle1limits[1],
				(double) angle2limits[0], (double) angle2limits[1],
				(double) angle3limits[0], (double) angle3limits[1]
				);
		reply.catf("/%.1f/%.1f/%.1f/%.1f",
				(double) angle4limits[0], (double) angle4limits[1],
				(double) angle5limits[0], (double) angle5limits[1]
				);

		return seen;
	}
	else
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
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

	// todo should angle limits be considered here also? Then inverse kinematics must be calculated here first,
	// calling constraintsOk() after. But not all parameters of CartesianToMotorSteps are available here (stepsPerMm eg).

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

void FiveAxisRobotKinematics::setPlannnedPath(float plpath[]) noexcept {
	for(int i=0; i < 6; i++) {
		plannedPath[i] = plpath[i];
	}
}

////////////////////////// private functions //////////////////////////////

void FiveAxisRobotKinematics::optimizeCode(int32_t valInt) {
	// not implemented yet
}

void FiveAxisRobotKinematics::Recalc() noexcept {
	// no cached values yet
}

bool FiveAxisRobotKinematics::getIntersectionUpper(float x1, float y1, float rad1, float x2, float y2,
		 	float rad2, float tangent[], bool upper) const {

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
		 if(upper) {
			 if(result[1] >= result[3]) {
				 tangent[0] = result[0];
				 tangent[1] = result[1];
			 }
			 else {
				 tangent[0] = result[2];
				 tangent[1] = result[3];
			 }
		 }
		 else {
			 if(result[1] < result[3]) {
				 tangent[0] = result[0];
				 tangent[1] = result[1];
			 }
			 else {
				 tangent[0] = result[2];
				 tangent[1] = result[3];
			 }
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

bool FiveAxisRobotKinematics::constraintsOk(const float angles[]) const noexcept {
 	if(angles[0] < angle1limits[0] - FLOATERROR || angles[0] > angle1limits[1] + FLOATERROR) {
 		return false;
 	}
 	if(angles[1] < angle2limits[0] - FLOATERROR || angles[1] > angle2limits[1] + FLOATERROR) {
 		return false;
 	}
 	if(angles[2] < angle3limits[0] - FLOATERROR || angles[2] > angle3limits[1]+ FLOATERROR) {
		return false;
 	}
 	if(angles[3] < angle4limits[0] - FLOATERROR || angles[3] > angle4limits[1] + FLOATERROR) {
		return false;
 	}
 	if(angles[4] < angle5limits[0] - FLOATERROR || angles[4] > angle5limits[1] + FLOATERROR) {
		return false;
 	}
 	return true;
 }

// more exact method name would be: getStartpointArm4
void FiveAxisRobotKinematics::getAxis4Coords(const float axis5c[], float axis4c[], float angle1) const {
	// todo axis5offset correction is only correct if arm 4 is vertical

	 // rotate axis5offset:
 	 float sinangle = sin(angle1/180.0*Pi);
 	 float cosangle = cos(angle1/180.0*Pi);
 	 float ax5x = axis5offset[0] * cosangle - axis5offset[1] * sinangle;
 	 float ax5y = axis5offset[0] * sinangle + axis5offset[1] * cosangle;

	 axis4c[0] = axis5c[0] - ax5x;
	 axis4c[1] = axis5c[1] - ax5y;

	 axis4c[2] = axis5c[2] + arm4length;	// axis4 is always vertical above axis5
}

void FiveAxisRobotKinematics::getAxis5Coords(float x, float y, float z, float angle1, float axis5c[]) const {
	 float sinangle = sin(angle1/180.0*Pi);
	 float cosangle = cos(angle1/180.0*Pi);

	 float ydiff = sinangle * arm5length;
	 float xdiff = cosangle * arm5length;

	 axis5c[0] = x - xdiff;
	 axis5c[1] = y - ydiff;
	 axis5c[2] = z;
}

void FiveAxisRobotKinematics::getAxis2Coords(float angle1, float axis2c[]) const {
	 float sinangle = sin(angle1/180.0*Pi);
	 float cosangle = cos(angle1/180.0*Pi);
	 float x2rotateby0 = axis2coords[0] - axis1coords[0];
	 float y2rotateby0 = axis2coords[1] - axis1coords[1];
	 float x2new = x2rotateby0 * cosangle - y2rotateby0 * sinangle;
	 float y2new = x2rotateby0 * sinangle + y2rotateby0 * cosangle;

	 axis2c[0] = x2new + axis1coords[0];
	 axis2c[1] = y2new + axis1coords[1];
	 axis2c[2] = axis2coords[2];
}

void FiveAxisRobotKinematics::getAxis3Coords(float angle1, const float axis2c[], const float axis4c[], float axis3c[],
				float angles[]) const {
	 // first get distance between axis 2 and axis4, three dimensional
	 float dist2to4 = sqrt(fsquare(axis2c[0]-axis4c[0]) + fsquare(axis2c[1]-axis4c[1])
			 + fsquare(axis2c[2]-axis4c[2]));

	 // get triangle of arm2 and arm3 above the line
	 float temp3[2];
	 getIntersectionUpper(0, 0, arm2length, dist2to4, 0, arm3length, temp3, true);
	 // angle between horizontal line and axis3
	 float angle2uncorrected = asin(temp3[1]/arm2length) * 180.0 / Pi;

	 // get z-angle between axis 2 and axis 4
	 float zdiff2to4 = axis2c[2] - axis4c[2];
	 float distxy2to4 = sqrt(fsquare(axis2c[0]-axis4c[0]) + fsquare(axis2c[1]-axis4c[1]));
	 float zangle2to4 = - atan(zdiff2to4/distxy2to4) * 180.0 / Pi;

	 // angle2
	 float angle2 =  angle2uncorrected + zangle2to4;
	 angles[1] = angle2;

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
	 angles[2] = angle3;

	 // get angle 4
	 float angle4 = -90 - angle2 - angle3;
	 angles[3] = angle4;
}

// find tangent https://silo.tips/download/geometrie-dossier-kreis-2 section Grundkonstruktion 2
// xyz are the endpoints (P4) or the axis5 coordinates (P2, P3)
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
			 float ax2corr[2];
			 ax2corr[0] = axis2coords[0] + axis5offset[0];
			 ax2corr[1] = axis2coords[1] + axis5offset[1];
			 float tangent[2];
			 float xmid = (x + axis1coords[0]) / 2.0;
			 float ymid = (y + axis1coords[1]) / 2.0;
			 float radiusMid = sqrt(fsquare(x-axis1coords[0]) + fsquare(y-axis1coords[1])) / 2.0;
			 if(axis1coords[1] < ax2corr[1]) {
				 getIntersectionUpper(axis1coords[0], axis1coords[1], (ax2corr[1]-axis1coords[1]), xmid, ymid,
						 radiusMid, tangent, true);
			 }
			 else {
				 getIntersectionUpper(axis1coords[0], axis1coords[1], (-ax2corr[1]+axis1coords[1]), xmid, ymid,
						 radiusMid, tangent, false);
			 }
			 float xdiff = x - tangent[0];
			 float ydiff = y - tangent[1];
			 angle1 = atan(ydiff/xdiff) * 180.0 / Pi;
		 }
	 }
	 return angle1;
}
