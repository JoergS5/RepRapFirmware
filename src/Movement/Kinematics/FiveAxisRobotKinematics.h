/*
 * FiveAxisRobotKinematics.h
 *
 *  Created on: 09 Aug 2020
 *      Author: JoergS5
 *
 *	documentation: https://duet3d.dozuki.com/Wiki/Configuring_RepRapFirmware_for_a_FiveAxisRobot?revisionid=HEAD
 */

#ifndef SRC_MOVEMENT_KINEMATICS_FIVEAXISROBOTKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_FIVEAXISROBOTKINEMATICS_H_

#include "RepRapFirmware.h"
#include "Kinematics.h"

class FiveAxisRobotKinematics : public Kinematics
{
public:
	// Constructors
	FiveAxisRobotKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;

	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool SupportsAutoCalibration() const noexcept override { return true; }
	bool DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) noexcept override;
	void SetCalibrationDefaults() noexcept override;

#if HAS_MASS_STORAGE
	bool WriteCalibrationParameters(FileStore *f) const noexcept override;
#endif

	float GetTiltCorrection(size_t axis) const noexcept override;
	bool IsReachable(float x, float y, bool isCoordinated) const noexcept override;
	LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	AxesBitmap AxesToHomeBeforeProbing() const noexcept override;
	MotionType GetMotionType(size_t axis) const noexcept override;
	size_t NumHomingButtons(size_t numVisibleAxes) const noexcept override { return 0; }
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;

#if HAS_MASS_STORAGE
	bool WriteResumeSettings(FileStore *f) const noexcept override;
#endif

	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;
	AxesBitmap GetLinearAxes() const noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:
	static constexpr const char *Home5AxisRobotFileName = "home5axisrobot.g";
	static constexpr float DefaultSegmentsPerSecond = 100.0;
	static constexpr float DefaultMinSegmentSize = 0.2;

	void Recalc() noexcept;
	void optimizeCode(int32_t valInt);
	void getIntersec(float result12[], float firstRadius, float secondRadius, float firstX, float firstY, float secondX, float secondY) const noexcept;
	bool getIntersectionUpper(float _axis1x, float _axis1y, float _axis2y, float xmid, float ymid, float radiusMid, float tangent[]) const;
	float getAngle1(float x, float y, float z) const;
	void getAxis2Coords(float angle1, float axis2coords[]) const;
	void getAxis3Coords(float angle1, const float axis2coords[], const float axis4coords[], float axis3coords[],
			float angles234[]) const;
	void getAxis4Coords(const float axis5coords[], float axis4coords[]) const;
	void getAxis5Coords(float x, float y, float z, float angle1, float axis5coords[]) const;

	// Primary parameters
	float axis1coords[2];		// XY
	float axis2coords[3];		// XYZ
	float axis6coords[1];		// Y rail 6th axis
	bool axis2yis0 = false;		// if true, arm 2 goes through axis 1
	bool useRail = false;				// if rail is used
	int32_t currentPstrategy = 4;	// current P setting, 4 is the default
	int32_t previousPstrategy;	// if P6, store value where it shall return
	float arm2length;		// starting at axis2
	float arm3length;		// starting at axis3
	float arm4length;		// starting at axis4
	float arm5length;		// starting at axis5

	float angle1limits[2] = {-45.0, 45.0};	// limit angle1 of vertical axis1 min and max (including values both)
	float angle2limits[2] = {0.0, 75.0};	// limit angle2 of axis2 min and max (including values both)
	float angle3limits[2] = {-5.0, -75.0};	// limit angle3 of axis3 min and max (including values both)
	float angle4limits[2] = {-170.0, 170.0};	// limit angle4 of axis4 min and max (including values both)
	float angle5limits[2] = {-135.0, 135.0};	// limit angle5 of vertical axis5 min and max (including values both)

	float arm2bendingFactor = 0.0;	// B parameter a2 value
	float arm3bendingFactor = 0.0;	// B parameter a3 value
	float arm5bendingFactor = 0.0;	// B parameter a5 value

};

#endif /* SRC_MOVEMENT_KINEMATICS_FIVEAXISROBOTKINEMATICS_H_ */
