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
	static constexpr const char *Home5AxisRobotFileName = "homeall.g"; // ex "home5axisrobot.g";
	static constexpr float DefaultSegmentsPerSecond = 100.0;
	static constexpr float DefaultMinSegmentSize = 0.2;

	void Recalc() noexcept;
	void optimizeCode(int32_t valInt);
	void getIntersec(float result12[], float firstRadius, float secondRadius, float firstX, float firstY, float secondX, float secondY) const noexcept;
	bool getIntersectionUpper(float _axis1x, float _axis1y, float _axis2y, float xmid, float ymid, float radiusMid,
			float tangent[], bool upper) const;
	float getAngle1(float x, float y, float z) const;
	void getAxis2Coords(float angle1, float axis2coords[]) const;
	void getAxis3Coords(float angle1, const float axis2coords[], const float axis4coords[], float axis3coords[],
			float angles234[]) const;
	void getAxis4Coords(const float axis5coords[], float axis4coords[], float angle1) const;
	void getAxis5Coords(float x, float y, float z, float angle1, float axis5coords[]) const;
	void setPlannedPath(float sourcePath[], float destPath[]) const noexcept;
	int32_t getActuatorsCount() const noexcept;		// without extruder, but with rail
	bool getAnglesCartesianToMotorSteps(const float machinePos[], float angles[]) const noexcept;

	// Primary parameters
	float axis1coords[2];		// XY
	float axis2coords[3];		// XYZ
	bool axis2yis0 = false;		// if true, arm 2 goes through axis 1
	bool arm4vertical = true;	// default behaviour
	float axis5offset[2] = {0.0, 0.0};		// Xo, Yo parameters
	int32_t pMode = 0;		// P setting
	float p2Angle = 0.0;		// only relevant for P2 mode
	int32_t rMode = 0;		// R setting

	mutable float plannedPath[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};		// xyz cartesian coords start and end of current G0/G1 movement
								// needed for P1 and P3
	mutable float plannedPathAngleXY = 0.0;		// absoulute angle, Z is constant, counterclockwise

	float arm2length = 0.0;		// starting at axis2
	float arm3length = 0.0;		// starting at axis3
	float arm4length = 0.0;		// starting at axis4
	float arm5length = 0.0;		// starting at axis5

	// arm orientation and bending factors
	int32_t armOrientation = 0;
	float arm2bending = 0.0;	// B parameter a2 value
	float arm3bending = 0.0;	// B parameter a3 value
	float arm4bending = 0.0;	// B parameter a4 value
	float arm5bending = 0.0;	// B parameter a5 value

	// rail parameters
	bool railUsed = false;				// if rail is used
	int32_t railMode = 0;
	float railX = 0.0;
	float railY = 0.0;
	float railZ = 0.0;

};

#endif /* SRC_MOVEMENT_KINEMATICS_FIVEAXISROBOTKINEMATICS_H_ */
