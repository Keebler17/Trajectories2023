#pragma once

#include <frc2/command/SubsystemBase.h>

#include <units/voltage.h>
#include <units/angle.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include <frc/drive/DifferentialDrive.h>

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

#include <ctre/phoenix/sensors/PigeonIMU.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc/simulation/DifferentialDrivetrainSim.h>

#include "Constants.h"

class DriveSubsystem : public frc2::SubsystemBase {
    public:
        DriveSubsystem();

        void Periodic() override;
        void SimulationPeriodic() override;

        void DriveArcade(double fwd, double omega);
        void DriveCurvature(double fwd, double omega);
        void DriveVoltage(units::volt_t left, units::volt_t right);

        double GetAngle();
        units::degree_t GetAngleDeg();

        double GetLeftDistance();
        double GetRightDistance();
        double GetAverageDistance();
        void ResetEncoders();

        frc::Pose2d GetPose();
        frc::DifferentialDriveKinematics GetKinematics();
    private:
        rev::CANSparkMax fl;
        rev::CANSparkMax fr;
        rev::CANSparkMax bl;
        rev::CANSparkMax br;

        rev::SparkMaxRelativeEncoder flEncoder;
        rev::SparkMaxRelativeEncoder frEncoder;
        rev::SparkMaxRelativeEncoder blEncoder;
        rev::SparkMaxRelativeEncoder brEncoder;

        frc::DifferentialDrive differentialDrive;

        ctre::phoenix::sensors::PigeonIMU pigeon;

        frc::DifferentialDriveKinematics kinematics;
        frc::DifferentialDriveOdometry odometry;
};