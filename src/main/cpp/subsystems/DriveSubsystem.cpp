#include "subsystems/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem() :
    fl{FL, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    fr{FR, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    bl{BL, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    br{BR, rev::CANSparkMaxLowLevel::MotorType::kBrushless},

    flEncoder{fl.GetEncoder()},
    frEncoder{fr.GetEncoder()},
    blEncoder{bl.GetEncoder()},
    brEncoder{br.GetEncoder()},

    differentialDrive{fl, fr},

    pigeon{0},

    kinematics{TRACK_WIDTH},
    odometry{frc::Rotation2d{0_deg}, 0_m, 0_m, frc::Pose2d{0_m, 0_m, 0_deg}}
{
    fl.RestoreFactoryDefaults();
    fr.RestoreFactoryDefaults();
    bl.RestoreFactoryDefaults();
    br.RestoreFactoryDefaults();

    fr.SetInverted(true);
    br.SetInverted(true);

    bl.Follow(fl);
    br.Follow(fr);
    
    pigeon.SetYaw(0);
}

void DriveSubsystem::Periodic() {
    odometry.Update(frc::Rotation2d{GetAngleDeg()}, units::meter_t{GetLeftDistance()}, units::meter_t{GetRightDistance()});
}

void DriveSubsystem::SimulationPeriodic() {

}

void DriveSubsystem::DriveArcade(double fwd, double omega) {
    differentialDrive.ArcadeDrive(fwd, omega, true);
}

void DriveSubsystem::DriveCurvature(double fwd, double omega) {
    differentialDrive.CurvatureDrive(fwd, omega, true);
}

void DriveSubsystem::DriveVoltage(units::volt_t left, units::volt_t right) {
    fl.SetVoltage(left);
    fr.SetVoltage(right);
}

double DriveSubsystem::GetAngle() {
    pigeon.GetYaw();
}

units::degree_t DriveSubsystem::GetAngleDeg() {
    return units::degree_t{GetAngle()};
}

double DriveSubsystem::GetLeftDistance() {
    return (flEncoder.GetPosition() + blEncoder.GetPosition()) / 2;
}

double DriveSubsystem::GetRightDistance() {
    return (frEncoder.GetPosition() + brEncoder.GetPosition()) / 2;
}

double DriveSubsystem::GetAverageDistance() {
    return (GetLeftDistance() + GetRightDistance()) / 2;
}

void DriveSubsystem::ResetEncoders() {
    flEncoder.SetPosition(0);
    frEncoder.SetPosition(0);
    blEncoder.SetPosition(0);
    brEncoder.SetPosition(0);
}

frc::Pose2d DriveSubsystem::GetPose() {
    return odometry.GetPose();
}

frc::DifferentialDriveKinematics DriveSubsystem::GetKinematics() {
    return kinematics;
}