package frc.robot.subsystems;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.SparkMaxMotor;

public class SwerveModule {
    private final SparkMaxMotor driveMotor;
    private final SparkMaxMotor angleMotor;

    private final AbsoluteEncoder wheelAngleAbsoluteEncoder; // rotations of the wheel, not the motor

    public SwerveModule(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        driveMotor = new SparkMaxMotor(driveMotorDeviceId, false, false);
        // option 1: true, true
        // option 2: false, false
        angleMotor = new SparkMaxMotor(angleMotorDeviceId, true, true);

        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(config, SensorDirectionValue.CounterClockwise_Positive);

        resetEncoders();
    }

    public void setDriveMotorSpeed(double speed) {
        driveMotor.set(DriveUtils.normalizeSpeed(speed));
    }
    // positive speed is counterclockwise
    public void setAngleMotorSpeed(double speed) {
        angleMotor.set(DriveUtils.normalizeSpeed(speed));
    }

    /**
     * This sets the SwerveModule to the desired state
     * @param state the desired speed and angle
     */
    public void setState(SwerveModuleState state) {
        double driveMotorSpeed = state.speedMetersPerSecond / DriveConstants.kMaxDriveSpeedMetersPerSecond;
        
        double currentWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        double desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(state.angle.getRadians());
        double wheelErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;
        
        // if greater than 90 deg, add 180 deg and flip drive motor direction
        if (Math.abs(wheelErrorRadians) > Math.PI / 2) {
            wheelErrorRadians = DriveUtils.normalizeAngleRadiansSigned(wheelErrorRadians + Math.PI);
            driveMotorSpeed = -driveMotorSpeed;
        }

        setAngleMotorSpeed(DriveUtils.getAngleMotorSpeed(wheelErrorRadians, currentWheelAngleRadians));

        setDriveMotorSpeed(driveMotorSpeed);
    }

    // Return the angle in radians formed by the x and y components
    public static double getAngleRadiansFromComponents(double y, double x) {
        return DriveUtils.normalizeAngleRadiansSigned(Math.atan2(y, x));
    }

    // Set the relative encoder values to default
    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        angleMotor.setEncoderPosition(DriveUtils.angleWheelToMotor(wheelAngleAbsoluteEncoder.getPositionRotations()));
    }

    // meters
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            DriveUtils.driveMotorToWheel(driveMotor.getPositionRadians()) * SwerveConstants.kWheelRadiusMeters,
            Rotation2d.fromRadians(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()))
        );
    }
}

