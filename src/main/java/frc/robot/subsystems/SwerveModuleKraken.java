package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import frc.robot.hardware.KrakenMotor;

// THIS CLASS IS NOT COMPLETE


public class SwerveModuleKraken {
    private final KrakenMotor driveMotor;
    private final KrakenMotor turnMotor;

    private final double P = 0.2;
    private final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(14);

    public SwerveModuleKraken(int driveMotorDeviceId, int turnMotorDeviceId) {
        driveMotor = new KrakenMotor(driveMotorDeviceId);
        turnMotor = new KrakenMotor(turnMotorDeviceId);
    }

    public KrakenMotor getDriveMotor() {
        return driveMotor;
    }

    public KrakenMotor getTurnMotor() {
        return turnMotor;
    }

    public void setState(SwerveModuleState state) {
        double speedMetersPerSecond = state.speedMetersPerSecond;
        driveMotor.set(speedMetersPerSecond / MAX_SPEED_METERS_PER_SECOND);
        setAngle(state.angle);
    }

    // this is not
    public void setAngle(Rotation2d desiredAngle) {
        Rotation2d currentAngle = Rotation2d.fromRotations(turnMotor.getPositionRotations());
        double errorRadians = desiredAngle.getRadians() - currentAngle.getRadians();
        if (Math.abs(errorRadians) < Math.PI) {
            errorRadians = -errorRadians;
        }
        double speed = errorRadians * P;
        turnMotor.set(speed);
    }
}