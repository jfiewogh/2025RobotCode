package frc.robot.hardware;

import com.ctre.phoenix6.hardware.TalonFX;

public class KrakenMotor {
    private final TalonFX motor;

    public KrakenMotor(int deviceId) {
        motor = new TalonFX(deviceId);
    }

    public double getPositionRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    public void set(double relativeSpeed) {
        motor.set(relativeSpeed);
    }
}
