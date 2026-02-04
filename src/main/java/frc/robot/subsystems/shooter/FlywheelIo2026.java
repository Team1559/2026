package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.LoggableIo;

public class FlywheelIo2026 extends LoggableIo<FlywheelIo2026.FlywheelInputs> {
    @AutoLog
    public static abstract class FlywheelInputs implements LoggableInputs {
        public double motorCurrent;
        public double currentVelocity;
        public double motorTemp;
        public double motorRpm;
    }

    public FlywheelIo2026(String name) {
        super(name, new FlywheelInputsAutoLogged());
    }

    public void setProjectileVelocity(double velocity) {
        Logger.recordOutput(getOutputLogPath("TargetProjectileVelocity"), velocity);
    }

    public void stop() {
        setProjectileVelocity(0);
    }

}
