package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LoggableIo;

public class TurretIo2026 extends LoggableIo<TurretIo2026.TurretInputs> {
    @AutoLog
    public static abstract class TurretInputs implements LoggableInputs {
        public boolean isHome;
        public Rotation2d currentPosition;
        public double motorCurrent;
        public double currentVelocity;
        public double motorTemp;

        public Rotation2d angleError;
    }

    public TurretIo2026(String name) {
        super(name, new TurretInputsAutoLogged());
    }

    public void setTarget(Rotation2d target) {
        Logger.recordOutput(getOutputLogPath("Target"), target);
    }

    public void stop() {
        Rotation2d currentPos = getInputs().currentPosition;
        setTarget(currentPos);
    }

    public void goHome() {
        Logger.recordOutput(getOutputLogPath("Target"), 0D);
    }
}
