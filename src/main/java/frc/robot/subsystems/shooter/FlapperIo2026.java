package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystems.LoggableIo;

public class FlapperIo2026 extends LoggableIo<FlapperIo2026.FlapperInputs> {
    @AutoLog
    public static abstract class FlapperInputs implements LoggableInputs {
        public Rotation2d currentPosition;
    }
    
    public FlapperIo2026(String name) {
        super(name, new FlapperInputsAutoLogged());
    }

    public void setTargetProjectileAngle(Rotation2d targetAngle) {
        Logger.recordOutput(getOutputLogPath("TargetAngle"), targetAngle);
    }

    public void stop() {
        Rotation2d currentPos = getInputs().currentPosition;
        setTargetProjectileAngle(currentPos);
    }

    public void goHome() {
        Logger.recordOutput(getOutputLogPath("TargetAngle"),(Rotation2d)null);
        
    }




}
