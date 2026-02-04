package frc.lib.angularPosition;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LoggableIo;

public class AngularPositionIO extends LoggableIo<AngularPositionIO.AngularPositionInputs> implements AngularPositionComponent{
    
    @AutoLog
    public static abstract class AngularPositionInputs implements LoggableInputs {
        public double currentPosition;

        public double motorCurrent;
        public double currentVelocity;
        public double motorTemp;
    }

    public AngularPositionIO(String name) {
        super(name, new AngularPositionInputsAutoLogged());
    }

    @Override
    public void setAngle(Rotation2d angle) {
        Logger.recordOutput(getOutputLogPath("TargetAngle"), angle);
    }

    @Override
    public Rotation2d getAngle() {
        return getInputs().currentPosition;
    }
    
}
