package frc.lib.subsystems.fixed_velocity;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.subsystems.LoggableIo;

public class VelocityIo extends LoggableIo<VelocityIo.VelocityInputs> {
    @AutoLog
    public static abstract class VelocityInputs implements LoggableInputs {
        public double motorCurrent;
        public double motorTemp;
        public double currentVelocity;
    }

    public VelocityIo(String name) {
        super(name, new VelocityInputsAutoLogged());
    }

    public void run(double targetVelocity) {
        Logger.recordOutput(getOutputLogPath("TargetVelocity"), targetVelocity);
        Logger.recordOutput(getOutputLogPath("Running"), true);
    }
    
    public void stop() {
        Logger.recordOutput(getOutputLogPath("Running"), false); 
    }
}
