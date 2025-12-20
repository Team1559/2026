package frc.lib.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.subsystems.LoggableIo;

public class ElevatorIo extends LoggableIo<ElevatorIo.ElevatorInputs> {
    @AutoLog
    public static abstract class ElevatorInputs implements LoggableInputs {
        public boolean isHome;
        public double currentPosition;
        public double motorCurrent;
        public double currentVelocity;
        public double motorTemp;

        public double heightError;
    }

    public ElevatorIo(String name) {
        super(name, new ElevatorInputsAutoLogged());
    }

    public void setTargetPosition(double pos) {
        Logger.recordOutput(getOutputLogPath("TargetPosition"), pos);
    }

    public void stop() {
        double currentPos = getInputs().currentPosition;
        setTargetPosition(currentPos);
    }

    public void goHome() {
        Logger.recordOutput(getOutputLogPath("TargetPosition"), 0D);
    }
}