package frc.lib.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.subsystems.LoggableIo;

public class ElevatorIo extends LoggableIo<ElevatorIo.ElevatorInputs> implements ElevatorComponent {
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

    @Override
    public void setTargetPosition(double pos) {
        Logger.recordOutput(getOutputLogPath("TargetPosition"), pos);
    }

    @Override
    public void stop() {
        double currentPos = getInputs().currentPosition;
        setTargetPosition(currentPos);
    }

    @Override
    public void goHome() {
        Logger.recordOutput(getOutputLogPath("TargetPosition"), 0D);
    }

    @Override
    public boolean isHome() {
        return getInputs().isHome;
    }

    @Override
    public double getCurrentPosition() {
        return getInputs().currentPosition;
    }

    @Override
    public double getMotorCurrent() {
        return getInputs().motorCurrent;
    }

    @Override
    public double getCurrentVelocity() {
        return getInputs().currentVelocity;
    }

    @Override
    public double getMotorTemp() {
        return getInputs().motorTemp;
    }

    @Override
    public double getHeightError() {
        return getInputs().heightError;
    }
}