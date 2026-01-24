package frc.lib.velocity;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.LoggableIo;
import frc.lib.subsystems.velocity.VelocityInputsAutoLogged;

public class VelocityIo extends LoggableIo<VelocityIo.VelocityInputs> implements VelocityComponent {
    @AutoLog
    public static abstract class VelocityInputs implements LoggableInputs {
        public double motorCurrent;
        public double motorTemp;
        public double currentVelocity;
    }

    public VelocityIo(String name) {
        super(name, new VelocityInputsAutoLogged());
    }

    @Override
    public void run(double targetVelocity) {
        Logger.recordOutput(getOutputLogPath("TargetVelocity"), targetVelocity);
        Logger.recordOutput(getOutputLogPath("Running"), true);
    }

    @Override
    public void stop() {
        Logger.recordOutput(getOutputLogPath("Running"), false);
    }

    @Override
    public double getMotorCurrent() {
        return getInputs().motorCurrent;
    }

    @Override
    public double getMotorTemperature() {
        return getInputs().motorTemp;
    }

    @Override
    public double getCurrentVelocity() {
        return getInputs().currentVelocity;
    }
}
