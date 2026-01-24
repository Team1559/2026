package frc.lib.velocity;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SparkFlexVelocityIo extends VelocityIo {
    private final SparkFlex motor;
    private final SparkClosedLoopController motorController;
    private final RelativeEncoder encoder;

    public SparkFlexVelocityIo(String name, SparkFlex motor, SparkFlexConfig motorConfig) {
        super(name);
        this.motor = motor;
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorController = motor.getClosedLoopController();
        encoder = motor.getEncoder();
    }

    @Override
    protected void updateInputs(VelocityInputs inputs) {
        inputs.currentVelocity = encoder.getVelocity();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
    }

    @Override
    public void run(double targetVelocity) {
        super.run(targetVelocity);
        motorController.setSetpoint(targetVelocity, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        super.stop();
        motor.stopMotor();
    }
}
