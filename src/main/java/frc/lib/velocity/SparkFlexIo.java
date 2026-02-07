package frc.lib.velocity;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.LoggableIo;
import frc.lib.angularPosition.AngularPositionComponent;

public class SparkFlexIo extends LoggableIo<SparkFlexIo.SparkFlexIoInputs> implements AngularVelocityComponent, AngularPositionComponent {
    @AutoLog
    public static abstract class SparkFlexIoInputs implements LoggableInputs {
        public double motorCurrent;
        public double motorTemp;
        public AngularVelocity currentVelocity;
        public Angle position = Angle.ofRelativeUnits(0, Units.Rotations);
    }

    private final SparkFlex motor;
    private final SparkClosedLoopController motorController;
    private final RelativeEncoder encoder;

    public SparkFlexIo(String name, SparkFlex motor, SparkFlexConfig motorConfig) {
        super(name, new SparkFlexIoInputsAutoLogged());
        this.motor = motor;
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorController = motor.getClosedLoopController();
        encoder = motor.getEncoder();
    }

    @Override
    protected void updateInputs(SparkFlexIoInputs inputs) {
        inputs.currentVelocity = RPM.of(encoder.getVelocity());
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.position = Rotations.of(encoder.getPosition());
    }

    @Override
    public void setTargetVelocity(AngularVelocity targetVelocity) {
        Logger.recordOutput(getOutputLogPath("TargetVelocity"), targetVelocity);
        Logger.recordOutput(getOutputLogPath("Running"), true);
        motorController.setSetpoint(targetVelocity.in(Units.RPM), ControlType.kVelocity);
    }

    @Override
    public void stop() {
        Logger.recordOutput(getOutputLogPath("Running"), false);
        motor.stopMotor();
        Logger.recordOutput(getOutputLogPath("TargetVelocity"), RPM.zero());
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
    public AngularVelocity getCurrentVelocity() {
        return getInputs().currentVelocity;
    }

    @Override
    public void setTargetAngle(Angle angle) {
        Logger.recordOutput(getOutputLogPath("TargetAngle"), angle);
        motorController.setSetpoint(angle.in(Units.Rotations), ControlType.kPosition);
    }

    @Override
    public Angle getAngle() {
        return getInputs().position;
    }

    @Override
    public void setPercievedAngle(Angle angle) {
        encoder.setPosition(angle.in(Units.Rotations));
    }
}
