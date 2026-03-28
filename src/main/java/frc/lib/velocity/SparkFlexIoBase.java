package frc.lib.velocity;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.angular_position.AngularPositionComponent;
import frc.lib.logging.LoggableIo;
import frc.lib.voltage.VoltageComponent;
import static edu.wpi.first.units.Units.Volts;

public class SparkFlexIoBase extends LoggableIo<SparkFlexIoBase.SparkFlexIoInputs>
        implements AngularVelocityComponent, AngularPositionComponent, VoltageComponent {
    @AutoLog
    public static abstract class SparkFlexIoInputs implements LoggableInputs {
        public double motorCurrent;
        public double motorTemp;
        public AngularVelocity currentVelocity;
        public Angle position = Angle.ofRelativeUnits(0, Units.Rotations);
    }

    public SparkFlexIoBase() {
        super(new SparkFlexIoInputsAutoLogged());
    }

    @Override
    public void neutralOutput() {
    }

    @Override
    public void setVelocity(AngularVelocity setpoint) {
        logger().debug("VelocitySetpoint", setpoint)
                .debug("Active", true);
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
    public Angle getAngle() {
        return getInputs().position;
    }

    @Override
    public void setVoltage(Voltage voltage) {
        logger().debug("Voltage", voltage.in(Volts))
                .debug("Active", true);
    }

    @Override
    public void setAngle(Angle setpoint) {
        logger().debug("AngleSetpoint", setpoint)
                .debug("Active", true);
    }

    @Override
    public void setPercievedAngle(Angle angle) {
    }
}
