package frc.lib.io;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.component.AngleComponent;
import frc.lib.component.AngularVelocityComponent;
import frc.lib.component.VoltageComponent;
import frc.lib.logging.LoggableIo;

public class SparkFlexIoBase extends LoggableIo<SparkFlexIoBase.SparkFlexIoInputs>
        implements AngularVelocityComponent, AngleComponent, VoltageComponent {
    private static final String ACTIVE = "Active";

    @AutoLog
    public static abstract class SparkFlexIoInputs implements LoggableInputs {
        public double motorCurrent;
        public double motorTemp;
        public AngularVelocity currentVelocity = RPM.zero();
        public Angle position = Angle.ofRelativeUnits(0, Units.Rotations);
        public double appliedOutput = 0.0;
    }

    public SparkFlexIoBase() {
        super(new SparkFlexIoInputsAutoLogged());
    }

    @Override
    public void neutralOutput() {
        // neutralOutput() Does not do anything in Replay Mode
        logger().debug(ACTIVE, false);
    }

    @Override
    public void setVelocity(AngularVelocity setpoint) {
        logger().debug("VelocitySetpoint", setpoint)
                .debug(ACTIVE, true);
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
                .debug(ACTIVE, true);
    }

    @Override
    public void setAngle(Angle setpoint) {
        logger().debug("AngleSetpoint", setpoint)
                .debug(ACTIVE, true);
    }

    @Override
    public void setPercievedAngle(Angle angle) {
        // Doesn't do anything in Replay Mode
    }
}
