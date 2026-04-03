package frc.lib.io;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.component.AngleComponent;
import frc.lib.component.AngularVelocityComponent;
import frc.lib.component.VoltageComponent;
import frc.lib.logging.LoggableIo;

public class TalonFXIoBase extends LoggableIo<TalonFXIoBase.TalonFXIoInputs>
        implements AngularVelocityComponent, AngleComponent, VoltageComponent {
    private static final String ACTIVE = "Active";

            @AutoLog
            public static abstract class TalonFXIoInputs implements LoggableInputs {
                public Current statorCurrent;
                public Current supplyCurrent;
                public Temperature motorTemp;
                public AngularVelocity currentVelocity = RPM.zero();
                public Angle position = Angle.ofRelativeUnits(0, Units.Rotations);
            }

            public TalonFXIoBase() {
                super(new TalonFXIoInputsAutoLogged());
            }

            @Override
            public void neutralOutput() {
                logger().debug(ACTIVE, false);
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
            public void setAngle(Angle angle) {
                logger().debug("AngleSetpoint", angle)
                        .debug(ACTIVE, true);
            }

            @Override
            public void setPercievedAngle(Angle angle) {
                // Doesn't do anything in Replay Mode
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
}
