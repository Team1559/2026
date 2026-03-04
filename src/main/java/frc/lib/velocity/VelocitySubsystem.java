package frc.lib.velocity;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.NeutralOutput;
import frc.lib.loggable.LoggableSubsystem;

public class VelocitySubsystem extends LoggableSubsystem implements NeutralOutput {
    private final AngularVelocityComponent[] children;
    private AngularVelocity velocitySetpoint;
   

    public VelocitySubsystem(String name, AngularVelocityComponent... children) {
        super(name);
        this.children = children;
        addChildren(children);
    }

    public void run(AngularVelocity setpoint) {
        velocitySetpoint = setpoint;
    }

    @Override
    public void neutralOutput() {
      velocitySetpoint=null;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (velocitySetpoint!=null) {
            for (AngularVelocityComponent i : children) {
                i.setVelocity(velocitySetpoint);
            }
        } else {
            for (AngularVelocityComponent i : children) {
                i.neutralOutput();
            }
        }
    }
}
