package frc.lib.velocity;

import java.util.Map;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.NeutralOutput;
import frc.lib.logging.LoggableSubsystem;

public class VelocitySubsystem extends LoggableSubsystem implements NeutralOutput {
    private final AngularVelocityComponent[] children;
    private AngularVelocity velocitySetpoint;
   

    public VelocitySubsystem(String name, Map<String, AngularVelocityComponent> children) {
        super(name);
        this.children = children.values().toArray(AngularVelocityComponent[]::new);
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
