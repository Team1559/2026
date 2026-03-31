package frc.lib.component;

import edu.wpi.first.units.measure.Angle;

import frc.lib.intermediate.AngleLimiter;
import frc.lib.intermediate.AngleRatio;
import frc.lib.util.NeutralOutput;

public interface AngleComponent extends AngleSensor, NeutralOutput{
    void setAngle(Angle angle);

    void setPercievedAngle(Angle angle);

    default AngleComponent withRatio(double reductionRatio) {
        return new AngleRatio(reductionRatio, this);
    }

    default AngleComponent withLimits(Angle max, Angle min) {
        return new AngleLimiter(min, max, this);
    }
}
