package frc.lib.component;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

import frc.lib.intermediate.AngleComponentOffsetter;
import frc.lib.intermediate.AngleLimiter;
import frc.lib.intermediate.AngleRatio;
import frc.lib.util.NeutralOutput;

public interface AngleComponent extends AngleSensor, NeutralOutput{
    void setAngle(Angle angle);
    
    void setPercievedAngle(Angle angle);
    
    default void setAngle(Rotation2d angle){
        setAngle(angle.getMeasure());
    }

    @Override
    default AngleComponent withOffset(Angle offset){
        return new AngleComponentOffsetter<>(offset, this);
    }

    @Override
    default AngleComponent withOffset(Rotation2d offset) {
        return new AngleComponentOffsetter<>(offset.getMeasure(), this);
    }

    default AngleComponent withAngleRatio(double reductionRatio) {
        return new AngleRatio(reductionRatio, this);
    }

    default AngleComponent withLimits(Angle max, Angle min) {
        return new AngleLimiter(min, max, this);
    }
}
