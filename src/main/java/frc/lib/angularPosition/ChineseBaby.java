package frc.lib.angularPosition;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.lib.LoggableIntermediate;

public class ChineseBaby extends LoggableIntermediate implements AngularPositionSensor {

    private final AngularPositionSensor childOne;
    private final AngularPositionSensor childTwo;

    private final int gearOneTeeth;
    private final int gearTwoTeeth;
    private final int largeGearTeeth;

    private final Angle minAngle;
    private final Angle maxAngle;

    /**
     * gearOne causes consistent, small errors
     * gearTwo causes inconsistent, large errors
     */
    public ChineseBaby(String name, int gearOneTeeth, int gearTwoTeeth, int largeGearTeeth, AngularPositionSensor childOne, AngularPositionSensor childTwo, Angle minAngle, Angle maxAngle){
        
        super(name);

        this.childOne = childOne;
        this.childTwo = childTwo;

        this.gearOneTeeth = gearOneTeeth;
        this.gearTwoTeeth = gearTwoTeeth;
        this.largeGearTeeth = largeGearTeeth;

        this.minAngle = minAngle;
        this.maxAngle = maxAngle;

        this.addChildren(childOne, childTwo);
    }

    @Override 
    public Angle getAngle(){
        Rotation2d gearOneAngle = Rotation2d.fromRotations(childOne.getAngle().in(Rotations) % 1);
        Rotation2d gearTwoAngle = Rotation2d.fromRotations(childTwo.getAngle().in(Rotations) % 1);

        double minErrRotations = Double.MAX_VALUE;
        Angle ans = null;
        Angle largeGearAngle = gearOneAngle.times((double)gearOneTeeth / largeGearTeeth).getMeasure();
        while(largeGearAngle.gte(minAngle)) {
            Rotation2d expectedGearTwoAngle = Rotation2d.fromRotations(largeGearAngle.times((double)largeGearTeeth / gearTwoTeeth).in(Rotations) % 1);
            double err = Math.abs(expectedGearTwoAngle.minus(gearTwoAngle).getRotations());
            if(minErrRotations > err) {
                ans = largeGearAngle;
                minErrRotations = err;
            }
            largeGearAngle = largeGearAngle.minus(Rotations.of((double)gearOneTeeth / largeGearTeeth));
        }

        largeGearAngle = gearOneAngle.times((double)gearOneTeeth / largeGearTeeth).getMeasure();

        while(largeGearAngle.lte(maxAngle)) {
            Rotation2d expectedGearTwoAngle = Rotation2d.fromRotations(largeGearAngle.times((double)largeGearTeeth / gearTwoTeeth).in(Rotations) % 1);
            double err = Math.abs(expectedGearTwoAngle.minus(gearTwoAngle).getRotations());
            if(minErrRotations > err) {
                ans = largeGearAngle;
                minErrRotations = err;
            }
            largeGearAngle = largeGearAngle.plus(Rotations.of((double)gearOneTeeth / largeGearTeeth));
        }
        return ans;
    }
    
    
}
