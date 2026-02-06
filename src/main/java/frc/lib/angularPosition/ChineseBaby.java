package frc.lib.angularPosition;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.LoggableIntermediate;

public class ChineseBaby extends LoggableIntermediate implements AngularPositionSensor {

    private final AngularPositionSensor childOne;
    private final AngularPositionSensor childTwo;

    private final int gearOneTeeth;
    private final int gearTwoTeeth;
    private final int largeGearTeeth;
    
    private final int coefficentOne;
    private final int coefficentTwo;


    public ChineseBaby(String name, int gearOneTeeth, int gearTwoTeeth, int largeGearTeeth, int coefficentOne, int coefficentTwo, AngularPositionSensor childOne, AngularPositionSensor childTwo){
        super(name);

        this.childOne = childOne;
        this.childTwo = childTwo;

        this.gearOneTeeth = gearOneTeeth;
        this.gearTwoTeeth = gearTwoTeeth;
        this.largeGearTeeth = largeGearTeeth;

        this.coefficentOne = coefficentOne;
        this.coefficentTwo = coefficentTwo;

        this.addChildren(childOne, childTwo);
    }

    @Override
    public Angle getAngle() {
        double modOne = childOne.getAngle().in(Units.Rotations) * gearOneTeeth;
        double modTwo = childTwo.getAngle().in(Units.Rotations) * gearTwoTeeth;

        return Rotations.of((modOne * gearTwoTeeth * coefficentTwo + modTwo * gearOneTeeth * coefficentOne) % (gearTwoTeeth * gearOneTeeth) / largeGearTeeth);
    }
    
    
}
