package frc.lib.elevator;

import frc.lib.NeutralOutput;
import frc.lib.loggable.LoggableComponent;

public interface ElevatorComponent extends LoggableComponent, NeutralOutput {
    void setTargetPosition(double pos);

    void stop();

    void goHome();

    boolean isHome();

    double getCurrentPosition();

    double getMotorCurrent();

    double getCurrentVelocity();

    double getMotorTemp();

    double getHeightError();
}
