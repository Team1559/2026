package frc.lib.elevator;

import frc.lib.LoggableComponent;

public interface ElevatorComponent extends LoggableComponent {
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
