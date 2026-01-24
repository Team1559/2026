package frc.lib.subsystems.elevator;

import frc.lib.subsystems.LoggableComponent;

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
