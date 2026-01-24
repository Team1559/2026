// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.commands.StopCommand;
import frc.lib.subsystems.DriverAssist;
import frc.lib.subsystems.swerve.TeleopDriveCommand;
import frc.robot.subsystems.SwerveDrive2026;
import frc.robot.subsystems.Vision2026;

public class Robot extends LoggedRobot {

    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController pilotController;
    private final CommandXboxController coPilotController;
    private final DriverAssist driverAssist;
    private final SwerveDrive2026 drivetrain;
    private final Vision2026 vision;
    @SuppressWarnings("resource") //pdh must stay open for connection
    public Robot() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("hi/test", ":)"); // Leave as easter egg
        pilotController = new CommandXboxController(0);
        coPilotController = new CommandXboxController(1);
        driverAssist = new DriverAssist("DriverAssist");
        drivetrain = new SwerveDrive2026();
        vision= new Vision2026 (drivetrain);
        
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("a");
        SmartDashboard.putData(autoChooser);

        DriverStation.silenceJoystickConnectionWarning(true);

        PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
        pdh.setSwitchableChannel(true);
    }

    public void clearCommandBindings() {
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("StopCommand", new StopCommand(drivetrain).withTimeout(1));
    }

    public void setTeleopBindings() {
        drivetrain.setDefaultCommand(new TeleopDriveCommand(()->pilotController.getLeftY()*-1, ()->pilotController.getLeftX()*-1, () -> pilotController.getRightX(), SwerveDrive2026.SWERVE_CONSTRAINTS, drivetrain, ()->false)); //() -> pilotController.getRightX()
    }

    public void setTestBindings() {
    }

    @Override
    public void robotInit() {
        
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(autoChooser.getSelected());
    }

    @Override
    public void teleopInit() {
        clearCommandBindings();
        setTeleopBindings();
    }

    @Override
    public void testInit() {
        clearCommandBindings();
        setTestBindings();
    }

}
