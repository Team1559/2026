// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.commands.StopCommand;
import frc.lib.swerve.SwerveDrive;
import frc.lib.swerve.TeleopDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Indexer2026;
import frc.robot.subsystems.Intake2026;
import frc.robot.subsystems.Shooter2026;
import frc.robot.subsystems.SwerveDrive2026Competition;
import frc.robot.subsystems.SwerveDrive2026Practice;
import frc.robot.subsystems.Vision2026;

public class Robot extends LoggedRobot {

    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController pilotController;
    private final CommandXboxController coPilotController;
    private final SwerveDrive drivetrain;
    private final Vision2026 vision;
    private final Shooter2026 shooter;
    private final Intake2026 intake;
    private final Indexer2026 indexer;
    private static final boolean IS_REPLAY = false;
    private int loopIterations = 0;
    @SuppressWarnings("resource") //pdh must stay open for connection
    public Robot() {
        super(0.02);
        if (IS_REPLAY) {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        } else {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        }
        
        Logger.start();
        Logger.recordOutput("hi/test", ":)"); // Leave as easter egg
        pilotController = new CommandXboxController(0);
        coPilotController = new CommandXboxController(1);
        drivetrain = new SwerveDrive2026Competition();
        vision = new Vision2026 (drivetrain);
        shooter = new Shooter2026(drivetrain::getPosition);
        intake = new Intake2026();
        indexer = new Indexer2026();

        
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);

        DriverStation.silenceJoystickConnectionWarning(true);

        PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
        pdh.setSwitchableChannel(true);
    }

    public void clearCommandBindings() {
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("DrivetrainStop", new StopCommand(drivetrain).withTimeout(1));
        // NamedCommands.registerCommand("IntakeDown", intake.downCommand());
        // NamedCommands.registerCommand("IntakeUp", intake.upCommand());
        NamedCommands.registerCommand("HubAim", shooter.getAimCommand(Shooter2026.ourHubLocation));
        NamedCommands.registerCommand("Shoot", new ShootCommand(indexer, shooter));
        NamedCommands.registerCommand("RunIntakeForwards", new InstantCommand(() -> intake.runForwards()));
        NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> intake.stop()));
    }

    public void setTeleopBindings() {
        drivetrain.setDefaultCommand(new TeleopDriveCommand(() -> pilotController.getLeftY()*-1, () -> pilotController.getLeftX()*-1, () -> pilotController.getRightX() * -1, SwerveDrive2026Competition.SWERVE_CONSTRAINTS, drivetrain, () -> pilotController.rightBumper().getAsBoolean()));
        
        pilotController.a().whileTrue(new StartEndCommand(() -> intake.runForwards(), () -> intake.stop(), intake));
        pilotController.b().whileTrue(new StartEndCommand(() -> intake.runReverse(), () -> intake.stop(), intake));
        pilotController.rightTrigger().whileTrue(new ShootCommand(indexer, shooter));
        
        pilotController.povUp().whileTrue(new StartEndCommand(() -> intake.moveElbowUp(), () -> intake.stopElbow(), intake));
        pilotController.povDown().whileTrue(new StartEndCommand(() -> intake.moveElbowDown(), () -> intake.stopElbow(), intake));
    }

    public void setTestBindings() {
        drivetrain.setDefaultCommand(new TeleopDriveCommand(() -> pilotController.getLeftY()*-1, () -> pilotController.getLeftX()*-1, () -> pilotController.getRightX() * -1, SwerveDrive2026Competition.SWERVE_CONSTRAINTS, drivetrain, () -> pilotController.rightBumper().getAsBoolean()));

        pilotController.a().whileTrue(new StartEndCommand(() -> shooter.setSpinFeedwheel(true), () -> shooter.setSpinFeedwheel(false), shooter));
        pilotController.b().whileTrue(new StartEndCommand(() -> shooter.setSpinFlywheel(true), () -> shooter.setSpinFlywheel(false)));
        pilotController.povRight().whileTrue(new InstantCommand(() -> shooter.setTurretAngle(Degrees.of(60)), shooter));
        pilotController.povLeft().whileTrue(new InstantCommand(() -> shooter.setTurretAngle(Degrees.of(-60)), shooter));

        pilotController.rightTrigger().onTrue(shooter.getAimCommand(Shooter2026.ourHubLocation));

        pilotController.povUp().whileTrue(new StartEndCommand(() -> intake.moveElbowUp(), () -> intake.stopElbow(), intake));
        pilotController.povDown().whileTrue(new StartEndCommand(() -> intake.moveElbowDown(), () -> intake.stopElbow(), intake));

        pilotController.rightBumper().whileTrue(new StartEndCommand(() -> intake.runForwards(), () -> intake.stop(), intake));
        pilotController.leftBumper().whileTrue(new StartEndCommand(() -> intake.runReverse(), () -> intake.stop(), intake));

        pilotController.y().whileTrue(new StartEndCommand(() -> indexer.runForwards(), () -> indexer.stop(), indexer));
    }

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {
        if (loopIterations % 1 == 0){
            CommandScheduler.getInstance().run();
        } else {
            drivetrain.periodic();
        }
        loopIterations ++;
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
