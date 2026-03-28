// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.util.StatusLogger;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.commands.StopCommand;
import frc.lib.swerve.SwerveDrive;
import frc.lib.swerve.TeleopDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.WiggleIntakeCommand;
import frc.robot.subsystems.Indexer2026;
import frc.robot.subsystems.Intake2026;
import frc.robot.subsystems.Shooter2026;
import frc.robot.subsystems.SwerveDrive2026Competition;
import frc.robot.subsystems.Vision2026;

public class Robot extends LoggedRobot {

    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController pilotController;
    private final CommandXboxController coPilotController;
    private final SwerveDrive drivetrain;

    @SuppressWarnings("unused") // To avoid garbage collection of our vision
    private final Vision2026 vision;
    private final Shooter2026 shooter;
    private final Intake2026 intake;
    private final Indexer2026 indexer;

    private static final boolean IS_REPLAY = true;

    @SuppressWarnings("resource") // pdh must stay open for connection
    public Robot() {
        super(0.02);
        StatusLogger.disableAutoLogging();
        SignalLogger.enableAutoLogging(false);
        if (IS_REPLAY) {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        } else {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        }

        //BaseLogger.overrideDebugMode(false)
        
        Logger.recordMetadata("Git Branch", GitVersion.GIT_BRANCH);
        Logger.recordMetadata("Git Commit Hash", GitVersion.GIT_SHA);
        Logger.recordMetadata("Uncommited Changes",
        GitVersion.DIRTY != 0 ? "Uncommited Changes" : "All Changes Commited");
        Logger.recordMetadata("Project Name", GitVersion.MAVEN_NAME);
        Logger.recordMetadata("Build Date", GitVersion.BUILD_DATE);
        Logger.recordMetadata("Easter Egg", ":)"); // Leave as easter egg (hi/test)
        
        Logger.start();
        
        drivetrain = new SwerveDrive2026Competition();
        vision = new Vision2026(drivetrain);
        shooter = new Shooter2026(drivetrain::getPosition, drivetrain::getCurrentSpeed);
        intake = new Intake2026();
        indexer = new Indexer2026();

        pilotController = new CommandXboxController(0);
        coPilotController = new CommandXboxController(1);

        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);

        DriverStation.silenceJoystickConnectionWarning(true);

        PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
        pdh.setSwitchableChannel(true);
    }

    private static void clearCommandBindings() {
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("DrivetrainStop", new StopCommand(drivetrain).withTimeout(1));
        NamedCommands.registerCommand("Wiggle", new WiggleIntakeCommand(intake));
        NamedCommands.registerCommand("IntakeUp", new InstantCommand(intake::moveElbowUp));
        NamedCommands.registerCommand("IntakeDown", new InstantCommand(intake::moveElbowDown));
        NamedCommands.registerCommand("Shoot", new ShootCommand(shooter,
                Shooter2026::ourHubLocation));
        NamedCommands.registerCommand("RunIntakeForwards", new InstantCommand(intake::runForwards));
        NamedCommands.registerCommand("StopIntake", new InstantCommand(intake::neutralOutput));
        NamedCommands.registerCommand("Intake", new StartEndCommand(intake::runForwards, intake::neutralOutput));
    }

    private void setTeleopBindings() {
        drivetrain.setDefaultCommand(new TeleopDriveCommand(() -> pilotController.getLeftY() * -1,
                () -> pilotController.getLeftX() * -1, () -> pilotController.getRightX() * -1,
                SwerveDrive2026Competition.SWERVE_CONSTRAINTS, drivetrain, () -> false));

        pilotController.leftBumper()
                .whileTrue(new TeleopDriveCommand(() -> pilotController.getLeftY() * -1,
                        () -> pilotController.getLeftX() * -1, () -> pilotController.getRightX() * -1,
                        SwerveDrive2026Competition.SLOW_SWERVE_CONSTRAINTS, drivetrain, () -> false));

        pilotController.leftTrigger()
                .whileTrue(new StartEndCommand(intake::runForwards, intake::neutralOutput,
                        intake));

        pilotController.leftTrigger().onTrue(new InstantCommand(intake::moveElbowDown));

        pilotController.rightStick().onTrue(new InstantCommand(intake::moveElbowUp,
                intake));

        pilotController.rightTrigger().whileTrue(new ShootCommand(shooter,
                shooter::targetLocation));
        pilotController.rightBumper().whileTrue(new WiggleIntakeCommand(intake));

        pilotController.a().onTrue(new InstantCommand(shooter::useAbsoluteAngle));
        pilotController.b().onTrue(new InstantCommand(shooter::zeroTurret));

        // Copilot gets uh oh buttons
        coPilotController.leftTrigger()
                .whileTrue(new StartEndCommand(intake::runReverse, intake::neutralOutput, intake));
        coPilotController.leftBumper().onTrue(new InstantCommand(intake::elbowNeutral));

        coPilotController.rightTrigger()
                .whileTrue(new StartEndCommand(shooter::reverseAll, shooter::neutralAll, shooter));
    }

    private void setTestBindings() {
        pilotController.leftTrigger()
                .whileTrue(new StartEndCommand(intake::runForwards, intake::neutralOutput,
                        intake));
    }

    private void setUniversalBindings() {
        Trigger indexerTrigger = new Trigger(shooter::isShooting).or(intake::isIntaking);
        indexerTrigger.whileTrue(new StartEndCommand(indexer::runForwards,
                indexer::neutralOutput, indexer));
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
        shooter.zeroTurret();
        setUniversalBindings();
        CommandScheduler.getInstance().schedule(autoChooser.getSelected());
    }

    @Override
    public void teleopInit() {
        intake.neutralOutput();
        indexer.neutralOutput();
        shooter.setSpinFlywheel(false);
        shooter.setShooting(false);
        clearCommandBindings();
        setUniversalBindings();
        setTeleopBindings();
    }

    @Override
    public void testInit() {
        intake.neutralOutput();
        indexer.neutralOutput();
        shooter.setSpinFlywheel(false);
        shooter.setShooting(false);
        clearCommandBindings();
        setUniversalBindings();
        setTestBindings();
    }
}
