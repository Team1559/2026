// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.lib.Robot;
import frc.lib.command.TeleopDriveCommand;
import frc.lib.subsystem.SwerveDrive;

import frc.robot.commands.ShootCommand;
import frc.robot.commands.WiggleIntakeCommand;
import frc.robot.subsystems.Indexer2026;
import frc.robot.subsystems.Intake2026;
import frc.robot.subsystems.Shooter2026;
import frc.robot.subsystems.SwerveDrive2026Competition;
import frc.robot.subsystems.Vision2026;

public class Robot2026 extends Robot {

    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController pilotController;
    private final CommandXboxController coPilotController;
    private final SwerveDrive drivetrain;

    @SuppressWarnings("unused") // To avoid garbage collection of our vision
    private final Vision2026 vision;
    private final Shooter2026 shooter;
    private final Intake2026 intake;
    private final Indexer2026 indexer;

    public Robot2026() {
        // BaseLogger.overrideDebugMode(false)
        drivetrain = new SwerveDrive2026Competition();
        vision = new Vision2026(drivetrain);
        shooter = new Shooter2026(drivetrain::getPosition, drivetrain::getCurrentSpeed);
        intake = new Intake2026();
        indexer = new Indexer2026(shooter::isShooting, intake::isIntaking);

        pilotController = new CommandXboxController(0);
        coPilotController = new CommandXboxController(1);

        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("Wiggle", new WiggleIntakeCommand(intake));
        NamedCommands.registerCommand("IntakeUp", new InstantCommand(intake::moveElbowUp));
        NamedCommands.registerCommand("IntakeDown", new InstantCommand(intake::moveElbowDown));
        NamedCommands.registerCommand("Shoot", new ShootCommand(shooter,
                Shooter2026::ourHubLocation));
        NamedCommands.registerCommand("RunIntakeForwards", new InstantCommand(intake::runForwards));
        NamedCommands.registerCommand("StopIntake", new InstantCommand(intake::stop));
        NamedCommands.registerCommand("Intake", new StartEndCommand(intake::runForwards, intake::stop));
    }

    @Override
    protected void setTeleopBindings() {
        drivetrain.setDefaultCommand(new TeleopDriveCommand(() -> pilotController.getLeftY() * -1,
                () -> pilotController.getLeftX() * -1, () -> pilotController.getRightX() * -1,
                SwerveDrive2026Competition.SWERVE_CONSTRAINTS, drivetrain, () -> false));

        pilotController.leftBumper()
                .whileTrue(new TeleopDriveCommand(() -> pilotController.getLeftY() * -1,
                        () -> pilotController.getLeftX() * -1, () -> pilotController.getRightX() * -1,
                        SwerveDrive2026Competition.SLOW_SWERVE_CONSTRAINTS, drivetrain, () -> false));

        pilotController.leftTrigger()
                .whileTrue(new StartEndCommand(intake::runForwards, intake::stop, intake));

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
                .whileTrue(new StartEndCommand(intake::runReverse, intake::stop, intake));
        coPilotController.leftBumper().onTrue(new InstantCommand(intake::elbowNeutral));

        coPilotController.rightTrigger()
                .whileTrue(new StartEndCommand(shooter::reverseAll, shooter::neutralAll, shooter));
    }

    @Override
    protected void setTestBindings() {
        pilotController.leftTrigger()
                .whileTrue(new StartEndCommand(intake::runForwards, intake::stop, intake));
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
        intake.stop();
        indexer.stop();
        shooter.setSpinFlywheel(false);
        shooter.setShooting(false);
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
        shooter.zeroTurret();
    }

	@Override
	protected Command getAutoCommand() {
        return autoChooser.getSelected();
	}
}
