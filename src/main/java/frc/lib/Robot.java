package frc.lib;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Scanner;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GitVersion;

public abstract class Robot extends LoggedRobot {
    private static final String ENVIRONMENT_VARIABLE = "AKIT_LOG_PATH";
    private static final String ADVANTAGE_SCOPE_FILE_NAME = "akit-log-path.txt";

    protected Robot() {
        super(0.02);

        DriverStation.silenceJoystickConnectionWarning(true);
        StatusLogger.disableAutoLogging();
        SignalLogger.enableAutoLogging(false);

        String logPath = findReplayLog();
        if (isSimulation() && logPath != null) {
            setUseTiming(false);
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        } else {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.recordMetadata("Git Branch", GitVersion.GIT_BRANCH);
        Logger.recordMetadata("Git Commit Hash", GitVersion.GIT_SHA);
        Logger.recordMetadata("Uncommited Changes",
                GitVersion.DIRTY != 0 ? "Uncommited Changes" : "All Changes Commited");
        Logger.recordMetadata("Project Name", GitVersion.MAVEN_NAME);
        Logger.recordMetadata("Build Date", GitVersion.BUILD_DATE);
        Logger.recordMetadata("Easter Egg", ":)"); // Leave as easter egg (hi/test)

        Logger.start();
    }

    private static String findReplayLog() {
        // Read environment variables
        String envPath = findReplayLogEnvVar();
        if (envPath != null) {
            System.out.println(
                    "[AdvantageKit] Replaying log from "
                            + ENVIRONMENT_VARIABLE
                            + " environment variable: \""
                            + envPath
                            + "\"");
            return envPath;
        }

        // Read file from AdvantageScope
        String advantageScopeLogPath = findReplayLogAdvantageScope();
        if (advantageScopeLogPath != null) {
            System.out.println(
                    "[AdvantageKit] Replaying log from AdvantageScope: \"" + advantageScopeLogPath + "\"");
            return advantageScopeLogPath;
        }

        System.out.println("No replay log found...");
        return null;
    }

    /** Read the replay log from the environment variable. */
    private static String findReplayLogEnvVar() {
        return System.getenv(ENVIRONMENT_VARIABLE);
    }

    /** Read the replay log from AdvantageScope. */
    @SuppressWarnings("java:S1166")
    private static String findReplayLogAdvantageScope() {
        Path advantageScopeTempPath = Paths.get(System.getProperty("java.io.tmpdir"), ADVANTAGE_SCOPE_FILE_NAME);

        try (Scanner fileScanner = new Scanner(advantageScopeTempPath)) {
            return fileScanner.nextLine(); // AdvantageScope log path
        } catch (IOException e) {
            return null; // Error anytime there's no log in AdvantageScope
        }

    }

    protected static void clearCommandBindings() {
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }

    protected abstract Command getAutoCommand();

    protected abstract void setTeleopBindings();

    protected abstract void setTestBindings();

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(getAutoCommand());
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        clearCommandBindings();
    }

    @Override
    public void teleopInit() {
        setTeleopBindings();
    }

    @Override
    public void testInit() {
        setTestBindings();
    }

}
