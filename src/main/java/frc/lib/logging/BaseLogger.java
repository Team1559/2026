package frc.lib.logging;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.RobotBase;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@GenerateLogger
public sealed class BaseLogger permits CustomLogger {
    private static Boolean debugOverride = null;
    private String logPath;

    protected BaseLogger(String logPath) {
        this.logPath = logPath;
    }

    public static void overrideDebugMode(boolean debugMode) {
        debugOverride = debugMode;
    }

    @SuppressWarnings("java:S3400") // Necessary method
    protected static boolean shouldLogDashboard() {
        return true;
    }

    protected static boolean shouldLogDebug() {
        
        if (Logger.hasReplaySource()) {
            return true;
        }
        if (RobotBase.isSimulation()) {
            return true;
        }
        if (RobotState.isFMSAttached()) {
            return false;
        }
        if (debugOverride != null) {
            return debugOverride;
        }

        return RobotState.isUtility();
    }

    protected String getOutputLogPath(String key) {
        return logPath + "/" + key;
    }

    protected void processInputs(LoggableInputs inputs) {
        Logger.processInputs(logPath, inputs);
    }

    public void debugPrintln(String message){
        if (shouldLogDebug()) {
            System.out.println(message);
        }
    }
}
