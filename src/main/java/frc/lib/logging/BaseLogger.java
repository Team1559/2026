package frc.lib.logging;

import edu.wpi.first.wpilibj.DriverStation;

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
        if (DriverStation.isFMSAttached()) {
            return false;
        }

        if (Logger.hasReplaySource()) {
            return true;
        }

        if (debugOverride != null) {
            return debugOverride;
        }

        return DriverStation.isTest();
    }

    protected String getOutputLogPath(String key) {
        return logPath + "/" + key;
    }

    protected void processInputs(LoggableInputs inputs) {
        Logger.processInputs(logPath, inputs);
    }
}
