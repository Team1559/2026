package frc.lib.subsystem;

import java.util.function.DoubleSupplier;

import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.AddressableLEDBufferView;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.util.Color;
import org.wpilib.command2.SubsystemBase;

public class Leds extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private final AddressableLEDBufferView side1;
    private final AddressableLEDBufferView side2;

    private LEDPattern pattern;

    private DoubleSupplier progress;
    private Mode currentMode = Mode.PATTERN;

    private enum Mode {
        PROGRESS,
        PATTERN;
    }

    public Leds(int port, int length) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(buffer);

        side1 = buffer.createView(0, buffer.getLength() / 2 - 1);
        side2 = buffer.createView(buffer.getLength() / 2, length - 1);

        led.start();
        pattern = LEDPattern.solid(Color.BLACK);
        currentMode = Mode.PATTERN;
    }

    public void setAllianceColor() {
        boolean isBlue = MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE;
        pattern = LEDPattern.solid(isBlue ? Color.BLUE : Color.RED);
        currentMode = Mode.PATTERN;
    }

    public void setColor(Color c) {
        pattern = LEDPattern.solid(c);
        currentMode = Mode.PATTERN;
    }

    public void setPattern(LEDPattern p) {
        pattern = p;
        currentMode = Mode.PATTERN;
    }

    public void setProgressSupplier(DoubleSupplier progress) {
        this.progress = progress;
        currentMode = Mode.PROGRESS;
    }

    private void renderProgressBar(DoubleSupplier progress) {
        Color bg = MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE ? Color.ALICE_BLUE
                : Color.DEEP_PINK;
        Color fg = MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE ? Color.BLUE : Color.RED;

        int width = 10;

        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, bg);
        }

        double percent = progress.getAsDouble();
        if (percent < 0) {
            percent = 0;
        }

        int barStart = (int) (percent * (side1.getLength() - width));
        int barEnd = barStart + width;
        for (int i = barStart; i < barEnd; i++) {
            side2.setLED(i, fg);
            side1.setLED(i, fg);
        }
    }

    @Override
    public void periodic() {
        if (currentMode == Mode.PATTERN) {
            pattern.applyTo(buffer);
        } else if (currentMode == Mode.PROGRESS) {
            renderProgressBar(progress);
        }
        led.setData(buffer);
    }
}
