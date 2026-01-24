package frc.lib.leds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        setColor(Color.kBlack);
    }

    public void setAllianceColor() {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        pattern = LEDPattern.solid(isBlue ? Color.kBlue : Color.kRed);
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
        Color bg = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Color.kAliceBlue
                : Color.kDeepPink;
        Color fg = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Color.kBlue : Color.kRed;

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
