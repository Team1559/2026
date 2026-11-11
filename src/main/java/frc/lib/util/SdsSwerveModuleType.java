package frc.lib.util;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

import com.ctre.phoenix6.signals.InvertedValue;

    public enum SdsSwerveModuleType {
        MK4_L1(50d / 14 * 19 / 25 * 45 / 15, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive),
        MK4_L2(50d / 14 * 17 / 27 * 45 / 15, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive),
        MK4_L3(50d / 14 * 16 / 28 * 45 / 15, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive),
        MK4_L4(48d / 16 * 16 / 28 * 45 / 15, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive),

        MK4I_L1(50d / 14 * 19 / 25 * 45 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.Clockwise_Positive),
        MK4I_L2(50d / 14 * 17 / 27 * 45 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.Clockwise_Positive),
        MK4I_L3(50d / 14 * 16 / 28 * 45 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.Clockwise_Positive),

        MK5_R1(54d / 12 * 25 / 32 * 30 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.CounterClockwise_Positive),
        MK5_R2(54d / 14 * 25 / 32 * 30 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.CounterClockwise_Positive),
        MK5_R3(54d / 16 * 25 / 32 * 30 / 15, InvertedValue.CounterClockwise_Positive,
                InvertedValue.CounterClockwise_Positive);

        public final double driveRatio;
        public final InvertedValue driveDirection;
        public final InvertedValue steerDirection;
        public static final Distance WHEEL_RADIUS = Inches.of(2.0);

        SdsSwerveModuleType(double driveRatio, InvertedValue driveDirection, InvertedValue steerDirection) {
            this.driveRatio = driveRatio;
            this.driveDirection = driveDirection;
            this.steerDirection = steerDirection;
        }
    }
