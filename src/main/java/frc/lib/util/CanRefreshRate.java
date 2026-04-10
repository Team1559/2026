package frc.lib.util;

public enum CanRefreshRate {
    SLOW(10), DEFAULT(100), FAST(500);

    public final int rateHz;

    CanRefreshRate(int rateHz) {
        this.rateHz = rateHz;
    }
}
