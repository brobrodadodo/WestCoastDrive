package org.firstinspires.ftc.teamcode.WestCoastDrive;

import androidx.core.util.Supplier;

import me.wobblyyyy.pathfinder2.kinematics.EncoderConverter;

public class SpeedTracker {
    private final EncoderConverter converter;
    private final Supplier<Integer> getTicks;
    private int lastTicks = 0;
    private long lastMs = 0;

    public SpeedTracker(EncoderConverter converter, Supplier<Integer> getTicks) {
        this.converter = converter;
        this.getTicks = getTicks;
    }

    public double getSpeedWithTime(double elapsedSeconds) {
        int currentTicks = getTicks.get();
        int elapsedTicks = currentTicks - lastTicks;
        lastTicks = currentTicks;
        double elapsedDistance = converter.distanceFromTicks(elapsedTicks);
        return elapsedDistance / elapsedSeconds;
    }

    /**
     * Get the speed of the encoder.
     *
     * @return the speed of the encoder.
     */
    public double getSpeed() {
        long currentMs = System.currentTimeMillis();
        if (lastMs == 0) lastMs = currentMs;
        long elapsedMs = currentMs - lastMs;
        lastMs = currentMs;
        double elapsedSeconds = elapsedMs / 1_000d;
        return getSpeedWithTime(elapsedSeconds);
    }
}
