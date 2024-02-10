package frc.team2412.robot.util;

import edu.wpi.first.math.interpolation.Interpolator;

public class LauncherDataPoint {

    public static final Interpolator<LauncherDataPoint> INTERPOLATOR = new Interpolator<LauncherDataPoint>() {
        @Override
        public LauncherDataPoint interpolate(LauncherDataPoint startValue, LauncherDataPoint endValue, double t) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'interpolate'");
        }
    };

    public final double distance;
    public final double rpm;

    public LauncherDataPoint(double distance, double rpm) {
        this.distance = distance;
        this.rpm = rpm;
    }
    
}
