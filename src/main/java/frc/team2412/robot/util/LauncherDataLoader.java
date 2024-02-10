package frc.team2412.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class LauncherDataLoader {

    private static final String CSV_DATA_PATH = "launcher_data.csv";

    public InterpolatingTreeMap<Double, LauncherDataPoint> fromCSV() {
        InterpolatingTreeMap<Double, LauncherDataPoint> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LauncherDataPoint.INTERPOLATOR);

        // TODO: make this actually use the csv file

        map.put(1.0, new LauncherDataPoint(0, 0));
        map.put(10.0, new LauncherDataPoint(1.0, 2.0));

        return map;
    }

}
