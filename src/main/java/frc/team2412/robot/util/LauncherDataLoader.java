package frc.team2412.robot.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class LauncherDataLoader {

    private static final String CSV_DATA_PATH = "launcher_data.csv";

    public InterpolatingTreeMap<Double, LauncherDataPoint> fromCSV() {
        InterpolatingTreeMap<Double, LauncherDataPoint> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LauncherDataPoint.INTERPOLATOR);
    }

}
