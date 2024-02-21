package frc.team2412.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.Filesystem;
import frc.team2412.robot.Robot;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.function.Consumer;

public class LauncherDataLoader {

	public static InterpolatingTreeMap<Double, LauncherDataPoint> fromCSV(Path path) {
		InterpolatingTreeMap<Double, LauncherDataPoint> map =
				new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LauncherDataPoint.INTERPOLATOR);

		try (BufferedReader reader = Files.newBufferedReader(path);
				BufferedWriter debugWriter =
						Files.newBufferedWriter(
								FileSystems.getDefault()
										.getPath(
												Filesystem.getOperatingDirectory().getPath(),
												Robot.isReal() ? "logs/CSVInterpreter.log" : "CSVInterpreter.log"),
								StandardOpenOption.CREATE,
								StandardOpenOption.WRITE); ) {

			String line;
			int lineNumber = 0;
			while ((line = reader.readLine()) != null) {
				lineNumber++;
				String debugPrefix = "Line #" + lineNumber + ": ";
				Consumer<String> debug =
						(msg) -> {
							try {
								debugWriter.append(debugPrefix + msg);
								debugWriter.newLine();
							} catch (IOException e) {
								e.printStackTrace();
							}
						};

				if (line.length() <= 0) {
					debug.accept("Empty line, skipping");
					continue;
				}

				int hashtagIndex = line.indexOf('#');
				int doubleSlashIndex = line.indexOf("//");

				if (hashtagIndex == 0) {
					debug.accept("Starts with '#', skipping line");
					continue;
				}
				if (doubleSlashIndex == 0) {
					debug.accept("Starts with '//', skipping line");
					continue;
				}
				if (hashtagIndex != -1 && (doubleSlashIndex == -1 || hashtagIndex < doubleSlashIndex)) {
					debug.accept("'#' at char index " + hashtagIndex + ", trimming comment");
					line = line.substring(0, hashtagIndex);
				}
				if (doubleSlashIndex != -1 && (hashtagIndex == -1 || doubleSlashIndex < hashtagIndex)) {
					debug.accept("'//' at char index " + doubleSlashIndex + ", trimming comment");
					line = line.substring(0, doubleSlashIndex);
				}

				String[] items = line.split(",", -1);
				if (items.length < 3) {
					debug.accept("Less than 3 items, skipping line");
					continue;
				} else if (items.length > 3) {
					debug.accept("More than 3 items, ignoring extra items");
				}

				double distance, angle, RPM;

				try {
					distance = Double.parseDouble(items[0]);
					angle = Double.parseDouble(items[1]);
					RPM = Double.parseDouble(items[2]);

					debug.accept(
							"Identified data! DISTANCE: " + distance + " ANGLE: " + angle + " RPM " + RPM);
				} catch (NumberFormatException e) {
					debug.accept("Non-numerical value, skipping line");
					continue;
				}

				if (distance < 0) {
					debug.accept("Distance " + distance + " is negative, skipping line");
					continue;
				}
				if (RPM < 0) {
					debug.accept("Flywheel RPM " + RPM + " is negative, skipping line");
					continue;
				}

				map.put(distance, new LauncherDataPoint(angle, RPM));
			}

			debugWriter.newLine();
			debugWriter.append("Done serializing CSV");
			debugWriter.newLine();
			debugWriter.close();
			System.out.println("Done serializing CSV");

			return map;
		} catch (IOException e) {
			e.printStackTrace();
			map.put(
					0.,
					new LauncherDataPoint(
							70, 2000)); // TODO: figure out the subwoofer data points and enter them here
			return map;
		}
	}
}
