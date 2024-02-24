package frc.team2412.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class PathPlannerAutos {
	// Parsing utils

	private static int indexOfEnd(String text, String target, int fromIndex) {
		int i = text.indexOf(target, fromIndex);
		if (i < 0) {
			return i;
		}
		return i + target.length();
	}

	private static int skipWhitespace(String text, int i) {
		while (Character.isWhitespace(text.charAt(i))) {
			++i;
		}
		return i;
	}

	private static String readJsonString(String text, int quoteStart) {
		int stringStart = quoteStart + 1;
		int stringEnd = text.indexOf("\"", stringStart);
		return text.substring(stringStart, stringEnd);
	}

	private static String readJsonValue(String text, int start) {
		start = skipWhitespace(text, start);
		int comma = text.indexOf(",", start);
		int objectEnd = text.indexOf("}", start);
		if (comma < 0) {
			return text.substring(start, objectEnd);
		}
		if (objectEnd < 0) {
			return text.substring(start, comma);
		}
		return text.substring(start, Math.min(comma, objectEnd));
	}

	private static boolean readJsonBoolean(String text, int start) {
		// False for values that are not "true" or "false" (case insensitive)
		return Boolean.parseBoolean(readJsonValue(text, start));
	}

	private static double readJsonDouble(String text, int start) throws NumberFormatException {
		return Double.parseDouble(readJsonValue(text, start));
	}

	// Individual paths

	private static PathPlannerPath loadPathPlannerPath(String name) {
		return PathPlannerPath.fromPathFile(name);
	}

	private static PathPlannerPath loadChoreoTrajectory(String name) {
		return PathPlannerPath.fromChoreoTrajectory(name);
	}

	private static final Map<String, PathPlannerPath> pathPlannerPathCache = new HashMap<>();
	private static final Map<String, PathPlannerPath> choreoTrajectoryCache = new HashMap<>();

	public static PathPlannerPath getPathPlannerPath(String name) {
		return pathPlannerPathCache.computeIfAbsent(name, PathPlannerAutos::loadPathPlannerPath);
	}

	public static PathPlannerPath getChoreoTrajectory(String name) {
		return choreoTrajectoryCache.computeIfAbsent(name, PathPlannerAutos::loadChoreoTrajectory);
	}

	// Whole autos

	private static boolean isChoreoAuto(String autoJson) {
		int boolStart = skipWhitespace(autoJson, indexOfEnd(autoJson, "\"choreoAuto\":", -1));
		return readJsonBoolean(autoJson, boolStart);
	}

	private static int getNextPathNameIndex(String autoJson, int start) {
		while (true) {
			// Find next occurrence of "type":
			start = indexOfEnd(autoJson, "\"type\":", start);
			if (start < 0) {
				break;
			}
			int i = skipWhitespace(autoJson, start);
			// Skip the command if it's not a path command
			if (!autoJson.startsWith("\"path\"", i)) {
				continue;
			}
			// Find start of the JSON object representing the command
			int objectStart = autoJson.lastIndexOf("{", start);
			// Find pathName within the command
			int pathNameIndex = indexOfEnd(autoJson, "\"pathName\":", objectStart);
			// Skip whitespace
			pathNameIndex = skipWhitespace(autoJson, pathNameIndex);
			// Double check we have a string
			if (autoJson.charAt(pathNameIndex) != '"') {
				continue;
			}
			// Done
			return pathNameIndex;
		}
		return -1;
	}

	private static List<PathPlannerPath> getPaths(String autoJson) {
		List<PathPlannerPath> paths = new ArrayList<>();
		boolean isChoreo = isChoreoAuto(autoJson);
		int start = -1;
		while (true) {
			int pathNameStart = getNextPathNameIndex(autoJson, start);
			start = pathNameStart;
			if (pathNameStart < 0) {
				break;
			}
			String pathName = readJsonString(autoJson, pathNameStart);
			if (isChoreo) {
				paths.add(getChoreoTrajectory(pathName));
			} else {
				paths.add(getPathPlannerPath(pathName));
			}
		}
		return paths;
	}

	private static Rotation2d getStartingRotation(String autoJson) {
		int startingPoseIndex = indexOfEnd(autoJson, "\"startingPose\":", -1);
		if (startingPoseIndex < 0) {
			return null;
		}
		int poseObjectStart = skipWhitespace(autoJson, startingPoseIndex);
		if (!autoJson.startsWith("{", poseObjectStart)) {
			return null;
		}
		int rotationStart = indexOfEnd(autoJson, "\"rotation\":", poseObjectStart);
		if (rotationStart < 0) {
			return null;
		}
		double rotation;
		try {
			rotation = readJsonDouble(autoJson, skipWhitespace(autoJson, rotationStart));
		} catch (NumberFormatException e) {
			return null;
		}
		return Rotation2d.fromDegrees(rotation);
	}

	private static ChassisSpeeds speedsFromState(PathPlannerTrajectory.State state) {
		return new ChassisSpeeds(
				state.heading.getCos() * state.velocityMps,
				state.heading.getSin() * state.velocityMps,
				state.headingAngularVelocityRps);
	}

	private static List<PathPlannerTrajectory> loadAutoTrajectories(String autoName) {
		File autoFile =
				new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto");
		if (!autoFile.exists()) {
			DriverStation.reportWarning(
					"Attempted to load non-existent auto \"" + autoName + "\"", false);
			return List.of();
		}
		String text;
		try {
			text = Files.readString(autoFile.toPath());
		} catch (IOException e) {
			DriverStation.reportWarning("Could not load auto \"" + autoName + "\"", e.getStackTrace());
			return List.of();
		}
		List<PathPlannerPath> paths = getPaths(text);
		Rotation2d startingRotation = getStartingRotation(text);
		ChassisSpeeds startingSpeeds = new ChassisSpeeds();
		List<PathPlannerTrajectory> trajectories = new ArrayList<>(paths.size());
		for (var path : paths) {
			PathPlannerTrajectory trajectory = path.getTrajectory(startingSpeeds, startingRotation);
			trajectories.add(trajectory);
			startingRotation = trajectory.getEndState().targetHolonomicRotation;
			startingSpeeds = speedsFromState(trajectory.getEndState());
		}
		return List.copyOf(trajectories);
	}

	private static final Map<String, List<PathPlannerTrajectory>> autoTrajectoriesCache =
			new HashMap<>();

	public static List<PathPlannerTrajectory> getAutoTrajectories(String autoName) {
		return autoTrajectoriesCache.computeIfAbsent(autoName, PathPlannerAutos::loadAutoTrajectories);
	}

	// Pre-loading everything

	private static List<String> listFileNames(String deploySubdirectory, String extension) {
		File[] files = new File(Filesystem.getDeployDirectory(), deploySubdirectory).listFiles();

		if (files == null) {
			return List.of();
		}

		List<String> filenames = new ArrayList<>(files.length);
		for (File file : files) {
			if (!file.isDirectory()) {
				String name = file.getName();
				if (name.endsWith(extension)) {
					filenames.add(name.substring(0, name.lastIndexOf(".")));
				}
			}
		}
		return filenames;
	}

	private static <T> void loadAll(
			String deploySubdirectory,
			String extension,
			Map<String, T> cache,
			Function<String, T> loader) {
		List<String> names = listFileNames(deploySubdirectory, extension);
		for (String name : names) {
			cache.put(name, loader.apply(name));
		}
	}

	public static void loadAllPathPlannerPaths() {
		loadAll(
				"pathplanner/paths", ".path", pathPlannerPathCache, PathPlannerAutos::loadPathPlannerPath);
	}

	public static void loadAllChoreoTrajectories() {
		loadAll("choreo", ".traj", choreoTrajectoryCache, PathPlannerAutos::loadChoreoTrajectory);
	}

	public static void loadAllAutoTrajectories() {
		loadAll(
				"pathplanner/autos",
				".auto",
				autoTrajectoriesCache,
				PathPlannerAutos::loadAutoTrajectories);
	}
}
