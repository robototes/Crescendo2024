package frc.team2412.robot;

public class Hardware {
	// apriltags stuff
	public static final String PHOTON_CAM = "Arducam_OV9281_USB_Camera";
	public static final String PHOTON_IP = "10.24.12.12";

	public static final int PDP_ID = 1;
	// motor IDs:

	// drive devices [1 - 19]

	// cameras

	// climb [20 - 29]

	// launcher [30 - 39]
	public static final int LAUNCHER_TOP_MOTOR_ID = 30;
	public static final int LAUNCHER_BOTTOM_MOTOR_ID = 31;
	public static final int LAUNCHER_PIVOT_ONE_MOTOR_ID = 33;
	public static final int LAUNCHER_PIVOT_TWO_MOTOR_ID = 32;

	// intake [40 - 49]
	public static final int INTAKE_MOTOR_FRONT = 40;
	public static final int INTAKE_MOTOR_BACK = 41;
	public static final int INTAKE_MOTOR_LEFT = 42;
	public static final int INTAKE_MOTOR_RIGHT = 43;

	public static final int INGEST_MOTOR = 44;

	public static final int INDEX_MOTOR_UPPER = 45;

	public static final int FEEDER_MOTOR = 46;
	// LED strip is PWM port 8
	public static final int BLINKIN_LED = 8;

	// intake sensors  (Digital IO)
	public static final int INDEX_SENSOR = 1;
	public static final int FEEDER_SENSOR = 2;
}
