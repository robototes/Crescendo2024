package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
	private static final double RED_LED_COLOR = 0.61;
    private static final double GREEN_LED_COLOR = 0.77;
    private static final double BLUE_LED_COLOR = 0.87;
    private static final double YELLOW_LED_COLOR = 0.69;

	private final PWM blinkin;

	public LEDSubsystem() {
		blinkin = new PWM(BLINKIN_LED);
		blinkin.setSpeed(RED_LED_COLOR); // sets to red
	}

	public void setLED(double color) {
		blinkin.setSpeed(color);
		System.out.println("set color " + color);
	}

}
