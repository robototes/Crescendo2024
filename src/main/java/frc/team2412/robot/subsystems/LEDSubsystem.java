package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
	private static final int RED_LED_COLOR = 1805;
	private static final int GREEN_LED_COLOR = 1885;
	private static final int BLUE_LED_COLOR = 1935;
	private static final int YELLOW_LED_COLOR = 1845;
	private static final int VIOLET_LED_COLOR = 1955;

	private final PWM blinkin;

	public LEDSubsystem() {
		blinkin = new PWM(BLINKIN_LED);
		blinkin.setPulseTimeMicroseconds(RED_LED_COLOR); // sets to red
	}

	private void setLED(int color) {
		blinkin.setPulseTimeMicroseconds(color);
	}

	public void disableLED() {
		blinkin.setDisabled();
	}

	public void setRED_LED() {
		setLED(RED_LED_COLOR);
	}

	public void setGREEN_LED() {
		setLED(GREEN_LED_COLOR);
	}

	public void setBLUE_LED() {
		setLED(BLUE_LED_COLOR);
	}

	public void setYELLOW_LED() {
		setLED(YELLOW_LED_COLOR);
	}

	public void setVIOLET_LED() {
		setLED(VIOLET_LED_COLOR);
	}
}
