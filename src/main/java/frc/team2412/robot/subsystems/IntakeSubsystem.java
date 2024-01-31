package frc.team2412.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team2412.robot.Hardware.*;

public class IntakeSubsystem extends SubsystemBase {
	public static final double INTAKE_IN_SPEED = 0.3;
	public static final double INGEST_SPEED = 0.2;
    public static final double INDEX_SPEEED = 0.2;
    public static final double FEEDER = 0.25; //YUMNAH please initialize THIS
    public static final double DEEPS = 2.0;


    //'skirt' intake
    private final CANSparkFlex intakeMotor1;
    private final CANSparkFlex intakeMotor2;
    private final CANSparkFlex intakeMotor3;
    private final CANSparkFlex intakeMotor4;

    private final CANSparkFlex injestMotor1;
    private final CANSparkFlex injestMotor2;
    private final CANSparkFlex injestMotor3;

    public IntakeSubsystem() {

        intakeMotor1 = new CANSparkFlex(Hardware.INTAKE_1, MotorType.kBrushless);
        intakeMotor2 = new CANSparkFlex(Hardware.INTAKE_2, MotorType.kBrushless);
        intakeMotor3 = new CANSparkFlex(Hardware.INTAKE_3, MotorType.kBrushless);
        intakeMotor4 = new CANSparkFlex(Hardware.INTAKE_4, MotorType.kBrushless);

        resetMotors();

        injestMotor1 = new CANSparkFlex(Hardware.INJEST_1, MotorType.kBrushless);
        injestMotor2 = new CANSparkFlex(Hardware.INJEST_2, MotorType.kBrushless);
        injestMotor3 = new CANSparkFlex(Hardware.INJEST_3, MotorType.kBrushless);

        intakeMotor2.follow(intakeMotor1);
        intakeMotor3.follow(intakeMotor1);
        intakeMotor4.follow(intakeMotor1);
    }
    
    public void resetMotors() {
        intakeMotor1.restoreFactoryDefaults();
        intakeMotor2.restoreFactoryDefaults();
        intakeMotor3.restoreFactoryDefaults();
        intakeMotor4.restoreFactoryDefaults();

        injestMotor1.restoreFactoryDefaults();
        injestMotor2.restoreFactoryDefaults();
        injestMotor3.restoreFactoryDefaults();

        intakeMotor1.setIdleMode(IdleMode.kBrake);
        intakeMotor2.setIdleMode(IdleMode.kBrake);
        intakeMotor3.setIdleMode(IdleMode.kBrake);
        intakeMotor4.setIdleMode(IdleMode.kBrake);

        injestMotor1.setIdleMode(IdleMode.kBrake);
        injestMotor2.setIdleMode(IdleMode.kBrake);
        injestMotor3.setIdleMode(IdleMode.kBrake);

        intakeMotor1.setInverted(true);
        intakeMotor2.setInverted(true);
        intakeMotor3.setInverted(true);
        intakeMotor4.setInverted(true);

        intakeMotor1.setInverted(true);
        intakeMotor2.setInverted(true);
        intakeMotor3.setInverted(true);

        intakeMotor1.setSmartCurrentLimit(20);
        intakeMotor2.setSmartCurrentLimit(20);
        intakeMotor3.setSmartCurrentLimit(20);
        intakeMotor4.setSmartCurrentLimit(20);

        injestMotor1.setSmartCurrentLimit(20);
        injestMotor2.setSmartCurrentLimit(20);
        injestMotor3.setSmartCurrentLimit(20);

        intakeMotor1.burnFlash();
        intakeMotor2.burnFlash();
        intakeMotor3.burnFlash();
        intakeMotor4.burnFlash();

        injestMotor1.burnFlash();
        injestMotor2.burnFlash();
        injestMotor3.burnFlash();

    }

    public void intake() {
        intakeMotor1.set(INTAKE_IN_SPEED);
    }

    void stopMotor() {
        intakeMotor1.set(0);
    }

    public void spitOut() {
        intakeMotor1.set(INTAKE_FAST_OUT_SPEED);
    }

}