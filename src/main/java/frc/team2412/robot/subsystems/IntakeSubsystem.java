package frc.team2412.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team2412.robot.Hardware.*;

public class IntakeSubsystem extends SubsystemBase{
	public static final double INTAKE_IN_SPEED = 0.3;
	public static final double INTAKE_FAST_OUT_SPEED = -0.7;

    CANSparkMax intakeMotorBack;
    CANSparkMax intakeMotorRight;
    CANSparkMax intakeMotorFront;
    CANSparkMax intakeMotorLeft;

    CANSparkMax conveyorBeltOne;
    CANSparkMax conveyorBeltTwo;

    public IntakeSubsystem() {

        intakeMotorBack = new CANSparkMax(intakeMotorBackID, MotorType.kBrushless);
        intakeMotorRight = new CANSparkMax(intakeMotorRightID, MotorType.kBrushless);
        intakeMotorFront = new CANSparkMax(intakeMotorFrontID, MotorType.kBrushless);
        intakeMotorLeft = new CANSparkMax(intakeMotorLeftID, MotorType.kBrushless);

        conveyorBeltOne = new CANSparkMax(conveyorBeltOneID, MotorType.kBrushless);
        conveyorBeltTwo = new CANSparkMax(conveyorBeltTwoID, MotorType.kBrushless);

        intakeMotorBack.follow(intakeMotorFront);
        intakeMotorLeft.follow(intakeMotorFront);
        intakeMotorRight.follow(intakeMotorFront);

    }
    
    public void intake(double speed, CANSparkMax motor){
        motor.set(speed);
    }

    public void stopMotor(CANSparkMax motor){
        motor.set(0);
    }

    public void spitOut(){
        intakeMotorFront.set(INTAKE_FAST_OUT_SPEED);
    }

    
    
    /*
    public void takeIn(double speed, CANSparkMax motor, CANSparkMax motor2){
        motor.set(speed);
        motor2.set(speed); 
    }

    public void takeIn(double speed, CANSparkMax motor, CANSparkMax motor2, CANSparkMax motor3){
        motor.set(speed);
        motor2.set(speed);
        motor3.set(speed);
    }*/


}