package frc.team2412.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.Robot;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkFlexLowLevel.MotorType;
import com.revrobotics.CANSparkFlex.IdleMode;

public class ClimbSubsystem extends SubsystemBase{
    //public or private?
    //CONSTANT VARIABLES
        //speeds (fix later)
    private final double EXTEND_SPEED = 0.1;
    private final double RETRACT_SPEED = -0.1;
    //private final double MOTOR_SPEED = 0.15;
        //PID extension values
        //PID retraction values
        //height of chain in encoder??? units

    //  INSTANCE VARIABLES
    private final CANSparkFlex motorLeft;
    private final CANSparkFlex motorRight;
    private final SparkFlexAbsoluteEncoder encoderLeft;
    private final SparkFlexAbsoluteEncoder encoderRight;
    private final SparkPIDController motorLeftPIDController;
    private final SparkPIDController motorRightPIDController;

    //encoder position
    public ClimbSubsystem() {
        //motor = new MOTOR_NAME (port number found in hardware);
        this.motorLeft = new motor(CLIMB_MOTOR_ONE, MotorType.kBrushless);
        this.motorRight = new motor(CLIMB_MOTOR_TWO,MotorType.kBrushless );
        this.encoderLeft = new motorLeft.getAbsoluteEncoder(Type.kDutyCycle);
        this.encoderRight = new motorRight.getAbsoluteEncoder(Type.kDutyCycle);

        //PID
        this.motorLeftPIDController = motorLeft.getPIDController();
        this.motorRightPIDController = motorRight.getPIDController();
        motorLeftPIDController.setFeedbackDevice(encoderLeft);
        motorRightPIDController.setFeedbackDevice(encoderRight);

        //motor configuration
        //kBrake refers to motor idleness (completely stopped)
        motorLeft.setIdleMode(IdleMode.kBrake);
        motorRight.setIdleMode(IdleMode.kBrake);
        

    }
    
    //Methods used in rapid react climb subsystem--
    //start arm
    //stop arm
    //    stolen code omg

    //extend arm
    //retract arm
    //set motor position
    //get+set motor speed
    //stop arm
    public double getSpeed() {
        return motorLeft.get();
    }

    public void setMotorUp() {
        setMotor(EXTEND_SPEED);
    }
    
    public void setMotorDown() {
        setMotor(-RETRACT_SPEED);
    }
    
    public void setMotor(double speed) {
        motorLeft.set(speed);
        motorRight.set(speed);
    }

    public void stopArm() {
        setMotor(0);
    }

    //return encoder angle
    public double encoderLeftPosition() {
        return motorLeft.getPosition();
    }
    
    public double encoderRightPosition() {
        return motorRight.getPosition();
    }

    //calculate height climbed
    //reset encoder to 0 if reset == true
    public void resetEncoder(boolean reset) {
        if (reset) {
            motorLeft.setPositionConversionFactor(0);
            motorRight.setPositionConversionFactor(0);
        }
    }


    //changes the spring in climb
    //please change rotation variable later
    public void climb(double rotations) {
        motorLeftPIDController.setReference(rotations, ControlType.kPosition);
        motorRightPIDController.setReference(rotations, ControlType.kPosition);
    }
    //if exceeding limit (limit switch functions)
    //possibly print out encoder position for good measure
    /*
     * public void periodic(){
     * System.out.println(encoderPosition());
     * }
     */
}
