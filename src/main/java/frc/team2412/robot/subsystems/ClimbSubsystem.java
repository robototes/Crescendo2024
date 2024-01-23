package frc.team2412.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.Robot;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class ClimbSubsystem extends SubsystemBase{
    //public or private?
    //  CONSTANT VARIABLES
        //speeds (fix later)
    private final double EXTEND_SPEED = 0.1;
    private final double RETRACT_SPEED = -0.1;
    private final double MOTOR_SPEED = 0.15;
        //PID extension values
        //PID retraction values
        //height of chain in encoder??? units

    //  INSTANCE VARIABLES 
    private final CANSparkMax motor1;
    private final CANSparkMax motor2;
    private final SparkMaxAbsoluteEncoder encoder;

    //encoder position
    public ClimbSubsystem() {
        //motor = new MOTOR_NAME (port number found in hardware);
        this.motor1 = new motor(CLIMB_MOTOR_ONE, MotorType.kBrushless);
        this.motor2= new motor(CLIMB_MOTOR_TWO,MotorType.kBrushless );
        this.encoder1 = new motor1.getAbsoluteEncoder(Type.kDutyCycle);
        this.encoder2 = new motor2.getAbsoluteEncoder(Type.kDutyCycle);


        //motor configuration
        //kBreak refers to motor idleness (completely stopped)
        motor1.setIdleMode(IdleMode.kBrake);
        motor2.setIdleMode(IdleMode.kBrake);
        //set PID

    }
    
    //Methods used in rapid react climb subsystem--
    //start arm
    //stop arm
    //    stolen code omg
    /*public void stopArm(boolean stop) {
        if (stop) {
            motor.stopMotor();
            motor2.stopMotor();
        } 
    */

    //extend arm
    //retract arm
    //set motor position
    //get+set motor speed
    public double getSpeed() {
        return motor1.get();
        return motor2.get();
    }

    public void setMotorUp() {
        setMotor(MOTOR_SPEED);
    }
    
    public void setMotorDown() {
        setMotor(-MOTOR_SPEED);
    }
    
    public void setMotor(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }

    //return encoder angle
    public double encoderPosition() {
        return motor1.getPosition();
        return motor2.getPosition();
    }

    //calculate height climbed
    //reset encoder to 0 if reset == true
    public void resetEncoder(boolean reset) {
        if (reset) {
            motor1.setPositionConversionFactor(0);
            motor2.setPositionConversionFactor(0);
        }
    }
    //if exceeding limit (limit switch functions)
    //possibly print out encoder position for good measure
    /*
     * public void periodic(){
     * System.out.println(encoderPosition());
     * }
     */
}
