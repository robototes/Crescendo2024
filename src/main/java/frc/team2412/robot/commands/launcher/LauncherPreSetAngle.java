package frc.team2412.robot.commands.launcher;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class LauncherPreSetAngle extends Command{
    private final LauncherSubsystem launcherSubsystem;
    private final double launcherAngle;

    public LauncherPreSetAngle(LauncherSubsystem launcherSubsystem, double angle){
        this.launcherSubsystem = launcherSubsystem;
        this.launcherAngle = angle;
        addRequirements(launcherSubsystem);
    }


    @Override
    public void initialize(){
        launcherSubsystem.setAngle(launcherAngle);
    }

    @Override
    public boolean isFinished(){
        return (MathUtil.isNear(launcherAngle, launcherSubsystem.getAngle(), LauncherSubsystem.ANGLE_TOLERANCE));
    }
}
