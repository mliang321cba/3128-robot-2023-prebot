package frc.team3128.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.AutoConstants.*;

import frc.team3128.Constants.SwerveConstants;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Swerve;

public class CmdAutoBalance extends CommandBase{
    private final Swerve swerve;
    private final int direction;
    private double prevAngle;
    private double angleVelocity;
    private int plateauCount;
    private boolean onRamp;

    public CmdAutoBalance(boolean isForward) {
        swerve = Swerve.getInstance();
        this.direction = isForward ? 1 : -1;
        addRequirements(Swerve.getInstance());
    }
    
    @Override
    public void initialize() {
        swerve.zeroAxis();
        prevAngle = swerve.getPitch();
        onRamp = false;
        plateauCount = 0;
        angleVelocity = 0;
    }

    public double getAngleVelocity(double currAngle) {
        if (plateauCount < 5) {
            plateauCount ++;
            return angleVelocity;
        }
        plateauCount = 0;
        angleVelocity = (currAngle - prevAngle) / 0.1;
        prevAngle = currAngle;
        return angleVelocity;
    }

    @Override
    public void execute() {
        final double advAngle = swerve.getPitch();
        // final double angleVelocity = (advAngle - prevAngle) / 0.02;
        final double angleVelocity = getAngleVelocity(advAngle);

        if (Math.abs(advAngle) > RAMP_THRESHOLD) onRamp = true;

        if (Math.abs(advAngle) < ANGLE_THRESHOLD && onRamp) {
            swerve.xlock();
            if (direction == 1) {
                // Manipulator.getInstance().set(-1);
            }
            return;
        }

        NAR_Shuffleboard.addData("Drivetrain", "Angle Velocity", angleVelocity, 6, 2);
        if (((advAngle < 0.0 && angleVelocity > VELOCITY_THRESHOLD) || (advAngle > 0.0 && angleVelocity < -VELOCITY_THRESHOLD)) && onRamp) {
            swerve.xlock();
            return;
        }

        swerve.drive(new Translation2d(onRamp ? DRIVE_SPEED * (advAngle > 0.0 ? 1.0 : -1.0) : SwerveConstants.maxSpeed * direction, 0), 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.xlock();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
