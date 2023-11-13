package frc.team3128.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.SwerveConstants.Mod0;
import frc.team3128.Constants.SwerveConstants.Mod1;
import frc.team3128.Constants.SwerveConstants.Mod2;
import frc.team3128.Constants.SwerveConstants.Mod3;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.commands.CmdManager;
import frc.team3128.common.swerveNeo.SwerveModule;
import frc.team3128.common.utility.NAR_Shuffleboard;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;

public class Swerve extends SubsystemBase {

    private static Swerve instance;

    private SwerveDrivePoseEstimator odometry;
    private SwerveModule[] modules;
    private WPI_Pigeon2 gyro;
    private Pose2d estimatedPose;

    private Field2d field;

    public boolean fieldRelative;
    public double throttle = 1;
    public double speed = 0;
    private Translation2d prevTrans = new Translation2d();
    public double acceleration = 0;
    private double prevSpeed = 0;

    private double initialRoll, initialPitch;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public Swerve() {
        gyro = new WPI_Pigeon2(pigeonID);
        gyro.configFactoryDefault();
        fieldRelative = true;
        estimatedPose = new Pose2d();
        zeroAxis();
        prevTrans = new Translation2d();

        modules = new SwerveModule[] {
            new SwerveModule(0, Mod0.constants),
            new SwerveModule(1, Mod1.constants),
            new SwerveModule(2, Mod2.constants),
            new SwerveModule(3, Mod3.constants)
        };

        Timer.delay(1.5);
        resetEncoders();

        odometry = new SwerveDrivePoseEstimator(swerveKinematics, getGyroRotation2d(), getPositions(), 
                                                estimatedPose, SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD);


        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getGyroRotation2d())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        setModuleStates(moduleStates);
    }

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public void setBrakeMode(boolean isBrake) {
        for (final SwerveModule module : modules) {
            module.setBrakeMode(isBrake);
        }
    }

    public void initShuffleboard() {
        // General Tab
        NAR_Shuffleboard.addComplex("General","Gyro",gyro,7,2,2,2);//.withWidget("Gyro");
        NAR_Shuffleboard.addData("General","Heading",this::getHeading,1,2);
        // // Drivetrain Tab
        NAR_Shuffleboard.addData("Drivetrain","Pose",() -> (getPose().toString()),2,0,4,1);
        NAR_Shuffleboard.addComplex("Drivetrain","Gyro",gyro,3,1,2,2);//.withWidget("Gyro");
        NAR_Shuffleboard.addData("Drivetrain","Yaw",this::getYaw,4,1);
        NAR_Shuffleboard.addData("Drivetrain","Pitch",this::getPitch,5,1);
        NAR_Shuffleboard.addData("Drivetrain", "Roll", this::getRoll, 0, 2);
        NAR_Shuffleboard.addData("Drivetrain","Heading/Angle",this::getHeading,6,1);
        NAR_Shuffleboard.addComplex("Drivetrain","Drivetrain", this,0,0);
        NAR_Shuffleboard.addData("Drivetrain", "ENABLE", ()-> CmdManager.ENABLE, 0, 1);
        NAR_Shuffleboard.addData("Drivetrain", "Single Station", ()-> CmdManager.SINGLE_STATION, 0, 3);
        NAR_Shuffleboard.addData("Drivetrain", "Speed", ()-> speed, 2, 1);
        NAR_Shuffleboard.addData("Drivetrain", "Acceleration", ()-> acceleration, 3, 1);
    }

    public Pose2d getPose() {
        return new Pose2d(estimatedPose.getTranslation(), getGyroRotation2d());
    }

    public void addVisionMeasurement(Pose2d pose, double timeStamp) {
        if (Math.abs(pose.getX() - getPose().getX()) > VisionConstants.POSE_THRESH || 
            Math.abs(pose.getY() - getPose().getY()) > VisionConstants.POSE_THRESH) return;
        odometry.addVisionMeasurement(pose, timeStamp);
    }

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }
    }

    public void resetOdometry(Pose2d pose) {
        zeroGyro(pose.getRotation().getDegrees());
        odometry.resetPosition(getGyroRotation2d(), getPositions(), pose);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }
    
    public void toggle() {
        fieldRelative = !fieldRelative;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for (SwerveModule module : modules){
            module.setDesiredState(desiredStates[module.moduleNumber]);
        }
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getPositions());
        estimatedPose = odometry.getEstimatedPosition();
        for (SwerveModule module : modules) {
            SmartDashboard.putNumber("module " + module.moduleNumber, module.getCanCoder().getDegrees());
        }
        updateSpeed();
        updateAcceleration();
    }

    public void updateSpeed() {
        Translation2d translation = getPose().getTranslation();
        speed = translation.getDistance(prevTrans) / 0.02;
        prevTrans = translation;
    }

    public void updateAcceleration(){
        acceleration = speed - prevSpeed / 0.02;
        prevSpeed = speed;
    }

    public void resetAll() {
        resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
        resetEncoders();
    }
    
    //DON't USE RELIES ON APRIL TAG BAD ANGLE MEASUREMENT
    public Rotation2d getRotation2d() {
        return estimatedPose.getRotation();
    }

    public void xlock() {
        modules[0].xLock(Rotation2d.fromDegrees(45));
        modules[1].xLock(Rotation2d.fromDegrees(-45));
        modules[2].xLock(Rotation2d.fromDegrees(-45));
        modules[3].xLock(Rotation2d.fromDegrees(45));
    }

    public double getYaw() {
        return MathUtil.inputModulus(gyro.getYaw(),-180,180);
    }

    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    //DONT USE THIS METHOD, it relies on the bad april tag angle measurements
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public double getPitch() {
        return gyro.getPitch() - initialPitch;
    }

    public double getRoll() {
        return gyro.getRoll() - initialRoll;
    }

    public void zeroGyro() {
        gyro.reset();
        zeroAxis();
    }

    public void zeroAxis() {
        initialRoll = gyro.getRoll();
        initialPitch = gyro.getPitch();
    }

    public void zeroGyro(double reset) {
        gyro.setYaw(reset);
    }

    public boolean compare(SwerveModuleState measured, SwerveModuleState theoretical) {
        return (Math.abs(measured.speedMetersPerSecond - theoretical.speedMetersPerSecond)/ theoretical.speedMetersPerSecond) < 0.05 
        && (Math.abs(measured.angle.getDegrees() - theoretical.angle.getDegrees())/ theoretical.angle.getDegrees()) < 0.05;
      }
}