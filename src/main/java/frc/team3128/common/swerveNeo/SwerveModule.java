package frc.team3128.common.swerveNeo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax.EncoderType;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.common.swerveNeo.SwerveConversions.*;


/**
 * Team 3128's Swerve Module class
 * @since 2022 Rapid React
 * @author Mika Okamato, Mason Lam
 */
public class SwerveModule {
    public final int moduleNumber;
    private final double angleOffset;
    private final NAR_CANSparkMax angleMotor;
    private final NAR_CANSparkMax driveMotor;
    private final CANCoder angleEncoder;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

    private Rotation2d lastAngle;

    /**
     * Creates a new Swerve Module object
     * @param moduleNumber The module number from 0 - 3.
     * @param moduleConstants The constants for the Swerve module, ie. motor ids.
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new NAR_CANSparkMax(moduleConstants.angleMotorID, EncoderType.Relative, MotorType.kBrushless, SwerveConstants.angleKP, SwerveConstants.angleKI, SwerveConstants.angleKD);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new NAR_CANSparkMax(moduleConstants.driveMotorID, EncoderType.Relative, MotorType.kBrushless, SwerveConstants.driveKP, SwerveConstants.driveKI, SwerveConstants.driveKD);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    /**
     * Initializes the angle motor
     */
    private void configAngleMotor(){
        angleMotor.setSmartCurrentLimit(angleLimit);
        angleMotor.setInverted(angleMotorInvert);
        angleMotor.setIdleMode(IdleMode.kCoast);
        angleMotor.enableContinuousInput(-180, 180, degreesToRotations(1, angleGearRatio));
        angleMotor.setDefaultStatusFrames();
        resetToAbsolute();
    }

    /**
     * Intializes the drive motor
     */
    private void configDriveMotor(){        
        driveMotor.setSmartCurrentLimit(driveLimit);
        driveMotor.setInverted(driveMotorInvert);
        driveMotor.setIdleMode(IdleMode.kCoast); 
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.setDefaultStatusFrames();
    }

    /**
     * Changes the modules velocity and angular position to the desired state
     * @param desiredState The desired state with a velocity and angular component
     */
    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        setAngle(desiredState);
        setSpeed(desiredState);
    }

    /**
     * Changes the modules angular position to the desired state
     * @param desiredState The desired state with a velocity and angular component
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.025)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleMotor.set(degreesToRotations(angle.getDegrees(), angleGearRatio), ControlType.kPosition);
        lastAngle = angle;
    }

    /**
     * Changes the modules velocity to the desired state
     * @param desiredState The desired state with a velocity and angular component
     */
    private void setSpeed(SwerveModuleState desiredState) {
        double velocity = MPSToRPM(desiredState.speedMetersPerSecond, wheelCircumference, driveGearRatio);
        driveMotor.set(velocity, ControlType.kVelocity, feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    /**
     * Sets the module in its x-lock angle and sets the velocity to 0
     * @param angle The desired angle of the module
     */
    public void xLock(Rotation2d angle) {
        double desiredAngle = CTREModuleState.optimize(new SwerveModuleState(0, angle), getState().angle).angle.getDegrees();
        driveMotor.set(0, ControlType.kVelocity);
        angleMotor.set(degreesToRotations(desiredAngle, angleGearRatio), ControlType.kPosition); 
    }

    /**
     * Resets the angle motor to the CANCoder position
     */
    public void resetToAbsolute(){
        double absolutePosition = degreesToRotations(getCanCoder().getDegrees(), angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    /**
     * Returns the current angle of the CANCoder
     */
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(MathUtil.inputModulus(angleEncoder.getAbsolutePosition() - angleOffset, -180, 180));
    }

    /**
     * Returns the angular position of the swerve module
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(rotationsToDegrees(angleMotor.getSelectedSensorPosition(), angleGearRatio));
    }

    /**
     * Returns the Swerve module's state consisting of velocity and angular position
     * @return A swerve module state
     */
    public SwerveModuleState getState(){
        double velocity = RPMToMPS(driveMotor.getSelectedSensorVelocity(), wheelCircumference, driveGearRatio);
        Rotation2d angle = getAngle();
        return new SwerveModuleState(velocity, angle);
    }

    /**
     * Returns the Swerve module's position containing the drive and angular position
     * @return A swerve module position
     */
    public SwerveModulePosition getPosition() {
        double position = rotationsToMeters(driveMotor.getSelectedSensorPosition(), wheelCircumference, driveGearRatio);
        Rotation2d angle = getAngle();
        return new SwerveModulePosition(position, angle);
    }

    /**
     * Stops the Swerve Module from moving
     */
    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public void setBrakeMode(boolean isBrake) {
        driveMotor.setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    /**
     * Configures the CANCoder
     */
    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(CTREConfigs.swerveCancoderConfig());
    }
    
}