package frc.team3128;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.common.hardware.camera.Camera;

import frc.team3128.common.swerveNeo.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;


public class Constants {

    public static class TrajectoryConstants {
        public static final Rotation2d HEADING = Rotation2d.fromDegrees(180);
        
        public static final Translation2d POINT_1 = new Translation2d(12.7, 6.75);
        public static final double CONDITION_1 = 12.7;

        public static final Translation2d POINT_2A = new Translation2d(4.85, 0.8);
        public static final Translation2d POINT_2B = new Translation2d(4.85, 4.7);
        public static final double CONDITION_2 = 4.85;

        public static final Translation2d POINT_3A = new Translation2d(2.5, 0.8);
        public static final Translation2d POINT_3B = new Translation2d(2.5, 4.7);
        public static final double CONDITION_3 = 2.5;

        public static final Pose2d[] END_POINTS = new Pose2d[]{
            new Pose2d(1.75,0.5,Rotation2d.fromDegrees(180)),
            new Pose2d(1.75,0.95,Rotation2d.fromDegrees(180)),
            new Pose2d(1.75,1.55,Rotation2d.fromDegrees(180)),
            new Pose2d(1.75,2.05,Rotation2d.fromDegrees(180)),
            new Pose2d(1.75,2.65,Rotation2d.fromDegrees(180)),
            new Pose2d(1.75,3.2,Rotation2d.fromDegrees(180)),
            new Pose2d(1.75,3.75,Rotation2d.fromDegrees(180)),
            new Pose2d(1.75,4.35,Rotation2d.fromDegrees(180)),
            new Pose2d(1.75,4.79,Rotation2d.fromDegrees(180))
        };
    }

    public static class AutoConstants {

        public static final double slowSpeed = 1.5;
        public static final double slowAcceleration = 2;

        public static final PathConstraints fast = new PathConstraints(SwerveConstants.maxSpeed, SwerveConstants.maxAcceleration); 
        public static final PathConstraints slow = new PathConstraints(slowSpeed, slowAcceleration);

        /* Translation PID Values */
        public static final double translationKP = 3;
        public static final double translationKI = 0;
        public static final double translationKD = 0;
      
        /* Rotation PID Values */
        public static final double rotationKP = 2;
        public static final double rotationKI = 0;
        public static final double rotationKD = 0;

        public static final double ANGLE_THRESHOLD = 8; //7, 9
        public static final double VELOCITY_THRESHOLD = 4; //6, 3
        public static final double RAMP_THRESHOLD = 9; //8, 10
        public static final double DRIVE_SPEED = Units.inchesToMeters(30); //30, 40

    }

    public static class SwerveConstants {
        public static final int pigeonID = 30; 

        /* Drivetrain Constants */
        public static final double bumperLength = Units.inchesToMeters(5);
        public static final double trackWidth = Units.inchesToMeters(20.75); //Hand measure later
        public static final double wheelBase = Units.inchesToMeters(20.75); //Hand measure later
        public static final double robotLength = bumperLength + trackWidth;
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.75; 
        public static final double angleGearRatio = (150.0 / 7.0); 

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); 

        /* Swerve Current Limiting */
        public static final int angleLimit = 30; //30
        public static final int driveLimit = 40; //40;

        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Turning PID Values */
        public static final double turnKP = 5;
        public static final double turnKI = 0;
        public static final double turnKD = 0;

        /* Angle Motor PID Values */
        // switched 364 pid values to SDS pid values
        public static final double angleKP = 0.15; // 0.6; // citrus: 0.3 //0.15
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0; // 12.0; // citrus: 0
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 4e-5; //4e-5, //0.05
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.19255;//0.60094; // 0.19225;
        public static final double driveKV = 2.4366;//1.1559;  // 2.4366
        public static final double driveKA = 0.34415; //0.12348; // 0.34415

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double maxSpeed = 4.4; //meters per second - 16.3 ft/sec
        public static final double maxAcceleration = 3;
        public static final double maxAngularVelocity = 8; //3; //11.5; // citrus: 10 - Mason look at this later wtf
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration);

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        // TODO: Figure out angle offsets
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 20;
            public static final double angleOffset = -9.9;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 21;
            public static final double angleOffset = -64.072;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 22;
            public static final double angleOffset = -70.137;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 23;
            public static final double angleOffset = -108.633;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }


    public static class VisionConstants {

        public static final Camera FRONT_LEFT = new Camera("FRONT_LEFT", true, 0, 0, 0, 
                                                        new Transform2d(
                                                            new Translation2d(Units.inchesToMeters(5.43), Units.inchesToMeters(-11.9)), 
                                                                Rotation2d.fromDegrees(0)));

        public static final Camera FRONT_RIGHT = new Camera("FRONT_RIGHT", true, 0, 0, 0, 
                                                        new Transform2d(
                                                            new Translation2d(Units.inchesToMeters(5.43), Units.inchesToMeters(11.7)), 
                                                                Rotation2d.fromDegrees(0)));

        public static final Camera BACK_RIGHT = new Camera("BACK_RIGHT", true, 0, 0, 0, 
                                                        new Transform2d(
                                                            new Translation2d(Units.inchesToMeters(6.57), Units.inchesToMeters(-11.7)), 
                                                                Rotation2d.fromDegrees(180)));

        public static final Camera BACK_LEFT = new Camera("BACK_LEFT", true, 0, 0, 0, 
                                                        new Transform2d(
                                                            new Translation2d(Units.inchesToMeters(6.57), Units.inchesToMeters(18.89)), 
                                                                Rotation2d.fromDegrees(180)));

        public static final double POSE_THRESH = 100;

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(0.1,0.1,Units.degreesToRadians(3));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(1,1,Units.degreesToRadians(10));

        public static final HashMap<Integer,Pose2d> APRIL_TAG_POS = new HashMap<Integer,Pose2d>();

        static {
            APRIL_TAG_POS.put(1, new Pose2d(
                new Translation2d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(2, new Pose2d(
                new Translation2d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(3, new Pose2d(
                new Translation2d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(4, new Pose2d(
                new Translation2d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(5, new Pose2d(
                new Translation2d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74)),
                Rotation2d.fromDegrees(0))
            );
            APRIL_TAG_POS.put(6, new Pose2d(
                new Translation2d( Units.inchesToMeters(40.45), Units.inchesToMeters(174.19)),
                Rotation2d.fromDegrees(0))
            );
            APRIL_TAG_POS.put(7, new Pose2d(
                new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19)),
                Rotation2d.fromDegrees(0))
            );
            APRIL_TAG_POS.put(8, new Pose2d(
                new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19)),
                Rotation2d.fromDegrees(0))
            );
        } 
    }
    
    public static class FieldConstants{
        public static final double FIELD_X_LENGTH = Units.inchesToMeters(651.25); // meters
        public static final double FIELD_Y_LENGTH = Units.inchesToMeters(315.5); // meters
        public static final double HUB_RADIUS = Units.inchesToMeters(26.69); // meters

        public static final double LOADING_X_LEFT = 13.2; // meters
        public static final double LOADING_X_RIGHT = FIELD_X_LENGTH;
        public static final double tapeWidth = Units.inchesToMeters(2.0);
        public static final double midX = Units.inchesToMeters(132.375); // Tape to the left of charging station
        public static final double outerX = Units.inchesToMeters(193.25); // Tape to the right of charging station
        public static final double leftY = Units.feetToMeters(18.0);
        public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;

        public static final double chargingStationLength = Units.inchesToMeters(76.125);
        public static final double chargingStationWidth = Units.inchesToMeters(97.25);
        public static final double chargingStationOuterX = outerX - tapeWidth;
        public static final double chargingStationInnerX = chargingStationOuterX - chargingStationLength;
        public static final double chargingStationLeftY = midY - tapeWidth;
        public static final double chargingStationRightY = chargingStationLeftY - chargingStationWidth;
        public static final Translation2d[] chargingStationCorners =
        new Translation2d[] {
          new Translation2d(chargingStationInnerX, chargingStationRightY),
          new Translation2d(chargingStationInnerX, chargingStationLeftY),
          new Translation2d(chargingStationOuterX, chargingStationRightY),
          new Translation2d(chargingStationOuterX, chargingStationLeftY)
        };

        public static final double cableBumpInnerX = Units.inchesToMeters(149.5);
        public static final double cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
        public static final Translation2d[] cableBumpCorners =
        new Translation2d[] {
          new Translation2d(cableBumpInnerX, 0.0),
          new Translation2d(cableBumpInnerX, chargingStationRightY),
          new Translation2d(cableBumpOuterX, 0.0),
          new Translation2d(cableBumpOuterX, chargingStationRightY)
        };

        public static Pose2d allianceFlip(Pose2d pose) {
            if (DriverStation.getAlliance() == Alliance.Red) {
                return flip(pose);
            }
            return pose;
        }

        public static Translation2d allianceFlip(Translation2d translation) {
            if (DriverStation.getAlliance() == Alliance.Red) {
                return flipTranslation(translation);
            }
            return translation;
        }

        public static Rotation2d allianceFlip(Rotation2d rotation) {
            if (DriverStation.getAlliance() == Alliance.Red) {
                return flipRotation(rotation);
            }
            return rotation;
        }

        public static Pose2d flip(Pose2d pose) {
            return new Pose2d(flipTranslation(pose.getTranslation()), flipRotation(pose.getRotation()));
        }

        public static Translation2d flipTranslation(Translation2d translation) {
            return new Translation2d (
                FIELD_X_LENGTH - translation.getX(),
                translation.getY()
            );
        }

        public static Rotation2d flipRotation(Rotation2d rotation) {
            return Rotation2d.fromDegrees(MathUtil.inputModulus(180 - rotation.getDegrees(), -180, 180));
        }
    }

    public static class WristConstants {

        public static final double kP = 0.4;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.19;
        public static final double kV = 0;
        public static final double kG = 0.2;

        public static final double GEAR_RATIO = 76.235; //72,3

        public static final double ROTATION_TO_DEGREES = 360;
        
        public static final double ANGLE_OFFSET = 0; 

        public static final int MIN_ANGLE = -90;
        public static final int MAX_ANGLE = 90;

        public static final double WRIST_TOLERANCE = 1;

        public static final int WRIST_ID = 21;
        
    }

    public static class ManipulatorConstants{
        public static final int ROLLER_MOTOR_ID = 31;
        public static final double ROLLER_POWER = 0.9;
        public static final double STALL_POWER_CONE = 0.15;
        public static final double STALL_POWER_CUBE = 0.1;


        public static final double CURRENT_THRESHOLD_CONE = 35;
        public static final double CURRENT_THRESHOLD_CUBE = 25;
    }

    public static class ElevatorConstants {
        public static final int ELV1_ID = 11;
        public static final int ELV2_ID = 12;

        public static final double kP = 1.5;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.575;//0.2375; //0.975; //1.05;
        public static final double kV = 0;
        public static final double kG = 0.725;//0.4125;//0.975; 1.35;

        public static final double MIN_DIST = 2; //Ask Charlie
        public static final double MAX_DIST = 55; //Ask Charlie

        public static final double GEAR_RATIO = 10;
        public static final double SPOOL_CIRCUMFERENCE = 3 * Math.PI;
        public static final double FRAME_LENGTH = 15;

        public static final double ELV_TOLERANCE = 0.25;
        
        public static final int CURRENT_LIMIT = 80;
    }

    public static class LedConstants{
        public static final int CANDLE_ID = 52;
        
        public static final int WHITE_VALUE = 0; //leds used don't have a white value
        
        public static final int STARTING_ID = 8;
        public static final int PIVOT_COUNT = 200;
        public static final int PIVOT_COUNT_FRONT = 50; //change
        public static final int PIVOT_COUNT_BACK = 50; //change

        public static final double HOLDING_SPEED = 2;

        public static class RainbowAnimation {
            public static final double BRIGHTNESS = 1;
            public static final double SPEED = 1;

        }

        public enum Colors {
            OFF(0,0,0,false),
            CONE(255,255,0,false),
            CUBE(255,0,255,false),
            HOLDING(255,0,0,false),
    
            AUTO(0,0,0,true),
            SHELF(255, 105, 180, false),
            CHUTE(0,0,225,false);
    
            public final int r;
            public final int b;
            public final int g;
            public final boolean animation;
    
            Colors(int r, int g, int b,boolean animation) {
                this.r = r;
                this.g = g;
                this.b = b;
                this.animation = animation;
            }
    
        }
    }


}


