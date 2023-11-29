package frc.team3128;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.team3128.commands.CmdAutoBalance;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.PositionConstants.Position;
import static frc.team3128.commands.CmdManager.*;

import common.utility.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.hardware.input.NAR_ButtonBoard;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;

    private NAR_Joystick rightStick;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;

    private NarwhalDashboard dashboard;

    public RobotContainer() {

        swerve = Swerve.getInstance();

        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);
        
        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true));
        initDashboard();
        configureButtonBindings();
        
        DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        controller.getButton("RightTrigger").onTrue(score(Position.LOW, true)).onFalse(runOnce(()-> ENABLE = false));
        controller.getButton("LeftTrigger").onTrue(runOnce(()-> ENABLE = true)).onFalse(runOnce(()-> ENABLE = false));
        controller.getButton("X").onTrue(xLock()).onFalse(stop());
        controller.getButton("B").onTrue(resetSwerve());
        controller.getButton("Start").onTrue(resetGyro());
        controller.getButton("RightBumper").onTrue(pickup(Position.GROUND_CUBE, true)).onFalse(retract(Position.NEUTRAL).andThen(stallPower()).andThen(resetLeds()));
        controller.getButton("LeftBumper").onTrue(pickup(Position.GROUND_CONE, true)).onFalse(retract(Position.NEUTRAL).andThen(stallPower()).andThen(resetLeds()));
        controller.getButton("RightStick").onTrue(runOnce(()-> CmdSwerveDrive.setTurnSetpoint()));

        controller.getUpPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.rSetpoint = DriverStation.getAlliance() == Alliance.Red ? 180 : 0;
            CmdSwerveDrive.enabled = true;
        }));
        controller.getDownPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.rSetpoint = DriverStation.getAlliance() == Alliance.Red ? 0 : 180;
            CmdSwerveDrive.enabled = true;
        }));

        controller.getRightPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.rSetpoint = DriverStation.getAlliance() == Alliance.Red ? 90 : 270;
            CmdSwerveDrive.enabled = true;
        }));

        controller.getLeftPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.rSetpoint = DriverStation.getAlliance() == Alliance.Red ? 270 : 90;
            CmdSwerveDrive.enabled = true;
        }));
        
        rightStick.getButton(1).onTrue(resetGyro());
        rightStick.getButton(2).onTrue(moveElv(0.4)).onFalse(moveElv(0));
        rightStick.getButton(3).onTrue(moveElv(-0.4)).onFalse(moveElv(0));
        rightStick.getButton(4).onTrue(moveElevator(30));
        rightStick.getButton(5).onTrue(resetElevator());
        rightStick.getButton(6).onTrue(moveWri(0.4)).onFalse(moveWri(0));
   
   
        rightStick.getButton(7).onTrue(moveWri(-0.4)).onFalse(moveWri(0));
        rightStick.getButton(8).onTrue(moveWrist(30));
        rightStick.getButton(9).onTrue(resetWrist());
        rightStick.getButton(10).onTrue(intake(true));
        rightStick.getButton(11).onTrue(intake(false));
        rightStick.getButton(12).onTrue(outtake());
        rightStick.getButton(13).onTrue(stopManip());
        rightStick.getButton(14).onTrue(new CmdAutoBalance(true));
        rightStick.getButton(15).onTrue(new CmdAutoBalance(false));


        buttonPad.getButton(5).onTrue(
            score(Position.LOW, 1)
        );
        buttonPad.getButton(8).onTrue(
            score(Position.MID_CUBE, 1)
        );
        buttonPad.getButton(11).onTrue(
            score(Position.HIGH_CUBE, 1)
        );

        buttonPad.getButton(13).onTrue(toggleLeds());

        buttonPad.getButton(14).onTrue(retract(Position.NEUTRAL).beforeStarting(stopManip()));


        buttonPad.getButton(16).onTrue(
            HPpickup(Position.CHUTE_CONE, Position.SHELF_CONE)
        );

        buttonPad.getButton(15).onTrue(
            HPpickup(Position.CHUTE_CUBE, Position.SHELF_CUBE)
        );
        
        buttonPad.getButton(1).onTrue(runOnce (()-> {
            Vision.SELECTED_GRID = DriverStation.getAlliance() == Alliance.Red ? 0 : 2;
        }));
        buttonPad.getButton(2).onTrue(runOnce(()-> Vision.SELECTED_GRID = 1));
        buttonPad.getButton(3).onTrue(runOnce(()-> {
            Vision.SELECTED_GRID = DriverStation.getAlliance() == Alliance.Red ? 2 : 0;
        }));

    }

    public void init() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            buttonPad.getButton(4).onTrue(
                score(Position.LOW, 0)
            );
            buttonPad.getButton(6).onTrue(
                score(Position.LOW, 2)
            );
            buttonPad.getButton(7).onTrue(
                score(Position.MID_CONE, 0)
            );
            buttonPad.getButton(9).onTrue(
                score(Position.MID_CONE, 2)
            );
            buttonPad.getButton(10).onTrue(
                score(Position.HIGH_CONE, 0)
            );
            buttonPad.getButton(12).onTrue(
                score(Position.HIGH_CONE, 2)
            );
            
        }
        else {
            buttonPad.getButton(6).onTrue(
                score(Position.LOW, 0)
            );
            buttonPad.getButton(4).onTrue(
                score(Position.LOW, 2)
            );
            buttonPad.getButton(9).onTrue(
                score(Position.MID_CONE, 0)
            );
            buttonPad.getButton(7).onTrue(
                score(Position.MID_CONE, 2)
            );
            buttonPad.getButton(12).onTrue(
                score(Position.HIGH_CONE, 0)
            );
            buttonPad.getButton(10).onTrue(
                score(Position.HIGH_CONE, 2)
            );
        }
    }

    private void initDashboard() {

        swerve.initShuffleboard();
        Vision.getInstance().initShuffleboard();

        dashboard = NarwhalDashboard.getInstance();
        dashboard.addUpdate("time", ()-> Timer.getMatchTime());
        dashboard.addUpdate("voltage",()-> RobotController.getBatteryVoltage());
        dashboard.addUpdate("x", ()-> swerve.getPose().getX());
        dashboard.addUpdate("y", ()-> swerve.getPose().getY());
    }

    public void updateDashboard() {
        dashboard.update();
        if (DriverStation.getMatchType() == MatchType.None) {
            NAR_Shuffleboard.update();
        }
    }
}
