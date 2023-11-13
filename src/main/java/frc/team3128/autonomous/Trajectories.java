package frc.team3128.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.PositionConstants.Position;
import static frc.team3128.commands.CmdManager.*;

import frc.team3128.commands.CmdAutoBalance;
import frc.team3128.subsystems.Leds;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Swerve;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static final HashMap<String, List<PathPlannerTrajectory>> trajectories = new HashMap<String, List<PathPlannerTrajectory>>();

    private static final HashMap<String, Command> CommandEventMap = new HashMap<String, Command>();

    private static final Swerve swerve = Swerve.getInstance();

    private static SwerveAutoBuilder builder;

    public static void initTrajectories() {
        final String[] trajectoryNames = {
                                        //Blue Autos
                                            //Cable
                                            "b_cable_1Cone+1Cube", "b_cable_1Cone+1.5Cube","b_cable_1Cone+2Cube", "b_cable_1Cone+1.5Cube+Climb",
                                            //Mid
                                            "b_mid_1Cone+Climb","b_mid_1Cone+0.5Cube+Climb", "b_mid_1Cone+1Cube+Climb",
                                            //Hp
                                            "b_hp_1Cone+1Cube", "b_hp_1Cone+1.5Cube",
                                            
                                        //Red Autos
                                            //Cable
                                            "r_cable_1Cone+1Cube", "r_cable_1Cone+1.5Cube","r_cable_1Cone+2Cube", "r_cable_1Cone+1.5Cube+Climb",
                                            //Mid
                                            "r_mid_1Cone+Climb","r_mid_1Cone+0.5Cube+Climb","r_mid_1Cone+1Cube+Climb",
                                            //Hp
                                            "r_hp_1Cone+1Cube", "r_hp_1Cone+1.5Cube"
                                        };

        CommandEventMap.put("ScoreConeHigh", sequence(score(Position.HIGH_CONE_AUTO, true)));

        CommandEventMap.put("ScoreCubeHigh", score(Position.HIGH_CUBE, true));

        CommandEventMap.put("ScoreLow", score(Position.LOW, true));
        
        CommandEventMap.put("PickupCube", pickup(Position.GROUND_CUBE, true));

        CommandEventMap.put("Neutral", sequence(stopManip(), retract(Position.NEUTRAL)));
        
        CommandEventMap.put("Balance", new ScheduleCommand(new CmdAutoBalance(true)));
        
        CommandEventMap.put("Balance2", new ScheduleCommand(new CmdAutoBalance(false)));

        for (final String trajectoryName : trajectoryNames) {

            if (trajectoryName.contains("mid")) {
                trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, slow));
            } 
            else {
                trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, fast));
            }
        }

        builder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            swerveKinematics,
            new PIDConstants(translationKP, translationKI, translationKD),
            new PIDConstants(rotationKP, rotationKI, rotationKD),
            swerve::setModuleStates,
            CommandEventMap,
            swerve
        );
    }

    public static CommandBase generateAuto(PathPlannerTrajectory trajectory) {
        return builder.fullAuto(trajectory);
    }

    public static CommandBase get(String name) {
        return builder.fullAuto(trajectories.get(name));
    }

    public static CommandBase resetAuto() {
        return sequence(
            runOnce(()-> Leds.getInstance().defaultColor = Colors.AUTO),
            resetLeds(),
            resetGyro(DriverStation.getAlliance() == Alliance.Red ? 0 : 180),
            runOnce(()-> Manipulator.getInstance().set(-0.5), Manipulator.getInstance()),
            runOnce(()-> Manipulator.getInstance().isCone = true),
            resetAll(),
            retract(Position.NEUTRAL)
        );
    }
    
}