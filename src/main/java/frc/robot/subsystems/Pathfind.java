package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Pathfind {

    public static Command goToPose(Pose2d  drivePose, Pose2d pose){
        PathConstraints constraints = new PathConstraints(Constants.Pathfind.mVelMS, 
            Constants.Pathfind.mAccMS, Constants.Pathfind.mRVelRS,Constants.Pathfind.mRAccRS);
         return AutoBuilder.pathfindToPose(
                    pose,
                    constraints);
    }
    
}
