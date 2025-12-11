package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.subsystems.drive.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;



public class PathOnFlyCmdLeft extends Command {
   
    private final PoseEstimator poseEstimator;
    private final SwerveDrive swerveDrive;
    private Command pathCommand;
        
    
        public PathOnFlyCmdLeft(PoseEstimator poseEstimator, SwerveDrive swerveDrive) {
           this.poseEstimator = poseEstimator;
           this.swerveDrive = swerveDrive;
      
     addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = poseEstimator.getPose();

        PathPlannerPath LoadingLeft = PathFactory.getPathFromPose(currentPose);
        pathCommand = AutoBuilder.followPath(LoadingLeft);
        pathCommand.schedule();
       
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) { 
        if(interrupted && pathCommand != null){
            pathCommand.cancel();
        }
        }
    

    @Override 
    public boolean isFinished(){
        return pathCommand != null && pathCommand.isFinished();
    }
}