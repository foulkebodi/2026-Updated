package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;

public class PathFactory {

public static PathPlannerPath getPathFromPose(Pose2d currentPose){

   
    List<Waypoint> autoAlignWaypoints;
    PathConstraints pathConstraints = new PathConstraints(4.0, 1.5,12.6 , 6.3);
    GoalEndState goalEndState;

    
    
    Pose2d[] aprilTagLocations = new Pose2d[] {
        new Pose2d(4.028, 3.222, null),              // april tag 17 / BLUE CLOSE RIGHT
        new Pose2d(3.6, 4.0, null),                  // april tag 18 / BLUE CLOSE MIDDLE
        new Pose2d(4.028, 4.8, null),           // april tag 19 / BLUE CLOSE LEFT
        new Pose2d(4.9, 4.8, null),           // april tag 20 / BLUE FAR LEFT
        new Pose2d(5.4, 4.0, null),           // april tag 21 / BLUE FAR MIDDLE
        new Pose2d(4.9, 3.222, null),           // april tag 22 / BLUE FAR RIGHT
        new Pose2d(13.5,3.222, null),           // april tag 6 / RED CLOSE LEFT
        new Pose2d(14.0, 4.0, null),           // april tag 7 / RED CLOSE MIDDLE
        new Pose2d(13.5, 4.8, null),           // april tag 8 / RED CLOSE RIGHT
        new Pose2d(12.6, 4.8, null),           // april tag 9 / RED FAR RIGHT
        new Pose2d(12.2, 4.0, null),           // april tag 10 / RED FAR MIDDLE
        new Pose2d(12.6, 3.222, null)            // april tag 11 / RED FAR LEFT
        
    };

    Translation2d robotTranslation = currentPose.getTranslation();
    Translation2d tagTranslation;
    double[] tagDistanceMeters = new double[12];

    for(int i = 0; i < 12; i++){
        tagTranslation = aprilTagLocations[i].getTranslation();
        tagDistanceMeters[i] = robotTranslation.getDistance(tagTranslation);
    }

    double placeholder = tagDistanceMeters[0];
   
    for(int j = 0; j < 12; j++){
        if(placeholder > tagDistanceMeters[j]) {
            placeholder = tagDistanceMeters[j];
        }
    }



// Optional<Alliance> alliance = DriverStation.getAlliance();

 //if(alliance.get() == Alliance.Red){}


    if(placeholder <= 0.5 && currentPose.getY() <= 4.5){
        goalEndState = new GoalEndState(0.0, currentPose.getRotation());
        autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
             currentPose,
            new Pose2d(currentPose.getX(), currentPose.getY() - 0.5, currentPose.getRotation())); 
    } else if(placeholder <= 0.5 && currentPose.getY() >= 4.5){
        goalEndState = new GoalEndState(0.0, currentPose.getRotation());
        autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(currentPose.getX(), currentPose.getY() + 0.5, currentPose.getRotation()));


    }else if(placeholder == tagDistanceMeters[0]){
         goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(60));

        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
                currentPose,
                new Pose2d(3.7, 3.0, Rotation2d.fromDegrees(60)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
                currentPose,
                new Pose2d(3.98, 2.8, Rotation2d.fromDegrees(60)));
        }

    }else if(placeholder == tagDistanceMeters[1]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(0));

        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
                currentPose,
                new Pose2d(3.17, 4.195, Rotation2d.fromDegrees(0)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
                currentPose,
                new Pose2d(3.17, 3.85, Rotation2d.fromDegrees(0)));

        }

    }else if(placeholder == tagDistanceMeters[2]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(300));

        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d( 3.97, 5.232, Rotation2d.fromDegrees(300)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(3.68,5.07, Rotation2d.fromDegrees(300)));

        }

    }else if(placeholder == tagDistanceMeters[3]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(240));
        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d( 5.0, 5.25, Rotation2d.fromDegrees(240)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(5.28, 5.07, Rotation2d.fromDegrees(240)));

        }

    }else if(placeholder == tagDistanceMeters[4]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(180));
        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d( 5.8, 4.185, Rotation2d.fromDegrees(180)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,          
            new Pose2d(5.8,3.85, Rotation2d.fromDegrees(180)));
            
        }

    }else if(placeholder == tagDistanceMeters[5]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(120));
        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
           
            new Pose2d( 5.0, 2.8, Rotation2d.fromDegrees(120)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
           
            new Pose2d(4.98, 2.83, Rotation2d.fromDegrees(120)));
                
        }
    }else if(placeholder == tagDistanceMeters[6]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(120));
        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d( 13.57, 2.83, Rotation2d.fromDegrees(120)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(13.85, 2.98, Rotation2d.fromDegrees(120)));
               
        }
    }else if(placeholder == tagDistanceMeters[7]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(180));
        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d( 14.37, 3.84, Rotation2d.fromDegrees(180)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(14.37, 4.2, Rotation2d.fromDegrees(180)));
        }
        
    }else if(placeholder == tagDistanceMeters[8]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(240));
        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(13.86, 5.07, Rotation2d.fromDegrees(240)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(13.58, 5.22, Rotation2d.fromDegrees(240)));
        }
    }else if(placeholder == tagDistanceMeters[9]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(300));
        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d( 12.280, 5.07, Rotation2d.fromDegrees(300)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(12.56, 5.23, Rotation2d.fromDegrees(300)));
               
        }
    }else if(placeholder == tagDistanceMeters[10]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(0));
        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(11.76, 3.85, Rotation2d.fromDegrees(0)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            
            new Pose2d( 11.76, 4.185, Rotation2d.fromDegrees(0)));
        }
    }else if(placeholder == tagDistanceMeters[11]){
        goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(60));
        if( RobotContainer.leftCoralMode == true){
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d( 12.55, 2.84, Rotation2d.fromDegrees(60)));
        }else{
            autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(12.26, 2.98, Rotation2d.fromDegrees(60)));
        }
    }else{
        goalEndState = new GoalEndState(0.0, currentPose.getRotation());
        autoAlignWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            new Pose2d(currentPose.getX() + 0.1, currentPose.getY() + 0.1, currentPose.getRotation()));
    }

    PathPlannerPath LoadingPathUpperHalf = new PathPlannerPath(
        autoAlignWaypoints, 
        pathConstraints,
        null, 
        goalEndState
        );
     
    LoadingPathUpperHalf.preventFlipping = true;

    return LoadingPathUpperHalf;
    }
}