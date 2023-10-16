package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class OdometryOpMode {
    MecanumTrain drive = new MecanumTrain(hardwareMap);
    public void placePurplePixel() {
        // The purple pixel placement depends on spike mark placement
        // so add this to other files as needed
        Trajectory driveToRightSpike = drive.trajectoryBuilder(new Pose2d())
                .strafeRight() //Add coordinates for where it needs to go
                .forward() //Add coordinates for where it needs to go
                .build();
        Trajectory driveToLeftSpike = drive.trajectoryBuilder(new Pose2d())
                .strafeRight() //Add coordinates for where it needs to go
                .forward() //Add coordinates for where it needs to go
                .build();
        Trajectory driveToBackSpike = drive.trajectoryBuilder(new Pose2d())
                .strafeRight() //Add coordinates for where it needs to go
                .forward() //Add coordinates for where it needs to go
                .build();
        // Add code here for placing the pixel
    }

    public void driveToBackdrop() {
        // Again, this depends on spike mark placement,
        // so add this to other files as needed

        // The trajectory names refer to the 3 spike marks,
        // right is the right spike mark
        // left is the left spike mark
        // back is the last spike mark
        Trajectory driveToBackdropFromRight = drive.trajectoryBuilder(new Pose2d()) // Keeps facing forward
                .splineTo() //Add coordinates for where it needs to go
                            //Depending on how the spike mark placement is, we can use a different movement instead of spline
                .build();
        Trajectory driveToBackdropFromLeft = drive.trajectoryBuilder(new Pose2d()) // Keeps facing forward
                .splineTo() //Add coordinates for where it needs to go
                            //Depending on how the spike mark placement is, we can use a different movement instead of spline
                .build();
        Trajectory driveToBackdropFromBack = drive.trajectoryBuilder(new Pose2d()) // Keeps facing forward
                .splineTo() //Add coordinates for where it needs to go
                            //Depending on how the spike mark placement is, we can use a different movement instead of spline
                .build();

    }

    // Go back to start position to reset the odometry
    // Can be used in other things
    public void driveToStart() {

        //The traj names are diff based on where the robot will be at a given time
        //The leftBackdrop refers to the left side of the backdrop and so on

        Trajectory leftBackdropToStart = drive.trajectoryBuilder(new Pose2d()) // Keeps facing forward
                .splineTo() //Add coordinates for where it needs to go
                            //Depending on how the spike mark placement is, we can use a different movement instead of spline
                .build();
        Trajectory rightBackdropToStart = drive.trajectoryBuilder(new Pose2d()) // Keeps facing forward
                .splineTo() //Add coordinates for where it needs to go
                //Depending on how the spike mark placement is, we can use a different movement instead of spline
                .build();
        Trajectory centerBackdropToStart = drive.trajectoryBuilder(new Pose2d()) // Keeps facing forward
                .splineTo() //Add coordinates for where it needs to go
                //Depending on how the spike mark placement is, we can use a different movement instead of spline
                .build();
    }




}
