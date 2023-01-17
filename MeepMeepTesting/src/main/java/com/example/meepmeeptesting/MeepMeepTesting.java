package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(14,17)
                .setConstraints(52.48, 52.48, Math.toRadians(266.7532763442901), Math.toRadians(184.02607784577722), 11.31)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                /*
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> motorControl.claw.setPower(1))
                                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> motorControl.slide.setTargetPosition(400))

                                 */


                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(15))
                                .strafeRight(12)
                                .splineToSplineHeading(new Pose2d(-12, -36, Math.toRadians(0)), Math.toRadians(90))
                                .strafeLeft(10)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(266.753), 11.31))
                                .forward(3.5)


                                /*
                                .UNSTABLE_addTemporalMarkerOffset(-3, () -> motorControl.setMode(motorControl.combinedMode.TOP))

                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> motorControl.slide.setTargetPosition(motorControl.slide.getTargetPosition() - 300))
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> motorControl.claw.setPower(0.5))
                                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> motorControl.setMode(motorControl.combinedMode.TOP))

                                 */


                                .waitSeconds(1.5)

                                .back(3.5)
                                //.addDisplacementMarker(() -> motorControl.setMode(motorControl.combinedMode.BOTTOM))

                                .resetVelConstraint()
                                .strafeLeft(2)
                                //.addDisplacementMarker(() -> motorControl.slide.targetPosition = 350) // TODO: TUNE THIS
                                .splineToSplineHeading(new Pose2d(-24, -10.5, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-57, -10.5), Math.toRadians(180))
                                // pickup cone
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(10), 11.31))
                                //.addDisplacementMarker(() -> motorControl.arm.setMode(motorControl.arm.armMode.MOVING_DOWN))

                                .forward(5.25)
                                /*
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> motorControl.claw.setPower(1))
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> motorControl.slide.setTargetPosition(1100))

                                 */
                                .waitSeconds(1.5)


                                .back(6)
                                .resetVelConstraint()


                                // place cone
                                .splineToSplineHeading(new Pose2d(-25, -12, Math.toRadians(90)), Math.toRadians(0))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(10), 11.31))
                                .forward(5)
                                /*
                                .UNSTABLE_addTemporalMarkerOffset(-3, () -> motorControl.setMode(motorControl.combinedMode.TOP))

                                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> motorControl.slide.setTargetPosition(motorControl.slide.getTargetPosition() - 300))
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> motorControl.claw.setPower(0.5))
                                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> motorControl.setMode(motorControl.combinedMode.TOP))

                                 */


                                .waitSeconds(1.5)
                                .back(5)
                                .resetVelConstraint()
                                /*
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    motorControl.setMode(motorControl.combinedMode.BOTTOM);
                                    if (tagOfInterest != null) {
                                        if (tagOfInterest.id == 2) {
                                            // TODO: drive to 1 position
                                            TrajectorySequence parkSmall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(10), 11.31))
                                                    // park1
                                                    .strafeRight(2)
                                                    .splineToConstantHeading(new Vector2d(-12, -24), Math.toRadians(270))
                                                    .build();
                                            drive.followTrajectorySequenceAsync(parkSmall);

                                        } else if (tagOfInterest.id == 1) {
                                            // TODO: drive to 2 position

                                            TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(10), 11.31))
                                                    .strafeLeft(2)
                                                    .splineToConstantHeading(new Vector2d(-36, -24), Math.toRadians(270))
                                                    .build();
                                            drive.followTrajectorySequenceAsync(parkMiddle);
                                        } else if (tagOfInterest.id == 0) {
                                            TrajectorySequence backSmall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(10), 11.31))
                                                    .strafeLeft(2)
                                                    .splineToConstantHeading(new Vector2d(-36, -26), Math.toRadians(270))
                                                    .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(0))
                                                    .build();
                                            drive.followTrajectorySequenceAsync(backSmall);
                                        }
                                    } else {
                                        TrajectorySequence backSmall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                .strafeRight(2)
                                                .splineToConstantHeading(new Vector2d(-12, -24), Math.toRadians(270))
                                                .build();
                                        drive.followTrajectorySequenceAsync(backSmall);
                                    }

                                })

                                 */

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}