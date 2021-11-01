package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.entity.BotEntity;
import com.noahbres.meepmeep.core.entity.Entity;
import com.noahbres.meepmeep.roadrunner.Constraints;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;


public class MeepMeepTesting {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                .setBotDimensions(12, 16)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 60, Math.toRadians(90)))
                                .back(4)
                                .turn(Math.toRadians(90))
                                .strafeRight(5)
                                .back(30)
                                .build()


                )
                .start();
    }
}




//public class MeepMeepTesting {
//    public static void main(String[] args) {
//        // Declare a MeepMeep instance
//        // With a field size of 800 pixels
//        MeepMeep mm = new MeepMeep(800)
//                // Set field image
//                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
//                // Set theme
//                .setTheme(new ColorSchemeRedDark())
//                // Background opacity from 0-1
//                .setBackgroundAlpha(1f)
//                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .addDisplacementMarker(() -> {
//                                    /* Everything in the marker callback should be commented out */
//
//                                    // bot.shooter.shoot()
//                                    // bot.wobbleArm.lower()
//                                })
//                                .turn(Math.toRadians(90))
//                                .splineTo(new Vector2d(10, 15), 0)
//                                .turn(Math.toRadians(90))
//                                .build()
//                )
//                .start();
//    }
//}
//
//
//public class MeepMeepTesting {
//    public static void main(String[] args) {
////        MeepMeep mm = new MeepMeep(800);
////        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL);
////        mm.start();
////        DefaultBotBuilder bot = new DefaultBotBuilder(mm);
////        bot.setDimensions(15, 8);
////        bot.setColorScheme(new ColorSchemeRedDark());
////        Pose2d startPose = new Pose2d(0, 0, 0);
//        //bot.setStartPose(startPose);
//        //bot.build();
//        //Constraints cnst = new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15);
//
//   //     bot.followTrajectorySequence(drive ->
//       //         drive.trajectorySequenceBuilder(startPose)
//          //      .forward(20)
//          //      .strafeRight(35)
//          //      .turn(Math.toRadians(90))
//         //       .build())
//          //      .start();
//
//        // TODO: If you experience poor performance, enable this flag
//        // System.setProperty("sun.java2d.opengl", "true");
//
//        // Declare a MeepMeep instance
//        // With a field size of 800 pixels
//       MeepMeep mm = new MeepMeep(800)
////                // Set field image
//                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
////                // Set theme
//               .setTheme(new ColorSchemeRedDark())
//                // Background opacity from 0-1
//               .setBackgroundAlpha(1f)
//      //  Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
////
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(300))) //BLUE DUCK
////                 //       drive.trajectorySequenceBuilder(new Pose2d(5, 60, Math.toRadians(240))) //BLUE LEFT NO DUCK
////                     //   drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-240))) //RED SIDE DUCKS
////
////
////                            //THIS IF FOR  DUCK SIDE DELIVER AND PARK
//                                .forward(5)
//                              .turn(Math.toRadians(-30))
//                               .forward(6)
//                              .strafeRight(28)
//                               .back(8)
//                                .forward(22)
////
////
////                           //THIS IS FOR BLUE, LEFT SIDE DELIVER AND PARK
////
////                                //.forward(5)
////                                //.turn(Math.toRadians(30))
////                                //.back(6)
////                                //.strafeLeft(40)
////
////
////                                //THIS IS FOR RED,DUCK SIDE WITH PARK AND DELIVERY
////
//                                .build()
////                //)
//                .start();
////        //Constraints constr = new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15);
////        //Pose2d p2d = new Pose2d(0,0,Math.toRadians(0));
////        //RoadRunnerBotEntity be = new RoadRunnerBotEntity(mm, constr, p2d, new ColorSchemeRedDark(), 0.5, )
////        //RoadRunnerBotEntity be = new RoadRunnerBotEntity();
////        DefaultBotBuilder bb = new DefaultBotBuilder(mm);
////        Pose2d startPose = new Pose2d(0, 0, 0);
////        TrajectorySequence ts = TrajectorySequence(startPose).forward(20).strafeLeft(35).build();
////        bb..followTrajectorySequence(drive ->
////                // drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(300))) //BLUE DUCK
////                //       drive.trajectorySequenceBuilder(new Pose2d(5, 60, Math.toRadians(240))) //BLUE LEFT NO DUCK
////                //   drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-240))) //RED SIDE DUCKS
////
////
////                //THIS IF FOR  DUCK SIDE DELIVER AND PARK
////                      .forward(5)
////                  .turn(Math.toRadians(-30))
////                    .forward(6)
////                    .strafeRight(28)
////                    .back(8)
////                     .forward(22)
////                .build()).start();
//    }
//}