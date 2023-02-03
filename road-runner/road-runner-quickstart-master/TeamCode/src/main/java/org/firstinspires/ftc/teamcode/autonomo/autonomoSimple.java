package org.firstinspires.ftc.teamcode.autonomo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomo.modulo_cam.SleeveDetection;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Prueba simple azul")
public class autonomoSimple extends LinearOpMode {
    enum State{
        DEJAR,
       MOVE,
       WAIT
    }

    State currentState = State.DEJAR;

    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private CRServo garra;
    double garraPoder=0;
    private String color_vision;
    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(270));


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        garra = hardwareMap.get(CRServo.class, "garra");

        TrajectorySequence dejarCono = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.5)
                .turn(Math.toRadians(270))
                .lineToConstantHeading(new Vector2d(-60, 60))
                .build();

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(-60, 26))
                .build();

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(-34.5, 60))
                .lineToConstantHeading(new Vector2d(-34.5, 26))
                .build();

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(-12.5, 60))
                .lineToConstantHeading(new Vector2d(-12.5, 26))
                .build();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            SleeveDetection.ParkingPosition hola = sleeveDetection.getPosition();
            color_vision = hola.toString();
        }

        waitForStart();

        currentState = State.DEJAR;

        while (opModeIsActive() && !isStopRequested()) {


            switch(currentState) {
                case DEJAR:
                    garraPoder = 0.5;

                    currentState=State.MOVE;
                    break;
                case MOVE:
                if (color_vision == "LEFT") {
                    //
                    garraPoder = -.5;
                    drive.followTrajectorySequenceAsync(trajectory3);

                    currentState=State.WAIT;
                } else if (color_vision == "CENTER") {
                    garraPoder = -.5;
                    drive.followTrajectorySequenceAsync(trajectory2);

                    currentState=State.WAIT;
                } else if (color_vision == "RIGHT") {
                    garraPoder = -.5;
                    drive.followTrajectorySequenceAsync(trajectory1);

                    currentState=State.WAIT;
                }

                break;
                case WAIT:
                    break;
            }

            drive.update();
            garra.setPower(garraPoder);

            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;


        }
    }
}
