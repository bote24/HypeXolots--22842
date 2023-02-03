package org.firstinspires.ftc.teamcode.autonomo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomo.modulo_cam.SleeveDetection;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Prueba compleja")
public class aut extends LinearOpMode {
    private DcMotor Elevador1 = null, Elevador2 = null;
    private CRServo garra;

    enum State {

        ELEVATOR_1,
        ELEVATOR_2,
        END
    }

    State currentState = State.ELEVATOR_1;

    Pose2d startPose = new Pose2d(-36, 62.35, Math.toRadians(270));

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor UpLeft = null, UpRight = null, DownLeft = null, DownRight = null;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    private String color_vision;
    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    final int LIFT_ZERO = 0;
    final int LIFT_LOW = 2353;
    final int LIFT_MED = 4000;
    final int LIFT_HIGH = 5600;
    int LIFT_GOING;
    double motorpower = 0.4;


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevador1 = hardwareMap.get(DcMotor.class, "ElevadorIzq");
        Elevador2 = hardwareMap.get(DcMotor.class, "ElevadorDer");
        garra = hardwareMap.get(CRServo.class, "garra");
        Elevador2.setDirection(DcMotor.Direction.REVERSE);
        Elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevador2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevador1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Elevador2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(180))) //Ir a primer high junc.
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    Elevador1.setTargetPosition(LIFT_HIGH);
                    Elevador2.setTargetPosition(LIFT_HIGH);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    // Subir elevador a high 3 segundos antes de llegar
                })


                .lineToConstantHeading(new Vector2d(-14, 0)) // Ajustarse
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //servo abrir
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Elevador1.setTargetPosition(LIFT_ZERO);
                    Elevador2.setTargetPosition(LIFT_ZERO);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //Bajar elevador 1.5 segundos despues de soltar cono
                })
                .waitSeconds(3)
                .strafeRight(12) // ajustar
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(-56, 12)) // ir por cono
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    Elevador1.setTargetPosition(150);
                    Elevador2.setTargetPosition(150);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //subir elevador poquito 1 segundo antes de llegar por el cono
                    //2do cono
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //servo cerrar
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    Elevador1.setTargetPosition(LIFT_ZERO);
                    Elevador2.setTargetPosition(LIFT_ZERO);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //bajar el elevador poquito
                })
                .waitSeconds(1.5)
                .lineToSplineHeading(new Pose2d(-34, 12, Math.toRadians(315))) // ir a la high junc.
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    Elevador1.setTargetPosition(LIFT_HIGH);
                    Elevador2.setTargetPosition(LIFT_HIGH);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //subir el elevador 3 segundos antes de llegar
                })
                .forward(4) // ajustar
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //abrir servo
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Elevador1.setTargetPosition(LIFT_ZERO);
                    Elevador2.setTargetPosition(LIFT_ZERO);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //Bajar elevador 1.5 segundos despues de soltar cono
                })
                .lineToSplineHeading(new Pose2d(-56, 12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    Elevador1.setTargetPosition(90);
                    Elevador2.setTargetPosition(90);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //subir elevador poquito 1 segundo antes de llegar por el cono
                    //3er cono
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //servo cerrar
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    Elevador1.setTargetPosition(LIFT_ZERO);
                    Elevador2.setTargetPosition(LIFT_ZERO);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //bajar el elevador poquito
                })
                .waitSeconds(1.5) //a
                .lineToSplineHeading(new Pose2d(-34, 12, Math.toRadians(315))) // ir a la high junc.
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    Elevador1.setTargetPosition(LIFT_HIGH);
                    Elevador2.setTargetPosition(LIFT_HIGH);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //subir el elevador 3 segundos antes de llegar
                })
                .forward(4) // ajustar
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //abrir servo
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Elevador1.setTargetPosition(LIFT_ZERO);
                    Elevador2.setTargetPosition(LIFT_ZERO);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //Bajar elevador 1.5 segundos despues de soltar cono
                })
                .lineToSplineHeading(new Pose2d(-56, 12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    Elevador1.setTargetPosition(60);
                    Elevador2.setTargetPosition(60);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //subir elevador poquito 1 segundo antes de llegar por el cono
                    //4to cono
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //servo cerrar
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    Elevador1.setTargetPosition(LIFT_ZERO);
                    Elevador2.setTargetPosition(LIFT_ZERO);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //bajar el elevador poquito
                })

                .lineToSplineHeading(new Pose2d(-34, 12, Math.toRadians(315))) // ir a la high junc.
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    Elevador1.setTargetPosition(LIFT_HIGH);
                    Elevador2.setTargetPosition(LIFT_HIGH);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //subir el elevador 3 segundos antes de llegar
                })
                .forward(4) // ajustar
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //abrir servo
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    Elevador1.setTargetPosition(LIFT_ZERO);
                    Elevador2.setTargetPosition(LIFT_ZERO);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    //Bajar elevador 1.5 segundos despues de soltar cono
                })



                .build();


        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .lineToConstantHeading(new Vector2d(-56, 62))
                .lineToConstantHeading(new Vector2d(-40, 62))
        //estacionar pegado a pared
        //revisar coordenadas!
                .build();

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory1.end())
                .lineToConstantHeading(new Vector2d(-56, 42))
                .lineToConstantHeading(new Vector2d(-40, 42))
                //estacionar pegado a pared
                //revisar coordenadas!
                .build();

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory1.end())
                .lineToConstantHeading(new Vector2d(-56, 22))
                .lineToConstantHeading(new Vector2d(-40, 22))
                //estacionar pegado a pared
                //revisar coordenadas!
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

            telemetry.update();
        }
        TrajectorySequence Sleeve;
        if (color_vision == "LEFT"){
            Sleeve = trajectory2;
        }
        else if (color_vision == "CENTER"){
            Sleeve = trajectory3;
        }
        else{
            Sleeve = trajectory4;
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (currentState){
            case ELEVATOR_1:
                if (!drive.isBusy()){
                    drive.followTrajectorySequenceAsync(trajectory1);
                    currentState=State.ELEVATOR_2;
                }
                break;
            case ELEVATOR_2:
                if (!drive.isBusy()){
                    drive.followTrajectorySequenceAsync(Sleeve);
                    currentState=State.END;
                }
                break;
        }




//            if (color_vision == "LEFT") {
//                UpLeft.setPower(1);
//                UpRight.setPower(1);
//                DownLeft.setPower(1);
//                DownRight.setPower(1);
//                sleep(3000);
//                UpLeft.setPower(-.4);
//                UpRight.setPower(.4);
//                DownLeft.setPower(.4);
//                DownRight.setPower(-.4);
//                sleep(2000);
//
//            } else if (color_vision == "RIGHT") {
//                UpLeft.setPower(1);
//                UpRight.setPower(1);
//                DownLeft.setPower(1);
//                DownRight.setPower(1);
//                sleep(3000);
//                UpLeft.setPower(.4);
//                UpRight.setPower(-.4);
//                DownLeft.setPower(-.4);
//                DownRight.setPower(.4);
//                sleep(2000);
//            } else {
//                UpLeft.setPower(1);
//                UpRight.setPower(1);
//                DownLeft.setPower(1);
//                DownRight.setPower(1);
//                sleep(3000);
//
//            }
//            UpLeft.setPower(0);
//            UpRight.setPower(0);
//            DownLeft.setPower(0);
//            DownRight.setPower(0);
//            sleep(3000);
//        }
    }
}