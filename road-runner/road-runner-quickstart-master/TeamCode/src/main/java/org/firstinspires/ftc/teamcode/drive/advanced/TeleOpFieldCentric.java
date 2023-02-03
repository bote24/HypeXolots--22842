package org.firstinspires.ftc.teamcode.drive.advanced;

import static org.firstinspires.ftc.teamcode.drive.advanced.TeleOpFieldCentric.LiftState.LIFT_END;
import static org.firstinspires.ftc.teamcode.drive.advanced.TeleOpFieldCentric.LiftState.LIFT_START;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
//@Config
@TeleOp(group = "advanced")
public class TeleOpFieldCentric extends LinearOpMode {
    private DcMotor Elevador1=null,Elevador2=null;
    private CRServo garra;
    public enum LiftState {
        LIFT_START,
        LIFT_END,
        };

//    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 537.7;

    final int LIFT_ZERO = 0;
    final int LIFT_LOW = 2353;
    final int LIFT_MED = 4000;
    final int LIFT_HIGH = 5600;
    int LIFT_GOING;
    double motorpower = 0.4;

    @Override

    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevador1  = hardwareMap.get(DcMotor.class, "ElevadorIzq");
        Elevador2  = hardwareMap.get(DcMotor.class, "ElevadorDer");
        garra = hardwareMap.get(CRServo.class, "garra");
        Elevador2.setDirection(DcMotor.Direction.REVERSE);
        Elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevador2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevador1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Elevador2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor.setTargetPosition(1440);
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

//        controller = new PIDController(p,i,d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getinstance().getTelemetry());

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        myLocalizer.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        LiftState liftState = LIFT_START;
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
//            double poder_ele=0;
            double garraPoder=0;
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            //Elevador1.setPower(motorpower);
            //Elevador2.setPower(motorpower);
            switch (liftState){
                case LIFT_START:
                if(gamepad2.a){
                    Elevador1.setTargetPosition(LIFT_ZERO);
                    Elevador2.setTargetPosition(LIFT_ZERO);
                    Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevador1.setPower(motorpower);
                    Elevador2.setPower(motorpower);
                    LIFT_GOING = LIFT_ZERO;
                    liftState = LIFT_END;
                }

                    if(gamepad2.b){
                        Elevador1.setTargetPosition(LIFT_LOW);
                        Elevador2.setTargetPosition(LIFT_LOW);
                        Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Elevador1.setPower(motorpower);
                        Elevador2.setPower(motorpower);
                        LIFT_GOING = LIFT_LOW;
                        liftState = LIFT_END;
                    }

                    if(gamepad2.x){
                        Elevador1.setTargetPosition(LIFT_MED);
                        Elevador2.setTargetPosition(LIFT_MED);
                        Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Elevador1.setPower(motorpower);
                        Elevador2.setPower(motorpower);
                        LIFT_GOING = LIFT_MED;
                        liftState = LIFT_END;
                    }

                    if(gamepad2.y){
                        Elevador1.setTargetPosition(LIFT_HIGH);
                        Elevador2.setTargetPosition(LIFT_HIGH);
                        Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Elevador1.setPower(motorpower);
                        Elevador2.setPower(motorpower);
                        LIFT_GOING = LIFT_HIGH;
                        liftState = LIFT_END;

                    }

                break;
                case LIFT_END:
                    if(Math.abs(Elevador1.getCurrentPosition() - LIFT_GOING) < 30){
                        liftState = LIFT_START;
                    }
                break;
                default:
                    liftState = LIFT_START;
            }

/*
            if (gamepad2.a){
                SlidePOS(0,.6);
            }
            if (gamepad2.b){
                SlidePOS(2353,.4);

            }
            if (gamepad2.x){
                SlidePOS(4000,.4);

            }
            if(gamepad2.y){
                SlidePOS(5600,.4);
            }
*/
            if (gamepad2.dpad_up){
                garraPoder=.5;
            }
            if (gamepad2.dpad_down){
                garraPoder=-.5;
            }
//            if(gamepad2.left_bumper){
//                poder_ele = .6;
//            }
//            if (gamepad2.right_bumper){
//                poder_ele=-.3;
//            }

            if (gamepad1.dpad_down) {
                drive.setMotorPowers(-.4,-.4,-.4,-.4);

            }
            if (gamepad1.dpad_left) {
                drive.setMotorPowers(-.4, .4, -.4, .4);
                 }
            if (gamepad1.dpad_right) {
                drive.setMotorPowers(.4, -.4, .4, -.4);

            }
            if (gamepad1.dpad_up) {
                drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);
            }


            // Update everything. Odometry. Etc.
            drive.update();
            garra.setPower(garraPoder);
//            Elevador1.setPower(poder_ele);
//            Elevador2.setPower(poder_ele);
            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Starting at",  "%7d :%7d",
                    Elevador1.getCurrentPosition(),
                    Elevador2.getCurrentPosition());

//            telemetry.addData("pos ", slidePos1, slidePos2);
            telemetry.addData("target ", target);
            telemetry.update();

            /*
            int slidePos1 = Elevador1.getCurrentPosition();
            int slidePos2 = Elevador2.getCurrentPosition();
            double pid1 = controller.calculate(slidePos1, target);
            double pid2 = controller.calculate(slidePos2, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

            double power = pid + ff;

            Elevador1.setPower(power);
            Elevador2.setPower(power);
            */


        }
    }
    /*
    private void SlidePOS(int slidePOS, double motorpower) {
        Elevador1.setTargetPosition(slidePOS);
        Elevador2.setTargetPosition(slidePOS);
        Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Elevador2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Elevador1.setPower(motorpower);
        Elevador2.setPower(motorpower);
        while (Elevador1.isBusy() || Elevador2.isBusy()) {
            telemetry.addData("Starting at",  "%7d :%7d",
                    Elevador1.getCurrentPosition(),
                    Elevador2.getCurrentPosition());

        }
    }
    */
}
