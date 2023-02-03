package org.firstinspires.ftc.teamcode.drive.advanced;

import static org.firstinspires.ftc.teamcode.drive.advanced.TeleOpDrive.LiftState.LIFT_END;
import static org.firstinspires.ftc.teamcode.drive.advanced.TeleOpDrive.LiftState.LIFT_START;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
public class TeleOpDrive extends LinearOpMode {
    private DcMotor Elevador1=null;
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
    double motorpower = 0.6;

    @Override

    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Elevador1  = hardwareMap.get(DcMotor.class, "Elevador");

        garra = hardwareMap.get(CRServo.class, "garra");

        Elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Elevador1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Elevador1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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



        waitForStart();
        LiftState liftState = LIFT_START;
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            double garraPoder=0;
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            switch (liftState){
                case LIFT_START:
                    if(gamepad2.a){
                        Elevador1.setTargetPosition(LIFT_ZERO);
                        Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Elevador1.setPower(0.45);
                        LIFT_GOING = LIFT_ZERO;
                        liftState = LiftState.LIFT_END;
                    }

                    if(gamepad2.b){
                        Elevador1.setTargetPosition(LIFT_LOW);
                        Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Elevador1.setPower(motorpower);
                        LIFT_GOING = LIFT_LOW;
                        liftState = LiftState.LIFT_END;
                    }

                    if(gamepad2.x){
                        Elevador1.setTargetPosition(LIFT_MED);
                        Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Elevador1.setPower(motorpower);
                        LIFT_GOING = LIFT_MED;
                        liftState = LiftState.LIFT_END;
                    }

                    if(gamepad2.y){
                        Elevador1.setTargetPosition(LIFT_HIGH);
                        Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Elevador1.setPower(motorpower);
                        LIFT_GOING = LIFT_HIGH;
                        liftState = LiftState.LIFT_END;

                    }

                    break;
                case LIFT_END:
                    if(Math.abs(Elevador1.getCurrentPosition() - LIFT_GOING) < 30){
                        liftState = LiftState.LIFT_START;
                    }
                    break;
                default:
                    liftState = LiftState.LIFT_START;
            }
            if (gamepad2.right_bumper){
                Elevador1.setTargetPosition(Elevador1.getCurrentPosition()+50);
                Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Elevador1.setPower(0.35);
            }
            if (gamepad2.left_bumper){
                Elevador1.setTargetPosition(Elevador1.getCurrentPosition()-50);
                Elevador1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Elevador1.setPower(0.35);
            }
            if (gamepad2.dpad_up){
                garraPoder=.5;
            }
            if (gamepad2.dpad_down){
                garraPoder=-.5;
            }
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

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
