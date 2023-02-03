/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="Hola", group="Linear Opmode")

public class Prueba extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor UpLeft = null, UpRight = null, DownLeft = null, DownRight = null, Elevador1 = null, Elevador2 = null;
    private CRServo garra;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        UpLeft  = hardwareMap.get(DcMotor.class, "rightFront");
        UpRight = hardwareMap.get(DcMotor.class, "leftFront");
        DownLeft  = hardwareMap.get(DcMotor.class, "rightRear");
        DownRight  = hardwareMap.get(DcMotor.class, "leftRear");
        Elevador1  = hardwareMap.get(DcMotor.class, "ElevadorIzq");
        Elevador2  = hardwareMap.get(DcMotor.class, "ElevadorDer");
        Elevador2.setDirection(DcMotor.Direction.REVERSE);
        Elevador1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevador1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevador2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        garra = hardwareMap.get(CRServo.class, "garra");




        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        UpLeft.setDirection(DcMotor.Direction.REVERSE);
        DownLeft.setDirection(DcMotor.Direction.REVERSE);
        Elevador2.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double UpLeftPower = 0;
            double UpRightPower = 0;
            double DownLeftPower = 0;
            double DownRightPower = 0;
            double ElevadorPower = 0;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double acelerar = gamepad1.right_trigger;
            double regresar = gamepad1.left_trigger;
            double turn  =  -gamepad1.right_stick_x;
            double subir = gamepad2.right_trigger;
            double bajar = gamepad2.left_trigger;
            UpLeftPower   = Range.clip(acelerar - regresar + turn, -1.0, 1.0) ;
            UpRightPower   = Range.clip(acelerar - regresar - turn, -1.0, 1.0) ;
            DownLeftPower = Range.clip(acelerar - regresar + turn, -1.0, 1.0);
            DownRightPower = Range.clip(acelerar - regresar - turn, -1.0, 1.0);
            ElevadorPower = Range.clip(subir - bajar, -.20, .20);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;




            if (gamepad2.dpad_up){
                garra.setPower(1);
            }
            if (gamepad2.dpad_down){
                garra.setPower(-1);
            }


            if (gamepad1.dpad_down) {
                UpLeft.setPower(-.4);
                UpRight.setPower(-.4);
                DownLeft.setPower(-.4);
                DownRight.setPower(-.4);
            }
            if (gamepad1.dpad_left) {
                UpLeft.setPower(-.4);
                UpRight.setPower(.4);
                DownLeft.setPower(.4);
                DownRight.setPower(-.4);
            }
            if (gamepad1.dpad_right) {
                UpLeft.setPower(.4);
                UpRight.setPower(-.4);
                DownLeft.setPower(-.4);
                DownRight.setPower(.4);
            }
            if (gamepad1.dpad_up) {
                UpLeft.setPower(.4);
                UpRight.setPower(.4);
                DownLeft.setPower(.4);
                DownRight.setPower(.4);
            }
            // Send calculated power to wheels
            UpLeft.setPower(UpLeftPower);
            UpRight.setPower(UpRightPower);
            DownLeft.setPower(DownLeftPower);
            DownRight.setPower(DownRightPower);
            Elevador1.setPower(ElevadorPower);
            Elevador2.setPower(ElevadorPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Starting at",  "%7d :%7d",
                    Elevador1.getCurrentPosition(),
                    Elevador2.getCurrentPosition());


            telemetry.update();
        }
    }
}