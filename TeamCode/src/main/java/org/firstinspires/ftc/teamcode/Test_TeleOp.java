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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="New TeleOp Test for RR", group="Linear Opmode")
//@Disabled
public class Test_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;
    private DcMotorEx lift;
    private GyroSensor gyro;
    private ColorSensor color;
    private Servo rightRamp;
    private Servo leftRamp;
    private Servo rightDoor;
    private Servo leftDoor;
    public DcMotor intakeMotorRight;
    public DcMotor intakeMotorLeft;
    //color sensor arm \/
    private Servo jewelExcavator;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("left1"); //1
        left2 = hardwareMap.dcMotor.get("left2"); //1
        right1 = hardwareMap.dcMotor.get("right1"); //1
        right2 = hardwareMap.dcMotor.get("right2"); //1
        intakeMotorRight = hardwareMap.dcMotor.get("intake2");
        intakeMotorLeft = hardwareMap.dcMotor.get("intake1");
        lift = (DcMotorEx) hardwareMap.dcMotor.get("lift"); //2
        gyro = hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        rightRamp = hardwareMap.servo.get("servo2"); //3
        leftRamp = hardwareMap.servo.get("servo1"); //3
        rightDoor = hardwareMap.servo.get("servo3"); //3
        leftDoor = hardwareMap.servo.get("servo4"); //3
        jewelExcavator = hardwareMap.servo.get("servo5");

        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        //rightRamp.setDirection(Servo.Direction.REVERSE);
        //rightDoor.setDirection(Servo.Direction.REVERSE);

        //init lift
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(2000);
        lift.setPower(-0.1);
        sleep(4000);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(0);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //lift has been init-ed

        leftRamp.setPosition(1);
        rightRamp.setPosition(0);
        leftDoor.setPosition(0.5);
        rightDoor.setPosition(0.5);

        waitForStart();
        runtime.reset();

        boolean liftUp = false;
        boolean gamepadAWas = false;
        boolean gamepadAIs;

        boolean flipUp = false;
        boolean gamepadBWas = false;
        boolean gamepadBIs;


        while (opModeIsActive()) {
            //Part 1
            if(Math.abs(gamepad1.left_stick_y) < 0.15){
                setLeftPower(0);
            }
            else {
                setLeftPower(gamepad1.left_stick_y);
            }
            if(Math.abs(gamepad1.right_stick_y) < 0.15){
                setRightPower(0);
            }
            else{
                setRightPower(gamepad1.right_stick_y);
            }

//game pad 2
            if (Math.abs(gamepad2.left_stick_y) < .15){
                intakeMotorRight.setPower(0);
            }
            else if (Math.abs(gamepad2.left_stick_y) < .75){
                intakeMotorRight.setPower(gamepad2.left_stick_y);
            }
            else {
                intakeMotorLeft.setPower(.75);
            }

            if (Math.abs(gamepad2.right_stick_y) < .15){
                intakeMotorLeft.setPower(0);
            } else if (Math.abs(gamepad2.right_stick_y) < .75){
                intakeMotorLeft.setPower(gamepad2.right_stick_y);
            }
            else {
                intakeMotorRight.setPower(0.75);
            }
            //Part 2
            /*
            gamepadAIs = gamepad2.a;
            if(!gamepadAWas && gamepadAIs ? !liftUp : liftUp){
                gamepadAWas = gamepadAIs;
                setLiftPowerUp();
                liftUp = true;
            }
            else{
                gamepadAWas = gamepadAIs;
                lift.setPower(-0.05);
                lift.setTargetPosition(0);
                liftUp = false;
            }
            */
            //PIDCoefficients topPID = new PIDCoefficients(10, 1, 0);
            //lift.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, topPID);
            if(gamepad1.a & !gamepad1.b && !liftUp){
                lift.setTargetPosition(-1100);
                lift.setPower(1);
                liftUp = true;
            }
            if(!gamepad1.a & gamepad1.b && liftUp){
                lift.setTargetPosition(0);
                lift.setPower(0.6);
                liftUp = false;
            }
            //press x to reset; press y to dump

            if(gamepad1.x & !gamepad1.y && !flipUp){
                leftRamp.setPosition(1);
                rightRamp.setPosition(0);
                leftDoor.setPosition(0.5);
                rightDoor.setPosition(0.5);
                flipUp = true;
            }
            if(!gamepad1.x & gamepad1.y && flipUp){
                leftRamp.setPosition(0);
                rightRamp.setPosition(1);
                leftDoor.setPosition(1);
                rightDoor.setPosition(0);
                flipUp = false;
            }

            if(jewelExcavator.getPosition() != 0){
                jewelExcavator.setPosition(0);
            }

            //PIDCoefficients pidOriginal = lift.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("rightRamp", rightRamp.getPosition());
            telemetry.addData("leftRamp", leftRamp.getPosition());
            telemetry.addData("leftDoor", leftDoor.getPosition());//port
            telemetry.addData("rightDoor", rightDoor.getPosition());//star
            telemetry.update();
            //Part 3
            /*
            gamepadBIs = gamepad2.b;
            if(!gamepadBWas && gamepadBIs ? !flipUp : flipUp){
                gamepadBWas = gamepadBIs;
                dumpOut();
                flipUp = true;
            }
            else{
                gamepadBWas = gamepadBIs;
                dumperIn();
                flipUp = false;
            }
            */
            //Part 4?
        }
    }

    void setLiftPowerUp(){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.5);
        lift.setTargetPosition(-1100);
    }
    void setLeftPower(double n){
        left1.setPower(n);
        left2.setPower(n);
    }
    void setRightPower(double n){
        right1.setPower(n);
        right2.setPower(n);
    }
    void setLRPower(double l, double r){
        setLeftPower(l);
        setRightPower(r);
    }
    void dumpOut(){
        moveToward(leftRamp, 0.5);
        moveToward(rightRamp, 0.5);
        moveToward(rightDoor, 0.5);
        moveToward(leftDoor, -0.5);
    }
    void dumperIn(){
        moveToward(leftRamp, 0);
        moveToward(rightRamp, 0);
        moveToward(rightDoor, 0);
        moveToward(leftDoor, 0);
    }
    void moveToward(Servo s, double position){
        double change = position - s.getPosition();
        if(Math.abs(change) > 0.005){
            int direction = (int) (Math.abs(change)/change);
            s.setPosition(s.getPosition() + direction * 0.005);
        }
        else{
            s.setPosition(position);
        }
    }
}