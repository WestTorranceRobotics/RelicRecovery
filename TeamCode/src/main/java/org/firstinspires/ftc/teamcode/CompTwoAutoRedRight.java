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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="comp2AutoRedRight", group="Linear Opmode")
//@Disabled
public class CompTwoAutoRedRight extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;
    private DcMotor lift;
    private GyroSensor gyro;
    private ColorSensor color;
    private Servo rightRamp;
    private Servo leftRamp;
    private Servo rightDoor;
    private Servo leftDoor;
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

        //init lift
        sleep(2000);
        lift.setPower(0.1);
        sleep(4000);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(0);
        //lift has been init-ed

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        lift.setTargetPosition(-500);
        lift.setPower(0.6);
        runtime.reset();
        final double JEWEL_EXCAVATOR_SERVO_ARM_POSITION = 0.6;
        final double RED_THRESHOLD = 5;

        jewelExcavator.setPosition(JEWEL_EXCAVATOR_SERVO_ARM_POSITION);
        sleep(1000);
        if (color.red() >= RED_THRESHOLD){
            setLRPower(.5, .5, 300);
            stopRobot();
            sleep(1000);
            jewelExcavator.setPosition(0);
            sleep(3000);
            setLRPower(-1, -1, 1700);
        }
        else{
            setLRPower(-1, -1, 300);
            stopRobot();
            sleep(1000);
            jewelExcavator.setPosition(0);
            sleep(3000);
            setLRPower(-1, -1, 1000);
        }
        /*
        stopRobot();

        leftRamp.setPosition(1);
        rightRamp.setPosition(0);
        leftDoor.setPosition(0.5);
        rightDoor.setPosition(0.5);

        sleep(1000);
        setLRPower(1, 1, 200);
        setLRPower(-1, -1, 200);
        stopRobot();

        leftRamp.setPosition(0);
        rightRamp.setPosition(1);
        leftDoor.setPosition(1);
        rightDoor.setPosition(0);

        setLRPower(1, 1, 200);
        stopRobot(); */

        telemetry.addData("redvalue", color.red());
    }
    void setLRPower(double l, double r, long s){
        left1.setPower(l);
        left2.setPower(l);
        right1.setPower(r);
        right2.setPower(r);
        sleep(s);
    }
    void stopRobot(){
        setLRPower(0, 0, 0);
    }
}
