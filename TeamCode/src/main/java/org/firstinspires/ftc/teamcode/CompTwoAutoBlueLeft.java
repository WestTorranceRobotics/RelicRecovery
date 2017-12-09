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


@Autonomous(name="comp2AutoBlueLeft", group="Linear Opmode")
//@Disabled
public class CompTwoAutoBlueLeft extends LinearOpMode {

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
        final double RED_THRESHOLD = 10;

        jewelExcavator.setPosition(JEWEL_EXCAVATOR_SERVO_ARM_POSITION);
        sleep(1000);
        if (color.red() >= RED_THRESHOLD){
            moveForwardInches(-1, 6);
            stopRobot();
            sleep(1000);
            jewelExcavator.setPosition(0);
            sleep(1000);
            moveForwardInches(1, 28);
        }
        else{
            moveForwardInches(1, 6);
            stopRobot();
            sleep(1000);
            jewelExcavator.setPosition(0);
            sleep(3000);
            moveForwardInches(1, 16);
        }
        stopRobot();
    }
    void moveForwardInches(double p, double i){
        //init wheel stats
        final double PI = 3.14159265358979;
        final double WHEEL_D = 4;
        final int ENCODER_TICKS_PER_R = 1120;
        //determine target
        int LtargEncPos = left1.getCurrentPosition() + (int) (i * ENCODER_TICKS_PER_R / PI / WHEEL_D);
        int RtargEncPos = right1.getCurrentPosition() + (int) (i * ENCODER_TICKS_PER_R / PI / WHEEL_D);
        //set modes
        if (!left1.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
            left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (!right1.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
            right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (!left2.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
            left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (!right2.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
            right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        left1.setTargetPosition(LtargEncPos);
        right1.setTargetPosition(RtargEncPos);
        setLPower(p);
        setRPower(p);
        while(left1.isBusy() || right1.isBusy() && opModeIsActive()){
            if (!left1.isBusy()){
                setLPower(0);
            }
            if (!right1.isBusy()){
                setRPower(0);
            }
        }
        stopRobot();
    }
    void stopRobot(){
        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);
    }
    void setLPower(double l){
        left1.setPower(l);
        left2.setPower(l);
    }
    void setRPower(double l){
        right1.setPower(l);
        right2.setPower(l);
    }
}