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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Encoder2", group="Linear Opmode")
//@Disabled
public class usingEncoders extends LinearOpMode {

    // Declare OpMode members.
    public DcMotor leftMotor1;
    public DcMotor leftMotor2;
    public DcMotor rightMotor1;
    public DcMotor rightMotor2;


    @Override
    public void runOpMode() {
        leftMotor1 = hardwareMap.get(DcMotor.class, "left_motor_front");
        leftMotor2 = hardwareMap.get(DcMotor.class, "left_motor_back");
        rightMotor1 = hardwareMap.get(DcMotor.class, "right_motor_front");
        rightMotor2 = hardwareMap.get(DcMotor.class, "right_motor_back");

        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        moveForwardInchesAtPower(12, 1);
        // run until the end of the match (driver presses STOP)
    }

    public void setLeftRightPower(double leftPower, double rightPower){
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    public void setLeftPower(double leftPower){
        leftMotor1.setPower(leftPower);
        leftMotor2.setPower(leftPower);
    }
    public void setRightPower(double rightPower){
        rightMotor1.setPower(rightPower);
        rightMotor2.setPower(rightPower);
    }

    public final double WHEEL_DIAMETER = 4;
    public final double TICKS_PER_REVOLUTION = 1120;
    public final double PI = 3.141592653589793;

    public void moveForwardInchesAtPower(double inputInchesForward, double inputSpeed){
        int initialRightMotorPosition = rightMotor1.getCurrentPosition();
        int initialLeftMotorPosition  = leftMotor1.getCurrentPosition();
        int ticksToMove = (int) (TICKS_PER_REVOLUTION/(PI * WHEEL_DIAMETER) * inputInchesForward);
        int targetRightMotorPosition = initialRightMotorPosition + ticksToMove;
        int targetLeftMotorPosition  = initialLeftMotorPosition  + ticksToMove;
        boolean isInLeftRange = true;
        boolean isInRightRange = true;
        while(isInLeftRange || isInRightRange){
            if(isInRightRange){
                setRightPower(inputSpeed);
            }
            else{
                setRightPower(0);
            }
            if(isInLeftRange){
                setLeftPower(inputSpeed);
            }
            else{
                setLeftPower(0);
            }
            isInRightRange = rightMotor1.getCurrentPosition() < targetRightMotorPosition;
            isInLeftRange  = leftMotor1.getCurrentPosition() < targetLeftMotorPosition;
        }
        setLeftRightPower(0, 0);
    }
}