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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="ServoTest2", group="Linear Opmode")
//@Disabled
public class servotest2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private  DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;
    private GyroSensor gyro;
    private ColorSensor color;
    private Servo rightramp;
    private Servo leftramp;
    private Servo starboardDoor;
    private Servo portDoor;
    private Servo jewelExcavator;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");
        gyro = hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        rightramp = hardwareMap.servo.get("servo1");
        leftramp = hardwareMap.servo.get("servo2");
        starboardDoor = hardwareMap.servo.get("servo3");
        portDoor = hardwareMap.servo.get("servo4");
        jewelExcavator = hardwareMap.servo.get("servo5");

        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            leftramp.setPosition(.4);
            sleep(2000);
            /*
            telemetry.addData("ramp", "active");
            telemetry.update();
            righramp.setPosition(.4);
            leftramp.setPosition(.4);
            sleep(2000);
            starboardDoor.setPosition(.4);
            portDoor.setPosition(.4);
            sleep(2000);
            */
        }
    }

}
