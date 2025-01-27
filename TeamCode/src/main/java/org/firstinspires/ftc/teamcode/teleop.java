package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

    /*
     * This OpMode illustrates the concept of driving a path based on time.
     * The code is structured as a LinearOpMode
     *
     * The code assumes that you do NOT have encoders on the wheels,
     *   otherwise you would use: RobotAutoDriveByEncoder;
     *
     *   The desired path in this example is:
     *   - Drive forward for 3 seconds
     *   - Spin right for 1.3 seconds
     *   - Drive Backward for 1 Second
     *
     *  The code is written in a simple form with no optimizations.
     *  However, there are several ways that this type of sequence could be streamlined,
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
     */

    @Autonomous(name="Robot: Auto Drive By Time", group="Robot")
//    @Disabled
    public class teleop extends RobotLinearOpMode {

        /* Declare OpMode members. */
        public DcMotor  leftFrontDrive   = null; //the left front drivetrain motor
        public DcMotor  rightFrontDrive  = null; //the right front drivetrain motor
        public DcMotor  rightBackDrive  = null; //the right back drivetrain motor
        public DcMotor  leftBackDrive  = null; //the left back drivetrain motor
        public DcMotor  armMotor    = null; //the arm motor
        public DcMotor  VSlide   = null; //the left arm motor
        public CRServo intake = null;



        private ElapsedTime runtime = new ElapsedTime();


        static final double     FORWARD_SPEED = 0.6;
        static final double     TURN_SPEED    = 0.5;

        @Override
        public void runOpMode() {

            // Initialize the drive system variables.
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive"); //the left front drivetrain motor
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //the right front drivetrain motor
            leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); //the left drivetrain motor
            rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive"); //the left drivetrain motor
            armMotor  = hardwareMap.get(DcMotor.class, "arm_motor"); //the arm motor
            VSlide = hardwareMap.get(DcMotor.class, "vslide");
            intake = hardwareMap.get(CRServo.class, "intake");

            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        /*
        Forward 3s
        Turn for 1 sec
        Forward 3s again

        Turn 90,
        Forward 3s

        int speed = 5;
        waitForStart();
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while(opModeIsActive() and (runtime.seconds() < 3.0)) {
            sleep(0.1);
        }

        runtime.reset();

        leftDrive.setPower(5);
        rightDrive.setPower(-5);
        while(opModeIsActive() and runtime.seconds() < 1.0) {
            sleep(0.1);
        }

        runtime.reset();

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while(opModeIsActive() and (runtime.seconds() < 3.0)) {
            sleep(0.1);
        }

         */
            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

            // Step 1:  Drive forward for 3 seconds
            leftFrontDrive.setPower(FORWARD_SPEED);
            rightFrontDrive.setPower(FORWARD_SPEED);
            leftBackDrive.setPower(FORWARD_SPEED);
            rightBackDrive.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Spin right for 1.3 seconds
            leftFrontDrive.setPower(TURN_SPEED);
            rightFrontDrive.setPower(-TURN_SPEED);
            leftBackDrive.setPower(TURN_SPEED);
            rightBackDrive.setPower(-TURN_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 3:  Drive Backward for 1 Second
            leftFrontDrive.setPower(-FORWARD_SPEED);
            rightFrontDrive.setPower(-FORWARD_SPEED);
            leftBackDrive.setPower(-FORWARD_SPEED);
            rightBackDrive.setPower(-FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            // step 4: (unlabeled but assumed to be turn)
            leftFrontDrive.setPower(TURN_SPEED);
            rightFrontDrive.setPower(-TURN_SPEED);
            leftBackDrive.setPower(TURN_SPEED);
            rightBackDrive.setPower(-TURN_SPEED);
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            // Step 5:  Stop
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }

}
