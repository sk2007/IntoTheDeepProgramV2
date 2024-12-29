
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.checkerframework.checker.units.qual.A;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="BOPMode")
public class BOPMode extends LinearOpMode {

    // Create variables
    private DcMotor MOTOR1;
    private DcMotor MOTOR2;
    private DcMotor MOTOR3;
    private DcMotor MOTOR4;
    private static DcMotor ARM_LIFT;
    private static DcMotor ARM_JOINT;
    private Servo intakeOpening, intakeUpDown, intakeRotate;

    @Override
    public void runOpMode() {

        // Initialize hardware variables
        // DC Motors
        MOTOR1 = hardwareMap.get(DcMotor.class, "MOTOR1");
        MOTOR2 = hardwareMap.get(DcMotor.class, "MOTOR2");
        MOTOR3 = hardwareMap.get(DcMotor.class, "MOTOR3");
        MOTOR4 = hardwareMap.get(DcMotor.class, "MOTOR4");
        ARM_LIFT = hardwareMap.get(DcMotor.class, "ARM-LIFT");
        ARM_JOINT = hardwareMap.get(DcMotor.class, "ARM-JOINT");

        // Servo Motors
        intakeUpDown = hardwareMap.get(Servo.class, "INTAKE-OPEN");
        intakeOpening = hardwareMap.get(Servo.class, "INTAKE-UD");
        intakeRotate = hardwareMap.get(Servo.class, "INTAKE-R");

        // DcMotor constants
        double MotorPower = 0.0;
        double reduceSpeedFactor = 0.8;

        // Open Close
        double intakeOpenPos = 0.55;
        double intakeClosePos = 0.2;

        // Up Down
        double intakeUpDownMax = 0.75;
        double intakeUpDownMin = 0;
        double intakeUpDownHomePos = 0.2;   // Inside the robot to init
        double intakeUpDownPickUp = 0.5;
        double intakeUpDownStraightPos = 0.406;
        double intakeUpDownGrabFromWall = 0.51;

        // Rotate
        double intakeRotateHomePos = 0.58;
        double intakeRotateMin = 0;
        double intakeRotateMax = 1;
        double intakeRotateFlipped = 0;

        // Arm joint
        int jointHomePos = 350;
        int jointDeliverPos1 = -1800;          // Pos 1 = High basket
        int jointDeliverPos2 = -2500;          // Pos 3 = Low basket
        int jointDeliverPos3 = -760;           // Pos 3 = High Bar
        int jointDeliverPos4 = -280;           // Pos 5 = Low Bar
        int jointPickUpPos   = -790;           // Height to pick up specimens in the middle
        int jointPickUpWallPos = -300;         // Height to pick up specimens on the wall

        // For climbing
        int stage           = 0;

        // Arm Lift
        int liftHomePos     = 0;
        int liftDeliverPos1 = -9680;            // Pos 1 = High basket
        int liftDeliverPos2 = -8470;            // Pos 3 = Low basket
        int liftDeliverPos3 = -9485;            // Pos 3 = High Bar
        int liftDeliverPos4 = -10150;           // Pos 5 = Low Bar
        int liftPickUpPos   = -3035;            // Height to pick up specimens in the middle
        int liftPickUpWallPos = -1730;          // Height to pick up specimens on the wall

        int[] liftDeliverPositions = {liftDeliverPos1, liftDeliverPos2, liftDeliverPos3, liftDeliverPos4};  // Positions for lift
        int[] jointDeliverPositions = {jointDeliverPos1, jointDeliverPos2, jointDeliverPos3, jointDeliverPos4}; // Positions for joint

        int currentArmPos = 0;                  // Tracks arm position - starts at home (0)

        // Set up motors and encoders
        MOTOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARM_LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARM_JOINT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MOTOR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ARM_LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ARM_JOINT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MOTOR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MOTOR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MOTOR3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MOTOR4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ARM_LIFT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ARM_JOINT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set claw home positions on init
        intakeOpening.setPosition(intakeOpenPos);
        intakeUpDown.setPosition(intakeUpDownHomePos);
        intakeRotate.setPosition(intakeRotateHomePos);

        // Init joint "home" - higher position to fit dimensions
        while (ARM_JOINT.getCurrentPosition() <= jointHomePos - 100 || ARM_JOINT.getCurrentPosition() >= jointHomePos + 100) {
            ARM_JOINT.setTargetPosition(jointHomePos);

            ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
            else ARM_JOINT.setPower(0);
        }

        // Init lift "home"
        while (ARM_LIFT.getCurrentPosition() >= liftHomePos + 100 || ARM_LIFT.getCurrentPosition() <= liftHomePos - 100) {
            ARM_LIFT.setTargetPosition(liftHomePos);

            ARM_LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (ARM_LIFT.isBusy()) ARM_LIFT.setPower(1);
            else ARM_LIFT.setPower(0);
        }

        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Game-pad 1

            // Driving
            if (gamepad1.right_stick_y != 0)                           // Robot Movement: Forward and Backward (flipped)
            {
                MotorPower = -gamepad1.right_stick_y * reduceSpeedFactor;

                MOTOR1.setPower(-MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(MotorPower);
                MOTOR4.setPower(MotorPower);
            } else if (gamepad1.right_stick_x != 0)                // Robot Movement: Turning
            {
                MotorPower = gamepad1.right_stick_x * reduceSpeedFactor;

                MOTOR1.setPower(-MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(-MotorPower);
            } else if (gamepad1.left_stick_x != 0)                 // Robot Movement: Strafing
            {
                MotorPower = -gamepad1.left_stick_x * reduceSpeedFactor;

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(MotorPower);

            } else {
                MotorPower = 0.0;                                      // Default power to zero

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(MotorPower);
                MOTOR3.setPower(MotorPower);
                MOTOR4.setPower(MotorPower);
            }


            // Open/Close Claw
            if (gamepad1.a) {
                if (intakeOpening.getPosition() == intakeOpenPos) {
                    intakeOpening.setPosition(intakeClosePos);
                    sleep(200);
                } else {
                    intakeOpening.setPosition(intakeOpenPos);
                    sleep(200);
                }
            }

            // Release specimen onto the wall
            if (gamepad1.left_bumper) {
                ARM_JOINT.setTargetPosition(ARM_JOINT.getCurrentPosition() + 600);          // Slam the arm down
                ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                else ARM_JOINT.setPower(0);


                sleep(500);
                intakeOpening.setPosition(intakeOpenPos);                                   // Open the claw
            }

            // Take specimen off the wall
            if (gamepad1.right_bumper) {
                intakeOpening.setPosition(intakeClosePos);                                  // Close the claw

                sleep(500);
                intakeUpDown.setPosition(intakeUpDown.getPosition() - 0.1);                 // Lift the claw off the wall

            }

            // Hang Bot
            if (gamepad1.y) {
                if (stage == 0){
                    intakeUpDown.setPosition(intakeUpDownMax);                              // Tuck in the claw
                    intakeRotate.setPosition(intakeRotateFlipped);
                    sleep(200);

                    ARM_LIFT.setTargetPosition(-7700);                                      // Initial pos on the ground
                    ARM_LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (ARM_LIFT.isBusy()) ARM_LIFT.setPower(1);
                    else ARM_LIFT.setPower(0);
                    sleep(1000);

                    ARM_JOINT.setTargetPosition(200);
                    ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                    else ARM_JOINT.setPower(0);

                    sleep(200);

                    stage = 1;
                } else {
                    ARM_LIFT.setTargetPosition(-1342);                                     // Lift up to 1st stage
                    ARM_LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (ARM_LIFT.isBusy()) ARM_LIFT.setPower(1);
                    else ARM_LIFT.setPower(0);
                    sleep(2000);

                    ARM_JOINT.setTargetPosition(350);
                    ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                    else ARM_JOINT.setPower(0);

                    sleep(2000);                                               // Release slightly onto the bar
                    ARM_LIFT.setTargetPosition(ARM_LIFT.getCurrentPosition() - 600);
                    ARM_LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (ARM_LIFT.isBusy()) ARM_LIFT.setPower(1);
                    else ARM_LIFT.setPower(0);

                    sleep(3000);

                    ARM_LIFT.setTargetPosition(-6550);                                     // Move to top before to prep hang
                    ARM_LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (ARM_LIFT.isBusy()) ARM_LIFT.setPower(1);
                    else ARM_LIFT.setPower(0);
                    sleep(1500);

                    ARM_JOINT.setTargetPosition(-1217);
                    ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                    else ARM_JOINT.setPower(0);

                    sleep(200);
                }

            }

            // Move Claw Angle
            while (gamepad1.dpad_up) {                                                  // Move claw Up
                if (intakeUpDown.getPosition() < intakeUpDownMax)
                    intakeUpDown.setPosition(intakeUpDown.getPosition() + 0.0006);
            }
            while (gamepad1.dpad_down) {                                        // Move Down
                if (intakeUpDown.getPosition() > intakeUpDownMin)
                    intakeUpDown.setPosition(intakeUpDown.getPosition() - 0.0006);
            }
            while (gamepad1.dpad_right) {
                if (intakeRotate.getPosition() > intakeRotateMin)               // Turn Right
                    intakeRotate.setPosition(intakeRotate.getPosition() - 0.0006);
            }
            while (gamepad1.dpad_left) {
                if (intakeRotate.getPosition() < intakeRotateMax)               // Turn Left
                    intakeRotate.setPosition(intakeRotate.getPosition() + 0.0006);
            }


            // Game-pad 2

            // Deliver Specimen to basket
            if (gamepad2.y) {
                // Cycle between 1 (high) and 2 (low) positions
                currentArmPos = (currentArmPos == 1) ? 2 : 1;

                // Set target positions for the arm
                ARM_LIFT.setTargetPosition(liftDeliverPositions[currentArmPos - 1]);

                // Set motor modes to RUN_TO_POSITION for controlled movement
                ARM_LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (ARM_LIFT.isBusy()) ARM_LIFT.setPower(1);
                else ARM_LIFT.setPower(0);
                sleep(1500);

                ARM_JOINT.setTargetPosition(jointDeliverPositions[currentArmPos - 1]);
                ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                else ARM_JOINT.setPower(0);

                sleep(200);

                intakeUpDown.setPosition(intakeUpDownStraightPos);                  // straight upwards

            }

            // Bring "Home"
            if (gamepad2.a) {

                ARM_JOINT.setTargetPosition(jointHomePos-250);
                ARM_LIFT.setTargetPosition(liftHomePos);


                ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ARM_LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                ARM_JOINT.setTargetPosition(jointHomePos-250);
                ARM_LIFT.setTargetPosition(liftHomePos);

                if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                else ARM_JOINT.setPower(0);

                sleep(1000);

                if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                else ARM_JOINT.setPower(0);

                intakeRotate.setPosition(intakeRotateHomePos);
                intakeUpDown.setPosition(intakeUpDownHomePos);
            }

            // Move to Cage Positions
            if (gamepad2.x) {
                if (currentArmPos != 3) { //working?
                    currentArmPos = 3;
                } else {
                    currentArmPos = 4;
                }

                // Set target positions for the arm
                ARM_LIFT.setTargetPosition(liftDeliverPositions[currentArmPos - 1]);
                ARM_LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (ARM_LIFT.isBusy()) ARM_LIFT.setPower(1);
                else ARM_LIFT.setPower(0);
                sleep(900);

                ARM_JOINT.setTargetPosition(jointDeliverPositions[currentArmPos - 1]);
                ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                else ARM_JOINT.setPower(0);

                sleep(200);

                intakeRotate.setPosition(intakeRotateFlipped);
                if(currentArmPos == 4 )intakeUpDown.setPosition(0.585);
                else intakeUpDown.setPosition(0.5); //check


            }

            // Positioning to pick up specimens from center box
            if (gamepad2.right_bumper) {

                // Set target positions for the arm
                ARM_LIFT.setTargetPosition(liftPickUpPos);
                ARM_LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (ARM_LIFT.isBusy()) ARM_LIFT.setPower(1);
                else ARM_LIFT.setPower(0);
                sleep(1000);

                ARM_JOINT.setTargetPosition(jointPickUpPos);
                ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                else ARM_JOINT.setPower(0);

                intakeUpDown.setPosition(intakeUpDownPickUp);
                intakeRotate.setPosition(intakeRotateHomePos);
            }

            // Arm Position to pick up specimens from wall
            if (gamepad2.left_bumper) {


                intakeUpDown.setPosition(intakeUpDownGrabFromWall);
                intakeRotate.setPosition(intakeRotateHomePos);


            }

            // Adjust Joint Position
            if (gamepad2.dpad_up){

                ARM_JOINT.setTargetPosition(ARM_JOINT.getCurrentPosition() - 100);
                ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                else ARM_JOINT.setPower(0);

            }
            if (gamepad2.dpad_down){

                ARM_JOINT.setTargetPosition(ARM_JOINT.getCurrentPosition() + 100);
                ARM_JOINT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (ARM_JOINT.isBusy()) ARM_JOINT.setPower(1);
                else ARM_JOINT.setPower(0);

            }

            telemetry.addData("Claw up down: ", intakeUpDown.getPosition());
            telemetry.addData("Claw rotate: ", intakeRotate.getPosition());
            telemetry.addData("Arm Lift: ", ARM_LIFT.getCurrentPosition());
            telemetry.addData("Arm Joint: ", ARM_JOINT.getCurrentPosition());

            telemetry.update();



        }
    }
}




//TODO:

//Odometry
//left is 1
//middle is 2
//right is 3 (claw as front)