package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Right1"/*, preselectTeleOp="Field Centric (main)"*/)
public class AutoRight1 extends Auto {
	/**
	 * This function is executed when this Op Mode is initialized from the Driver Station.
	 */
	@Override
	public void runOpMode() {
		Initialize();
		MotorSetup();

		waitForStart();

		if (opModeIsActive()) {
			// Raise the arm to the high bar height
			
			arm.Move(BotConfig.BAR_HEIGHT, false);
			// Move the wrist to place the specimen
			intake.MoveWrist(900, false);
			// Drive towards the high bar
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 0.9 ));
			// Wait half a second before moving arm downwards
			arm.Move(0, false);
			
			
			// Wait 2 seconds so the arm can go all the way down
			sleep(1000);
			// Open the claw & move wrist down
			intake.OpenClaw();
			intake.MoveWrist(0, false);
			sleep(500);
			// Move towards the 3 colored samples
			driveMotors.Move(Direction.RIGHT, (int)( BotConfig.TILE_LENGTH * 1.3 ));
			driveMotors.Turn(180);
			rotateTo(180);
			driveMotors.Move(Direction.BACKWARD, (int)( BotConfig.TILE_LENGTH * 1.1 ));
			driveMotors.Move(Direction.LEFT, (int)( BotConfig.TILE_LENGTH * 0.6 ));
			// Push block into observation zone
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 1.5 ));
			driveMotors.Move(Direction.BACKWARD, (int)( BotConfig.TILE_LENGTH * 0.3 ));
			// Move wrist to pick up specimen
			
			intake.MoveWrist(BotConfig.SPECIMEN_HEIGHT, true);
			intake.CloseClaw();
			
			sleep(500);
			
			intake.MoveWrist(700, false);
			
			// Move right towards the high bar
			arm.Move(BotConfig.BAR_HEIGHT, false);
			driveMotors.Move(Direction.RIGHT, (int)( BotConfig.TILE_LENGTH * 2 ));
			driveMotors.Turn(180);
			
			
			// Move the wrist to place the specimen
			intake.MoveWrist(900, false);
			// Drive towards the high bar
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 0.3 ));
			// Wait half a second before moving arm downwards
			arm.Move(0, false);
			
			sleep(5000);
			
		}
		saveHeading();
	}
}

