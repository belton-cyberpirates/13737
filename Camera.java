package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config;


public class Camera {
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

	private static final int CameraResoX = 640;

	// TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
	private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/teampiece.tflite";

	// Define the labels recognized in the model for TFOD (must be in training order!)
	private static final String[] LABELS = {
		"BLUE",
		"RED",
	};

    public void Init() {

		// Create the TensorFlow processor by using a builder.
		tfod = new TfodProcessor.Builder()
			.setModelFileName(TFOD_MODEL_FILE)
			.setModelLabels(LABELS)
			.build();

		// Create the vision portal by using a builder.
		VisionPortal.Builder builder = new VisionPortal.Builder();

		// Set the camera (webcam vs. built-in RC phone camera).
		builder.setCamera(hardwareMap.get(WebcamName.class, Config.CAMERA_CONFIG_NAME));

		// Set and enable the processor.
		builder.addProcessor(tfod);

		// Build the Vision Portal, using the above settings.
		visionPortal = builder.build();
	}

	public int GetPropPos() { // 0 = left, 1 = center, 2 = right
		Recognition recognition = highestConfidence();

		if (recognition != null) {
			double propX = (recognition.getLeft() + recognition.getRight()) / 2 ;

			if ((CameraResoX / 2) > propX) {
				return 1;
			}
			return 2;
		}
		return 0;
	}

	private Recognition highestConfidence() {
		Recognition recognition = null;

		for (int i = 0; i <= 1000; i++) {
			List<Recognition> currentRecognitions = tfod.getRecognitions();

			if (currentRecognitions.size() < 1) {
				sleep(10);
				continue;
			}
			
			// Select highest confidence object
			for (Recognition potentialRecognition : currentRecognitions) {

				if (recognition != null && recognition.getConfidence() > potentialRecognition.getConfidence()) {
					continue;
				}
				recognition = potentialRecognition;
			}
			return recognition;
		}
		return recognition;
	}

}