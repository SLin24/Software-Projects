package com.team254.devboard.controlboard;

import com.team254.devboard.Constants;
import com.team254.devboard.controlboard.XboxController.Button;
import com.team254.lib.geometry.Rotation2d;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadDriveControlBoard() {
        mController = new XboxController(Constants.kDriveGamepadPort);
    }
}