package com.team254.devboard.controlboard;

import com.team254.devboard.Constants;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Joystick;

public class MainDriveControlBoard implements IDriveControlBoard {
    private static MainDriveControlBoard mInstance = null;

    public static MainDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainDriveControlBoard();
        }

        return mInstance;
    }

    private MainDriveControlBoard() {
    }
}