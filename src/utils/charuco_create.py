import numpy as np
import cv2
import os
import context

directory = context.get_context(os.path.abspath(__file__))

# ------------------------------
ARUCO_DICT = cv2.aruco.DICT_4X4_1000
SQUARES_X = 5          # columns
SQUARES_Y = 7          # rows
SQUARE_LENGTH = 0.025  # 25 mm (units: meters)
MARKER_LENGTH = 0.015  # 15 mm (units: meters)
WIDTH_PX = 2000        # output image width in pixels
MARGIN_PX = 40         # white margin around the board in pixels
SAVE_NAME = f"{directory}/utils/calibration_imgs/ChArUco_Marker.png"
# ------------------------------

def create_and_save_new_board():
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH,
        MARKER_LENGTH,
        dictionary
    )

    # preserve board aspect: width:height = SQUARES_X : SQUARES_Y
    height_px = int(WIDTH_PX * (SQUARES_Y / SQUARES_X))

    # generate image from board instance
    img = board.generateImage(
        (WIDTH_PX, height_px),
        marginSize=MARGIN_PX  # borderBits defaults to 1
    )

    cv2.imwrite(SAVE_NAME, img)
    # Optional preview
    cv2.imshow("ChArUco", img)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    create_and_save_new_board()
