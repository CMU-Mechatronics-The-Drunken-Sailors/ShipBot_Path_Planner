import cv2
import numpy as np

from ..CV_Module.nn import apply_nn, NN_Labels
from ..CV_Module.breaker import detect_breaker_state, BreakerState
from ..CV_Module.valve import get_rotary_valve_position, get_spigot_valve_position, get_stopcock_valve_position

from ..Command_Hub import *

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

def test_realsense():
    while 1:
        frame, _ = get_image()
        results = apply_nn(frame)
        
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) # convert colorspace

        for x1, y1, x2, y2, _, class_id in results.xyxy[0]:
            # Convert from pytorch tensor to int
            class_id = int(class_id)
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)

            # Check that width and height are greater than zero
            if x2 - x1 > 0 and y2 - y1 > 0:

                if class_id == NN_Labels.BREAKER:
                    res = detect_breaker_state(frame, (x1, y1, x2, y2))
                    if res is not None:
                        breaker_state, bx, by = res

                        if breaker_state == BreakerState.UP:
                            color = (255, 0, 0)
                        elif breaker_state == BreakerState.DOWN:
                            color = (0, 255, 0)
                        elif breaker_state == BreakerState.UP_UPSIDE_DOWN:
                            color = (0, 0, 255)
                        elif breaker_state == BreakerState.DOWN_UPSIDE_DOWN:
                            color = (255, 255, 0)

                        cv2.circle(frame, (bx, by), 10, color, -1)

                elif class_id == NN_Labels.ROTARY:
                    res = get_rotary_valve_position(frame, (x1, y1, x2, y2))
                    if res is not None:
                        rot, bx, by = res

                        # Draw line from center at angle rot
                        cv2.line(
                            frame,
                            (bx, by),
                            (bx + int(np.cos(rot) * 100), by + int(np.sin(rot) * 100)),
                            (0, 255, 0),
                            2,
                        )

                elif class_id == NN_Labels.SPIGOTSIDEVIEW:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                elif class_id == NN_Labels.SPIGOTTOPVIEW:
                    res = get_spigot_valve_position(frame, (x1, y1, x2, y2))
                    if res is not None:
                        rot, bx, by = res

                        # Draw line from center at angle rot
                        cv2.line(
                            frame,
                            (bx, by),
                            (bx + int(np.cos(rot) * 100), by + int(np.sin(rot) * 100)),
                            (0, 255, 0),
                            2,
                        )

                elif class_id == NN_Labels.STOPCOCKSIDEVIEW:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                elif class_id == NN_Labels.STOPCOCKTOPVIEW:
                    res = get_stopcock_valve_position(frame, (x1, y1, x2, y2))
                    if res is not None:
                        rot, bx, by = res

                        # Draw line from center at angle rot
                        cv2.line(
                            frame,
                            (bx, by),
                            (bx + int(np.cos(rot) * 300), by + int(np.sin(rot) * 300)),
                            (0, 255, 0),
                            2,
                        )

        # Display YOLO Results frame
        imgs = results.render()
        small_img = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        cv2.imshow("Frame", small_img)

        # If the user presses the 'q' key...
        key = cv2.waitKey(1)
        if key == ord("q"):
            break


def get_breaker_x_in_center_of_frame():
    frame, _ = get_image()
    results = apply_nn(frame)

    best_cx = 100000

    for x1, y1, x2, y2, _, class_id in results.xyxy[0]:
        # Convert from pytorch tensor to int
        class_id = int(class_id)
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)

        # Check that width and height are greater than zero
        if x2 - x1 > 0 and y2 - y1 > 0:

            if class_id == NN_Labels.BREAKER:
                cx, _ = (x1 + x2) / 2, (y1 + y2) / 2

                res = detect_breaker_state(frame, (x1, y1, x2, y2))
                if res is not None:
                    _, cx, _ = res

                cx_norm = cx - frame.shape[1] / 2

                if abs(cx_norm) < abs(best_cx):
                    best_cx = cx_norm

    return best_cx
                
