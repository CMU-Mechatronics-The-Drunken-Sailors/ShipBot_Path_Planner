import numpy as np
import time

from ..Command_Hub import *

##### CONSTANTS #####
DEBUG = True

TIME_PAUSE = 0.1
UPDATE_FREQ = 0.05  # Time for each loop iter
WHEELBASE_X = 0.314 / 2  # meters
WHEELBASE_Y = 0.2 / 2  # meters
MECANUM_IK = np.array(
    [
        [1, -1, -(WHEELBASE_X + WHEELBASE_Y)],
        [1, 1, (WHEELBASE_X + WHEELBASE_Y)],
        [1, 1, -(WHEELBASE_X + WHEELBASE_Y)],
        [1, -1, (WHEELBASE_X + WHEELBASE_Y)],
    ]
)
MECANUM_FK = 0.25 * np.array(
    [
        [1, 1, 1, 1],
        [-1, 1, 1, -1],
        np.array([-1, 1, -1, 1]) * (1.0 / (WHEELBASE_X + WHEELBASE_Y)),
    ]
)

AMAX = 0.4 #0.25
VMAX = 0.35
ALPHAMAX = 0.7
WMAX = 1

FBK_ON = True
FBK_lambda = 7 #11
FBK_kX = 1.0 / FBK_lambda
FBK_kY = 1.0 / (FBK_lambda)
FBK_kTh = 2.0 / FBK_lambda * 1.5

DEGREES_PER_TICK = 360.0 / 2797.0
METERS_PER_DEGREE = 0.097 * np.pi / 360

##### HELPER FUNCTIONS #####


def trapezoidalVelocityProfile(t, dist, vmax, amax):
    # t: the current time
    # dist: distance to travel
    # backwards: do trajectory in reverse
    # [return] the velocity to command
    tramp = vmax / amax
    tf = (abs(dist) + vmax * vmax / amax) / vmax

    if tramp > tf / 2.0:
        # Switch to triangular velocity profile
        tf = np.sqrt(4.0 * abs(dist) / amax)
        tramp = tf / 2.0

        v = 0

    t -= TIME_PAUSE
    if t < 0:
        v = 0
    elif t < tramp:
        v = amax * t
    elif t < tf - tramp:
        v = vmax
    elif t < tf:
        v = amax * (tf - t)
    else:
        v = 0

    if dist < 0:
        v *= -1

    return v


def calcTrapVelTrajectoryTime(dist, vmax, amax):
    return (abs(dist) + vmax * vmax / amax) / vmax + TIME_PAUSE * 2


def mecanum_ik(Vx, Vy, w):
    # Vx, Vy, w: the linear and angular velocities of the robot
    # [return] the wheel velocities
    v = np.array([Vx, Vy, w])
    wheel_speeds = MECANUM_IK @ v
    wfl, wfr, wrl, wrr = wheel_speeds

    return [wfr, wfl, wrr, wrl]


def mecanum_fk(wfr, wfl, wrr, wrl):
    # wfl, wfr, wrl, wrr: the wheel velocities
    # [return] the linear and angular velocities of the robot
    wheel_speeds = np.array([wfl, wfr, wrl, wrr])
    v = MECANUM_FK @ wheel_speeds
    Vx, Vy, w = v

    return [Vx, Vy, w]


##### FEEDFORWARD/FEEDBACK FUNCTIONS #####
ffX = 0
ffY = 0
ffTh = 0
fbX = 0
fbY = 0
fbTh = 0

last_enc = [0, 0, 0, 0]

lastFFTickTime = time.time()


def resetFF(newX, newY, newTh):
    global ffX, ffY, ffTh, fbX, fbY, fbTh, lastFFTickTime

    ffX = newX
    ffY = newY
    ffTh = newTh
    fbX = newX
    fbY = newY
    fbTh = newTh
    lastFFTickTime = time.time()


def ffTick(Vx, Vy, w):
    global ffX, ffY, ffTh, lastFFTickTime

    currentTime = time.time()
    dt = currentTime - lastFFTickTime

    dx = Vx * dt
    dy = Vy * dt
    dth = w * dt

    # thetaNext = ffTh + dth / 2
    # ffX += dx * np.cos(thetaNext) - dy * np.sin(thetaNext)
    # ffY += dx * np.sin(thetaNext) + dy * np.cos(thetaNext)
    # ffTh += dth;

    ffX += dx
    ffY += dy
    ffTh += dth
    ffTh = np.arctan2(np.sin(ffTh), np.cos(ffTh))

    lastFFTickTime = currentTime


def update_odometry():
    global last_enc, fbX, fbY, fbTh

    new_enc = get_encoder_pos()
    # print(new_enc)
    d_enc = [new_enc[i] - last_enc[i] for i in range(4)]
    dx, dy, dth = mecanum_fk(*d_enc)
    fbX += dx
    fbY += dy
    fbTh += dth
    fbTh = np.arctan2(np.sin(fbTh), np.cos(fbTh))

    # print(fbX, fbY, fbTh)
    last_enc = new_enc


def add_feedback(Vx, Vy, w):
    global ffX, ffY, ffTh, fbX, fbY, fbTh

    if FBK_ON:
        errX, errY, errTh = [
            ff - fb for ff, fb in zip([ffX, ffY, ffTh], [fbX, fbY, fbTh])
        ]
        return [Vx + FBK_kX * errX, Vy + FBK_kY * errY, w + FBK_kTh * errTh]
    else:
        return [Vx, Vy, w]


##### MOVE FUNCTIONS #####


def moveRelDist(dist, traj_fn, tf):
    global last_enc

    # Don't do anything if dist is 0 (or very close to)
    if abs(dist) < 0.001:
        time.sleep(UPDATE_FREQ)
        return

    last_enc = get_encoder_pos()

    t = time.time()
    startTime = t

    while t - startTime < tf:
        t = time.time()

        vx, vy, w = traj_fn(t - startTime, dist)

        ffTick(vx, vy, w)
        vx, vy, w = add_feedback(vx, vy, w)
        cmd_mecanum_drive_kinematics(vx, vy, w)

        # if DEBUG:
            # print(f"{vx:.3f}, {vy:.3f}, {w:.3f}")
            # print(f"{fbX:.3f}, {fbY:.3f}, {fbTh:.3f}")

        update_odometry()
        time.sleep(max(0, UPDATE_FREQ - time.time() - t))

    stop()


def moveRelDistX(dist):
    moveRelDist(
        dist,
        lambda t, dist: [trapezoidalVelocityProfile(t, dist, VMAX, AMAX), 0, 0],
        calcTrapVelTrajectoryTime(dist, VMAX, AMAX),
    )
    # resetFF(0,0,0)

def moveRelDistXSLOW(dist):
    moveRelDist(
        dist,
        lambda t, dist: [trapezoidalVelocityProfile(t, dist, VMAX/1.75, AMAX), 0, 0],
        calcTrapVelTrajectoryTime(dist, VMAX, AMAX),
    )
    # resetFF(0,0,0)

def moveRelDistXVERYSLOW(dist):
    moveRelDist(
        dist,
        lambda t, dist: [trapezoidalVelocityProfile(t, dist, VMAX/4, AMAX), 0, 0],
        calcTrapVelTrajectoryTime(dist, VMAX, AMAX),
    )
    # resetFF(0,0,0)


def moveRelDistY(dist):
    moveRelDist(
        dist,
        lambda t, dist: [0, trapezoidalVelocityProfile(t, dist, VMAX, AMAX), 0],
        calcTrapVelTrajectoryTime(dist, VMAX, AMAX),
    )
    # resetFF(0,0,0)


def moveRelDistYSLOW(dist):
    moveRelDist(
        dist,
        lambda t, dist: [0, trapezoidalVelocityProfile(t, dist, VMAX/2, AMAX), 0],
        calcTrapVelTrajectoryTime(dist, VMAX, AMAX),
    )
    # resetFF(0,0,0)


def turnRelAngle(angle):
    moveRelDist(
        angle,
        lambda t, dist: [0, 0, trapezoidalVelocityProfile(t, dist, WMAX, ALPHAMAX)],
        calcTrapVelTrajectoryTime(angle, WMAX, ALPHAMAX),
    )
    resetFF(0,0,0)


##### Interface with motors #####
def cmd_mecanum_drive_kinematics(Vx, Vy, w):
    # Vx, Vy, w: the linear and angular velocities of the robot
    # [return] the wheel velocities
    wheel_speeds = mecanum_ik(-Vx, -Vy, -w)

    cmd_wheels(*wheel_speeds)


def stop():
    cmd_wheels(0, 0, 0, 0)


def cmd_wheels(wfr, wfl, wrr, wrl):
    wfr, wfl, wrr, wrl = [
        int(round(x / METERS_PER_DEGREE)) for x in [wfr, wfl, wrr, wrl]
    ]

    # TODO: send to Joel's code
    set_DC_motors(wfr, wfl, wrr, wrl)
    # print(f"{wfr}, {wfl}, {wrr}, {wrl}")
    pass


last_enc_pos = [0, 0, 0, 0]


def get_encoder_pos():
    # TODO: get from Joel's code
    enc_pos = get_encoder_ticks()
    tries = 1
    while len(enc_pos) != 4:
        enc_pos = get_encoder_ticks()
        tries += 1
        if tries > 3:
            print("ENCODERS NOT WORKING")
            return last_enc_pos

    enc_pos = [x * METERS_PER_DEGREE * DEGREES_PER_TICK for x in enc_pos]
    enc_pos[1] *= -1
    enc_pos[3] *= -1
    last_enc_pos = enc_pos

    return enc_pos
