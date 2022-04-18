import numpy as np
import time

##### CONSTANTS #####
DEBUG = True

TIME_PAUSE = 0.1
UPDATE_FREQ = 0.05 # Time for each loop iter
WHEELBASE_X = 0.314 / 2 # meters
WHEELBASE_Y = 0.2 / 2 # meters
MECANUM_IK = np.array([
    [1, -1, -(WHEELBASE_X + WHEELBASE_Y)],
    [1,  1,  (WHEELBASE_X + WHEELBASE_Y)],
    [1,  1, -(WHEELBASE_X + WHEELBASE_Y)],
    [1, -1,  (WHEELBASE_X + WHEELBASE_Y)]
])
MECANUM_FK = 0.25 * np.array([
    [1,   1,   1,   1],
    [-1,  1,   1,  -1],
    [-1,  1,  -1,   1] * (1.0 / (WHEELBASE_X + WHEELBASE_Y)),
])

AMAX = 0.2
VMAX = 0.2
ALPHAMAX = 0.2
WMAX = 0.5

FBK_ON = False
FBK_lambda = 0.1
FBK_kX = (1.0/FBK_lambda)
FBK_kY = 1.0/(FBK_lambda)
FBK_kTh = (2.0/FBK_lambda)

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

    if (tramp > tf / 2.0):
        # Switch to triangular velocity profile
        tf = np.sqrt(4.0 * abs(dist) / amax)
        tramp = tf / 2.0

        v = 0

    t -= TIME_PAUSE
    if (t < 0):
        v = 0
    elif (t < tramp):
        v = amax * t
    elif (t < tf - tramp):
        v = vmax
    elif (t < tf):
        v = amax * (tf - t)
    else:
        v = 0

    if (dist < 0):
        v *= -1

    return v

def calcTrapVelTrajectoryTime( dist,  vmax,  amax):
    return (abs(dist) + vmax * vmax / amax) / vmax + TIME_PAUSE * 2

def mecanum_fk(Vx, Vy, w):
    # Vx, Vy, w: the linear and angular velocities of the robot
    # [return] the wheel velocities
    v = np.array([Vx, Vy, w])
    wheel_speeds = np.dot(MECANUM_FK, v)
    wfl, wfr, wrl, wrr = wheel_speeds

    return [wfr, wfl, wrr, wrl]

def mecanum_ik(wfr, wfl, wrr, wrl):
    # wfl, wfr, wrl, wrr: the wheel velocities
    # [return] the linear and angular velocities of the robot
    wheel_speeds = np.array([wfl, wfr, wrl, wrr])
    v = np.dot(MECANUM_IK, wheel_speeds)
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
    ffX = newX
    ffY = newY
    ffTh = newTh
    lastFFTickTime = time.time()

def ffTick(Vx, Vy, w):
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
    new_enc = get_encoder_pos()
    d_enc = [new_enc[i] - last_enc[i] for i in range(4)]
    dx, dy, dth = mecanum_ik(*d_enc)
    fbX += dx
    fbY += dy
    fbTh += dth
    fbTh = np.arctan2(np.sin(fbTh), np.cos(fbTh))

def add_feedback(Vx, Vy, w):
    if FBK_ON:
        errX, errY, errTh = [ff - fb for ff, fb in zip([ffX, ffY, ffTh], [fbX, fbY, fbTh])]
        return [Vx + FBK_kX * errX, Vy + FBK_kY * errY, w + FBK_kTh * errTh]
    else:
        return [Vx, Vy, w]

##### MOVE FUNCTIONS #####

def moveRelDist(dist, traj_fn):
    # Don't do anything if dist is 0 (or very close to)
    if (abs(dist) < 0.001):
        time.sleep(UPDATE_FREQ)
        return

    V = 0
    w = 0

    t = time.time()
    startTime = t
    tf = calcTrapVelTrajectoryTime(dist, VMAX, AMAX)

    while (t - startTime < tf):
        t = time.time()

        vx, vy, w = traj_fn(t, dist)

        ffTick(vx, vy, w)
        add_feedback(vx, vy, w)
        cmd_mecanum_drive_kinematics(vx, vy, w)

        if DEBUG:
            print(f"{vx:.3f}, {vy:.3f}, {w:.3f}")

        update_odometry()
        time.sleep(max(0, UPDATE_FREQ - time.time() - t))

    stop()

def moveRelDistX(dist):
    moveRelDist(dist, lambda t, dist : [trapezoidalVelocityProfile(t, dist, VMAX, AMAX), 0, 0])

def moveRelDistY(dist):
    moveRelDist(dist, lambda t, dist : [0, trapezoidalVelocityProfile(t, dist, VMAX, AMAX), 0])

def turnRelAngle(angle):
    moveRelDist(angle, lambda t, dist : [0, 0, trapezoidalVelocityProfile(t, dist, WMAX, ALPHAMAX)])


##### Interface with motors #####
def cmd_mecanum_drive_kinematics(Vx, Vy, w):
    # Vx, Vy, w: the linear and angular velocities of the robot
    # [return] the wheel velocities
    wheel_speeds = mecanum_fk(Vx, Vy, w)

    cmd_wheels(*wheel_speeds)

def stop():
    cmd_wheels(0, 0, 0, 0)

def cmd_wheels(wfr, wfl, wrr, wrl):
    wfr, wfl, wrr, wrl = [int(round(x / METERS_PER_DEGREE)) for x in [wfr, wfl, wrr, wrl]]

    # TODO: send to Joel's code
    pass

def get_encoder_pos():
    # TODO: get from Joel's code
    enc_pos = [0,0,0,0]

    enc_pos = [x * METERS_PER_DEGREE * DEGREES_PER_TICK for x in enc_pos]
    return enc_pos