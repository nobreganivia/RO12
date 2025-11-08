"""
Extended Kalman Filter SLAM example

author: Atsushi Sakai (@Atsushi_twi)

Modified : Goran Frehse, David Filliat, Nivia Araujo
"""

import math

import matplotlib.pyplot as plt
import numpy as np

DT = 0.1  # time tick [s]
SIM_TIME = 80.0  # simulation time [s]
MAX_RANGE = 10.0  # maximum observation range
M_DIST_TH = 11.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x, y, yaw]
LM_SIZE = 2  # Landmark state size [x, y]
KNOWN_DATA_ASSOCIATION = 0  # Whether we use the true landmark id or not

# --- Bearing-only + undelayed init parameters ---

N_HYP = 5                       # number of hypotheses along the bearing ray
HYP_DISTS = np.linspace(2.0, 10.0, N_HYP)  # initial "fake" distances
HYP_COV_BASE = 5.0              # base covariance of landmarks
HYP_COV_GROW = 3.0              # covariance growth with distance
PRUNE_THRESH = 0.05             # probability threshold for pruning

# Probabilities of landmark hypotheses (one per hypothesis)
lm_probs = []


# Simulation parameter
# noise on control input
Q_sim = (3 * np.diag([0.1, np.deg2rad(1)])) ** 2
# noise on measurement
Py_sim = (1 * np.diag([0.1, np.deg2rad(5)])) ** 2

# Kalman filter Parameters
# Estimated input noise for Kalman Filter
Q = 3 * Q_sim
# Estimated measurement noise for Kalman Filter
Py = 3 * Py_sim

# Initial estimate of pose covariance
initPEst = 0.01 * np.eye(STATE_SIZE)
initPEst[2, 2] = 0.0001  # low orientation error

# True Landmark id for known data association
trueLandmarkId = []

# Init displays
show_animation = True
f, (ax1, ax2) = plt.subplots(1, 2, sharey=True, figsize=(14, 7))
ax3 = plt.subplot(3, 2, 2)
ax4 = plt.subplot(3, 2, 4)
ax5 = plt.subplot(3, 2, 6)


# --- Helper functions

def calc_n_lm(x):
    """
    Computes the number of landmarks in the state vector
    """

    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n


def calc_landmark_position(x, y):
    """
    Computes absolute landmark position from robot pose and observation
    """

    y_abs = np.zeros((2, 1))

    y_abs[0, 0] = x[0, 0] + y[0] * math.cos(x[2, 0] + y[1])
    y_abs[1, 0] = x[1, 0] + y[0] * math.sin(x[2, 0] + y[1])

    return y_abs


def get_landmark_position_from_state(x, ind):
    """
    Extract landmark position from state vector
    """

    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def pi_2_pi(angle):
    """
    Normalize an angle to [-pi, pi]
    """

    return (angle + math.pi) % (2 * math.pi) - math.pi


def plot_covariance_ellipse(xEst, PEst, axes, lineType):
    """
    Plot one covariance ellipse from covariance matrix
    """

    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    if eigval[smallind] < 0:
        print('Pb with Pxy :\n', Pxy)
        exit()

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [3 * a * math.cos(it) for it in t]
    y = [3 * b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    rot = np.array([[math.cos(angle), math.sin(angle)],
                    [-math.sin(angle), math.cos(angle)]])
    fx = rot @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    axes.plot(px, py, lineType)


# --- Motion model related functions

def calc_input():
    """
    Generate a control vector to make the robot follow a circular trajectory
    """

    v = 1  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u


def motion_model(x, u):
    """
    Compute future robot position from current position and control
    """

    xp = np.array([[x[0, 0] + u[0, 0] * DT * math.cos(x[2, 0])],
                   [x[1, 0] + u[0, 0] * DT * math.sin(x[2, 0])],
                   [x[2, 0] + u[1, 0] * DT]])
    xp[2] = pi_2_pi(xp[2])

    return xp.reshape((3, 1))


def jacob_motion(x, u):
    """
    Compute the Jacobians of motion model w.r.t. x and u
    """

    # Jacobian of f(X,u) w.r.t. X
    A = np.array([[1.0, 0.0, float(-DT * u[0, 0] * math.sin(x[2, 0]))],
                  [0.0, 1.0, float(DT * u[0, 0] * math.cos(x[2, 0]))],
                  [0.0, 0.0, 1.0]])

    # Jacobian of f(X,u) w.r.t. u
    B = np.array([[float(DT * math.cos(x[2, 0])), 0.0],
                  [float(DT * math.sin(x[2, 0])), 0.0],
                  [0.0, DT]])

    return A, B


# --- Observation model related functions

def observation(xTrue, xd, uTrue, Landmarks):
    """
    Generate noisy control and observation and update true position and dead reckoning
    """
    xTrue = motion_model(xTrue, uTrue)

    # add noise to range and bearing
    y = np.zeros((0, 3))

    for i in range(len(Landmarks[:, 0])):

        dx = Landmarks[i, 0] - xTrue[0, 0]
        dy = Landmarks[i, 1] - xTrue[1, 0]
        d = math.hypot(dx, dy)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Py_sim[0, 0] ** 0.5  # add noise
            dn = max(dn, 0)
            angle_n = angle + np.random.randn() * Py_sim[1, 1] ** 0.5  # add noise
            yi = np.array([dn, angle_n, i])
            y = np.vstack((y, yi))

    # add noise to input
    u = np.array([[
        uTrue[0, 0] + np.random.randn() * Q_sim[0, 0] ** 0.5,
        uTrue[1, 0] + np.random.randn() * Q_sim[1, 1] ** 0.5]]).T

    xd = motion_model(xd, u)

    return xTrue, y, xd, u


def search_correspond_landmark_id(xEst, PEst, yi):
    """
    Landmark association with Mahalanobis distance (bearing-only)

    Returns:
        min_id: index of the most likely landmark or nLM (= new landmark)
        dists: list of Mahalanobis distances (for existing landmarks only)
    """

    nLM = calc_n_lm(xEst)
    dists = []

    for i in range(nLM):
        innov, S, H = calc_innovation_bearing_only(xEst, PEst, yi, i)
        d2 = (innov.T @ np.linalg.inv(S) @ innov)[0, 0]
        dists.append(d2)

    # "fake" distance for a new landmark
    dists_with_new = dists + [M_DIST_TH]
    min_id = dists_with_new.index(min(dists_with_new))

    return min_id, dists


def jacob_h(q, delta, x, i):
    """
    Compute the Jacobian of the observation model
    """

    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], -sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], -delta[0, 0], -q, -delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_lm(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * i)),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * (i + 1)))))

    F = np.vstack((F1, F2))

    H = G @ F

    return H


def jacob_augment(x, y):
    """
    Compute the Jacobians for extending the covariance matrix
    """

    Jr = np.array([[1.0, 0.0, -y[0] * math.sin(x[2, 0] + y[1])],
                   [0.0, 1.0, y[0] * math.cos(x[2, 0] + y[1])]])

    Jy = np.array([[math.cos(x[2, 0] + y[1]), -y[0] * math.sin(x[2, 0] + y[1])],
                   [math.sin(x[2, 0] + y[1]), y[0] * math.cos(x[2, 0] + y[1])]])

    return Jr, Jy


# --- Kalman filter related functions

def calc_innovation_bearing_only(xEst, PEst, y, LMid):
    """
    Compute innovation and Kalman matrices using only landmark bearing.
    y: measured vector [range, bearing]
    """

    lm = get_landmark_position_from_state(xEst, LMid)
    delta = lm - xEst[0:2]
    q = (delta.T @ delta)[0, 0]

    # predicted bearing
    y_angle_pred = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    z_pred = np.array([[pi_2_pi(y_angle_pred)]])  # 1x1
    z = np.array([[y[1]]])                        # measured bearing only

    innov = z - z_pred
    innov[0, 0] = pi_2_pi(innov[0, 0])

    # full Jacobian (2 rows), we only use the bearing row
    H_full = jacob_h(q, delta, xEst, LMid)        # 2 x n
    H = H_full[1:2, :]                            # 1 x n

    # measurement noise only for bearing
    Py_dir = np.array([[Py[1, 1]]])               # 1x1

    S = H @ PEst @ H.T + Py_dir                   # 1x1

    return innov, S, H


def ekf_slam(xEst, PEst, u, y):
    """
    Apply one step of EKF predict/correct cycle
    """

    global lm_probs

    S = STATE_SIZE

    # Predict
    A, B = jacob_motion(xEst[0:S], u)

    xEst[0:S] = motion_model(xEst[0:S], u)

    PEst[0:S, 0:S] = A @ PEst[0:S, 0:S] @ A.T + B @ Q @ B.T
    PEst[0:S, S:] = A @ PEst[0:S, S:]
    PEst[S:, 0:S] = PEst[0:S, S:].T

    PEst = (PEst + PEst.T) / 2.0  # ensure symmetry

    # Update
    for iy in range(len(y[:, 0])):  # for each observation
        nLM = calc_n_lm(xEst)

        if KNOWN_DATA_ASSOCIATION:
            # If using known data association, you can keep this logic,
            # but typically for this exercise we use KNOWN_DATA_ASSOCIATION = 0
            try:
                min_id = trueLandmarkId.index(y[iy, 2])
                dists = []
            except ValueError:
                min_id = nLM
                dists = []
        else:
            min_id, dists = search_correspond_landmark_id(xEst, PEst, y[iy, 0:2])

        # Update hypothesis probabilities (if there are landmarks)
        if len(dists) > 0:
            update_lm_probs(dists)

        # Extend map if required – undelayed multi-hypothesis init
        if min_id == nLM:
            print("New LM (multi-hypotheses)")
            xEst, PEst = add_bearing_hypotheses(xEst, PEst, y[iy, 0:2])
        else:
            # Perform Kalman update using bearing-only
            innov, S_meas, H = calc_innovation_bearing_only(xEst, PEst, y[iy, 0:2], min_id)
            K = (PEst @ H.T) @ np.linalg.inv(S_meas)

            xEst = xEst + (K @ innov)
            PEst = (np.eye(len(xEst)) - K @ H) @ PEst
            PEst = 0.5 * (PEst + PEst.T)  # Ensure symmetry

        # Prune landmarks with low probability
        xEst, PEst = prune_landmarks(xEst, PEst)

    xEst[2] = pi_2_pi(xEst[2])

    return xEst, PEst


def add_bearing_hypotheses(xEst, PEst, y):
    """
    Create N_HYP landmark hypotheses along the observed bearing (bearing-only),
    using HYP_DISTS as fake range values.
    Updates xEst, PEst and lm_probs.
    """

    global lm_probs

    xR = xEst[0:3].copy()

    for idx_dist, r in enumerate(HYP_DISTS):
        # virtual measurement: [fake_range, real_bearing]
        y_fake = np.array([r, y[1]])

        # landmark position
        lm_pos = calc_landmark_position(xR, y_fake)
        xEst = np.vstack((xEst, lm_pos))

        # Jacobians to extend the covariance matrix
        Jr, Jy = jacob_augment(xR, y_fake)

        # augmentation blocks
        top = np.hstack((PEst, (Jr @ PEst[0:3, :]).T))
        bottom_left = Jr @ PEst[0:3, :]
        P_new_lm = Jr @ PEst[0:3, 0:3] @ Jr.T + Jy @ Py @ Jy.T

        # increase uncertainty with distance (growing covariance)
        extra_cov = (HYP_COV_BASE + HYP_COV_GROW * (r ** 2)) * np.eye(2)
        P_new_lm = P_new_lm + extra_cov

        bottom = np.hstack((bottom_left, P_new_lm))
        PEst = np.vstack((top, bottom))

        # initial uniform probability for each newly created hypothesis
        lm_probs.append(1.0)

    # normalize probabilities
    s = sum(lm_probs)
    if s > 0:
        lm_probs = [p / s for p in lm_probs]

    return xEst, PEst


def prune_landmarks(xEst, PEst):
    """
    Remove landmarks whose probabilities have dropped below PRUNE_THRESH.
    """

    global lm_probs

    nLM = calc_n_lm(xEst)
    if nLM == 0:
        return xEst, PEst

    keep_indices = [i for i, p in enumerate(lm_probs) if p >= PRUNE_THRESH]

    if len(keep_indices) == nLM:
        return xEst, PEst  # nothing to prune

    # update probability vector
    lm_probs = [lm_probs[i] for i in keep_indices]
    s = sum(lm_probs)
    if s > 0:
        lm_probs = [p / s for p in lm_probs]

    # indices of state vector to keep (pose + selected landmarks)
    idxs = [0, 1, 2]
    for k in keep_indices:
        base = STATE_SIZE + 2 * k
        idxs.extend([base, base + 1])

    idxs = np.array(idxs, dtype=int)

    xEst = xEst[idxs, :]
    PEst = PEst[np.ix_(idxs, idxs)]

    return xEst, PEst


def update_lm_probs(dists):
    """
    Update hypothesis probabilities based on the likelihood of the
    observation (bearing). In a single-landmark environment, all hypotheses compete.
    """

    global lm_probs

    if len(dists) != len(lm_probs):
        return

    for i, d2 in enumerate(dists):
        # prob ~ exp(-0.5 * d^2) (clip d2 to avoid underflow)
        d2_clip = min(d2, 50.0)
        likelihood = math.exp(-0.5 * d2_clip)
        lm_probs[i] *= likelihood

    s = sum(lm_probs)
    if s > 0:
        lm_probs = [p / s for p in lm_probs]


# --- Main script

def main():
    print(__file__ + " start!!")

    time = 0.0

    # Define landmark positions [x, y]
    Landmarks = np.array([[5.0, 5.0]])  # a single landmark

    # Init state vector [x y yaw]' and covariance for Kalman
    xEst = np.zeros((STATE_SIZE, 1))
    PEst = initPEst

    # Init true state for simulator
    xTrue = np.zeros((STATE_SIZE, 1))

    # Init dead reckoning (sum of individual controls)
    xDR = np.zeros((STATE_SIZE, 1))

    # Init history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hxError = np.abs(xEst - xTrue)  # pose error
    hxVar = np.sqrt(np.diag(PEst[0:STATE_SIZE, 0:STATE_SIZE]).reshape(3, 1))  # state std dev


    # counter for plotting
    count = 0

    while time <= SIM_TIME:
        count = count + 1
        time += DT

        # Simulate motion and generate u and y
        uTrue = calc_input()
        xTrue, y, xDR, u = observation(xTrue, xDR, uTrue, Landmarks)

        xEst, PEst = ekf_slam(xEst, PEst, u, y)

        # Store maximum hypothesis probability of landmarks
        if len(lm_probs) > 0:
            maxProb = max(lm_probs)
        else:
            maxProb = 0.0

        # store data history
        hxEst = np.hstack((hxEst, xEst[0:STATE_SIZE]))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        err = xEst[0:STATE_SIZE] - xTrue
        err[2] = pi_2_pi(err[2])
        hxError = np.hstack((hxError, err))
        hxVar = np.hstack((hxVar, np.sqrt(np.diag(PEst[0:STATE_SIZE, 0:STATE_SIZE]).reshape(3, 1))))

        if show_animation and count % 15 == 0:
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            ax1.cla()

            # Plot true landmark and trajectory
            ax1.plot(Landmarks[:, 0], Landmarks[:, 1], "*k")
            ax1.plot(hxTrue[0, :], hxTrue[1, :], "-k", label="True")

            # Plot odometry trajectory
            ax1.plot(hxDR[0, :], hxDR[1, :], "-g", label="Odom")

            # Plot estimated trajectory, pose and landmarks
            ax1.plot(hxEst[0, :], hxEst[1, :], "-r", label="EKF")
            ax1.plot(xEst[0], xEst[1], ".r")
            plot_covariance_ellipse(xEst[0: STATE_SIZE],
                                    PEst[0: STATE_SIZE, 0: STATE_SIZE], ax1, "--r")

            for i in range(calc_n_lm(xEst)):
                idx = STATE_SIZE + i * 2

                # get probability of hypothesis i (if it exists)
                if i < len(lm_probs):
                    prob = lm_probs[i]
                else:
                    prob = 1.0

                # marker size proportional to probability
                marker_size = 50 + 300 * prob

                # use scatter to allow different sizes
                ax1.scatter(xEst[idx], xEst[idx + 1],
                            s=marker_size, c='r', alpha=0.7, label='_nolegend_')

                # covariance ellipse
                plot_covariance_ellipse(xEst[idx:idx + 2],
                                        PEst[idx:idx + 2, idx:idx + 2], ax1, "--r")

            ax1.axis([-12, 12, -2, 22])
            ax1.grid(True)
            ax1.legend()

            # plot error curves
            ax3.plot(hxError[0, :], 'b')
            ax3.plot(3.0 * hxVar[0, :], 'r')
            ax3.plot(-3.0 * hxVar[0, :], 'r')
            ax3.set_ylabel('x')
            ax3.set_title('Real error (blue) and 3 $\\sigma$ covariances (red)')

            ax4.plot(hxError[1, :], 'b')
            ax4.plot(3.0 * hxVar[1, :], 'r')
            ax4.plot(-3.0 * hxVar[1, :], 'r')
            ax4.set_ylabel('y')

            ax5.plot(hxError[2, :], 'b')
            ax5.plot(3.0 * hxVar[2, :], 'r')
            ax5.plot(-3.0 * hxVar[2, :], 'r')
            ax5.set_ylabel(r"$\theta$")

            plt.pause(0.1)

    plt.savefig('EKFSLAM.png')

    tErrors = np.sqrt(np.square(hxError[0, :]) + np.square(hxError[1, :]))
    oErrors = np.sqrt(np.square(hxError[2, :]))
    print("Mean (var) translation error : {:e} ({:e})".format(np.mean(tErrors), np.var(tErrors)))
    print("Mean (var) rotation error : {:e} ({:e})".format(np.mean(oErrors), np.var(oErrors)))
    print("Press Q in figure to finish...")
    plt.show()


if __name__ == '__main__':
    main()
