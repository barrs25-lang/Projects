"""
main_cosim.py
Co-simulation: bicycle MRAC controller (from main.py) + PyChrono HMMWV.

Every parameter, gain, initial condition, and plot is identical to main.py.
The plant states and their derivatives come from Chrono rather than the
bicycle model equations, giving the controller accurate information about
the real vehicle physics.

Chrono quantities injected into dynamical_system at each step
─────────────────────────────────────────────────────────────
  r(t)       : XY position of rear axle midpoint             [m]
  v1(t)      : longitudinal speed at rear axle (body-X)      [m/s]
  theta(t)   : yaw angle                                     [rad]
  v(t)       : true world velocity of rear axle              [m/s, (2,)]
  v_dot(t)   : true world acceleration of rear axle          [m/s², (2,)]
  theta_dot  : true yaw rate                                 [rad/s]

v, v_dot, theta_dot replace the bicycle-model reconstructions inside
dynamical_system so OL_error and rho_tran are computed against true Chrono
physics rather than the simplified bicycle model.

Run:  python main_cosim.py
"""

import math
import numpy as np
from scipy.linalg import solve_continuous_lyapunov as lyap
from scipy.linalg import sqrtm
import matplotlib
import matplotlib.pyplot as plt
matplotlib.rcParams.update({
    'text.usetex': False, 'font.size': 13,
    'axes.grid': True, 'lines.linewidth': 1.5, 'axes.linewidth': 1.1,
})

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

from functions import rk4singlestep as rk4
from reference import reference
from dynamical_system import dynamical_system


# ══════════════════════════════════════════════════════════════════════════════
# 1.  Variant
# ══════════════════════════════════════════════════════════════════════════════
variant = 'Projection_Operator'  # 'Classical', 'Deadzone', 'Modified-Deadzone', 'sigma-Modification', 'e-Modification', 'Projection_Operator', 'Constrained', 'Two_Layer'


# ══════════════════════════════════════════════════════════════════════════════
# 2.  Simulation / Chrono settings
# ══════════════════════════════════════════════════════════════════════════════
step_size      = 2e-3
tire_step_size = 1e-3
T_END          = 27.0
RENDER_STEP    = 5

MAX_STEER_RAD    = np.pi/2
REAR_AXLE_OFFSET = -1.7     # [m] local X offset CoM -> rear axle; calibrate once
PRINT_AXLE_OFFSET = False

# ── Speed controller (maps u_force → throttle/brake via PI on longitudinal speed) ──
# u_force is integrated as a velocity rate (u_force / M_EFF → v_des_dot),
# then a PI controller converts the speed error to a throttle/brake command.
M_EFF       = 100   # [kg]   nominal HMMWV mass for velocity integrator
KP_SPEED    =  3     # [1/(m/s)] P gain: speed error → throttle fraction
KI_SPEED    =  .3     # [1/(m/s·s)] I gain
V_DES_MAX   = 25.0     # [m/s]  cap on integrated desired speed


# ══════════════════════════════════════════════════════════════════════════════
# 3.  MRAC parameters  
# ══════════════════════════════════════════════════════════════════════════════
L    = 1.0;   Lbar = 3.4
m_p  = 1.0;   mbar = 4500

parameters = {
    'L': L, 'Lbar': Lbar, 'm': m_p, 'mbar': mbar,
    'M': np.eye(1), 'emax': 7.0,
}

K_P_tran_kin = 0.0001 * np.eye(2)
K_I_tran_kin = 0.0001 * np.eye(2)
A_ref_tran   = -.05 * np.eye(2)
K_P_tran_dyn = 0.1 * np.eye(2)
K_I_tran_dyn = 0.10 * np.eye(2)

parameters.update({
    'K_P_tran_kin': K_P_tran_kin, 'K_I_tran_kin': K_I_tran_kin,
    'A_ref_tran':   A_ref_tran,   'K_P_tran_dyn': K_P_tran_dyn,
    'K_I_tran_dyn': K_I_tran_dyn,
})

A_bar_tran = np.zeros((2, 2))
B_tran     = mbar**(-1) * np.eye(2)
Q_tran     = 1.0 * np.eye(2)
P_tran     = lyap(A_ref_tran.T, -Q_tran)

Gamma_tran = np.block([
    [1e-1*np.eye(2), np.zeros((2,2)), np.zeros((2,3))],
    [np.zeros((2,2)), 1e-1*np.eye(2), np.zeros((2,3))],
    [np.zeros((3,2)), np.zeros((3,2)), 1e-1*np.eye(3)],
])

sqrtP_tran = sqrtm(P_tran)
xi_bar     = 3.0
E_tran     = (2 * np.max(np.linalg.eigvalsh(sqrtP_tran))
                * np.max(np.linalg.eigvalsh(P_tran))
                / np.min(np.linalg.eigvalsh(Q_tran)) * xi_bar)

A_e_tran     = -1 * np.eye(2)
Q_e_tran     =  1e1 * np.eye(2)
P_e_tran     = lyap(A_e_tran.T, -Q_e_tran)
Gamma_g_tran = 100 * np.eye(A_bar_tran.shape[0])

parameters['outer_loop_adaptive_law_settings'] = {
    'variant': variant, 'sqrtP': sqrtP_tran,
    'E0': 0.9*E_tran, 'E1': 1.1*E_tran, 'd': 2,
    'Sigma': 1e-3, 'epsilon': 0.8, 's': 0.2**2,
    'P': P_tran, 'Q': Q_tran, 'B': B_tran, 'Gamma': Gamma_tran,
    'lambda_min_Q': float(np.min(np.linalg.eigvalsh(Q_tran))),
    'lambda_max_P': float(np.max(np.linalg.eigvalsh(P_tran))),
    'A_e': A_e_tran, 'P_e': P_e_tran, 'Gamma_g': Gamma_g_tran,
}

# Inner-loop
# main.py: natural_frequency_IL_LPF = 1e4, 
# Lower step_size to 1e-4 to restore the original 1e4 value.
natural_frequency_IL_LPF = 10
damping_ratio_IL_LPF     = 1.2
A_ref_rot   = -0.10
K_I_rot_kin =  0.1

parameters.update({
    'natural_frequency_IL_LPF': natural_frequency_IL_LPF,
    'damping_ratio_IL_LPF':     damping_ratio_IL_LPF,
    'A_ref_rot':                A_ref_rot,
    'K_I_rot_kin':              K_I_rot_kin,
})

A_bar_rot = 0.0;  B_rot = 1;  Q_rot = 1
P_rot     = float(lyap(A_ref_rot, -Q_rot))

Gamma_rot = np.block([
    [0.05*np.array([[1]]),  np.zeros((1,1)), np.zeros((1,2))],
    [np.zeros((1,1)),    0.5*np.array([[1]]),np.zeros((1,2))],
    [np.zeros((2,1)),    np.zeros((2,1)), 0.05*np.eye(2)],
])

sqrtP_rot   = float(np.sqrt(P_rot))
E_rot       = 1 * sqrtP_rot * P_rot / Q_rot * xi_bar
A_e_rot     = -1e1
Q_e_rot     = 1e1
P_e_rot     = float(lyap(np.array([[A_e_rot]]).T, np.array([[-Q_e_rot]])))
Gamma_g_rot = 10 * np.eye(1)

parameters['inner_loop_adaptive_law_settings'] = {
    'variant': variant, 'sqrtP': np.array([[sqrtP_rot]]),
    'E0': 0.9*E_rot, 'E1': 1.1*E_rot, 'd': 2,
    'Sigma': 1e-2, 'epsilon': 0.8, 's': 0.2**2,
    'P': np.array([[P_rot]]), 'Q': np.array([[Q_rot]]),
    'B': np.array([[B_rot]]), 'Gamma': Gamma_rot,
    'lambda_min_Q': Q_rot, 'lambda_max_P': P_rot,
    'A_e': A_e_rot, 'P_e': np.array([[P_e_rot]]),
    'Gamma_g': Gamma_g_rot,
}

parameters['Constrained_Active'] = (variant == 'Constrained')
parameters['Two_Layer_Active']   = (variant == 'Two_Layer')
parameters['alpha_unmatched']    = 0.2 if variant == 'Classical' else 0.0


# ══════════════════════════════════════════════════════════════════════════════
# 4.  Initial conditions  
# ══════════════════════════════════════════════════════════════════════════════
two_layer  = parameters['Two_Layer_Active']
L_bar_tran = np.eye(2)

r_0     = np.array([0.0, 0.0])
v1_0    = 0.0
theta_0 = math.pi / 2

r_cmd_0        = r_0.copy()
E_OL_PI_0      = np.zeros(2)
v_ref_0        = np.array([0.0, 0.0])
E_OL_RM_PI_1_0 = np.zeros(2)
E_OL_RM_PI_2_0 = np.zeros(2)

if two_layer:
    K_hat_OL_0 = np.concatenate([
        (B_tran.T @ (A_ref_tran - A_bar_tran) @ np.linalg.inv(L_bar_tran)).T.ravel(),
        np.eye(2).ravel(), np.zeros(5*2),
    ])  # 18
    K_hat_IL_0 = np.concatenate([
        [B_rot * (A_ref_rot - A_bar_rot) / (1.0/Lbar)], [1.0], np.zeros(3),
    ])  # 5
    e_ref_OL_0 = np.zeros(2)
    e_ref_IL_0 = np.array([0.0])
else:
    K_hat_OL_0 = np.concatenate([
        (B_tran.T @ (A_ref_tran - A_bar_tran) @ np.linalg.inv(L_bar_tran)).T.ravel(),
        np.eye(2).ravel(), np.zeros(3*2),
    ])  # 14
    K_hat_IL_0 = np.concatenate([
        [B_rot * (A_ref_rot - A_bar_rot) / (1.0/Lbar)], [1.0], np.zeros(2),
    ])  # 4
    e_ref_OL_0 = np.empty(0)
    e_ref_IL_0 = np.empty(0)

x_IL_LPF_0 = np.array([theta_0, 0.0])
v_debug_0  = np.array([0.0, v1_0])

x0 = np.concatenate([
    r_0, [v1_0], [theta_0],
    r_cmd_0, E_OL_PI_0,
    v_ref_0, E_OL_RM_PI_1_0, E_OL_RM_PI_2_0,
    K_hat_OL_0,
    x_IL_LPF_0, [0.0], [theta_0],
    K_hat_IL_0, v_debug_0,
    e_ref_OL_0, e_ref_IL_0,
])

K_OL_start    = 14
K_OL_end      = K_OL_start + len(K_hat_OL_0)
K_IL_start    = K_OL_end + 4
K_IL_end      = K_IL_start + len(K_hat_IL_0)
THETA_REF_IDX = 35 if two_layer else 31


# ══════════════════════════════════════════════════════════════════════════════
# 5.  Chrono state extraction
#     Returns all quantities needed by dynamical_system:
#       r, v1, theta  — primary plant states (Eq. 3.70)
#       v             — true world velocity of rear axle  (replaces cos/sin*v1)
#       v_dot         — true world acceleration of rear axle
#       theta_dot     — true yaw rate
# ══════════════════════════════════════════════════════════════════════════════

def get_yaw(q) -> float:
    """Yaw about world Z from a Chrono quaternion (Z-up frame)."""
    w, x, y, z = q.e0, q.e1, q.e2, q.e3
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


def extract_plant_state(hmmwv):
    """
    Extract all plant quantities from the HMMWV chassis.

    Returns
    -------
    r         : (2,) XY position of rear axle midpoint
    v1        : float longitudinal speed at rear axle
    theta     : float yaw angle
    chrono_state : dict with keys 'v', 'v_dot', 'theta_dot'
                   to be passed directly to dynamical_system
    """
    body      = hmmwv.GetChassisBody()
    pos       = body.GetPos()
    q         = body.GetRot()
    vel_world = body.GetPosDt()       # world-frame velocity of CoM
    acc_world = body.GetPosDt2()      # world-frame acceleration of CoM

    # ── Rear axle midpoint (position) ─────────────────────────────────────────
    offset     = q.Rotate(chrono.ChVector3d(REAR_AXLE_OFFSET, 0.0, 0.0))
    rear_world = pos + offset
    r          = np.array([rear_world.x, rear_world.y])

    # ── Yaw ──────────────────────────────────────────────────────────────────
    theta = get_yaw(q)

    # ── Body-X unit vector in world frame ─────────────────────────────────────
    bx = q.Rotate(chrono.ChVector3d(1, 0, 0))
    bx_np = np.array([bx.x, bx.y])          # XY components only

    # ── v1: longitudinal speed (body-X component of world velocity) ───────────
    vel_np = np.array([vel_world.x, vel_world.y])
    v1     = float(np.dot(vel_np, bx_np))

    # ── v: true world velocity of rear axle ───────────────────────────────────
    # The rear axle offset is fixed in the body frame, so its world velocity
    # includes the rigid-body rotation contribution:
    #   v_rear = v_CoM + omega x r_offset
    # In 2D (Z-up) omega x r_offset = theta_dot * k x offset_world
    #                                = theta_dot * (-offset_y, offset_x)
    ang_vel = body.GetAngVelParent()       # world-frame angular velocity
    theta_dot = float(ang_vel.z)           # yaw rate about world Z

    offset_np = np.array([offset.x, offset.y])
    # omega x offset in 2D: [-theta_dot * offset_y, theta_dot * offset_x]
    v_rot     = theta_dot * np.array([-offset_np[1], offset_np[0]])
    v         = vel_np + v_rot             # true world velocity of rear axle

    # ── v_dot: true world acceleration of rear axle ───────────────────────────
    # a_rear = a_CoM + alpha x r_offset + omega x (omega x r_offset)
    # In 2D, alpha x r = alpha*k x r = alpha*(-r_y, r_x)
    #         omega x (omega x r) = -omega^2 * r_xy
    ang_acc   = body.GetAngAccParent()
    theta_ddot = float(ang_acc.z)
    acc_np    = np.array([acc_world.x, acc_world.y])

    a_alpha   = theta_ddot * np.array([-offset_np[1],  offset_np[0]])
    a_cent    = -(theta_dot**2) * offset_np
    v_dot     = acc_np + a_alpha + a_cent

    if PRINT_AXLE_OFFSET:
        diff_local = q.RotateBack(rear_world - pos)
        print(f"[calibrate] local offset = "
              f"({diff_local.x:.4f}, {diff_local.y:.4f}, {diff_local.z:.4f})")

    chrono_state = {
        'r':         r,
        'v1':        v1,
        'theta':   theta,
        'v':         v,
        'v_dot':     v_dot,
        'theta_dot': theta_dot,
    }

    return r, float(v1), float(theta), chrono_state


# ══════════════════════════════════════════════════════════════════════════════
# 6.  MRAC outputs -> Chrono driver inputs
# ══════════════════════════════════════════════════════════════════════════════

def make_driver_inputs(throttle: float, braking: float,
                       delta: float, v1: float) -> veh.DriverInputs:
    inp = veh.DriverInputs()
    inp.m_steering = float(np.clip(delta / MAX_STEER_RAD, -1.0, 1.0))
    inp.m_throttle = float(np.clip(throttle, 0.0, 1.0))
    inp.m_braking  = float(np.clip(braking,  0.0, 1.0))
    return inp


# ══════════════════════════════════════════════════════════════════════════════
# 7.  Controller ODE for RK4
#     Injects frozen Chrono state + chrono_state dict at every k-evaluation.
#     Zeros xdot[0:4] so RK4 never integrates plant slots.
# ══════════════════════════════════════════════════════════════════════════════

def make_ctrl_ode(r_c, v1_c, theta_c, chrono_state, parameters):
    def f(t, x):
        x_in      = x.copy()
        x_in[0:2] = r_c
        x_in[2]   = v1_c
        x_in[3]   = theta_c
        xdot, _   = dynamical_system(t, x_in, parameters,
                                     chrono_state=chrono_state)
        xdot[0:4] = 0.0
        return xdot
    return f


# ══════════════════════════════════════════════════════════════════════════════
# 8.  Main loop
# ══════════════════════════════════════════════════════════════════════════════

def run():

    # ── HMMWV (identical pattern to demo_VEH_SteeringController.py) ──────────
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(r_0[0], r_0[1], 0.5),
        chrono.QuatFromAngleZ(theta_0),
    ))
    hmmwv.SetInitFwdVel(v1_0)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_RWD)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # ── Terrain ───────────────────────────────────────────────────────────────
    terrain   = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 300, 300)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # ── Visualisation (same class as demo) ────────────────────────────────────
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle(f'HMMWV — MRAC [{variant}]')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Reference path spheres
    for i in range(300):
        ru, _,_,_= reference(i * T_END / 299)
        sph = chrono.ChBodyEasySphere(0.1, 100, True, False)
        sph.SetPos(chrono.ChVector3d(float(ru[0]), float(ru[1]), 0.1))
        sph.SetFixed(True)
        mat = chrono.ChVisualMaterial()
        mat.SetDiffuseColor(chrono.ChColor(0.2, 0.5, 1.0))
        sph.GetVisualShape(0).SetMaterial(0, mat)
        hmmwv.GetSystem().Add(sph)

    # Sentinel / target balls (same as demo)
    ballT = vis.GetSceneManager().addSphereSceneNode(0.5)
    ballS = vis.GetSceneManager().addSphereSceneNode(0.5)
    ballT.getMaterial(0).EmissiveColor = irr.SColor(0, 0, 255, 0)   # green = current
    ballS.getMaterial(0).EmissiveColor = irr.SColor(0, 255, 0, 0)   # red   = lookahead

    # ── Suspension settle ─────────────────────────────────────────────────────
    print("Settling suspension …")
    zero_inp = veh.DriverInputs()
    zero_inp.m_throttle = zero_inp.m_braking = zero_inp.m_steering = 0.0
    st = 0.0
    while st < 0.5:
        terrain.Synchronize(st)
        hmmwv.Synchronize(st, zero_inp, terrain)
        vis.Synchronize(st, zero_inp)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)
        st += step_size

    # Snap controller ICs to settled pose
    r_c, v1_c, theta_c, cs = extract_plant_state(hmmwv)
    theta_prev      = theta_c          # last raw wrapped value from Chrono
    theta_unwrapped = theta_c          # continuous accumulator fed to controller
    x = x0.copy()
    x[0:2] = r_c;  x[2] = v1_c;  x[3] = theta_c
    x[4:6]           = r_c        # r_cmd
    x[K_OL_end]      = theta_unwrapped    # x_IL_LPF[0]
    x[K_OL_end + 2]  = theta_unwrapped    # theta_ref
    print(f"Settled: r={r_c}  v1={v1_c:.3f} m/s  "
          f"theta={math.degrees(theta_c):.1f} deg")

    # ── Speed controller state ────────────────────────────────────────────────
    v_des   = float(v1_c)   # desired speed, starts at settled speed (≈0)
    e_v_int = 0.0           # integral of speed error

    # ── Log storage (same variable names as main.py section 8) ───────────────
    N_max = int(T_END / step_size) + 200
    t_arr          = np.empty(N_max)
    r_arr          = np.empty((N_max, 2))
    v1_arr         = np.empty(N_max)
    theta_arr      = np.empty(N_max)
    r_cmd_arr      = np.empty((N_max, 2))
    r_user_arr_    = np.empty((N_max, 2))
    r_user_dot_arr_= np.empty((N_max, 2))
    v_ref_arr      = np.empty((N_max, 2))
    u_force_arr    = np.empty(N_max)
    theta_cmd_arr  = np.empty(N_max)
    delta_arr      = np.empty(N_max)
    u_tilde_arr    = np.empty((N_max, 2))
    OL_error_arr   = np.empty((N_max, 2))
    IL_error_arr   = np.empty(N_max)
    K_OL_norm_arr  = np.empty(N_max)
    K_IL_norm_arr  = np.empty(N_max)
    theta_ref_arr  = np.empty(N_max)
    v_chrono_arr   = np.empty((N_max, 2))    # true rear-axle world velocity
    theta_dot_arr  = np.empty(N_max)         # true yaw rate
    u_moment_arr   = np.empty(N_max)         # inner-loop moment command
    v_des_arr      = np.empty(N_max)         # desired speed from PI integrator

    step = 0
    print(f"Running [{variant}]  T={T_END}s  dt={step_size}s …")

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()
        if time >= T_END:
            break

        # 1. Plant state + all derivatives from Chrono
        r_c, v1_c, theta_c, cs = extract_plant_state(hmmwv)
        dth = theta_c - theta_prev
        dth = (dth + math.pi) % (2 * math.pi) - math.pi   # wrap diff to (-π, π]
        theta_unwrapped += dth
        theta_prev = theta_c
        x[0:2] = r_c;  x[2] = v1_c;  x[3] = theta_unwrapped

        # 2. MRAC outputs — pass chrono_state so v, v_dot, theta_dot
        #    are used directly instead of the bicycle model reconstructions
        _, out  = dynamical_system(time, x, parameters, chrono_state=cs)
        u_force = out['u_force']
        delta   = out['delta']

        # 3. Speed PI controller: track reference speed directly
        v_des   = float(np.linalg.norm(out['r_user_dot']))
        e_v     = v_des - v1_c
        e_v_int += e_v * step_size
        raw_cmd  = KP_SPEED * e_v + KI_SPEED * e_v_int
        driver_inputs = make_driver_inputs(max(raw_cmd, 0.0), max(-raw_cmd, 0.0),
                                           delta, v1_c)

        # 4. Synchronise (demo pattern)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # 5. Advance Chrono
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        # 6. RK4 — controller states only, with Chrono state injected
        x = rk4(make_ctrl_ode(r_c, v1_c, theta_unwrapped, cs, parameters),
                 step_size, time, x)
        x[0:2] = r_c;  x[2] = v1_c;  x[3] = theta_unwrapped  # re-stamp

        # 7. Moving balls
        ru_now,   _,_,_ = reference(time)
        ru_ahead, _,_,_ = reference(min(time + 3.0, T_END))
        ballT.setPosition(irr.vector3df(float(ru_now[0]),   float(ru_now[1]),   0.4))
        ballS.setPosition(irr.vector3df(float(ru_ahead[0]), float(ru_ahead[1]), 0.4))

        # 8. Render
        if step % RENDER_STEP == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # 9. Log
        ru, rud, _,_ = reference(time)
        t_arr[step]           = time
        r_arr[step]           = r_c
        v1_arr[step]          = v1_c
        theta_arr[step]       = theta_unwrapped
        r_cmd_arr[step]       = x[4:6]
        r_user_arr_[step]     = ru
        r_user_dot_arr_[step] = rud
        v_ref_arr[step]       = x[8:10]
        u_force_arr[step]     = u_force
        theta_cmd_arr[step]   = out['theta_cmd']
        delta_arr[step]       = delta
        u_tilde_arr[step]     = out['u_tilde_force']
        OL_error_arr[step]    = out['OL_error']
        IL_error_arr[step]    = float(out['IL_error'])
        K_OL_norm_arr[step]   = np.linalg.norm(x[K_OL_start:K_OL_end])
        K_IL_norm_arr[step]   = np.linalg.norm(x[K_IL_start:K_IL_end])
        theta_ref_arr[step]   = x[THETA_REF_IDX]
        v_chrono_arr[step]    = cs['v']
        theta_dot_arr[step]   = cs['theta_dot']
        u_moment_arr[step]    = out['u_moment']
        v_des_arr[step]       = v_des

        step += 1
        print(f"t = {time:.3f} s", end="\r")

    print(f"\nFinished — {step} steps")
    s = step
    return (t_arr[:s], r_arr[:s], v1_arr[:s], theta_arr[:s],
            r_cmd_arr[:s], r_user_arr_[:s], r_user_dot_arr_[:s],
            v_ref_arr[:s], u_force_arr[:s], theta_cmd_arr[:s],
            delta_arr[:s], u_tilde_arr[:s], OL_error_arr[:s],
            IL_error_arr[:s], K_OL_norm_arr[:s], K_IL_norm_arr[:s],
            theta_ref_arr[:s], v_chrono_arr[:s], theta_dot_arr[:s],
            u_moment_arr[:s], v_des_arr[:s])


# ══════════════════════════════════════════════════════════════════════════════
# 9.  Plots 
# ══════════════════════════════════════════════════════════════════════════════

def plot_results(t, r, v1, theta, r_cmd, r_user_arr, r_user_dot_arr,
                 v_ref_state, u_force, theta_cmd, delta,
                 u_tilde_force, OL_error, IL_error,
                 K_hat_OL_norm, K_hat_IL_norm, theta_ref,
                 v_chrono, theta_dot, u_moment, v_des):

    blue = [0, 102/255, 1]
    tag  = variant.replace('-','_')

    def save(name):
        plt.axis('tight')
        plt.savefig(f'cosim_{tag}_{name}.png')

    # Path (XY)
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(r[:,0], r[:,1],             'k',       lw=2.0, label='r(t)')
    ax.plot(r_cmd[:,0], r_cmd[:,1],     'r',       lw=2.0, label='r_cmd(t)')
    ax.plot(r_user_arr[:,0], r_user_arr[:,1], color=blue,  label='r_user(t)')
    ax.set_aspect('equal'); ax.legend(); ax.set_title('Path — rear wheel'); ax.set_xlabel('r[0] (m)'); ax.set_ylabel('r[1] (m)')
    save('path')

    # Position vs time
    fig, ax = plt.subplots(figsize=(7, 4))
    # ax.plot(t, r[:,0],           '--k', lw=1.5)
    ax.plot(t, r[:,1],            '-k', lw=1.5, label='r(t)')
    # ax.plot(t, r_cmd[:,0],       '--r', lw=2.0)
    ax.plot(t, r_cmd[:,1],        '-r', lw=2.0, label='r_cmd(t)')
    # ax.plot(t, r_user_arr[:,0],  '--',  color=blue)
    ax.plot(t, r_user_arr[:,1],   '-',  color=blue, label='r_user(t)')
    ax.set_xlabel('Time [s]'); ax.set_ylabel('r[1] (m)'); ax.legend(); ax.set_title('Position vs time')
    save('position')

    # Velocity components — use true Chrono velocity, not reconstructed
    fig, axes = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t, v_chrono[:,i],        '-k', lw=2, label=f'v_chrono[{i+1}]')
        ax.plot(t, v_ref_state[:,i],     '-r', lw=2, label=f'v_ref[{i+1}]')
        ax.plot(t, r_user_dot_arr[:,i],  '--b',lw=2, label=f'r_user_dot[{i+1}]')
        ax.legend(fontsize=10)
    axes[-1].set_xlabel('Time [s]')
    plt.suptitle('Velocity components (Chrono rear-axle truth)')
    save('velocity')

    # Steering angle
    fig, ax = plt.subplots(figsize=(7, 3))
    ax.plot(t, np.degrees(delta), 'k', lw=2)
    ax.set_xlabel('t [s]'); ax.set_ylabel('delta [deg]')
    ax.set_title('Steering angle'); save('delta')

    # Heading angles
    fig, ax = plt.subplots(figsize=(7, 3))
    ax.plot(t, np.degrees(theta),     '-k',  lw=2, label='theta(t)')
    ax.plot(t, np.degrees(theta_ref), '--r', lw=2, label='theta_ref(t)')
    ax.plot(t, np.degrees(theta_cmd), '-.b', lw=2, label='theta_cmd(t)')
    ax.set_xlabel('t [s]'); ax.set_ylabel('[deg]'); ax.legend()
    ax.set_title('Heading angles'); save('heading')

    # Yaw rate (extra co-sim plot)
    fig, ax = plt.subplots(figsize=(7, 3))
    ax.plot(t, np.degrees(theta_dot), 'k', lw=2)
    ax.set_xlabel('t [s]'); ax.set_ylabel('theta_dot [deg/s]')
    ax.set_title('Yaw rate (Chrono)'); save('yaw_rate')

    # Adaptive gain norms
    fig, ax = plt.subplots(figsize=(7, 3))
    ax.plot(t, K_hat_OL_norm, 'k',        label='||K_hat_OL||')
    ax.plot(t, K_hat_IL_norm,  color=blue, label='||K_hat_IL||')
    ax.legend(); ax.set_xlabel('Time [s]'); ax.set_title('Adaptive gain norms')
    save('gains')

    # Tracking errors
    fig, ax = plt.subplots(figsize=(7, 3))
    ax.plot(t, OL_error[:,0], 'k',        label='e_OL[1]')
    ax.plot(t, OL_error[:,1], '--k',      label='e_OL[2]')
    ax.plot(t, IL_error,       color=blue, lw=2, label='e_IL')
    ax.legend(); ax.set_xlabel('Time [s]'); ax.set_title('Tracking errors')
    save('errors')

    # Moment command (inner loop)
    fig, ax = plt.subplots(figsize=(7, 3))
    ax.plot(t, u_moment, 'k', lw=2)
    ax.set_xlabel('t [s]'); ax.set_ylabel('u_moment [N·m]')
    ax.set_title('Inner-loop moment command'); save('u_moment')

    # Speed tracking (PI controller)
    fig, ax = plt.subplots(figsize=(7, 3))
    ax.plot(t, v_des, '-r', lw=2, label='v_des')
    ax.plot(t, v1,    '-k', lw=2, label='v1 (Chrono)')
    ax.set_xlabel('t [s]'); ax.set_ylabel('Speed [m/s]')
    ax.legend(); ax.set_title('Longitudinal speed (PI controller)'); save('speed')

    # Force direction mismatch
    fig, ax = plt.subplots(figsize=(7, 3))
    ax.plot(t, u_tilde_force[:,0] - u_force*np.cos(theta),
            label='u_tilde1 - u_force*cos(theta)')
    ax.plot(t, u_tilde_force[:,1] - u_force*np.sin(theta),
            label='u_tilde2 - u_force*sin(theta)')
    ax.legend(); ax.set_xlabel('Time [s]')
    ax.set_title('Force direction mismatch'); save('force_mismatch')

    print(f"Plots saved to cosim_{tag}_*.pdf")
    plt.show()


# ══════════════════════════════════════════════════════════════════════════════
# 10.  CSV export
# ══════════════════════════════════════════════════════════════════════════════

def save_csv(t, r, v1, theta, r_cmd, r_user_arr, r_user_dot_arr,
             v_ref_state, u_force, theta_cmd, delta,
             u_tilde_force, OL_error, IL_error,
             K_hat_OL_norm, K_hat_IL_norm, theta_ref,
             v_chrono, theta_dot, u_moment, v_des):

    tag  = variant.replace('-', '_')
    path = f'cosim_{tag}_data.csv'

    header = ','.join([
        't',
        'r_x', 'r_y',
        'v1',
        'theta',
        'r_cmd_x', 'r_cmd_y',
        'r_user_x', 'r_user_y',
        'r_user_dot_x', 'r_user_dot_y',
        'v_ref_x', 'v_ref_y',
        'u_force',
        'theta_cmd',
        'delta',
        'u_tilde_x', 'u_tilde_y',
        'OL_error_x', 'OL_error_y',
        'IL_error',
        'K_OL_norm', 'K_IL_norm',
        'theta_ref',
        'v_chrono_x', 'v_chrono_y',
        'theta_dot',
        'u_moment',
        'v_des',
    ])

    data = np.column_stack([
        t,
        r,
        v1,
        theta,
        r_cmd,
        r_user_arr,
        r_user_dot_arr,
        v_ref_state,
        u_force,
        theta_cmd,
        delta,
        u_tilde_force,
        OL_error,
        IL_error,
        K_hat_OL_norm,
        K_hat_IL_norm,
        theta_ref,
        v_chrono,
        theta_dot,
        u_moment,
        v_des,
    ])

    np.savetxt(path, data, delimiter=',', header=header, comments='')
    print(f'Data saved to {path}')


# ══════════════════════════════════════════════════════════════════════════════
# 11.  Entry point
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    results = run()
    save_csv(*results)
    plot_results(*results)
