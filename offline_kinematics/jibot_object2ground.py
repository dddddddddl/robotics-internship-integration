from matlab_style_scripts import run_object2ground


LEG_ANGLES_DEG = (0.0, -0.0, 0.0, 0.0)
HEAD_ANGLES_DEG = (0.0, 0.0)
OBJECT_XYZ_CAM = (0.30, 0.00, 0.80)


if __name__ == "__main__":
    run_object2ground(q_leg_deg=LEG_ANGLES_DEG, q_head_deg=HEAD_ANGLES_DEG, object_xyz_cam=OBJECT_XYZ_CAM)
