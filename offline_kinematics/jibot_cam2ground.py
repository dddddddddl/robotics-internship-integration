from matlab_style_scripts import run_cam2ground


LEG_ANGLES_DEG = (0.0, -0.0, 0.0, 0.0)
HEAD_ANGLES_DEG = (0.0, 0.0)


if __name__ == "__main__":
    run_cam2ground(q_leg_deg=LEG_ANGLES_DEG, q_head_deg=HEAD_ANGLES_DEG)
