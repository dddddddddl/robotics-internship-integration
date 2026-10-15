from matlab_style_scripts import run_all_frames


LEG_ANGLES_DEG =    [0.0, 0.0, 0.0, 0.0]
LEFT_ARM_ANGLES_DEG =   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0]

RIGHT_ARM_ANGLES_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0]
HEAD_ANGLES_DEG = (0.0, 0.0)


if __name__ == "__main__":
    run_all_frames(
        q_leg_deg=LEG_ANGLES_DEG,
        q_left_deg=LEFT_ARM_ANGLES_DEG,
        q_right_deg=RIGHT_ARM_ANGLES_DEG,
        q_head_deg=HEAD_ANGLES_DEG,
    )
