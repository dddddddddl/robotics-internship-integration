from matlab_style_scripts import run_left_arm


LEFT_ARM_ANGLES_DEG = (31.458808, -66.508254, -51.083165, -31.341856, 13.96246, -18.133934, 8.254045)


if __name__ == "__main__":
    run_left_arm(q_deg=LEFT_ARM_ANGLES_DEG)
