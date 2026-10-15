from matlab_style_scripts import run_body2ground


LEG_ANGLES_DEG = [57.7940698, -79.0634571, 21.2693873, 8.90554396e-15]


if __name__ == "__main__":
    run_body2ground(q_deg=LEG_ANGLES_DEG)
