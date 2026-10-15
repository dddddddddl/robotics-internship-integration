# IK experiment utilities

[ik_experiment_core.py](ik_experiment_core.py) is the supplied pure-NumPy helper module for rigid-transform validation, position/orientation residuals, line waypoint spacing, X–Z grids, and sampled boundary summaries. These functions generate and analyze targets; they do not invoke an IK solver or guarantee collision avoidance.

[view_ik_result.py](view_ik_result.py) reads seven joint values from a CSV row and invokes the included offline model. Its model import path was repaired after reorganizing the source:

```bash
python experiments/ik/view_ik_result.py data/ik/records.csv --point-index 7 --arm A --no-plot
```

The existing helper tests are in [tests/test_ik_experiment_core.py](../../tests/test_ik_experiment_core.py). Extra preparation checks cover source-record reconciliation and protocol decoding.

The later original client files `20_ik_hand_move_target_matrix.py` and `21_arm_ik_line_boundary.py` are retained in [later_full_body_clients.zip](../../references/later_full_body_clients.zip). They depend on a `PinocchioFullBodyIK` interface and a matching full-body backend absent from the supplied August runtime. Their helper module is included here, but the archive is a reference snapshot rather than a runnable full-body system. The exact dated client filename cited in the test transcripts is also absent; do not assume these undated clients are byte-identical to that tested version.


[Fingertip-target IK development](../../docs/fingertip-ik-development.md) adds the later client's sensor/TCP transformation, missing-body-state diagnosis, explicit zero-state calculation mode and separation of arm execution from full-body calculation. The source versions remain the existing reference archive.
