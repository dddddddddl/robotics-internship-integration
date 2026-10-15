# Recorded IK experiments

## Evidence files

- [records.csv](records.csv): English field names, numeric values, outcome status, source record, and source SHA-256. Includes 70 formal tests and one explicitly separated repeat.
- [summary.json](summary.json): counts recomputed by [summarize_ik.py](../../tools/summarize_ik.py).
- [source_evidence.zip](source_evidence.zip): supplied JSON summaries, test 001 terminal output, the final arm-test workbook, and a separate leg/body IK reference workbook. Local attachment/user paths in JSON and an embedded workbook source path have been redacted; numerical results are unchanged. Original Chinese narrative fields are retained for traceability.

Most JSON files summarize terminal output discussed during testing; they are not raw controller logs. Only test 001 has a supplied raw terminal transcript. Source attachments mentioned by the other summaries were not included. The final arm workbook is another presentation of the test records, not an independent measurement source. The leg workbook concerns leg/body scans and must not be treated as arm-workspace evidence.

## Recomputed outcome counts

| Category | Count |
| --- | ---: |
| Formal tests 001–070 | 70 |
| Recorded IK successes | 54 |
| Recorded IK failures or service safety rejections | 16 |
| Formal records describing actual arm motion | 29 |
| Additional repeat of test 070 | 1 |
| Recorded suspected-contact events | 1 (test 028) |

`recorded_motion` comes from explicit motion fields where available and otherwise the record's executed-status label. `execute_requested` alone is insufficient: several requests failed before motion. These counts reconcile the supplied summaries; no physical experiment was rerun during preparation.

## Boundary observations

The later full-body runs 046–070 were dry runs. Enabling a leg solution in the solver did not enable or move the robot's legs. Each bracket below is conditional on its recorded starting pose, orientation, control point, solver settings, and acceptance rules.

| Direction / setup | Last recorded success | First nearby rejection | Interval | Record IDs / interpretation |
| --- | ---: | ---: | ---: | --- |
| Full-body X+ | 468.75 mm | 475 mm | 6.25 mm | 052 / 050; service rejected excessive leg-joint change |
| Full-body Y+ | 268.75 mm | 275 mm | 6.25 mm | 058 / 056; numerical iteration limit |
| Full-body Z+ | 756.25 mm | 762.5 mm | 6.25 mm | 066 / 065; service leg-change constraint |
| Full-body Z− | 350 mm | 400 mm | 50 mm | 070 / 069; no 375 mm test was supplied |
| Earlier low-pose Y+ | 75 mm | 80 mm | 5 mm | 018 / 021; distinct baseline from later full-body tests |

These intervals are not validated absolute workspace limits. A numerical failure can depend on initialization or iteration budget; a safety rejection can occur despite numerical convergence. Intervals from different poses cannot be merged into a single robot workspace.

Physical Y− testing provides a different observation: at cumulative −400 mm no contact was reported (027), while at −450 mm the summary reports suspected waist contact despite successful IK and arrival (028). Retraction was recorded in 029. The 50 mm interval is a setup-specific clearance concern; absence of a contact report is not a verified collision-free certificate.

## Reproduce the tables

```bash
python tools/summarize_ik.py --output-dir results/ik --plot results/ik-outcomes.png
python experiments/ik/view_ik_result.py data/ik/records.csv --point-index 7 --arm A --no-plot
```

The viewer displays a model FK result for the recorded arm joints. It does not recreate the original full-body/fingertip frame or independently validate those reported errors. The original backend and `PinocchioFullBodyIK` interface are missing; see [runtime setup](../../docs/robot-setup.md).
