# Showcase evidence: what the recording is, and what it is not

A demonstration video of a robot is worth nothing unless a viewer can tell
whether they are watching a machine or a render. This states the guarantee, and
`tests/test_showcase_video_assets.py` enforces it — the claims below are checked
by a test, not left to the honesty of whoever last edited this file.

## The two scripts

| Script | What it produces |
|---|---|
| `run-ai4all-gui-evidence.sh` | Captures the run: Gazebo, the robot, the operator view |
| `render-ai4all-verification-video.sh` | Assembles the capture into the verification video |

## The guarantees

**No generative imagery.** Nothing in the output is synthesised, in-painted,
interpolated or model-generated. Every frame came off a real capture of a real
run. A viewer comparing the video to the machine sees the same thing.

**A continuous timeline.** The recording is not cut to hide a failure, a retry
or a reset. If the robot stopped, waited, or was refused by a safety check, that
is in the recording at the time it happened. Editing that out would turn
evidence into an advertisement, which is the whole distinction this file exists
to hold.

## Why it is a test and not a promise

Both properties are asserted in `test_showcase_documentation_distinguishes_verification_from_promotion`.
A document nothing checks drifts the first time someone is in a hurry before a
deadline — which is exactly when the temptation to cut a bad take is highest.
