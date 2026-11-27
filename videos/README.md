# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `MOT-U02`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | obstacle | OBS-U04 | `abc1234` | | <https://youtu.be/...> | servo scans 60° and 120° when blocked |
| 2026-11-20 | terrain | TER-U34 (S6) | `bc6bd73` | | [video](terrain/2026-11-20_S6_TER-U34_lift_side_overturn.MOV) | brisk lift, on its side and upside down: each stops the motors until Start |
| 2026-11-20 | terrain | TER-U17 (S4) | `0fec680` (branch `189`) | | [video](terrain/2026-11-20_S4_TER-U17_flat_lap_0_humps.MOV) | full flat lap with the hump off the track: Humps stays 0 |
| 2026-11-20 | terrain | TER-U33 (S5) | `0fec680` (branch `189`) | | [video](terrain/2026-11-20_S5_TER-U33_hump_lap_1_hump.MOV) | lap over the 11 mm hump: counted once, Humps 1 |
| 2026-11-20 | terrain | TER-U33 (S5) | `0fec680` (branch `189`) | | [video](terrain/2026-11-20_S5_TER-U33_hump_left_wheel.MOV) | hump under the left wheel |
| 2026-11-20 | terrain | TER-U33 (S5) | `0fec680` (branch `189`) | | [video](terrain/2026-11-20_S5_TER-U33_hump_right_wheel.MOV) | hump under the right wheel |
