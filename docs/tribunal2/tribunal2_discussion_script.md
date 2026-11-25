# Tribunal 2 discussion script

Follows `tribunal2_analysis.txt` row by row. About 55 minutes at a normal
speaking pace, with pauses to look at the file.

**Who speaks for what**

- Buddy 1: communication (Wi-Fi, MQTT, dashboard commands, telemetry)
- Buddy 2: motion (tMotion, encoders, motors)
- Buddy 3: line following and barcode
- Buddy 4: terrain (IMU)
- Buddy 5: obstacle (ultrasonic, servo, bypass, motor hold)

There is no chair. Each row ends with one member putting the decision to the
team and everyone agreeing it.

**Timing guide**

- Opening: 3 min
- Part 1, fault record (row 2): 4 min
- Part 2, obstacle inputs (rows 3, 4): 6 min
- Part 3, motion (rows 5, 6, 7): 7 min
- Part 4, line and barcode (rows 8, 9): 6 min
- Part 5, terrain (rows 10, 11): 7 min
- Part 6, communication (rows 12, 13): 6 min
- Part 7, tasks that hang (rows 14, 15, 16): 7 min
- Part 8, state machines, interfaces, interrupts (rows 17, 18, 19): 5 min
- Part 9, timing and memory (rows 20, 21): 3 min
- Part 10, recovery, tests, requirement wording (rows 22, 23, 1): 6 min
- Close: 2 min

**Before you start**

- Each row is a suggestion from the draft analysis. The owner's job is to
  check it against their own code and say whether the decision is right.
- Read your own lines against your code first. If a figure or a decision is
  wrong, change the line and the row in the text file.
- "Proposed" means a new design value that is not in the code. Say so when
  you use one.
- Use this as a guide, not word for word. Know the figure behind each line
  you say, because the assessors will ask.

---

## Opening (3 min)

**Buddy 1:** The injected requirement is: if any part of the system fails or
gives invalid data, the system must respond in a safe and predictable way.

**Buddy 3:** The example in it is a measurement that becomes unavailable or
invalid. The system must not carry on as though the value were correct. It
needs a defined response: limited operation, stopping the function,
attempting recovery, or asking for intervention.

**Buddy 2:** I suggest we go through the car input by input and ask the same
four questions each time. How can this input fail or lie? How do we know, and
how fast? What does the car do? How does it come back?

**Buddy 4:** And we say which parts change and which stay. Coverage is
assessed, so a No Change has to be argued, not skipped.

**Buddy 5:** Implementation is not required. Where we propose a new value we
call it proposed and name the test that would prove it.

**Buddy 1:** Row 1 is the requirement wording. It depends on everything else,
so we confirm it last.

**All:** Agreed.

---

## Part 1: one shared fault record (row 2, 4 min)

**Buddy 4:** I start, because my module shows the problem. In terrain I
already keep an IMU fault bit in `terrain_fault.h`. It is shaped for a team
fault mask, but the team mask does not exist. Only my own task reads the bit,
and only to print it.

**Buddy 2:** Motion has the same gap. `motion.h` has a TODO to move STALL and
ENC_FAULT into a shared mask.

**Buddy 5:** Mine is the opposite. My bypass publishes a 4-bit fault reason
in the obstacle control word, so my faults are visible. But only my own.

**Buddy 1:** The status record carries the mode, the obstacle status, the
bypass state, the calibration state and the lift count. There is no single
place that says "something is invalid".

**Buddy 3:** And rows 10 to 12 need one task to react to another task's
fault. My line task has to slow down when terrain is invalid. Without a
shared record I would have to reach into terrain's internals.

**Buddy 1:** The suggestion was one global word that every task ORs its bit
into. Why not just do that?

**Buddy 2:** Our tasks run on both cores. Two tasks doing read-modify-write
on one word can lose a bit, and then the fault record is itself invalid data.
The kernel's `tk_set_flg` and `tk_clr_flg` are atomic, so we use one event
flag object.

**Buddy 4:** Could each owner keep its own byte, as I do now?

**Buddy 2:** That is safe to write, because each byte has one writer. But a
reader then needs five reads and the status record needs five fields. One
event flag gives one place, and a reader polls it with a zero timeout, the
way we already poll the telemetry mutex. Nobody waits, so NFR6 still holds.

**Buddy 5:** Who may clear a bit?

**Buddy 4:** Only the owner that set it. Mine clears on the first good read.
A latched one, like a bypass FAULT, stays until reboot.

**Buddy 1:** The budget allows it: event flags limit 16, we use 3. Nine fault
bits fit one flag word, and I publish them in the status record.

**Buddy 3:** So we use the idea but change the solution: an event flag, with
each owner setting and clearing only its own bits. Modify?

**All:** Agreed.

*Team decision: Modify.*

---

## Part 2: obstacle inputs (rows 3 and 4, 6 min)

### Row 3: ultrasonic reading invalid or missing

**Buddy 5:** This is my sensor. The driver gives three results, not two.
VALID is a captured pulse. NO_ECHO is a pulse that rose but ran past 23 ms.
FAULT is no rising edge within 2 ms, which means the sensor did not answer.
An unplugged sensor is a FAULT. It is never read as "clear".

**Buddy 2:** What does a fault do?

**Buddy 5:** It requests a stop at once. And if there is no usable reading
for 200 ms, that also requests a stop. The 200 comes from three pings at
60 ms plus two 10 ms scheduling margins.

**Buddy 2:** How far does the car travel in that 200 ms?

**Buddy 5:** 24 mm at 120 mm/s. The stop distance is 150 mm.

**Buddy 3:** NO_ECHO counts as clear. Is that using a missing value as
though it were correct?

**Buddy 5:** No, because the sensor answered. With our calibration, distance
is 0.17 times the echo time minus 18 mm, so a 23 ms pulse is about 3.9 m.
NO_ECHO means nothing within 3.9 m.

**Buddy 4:** What if the echo wire is stuck high or stuck low?

**Buddy 5:** Either way there is no rising edge, so it is a FAULT, the same
as unplugged.

**Buddy 1:** Does one bad ping stop the car for good?

**Buddy 5:** No. A fault uses up one of two extra attempts in the group of
three. If the next pings are good, the group completes and the car is
released. If the extras run out, the group is thrown away and three new votes
are needed. So one good ping can never release the car on its own.

**Buddy 2:** What do you publish while you do not know?

**Buddy 5:** UNKNOWN, not clear. And at power-up my stop request starts as
true, so the car is held until a clear path is confirmed. The default is the
safe one.

**Buddy 3:** Is there anything this does not catch?

**Buddy 5:** A surface that absorbs the sound returns nothing and looks like
open space. That is a limit of the sensor, not of the handling. OBS-U26
covers the echo line being disconnected.

**Buddy 2:** The sensor is never trusted when it does not answer, and the
blind travel is 24 mm against 150. No Change?

**All:** Agreed.

*Team decision: No Change.*

### Row 4: servo stuck

**Buddy 5:** The suggestion is to detect a stuck servo when the echoes are
all alike across the sweep. I reject it. The servo is PWM only, with no
feedback wire, so I cannot measure its angle. And on 8 November, 11, 16 and
21 cm obstacles all gave the same echoes over the same angles. "All alike" is
normal data for us, so the check would raise false faults.

**Buddy 4:** Then what protects us if it sticks during a bypass?

**Buddy 5:** Four things that exist already. The side scan times out at twice
its worst case. After a sidestep the corridor sweep has to verify the gap. A
reading under 80 mm during a move backs the car off 50 mm, twice at most. A
second problem latches FAULT with the motors held.

**Buddy 3:** What if it is stuck pointing sideways while we are only driving?

**Buddy 5:** Then the forward ping is not looking ahead, and software cannot
know. That is the residual risk in this row. Fixing it needs hardware: a
servo with feedback, or a fixed forward sensor. We state it, we do not hide
it. The operator stops still work.

**Buddy 1:** Should we add a fault bit for it anyway?

**Buddy 5:** There is nothing to set it from. A bit that can never be set
would be a false assurance.

**Buddy 4:** So the check is rejected, the existing bounds stay, and the
residual risk is written in the row. Reject?

**All:** Agreed.

*Team decision: Reject.*

---

## Part 3: motion (rows 5, 6 and 7, 7 min)

### Row 5: encoder invalid during a move or turn

**Buddy 2:** The encoders are counted in hardware by PIO, one state machine
per encoder, and I read both counts every 20 ms. They can be wrong in two
ways: no counts, or wrong counts.

**Buddy 3:** Take no counts first.

**Buddy 2:** No counts looks like a wheel that is not turning. After 300 ms
that is a stall: both wheels brake and the mode is STALLED. MOT-U10 and
MOT-U16 cover a held wheel in a move and in a turn.

**Buddy 4:** And wrong counts?

**Buddy 2:** An encoder counting the wrong way is not a stall, because the
wheel has speed. So every move and turn also has a time limit: three times
the expected time plus 500 ms. Then I raise ENC_FAULT, brake, and stay
TIMED_OUT.

**Buddy 3:** Give an example.

**Buddy 2:** A 300 mm move at 300 mm/s should take 1 second. The limit is
3.5 seconds.

**Buddy 5:** In a bypass I add my own limit on each leg: twice the expected
time plus 1 second, one retry, then FAULT. And ENC_FAULT from motion is
FAULT_ENCODER straight away.

**Buddy 4:** Can a reader get a half-written odometry value?

**Buddy 2:** No. It is a sequence-checked copy: the reader copies, checks the
sequence number did not change, and retries if it did.

**Buddy 5:** If I still cannot get a consistent copy, that is FAULT_ODOMETRY
and the motors are held.

**Buddy 1:** After STALLED or TIMED_OUT, can a new command slip through?

**Buddy 2:** No. A new move is refused until a STOP. That is MOT-U37.

**Buddy 3:** Both kinds of bad encoder end in a brake and a latched state,
and nobody reads a torn copy. No Change?

**All:** Agreed.

*Team decision: No Change.*

### Row 6: stall or dead encoder while following

**Buddy 2:** Following is different. A wheel commanded above 25 mm/s that
shows no speed for 300 ms is a stall, and both wheels brake. Then following
restarts. Since MOT-U43 there is no limit on restarts.

**Buddy 3:** So a dead encoder restarts for ever?

**Buddy 2:** Yes, that is the gap. I cannot tell a jammed wheel from a dead
encoder: both read zero. The suggestion was to stop for good on the first
stall. I would modify it: keep the restart, but after three stalls in a row
on the same wheel, with no counts from that wheel in between, latch a fault
and stop until Start.

**Buddy 4:** Why not stop on the first one?

**Buddy 2:** A real stall clears and the wheel counts again. MOT-U43 shows
following resuming after a stall. Stopping on the first would end runs that
recover today.

**Buddy 5:** Why three?

**Buddy 2:** MOT-U39 used a limit of three before MOT-U43 removed it. Two
bits per wheel hold the count.

**Buddy 1:** What resets the count?

**Buddy 2:** Any counts from that wheel between stalls. That is the evidence
that the encoder is alive.

**Buddy 5:** How long does the car misbehave in the worst case?

**Buddy 2:** Three stall periods of 300 ms each before the brake. Then a
latched stop and the encoder bit in the fault record.

**Buddy 4:** That keeps the recovery and bounds the dead encoder. Modify,
with the limit of three?

**All:** Agreed.

*Team decision: Modify.*

### Row 7: battery voltage and motor driver

**Buddy 2:** The suggestion is to measure battery voltage and stop when it is
low. I reject it.

**Buddy 3:** There is no free analogue pin. GP26, 27 and 28 are my two line
sensors and the barcode sensor, used as digital inputs.

**Buddy 1:** The fourth ADC input, VSYS on GP29, is shared with the radio on
the Pico W, and the radio belongs to the kernel's radio task.

**Buddy 5:** What about the motor driver itself failing?

**Buddy 2:** We read no fault signal from the driver. My pins are PWM only.
But the effect is the same as a weak supply: the wheel does not reach speed,
and the 300 ms stall check and the move time limit act.

**Buddy 4:** And if the supply drops far enough to reset the Pico?

**Buddy 2:** Then it boots, and FR11 holds the motors until Start and a clear
path.

**Buddy 3:** So it is a hardware limit, and every effect we can see is
already caught. Reject?

**All:** Agreed.

*Team decision: Reject.*

---

## Part 4: line and barcode (rows 8 and 9, 6 min)

### Row 8: line sensor stuck, line lost, line task silent

**Buddy 3:** My sensors are digital, so a broken one looks like steady black
or steady white. I do not trust the level. I limit it by distance travelled.

**Buddy 5:** Go through the cases.

**Buddy 3:** One sensor stuck black: an edge black for 200 mm starts a
search. Both stuck white: the line is lost after 250 mm in a bend or 600 mm
on a straight, and that starts a search. Both stuck black: that first looks
like a junction, both black within 14 mm, so I take the set turn or go
straight. But black continues, so the 200 mm rule starts a search. Every case
ends in a search, and a search that fails ends in a stop within 8 seconds.

**Buddy 5:** 600 mm on a straight is a long way with no line.

**Buddy 3:** It is 5 seconds at 120 mm/s, and it is bounded. Meanwhile the
car holds its heading from odometry, so it runs straight, and the obstacle
stop stays active.

**Buddy 4:** What does the search do to the rest of us?

**Buddy 3:** It is a pivot on the spot: up to 120 degrees towards the side
the line was last seen, then the other way. There is no forward travel.

**Buddy 2:** And if your task stops sending, I stop following when the newest
request is 100 ms old. That is five of my updates, and 12 mm of travel. I
never see half a request either: it is one word through a double buffer, with
a VALID bit.

**Buddy 1:** What happens after the stop?

**Buddy 3:** The car stays stopped. Row 22 lists it as needing Start.
LINE-U07 covers the search both ways and the stop.

**Buddy 5:** Both the sensor side and the motion side are bounded by
distance or time. No Change?

**All:** Agreed.

*Team decision: No Change.*

### Row 9: barcode misread

**Buddy 3:** First what happens today. Four edges within half a second mean a
barcode. The car brakes, reverses 170 mm and reads it at 30 mm/s, in either
direction.

**Buddy 1:** And the suggestion?

**Buddy 3:** A Code 39 check character. I reject it. It would have to be
printed on the barcodes on the course, and our decoded text is only star, one
character, star. A failed read sets no turn, and FR3 says the car then goes
straight on. It never guesses a turn.

**Buddy 4:** What about a partial read?

**Buddy 3:** It is used only if the dashboard table already has that text.
That rule came from a false "." read on 6 November.

**Buddy 1:** What if the edge queue overflows?

**Buddy 3:** It holds 64 edges. Dropped edges are counted, the scan fails,
and the result is the same: straight on. The overflow sets the barcode bit in
the fault record from row 2.

**Buddy 2:** What is the worst a wrong decode can do?

**Buddy 3:** A wrong turn at one junction. That is a navigation error, not a
safety one: on the wrong branch the car still follows the line and still
stops for obstacles. And the wrong text must be in the 8-entry table to cause
a turn at all. Anything unlisted means straight on.

**Buddy 5:** Noise on the sensor?

**Buddy 3:** Pulses shorter than 10 ms are ignored, and a scan with fewer
elements than one character is treated as noise, not as a failed read.

**Buddy 4:** And if the sensor is dead?

**Buddy 3:** No edges, no barcode, every junction straight on. That is
predictable.

**Buddy 1:** Straight on is safe and predictable, and we cannot change the
printed barcodes. Reject the check character?

**All:** Agreed.

*Team decision: Reject.*

---

## Part 5: terrain (rows 10 and 11, 7 min)

### Row 10: IMU stops answering during a run

**Buddy 4:** This is the requirement's own example in our car. Ten failed
reads in a row, about 100 ms at 100 Hz, raise the IMU fault and I set the
sensor up again. The next good read clears it. Each I2C transfer is
time-limited, so my task does not hang on the bus.

**Buddy 2:** So detection works. What is wrong?

**Buddy 4:** Only my own log line reads the fault. Nobody else is told.

**Buddy 3:** I take "on a hump" from you with no validity. With a dead IMU I
keep 120 mm/s over the hump.

**Buddy 5:** I take the lift count the same way. So the lift stop, our only
stop that needs no Wi-Fi, is silently gone.

**Buddy 4:** So I accept the row. The fault sets the IMU bit in the fault
record. Hump and lift results are reported as unavailable, not as zero.

**Buddy 2:** Does that pattern exist anywhere already?

**Buddy 4:** Yes. The compass word carries a VALID bit. Hump and lift need
the same.

**Buddy 1:** When the sensor comes back, do we trust it straight away?

**Buddy 4:** The fault bit clears on the first good read. Whether the old
calibration still holds after the sensor is set up again is something I have
to check in my code. My proposal is the careful one: results stay unavailable
until the next standing calibration, so the limited run from row 11 continues
until the car is stopped and started.

**Buddy 5:** And if your task itself hangs, there is no fault bit at all.

**Buddy 4:** Right. Proposed: the same rule as row 14. If my snapshot stops
updating for 100 ms, the readers treat it as IMU unavailable. TER-U11 already
tests the fault being raised and cleared.

**Buddy 3:** Then my hump speed and the lift stop both know when the value is
missing. Accept?

**All:** Agreed.

*Team decision: Accept.*

### Row 11: IMU calibration invalid at Start

**Buddy 4:** Calibration takes 200 samples, 2 seconds, with the car standing
still, and the car has to be within 5 degrees of level. The suggestion is to
refuse Start until it is VALID. On 9 November, 7 of 9 boots read NOT_LEVEL.
Refusing Start would make the whole car depend on its least repeatable input.

**Buddy 2:** But full speed with no hump detection uses "no hump" as though
it was measured. That is exactly what the requirement forbids.

**Buddy 4:** So modify it. Start is allowed, but the run is limited: speed is
capped at the hump speed, 30 mm/s, for the whole run, and hump and lift are
reported as unavailable.

**Buddy 5:** Why not relax the 5 degree limit so that it passes?

**Buddy 4:** Then the level reference is wrong, and every tilt, hump and lift
threshold is measured from a wrong zero. That is invalid data by design.

**Buddy 3:** The slow bit already exists in my line request word, so there is
no new interface. Line following, the obstacle stop and the bypass do not
need the IMU and keep working.

**Buddy 2:** One side effect at 30 mm/s. I do not count a stall on a wheel
commanded below 25 mm/s, and when steering at 30 the inner wheel can be below
that. So stall detection is weaker in the limited run. A wheel at 30 that
does not turn still stalls.

**Buddy 1:** The cost is a lap four times slower. The status record already
carries the calibration state, so the dashboard shows why.

**Buddy 5:** And the lift stop is off in this mode. That is why row 12
matters.

**Buddy 4:** So Start is allowed, the run is limited, and the limits are
reported. Modify?

**All:** Agreed.

*Team decision: Modify.*

---

## Part 6: communication (rows 12 and 13, 6 min)

### Row 12: Wi-Fi or MQTT link lost

**Buddy 1:** The suggestion is to stop as soon as the link drops. That makes
driving depend on Wi-Fi, which NFR7 exists to prevent. On 17 November MOT-U34
ran with the Wi-Fi join failed, and tMotion still held 19,945 to 20,066
microseconds. Driving does not need the link.

**Buddy 4:** But if the link is down and my IMU is invalid, no person can
stop the car. Only an obstacle can.

**Buddy 1:** So I modify it to a rule: the car must always have one working
operator stop. Link down and the lift stop works: carry on, queue telemetry,
retry every 3 seconds, as today. Link down and the lift stop is unavailable:
hold the motors until the link is back and Start is pressed.

**Buddy 2:** Is lifting the car really an operator stop?

**Buddy 4:** Yes, when my data is valid. Upward acceleration over 1.30 times
the level reading as a 100 ms mean, or more than 60 degrees of tilt for
300 ms, stops the motors until Start. No radio is involved.

**Buddy 5:** The rule fits my code. `remote_hold` already holds the motors
for the dashboard Stop and for a lift, with the same run flag and INHIBIT.
This is one more hold reason.

**Buddy 3:** Why not fit a stop button on the car?

**Buddy 1:** None is fitted. That is a hardware change and a new pin owner.
The rule gets the same guarantee from what is on the car.

**Buddy 2:** How fast do we know the link is down?

**Buddy 1:** My task checks the link every 500 ms.

**Buddy 5:** What if the link flaps?

**Buddy 1:** The hold is latched until Start, so the car does not stop and go
with the link. For telemetry, up to 128 records wait in the queue. The status
record is QoS 0, so a lost one is simply replaced by the next, 0.5 seconds
later.

**Buddy 3:** The car keeps running through a dropout when lifting still
stops it, and stops only when both operator stops are gone. Modify, and NFR7
is reworded?

**All:** Agreed.

*Team decision: Modify.*

### Row 13: bad dashboard command, full telemetry queue

**Buddy 1:** There are four commands: start, stop, clear and map. Anything
else, or a malformed one, returns REMOTE_COMMAND_BAD and changes nothing. A
map with all 8 entries used returns REMOTE_COMMAND_FULL.

**Buddy 3:** And the telemetry queue?

**Buddy 1:** When it is full the newest record is rejected, but it still uses
a sequence number, so the loss shows as a gap. If the mutex is busy the
caller is told to retry the same sample. The queue never blocks the caller.
COM-U10 and COM-U14 cover these.

**Buddy 2:** What about Start while a bypass FAULT is latched?

**Buddy 5:** The command is accepted, but my bypass stays in FAULT, so the
motors stay held.

**Buddy 4:** None of those changes what the car does. No Change?

**All:** Agreed.

*Team decision: No Change.*

---

## Part 7: tasks that hang (rows 14, 15 and 16, 7 min)

### Row 14: the obstacle task hangs or starves

**Buddy 5:** Every stop in the car goes through my task: obstacle, critical
distance, lift, dashboard Stop, bypass fault. If my task hangs, nothing sets
INHIBIT, and the line task keeps sending, so the car keeps following with
nobody checking ahead.

**Buddy 1:** Why would your task hang or starve?

**Buddy 5:** It is priority 11 on core 0. The kernel's USB console, priority
3, and radio task, priority 4, are on the same core and outrank it. If
either runs long, my 10 ms loop is late.

**Buddy 4:** Does your own 200 ms stale check not catch that?

**Buddy 5:** No. It runs inside my task. If the task is not running, neither
is the check.

**Buddy 2:** I already do this check for line requests: I count the age in
updates against a limit of 5. I can do the same with a one-byte counter from
you. If it has not changed for 100 ms, I brake and report "task stale". I am
on core 1, so a problem on core 0 does not stop me checking.

**Buddy 3:** How far does the car go before the brake?

**Buddy 2:** 100 ms plus one 20 ms update at 120 mm/s is 14.4 mm. My longest
update is 495 microseconds in a 20,000 microsecond period, so one compare
costs nothing.

**Buddy 1:** Is 100 ms safe from false trips?

**Buddy 5:** Not measured yet. My loop is 10 ms, so 100 ms is ten missed
loops. Row 23 has to measure my worst gap with Wi-Fi running before we fix
the number.

**Buddy 4:** What happens when your task comes back?

**Buddy 5:** If I was away for more than 200 ms, my own stale check has
already requested a stop, and I need three new votes before I release. So
tMotion can resume when the counter moves and INHIBIT is clear. The counter
is one byte in the state I already publish on IF-07. No new kernel object.

**Buddy 2:** Then I add the check in tMotion. Accept?

**All:** Agreed.

*Team decision: Accept.*

### Row 15: add the tSafety task

**Buddy 5:** The suggestion is to build tSafety at priority 5, as in Week 6.
I reject it.

**Buddy 1:** It would fit. One 2,048-byte stack is 0.8% of RAM, and the task
limit is 32.

**Buddy 5:** Memory is not the reason. It moves the single point of failure
to another task, and it adds a second writer of INHIBIT, against our
one-owner rule.

**Buddy 2:** And where would it run? At priority 5 on core 1 it could delay
my 20 ms update, which NFR1 forbids. On core 0 it sits behind the same USB
and radio tasks as the obstacle task.

**Buddy 4:** Week 6 planned buttons and a buzzer with it, and those were
never built.

**Buddy 5:** The checks belong in tMotion, which is the last thing before the
motors. That is rows 14 and 16.

**Buddy 1:** Agreed, ownership and timing are the stronger reasons. Reject?

**All:** Agreed.

*Team decision: Reject.*

### Row 16: tMotion hangs

**Buddy 2:** This is the worst failure we have. If my task hangs, the PWM
stays at its last duty and no other task can brake. The bypass latches
FAULT_STOP_ACK after 500 ms, but that only records it. It cannot stop the
wheels.

**Buddy 4:** The suggestion was to feed the watchdog from a low-priority
task.

**Buddy 2:** That proves the scheduler is alive, not that the motors are
being controlled. And it could starve when higher tasks are busy and reset a
healthy car. So I modify it: feed the watchdog from my 20 ms update only,
with a 100 ms timeout. That is five periods, and my period measures 19,945 to
20,066 microseconds.

**Buddy 5:** Is the watchdog clock even set up?

**Buddy 2:** Yes. `usermain` already sets the watchdog tick to 1 MHz from the
12 MHz crystal, because TIMER needs it. Only the reset part is not enabled.

**Buddy 5:** Is a reset safe?

**Buddy 2:** Yes. With the watchdog set to restart the reset controller, the
pins go back to output disabled with a pull-down, so the PWM is gone. And
FR11: the car powers up with the motors held until Start and a clear path.

**Buddy 3:** What do we lose on a reset?

**Buddy 1:** Everything in RAM: the telemetry queue, the barcode map and the
run summary. The dashboard has to send the map again. So a watchdog reset is
a "needs intervention" recovery, not a silent one.

**Buddy 4:** Will we know it was the watchdog?

**Buddy 2:** The RP2040 keeps the reset reason in a watchdog register.
Proposed: print it at boot and send it once the link is up.

**Buddy 5:** Then a reset leaves the car in a safe state and tells us why.
Modify: fed from tMotion, not from a background task?

**All:** Agreed.

*Team decision: Modify.*

---

## Part 8: state machines, interfaces, interrupts (rows 17, 18, 19, 5 min)

### Row 17: let Start clear a bypass FAULT

**Buddy 5:** The suggestion is that Start clears FAULT so the car carries on
without a reboot. I reject it. FAULT is entered for causes Start cannot
check: motion not acknowledging a stop, a kernel call failing, inconsistent
odometry. The car is off the line, part way through a bypass. A reboot gives
the one known state.

**Buddy 1:** Is a reboot not heavy-handed?

**Buddy 5:** One retry has already happened by then. A leg that overruns
twice its expected time plus 1 second waits 1 second and tries once. FAULT is
what is left when that also failed.

**Buddy 2:** And a line missed twice after the obstacle?

**Buddy 5:** That is not a fault. The obstacle is already passed, so I hand
over to line following and its search.

**Buddy 3:** Does any state machine change?

**Buddy 5:** No. The 4-bit state field holds 13 states. The 4-bit reason
field holds 9 values, so 7 are spare for new reasons. OBS-U47 covers the
faults and the one retry.

**Buddy 3:** So nothing changes in any state machine, and FAULT stays
latched. Reject?

**All:** Agreed.

*Team decision: Reject.*

### Row 18: commands between tasks

**Buddy 2:** I validate every command. A word with reserved bits set, a 0 mm
move, or a turn above 180 degrees is refused with REJECTED. A STOP is never
refused. That is MOT-U35. Commands are discarded while INHIBIT is set.

**Buddy 5:** I treat a refusal as FAULT_REJECTED. I never assume a command
was accepted.

**Buddy 1:** What if the command buffer is full?

**Buddy 5:** It holds four commands. If I cannot queue one within 200 ms,
that is a fault, not a retry for ever.

**Buddy 3:** My interface to motion is different: one request word every
10 ms, and the newest wins. There is no queue to fill.

**Buddy 1:** Neither side assumes a command worked. No Change?

**All:** Agreed.

*Team decision: No Change.*

### Row 19: interrupts and shared data

**Buddy 5:** There is one GPIO bank handler, in `drv_gpio_irq.c`. It calls
the echo handler and then the barcode handler. The handlers only time-stamp
and queue. Deciding whether a reading is valid happens in task context.

**Buddy 3:** Mine queues up to 64 edges and counts the ones it drops.

**Buddy 2:** The shared copies stay as they are: a double buffer for the line
request, a sequence-checked copy for odometry, a copy under a short spin lock
for terrain. No reader waits.

**Buddy 4:** Have we ever had invalid timing?

**Buddy 5:** Yes, on 4 November. After a cold power-up TIMER counted about
23,672 in 1,010 ms, where it should count about 1,010,000. Every echo time
would have been wrong. We fixed it at the source: `usermain` sets the tick
before any task starts.

**Buddy 1:** Does any new fault response run inside an interrupt?

**Buddy 5:** No. Nothing is added to a handler.

**Buddy 2:** So the only concurrency change is the fault record in row 2.
No Change here?

**All:** Agreed.

*Team decision: No Change.*

---

## Part 9: timing and memory (rows 20 and 21, 3 min)

### Row 20: timing

**Buddy 2:** No period, priority or core changes. Every new check is a
compare inside a loop that already runs. Detection times: ultrasonic stale
200 ms, 24 mm of travel. Line request stale 100 ms, 12 mm. Obstacle task
heartbeat 120 ms, 14.4 mm, proposed. Stall 300 ms. IMU fault about 100 ms.
Link loss 500 ms. INHIBIT to brake: limit 20 ms, measured 251 microseconds.

**Buddy 3:** Which is the slowest one that matters for a collision?

**Buddy 2:** Ultrasonic stale: 24 mm of travel against a 150 mm stop
distance.

**Buddy 5:** For a lift the chain is: a 100 ms mean in terrain, my next
10 ms loop, then the brake within 20 ms.

**Buddy 4:** Overturn takes 300 ms by design, so that a bump is not taken
for one.

**Buddy 3:** Every one of those is inside the distance it protects, and no
period moves. No Change?

**All:** Agreed.

*Team decision: No Change.*

### Row 21: memory

**Buddy 1:** Static RAM is 104,300 of 270,336 bytes, 38.6%. We add one event
flag, the fourth of 16, one heartbeat byte, two 2-bit stall counters and
nine fault bits.

**Buddy 2:** Stacks?

**Buddy 1:** Six tasks at 2,048 bytes. No new task, so no new stack.

**Buddy 5:** And the new flag is created in an init function, so kernel
objects are still created only at start-up.

**Buddy 4:** That is bits, not buffers or tasks. No Change?

**All:** Agreed.

*Team decision: No Change.*

---

## Part 10: recovery, tests, requirement wording (rows 22, 23, 1, 6 min)

### Row 22: recovery and safe resumption

**Buddy 3:** The code already has three recovery classes, but they are not
written down. Let each of us state ours.

**Buddy 5:** Clears by itself: an ultrasonic fault, after three new votes.

**Buddy 2:** Clears by itself: a stall, when the wheel counts again. Needs
Start: three stalls on a dead encoder.

**Buddy 1:** Clears by itself: the link coming back while the lift stop
works. Needs Start: the dashboard Stop, and the "no operator stop" hold.

**Buddy 3:** Needs Start: a failed line search.

**Buddy 4:** Needs Start: a lift or overturn.

**Buddy 5:** Needs a reboot: a bypass FAULT and a watchdog reset.

**Buddy 2:** Why is resuming safe?

**Buddy 5:** Every resume goes through the same gate as power-up: Start, and
then the next clear path releases the car. The forward distance is always
checked again first.

**Buddy 2:** Should Start be refused while the cause is still there?

**Buddy 5:** Proposed, where we can see the cause. A car still lying over
60 degrees should not accept Start. A latched bypass FAULT already ignores
it.

**Buddy 1:** So an operator can tell whether to wait, press Start or
power-cycle. Accept?

**All:** Agreed.

*Team decision: Accept.*

### Row 23: verification

**Buddy 4:** Unplug the IMU during a run: fault within about 100 ms, speed
capped at 30, bit set. Then boot on a slope for the NOT_LEVEL limited run.

**Buddy 1:** Switch the hotspot off with the IMU valid: the car continues.
With it invalid: the car holds.

**Buddy 5:** First measure my worst loop gap with Wi-Fi running. Then a test
hook stops my heartbeat, and tMotion must brake within 120 ms.

**Buddy 2:** Hold one wheel three times with its encoder unplugged for the
latched stop. Starve the watchdog and check the car boots with the motors
held.

**Buddy 3:** Host tests for the pure decision logic, as we already do for
motion and terrain.

**Buddy 1:** One honest gap: the system test log has no whole-system fault
run yet.

**Buddy 4:** Each new response has a number in it, so each test can be passed
or failed. Accept?

**All:** Agreed.

*Team decision: Accept.*

### Row 1: requirement wording

**Buddy 4:** With the rows agreed, row 1 follows. We add FR16: every input
the car depends on has a validity state, and on an invalid or missing input
the car takes the listed response within the listed time and reports it.

**Buddy 5:** FR13 gains: if the IMU is in fault or not calibrated, the lift
stop is reported unavailable. FR15 gains the limited run at 30 mm/s.

**Buddy 1:** NFR7 becomes: the car shall always have one working operator
stop, and with the link down it continues only while the lift stop is
available. The other requirements keep their wording.

**Buddy 2:** That matches what we decided in rows 10, 11 and 12. Accept?

**All:** Agreed.

*Team decision: Accept.*

---

## Close (2 min)

**Buddy 1:** Coverage check. Requirements: rows 1, 10, 11, 12. Tasks: 14 to
16. State machines: 6 and 17. Interfaces: 2, 13, 18.

**Buddy 2:** Shared data and interrupts: 2 and 19. Timing: 20. Memory: 21.

**Buddy 5:** Safety and fault handling: 3 to 12, 14 and 16. Hardware: 4, 7
and 16.

**Buddy 4:** Recovery: 17 and 22. Verification: 23. All eleven areas have a
row.

**Buddy 3:** That is 8 No Change, 5 Accept, 5 Modify and 5 Reject. We read
the file through once together and submit.

**All:** Agreed.
