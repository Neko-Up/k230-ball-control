# Small-Angle Ball Control Design

## Goal

Replace the current error-to-step-frequency behavior with a small-angle rod
position controller. The ball controller must react quickly without driving the
rod through unnecessarily large angles.

## Control Structure

The vision pipeline continues to provide ball position and velocity in
centimeters. A position/velocity PD controller converts those measurements into
a target rod angle. A second, bounded position-tracking stage drives the D36A
stepper toward that target angle.

```text
ball position error + ball velocity
                 |
                 v
          target rod angle
                 |
          clamp to +/-5.0 deg
                 |
                 v
 estimated rod angle error
                 |
                 v
       STEP frequency and DIR
```

## Limits and Initial Parameters

- Rod angle limit: +/-5.0 degrees for fast edge recovery.
- Ball-position deadband: +/-0.15 cm.
- Step frequency range: 150-400 Hz.
- Direction changes must decelerate to zero before reversing.
- Inside the ball-position deadband, the target rod angle returns toward zero.
- Existing pulse conversion remains 8.8889 pulses per rod degree unless the
  mechanical transmission ratio changes.

The PD gains are configuration constants and must be tunable without changing
the control algorithm. Initial gains should favor damping and small motion over
fast large-angle correction.

## Safety

- The motor remains disabled until the user levels the rod and taps the zero
  button.
- Invalid or stale vision data immediately stops STEP output and disables EN.
- The software watchdog remains active and must use a valid, reusable CanMV
  Timer lifecycle.
- Estimated rod angle is always clamped to +/-5.0 degrees.
- Reaching an angle boundary prevents further motion into that boundary but
  still permits motion back toward zero.

## Display and UART

LCD and UART continue to publish the same real-time fields. The displayed `A`
field represents estimated rod angle and must remain within +/-5.0 degrees. No
additional filtering may delay the vision measurements sent to the controller.

## Verification

Automated tests must cover:

- Target angle is clamped to +/-5.0 degrees.
- A large ball error cannot command more than +/-5.0 degrees.
- Deadband commands the rod back toward zero.
- Direction reversal first reduces output to zero.
- Invalid vision disables the driver.
- Existing LCD, UART, touch-zero, and Timer lifecycle tests remain green.

On-board acceptance:

1. Level the rod and tap zero.
2. Move the ball 1-5 cm from target.
3. Confirm the rod never exceeds +/-5.0 degrees.
4. Confirm the rod returns near zero as the ball enters the deadband.
5. Confirm no LCD blackout or runtime exception occurs.
