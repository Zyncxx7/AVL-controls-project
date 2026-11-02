# Assumption audit of the nonlinear model

This audits the current `solve.py`, including the original model that it uses
for reference curves. It records choices made while implementing the PDF
and email requests, not additional instructions from the user. The status below records the subsequent corrections as well as remaining limitations.

The PDF supplies aerodynamic data and formulas; it does not specify a complete
loaded aircraft, engine, environment, initial condition and numerical experiment.
I filled some gaps rather than leaving them unresolved. That should have been
made explicit before presenting the results as a completed model comparison.
Descriptions already in the README do not constitute user approval.

## Added physical assumptions and unresolved interpretations

| Choice | What the implementation does | Consequence / missing justification |
| --- | --- | --- |
| Common aircraft loading | Uses mass 1043 kg and inertias 1285, 1825, 2667 kg m2 from the earlier model. | No loading condition establishes these as the correct values for the PDF's C172P. |
| Principal inertia axes | Uses only diagonal inertias; all products of inertia are zero. | Body axes have been treated as principal axes without loading/geometry evidence. |
| Constant loading | Holds mass and inertia fixed; no fuel burn or moving loads. | Inherited rigid-body approximation, not a supplied loading specification. |
| Fixed source reference CG | Uses the XML empty-aircraft reference CG (41, 0, 36.5) in with the PDF aerodynamic reference (43.8, 0, 58.7) in. Adds r cross F in trim and all-axis dynamics. | Corrects the former zero-arm treatment. This is a declared comparison configuration: the retained 1043 kg and inertias do not specify the source empty or loaded aircraft. The default XML pilot/fuel loaded CG is different. |
| Omitted thrust moment | Applies thrust along body x through the CG, without an engine-installation angle or offset moment. | No thrust-line geometry establishes this. |
| Borrowed propulsion | Copies the old 75-inch, 2400-rpm propeller and thrust-coefficient table. The original comments identify C172N/PyFME data. | This is not a propulsion model specified by the PDF for its C172P. |
| Linear throttle scaling | Throttle multiplies fixed-rpm thrust directly. | No engine torque, RPM response, power matching or engine transient is solved. |
| Propeller boundary behavior | Returns zero thrust outside the advance-ratio table. Negative table CT values can yield negative thrust inside it. | These are inherited implementation choices, not a validated operating envelope. |
| Propwash and wake | Disabled at the user's request. Tail/control coefficients use free-stream dynamic pressure. No induced-velocity closure is used. | Approved simplification; original force/moment derivatives remain. |
| Zero flaps | Uses only the clean columns and omits flap force/moment increments and flap-dependent lateral tables. | A clean configuration was selected; the user did not specify it for the requested comparison. The program does not implement every configuration in the PDF. |
| Free-air ground effect | Uses lift/drag ground-effect multipliers equal to one and omits the supplied ground-effect tables. | No height above ground was supplied to justify this selection. |
| Wing incidence interpretation | Does not separately apply the PDF's 1.5-degree wing incidence. | I treated the tables as body-alpha data already incorporating the necessary reference. That interpretation needs confirmation. |
| Force-frame interpretation | Uses an orthogonal wind-to-body rotation and treats the side coefficient as wind-axis side force. | The PDF describes side force in body y in one place and gives inconsistent frame expressions elsewhere. I selected an interpretation instead of resolving it with the user. |
| Extended roll damping | Retains `Clp = -0.484` after stall. | The PDF states that a post-stall sign reversal occurs, but gives no numerical replacement. Retaining attached-flow damping outside its stated regime is a limitation, not an established post-stall model. No positive value was invented. |
| Unmodelled propulsion effects | Adds no separate propeller reaction torque, gyroscopic moment, P-factor or asymmetric slipstream effect. | These mechanisms are not defined by the implemented propulsion inputs. Their absence matters especially for lateral motion and any future spin analysis. |

The propwash-dependent elevator quadratic has been removed. With no propwash,
the moment-balance equation has two linear cases because elevator drag uses
`abs(elevator)`. The solver checks the positive and negative cases explicitly.

## Inherited environment and rigid-body assumptions

| Choice | What the implementation does | Consequence |
| --- | --- | --- |
| Atmosphere | Uses `rho = 1.225*exp(-max(altitude,0)/8500)`. Trim uses 1.225 kg/m3. | This is the earlier exponential atmosphere, not a supplied weather/temperature profile or full standard atmosphere. |
| Below-sea-level density | Holds density at its sea-level value for negative altitude. | This is a clamp, not a physical atmosphere below that datum. |
| No wind | Treats the same body velocity as air-relative velocity for forces and ground-relative velocity for position. | Wind and gust inputs are zero implicitly. |
| Local Earth approximation | Uses flat North-East-Down position coordinates with fixed gravity 9.81 m/s2. | Earth curvature, rotation and variation of gravity are omitted. |
| Rigid aircraft | Uses the six rigid-body equations with no structural flexibility or changing geometry. | This is an inherited 6DOF framework choice. |
| No ground model | Does not stop at ground contact or model landing gear/runway forces. | Initial altitude zero is used as a density/coordinate datum, while free-air aerodynamics are assumed. It is not a runway simulation. |
| Source-data applicability | Applies the supplied coefficient model across the chosen speed range without additional Mach/Reynolds corrections. | No additional speed-dependent aerodynamic data were supplied. The accuracy of that use has not been established. |

## Table and stall-model implementation choices

| Choice | What the implementation does | Consequence |
| --- | --- | --- |
| Linear interpolation | Connects consecutive table entries with straight lines. | The PDF supplies points; I selected interpolation semantics from upstream JSBSim. |
| Endpoint holding | Uses the first/last coefficient when outside a table's argument range. | This adds behavior where the PDF has no tabulated value, rather than stopping and reporting unsupported inputs. |
| Different supported ranges | Lift covers -0.09 to 0.36 rad; drag covers -0.0873 to 0.3491 rad; CYp, CYr and Clr tables cover only 0 to 0.094 rad; sideslip tables cover +/-0.349 rad. | A state can be inside the lift table while already using clamped drag or lateral rate coefficients. |
| Binary hysteresis update | Uses strict `alpha > 0.36` to set state 1, strict `alpha < 0.09` to reset state 0, and otherwise holds state. | The numerical limits are in the PDF; the update/equality semantics were taken from JSBSim source, not stated explicitly by the user or fully spelled out in the PDF. |
| Instant branch change | No flow-separation time constant is integrated at a switch. | This follows the selected binary-state implementation, not a validated dynamic-stall lag model. |
| Initially attached | Starts the response with stall state 0. | No preceding flight history was provided. |
| Independently seeded trim branches | Searches state 0 and state 1 independently in the hysteresis band. | These roots are possible equilibria under assigned memory states, not evidence of an actual acceleration/deceleration trajectory reaching them. |
| Post-peak search range | Searches through the full lift-table range to 0.36 rad, beyond the PDF's 0.28-rad normal critical-lift limit. | I interpreted the latter as an attached-flow/stall-warning threshold, not a hard domain cutoff. |
| Alpha-rate treatment | Solves the alpha-dot aerodynamic feedback algebraically from the same-step accelerations. | This retains the listed alpha-rate derivatives, but differs from implementations that use a previous-step or filtered alpha-dot estimate. It is a numerical interpretation of the supplied formula. |

## Selected trim problem and experiment

| Choice | What was selected | Consequence |
| --- | --- | --- |
| Trim type | Straight, level, symmetric flight with beta, bank and p/q/r zero; pitch equals alpha. | Ranjith asked for trim versus speed, but the exact flight condition was not specified in the latest instructions. |
| Trim altitude/density | Sea-level density. | No trim altitude was provided. |
| Lateral trim controls | Zero aileron and rudder. | A complete asymmetric six-axis trim optimization is not performed. |
| Trim speed range | 40-125 kt, in 1-kt increments. | I selected the range and spacing, extending the earlier model's 55-125 kt / 5-kt sweep. |
| Root selection for response | Chooses the first feasible attached-state root, effectively the lowest-alpha accepted root. | Root selection was not specified; this does not establish stability. |
| Feasibility definition | Labels a trim feasible if required throttle is between 0 and 1. | Elevator travel/rate limits, engine torque/power limits, stall stability and the real aircraft envelope are not checked. |
| Initial response condition | Starts at each model's own 100-kt trim, zero lateral motion and zero angular rates. | Carried forward from the earlier demonstration; not a newly specified experiment for the PDF aircraft. |
| Response input | Additional -1-degree elevator from 2 to 3 seconds, trim throttle held fixed, neutral aileron/rudder. | Also carried forward; not prescribed in the latest email. There is no pilot/controller or actuator lag. |
| Response duration | 15 seconds. | A selected demonstration duration. |
| Initial position and heading | North/east/down and heading zero. | A coordinate/initial-condition choice; no ground location or starting altitude was supplied. |

## Numerical choices, not supplied physical parameters

- Forward Euler integration, with fixed `DT = 0.001` s. I selected this after
  initially using RK4 at 0.01 s. The smaller step improves convergence but
  does not validate the physical model.
- Fixed steps must divide the pulse boundaries and duration exactly.
- A 901-point alpha scan, augmented with table breakpoints, followed by 50
  bisection iterations. This finds bracketed sign-change roots; it does not
  mathematically guarantee every tangent or closely spaced root.
- Endpoint force tolerance `1e-8` N and root deduplication tolerance `1e-7` rad.
- Euler-angle orientation rather than quaternions, inherited from the earlier
  simple model. Stop when `abs(cos(pitch)) < 0.1`, speed is below 1 m/s or
  forward velocity is nonpositive. These are selected applicability guards.
- No automatic rejection or termination solely for leaving aerodynamic table
  bounds. Endpoint holding continues instead.
- No stability classification or eigenvalue calculation for trim roots.

## Assumptions in the original reference curves

The imported original DATCOM/AVL model is not ground truth. In addition to
the shared mass/atmosphere/propeller assumptions, it contains:

- A 0.029 parasite-drag addition to the original AVL drag values.
- Substitute DATCOM pitch/lift rate derivatives where its source table has
  missing values, and control/lateral derivatives from a separate reference.
- A `Cm0 + Cm_alpha(alpha)*alpha` moment construction rather than simply using
  the tabulated total Cm values. This is the earlier model's modeling choice.
- Straight-line interpolation, with extrapolation outside its four alpha
  points in time simulation; its trim search stays inside its source range.

I retained those choices for comparison without independently validating the
baseline against aircraft measurements. Matching it is not physical validation.

## Data that were not invented

The PDF's geometry values S/B/C, clean CL/CD table points, listed clean-force
and moment derivatives, and 0.09/0.36-rad hysteresis limits were transcribed,
not fitted. SI unit conversion is a conversion, not an aircraft assumption.
The post-stall positive roll-damping value was not fabricated; it is missing.

No spin solution has been implemented or claimed, in accordance with the
email's deferral of that work. Tests verify numerical consistency, memory
logic and convergence of this chosen implementation. They do not validate
the unsupported loading, CG, propulsion or environment assumptions above.

## Earlier implementations removed from the current main block

I previously added a prescribed -0.05-to-0.361-rad up/down alpha cycle,
822 steps each way and a 0.02-s sample interval. That was a chosen diagnostic,
not a flight manoeuvre supplied by the user. It has now been removed from
`main()` and its standalone plot/CSV deleted as requested. The binary stall
memory remains in the real time loop and regression checks.

## Status

The current outputs are computations under these choices. They must not be
described as a fully specified or physically validated reproduction of the
PDF aircraft. Remaining loading/propulsion and reference-configuration limitations
should have been identified as missing inputs before those gaps were filled.
