.. _marine_hydro:

ROSCO Control of Marine Hydrokinetic Turbines (MHKs)
====================================================

.. note::
   The fixed-blade-pitch (FBP) control method, its implementation in ROSCO and the
   ROSCO toolbox, and the verification study documented here were developed by
   **David Stockhouse** (University of Colorado).
   This page is adapted from his presentation *Fixed-Blade-Pitch Control in ROSCO
   for MHK Turbines* [Stockhouse2025]_.


Introduction
---------------

Here, we detail the control of MHK turbines in above-rated flow speeds.
In below-rated flow speeds, torque control is used to operate the turbine at its
maximum tip speed ratio (TSR) and pitch angle, as it is for wind turbines
[Johnson2006]_.

The ROSCO baseline for marine turbines is adapted from the variable-pitch wind
turbine controller [Abbas2022]_.
Many marine turbine designs, however, lack pitch actuators: the cost, complexity,
and reliability burden of a submerged pitch system is often prohibitive.
Assuming a fixed (optimal) blade pitch, we instead redefine the operating schedule
of TSR to find the equilibrium between hydrodynamic and generator torque.
This trades pitch actuation for trade-offs in blade loads, rotor speed, generator
sizing, and environmental impact, which are discussed throughout this page.

Regulating power above rated requires a means of reducing the power coefficient
:math:`C_p`.
For MHK turbines equipped with pitch actuators, those turbines can use control
schemes similar to those used by wind turbines, which reduce the :math:`C_p`
surface by increasing the pitch angle and decreasing TSR (black).
For MHK turbines without pitch actuation, we provide a few control methods for
controlling the power of the turbine using only torque control.
Any such controller is constrained by the requirement that steady-state operating
points be in equilibrium with the inflow torque.
Overspeed control increases the TSR along the fixed pitch line on the :math:`C_p`
surface below (blue circle) by decreasing the generator torque.
Underspeed control decreases the TSR (red circle) by increasing the torque.

.. _cp_surface_annotated:
.. figure:: /images/mhk/02_cp_surface_annotated.png
   :align: center
   :width: 90%

The feature set added to ROSCO consists of an FBP generator torque controller that
is configurable to automatically generate an overspeed or underspeed operating
schedule, a configurable control approach for realizing that operating schedule,
and automatically tuned feedback gains.


Over/Underspeed Reference Setpoints
-----------------------------------

The steady state generator-speed setpoints are determined by the :math:`C_p`
contours intersecting with the fine-pitch line.
Overspeed achieves up to 3x rated speed, which has additional consequences for
blade loads (e.g., cavitation, high thrust).

.. _cp_wg_sched:
.. figure:: /images/mhk/03_cp_wg_sched.png
   :align: center
   :width: 90%

Torque setpoints (:math:`\bar{\tau}`) determined by constant-power relationship :math:`\bar{\tau} = \frac{P_{rated}}{{\bar{\omega}}}`, where :math:`P_{rated}` is the rated power and :math:`\bar{\omega}` is the steady state generator speed.

.. _cp_tg_sched:
.. figure:: /images/mhk/04_cp_tg_sched.png
   :align: center
   :width: 90%

In Region 3, the relationship between torque and speed is nonmonotonic.
Thus, more careful reference control design is required for managing the transition region.
There are examples in the literature for saturation/smoothing during the transition region.

.. _cp_wg_tg_sched:
.. figure:: /images/mhk/05_cp_wg_tg_sched.png
   :align: center
   :width: 90%


Over/Underspeed Dynamics
------------------------

.. .. _cp_Agen_sched:
.. .. figure:: /images/mhk/06_cp_Agen_sched.png
..    :align: center
..    :width: 90%

At each operating point, the sensitivity is computed using the gradients of the
:math:`C_p` surface, in the same manner as the Region-2 controller.
The first-order system decay rate is represented by a single pole on the real axis: more negative means more rapidly stable (positive means unstable).
Underspeed set points are open-loop unstable at high flow speeds.
The sign follows from what each set point is asking the rotor to do.
Underspeed set points sit below :math:`TSR_{opt}`, where hydrodynamic torque still
grows with speed, and generator torque is the only thing holding the speed down; a
small speed perturbation grows rather than decays, so losing torque control lets
the rotor run away.
Overspeed set points are already past :math:`TSR_{opt}`, where hydrodynamic torque
falls off with speed, so the rotor is let out to a self-limiting operating point by
design and any speed perturbation decays on its own.

.. _cp_Agen_sched_annotated:
.. figure:: /images/mhk/07_cp_Agen_sched_annotated.png
   :align: center
   :width: 90%

.. .. _cp_wg_Ta_contour:
.. .. figure:: /images/mhk/08_cp_wg_Ta_contour.png
..    :align: center
..    :width: 90%

The same result can be viewed in the flow-speed/generator-speed plane.
Contours of constant hydrodynamic rotor torque intersect the constant-power
generator torque schedule at the equilibrium operating points; the overspeed and
underspeed branches are the two intersections available at each flow speed.
The local slope of the torque surface across each intersection determines whether
that equilibrium is open-loop stable.

.. TODO(DS): please confirm the description above matches what is plotted in
   09_cp_wg_Ta_contour_annotated.png (slide 11 carried no bullets).

.. _cp_wg_Ta_contour_annotated:
.. figure:: /images/mhk/09_cp_wg_Ta_contour_annotated.png
   :align: center
   :width: 90%


Fixed-Blade-Pitch (FBP) Control
--------------------------------

At each setpoint, the torque controller gains are determined using the same process as the normal torque control in ROSCO.
High magnitude gains are required to compensate for the open-loop instability of the underspeed system (red).

.. _cp_kp_ki_sched:
.. figure:: /images/mhk/10_cp_kp_ki_sched.png
   :align: center
   :width: 90%


Alternate Region 3 Operating Schedules
---------------------------------------

Using the ROSCO toolbox, we enable the user to determine their own operational power curve, besides a constant rated power.
Throughout this section, the *power requirement* is the generator power that the
selected Region-3 power curve demands at a given flow speed, i.e. the
:code:`VS_FBP_P` input evaluated at that speed.
It is an upper bound on what is asked of the machine, not on what is available:
the toolbox clips it to the power the inflow can actually supply at fine pitch,
so the MPPT curve is the ceiling and no request can exceed it.
Relaxing the constant-power constraint in Region 3 raises the peak of the power
requirement, and with it the sustained power the generator must be sized for.
The five power curves studied, with their peak power requirement at cut-out
relative to rated power (equivalently, the generator resizing factor), are:

.. list-table::
   :header-rows: 1
   :widths: auto

   * -  Region 3 power curve
     -  Peak power requirement (x rated)
   * -  Extended Region 2 (MPPT)
     -  8x
   * -  Quadratic extrapolation from tangent
     -  7x
   * -  Linear extrapolation from tangent
     -  4x
   * -  Linear to 2x rated power at cut-out
     -  2x
   * -  Constant power
     -  1x

.. _ext_P:
.. figure:: /images/mhk/11_ext_P.png
   :align: center
   :width: 90%

Each non-MPPT option can follow either the overspeed or the underspeed
equilibrium trajectory.
The alternative power curves result in different speed and torque set points
(dashed lines represent underspeed, solid overspeed).
Speed setpoints always lie within the envelope bounded by the over- and
underspeed constant-power trajectories.
Constant power has the lowest power requirement of the five, so it departs
furthest from :math:`TSR_{opt}` and produces the widest speed excursion in either
direction; the MPPT curve has the highest requirement and does not depart at all,
tracking :math:`\Omega \propto v` straight through rated.
The constant-power schedule is therefore the bounding case for rotor overspeed
and cavitation margin on the overspeed branch, and for generator torque headroom
on the underspeed branch.

.. _ext_wg_sched_annotated:
.. figure:: /images/mhk/16_ext_wg_sched_annotated.png
   :align: center
   :width: 90%

.. @DS: might not need this one, I'll leave the final revision decision to you.  I re-ordered, too
.. .. _ext_wg_tg_sched:
.. .. figure:: /images/mhk/12_ext_wg_tg_sched.png
..    :align: center
..    :width: 90%

.. _ext_tg_sched:
.. figure:: /images/mhk/17_ext_tg_sched.png
   :align: center
   :width: 90%

Contours in the :math:`C_p` surface intersect with the equilibrium points in the
hydrodynamic torque surface, which is what constrains the speed and torque
schedule generated from a generic power curve input.
Plotted in the generator torque/speed plane below, with contours of constant
generator power behind them, the equilibrium trajectories run nearly *along* the
power contours rather than across them.
A small reduction in the power requirement therefore produces a large change in
the equilibrium speed and torque.
Note that the schedule above is plotted against rotor speed :math:`\Omega`, while
the torque/speed plane below is plotted against generator speed
:math:`\Omega_g = N_g \Omega`; for the RM1 the gearbox ratio is
:math:`N_g = 53`, so the two speed axes differ by that factor.
Dropping from the MPPT curve to the quadratic curve, a reduction in peak power
requirement from 8x to 7x rated, moves the overspeed setpoint at cut-out from
roughly 2.5 to 3.5 rad/s rotor speed in the schedule above (about 130 to 185
rad/s at the generator): a 12% reduction in power buys a 40% change in speed.
The two branches also look very different in this plane: the underspeed branches
are nearly vertical (large torque change for small speed change), the overspeed
branches nearly horizontal (large speed change for small torque change).

.. _ext_wg_tg_P_contour:
.. figure:: /images/mhk/13_ext_wg_tg_P_contour.png
   :align: center
   :width: 90%

.. .. _ext_wg_sched:
.. .. figure:: /images/mhk/15_ext_wg_sched.png
..    :align: center
..    :width: 90%

The stability of the FBP set points can be represented by the sensitivity
:math:`\frac{d\tau}{d\Omega}`, following the same over/underspeed argument as
above.
Values less than 0 are open-loop stable; positive values must be stabilized by the
torque controller.

.. _ext_Agen_sched:
.. figure:: /images/mhk/18_ext_Agen_sched.png
   :align: center
   :width: 90%

The power curve selection also impacts the rotor thrust (F).
Underspeed control and lower power generally result in lower thrust.
Conversely, overspeed trajectories approach substantially higher rotor thrust; the
overspeed MPPT trajectory in particular reaches thrust levels that are likely
unacceptable for the structure, and should be checked against the design loads
before being selected.

.. _ext_wg_thrust_contour:
.. figure:: /images/mhk/19_ext_wg_thrust_contour.png
   :align: center
   :width: 90%

.. _ext_wg_thrust_sched:
.. figure:: /images/mhk/20_ext_wg_thrust_sched.png
   :align: center
   :width: 90%


Toolbox Implementation
-----------------------

The ROSCO toolbox works by determining the speed and torque set points required to operate at a TSR and Cp for the desired power across flow speeds.

.. _fbp_flow_chart:
.. figure:: /images/mhk/14_fbp_flow_chart.png
   :align: center
   :width: 90%

The following inputs to the ROSCO tuning yaml, under :code:`controller_params`,
will generate DISCON inputs to ROSCO.

.. list-table::
   :header-rows: 1
   :widths: auto

   * -  Parameter
     -  Description
   * -  VS_FBP
     -  FBP Control Mode:

        - 0 = variable pitch (disabled)
        - 1 = constant power overspeed (nonlinear)
        - 2 = WSE-lookup reference tracking
        - 3 = torque-lookup reference tracking
   * -  VS_FBP_speed_mode
     -  Over/underspeed mode:

        - 0 = underspeed
        - 1 = overspeed
   * -  VS_FBP_power_mode
     -  Normalized or exact power curve values:

        - 0 = relative to rated
        - 1 = exact
   * -  VS_FBP_U
     -  Flow speed setpoints for power curve lookup table [m/s]
   * -  VS_FBP_P
     -  Power curve lookup table, interpreted per :code:`VS_FBP_power_mode`

The following constraints apply when FBP control is enabled:

* :code:`PC_ControlMode`, :code:`VS_ConstPower`, and :code:`PRC_Mode` must all be
  0 whenever :code:`VS_FBP > 0`: blade pitch is fixed to fine pitch, and neither
  constant-power torque control nor power reference control may run concurrently.
  ROSCO aborts on any of these at runtime, and the toolbox raises an exception
  rather than write a configuration that cannot run.
* :code:`VS_FBP = 1` is the fixed control law
  :math:`\tau = \min(P_{rated}/\Omega_g,\ K\Omega_g^2)`, which is inherently
  constant-power and inherently overspeed. ROSCO ignores
  :code:`VS_FBP_speed_mode`, :code:`VS_FBP_power_mode`, and :code:`VS_FBP_P` in
  this mode, so the toolbox overrides them to the constant-power overspeed
  schedule and warns if they were set otherwise. The generated operating
  schedule is still used, both to seed the initial generator torque and to tune
  the torque gains, so it must match the law ROSCO will actually run.
* For :code:`VS_FBP = 3` (torque-lookup reference tracking), the generator torque
  schedule must be strictly increasing, since ROSCO inverts it to look up the
  speed reference. The toolbox checks the computed schedule and raises an
  exception otherwise. A nondecreasing power curve on the *underspeed* branch
  (:code:`VS_FBP_speed_mode = 0`) is sufficient to guarantee this, because
  generator speed falls with flow speed there, so :math:`\tau = P/\Omega_g` rises
  on both counts. Overspeed generally is not sufficient: generator speed rises
  with flow speed, so the torque falls unless the power curve rises faster than
  the speed, which in practice only holds near the MPPT curve.

The Region-2 torque control mode should be chosen to match the Region-3 FBP mode,
so that the two controllers hand off consistently through the transition region:

.. list-table::
   :header-rows: 1
   :widths: auto

   * -  :code:`VS_FBP`
     -  Recommended :code:`VS_ControlMode`
   * -  1 (constant power overspeed, nonlinear)
     -  1 (:math:`k\Omega^2`)
   * -  2 (WSE-lookup reference tracking)
     -  2 (WSE TSR-tracking)
   * -  3 (torque-lookup reference tracking)
     -  4 (torque TSR-tracking)

Note that the ROSCO input schema (:ref:`rt_tuning_yaml`) contains the latest input definitions.

A worked example of all six configurations is provided in
:code:`Examples/31_fixed_pitch_mhk.py`, which tunes against the RM1 marine turbine
tuning case :code:`Examples/Tune_Cases/RM1_MHK_FBP.yaml`.


ROSCO Implementation
-----------------------

The following DISCON parameters are generated using the ROSCO toolbox, or can be determined directly in the DISCON.IN file.

.. list-table::
   :header-rows: 1
   :widths: auto

   * -  Parameter
     -  Description
   * -  VS_FBP
     -  FBP Control Mode:

        - 0 = variable pitch
        - 1 = constant power overspeed (nonlinear)
        - 2 = WSE-lookup reference tracking
        - 3 = torque-lookup reference tracking
   * -  VS_FBP_n
     -  Number of values in operating schedule lookup table
   * -  VS_FBP_U
     -  Flow speed operating points in lookup table
   * -  VS_FBP_Omega
     -  Generator speed operating points in lookup table
   * -  VS_FBP_Tau
     -  Generator torque operating points in lookup table (must be monotonic for :code:`VS_FBP = 3`)

Note that the ROSCO input schema (:ref:`rt_tuning_yaml`) contains the latest input definitions (under :code:`controller_params`, :code:`DISCON`).


Simulation Verification
-----------------------

A handful of example controller case studies have been developed using the RM1
marine turbine [Neary2014]_ to showcase the implemented features of FBP control.
These configurations are

* Example 1: Overspeed, constant power
* Example 2: Underspeed, torque-based reference tracking
* Example 3: Underspeed, WSE-based reference tracking

For each example, a power curve is defined and input to the ROSCO toolbox to
generate operating schedules and auto-tune the gains used by the torque
controller.
The operating schedules for generator power, speed, and torque for each example
test case are shown in the following figure.

.. _cases_P_wg_tg_sched:
.. figure:: /images/mhk/21_cases_P_wg_tg_sched.png
   :align: center
   :width: 90%

Each example controller is then simulated with the RM1 marine turbine model
using OpenFAST in both steady and turbulent inflow. The steady-state performance
of each controller is compared to the operating schedules generated by the ROSCO
toolbox. The turbulent inflow uses the intensity shown in the following figure.

.. _turb_intensity:
.. figure:: /images/mhk/25_turb_intensity.png
   :align: center
   :width: 90%

Tidal flow design load cases differ from their wind counterparts in ways that
matter for controller verification. Turbulence characteristics are not those
typically encountered in atmospheric flow, the mean probability distribution is
more closely isolated by environmental condition, and turbulence intensity varies
heavily with site conditions, with the better tidal sites generally lower in
turbulence intensity than wind sites [Milne2013]_. The turbulence intensity and
wave statistics used here were varied to mimic buoy data collected in the Orkney
Islands, Scotland. Design requirements for tidal current converters are given in
[IEC62600-2]_.

.. TODO(DS): please supply the citation or dataset identifier for the Orkney buoy
   data so it can be referenced properly.


Example 1
^^^^^^^^^

The first example test case uses the naturally stable nonlinear feedback control
law. This controller is confined to operating in the constant power, overspeed
configuration. The explicit (non-reference-tracking) control law is analogous to
the :math:`k\Omega^2` control law sometimes used in Region 2 for wind and marine
turbines [Johnson2006]_, and is best paired with a :math:`k\Omega^2` Region-2
controller (:code:`VS_ControlMode = 1`).
It is easy to design and requires no tuning, but is rigid and inflexible, and its
overspeed operating points carry high tip speed and blade thrust.

.. _case1_P_wg_tg_ss:
.. figure:: /images/mhk/22_case1_P_wg_tg_ss.png
   :align: center
   :width: 90%

This controller has the best power tracking in Region 3, but it only allows constant power.
The power-focused feedback approach accommodates offsets in equilibrium speed and torque made by inaccuracies in the simplified tuning model.

.. _case1_P_wg_tg_turb:
.. figure:: /images/mhk/26_case1_P_wg_tg_turb.png
   :align: center
   :width: 90%


Example 2
^^^^^^^^^

The second example test case uses underspeed operation with a torque-based
reference and a semi-arbitrary power curve:

* Region-2 mode: 		TSR-tracking with torque-based reference (:code:`VS_ControlMode = 4`)
* Region-3 FBP mode: 	reference tracking with torque lookup (:code:`VS_FBP = 3`)
* The power curve may be arbitrarily specified, but should be a nondecreasing function so that the torque schedule is monotonically increasing

This configuration is highly general, but requires an aggressive torque controller
to stabilize the open-loop-unstable underspeed setpoints, and therefore requires a
high maximum torque signal (see :code:`max_torque_factor` in the tuning yaml).

.. _case2_P_wg_tg_ss:
.. figure:: /images/mhk/23_case2_P_wg_tg_ss.png
   :align: center
   :width: 90%

* Linearly increasing power in Region 3, up to 2x rated
* Power curve must be set so that torque schedule is monotonic
* Decent power tracking
* May have some misalignment with flow speed setpoint
* Accommodates some offsets in torque and speed
* Should be combined with reference-tracking controller in Region 2

.. _case2_P_wg_tg_turb:
.. figure:: /images/mhk/27_case2_P_wg_tg_turb.png
   :align: center
   :width: 90%


Example 3
^^^^^^^^^

The third example test case uses underspeed operation with a wind speed estimator
(WSE) reference [Ortega2013]_ and a fully arbitrary power curve:

* Region-2 mode: 		TSR-tracking with WSE-based reference (:code:`VS_ControlMode = 2`)
* Region-3 FBP mode: 	reference tracking with WSE lookup (:code:`VS_FBP = 2`)
* The power curve can be completely arbitrarily specified

This is the most general configuration and the easiest to combine with a Region-2
controller or with alternate references, but it is less robust to errors in the
tuning model than the torque lookup, and shares the torque lookup's need for an
aggressive controller and high torque headroom.

.. _case3_P_wg_tg_ss:
.. figure:: /images/mhk/24_case3_P_wg_tg_ss.png
   :align: center
   :width: 90%

* Smoothly increasing power curve in Region 3
* Would allow arbitrarily increasing or decreasing
* Best gen speed tracking
* May offset equilibrium torque leading to power curve error
* Should be combined with reference-tracking controller in Region 2

.. _case3_P_wg_tg_turb:
.. figure:: /images/mhk/28_case3_P_wg_tg_turb.png
   :align: center
   :width: 90%


Recommendations
-----------------------

FBP control is well suited to marine turbines without blade pitch actuators.
In certain applications, the ability to follow a generic power curve with a
limited actuation capability is more advantageous than using variable-blade-pitch
(VBP) control.
VBP control allows constant-power operation in Region 3 matched with constant
speed and torque for a flat operating schedule. Pitch-actuated turbines also
experience smaller blade loads in Region 3. The FBP approach satisfies
applications in which the cost and complexity of the actuators themselves are
prohibitive.

Generic user input allows flexibility for variety of applications.

FBP controller implementation in ROSCO with auto-tuning and automatic generation of operating schedule to follow power curve.

Because the Region-2 and Region-3 controllers utilize the same actuator, the
transition region is markedly different than what is required for a VBP
Region-3 controller.


Future Work
-----------------------

* Enhanced multi-constraint configuration, including constraining Region-3 rotor
  thrust to a constant level rather than only constraining power.
* Explicit power reference tracking.


References
-----------------------

.. [Stockhouse2025] Stockhouse, D. *Fixed-Blade-Pitch Control in ROSCO for MHK
   Turbines.* National Renewable Energy Laboratory presentation, 2025.
   (Source material for this page.)

.. [Abbas2022] Abbas, N. J., Zalkind, D. S., Pao, L., and Wright, A. *A reference
   open-source controller for fixed and floating offshore wind turbines.*
   Wind Energy Science, 7(1), 53–73, 2022. https://doi.org/10.5194/wes-7-53-2022

.. [Johnson2006] Johnson, K. E., Pao, L. Y., Balas, M. J., and Fingersh, L. J.
   *Control of variable-speed wind turbines: standard and adaptive techniques for
   maximizing energy capture.* IEEE Control Systems Magazine, 26(3), 70–81, 2006.
   https://doi.org/10.1109/MCS.2006.1636311

.. [Ortega2013] Ortega, R., Mancilla-David, F., and Jaramillo, F. *A globally
   convergent wind speed estimator for wind turbine systems.* International
   Journal of Adaptive Control and Signal Processing, 27(5), 413–425, 2013.
   https://doi.org/10.1002/acs.2319

.. [Neary2014] Neary, V. S., Previsic, M., Jepsen, R. A., Lawson, M. J., Yu, Y.-H.,
   Copping, A. E., Fontaine, A. A., Hallett, K. C., and Murray, D. K. *Methodology
   for Design and Economic Analysis of Marine Energy Conversion (MEC)
   Technologies.* SAND2014-9040, Sandia National Laboratories, 2014.
   (Source of the RM1 reference marine turbine model.)

.. [Milne2013] Milne, I. A., Sharma, R. N., Flay, R. G. J., and Bickerton, S.
   *Characteristics of the turbulence in the flow at a tidal stream power site.*
   Philosophical Transactions of the Royal Society A, 371:20120196, 2013.
   https://doi.org/10.1098/rsta.2012.0196

.. [IEC62600-2] IEC TS 62600-2:2019, *Marine energy – Wave, tidal and other water
   current converters – Part 2: Marine energy systems – Design requirements.*
   International Electrotechnical Commission, 2019.
