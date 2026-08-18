"""
Cart-yaw extraction from the tracked orientation quaternion
=============================================================

Measured from a real print's logged quaternions (``pass5.csv``, relative-
rotation axis-angle decomposition against the page normal from the user's
own calibration): cart yaw about the page normal spans **75.6 deg** over a
normal freehand pass, while tilt (pitch/roll) is small by comparison
(median 2.7 deg, max 7.8 deg) -- so correcting yaw alone recovers nearly all
of the error, and pitch/roll correction is a deliberate, measured
non-goal (see the module docstrings of ``coverage.py`` / ``tracking.py``
for where the resulting error actually shows up: a 62.36mm sensor->nozzle
lever arm rotating with the cart, up to ~76mm of position error at the
measured yaw span, and the 15.1mm nozzle-0-to-nozzle-151 span itself
sweeping across columns as it tilts, ~14.6mm at 75 deg).

Kept as its own module rather than folded into ``calibration.py``: this is
quaternion/rotation-matrix algebra used at PRINT time (every sample, via
``tracking.PageMapper`` and ``controller._print_freehand_pass``), not just
during the one-off page-edge-tracing calibration step that ``calibration.py``
is about -- and it is independently testable (see ``tests/test_rotation.py``)
without dragging in ``PageCalibration``/``fit_axis``/Gram-Schmidt at all.

``yaw_about_normal`` used to compute this by converting the relative
rotation to a matrix and reading off its axis-angle "rotation vector" (see
the removed ``_rotation_vector`` helper, kept only in git history). That
method is exact for a *pure* rotation about the page normal only up to
about 135 degrees: driving the operator's own real calibration
(``e_col``/``e_row``/boresight from their ``page_calibration.json``) with a
synthetic pure rotation about that calibration's own page normal measured
0/45/90/135 degrees back exactly, **934.2** degrees (garbage) at 180 --
the antisymmetric-part-over-``sin(angle)`` division the old
``_rotation_vector`` used blows up as ``angle -> pi`` -- and -135/-90
(sign-flipped, not just wrong) at 225/270: a 3x3 rotation matrix, unlike a
quaternion, has no memory of "which way around" a rotation past 180 degrees
went, so its axis-angle log-map is inherently confined to a signed
magnitude in ``[0, 180]``. On real hardware the operator sees exactly this
shape of failure well before the clean 180 boundary too -- a jump from -109
to +109 degrees around a real 180-degree turn -- because the *matrix*
reconstruction is already losing precision as the rotation angle
approaches the singularity, not only exactly at it.

``yaw_about_normal`` and ``cart_rotation_angles`` now use a swing-twist
decomposition of the relative rotation *quaternion* instead (see their
docstrings): quaternions double-cover ``SO(3)`` and never lose the "which
way around" information a plain rotation matrix does, so there is no
analogous blow-up or early sign-flip anywhere a real cart yaw lives.
"""

from __future__ import annotations

import math

import numpy as np


def quat_to_matrix(quat) -> np.ndarray:
    """
    Unit quaternion ``(qx, qy, qz, qw)`` (this project's component order --
    see ``tracking.AmfitrackTracker._extract_pose``) -> the 3x3 rotation
    matrix ``R`` such that ``R @ v_body`` expresses a body-frame vector in
    world coordinates.

    Normalises defensively: a raw sensor quaternion is not guaranteed to be
    exactly unit-norm, and the standard formula below silently returns a
    non-rotation (scaled) matrix if fed one that isn't.
    """
    q = np.asarray(quat, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("quat_to_matrix: zero-norm quaternion")
    x, y, z, w = q / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),       2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),       2.0 * (y * z + x * w),       1.0 - 2.0 * (x * x + y * y)],
    ])


def _quat_multiply(a, b) -> "tuple[float, float, float, float]":
    """
    Hamilton product ``a * b`` of two quaternions in this project's
    ``(qx, qy, qz, qw)`` component order: the rotation ``b`` applied FIRST,
    then ``a`` applied on top of it -- i.e. this is the quaternion
    equivalent of the matrix product ``R(a) @ R(b)``
    (``quat_to_matrix(_quat_multiply(a, b)) == quat_to_matrix(a) @
    quat_to_matrix(b)`` for unit quaternions).

    ``yaw_about_normal``/``cart_rotation_angles`` need this to build the
    relative rotation ``quat * conj(boresight_quat)`` directly as a
    quaternion. A tempting shortcut would be to keep computing the relative
    rotation as a MATRIX (``quat_to_matrix(quat) @
    quat_to_matrix(boresight_quat).T``, the old code) and convert that back
    to a quaternion -- but that round trip has its own well-known
    near-180-degree instability (picking between four standard formulas by
    whichever diagonal entry of the matrix is largest, each with its own
    division that gets ill-conditioned near its own singular case), which is
    exactly the kind of bug this module exists to eliminate. Composing the
    two input quaternions directly, as done here, never goes through a
    matrix at all, so that instability has no way back in.
    """
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_conjugate(q) -> "tuple[float, float, float, float]":
    """Conjugate of ``(qx, qy, qz, qw)`` -- for a unit quaternion, exactly
    its inverse rotation (the quaternion equivalent of ``R.T`` for a
    rotation matrix)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def _normalize_quat(quat) -> "tuple[float, float, float, float]":
    """
    Unit-normalise a quaternion ``(qx, qy, qz, qw)`` defensively, as a plain
    ``(x, y, z, w)`` tuple ready for ``_quat_multiply``/``_quat_conjugate``.

    A raw sensor quaternion, or a calibration's saved ``boresight_quat``, is
    not guaranteed to be exactly unit-norm -- the operator's own real
    boresight measures 1.00002 -- and the swing-twist formula below silently
    gives a slightly wrong angle if fed one that isn't (mirrors
    ``quat_to_matrix``'s own defensive normalisation, kept as a separate
    small helper here rather than reused because that function returns a
    matrix, not the ``(x, y, z, w)`` tuple this quaternion algebra needs).
    """
    q = np.asarray(quat, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("_normalize_quat: zero-norm quaternion")
    x, y, z, w = q / norm
    return (float(x), float(y), float(z), float(w))


def twist_about_axis(quat, axis) -> float:
    """
    Signed ABSOLUTE twist (radians, wrapped to ``(-pi, +pi]``) of ``quat``
    about the fixed world axis ``axis`` -- a swing-twist decomposition of the
    orientation itself, with NO reference/boresight pose subtracted.

    Supplied by the hardware owner as their own known-good z-rotation
    readout (``amfitrack_live_pose.py``'s ``quaternion_twist_angle_deg``,
    called with axis ``(0, 0, 1)``) and adopted verbatim for the simple page
    frame -- see ``tracking.PageMapper``. Kept a faithful port rather than
    re-derived: same normalisation of the input quaternion, same
    ``2 * atan2(dot(v, axis), w)``, same wrap to +-180 degrees.

    How this differs from ``yaw_about_normal``, and why the simple frame
    wants THIS one:

      * **No boresight.** ``yaw_about_normal`` reports rotation relative to
        a captured reference pose; this reports the cart's absolute twist
        about the axis. The simple frame's whole point is working without a
        traced calibration, and a captured reference has repeatedly been the
        weak link in practice (blind first-sample auto-capture picking up
        whatever pose the cart happened to be in; the rig's own saved
        boresight measuring ~110 deg away from flat). An absolute reading
        has no such failure mode: the same physical orientation always
        gives the same number, run to run.
      * **Wrapped to +-180.** ``yaw_about_normal`` deliberately spans
        ``(-360, +360]`` (see its docstring); this wraps, matching the
        operator's script exactly.
      * **Double-cover safe.** ``q`` and ``-q`` are the same physical
        orientation, and unlike ``yaw_about_normal``'s wide range this
        formula's wrap makes both give the same answer.

    ``axis`` need not be unit length (it is normalised here). A zero-length
    axis returns 0.0 rather than raising -- the operator's own version does
    the same, and a degenerate axis has no meaningful twist to report.
    """
    x, y, z, w = _normalize_quat(quat)
    axis = np.asarray(axis, dtype=float)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-12:
        return 0.0
    ax, ay, az = axis / axis_norm

    dot_v_axis = x * ax + y * ay + z * az
    # Normalising (w, dot) before atan2 is redundant for the angle itself
    # (atan2 is scale-invariant) but is kept to mirror the source script
    # line for line; it also gives the near-zero guard below something
    # meaningful to test.
    twist_norm = math.hypot(w, dot_v_axis)
    if twist_norm < 1e-12:
        return 0.0

    angle = 2.0 * math.atan2(dot_v_axis / twist_norm, w / twist_norm)
    # Wrap to (-pi, +pi]: 2*atan2 spans (-2pi, +2pi] on its own.
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_about_normal(quat, boresight_quat, e_col, e_row) -> float:
    """
    Signed yaw (radians) of the cart about the page normal, relative to the
    boresight pose -- the orientation captured with the nozzle bar aligned
    along the traced row edge (see ``calibration.PageCalibration.
    boresight_quat`` / ``calibration.calibrate_page``'s ``boresight_quat``
    parameter).

    Method: SWING-TWIST decomposition of the relative rotation, computed
    directly as a quaternion (``quat_rel = quat * conj(boresight_quat)``,
    via ``_quat_multiply``/``_quat_conjugate`` -- deliberately never routed
    through a rotation MATRIX, see ``_quat_multiply``'s docstring for why).
    With ``v, w`` the vector/scalar parts of ``quat_rel`` and ``n_hat`` the
    unit page normal, the "twist" of ``quat_rel`` about ``n_hat`` (the
    rotation about ``n_hat`` alone, with any other rotation --  "swing" --
    factored out) is::

        twist_rad = 2 * atan2(dot(v, n_hat), w)

    This replaces the module's old method, which built the same relative
    rotation as a MATRIX and read off the axis-angle component along
    ``n_hat`` from its "rotation vector" (axis * angle) log-map -- see the
    module docstring for the measured 934.2-degree blow-up at 180 degrees
    and the -135/-90-degree sign flips past it that method produced, and
    the real ~109-degree early jump the operator saw on hardware. The
    swing-twist formula above has no equivalent singularity anywhere: it
    never divides by anything that vanishes as the twist approaches 180
    degrees (unlike the old rotation-vector's division by ``sin(angle)`` as
    ``angle -> pi``), because it reads the twist off the quaternion's own
    components directly rather than reconstructing an axis from a nearly-
    degenerate matrix.

    It is also, unlike the old method, EXACT regardless of how much swing
    (tilt about an axis other than ``n_hat``) rides along with the twist --
    not just an approximation that happens to be good when swing is small.
    Algebraically: for ``quat_rel = Twist(n_hat, theta) * Swing`` (twist
    composed as the outer/second-applied factor, i.e. the twist is a
    subsequent WORLD/page-frame rotation about ``n_hat`` on top of whatever
    swing already existed -- exactly the physical situation here, since
    page yaw is a page-frame quantity applied on top of whatever incidental
    cart tilt exists), ``dot(v, n_hat) = w_swing * sin(theta/2)`` and
    ``w = w_swing * cos(theta/2)`` (the swing's own scalar part ``w_swing``
    cancels out of the ratio ``atan2`` computes), so the formula recovers
    ``theta`` exactly no matter how large ``Swing`` is. ``tests/
    test_rotation.py`` pins this directly: a case with a large FIXED tilt
    superimposed on an injected yaw recovers that yaw exactly, where the old
    rotation-vector method visibly drifts.

    Range, and the one property this method gives up to get it: the return
    value spans ``(-360, +360]`` degrees, recovering the injected twist
    continuously and exactly right through 180 (see
    ``tests/test_rotation.py``'s 0/45/.../315-degree cases) -- deliberately
    NOT clamped to ``(-180, +180]``, because a clamp would only relocate the
    operator's reported jump back to 180 degrees instead of removing it.

    The price is that this is NOT invariant to the quaternion double cover.
    ``q`` and ``-q`` are the same physical orientation, and the old
    matrix-based method was immune by construction (``R(q) == R(-q)``); this
    one reads the quaternion's components directly, so it is not. MEASURED,
    against the operator's own real calibration: feeding ``-q`` instead of
    ``q`` shifts the returned yaw by exactly 360 degrees, at EVERY angle
    tested (0/30/75/90/135/179/180/225/270/315), not merely near a full
    turn. Same for a sign-flipped ``boresight_quat``. So IF the tracker ever
    emitted a sign-flipped quaternion mid-stream, the displayed yaw would
    jump 360 degrees with the cart standing still.

    Accepted deliberately, on this evidence:

      * The print correction cannot be affected at all. ``tracking.
        PageMapper.project`` and ``coverage.CoverageEngine`` only ever
        consume ``sin``/``cos`` of this value, both exactly 360-degree
        periodic -- verified numerically over a full sweep, not just
        asserted: 0 of 52 sampled angles showed any ``sin``/``cos``
        difference between the two signs. ``cart_rotation_angles``'s
        roll/pitch are likewise unaffected (they are read off the SWING
        quaternion, after the twist is divided out).
      * Only the displayed yaw number could move, and only on a real sign
        flip. The operator's reported symptom was a jump reproducibly at
        REAL 180 degrees -- the old method's actual singularity -- never at
        random moments, which is what a sign-flipping tracker stream would
        have produced instead.

    If a 360-degree jump at a standstill is ever observed on hardware, that
    is the diagnosis, and the fix is one line: canonicalise ``quat_rel`` to
    ``qw >= 0`` before the ``atan2`` below. That restores double-cover
    invariance and costs exactly the wide range -- yaw would then wrap at
    +-180 again, so do it only if the flip is actually seen, not pre-emptively.

    This is deliberately NOT "project some fixed cart axis onto the page
    plane and measure the angle between its boresight and current
    projections" -- that naive approach was tried first during the
    pass5.csv analysis and gave INCONSISTENT answers (143 deg vs 76 deg,
    same data) depending on which body axis was probed, because projecting
    a tilted axis onto a plane distorts angles whenever any pitch/roll is
    present alongside the yaw -- exactly the case here (see the module
    docstring: tilt is small but non-zero, median 2.7 deg). The swing-twist
    decomposition instead operates on the rotation itself, not on any
    arbitrarily chosen probe vector, so it gives one answer regardless of
    how much tilt rides along with the yaw. Do not reach for the naive
    projection version even though it looks simpler -- it was already tried
    and found to be the wrong tool for this data.

    Sign convention: matches ``tracking.PageMapper``'s offset-rotation
    formula directly (``u += col*cos(yaw) - row*sin(yaw)``,
    ``v += col*sin(yaw) + row*cos(yaw)``) with no extra negation needed --
    pinned by ``tests/test_rotation.py``'s 90-degree case together with
    ``tests/test_page_mapper.py``'s matching 90-degree offset case.
    """
    e_col = np.asarray(e_col, dtype=float)
    e_row = np.asarray(e_row, dtype=float)
    n = np.cross(e_col, e_row)
    n_norm = float(np.linalg.norm(n))
    if n_norm < 1e-9:
        raise ValueError("yaw_about_normal: e_col and e_row are parallel "
                         "(degenerate page frame)")
    n = n / n_norm

    q = _normalize_quat(quat)
    q_bore = _normalize_quat(boresight_quat)
    qx, qy, qz, qw = _quat_multiply(q, _quat_conjugate(q_bore))
    v = np.array([qx, qy, qz])
    return 2.0 * math.atan2(float(np.dot(v, n)), qw)


def cart_rotation_angles(quat, boresight_quat, e_col, e_row) -> "tuple[float, float, float]":
    """
    Full aircraft-style (roll, pitch, yaw) breakdown of the cart's rotation
    relative to the boresight pose, in radians -- the same swing-twist
    decomposition ``yaw_about_normal`` uses, read out along all three
    page-frame axes instead of one.

    DIAGNOSTIC ONLY. Only the yaw component feeds an actual print-time
    correction (``tracking.PageMapper.project`` / ``coverage.CoverageEngine``
    rotate the sensor->nozzle offset and per-nozzle placement by yaw alone).
    Roll and pitch, returned here purely for live monitoring (see
    ``diagnostics.monitor_position`` / the web UI's readout tiles), are NOT
    fed back into any position/offset math anywhere. That is a deliberate,
    measured non-goal, not an oversight: this module's own docstring records
    that cart tilt (pitch/roll) measured from a real print (``pass5.csv``)
    is small -- median 2.7 deg, max 7.8 deg -- against 75.6 deg of yaw over
    the same pass, so yaw-only correction already recovers nearly all of the
    positioning error, and adding roll/pitch correction to the print-time
    path would add real complexity (which body-frame axis actually moves the
    nozzle bar under combined tilt, at what lever arm) for a small remaining
    error. Should that ever change (e.g. a rig with substantially more tilt),
    this function already computes the numbers a future correction would
    need -- it is kept in lock-step with ``yaw_about_normal`` specifically so
    that door stays open cheaply.

    Method -- one coherent swing-twist decomposition, not three independent
    single-axis reads:

      1. ``yaw`` is computed by calling ``yaw_about_normal`` directly with
         the same arguments (not re-derived independently), so the two can
         never disagree -- pinned by ``tests/test_rotation.py``'s exact
         (``==``) cross-check.
      2. The TWIST quaternion for that ``yaw`` (a pure rotation about
         ``n_hat``) is removed from the relative rotation
         (``quat_rel * conj(twist)``), leaving the SWING quaternion: by
         construction, a rotation with zero component along ``n_hat``, i.e.
         entirely within the ``e_col``/``e_row`` plane.
      3. ``roll``/``pitch`` are the swing quaternion's own twist-about-axis
         reads along ``e_col``/``e_row`` (the same ``2 * atan2(...)``
         formula ``yaw_about_normal`` uses, just applied to the swing
         quaternion's ``v``/``w`` and a different axis).

    Reading roll/pitch off the SWING quaternion specifically (rather than
    applying the same per-axis formula directly to the full, un-factored
    ``quat_rel``, the way an earlier draft of this function did) matters
    once yaw is large: ``quat_rel``'s own scalar part ``w`` reflects the
    FULL relative rotation angle (yaw *and* tilt combined), so reading roll/
    pitch directly off it can pick up a spurious 180-degree flip once yaw
    alone pushes past 180 and flips the sign of ``w`` -- even though the
    ACTUAL tilt is small. Removing the twist first fixes ``w`` back near
    +1 for the (typically small) swing that is left, so roll/pitch stay
    well-behaved regardless of how large yaw gets -- yaw and tilt no longer
    cross-talk through a shared, twist-polluted ``w``.

      * ``roll``  -- swing's twist-about-``e_col`` (the column/travel axis,
        the direction the cart rolls along a row): cart tipping side-to-side
        while moving along a row, like an aircraft rolling about its
        fuselage axis.
      * ``pitch`` -- swing's twist-about-``e_row`` (the row axis, along the
        nozzle bar): cart nodding forward/backward, like an aircraft
        pitching about its wing axis.

    ``e_col``/``e_row`` are defensively re-normalised by their own norm
    before use as projection axes (mirrors ``yaw_about_normal``'s defensive
    normalisation of ``n``, even though ``PageCalibration.e_col``/``e_row``
    are already Gram-Schmidt unit vectors -- see ``calibration.py``). Raises
    the same ``ValueError`` as ``yaw_about_normal`` if ``e_col``/``e_row``
    are parallel/degenerate (zero-length normal): a degenerate page frame
    makes roll/pitch/yaw equally meaningless, not just yaw.
    """
    e_col = np.asarray(e_col, dtype=float)
    e_row = np.asarray(e_row, dtype=float)
    n = np.cross(e_col, e_row)
    n_norm = float(np.linalg.norm(n))
    if n_norm < 1e-9:
        raise ValueError("cart_rotation_angles: e_col and e_row are parallel "
                         "(degenerate page frame)")
    n = n / n_norm

    col_norm = float(np.linalg.norm(e_col))
    row_norm = float(np.linalg.norm(e_row))
    e_col_unit = e_col / col_norm
    e_row_unit = e_row / row_norm

    yaw = yaw_about_normal(quat, boresight_quat, e_col, e_row)

    q = _normalize_quat(quat)
    q_bore = _normalize_quat(boresight_quat)
    quat_rel = _quat_multiply(q, _quat_conjugate(q_bore))

    half_yaw = 0.5 * yaw
    s, c = math.sin(half_yaw), math.cos(half_yaw)
    twist = (n[0] * s, n[1] * s, n[2] * s, c)
    sx, sy, sz, sw = _quat_multiply(quat_rel, _quat_conjugate(twist))
    v_swing = np.array([sx, sy, sz])

    roll = 2.0 * math.atan2(float(np.dot(v_swing, e_col_unit)), sw)
    pitch = 2.0 * math.atan2(float(np.dot(v_swing, e_row_unit)), sw)
    return roll, pitch, yaw
