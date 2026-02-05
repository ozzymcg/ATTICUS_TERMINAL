# Terminology

## Core concepts
- **MCL (Monte Carlo Localization):** particle filter representing many pose hypotheses.
- **Particle:** one candidate pose `(x, y, theta)` with weight `w`.
- **Weight normalization:** scaling all particle weights so they sum to 1.
- **Resampling:** drawing a new particle set from weighted particles to focus on likely states.
- **Effective N (`Neff`):** degeneracy metric; low values indicate few particles dominate.
- **KLD adaptive sampling:** adjusts particle count to match estimated posterior complexity.
- **Fused pose:** final pose used by control (typically MCL + EKF blend).
- **Confidence:** concentration of weights; higher means particles agree more strongly.

## Sensor + map terms
- **Likelihood field:** precomputed nearest-obstacle distance grid used for fast sensor scoring.
- **Beam model:** ray-style sensor likelihood model (usually more compute-heavy).
- **Innovation:** residual between predicted and measured sensor value.
- **Gate:** rule that rejects or downweights improbable innovations.
- **Perimeter map mode:** compare sensor rays only against outer field boundary geometry.
- **Objects map mode:** compare only against configured objects/obstacles.
- **Both map mode:** compare against perimeter + objects.

## Motion + fusion terms
- **Odometry delta:** incremental robot-frame motion (`dx`, `dy`, `dtheta`) between updates.
- **Alpha motion model:** noise scales with rotational/translational motion magnitude.
- **Random injection:** periodically reintroduces random particles for kidnapping recovery.
- **Re-localize:** force broad/global or estimate-centered reinitialization.
- **EKF (Extended Kalman Filter):** smooth state estimator over pose + covariance.
- **Covariance (`P`):** uncertainty matrix tracked by EKF.
- **Odom correction alpha:** blend ratio when nudging odom toward fused estimate.
- **Segment band:** temporary path corridor constraint to discourage off-route hypotheses.

## Coordinate words used in docs
- **Field frame:** global map frame (`x` forward, `y` left).
- **Robot frame:** local body frame (`dx` forward, `dy` left).
- **CW heading:** clockwise-positive angle in degrees.
