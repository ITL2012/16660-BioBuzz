# Obstacle Avoidance System — Technical Documentation

## Overview
The BioBuzz autonomy stack now includes a comprehensive obstacle avoidance system that handles:
1. **Static field obstacles** — perimeter walls, hive posts
2. **Dynamic opponent robots** — tracked via WorldModel with intent HMM
3. **Path planning integration** — both StrategicPlanner (greedy) and MCTSPlanner use obstacle costs

---

## Architecture

### Core Components

#### 1. `FieldObstacle` Data Class (`autonomy.kt:174+`)
```kotlin
data class FieldObstacle(
    val id: Int,
    val xCm: Double,           // Center X position (field cm)
    val yCm: Double,           // Center Y position (field cm)
    val radiusCm: Double,      // Obstacle radius + robot clearance
    val label: String,         // Debug label
    val isHivePost: Boolean,   // Special handling for hive posts
) {
    // Line-circle intersection test for path blocking
    fun blocksPath(fromX, fromY, toX, toY, robotRadiusCm = 25.0): Boolean
    
    // Distance from point to obstacle edge
    fun distanceTo(xCm, yCm): Double
}
```

#### 2. `WorldModel` Obstacle Management (`autonomy.kt:700+`)
```kotlin
// Static obstacles initialized once
private val fieldObstacles = mutableListOf<FieldObstacle>()

// Runtime hive post updates from AprilTag
fun updateHivePosts(hiveX: Double, hiveY: Double, hiveHeading: Double)

// Path blocking query (used by planners)
fun isPathBlocked(fromX, fromY, toX, toY, robotRadiusCm = 25.0): Boolean

// Simple waypoint offset for detours
fun findClearPath(fromX, fromY, toX, toY, robotRadiusCm = 25.0): Pair<Double, Double>?
```

#### 3. Static Obstacle Initialization (`autonomy.kt:683+`)
```kotlin
private fun initFieldObstacles() {
    // Perimeter walls (standard FTC field: 360cm x 240cm)
    fieldObstacles.addAll(listOf(
        FieldObstacle(1, -180, 0, 5, "Left Wall"),
        FieldObstacle(2, 180, 0, 5, "Right Wall"),
        FieldObstacle(3, 0, 120, 5, "Top Wall"),
        FieldObstacle(4, 0, -120, 5, "Bottom Wall"),
    ))
}
```

#### 4. Dynamic Hive Posts (`autonomy.kt:702+`)
```kotlin
fun updateHivePosts(hiveX, hiveY, hiveHeading) {
    // Remove old posts
    fieldObstacles.removeAll { it.isHivePost }
    
    // 4 posts at hive corners (±30cm from center)
    val corners = listOf(30 to 30, -30 to 30, -30 to -30, 30 to -30)
    for ((i, (cx, cy)) in corners.withIndex()) {
        // Rotate by hive heading
        val rx = cx * cos(hiveHeading) - cy * sin(hiveHeading)
        val ry = cx * sin(hiveHeading) + cy * cos(hiveHeading)
        fieldObstacles.add(FieldObstacle(
            100 + i, hiveX + rx, hiveY + ry, 5.0, "Hive Post ${i+1}", isHivePost = true
        ))
    }
}
```

---

## How Path Blocking Works

### Line-Circle Intersection Algorithm
For a path from `(fromX, fromY)` to `(toX, toY)` and obstacle at `(ox, oy)` with radius `R`:

```
dx = toX - fromX
dy = toY - fromY
fx = fromX - ox
fy = fromY - oy

a = dx² + dy²
b = 2 * (fx*dx + fy*dy)
c = fx² + fy² - R²

discriminant = b² - 4ac

if discriminant >= 0:
    t1 = (-b - sqrt(discriminant)) / (2a)
    t2 = (-b + sqrt(discriminant)) / (2a)
    blocked = (0 <= t1 <= 1) OR (0 <= t2 <= 1)
```

This detects if the robot's **center path** intersects the obstacle's **inflated radius** (obstacle radius + robot radius = 5cm + 25cm = 30cm for hive posts).

### Robot Radius
- **Robot radius**: 25cm (configured as `robotRadiusCm` parameter)
- **Obstacle radius**: 5cm for hive posts, 5cm for walls
- **Effective clearance**: 30cm from robot center to obstacle edge

---

## Planner Integration

### StrategicPlanner (Greedy) — `autonomy.kt:930+`
```kotlin
for (b in world.balls()) {
    val dist = b.distanceTo(robotXcm, robotYcm)
    val baseT = dist / ROBOT_SPEED_CM_PER_SEC
    val obstaclePenalty = if (world.isPathBlocked(robotXcm, robotYcm, b.fieldXcm, b.fieldYcm)) 2.0 else 0.0
    val t = baseT + PLANNER_COLLECT_HANDLE_SEC + obstaclePenalty
    // ... lower density score for blocked paths
}
```

**Penalty values:**
- Ball collection blocked: +2.0 seconds
- Flower cluster blocked: +3.0 seconds  
- Hive path blocked: +3.0 seconds
- Individual flower fallback: +2.0 seconds

### MCTSPlanner — `autonomy.kt:1275+`
```kotlin
private fun generateActions(world, ballsHeld, timeRemaining, robotX, robotY) {
    val hiveObstaclePenalty = if (hive != null && world.isPathBlocked(robotX, robotY, hive.first, hive.second)) 3.0 else 0.0
    val scoreTime = baseScoreTime + PLANNER_SCORE_TOTAL_SEC + hiveObstaclePenalty
    
    // Ball collection
    val obstaclePenalty = if (world.isPathBlocked(robotX, robotY, b.fieldXcm, b.fieldYcm)) 2.0 else 0.0
    
    // Flower clusters
    val obstaclePenalty = if (world.isPathBlocked(robotX, robotY, cluster.centerXcm, cluster.centerYcm)) 3.0 else 0.0
}
```

---

## Opponent Robot Avoidance

### Separate System (Dynamic)
Opponent robots are **not** in `fieldObstacles` — they're tracked in `WorldModel.tracks` with:
- Position + velocity (from vision)
- Intent HMM (SCORING/DEFENDING/COLLECTING/TRANSIT/UNKNOWN)
- Intent-based discounting in planner

### Planner Opponent Discount (`autonomy.kt:940+`)
```kotlin
for (o in opponents) {
    if (o.distanceTo(targetX, targetY) < ourDistance) {
        val intentDiscount = when (o.intent) {
            OpponentIntent.DEFENDING -> 0.3  // Guarding target
            OpponentIntent.SCORING -> 0.5    // Passing through
            OpponentIntent.COLLECTING -> 0.4 // Active competition
            OpponentIntent.TRANSIT -> 0.7    // Just passing
            OpponentIntent.UNKNOWN -> PLANNER_OPPONENT_DISCOUNT
        }
        value *= intentDiscount * o.intentConfidence.coerceIn(0.3, 1.0)
    }
}
```

### Active Dodge (TeleOp/SelfPlay) — `BioBuzzTeleOp.kt:337+`
```kotlin
if (DODGE_ENABLED && opponent within DODGE_TRIGGER_DIST_CM) {
    val timeToImpact = distCm / abs(radialVel)
    if (timeToImpact < DODGE_LOOKAHEAD_SEC) {
        strafe += dodgeDir * DODGE_STRAFE_POWER
        turn += dodgeDir * DODGE_TURN_POWER
    }
}
```

---

## Telemetry & Debugging

### Dashboard Visibility (`ShadowPlannerDashboard.kt`)
```kotlin
telemetry.addData("Shadow/Opponents", 
    opponents.map { "${it.label}@${dist}cm: ${it.intent} (${conf}%)" }.joinToString(" | "))
```

### WorldModel Summary (`autonomy.kt:768+`)
```kotlin
fun summary(): String {
    val oppSummary = opponents().map { "${it.label[0]}=${it.intent.name[0]}" }.joinToString(",")
    return "balls:%d flowers:%d opp:%d[%s] hive:%s"
}
```

### BlackBox Logging
```json
{
  "opp": [
    {"id": 5, "x": 120, "y": -30, "conf": 0.8, "vx": 45, "vy": -10, 
     "intent": "COLLECTING", "intentConf": 0.85}
  ]
}
```

---

## Tuning Parameters

### In `globals.kt`
```kotlin
// Obstacle avoidance
const val AVOID_ROBOT_BEARING_DEG = 15.0      // Opponent dodge trigger angle
const val AVOID_ROBOT_STRAFE = 0.15           // Sidestep power
const val DODGE_TRIGGER_DIST_CM = 120.0       // Start dodging distance
const val DODGE_MIN_DIST_CM = 60.0            // Emergency brake distance
const val DODGE_STRAFE_POWER = 0.45           // Lateral dodge power
const val DODGE_TURN_POWER = 0.30             // Turn-away power

// Planner obstacle penalties (seconds added to travel time)
const val PLANNER_OBSTACLE_PENALTY_BALL = 2.0
const val PLANNER_OBSTACLE_PENALTY_FLOWER = 3.0
const val PLANNER_OBSTACLE_PENALTY_HIVE = 3.0
```

### In `autonomy.kt` (FieldObstacle defaults)
```kotlin
val robotRadiusCm = 25.0      // Robot footprint radius
val hivePostRadius = 5.0      // Physical post radius
val wallThickness = 5.0       // Wall collision thickness
```

---

## Testing Obstacle Avoidance

### Unit Tests (JVM)
```bash
./gradlew :TeamCode:test --tests "*WorldModelTest*" --tests "*PhaseClockTest*"
```

### Simulation Harness
A Black Box session is one SQLite file — the state, the tracks, the planner
decisions, the events and the camera frames all live in it.

```kotlin
val harness = SimulationHarness()
    .loadSession(File("blackbox/auto_session.db"))
    .run()
harness.assertPlannerMatchesGolden(File("blackbox/auto_session.db"), "auto_decisions")
```

### Manual Testing Checklist
- [ ] Robot stops before hitting perimeter walls
- [ ] Robot paths around hive posts when approaching hive
- [ ] Planner assigns lower density to targets behind obstacles
- [ ] Opponent dodge triggers in teleop when robot approaches
- [ ] Hive posts appear in telemetry when AprilTag detected
- [ ] `world.isPathBlocked()` returns true for direct wall paths

---

## Visual Debugging

### FTC Dashboard Telemetry
```
Shadow/Opponents: R=Robot@120cm: COLLECTING (85%) | B=Robot@80cm: DEFENDING (90%)
Shadow/TimeBudget: Total: 28.0s | Collect: 12.0s | Fire: 6.5s | Park: 2.5s
```

### Driver Station Telemetry
```
WorldModel: balls:2 flowers:1 opp:2[R=C✓,B=D✓] hive:yes(85%)
FieldObs: [Left Wall, Right Wall, Top Wall, Bottom Wall, Hive Post 1, Hive Post 2, Hive Post 3, Hive Post 4]
```

---

## Future Improvements

1. **RRT* Global Planner** — Replace simple waypoint offset with full sampling-based planner
2. **Cost Map** — Discretized field grid with obstacle costs for smoother paths
3. **Opponent Trajectory Prediction** — Kalman filter on opponent tracks for better dodge timing
4. **Dynamic Obstacle Inflation** — Increase robot radius at high speeds
5. **Multi-Robot Coordination** — Share obstacle maps between alliance partners (if rules allow)

---

*Last updated: $(date)*
*Part of BioBuzz autonomy stack v2.0*