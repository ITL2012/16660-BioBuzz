# External Libraries Evaluation for BioBuzz Autonomy

## Recommended Libraries

### 1. **Pathfinding / Motion Planning**
| Library | Purpose | Pros | Cons |
|---------|---------|------|------|
| **MPC (Model Predictive Control)** | Optimal trajectory generation with constraints | Handles dynamics, obstacles, actuator limits | Complex to tune, heavy computation |
| **OMPL (Open Motion Planning Library)** | Sampling-based planners (RRT*, PRM) | Proven, many algorithms, ROS integration | C++, not Kotlin/JVM native |
| **Dijkstra/A* on Grid** | Simple global planning | Easy to implement, deterministic | Grid resolution tradeoffs |
| **Custom RRT* (pure Kotlin)** | Lightweight, no deps | Full control, JVM compatible | Reinventing wheel |

**Recommendation**: Implement a lightweight **RRT* in pure Kotlin** for global planning with obstacle avoidance. The current `findClearPath()` is a simple local planner; RRT* would handle complex multi-obstacle scenarios.

### 2. **State Estimation / Sensor Fusion**
| Library | Purpose | Pros | Cons |
|---------|---------|------|------|
| **Kalman/Extended Kalman Filter (pure Kotlin)** | Sensor fusion, state estimation | Lightweight, well-understood | Linear assumptions |
| **Unscented Kalman Filter (UKF)** | Nonlinear state estimation | Better for nonlinear dynamics | More complex |
| **Factor Graph (GTSAM)** | SLAM, bundle adjustment | Gold standard for SLAM | Heavy, C++ only |
| **ORB-SLAM3** | Visual SLAM | State-of-art monocular/stereo | Very heavy, not for FTC |

**Recommendation**: Keep current **HybridPoseEstimator** (visual odometry + RR fusion) but add a **lightweight EKF** for IMU + wheel odometry + vision fusion. The current fusion is essentially a complementary filter; EKF would provide covariance estimates for MCTS uncertainty modeling.

### 3. **Machine Learning / Neural Networks**
| Library | Purpose | Pros | Cons |
|---------|---------|------|------|
| **TensorFlow Lite** | On-device inference (current) | Optimized for mobile, delegates | Limited ops, model conversion pain |
| **ONNX Runtime Mobile** | Cross-framework inference | Runs PyTorch/TF/ONNX models | Larger binary |
| **MediaPipe** | Pre-built ML pipelines | Face/hands/pose/object detection | Fixed pipelines, less flexible |
| **Custom Kotlin MLP** (current ShotPredictor) | Tiny fully-connected nets | Zero deps, fast, differentiable | Only for tiny nets |

**Recommendation**: Keep **TFLite for detector** + **pure Kotlin MLP for ShotPredictor**. For future: consider **ONNX Runtime** if PyTorch training pipeline is preferred.

### 4. **Optimization / Control**
| Library | Purpose | Pros | Cons |
|---------|---------|------|------|
| **CVXPY / OSQP** | Convex optimization (MPC) | Fast QP solver | Python/JVM bridge needed |
| **Kotlin Math (custom)** | PID, LQR, simple MPC | Zero deps | Limited to small problems |
| **Apache Commons Math** | Linear algebra, optimization | Mature JVM lib | Heavyweight |

**Recommendation**: Use **custom Kotlin LQR/MPC** for trajectory tracking. The current RR controller is PID-based; LQR would provide optimal gains from system model.

### 5. **Simulation / Testing**
| Library | Purpose | Pros | Cons |
|---------|---------|------|------|
| **Gazebo / Webots** | High-fidelity physics sim | Realistic sensors, dynamics | Heavy, not JVM |
| **Custom JVM Sim** (current) | Fast headless replay | CI-friendly, deterministic | Simplified physics |
| **MuJoCo** | Fast physics, contacts | Research-grade | License, C++ |

**Recommendation**: Expand **current JVM simulation harness** with:
- Simple rigid-body dynamics (differential drive + slip model)
- Sensor noise models (vision, IMU, encoders)
- Opponent behavior scripts for closed-loop testing

### 6. **Logging / Data Analysis**
| Library | Purpose | Pros | Cons |
|---------|---------|------|------|
| **Protocol Buffers + Kotlinx Serialization** (current) | Binary logging | Fast, schema evolution | Requires .proto or @Serializable |
| **Apache Parquet** | Columnar analytics | Great for post-match analysis | JVM only, not real-time |
| **InfluxDB / TimescaleDB** | Time-series storage | Query language, retention | Server required |
| **MCAP** | Robotics bag format | ROS2 standard, fast | New, limited Kotlin support |

**Recommendation**: Current **Protobuf + kotlinx-serialization** is good. Add **MCAP writer** for ROS2 compatibility in post-match analysis.

### 7. **Configuration / Parameter Management**
| Library | Purpose | Pros | Cons |
|---------|---------|------|------|
| **Typesafe Config (HOCON)** | Hierarchical config | Includes, env vars, merge | Java-centric |
| **Kotlinx Serialization + JSON/YAML** | Type-safe config | Kotlin-native, multi-format | No runtime validation |
| **Custom @Config** (current) | Dashboard-tunable params | FTC Dashboard integration | Global mutable state |

**Recommendation**: Migrate to **Kotlinx Serialization + HOCON** for:
- Type-safe config classes
- Environment-specific overrides (comp vs practice)
- Versioned config schema with migration

### 8. **Dashboard / Visualization**
| Library | Purpose | Pros | Cons |
|---------|---------|------|------|
| **FTC Dashboard** (current) | Real-time telemetry | Built for FTC, web-based | Limited chart types |
| **Grafana + Prometheus** | Advanced dashboards | Industry standard | Overkill for match |
| **Custom WebSocket + Canvas** | Custom viz | Full control | Dev effort |

**Recommendation**: Extend **FTC Dashboard** with custom widgets (value density bars, opponent intent pie, obstacle map). The current `ShadowPlannerDashboard` is a good start.

---

## Priority Integration Roadmap

### Phase 1 (Immediate - 1-2 weeks)
1. **Lightweight RRT* in Kotlin** for global path planning with obstacle avoidance
2. **EKF for sensor fusion** (IMU + wheel odometry + vision) with covariance output
3. **MCAP logging** for ROS2-compatible post-match analysis

### Phase 2 (Before Next Event - 1 month)
1. **LQR trajectory tracker** replacing PID in RR
2. **ONNX Runtime** for detector (if PyTorch training desired)
3. **HOCON config system** with versioned schema

### Phase 3 (Long-term)
1. **Full MPC** with obstacle constraints (requires fast QP solver)
2. **MuJoCo-based sim** for hardware-in-the-loop testing
3. **Opponent behavior learning** (imitation learning from match logs)

---

## Library Dependencies to Add (Gradle)

```gradle
dependencies {
    // Pathfinding
    implementation "org.apache.commons:commons-math3:3.6.1"  // for RRT* math utilities
    
    // Sensor Fusion (EKF)
    implementation "org.ejml:ejml-simple:0.41"  // Efficient Java Matrix Library
    
    // Config
    implementation "com.typesafe:config:1.4.3"  // HOCON
    
    // Logging
    implementation "com.foxglove.mcap:mcap-java:0.1.0"  // MCAP support
    
    // ML (if switching detector)
    implementation "org.tensorflow:tensorflow-lite:2.14.0"
    // OR
    implementation "com.microsoft.onnxruntime:onnxruntime-android:1.16.0"
    
    // Optimization
    implementation "org.ojalgo:ojalgo:53.0.0"  // Linear algebra + QP
}
```

---

## Architecture Note

The current codebase has excellent **separation of concerns**:
- Pure Kotlin autonomy core (`autonomy.kt`) - testable on JVM
- Android-specific OpModes (`BioBuzzAutonomous.kt`, etc.)
- Vision subsystem with clean interfaces

**Keep this architecture**. Any new libraries should:
1. Be pure Kotlin/JVM where possible (for testability)
2. Have clear interfaces for Android-specific implementations
3. Not couple the autonomy core to Android SDK