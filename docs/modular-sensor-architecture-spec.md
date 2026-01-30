# Modular Sensor Architecture Specification

**Version:** 1.1
**Date:** 2026-01-30
**Status:** Draft for Review

---

## 1. Purpose

Define requirements for an extensible sensor monitoring system that allows plug-and-play addition of I2C sensors with minimal code changes.

---

## 2. Problem Statement

The current monolithic implementation has these limitations:

1. Adding a sensor requires modifying core display and main loop logic
2. Sensors cannot describe their capabilities to the system
3. Display layout is hardcoded for exactly 3 values + 1 graph
4. No separation between data acquisition and presentation
5. All code lives in a single 340-line file with global state

---

## 3. Goals

| Goal | Description |
|------|-------------|
| **Extensibility** | Add new sensors by creating isolated files + one-line registration |
| **Self-Description** | Sensors declare their values, units, ranges, and display preferences |
| **Flexible Display** | UI adapts to 1-6 values in multiple layout configurations |
| **Separation of Concerns** | Sensor logic, display logic, and orchestration are independent |
| **Maintainability** | Each sensor in its own file, testable in isolation |

---

## 4. Functional Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-01 | System shall support multiple I2C sensors on a shared bus | Must |
| FR-02 | Each sensor shall expose metadata describing its values (name, unit, expected range) | Must |
| FR-03 | Display shall render 1-6 sensor values without code changes | Must |
| FR-04 | Adding a new sensor shall not require modifying core system files | Must |
| FR-05 | Each sensor implementation shall be isolated in its own source file | Must |
| FR-06 | System shall support trend graphing for selected values | Should |
| FR-07 | Display layout shall be configurable between multiple modes | Should |
| FR-08 | Sensors shall report connection status | Should |
| FR-09 | System shall handle sensor errors gracefully (show placeholder, continue operation) | Should |

---

## 5. Non-Functional Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| NFR-01 | Total sensor polling cycle shall complete within 100ms | Must |
| NFR-02 | Display shall update at minimum 2Hz (every 500ms, currently 2s) | Must |
| NFR-03 | RAM usage per sensor shall not exceed 2KB | Should |
| NFR-04 | Adding a sensor requires only: new source file + registration call | Must |
| NFR-05 | No dynamic memory allocation (heap) | Should |

---

## 6. Sensor Value Metadata

Each value exposed by a sensor must provide:

| Field | Purpose |
|-------|---------|
| Identifier | Unique string for programmatic reference |
| Display Name | Human-readable label (short, fits UI) |
| Unit | Measurement unit string for display |
| Expected Range | Min/max values for display scaling |
| Display Color | Color for UI rendering |
| Graph Inclusion | Whether to include in trend graphs |
| Precision | Decimal places to display |

---

## 7. Display Requirements

### 7.1 Supported Layouts

| Mode | Description | Use Case |
|------|-------------|----------|
| Two-Column | 6 values in 2×3 grid | Multiple sensors, values only |
| Values + Graph | 3 values left, trend graph right | Current behavior, extended |
| Compact + Large Graph | Condensed value row, large graph area | Focus on trends |

### 7.2 Display Constraints

- Screen resolution: 160×80 pixels
- Must render any count from 1-6 values
- Graph should support multiple overlaid trend lines
- Values showing "---" or similar when sensor disconnected

---

## 8. System Behavior

### 8.1 Initialization

1. Initialize I2C bus
2. Register sensors (explicit, not auto-detect)
3. Initialize each sensor, track success/failure
4. Initialize display with current sensor count
5. Begin polling loop

### 8.2 Runtime Loop

1. Poll all connected sensors
2. Collect readings with validity flags
3. Update display with current values and history
4. Handle errors by marking readings invalid

### 8.3 Error Handling

| Condition | Behavior |
|-----------|----------|
| Sensor not found at startup | Log warning, exclude from value list |
| Sensor read fails | Mark reading invalid, display placeholder |
| Sensor disconnects mid-operation | Continue with stale data, show indicator |
| All sensors fail | Display error message |

---

## 9. File Organization

```
src/
├── main.cpp              # Orchestration only
├── core/                 # Interfaces and registry
├── display/              # Display management
└── sensors/              # One file pair per sensor type
include/
└── Config.h              # Pin definitions, constants
```

---

## 10. Developer Workflow: Adding a Sensor

1. Create sensor source files in `sensors/` directory
2. Implement required interface methods
3. Define value descriptors with metadata
4. Add registration call in `main.cpp` setup
5. **No other changes required** - display adapts automatically

---

## 11. Constraints

| Constraint | Rationale |
|------------|-----------|
| Explicit sensor registration | Simpler than auto-detect, predictable ordering |
| Maximum 6 displayed values | Screen size limitation (160×80) |
| Maximum 4 sensors | Memory budget, diminishing returns |
| I2C sensors only (for now) | Simplifies interface, covers common sensors |
| No heap allocation | Predictable memory, no fragmentation |

---

## 12. Open Questions

| # | Question | Options |
|---|----------|---------|
| 1 | Auto-detect sensors via I2C scan? | Explicit registration (simpler) vs auto-detect (more magic) |
| 2 | Support non-I2C sensors later? | Extend current interface vs separate abstraction |
| 3 | Runtime layout switching? | Button input vs compile-time selection |
| 4 | Value priority when >6 values? | First-registered wins vs configurable |

---

## 13. Acceptance Criteria

- [ ] BME280 works through new abstraction
- [ ] Mock/test sensor can be added with only new files + registration
- [ ] Display renders 1, 3, and 6 values correctly
- [ ] Disconnecting sensor shows placeholder, system continues
- [ ] Memory usage verified under budget

---

## 14. Related Documents

- **Design Document:** `modular-sensor-architecture-design.md` - Implementation details, interfaces, code

---

*End of Specification*
