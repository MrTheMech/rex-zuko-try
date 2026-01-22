# ROS2 Foxy → Humble Upgrade Assessment

## Executive Summary

**Verdict: MODERATE WORK - Doable in 2-4 hours, but requires careful testing**

**Risk Level: MEDIUM** - Most changes are straightforward, but Ubuntu version change (20.04→22.04) requires full reinstall.

---

## Current State Analysis

### What You Have
- **ROS2 Version:** Foxy Fitzroy (EOL - End of Life)
- **Ubuntu Version:** 20.04 LTS
- **Python Version:** Python 3.8
- **Codebase:** Clean ROS2 Python codebase, no ROS1 dependencies
- **Packages:** 8 ROS2 packages (7 Python, 1 CMake)

### What You Need
- **ROS2 Version:** Humble Hawksbill (Current LTS)
- **Ubuntu Version:** 22.04 LTS (required)
- **Python Version:** Python 3.10 (comes with Ubuntu 22.04)
- **Codebase:** Same structure, minor API updates needed

---

## Required Changes Breakdown

### 1. **Ubuntu OS Upgrade** ⚠️ **BIGGEST CHANGE**
- **Impact:** HIGH
- **Work:** Full OS reinstall required (can't upgrade in-place from 20.04→22.04)
- **Time:** 30-60 minutes
- **Risk:** Medium - Need to backup configurations, reinstall all software

**Why:** Humble requires Ubuntu 22.04. You cannot upgrade Ubuntu 20.04→22.04 directly on Raspberry Pi without issues.

### 2. **Launch File Updates** ✅ **EASY**
- **Impact:** LOW
- **Work:** Minor - Update `get_package_share_directory` → `get_package_share_path`
- **Files Affected:** 5 launch files
- **Time:** 15 minutes
- **Risk:** Low - Straightforward find/replace

**Current Issue:**
- Mixed usage: Some files use `get_package_share_directory()` (old)
- Some use `get_package_share_path()` (new, preferred in Humble)
- Need to standardize on `get_package_share_path()` and use Path objects

**Files to Update:**
- `quad_main/launch/quad_all.launch.py`
- `quad_main/launch/quad_main.launch.py`
- `quad_main/launch/quad_live.launch.py`
- `quad_motors/launch/quad_motors.launch.py`
- `quad_udrf/launch/display.launch.py` (already uses new API, but package name wrong)

### 3. **package.xml Updates** ✅ **EASY**
- **Impact:** LOW
- **Work:** Minor - Fix deprecated `<name>` tag
- **Files Affected:** 8 package.xml files
- **Time:** 10 minutes
- **Risk:** Low - XML formatting fix

**Current Issue:**
- Using `<name>` tag which is deprecated
- Should remove `<name>` tag (package name comes from filename)
- Or use proper format (but removing is cleaner)

### 4. **Python API Compatibility** ✅ **LIKELY OK**
- **Impact:** LOW
- **Work:** Minimal - Most APIs are compatible
- **Time:** 30 minutes testing
- **Risk:** Low - Standard rclpy APIs used

**What to Check:**
- `rclpy.logging._root_logger.log()` - May need update to `get_logger()`
- `rclpy.ok()` - Should be fine
- `Node` class usage - Should be fine
- Parameter handling - Should be fine

**Potential Issues:**
- Logging API might have changed (need to verify)
- Parameter value access might need `.value` attribute check

### 5. **CMakeLists.txt Updates** ✅ **LIKELY OK**
- **Impact:** LOW
- **Work:** Minimal - Mostly compatible
- **Time:** 10 minutes
- **Risk:** Low - Standard ament_cmake usage

**Current State:**
- Uses standard `ament_cmake` - Should work as-is
- C++ standard set to C++14 - Humble supports this
- Standard ROS2 CMake patterns used

### 6. **Dependencies** ✅ **SHOULD BE FINE**
- **Impact:** LOW
- **Work:** Verify availability in Humble
- **Time:** 15 minutes
- **Risk:** Low - Standard ROS2 packages

**Packages Used:**
- `sensor_msgs` - ✅ Available
- `std_msgs` - ✅ Available
- `rosidl_default_generators` - ✅ Available
- `ament_cmake` - ✅ Available
- `ament_python` - ✅ Available

**External Dependencies:**
- `numpy` - ✅ Available via pip
- `yaml` (pyyaml) - ✅ Available via pip
- `smbus2` - ✅ Available via pip
- `pybullet` - ✅ Available via pip (for simulation)

### 7. **Documentation Updates** ✅ **EASY**
- **Impact:** LOW
- **Work:** Update version references
- **Time:** 15 minutes
- **Risk:** None

**Files to Update:**
- `INSTALL_RPI.md` - Change Foxy→Humble, Ubuntu 20.04→22.04
- `PROJECT_OVERVIEW.md` - Update ROS2 version
- `ROBOT_SETUP_GUIDE.md` - Update version references
- `dev-notes/ros2-setup.txt` - Update installation instructions

---

## Detailed Change List

### Critical Changes (Must Do)

1. **Ubuntu 22.04 Installation**
   - Fresh install Ubuntu 22.04 Server on Raspberry Pi
   - Reinstall ROS2 Humble
   - Reinstall all Python dependencies

2. **Launch Files** (5 files)
   ```python
   # OLD (Foxy)
   from ament_index_python.packages import get_package_share_directory
   path = os.path.join(get_package_share_directory('package'), 'config', 'file.yaml')
   
   # NEW (Humble)
   from ament_index_python.packages import get_package_share_path
   path = get_package_share_path('package') / 'config' / 'file.yaml'
   ```

3. **package.xml** (8 files)
   ```xml
   <!-- OLD (deprecated) -->
   <package format="3">
     <name>quad_main</name>
   
   <!-- NEW (correct) -->
   <package format="3">
     <!-- name comes from filename: quad_main/package.xml -->
   ```

4. **Logging API** (if needed)
   ```python
   # OLD (might work but deprecated)
   rclpy.logging._root_logger.log("message", LoggingSeverity.INFO)
   
   # NEW (preferred)
   self.get_logger().info("message")
   ```

### Nice-to-Have Changes (Optional)

1. **Parameter Access**
   - Check if `.value` attribute needed for parameters
   - Current code: `self.get_parameter('name').value` - should work

2. **Launch File Improvements**
   - Use Path objects consistently
   - Better error handling

---

## Compatibility Analysis

### ✅ What Will Work Without Changes
- Core ROS2 Python APIs (`rclpy`, `Node`, `Publisher`, `Subscriber`)
- Message types (`sensor_msgs/Joy`, custom messages)
- Executors (`SingleThreadedExecutor`)
- Parameter system (mostly compatible)
- Build system (`colcon`, `ament_python`, `ament_cmake`)

### ⚠️ What Needs Updates
- Launch file path handling (minor)
- package.xml formatting (minor)
- Logging API (might need update)
- Ubuntu version (major - requires reinstall)

### ❌ What Won't Work
- Ubuntu 20.04 with Humble (incompatible)
- Old launch file path functions (deprecated)

---

## Migration Strategy

### Option 1: Clean Install (Recommended)
1. Backup current configuration files
2. Fresh install Ubuntu 22.04 on Raspberry Pi
3. Install ROS2 Humble
4. Clone repository
5. Apply code changes
6. Build and test

**Pros:** Clean, no legacy issues
**Cons:** Need to reconfigure everything

### Option 2: Gradual Migration
1. Make code changes first (backward compatible where possible)
2. Test on development machine with Humble
3. Then migrate Raspberry Pi

**Pros:** Test before hardware migration
**Cons:** More steps

---

## Estimated Time Breakdown

| Task | Time | Risk |
|------|------|------|
| Ubuntu 22.04 + ROS2 Humble install | 60 min | Medium |
| Launch file updates | 15 min | Low |
| package.xml fixes | 10 min | Low |
| Python API updates | 30 min | Low |
| Dependency verification | 15 min | Low |
| Build and test | 30 min | Medium |
| Documentation updates | 15 min | None |
| **TOTAL** | **~3 hours** | **Medium** |

---

## Risk Assessment

### Low Risk Items ✅
- Launch file path updates
- package.xml formatting
- CMakeLists.txt (should work as-is)
- Dependencies (standard ROS2 packages)

### Medium Risk Items ⚠️
- Ubuntu 22.04 installation (requires full reinstall)
- Python API changes (logging might need update)
- Testing on actual hardware (need to verify everything works)

### High Risk Items ❌
- None identified - codebase is clean and uses standard APIs

---

## Testing Checklist

After upgrade, verify:

- [ ] ROS2 Humble installed correctly
- [ ] All packages build without errors (`colcon build`)
- [ ] Launch files work (`ros2 launch quad_main quad_all.launch.py`)
- [ ] Gamepad node starts (`ros2 run quad_gamepad quad_gamepad`)
- [ ] Motor node starts (`ros2 run quad_motors quad_motors`)
- [ ] I2C communication works (PCA9685 detected)
- [ ] Servo calibration works
- [ ] Topics publish/subscribe correctly
- [ ] Robot moves correctly with controller

---

## Recommendation

**YES, upgrade is worth it and manageable.**

### Why Upgrade?
1. **Foxy is EOL** - No security updates or bug fixes
2. **Humble is LTS** - Supported until 2027
3. **Better tooling** - Improved debugging and development tools
4. **Future-proof** - Easier to upgrade to newer versions later

### Why It's Manageable?
1. **Clean codebase** - Uses standard ROS2 APIs
2. **Mostly Python** - Easier to update than C++
3. **No ROS1 dependencies** - Avoids complex migration
4. **Well-structured** - Clear package organization

### What to Watch Out For?
1. **Ubuntu reinstall** - Biggest hurdle, but one-time
2. **Testing** - Need to verify hardware still works
3. **Configuration** - Need to reconfigure I2C, Bluetooth, etc.

---

## Conclusion

**Upgrade is DOABLE and RECOMMENDED.**

The work is moderate but straightforward. Most changes are:
- Configuration file updates (easy)
- Launch file path handling (easy)
- OS reinstall (one-time, but necessary)

The codebase is well-structured and uses standard ROS2 APIs, making migration relatively smooth.

**Estimated effort: 2-4 hours** (including testing)

**Risk: Medium** (mostly due to OS reinstall requirement)

**Recommendation: Proceed with upgrade** - Benefits outweigh the effort, especially since Foxy is EOL.

---

## Next Steps (When Ready)

1. Backup current setup
2. Create upgrade branch
3. Update launch files
4. Update package.xml files
5. Test on development machine first (if available)
6. Update Raspberry Pi OS
7. Install ROS2 Humble
8. Build and test
9. Update documentation

**Awaiting your approval to proceed with the upgrade.**
