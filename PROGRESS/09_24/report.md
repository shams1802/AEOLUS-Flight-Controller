# 4.2 Stabilization

You give a **roll position command** (say **10° right**).

**Outer loop** compares **desired angle vs actual** → produces a **roll speed command**.  
**Inner loop** compares **commanded roll speed vs gyro measurement** → adjusts **motor PWMs**.

**Motors respond → drone rolls.**  
**Accelerometer + gyro** keep updating **real position and speed**.  
**Loop repeats hundreds of times per second.**

- **Outer loop** = “Where should I be?” (**angle control**)  
- **Inner loop** = “How fast should I move to get there?” (**speed control**)  

**Together, they make sure the drone tilts to the exact angle you want, smoothly and stably.**

---

## Time Constant

**What is `timeConstant`?**  
It’s a parameter that tells the filter **how quickly it should trust the accelerometer vs gyro**.

- **Small `timeConstant` → trust accelerometer more**
  - Faster correction
  - Noisier

- **Large `timeConstant` → trust gyro more**
  - Slower correction
  - Smoother, but more drift
