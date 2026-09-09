<html><head></head><body><h1>Motor CAN bring-up</h1>
<h2>Status</h2>

<h2>1. CAN protocol summary</h2>
<h3>Bus parameters</h3>

Item | Value
-- | --
Bitrate | 1 Mbps (motor default, register 35)
Frame format | Standard 11-bit IDs
Termination | 120 Ω at each end (DIP switch 4 on the motor enables built-in 120 Ω)
Default motor CAN ID (ESC_ID, reg 8) | 0x01
Default master/feedback ID (MST_ID, reg 7) | 0x00


<h3>MIT mode</h3>
<p>Full impedance control. The motor computes:
<code>τ = kp · (p_des − p) + kd · (v_des − v) + t_ff</code></p>
<p>Useful patterns:</p>
<ul>
<li><code>kp &gt; 0, kd &gt; 0</code> — position holding with stiffness/damping</li>
<li><code>kp = 0, kd &gt; 0, v_des = X</code> — constant velocity (similar to Velocity mode but more configurable)</li>
<li><code>kp = 0, kd = 0, t_ff = X</code> — pure torque control</li>
<li><strong>Never set <code>kd = 0</code> while <code>kp &gt; 0</code></strong> — causes oscillation/instability (manual warning).</li>
</ul>
<p>Encoding: position 16-bit, velocity/kp/kd/t_ff each 12-bit, packed into 8 bytes.</p>
<h3>Position-Velocity mode</h3>
<p>Cascaded P-PI control. Send target position; <code>v_des</code> is the speed cap during the move. Smooth motion, slower response than MIT.</p>
<h3>Velocity mode</h3>
<p>Constant-speed PI control. Send target speed in rad/s as raw float. Simplest mode; what we used for first bring-up. Damping factor (register 31, <code>Deta</code>) should be 2.0–10.0 (recommended 4.0).</p>
<h3>Force-Position hybrid mode</h3>
<p>Position target + velocity cap + per-unit current cap. Not tested yet.</p>
<hr>
<h2>3. ESP32 implementation notes</h2>
<h3>Hardware</h3>
<ul>
<li><strong>Board</strong>: ESP32-S3 (any dev module). Native USB → set "USB CDC On Boot = Enabled" in the IDE, otherwise serial output is invisible.</li>
<li><strong>CAN transceiver</strong>: required between the ESP32 and the motor. ESP32 GPIOs cannot drive a differential CAN bus directly. Tested with a standard 3.3 V transceiver module.</li>
<li><strong>Pinout used</strong>: <code>CAN_TX = GPIO5</code>, <code>CAN_RX = GPIO4</code>. Any free GPIOs work; the ESP32 TWAI peripheral is matrix-routable.</li>
</ul>
<h3>Software</h3>
<ul>
<li><strong>Library</strong>: <a href="https://github.com/handmade0octopus/ESP32-TWAI-CAN"><code>handmade0octopus/ESP32-TWAI-CAN</code></a>. Wraps the ESP-IDF TWAI driver. Works on Arduino-ESP32 cores 2.x and 3.x.</li>
<li><strong>Avoided</strong>: <code>sandeepmistry/arduino-CAN</code> and <code>miwagner/ESP32-Arduino-CAN</code>. Both reference <code>esp_intr.h</code> which was removed from ESP-IDF; both are unmaintained.</li>
<li><strong>Bus state</strong>: read via <code>twai_get_status_info()</code> from <code>driver/twai.h</code>. Useful counters: <code>state</code>, <code>tx_error_counter</code>, <code>rx_error_counter</code>, <code>bus_error_count</code>. <code>tx_err = 128 + state = BUS_OFF</code> at startup means no node is acknowledging — almost always missing transceiver, missing termination, or wrong baud rate.</li>
</ul>
<h3>What's working in the test sketch</h3>
<ul>
<li>Mode switching via register 10 write</li>
<li>Enable / disable / zero</li>
<li>Velocity, Position-Velocity, and MIT control frames</li>
<li>Feedback frame decode (position, velocity, torque, MOS temp, rotor temp, error code)</li>
<li>Register read/write via broadcast <code>0x7FF</code></li>
<li>Line-based UART command interface for manual testing</li>
</ul>
<hr>
<h2>4. Next tasks</h2>
<h3>Task 1 — Define the abstract motor interface</h3>
<p>Create an abstract C++ class <code>MotorBase</code> (working name) that defines the operations any CAN motor in our system must support. Concrete drivers inherit from it.</p>

<p>Sub-tasks:</p>
<ul>
<li>Decide ownership of the CAN bus — does the motor own the driver, or is a shared <code>CanBus</code> injected? Recommend the latter so multiple motors share one bus.</li>
<li>Decide error-handling style — return bool, throw, set error flag? Recommend bool + last-error getter (no exceptions on Arduino).</li>
</ul>
<h3>Task 2 — Implement <code>DamiaoJ10010L</code> driver</h3>
<p>First concrete subclass. Port the logic from the existing test sketch:</p>
<ul>
<li>1 Mbps init</li>
<li>Mode register write (reg 10)</li>
<li>Enable/disable/zero special frames</li>
<li>Velocity / Position-Velocity / MIT control frame builders</li>
<li>Feedback frame decoder (8-byte format, fixed-point unpacking)</li>
<li>Configurable <code>P_MAX</code> / <code>V_MAX</code> / <code>T_MAX</code> (constructor args; must match the values stored on the motor)</li>
</ul>
<p>Sub-tasks:</p>
<ul>
<li>Read the ranges from the motor at startup (registers 21/22/23) instead of hardcoding.</li>
<li>Auto-detect master ID (read register 7) so feedback decoding works regardless of configuration.</li>
<li>Add a <code>storeToFlash()</code> method (sends <code>0xAA</code>) for persisting changes.</li>
</ul>
<h3>Task 3 — Multi-motor bus support</h3>
<ul>
<li>Verify the abstract interface handles N motors on one bus cleanly.</li>
<li>Test with 2× Damiao on the same bus, different <code>ESC_ID</code>s.</li>
<li>Confirm feedback dispatch by ID works (each motor's feedback goes to its own object).</li>
</ul>

[CAN_motor_bringup.txt](https://github.com/user-attachments/files/27089190/CAN_motor_bringup.txt)