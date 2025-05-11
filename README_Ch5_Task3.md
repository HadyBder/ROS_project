Enhance the responsiveness of the HeadController Lifecycle node so that pan–tilt corrections occur with minimal latency,
yielding smoother and more reactive head tracking in simulation and on real hardware.

What We Changed:

Faster Control Loop: Cut the periodic update rate in half—from 100 ms to 50 ms—to double the command frequency and reduce motion latency.
