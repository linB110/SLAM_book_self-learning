## Assignment: Coordinate Transformation Between Two Camera Frames

Consider two robots, **Little Carrot 1** and **Little Carrot 2**, both positioned in the world coordinate system. The pose of Little Carrot 1 is given by:

- Quaternion: **q₁ = [0.35, 0.2, 0.3, 0.1]** (the first element is the real part)
- Translation: **t₁ = [0.3, 0.1, 0.1]ᵀ**

> **Note:** Please normalize **q₁** before performing any calculations.  
> The given pose represents the transformation **T<sub>cw</sub>**, i.e., from the **world** coordinate system to the **camera** coordinate system.

The pose of Little Carrot 2 is:

- Quaternion: **q₂ = [−0.5, 0.4, −0.1, 0.2]**
- Translation: **t₂ = [−0.1, 0.5, 0.3]ᵀ**

This also represents **T<sub>cw</sub>** for Little Carrot 2.

Now, suppose Little Carrot 1 observes a point with coordinates:

- **p = [0.5, 0, 0.2]ᵀ**

in its own camera coordinate frame. Compute the coordinates of this point **as observed from Little Carrot 2’s camera frame**.

Please write a program to implement this transformation.

