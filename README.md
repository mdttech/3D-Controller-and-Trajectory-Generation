# 3D-Controller-and-Trajectory-Generation
MATLAB Programming Exercise 3
# 1 Introduction
In this exercise, I will be extending PD controller to control a quadrotor in 3D. I will then generate the time parameterized trajectories such that the quadrotor will navigate through all of the given waypoints.

# 2 System Model

![1](https://user-images.githubusercontent.com/36922299/59974286-0f0cb680-95c8-11e9-85e1-948d603ad388.png)


# 2.1 Coordinate Systems
The coordinate systems and free body diagram for the quadrotor are shown in Fig. 1. The inertial frame, A, is deﬁned by the axes a1,a2, and a3 around the origin (O). The body frame, B, is attached to the center of mass of the quadrotor with b1 coinciding with the preferred forward direction and b3 perpendicular to the plane of the rotors pointing vertically up during perfect hover (see Fig. 1). These vectors are parallel to the principal axes. The center of mass is C. Rotor 1 is a distance L away along b1, 2 is L away along b2, while 3 and 4 are similarly L away along the negative b1 and b2 respectively. Since bi are principal axes, the inertia matrix referenced to the center of mass along the bi reference triad, I, is a diagonal matrix. This matrix will be provided to you as variable params.I.
# 2.2 Motor Model
Each rotor has an angular speed ωi and produces a vertical force Fi according to
        
        Fi = kF(ωi)^2 . (1) 
        
Experimentation with a ﬁxed rotor at steady-state shows that kF ≈ 6.11×10−8 N rpm2. The rotors also produce a moment according to

     Mi = kM(ωi)^2 . (2)
     
The constant, kM, is determined to be about 1.5×10−9 Nm rpm2 by matching the performance of the simulation to the real system.
# 2.3 Rigid Body Dynamics
We will use Z −X −Y Euler angles to model the rotation of the quadrotor in the world frame. To get from A to B, we ﬁrst rotate about a3 through the the yaw angle, ψ, to get the triad ei. A rotation about the e1 through the roll angle, φ gets us to the triad fi (not shown in the ﬁgure). A third pitch rotation about f2 through θ results in the body-ﬁxed triad bi. You will need the rotation matrix for transforming components of vectors in B to components of vectors in A:

     A[R]B=[cψcθ−sφsψsθ, −cφsψ, cψsθ + cθsφsψ; cθsψ + cψsφsθ, cφcψ, sψsθ−cψcθsφ; −cφsθ, sφ , cφcθ]. (3)
     
We will denote the components of angular velocity of the robot in the body frame by p, q, and r: 

       AωB = pb1 + qb2 + rb3.
 These values are related to the derivatives of the roll, pitch, and yaw angles according to
           
           [p q r]=[ cθ 0 −cφsθ; 0 1 sφ; sθ 0 cφcθ ][ ˙ φ ˙ θ ˙ ψ ]
 # Newton’s Equations of Motion
 Let r denote the position vector of C in A. The forces on the system are gravity, in the −a3 direction, and the forces from each of the rotors, Fi, in the b3 direction. The equations governing the acceleration of the center of mass are

     m¨ r =[0 0 −mg]+ R[0; 0; F1 + F2 + F3 + F4]. (5)
     
We will deﬁne our ﬁrst input u1 to be

    u1 = Σ4 i=1Fi.
    
# Euler’s Equations of Motion
In addition to forces, each rotor produces a moment perpendicular to the plane of rotation of the blade, Mi. Rotors 1 and 3 rotate in the −b3 direction while 2 and 4 rotate in the +b3 direction. Since the moment produced on the quadrotor is opposite to the direction of rotation of the blades, M1 and M3 act in the b3 direction while M2 and M4 act in the −b3 direction. L is the distance from the axis of rotation of the rotors to the center of mass of the quadrotor.
The angular acceleration determined by the Euler equations is

     I[ ˙ p ˙ q ˙ r]=[ L(F2 −F4); L(F3 −F1)M1 ; −M2 + M3 −M4 ]−[p q r]×I[ p q r]. (6)
     
We can rewrite this as: 

      I[ ˙ p ˙ q ˙ r]=[ 0 L 0 −L; −L 0 L 0; γ −γ γ −γ ] [F1 F2 F3 F4 ]− [ p q r]×I[ p q r]. (7) 
where γ = kM/ kF is the relationship between lift and drag given by Equations (1-2). Accordingly, we will deﬁne our second set of inputs to be the vector of moments u2 given by:
 
      u2 =[ 0 L 0 −L; −L 0 L 0 ;γ −γ γ −γ ] [ F1 F2 F3 F4 ]. 
      
 The angular acceleration is determined by Euler’s equation of motion as, 
 
     Ixx ¨ φ = L(F1 −F2) = u2
     
# 3 Robot Controllers
# 3.1 The Nominal State
Our controllers are derived by linearizing the equations of motion and motor models (5 – 2) at an operating point that corresponds to the nominal hover state, r = r0, θ = φ = 0,ψ = ψ0, ˙ r = 0, and ˙ φ = ˙ θ = ˙ ψ = 0, where the roll and pitch angles are small (cφ ≈ 1, cθ ≈ 1,sφ ≈ φ, and sθ ≈ θ). At this state the lift from the propellers is given by: Fi,0 = mg 4 ,
The nominal values for the inputs at hover are u1,0 = mg, u2,0 = 0. Linearizing (5), we get:
¨ r1 = g(∆θ cosψ0 + ∆φ sinψ0) (8) ¨ r2 = g(∆θ sinψ0 −∆φ cosψ0) ¨ r3 = 1 m u1 −g
Linearizing (6), we get:
 
˙ p ˙ q ˙ r
 = I−1 
0 L 0 −L −L 0 L 0 γ −γ γ −γ  
    F1 F2 F3 F4
    (9) If we assume the rotor craft is symmetric so Ixx = Iyy, we get:
˙ p =
u2,x Ixx
=
L Ixx
(F2 −F4)
˙ q =
u2,y Iyy
=
L Iyy
(F3 −F1),
˙ r =
u2,z Izz
=
γ Izz
(F1 −F2 + F3 −F4).
In other words, the equations of motion are decoupled in terms of angular accelerations. Each component of angular acceleration depends only on the appropriate component of u2.
3.2 Position and Attitude Control The control problem is to determine the four inputs, {u1,u2} required to hover or to follow a desired trajectory, zdes. As shown in Figure 2, we will use errors in the robot’s position to


 
 
 
 
 
 
 
 
