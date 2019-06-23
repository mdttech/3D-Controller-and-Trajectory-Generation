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

Our controllers are derived by linearizing the equations of motion and motor models (5 – 2) at an operating point that corresponds to the nominal hover state, r = r0, θ = φ = 0,ψ = ψ0, ˙ r = 0, and ˙ φ = ˙ θ = ˙ ψ = 0, where the roll and pitch angles are small (cφ ≈ 1, cθ ≈ 1,sφ ≈ φ, and sθ ≈ θ). At this state the lift from the propellers is given by:

     Fi0 = mg/ 4 ,
     
The nominal values for the inputs at hover are u1,0 = mg, u2,0 = 0. Linearizing (5), we get:

    ¨ r1 = g(∆θ cosψ0 + ∆φ sinψ0) (8)
    ¨ r2 = g(∆θ sinψ0 −∆φ cosψ0)
    ¨ r3 = 1/m u1 −g
Linearizing (6), we get:
      
      [˙ p ˙ q ˙ r] = I^−1[0 L 0 −L; −L 0 L 0 ; γ −γ γ −γ ][F1 F2 F3 F4] (9) 
      
If we assume the rotor craft is symmetric so Ixx = Iyy, we get:

    ˙ p = u2,x/ Ixx =L/Ixx(F2 −F4)
    ˙ q = u2,y/Iyy = L/Iyy(F3 −F1),
    ˙ r = u2,z/Izz = γ/Izz(F1 −F2 + F3 −F4).
    
In other words, the equations of motion are decoupled in terms of angular accelerations. Each component of angular acceleration depends only on the appropriate component of u2.

![2](https://user-images.githubusercontent.com/36922299/59974502-9a874700-95ca-11e9-9150-b6f3ecad9b5f.png)

# 3.2 Position and Attitude Control 
The control problem is to determine the four inputs, {u1,u2} required to hover or to follow a desired trajectory, zdes. As shown in Figure 2, we will use errors in the robot’s position to drive a position controller from (8) which directly determines u1. The model in (8) also allows us to derive a desired orientation. The attitude controller for this orientation is derived from the model in (9). The transformation of the inputs into {u1,u2} is straightforward and is described in [1]. The inner attitude control loop uses onboard accelerometers and gyros to control the roll, pitch, and yaw and runs at approximately 1kHz, while the outer position control loop uses estimates of position and velocity of the center of mass to control the trajectory in three dimensions at 100-200 Hz. 
# Attitude Control
We now present a proportional plus derivative (PD) attitude controller to track a trajectory in SO(3) speciﬁed in terms of a desired roll, pitch and yaw angle. Since our development of the controller will be based on linearized equations of motion, the attitude must be close to the nominal hover state where the roll and pitch angles are small. Near the nominal hover state the proportional plus derivative control laws take the form:

     u2 =[ kp,φ(φdes −φ) + kd,φ(pdes −p); kp,θ(θdes −θ) + kd,θ(qdes −q); k p,ψ(ψdes −ψ) + kd,ψ(rdes −r)] (10)
     
# Position Control
In the next subsection, we will present two position control methods that use the roll and pitch angles as inputs to drive the position of the quadrotor. In both methods, the position control algorithm will determine the desired roll and pitch angles, θdes and φdes, which can be used to compute the desired speeds from (10). The ﬁrst, a hover controller, is used for station-keeping or maintaining the position at a desired position vector, r0. The second tracks a speciﬁed trajectory, rT(t), in three dimensions. In both cases, the desired yaw angle, is speciﬁed independently. It can either be a constant, ψ0 or a time varying quantity, ψT(t). We will assume that the desired trajectory: 

    zdes =[rT(t); ψT(t)],
will be provided by a trajectory planner as an input to specify the trajectory of the position vector and the yaw angle we are trying to track.
# 3.3 Hover Controller
For hovering, rT(t) = r0 and ψT(t) = ψ0. The command accelerations, ¨ ri,des, are calculated from a proportional plus derivative (PD) controller. Deﬁne the position error in terms of components using the standard reference triad ai by:
     
     ei = (ri,T −ri).
In order to guarantee that this error goes exponentially to zero, we require

     (¨ ri,T −¨ ri,des) + kd,i(˙ ri,T − ˙ ri) + kp,i(ri,T −ri) = 0, (11)
     
where ˙ ri,T = ¨ ri,T = 0 for hover.
From (8) we can obtain the relationship between the desired accelerations and roll and pitch angles. Given that ∆θ = θ−θ0 = θ and ∆φ = φ−φ0 = φ, we can write 
      
     ¨ r1,des = g(θdes cosψT + φdes sinψT)
     ¨ r2,des = g(θdes sinψT −φdes cosψT)
     ¨ r3,des = 1 m u1 −g.
For hover, the last equation yields:

     u1 = mg + m¨ r3,des = mg−m(kd,3 ˙ r3 + kp,3(r3 −r3,0)) (13)
     
The other two equations can be used to compute the desired roll and pitch angles for the attitude controller:
     
     φdes = 1/g(¨ r1,des sinψT −¨ r2,des cosψT) (14a)
    θdes = 1/g(¨ r1,des cosψT + ¨ r2,des sinψT) (14b)
    
The desired roll and pitch velocities are taken to be zero.

     pdes = 0 (15a) 
     qdes = 0 (15b)
     
Since the yaw, ψT(t) is prescribed independently by the trajectory generator, we get:
 
     ψdes = ψT(t) (16a) 
     rdes = ˙ ψT(t) (16b)
     
These equations provide the setpoints for the attitude controller in (10). Note that the attitude controller must run an order of magnitude faster than the position control loop in order for this to work. In practice as discussed in [1], the position controller runs at 100Hz, while the inner attitude control loop runs at 1kHz.


# 4 Tasks
# 4.1 Controller
You will need to complete the implementation of the PD controller in the ﬁle controller.m. Before implementing your own functions, you should ﬁrst try running runsim.m in your MATLAB environment. If you see a quadrotor falling from starting position, then the simulator is running correctly and you may continue with other tasks. To test diﬀerent trajectories, you need to modify variable trajhandle inside runsim.m to point to the appropriate function. Examples are provided inside the ﬁle runsim.m.
# 4.2 Trajectory Generation
You will need to implement the function traj_generator.m to generate a trajectory through a series of way points. You may use the method outlined in section 4 or your own method of interpolating smoothly. The provided function gives an example trajectory which jumps directly between the waypoints. If you execute this, you can see the undesirable eﬀects of giving a controller a non-smooth trajectory.

# OUTPUT 

