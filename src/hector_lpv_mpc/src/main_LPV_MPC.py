#!/usr/bin/env python


# Modified controller code based on Python code from author Mark Misin. 
# The portion of the code for the Model Predictive Controller(MPC) based on Linear Parameter Varying (LPV) model was made freely available for distribution from the course Applied Control Systems 3: UAV
# drone (3D # Dynamics & control) by Mark Misin: https://www.udemy.com/course/applied-control-systems-for-engineers-2-uav-drone-control/



import platform
print("Python " + platform.python_version())
import support_files_drone as sfd #

# import numpy libraries
import math
import numpy as np

# ROS libraries for hector drone
import rospy
from std_msgs.msg import String
import geometry_msgs.msg


# Create an object for the support functions.
support=sfd.SupportFilesDrone()
constants=support.constants

# Load the constant values needed in the main file
Ts=constants[6]
controlled_states=constants[13] # number of outputs
innerDyn_length=constants[15] # number of inner control loop iterations
pos_x_y= constants[23]

if pos_x_y==1:
    extension=2.5
elif pos_x_y==0:
    extension=0
else:
    print("Please make pos_x_y variable either 1 or 0 in the support file initial funtcion (where all the initial constants are)")
    exit()

sub_loop=constants[24]

sim_version=constants[25]
if sim_version==1:
    sim_version=1
elif sim_version==2:
    sim_version=2
else:
    print("Please assign only 1 or 2 to the variable 'sim_version' in the input function")
    exit()

# Generate the reference signals
t=np.arange(0,100+Ts*innerDyn_length,Ts*innerDyn_length) # time from 0 to 100 seconds, sample time (Ts=0.4 second)
t_angles=np.arange(0,t[-1]+Ts,Ts)
t_ani=np.arange(0,t[-1]+Ts/sub_loop,Ts/sub_loop)
X_ref,X_dot_ref,X_dot_dot_ref,Y_ref,Y_dot_ref,Y_dot_dot_ref,Z_ref,Z_dot_ref,Z_dot_dot_ref,psi_ref=support.trajectory_generator(t)
plotl=len(t) # Number of outer control loop iterations

# Load the initial state vector
ut=0
vt=0
wt=0
pt=0
qt=0
rt=0
xt=0
yt=-1
zt=0
phit=0
thetat=0
psit=psi_ref[0]

states=np.array([ut,vt,wt,pt,qt,rt,xt,yt,zt,phit,thetat,psit])
statesTotal=[states] # It will keep track of all your states during the entire manoeuvre

# statesTotal2=[states]
statesTotal_ani=[states[6:len(states)]]

# Assume that first Phi_ref, Theta_ref, Psi_ref are equal to the first phit, thetat, psit
ref_angles_total=np.array([[phit,thetat,psit]])

velocityXYZ_total= np.array([[0,0,0]])

# Initial drone propeller states
omega1= 110*np.pi/3 # rad/s at t=-Ts s (Ts seconds before NOW)
omega2= 110*np.pi/3 # rad/s at t=-Ts s (Ts seconds before NOW)
omega3= 110*np.pi/3 # rad/s at t=-Ts s (Ts seconds before NOW)
omega4= 110*np.pi/3 # rad/s at t=-Ts s (Ts seconds before NOW)
omega_total= omega1-omega2+omega3-omega4

ct= constants[10]
cq= constants[11]
l= constants[12]

U1= ct*(omega1**2+omega2**2+omega3**2+omega4**2) # Input at t = -Ts s
U2= ct*l*(omega2**2-omega4**2) # Input at t = -Ts s
U3= ct*l*(omega3**2-omega1**2) # Input at t = -Ts s
U4= cq*(-omega1**2+omega2**2-omega3**2+omega4**2) # Input at t = -Ts s
UTotal= np.array([[U1,U2,U3,U4]]) # 4 inputs
omegas_bundle= np.array([[omega1,omega2,omega3,omega4]])
UTotal_ani= UTotal

########## Start of the global controller code #################################

for i_global in range(0,plotl-1):
    # Implement the position controller (state feedback linearization)
    phi_ref, theta_ref, U1=support.pos_controller(X_ref[i_global+1],X_dot_ref[i_global+1],X_dot_dot_ref[i_global+1],Y_ref[i_global+1],Y_dot_ref[i_global+1],Y_dot_dot_ref[i_global+1],Z_ref[i_global+1],Z_dot_ref[i_global+1],Z_dot_dot_ref[i_global+1],psi_ref[i_global+1],states)
    Phi_ref=np.transpose([phi_ref*np.ones(innerDyn_length+1)])
    Theta_ref=np.transpose([theta_ref*np.ones(innerDyn_length+1)])

    # Make Psi_ref increase continuosly in a linear fashion per outer loop
    Psi_ref=np.transpose([np.zeros(innerDyn_length+1)])
    for yaw_step in range(0, innerDyn_length+1):
        Psi_ref[yaw_step]=psi_ref[i_global]+(psi_ref[i_global+1]-psi_ref[i_global])/(Ts*innerDyn_length)*Ts*yaw_step

    temp_angles=np.concatenate((Phi_ref[1:len(Phi_ref)],Theta_ref[1:len(Theta_ref)],Psi_ref[1:len(Psi_ref)]),axis=1)
    ref_angles_total=np.concatenate((ref_angles_total,temp_angles),axis=0)
    # Create a reference vector
    refSignals=np.zeros(len(Phi_ref)*controlled_states)

    # Build up the reference signal vector:
    # refSignal = [Phi_ref_0, Theta_ref_0, Psi_ref_0, Phi_ref_1, Theta_ref_2, Psi_ref_2, ... etc.]
    k=0
    for i in range(0,len(refSignals),controlled_states):
        refSignals[i]=Phi_ref[k]
        refSignals[i+1]=Theta_ref[k]
        refSignals[i+2]=Psi_ref[k]
        k=k+1

    # Initiate the controller - simulation loops
    hz=support.constants[14] # horizon period
    k=0 # for reading reference signals
    # statesTotal2=np.concatenate((statesTotal2,[states]),axis=0)
    for i in range(0,innerDyn_length):
        # Generate the discrete state space matrices
        Ad,Bd,Cd,Dd,x_dot,y_dot,z_dot,phi,phi_dot,theta,theta_dot,psi,psi_dot=support.LPV_cont_discrete(states, omega_total)
        x_dot=np.transpose([x_dot])
        y_dot=np.transpose([y_dot])
        z_dot=np.transpose([z_dot])
        temp_velocityXYZ=np.concatenate(([[x_dot],[y_dot],[z_dot]]),axis=1)
        velocityXYZ_total=np.concatenate((velocityXYZ_total,temp_velocityXYZ),axis=0)

        # Generate the augmented current state and the reference vector
        x_aug_t=np.transpose([np.concatenate(([phi,phi_dot,theta,theta_dot,psi,psi_dot],[U2,U3,U4]),axis=0)])

        # Ts=0.1 s
        # From the refSignals vector, only extract the reference values from your [current sample (NOW) + Ts] to [NOW+horizon period (hz)]
        # Example: t_now is 3 seconds, hz = 15 samples, so from refSignals vectors, you move the elements to vector r:
        # r=[Phi_ref_3.1, Theta_ref_3.1, Psi_ref_3.1, Phi_ref_3.2, ... , Phi_ref_4.5, Theta_ref_4.5, Psi_ref_4.5]
        # With each loop, it all shifts by 0.1 second because Ts=0.1 s
        k=k+controlled_states
        if k+controlled_states*hz<=len(refSignals):
            r=refSignals[k:k+controlled_states*hz]
        else:
            r=refSignals[k:len(refSignals)]
            hz=hz-1

        # Generate the compact simplification matrices for the cost function
        Hdb,Fdbt,Cdb,Adc=support.mpc_simplification(Ad,Bd,Cd,Dd,hz)
        ft=np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)],r),axis=0),Fdbt)

        du=-np.matmul(np.linalg.inv(Hdb),np.transpose([ft]))

        # Update the real inputs
        U2= U2+du[0][0]
        U3= U3+du[1][0]
        U4= U4+du[2][0]

        # Keep track of your inputs
        UTotal= np.concatenate((UTotal,np.array([[U1,U2,U3,U4]])),axis=0)
        # print(UTotal)

        # Compute the new omegas based on the new U-s
        U1C= U1/ct
        U2C= U2/(ct*l)
        U3C= U3/(ct*l)
        U4C= U4/cq

        UC_vector= np.zeros((4,1))
        UC_vector[0,0]= U1C
        UC_vector[1,0]= U2C
        UC_vector[2,0]= U3C
        UC_vector[3,0]= U4C

        omega_Matrix= np.zeros((4,4))

        omega_Matrix[0,0]=1
        omega_Matrix[0,1]=1
        omega_Matrix[0,2]=1
        omega_Matrix[0,3]=1
        omega_Matrix[1,1]=1
        omega_Matrix[1,3]=-1
        omega_Matrix[2,0]=-1
        omega_Matrix[2,2]=1
        omega_Matrix[3,0]=-1
        omega_Matrix[3,1]=1
        omega_Matrix[3,2]=-1
        omega_Matrix[3,3]=1

        omega_Matrix_inverse= np.linalg.inv(omega_Matrix)
        omegas_vector= np.matmul(omega_Matrix_inverse,UC_vector)

        omega1P2= abs(omegas_vector[0,0])
        omega2P2= abs(omegas_vector[1,0])
        omega3P2= abs(omegas_vector[2,0])
        omega4P2= abs(omegas_vector[3,0])

        if omega1P2<=0 or omega2P2<=0 or omega3P2<=0 or omega4P2<=0:
            print("You can't take a square root of a negative number")
            print("The problem might be that the trajectory is too chaotic or it might have large discontinuous jumps")
            print("Try to make a smoother trajectory without discontinuous jumps")
            print("Other possible causes might be values for variables such as Ts, hz, innerDyn_length, px, py, pz")
            print("If problems occur, please download the files again, use the default settings and try to change values one by one.")
            exit()
        else:
            omega1=np.sqrt(omega1P2)
            omega2=np.sqrt(omega2P2)
            omega3=np.sqrt(omega3P2)
            omega4=np.sqrt(omega4P2)

        omegas_bundle= np.concatenate((omegas_bundle,np.array([[omega1,omega2,omega3,omega4]])),axis=0)

        # Compute the new total omega
        omega_total= omega1-omega2+omega3-omega4
        # Compute new states in the open loop system (interval: Ts/10)
        states,states_ani,U_ani=support.open_loop_new_states(states,omega_total,U1,U2,U3,U4)

        # print(states)
        # print(statesTotal)
        statesTotal= np.concatenate((statesTotal,[states]),axis=0)
        statesTotal_ani= np.concatenate((statesTotal_ani,states_ani),axis=0)
        UTotal_ani= np.concatenate((UTotal_ani,U_ani),axis=0)
        print(UTotal_ani)

################################ Drone velocities input Loop ###############################

statesTotal_x= statesTotal_ani[:,0]
statesTotal_y= statesTotal_ani[:,1]

statesTotal_phi= statesTotal_ani[:,3]
statesTotal_theta= statesTotal_ani[:,4]
statesTotal_psi= statesTotal_ani[:,5]


UTotal_U1= UTotal_ani[:,0]
UTotal_U2= UTotal_ani[:,1]
UTotal_U3= UTotal_ani[:,2]
UTotal_U4= UTotal_ani[:,3]

################################### END OF MPV-LPV CONTROLLER CODE CODE  #######################################################################################################
 
# Start of code for Hector quadcopter:
  

#function to takeoff 
def takeoff_command():
    
    cmd = geometry_msgs.msg.Twist()
 
    #linear:{x: 0, y: 0, z: 5.8}, ?angular: {x: 0, y: 0, z: 0}

    cmd.linear.x =0
    cmd.linear.y =0
    cmd.linear.z =40 # motion in vertical z-axis

    cmd.angular.x =0
    cmd.angular.y =0
    cmd.angular.z =0
   
    return cmd

def open_loop(statesTotal):
    rospy.init_node('open_loop', anonymous=True)
    quad_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    rate = rospy.Rate(10) # 10hz

    cmd = geometry_msgs.msg.Twist()

    i=0;
    n=0
         

    while not rospy.is_shutdown():

        if n<20:
		quad_vel.publish(takeoff_command())
		n=n+1
        else:
		cmd.linear.x =  UTotal_ani[i,0]
		cmd.linear.y =  UTotal_ani[i,1]
		#cmd.linear.z =  UTotal_ani[i,2]	       

		cmd.angular.x =  statesTotal_ani[i,3]
		cmd.angular.y =  statesTotal_ani[i,4] 
		cmd.angular.z =  statesTotal_ani[i,5]

		i=i+1;
		quad_vel.publish(cmd)
        	
        rate.sleep()
         

if __name__ == '__main__':
	try:
		open_loop(statesTotal)
	except rospy.ROSInterruptException:
		pass
