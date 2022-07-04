#!/usr/bin/env python

#ODEINT test

import rospy
from std_msgs.msg import String
import math
import geometry_msgs.msg

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def fixed_set_point(t, z_des):
	#FIXED_SET_POINT  
        #Outputs a constant desired state = [z_des;0] except at t = 0 where it returns [0;0]

	# Hover
	#z_des = 0;

	# Step
	#z_des = 1;

	if (t==0):
	  s_des=np.array([[0],[0]]);
	else:
	  s_des=np.array([[z_des],[0]]);

        return s_des;



def hover_controller(s_des):

    #s: 2x1 vector containing the current state [ z ]
    #                                           [v_z]:
    s = np.array([[0],[0]]);
    #   s_des: 2x1 vector containing desired state [ z ]
    #                                              [v_z]
    #s_des = 

    #   params: robot parameters
    m = 0.18;
    g = 9.81;
    Kp = 98; 
    Kv = 9.8;

    u_max = 1.2*m*g;
    u_min = 0;

    e = np.array(s_des[0]) - np.array(s[0]);

    e_dot= np.array(s_des[0]) - np.array(s[0]);

    zdes_dot_dot = 0;

    u = m*(zdes_dot_dot+Kp*e+Kv*e_dot+g);

    u_clamped = min(max(u_min, u), u_max);

    s_dot = np.array([[s[1]], [u_clamped/m - g]]);

    return s_dot

    #return u

#sys_eom Differential equation for the height control system

'''
def sys_eom(hover_controller_handl, trajhandle):
     
     s = np.array([[0],[0]]);
     m = 0;
     #s_des = trajhandle[m];  #t?
     s_des =1;
     u = hover_controller_handl(s_des);
     
     u_min = 0
     mass = 0.18
     gravity = 9.81
     u_max = 1.2*mass*gravity;
     

     u_clamped = min(max(u_min, u), u_max);

     s_dot = np.array([[s[1]], [u_clamped/mass - gravity]]);

     return s_dot
'''

def height_controller(controlhandle, trajhandle):

	# initial conditions
	max_iter  = 100;       # max iteration
	starttime = 0;         # start of simulation in seconds
	tstep     = 0.01;      # this determines the time step at which the solution is given
	cstep     = 0.05;      # image capture time interval
	nstep     = int(cstep/tstep);          ##cstep/tstep 
	time      = starttime; # current time

	#Get start and stop position
	des_start = np.array(trajhandle[0]); #s_des = [z_des;0];
	des_stop  = np.array(trajhandle[1]);
	stop_pos  = des_stop;
	x0        = des_start;

	#an_array = np.empty((x,y))
	xtraj = np.empty((max_iter*nstep, len(x0))) ##############length(x0) ?
	xtraj[:] = np.NaN

	ttraj = np.empty((max_iter*nstep, len(x0)));
	ttraj[:] = np.NaN;

	x=x0;   # state

        

	for iter in range(max_iter):
		
                # t = np.linspace(0,x)
		# timeint = time:tstep:time+cstep;
                timeint = np.linspace(time+tstep,time+cstep)


		# solve ODE
		#[tsave, xsave] = odeint(hover_controller,timeint,x)
                [xsave] = odeint(hover_controller,timeint,x)
                
		#x = np.transpose(xsave[None, :]);

		#Save to traj
		#xtraj[1:] = np.array(xsave[1:]);
                #l = np.array(xsave[1:]);
                #print(l)
		#ttraj[[iter-1]*nstep+1:iter*nstep] = np.array(tsave[1:None-1]);


		#t = toc;

                #t_out = np.array(ttraj[1:iter*nstep]);
		#z_out = np.array(xtraj[0:iter*nstep,1]);


        return xsave


def cmd_command(x):
    
    cmd = geometry_msgs.msg.Twist()
 
    
    #linear:{x: 0, y: 0, z: 5.8}, ?angular: {x: 0, y: 0, z: 0}

    cmd.linear.x =0
    cmd.linear.y =0
    cmd.linear.z = x*10 #multiply by a factor of 10 for display purposes

    cmd.angular.x =0
    cmd.angular.y =0
    cmd.angular.z =0
   
    return cmd

                    
#def open_loop():
    
	
if __name__ == "__main__":
        
        
  
        t=1;

        #hover
	#z_des = 0;

	#Step
	z_des = 1;
   

	#trajectory generator
	trajhandle = fixed_set_point(t, z_des); #When t=0 and z_des=0 or 1 => s_des = [0;0];
		                                #When t=1 and z_des=0 or 1 => s_des = [z_des;0];
	
        controlhandle = hover_controller(trajhandle);

	#Run simulation with given trajectory generator and controller
	xsave = height_controller(controlhandle,trajhandle);


        try:  
            rospy.init_node('open_loop', anonymous=True)
            quad_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
            rate = rospy.Rate(10) # 10hz
            
            i=0
            while not rospy.is_shutdown():

                quad_vel.publish(cmd_command(xsave[i]))
                i=i+1
                rate.sleep()
		#open_loop()

	except rospy.ROSInterruptException:
		pass
    


