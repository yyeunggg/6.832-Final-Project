#-------------
import time
import numpy as np




#Initiate Constants
PI = np.pi


#--------------------------------------------------------------------------------------------------------------------------------------
def PID(roll, pitch, yaw, f, x, y, z):
	#Define the global variables to prevent them from dying and resetting to zero, each time a function call occurs. Some of these variables 		may be redundant.
	global kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw
	global prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch
	global iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, flag, setpoint, sampleTime
	global kp_z, ki_z, kd_z
	global kp_x, ki_x, kd_x
	global kp_y, ki_y, kd_y
	#-----------------------
	#Debug,print input
	# print(z)

	#Assign your PID values here. From symmetry, control for roll and pitch is the same.
	kp_roll = 70
	ki_roll = 0.0002
	kd_roll = 89
	kp_pitch = kp_roll
	ki_pitch = ki_roll
	kd_pitch = kd_roll
	kp_yaw = 0.1
	ki_yaw = 0.1e-3
	kd_yaw = 0.1
	#Cartesian Coordinate
	kp_z = 10
	ki_z = 0.0015
	kd_z = 100

	#Rotational Coordinate
	setpoint_pitch = 0
	setpoint_roll = 0
	setpoint_yaw = 0

	#Cartesian Coordinate
	setpoint_z = 1

	###################Brand New Section
	#From the initial condition, +pitch = x, +roll = -y, don't touch yaw
	setpoint_x = 5
	setpoint_y = 0

	kp_x = 0.1
	ki_x = 0.1e-2
	kd_x = 0.1

	kp_y = 0.1
	ki_y = 0.1e-2
	kd_y = 10

	err_x = -(float(x) - setpoint_x)*1e3
	err_y = -(float(y) - setpoint_y)*1e3


	#Some initialization
	sampleTime = 0
	flag = 0

	#Get time
	currTime = time.time()

	#Calculate Error, reinitialize
	if flag == 0:
		prevTime = 0
		prevErr_x = 0
		pMem_x = 0
		iMem_x = 0
		dMem_x = 0

		prevErr_y = 0
		pMem_y = 0
		iMem_y = 0
		dMem_y = 0

	dErr_x = err_x - prevErr_x
	dErr_y = err_y - prevErr_y


	dTime = currTime - prevTime


	if(dTime >= sampleTime):
		#Kp*e(t)
		pMem_x = kp_x * err_x
		pMem_y = kp_y * err_y

		#integral(e(t))
		iMem_x += err_x * dTime
		iMem_y += err_y * dTime
		

		#Anti-Windup
		i_threshold = 200
		if(iMem_x > i_threshold): iMem_x = i_threshold
		if(iMem_x < -i_threshold): iMem_x = -i_threshold
		if(iMem_y > i_threshold): iMem_y = i_threshold
		if(iMem_y < -i_threshold): iMem_y = -i_threshold

		#derivative(e(t))
		dMem_x = dErr_x / dTime
		dMem_y = dErr_y / dTime

	prevErr_x = err_x
	prevErr_y = err_y
	
	output_x = pMem_x + ki_x * iMem_x + kd_x * dMem_x 
	output_y = pMem_y + ki_y * iMem_y + kd_y * dMem_y

	#Anti-Windup
	angle_threshold = 10.0
	if(output_x > angle_threshold): output_x = angle_threshold
	if(output_x < -angle_threshold): output_x = -angle_threshold
	if(output_y > angle_threshold): output_y = angle_threshold
	if(output_y < -angle_threshold): output_y = -angle_threshold

	# print('{:.6e}'.format(output_x),'{:.6e}'.format(output_y))

	#Map to xxx degree maximum
	map_angle = 0.5
	output_x = output_x/(angle_threshold/map_angle*10)
	output_y = output_y/(angle_threshold/map_angle*10)


	setpoint_pitch += output_x
	setpoint_roll -= output_y

	# print( output_y, setpoint_roll)


	# print('{:.2e}'.format(setpoint_pitch),'{:.2e}'.format(setpoint_roll))

	##############################################



	#Calculate error
	err_pitch = float(pitch)*(180 /PI) - setpoint_pitch
 	err_roll = float(roll)*(180 /PI) - setpoint_roll
	err_yaw = float(yaw)*(180/PI) - setpoint_yaw
	err_z = -(float(z) - setpoint_z)


	#-----------------------
	#Reset the following variables during the first run only.
	if flag == 0:
		prevErr_roll = 0
		prevErr_pitch = 0
		prevErr_yaw = 0
		pMem_roll = 0
		pMem_pitch = 0
		pMem_yaw = 0
		iMem_roll = 0
		iMem_pitch = 0
		iMem_yaw = 0
		dMem_roll = 0
		dMem_pitch = 0
		dMem_yaw = 0

		prevErr_z = 0
		pMem_z = 0
		iMem_z = 0
		dMem_z = 0

		flag += 1
	#------------------------
	#Define dt, dy(t) here for kd calculations.
	dErr_pitch = err_pitch - prevErr_pitch
	dErr_roll = err_roll - prevErr_roll
	dErr_yaw = err_yaw - prevErr_yaw
	dErr_z = err_z - prevErr_z

	
	
	#-------------------------------------------------------------------------------------------------------------------------------
	#This is the Heart of the PID algorithm. PID behaves more accurately, if it is sampled at regular intervals. You can change the sampleTime to whatever value is suitable for your plant.
	if(dTime >= sampleTime):
		#Kp*e(t)
		pMem_roll = kp_roll * err_roll
		pMem_pitch = kp_pitch * err_pitch
		pMem_yaw = kp_yaw * err_yaw
		pMem_z = kp_z * err_z

		#integral(e(t))
		iMem_roll += err_pitch * dTime
		iMem_pitch += err_roll * dTime
		iMem_yaw += err_yaw * dTime
		iMem_z += err_z * dTime

		#Anti-Windup
		if(iMem_roll > 400): iMem_roll = 400
		if(iMem_roll < -400): iMem_roll = -400
		if(iMem_pitch > 400): iMem_pitch = 400
		if(iMem_pitch < -400): iMem_pitch = -400
		if(iMem_yaw > 400): iMem_yaw = 400
		if(iMem_yaw < -400): iMem_yaw = 400
		if(iMem_z > 400): iMem_z = 400
		if(iMem_z < -400): iMem_z = 400

		#derivative(e(t))
		dMem_roll = dErr_roll / dTime
		dMem_pitch = dErr_pitch / dTime
		dMem_yaw = dErr_yaw / dTime
		dMem_z = dErr_z / dTime


	#Store the current variables into previous variables for the next iteration.
	prevTime = currTime
	prevErr_roll = err_roll
	prevErr_pitch = err_pitch
	prevErr_yaw = err_yaw
	prevErr_z = err_z

	
	#output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
	output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
	output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
	output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw 

	#Control Effort xyz
	output_z = pMem_z + ki_z * iMem_z + kd_z * dMem_z 

	#Incoorporate x,y
	print('{:.2e}'.format(x),'{:.2e}'.format(y),'{:.2e}'.format(z), '{:.2e}'.format(roll),'{:.2e}'.format(pitch),'{:.2e}'.format(yaw))

	#-------------------------------------------------------------------------------------------------------------------------------

	#br in my code is fr in gazebo's world
	esc_br = 1500 + output_roll + output_pitch - output_yaw
	#bl in my code is br in gazebo's world
	esc_bl = 1500 + output_roll - output_pitch + output_yaw
	#fl in my code is bl in gazebo's world
	esc_fl = 1500 - output_roll - output_pitch - output_yaw
	#fr in my code is fl in gazebo's world
	esc_fr = 1500 - output_roll + output_pitch + output_yaw

	
	#Limit the ESC pulses to upper limit and lower limit, in case the PID algorithm goes crazy
	if(esc_br > 2000): esc_br = 2000
	if(esc_bl > 2000): esc_bl = 2000
	if(esc_fr > 2000): esc_fr = 2000
	if(esc_fl > 2000): esc_fl = 2000
	
	if(esc_br < 1100): esc_br = 1100
	if(esc_bl < 1100): esc_bl = 1100
	if(esc_fr < 1100): esc_fr = 1100
	if(esc_fl < 1100): esc_fl = 1100
	

	#Map the esc values to motor values
	#This part doesn't need to be changed either
	br_motor_vel = ((esc_br - 1500)/25) + 50
	bl_motor_vel = ((esc_bl - 1500)/25) + 50
	fr_motor_vel = ((esc_fr - 1500)/25) + 50
	fl_motor_vel = ((esc_fl - 1500)/25) + 50

	# #Include the control of z
	# print(output_z)
	br_motor_vel += output_z
	bl_motor_vel += output_z
	fr_motor_vel += output_z
	fl_motor_vel += output_z

	#----------------------------------------------------------------------------------------------------------------------------------

	#Provide the motor velocities to the object 'f' that will now exit out of this function, and gets published to gazebo, providing velocities to each motor. Note that the sign here is +,-,+,- i.e CW, CCW, CW, CCW in gazebo model. Change view of gazebo model (by scrolling) such that the green line comes to your left, red line goes forward, and blue line goes upward. This is the convention that i refer to as "Gazebo model" incase you get confused.
	f.data = [fr_motor_vel,-fl_motor_vel,bl_motor_vel, -br_motor_vel]

	# print('{:.2e}'.format(f.data[0]),'{:.2e}'.format(f.data[1]),'{:.2e}'.format(f.data[2]),'{:.2e}'.format(f.data[3]))


	#Return these variables back to the control file.
	return f, err_roll, err_pitch, err_yaw
#--------------------------------------------------------------------------------------------------------------------------------------
