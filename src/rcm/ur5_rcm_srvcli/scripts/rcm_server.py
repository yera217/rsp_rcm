#!/usr/bin/env python

from ur5_rcm_msgs.srv import OptRCM, OptRCMResponse
import rospy

from std_msgs.msg import Float64MultiArray

import numpy as np
from numpy import linalg as LA
from numpy.linalg import norm

import pyOpt

from pyOpt import Optimization
from pyOpt import SLSQP



def handle_opt_rcm(req):
	### Retreiving data from srv message
	x_curr=req.fk.data[0:3]
	xyz_cl=req.fk.data[3:6]
	xyz_RCM=req.fk.data[6:9]
	x_des=req.fk.data[9:12]
	
	x_curr=np.array(x_curr)
	xyz_cl=np.array(xyz_cl)
	xyz_RCM=np.array(xyz_RCM)
	x_des=np.array(x_des)

	jac_tip_flattened=req.jac_tip.data
	jac_cl_flattened=req.jac_cl.data
	J_tip=np.zeros((3,6))
	J_cl=np.zeros((3,6))
	
	for i in range(3):
		for j in range(6):
			J_tip[i,j]=jac_tip_flattened[i*6+j]

	for i in range(3):
		for j in range(6):
			J_cl[i,j]=jac_cl_flattened[i*6+j]

	print("****************")
	print("xyz_tip: ", x_curr)
	print("xyz_cl ", xyz_cl)

	def helper(J,qdot,n):
	    res=0.0
	    for j in range(6):
	        res+=J[n,j]*qdot[j]
	    return res

	# Objective function for pyOpt                                                                             
	def ik(qdot):
	    #qdot=np.array(qdot)
	    f = LA.norm( np.matmul(J_tip,qdot)  - (x_des -x_curr) )
	    epsilon = 0.01
	    g=[0.0]*1
	    g[0] = LA.norm( [ xyz_cl[0]- xyz_RCM[0] + helper(J_cl,qdot, 0), xyz_cl[1] - xyz_RCM[1] + helper(J_cl,qdot, 1), xyz_cl[2] - xyz_RCM[2] + helper(J_cl,qdot, 2) ] ) - epsilon
	    
	    fail=0
	    return f,g,fail

	


	opt_prob = pyOpt.Optimization('IK velocity',ik)
	opt_prob.addObj('f')
	opt_prob.addVarGroup('qdot', 6, 'c', lower=-0.2, upper=0.2, value=0)
	opt_prob.addCon('g','i')

	slsqp = pyOpt.SLSQP()
	#slsqp.setOption('IPRINT', -1)

	[_, sol, _] = slsqp(opt_prob)
	#print("SOLUTION: ", sol)
	del_q = Float64MultiArray()
	del_q.data=sol

	response=OptRCMResponse(del_q)


	return response

def opt_rcm_server():
	rospy.init_node('opt_rcm_server')
	s = rospy.Service('opt_rcm', OptRCM, handle_opt_rcm)
	rospy.spin()

if __name__ == "__main__":
	opt_rcm_server()