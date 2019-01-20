# -*- coding: utf-8 -*-
"""
Created on 2019
@author: Richard Bloemenkamp
"""
import pybullet as p
import time
import numpy as np

p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
plId=p.createMultiBody(0,0)
p.resetDebugVisualizerCamera( cameraDistance=4, cameraYaw=10, cameraPitch=-20, 
                              cameraTargetPosition=[0.0, 0.0, 0.25])

sh_colFoot = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01,0.01,0.1])
sh_colBody = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.1,0.1,0.1])
sh_colPx = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01,0.1,0.1])
sh_colPy = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.1,0.01,0.1])

bodyId=p.createMultiBody(baseMass=0.8,baseCollisionShapeIndex = sh_colBody,
                        basePosition = [0,0,1.5],baseOrientation=[0,0,0,1])
footId=p.createMultiBody(baseMass=.1,baseCollisionShapeIndex = sh_colFoot,
                        basePosition = [0,0,0.5],baseOrientation=[0,0,0,1])
#4 inertia increasing plates
cubeId3=p.createMultiBody(baseMass=.1,baseCollisionShapeIndex = sh_colPx,
                        basePosition = [-0.5,0,1.5],baseOrientation=[0,0,0,1])
cubeId4=p.createMultiBody(baseMass=.1,baseCollisionShapeIndex = sh_colPx,
                        basePosition = [0.5,0,1.5],baseOrientation=[0,0,0,1])
cubeId5=p.createMultiBody(baseMass=.1,baseCollisionShapeIndex = sh_colPy,
                        basePosition = [0,-0.5,1.5],baseOrientation=[0,0,0,1])
cubeId6=p.createMultiBody(baseMass=.1,baseCollisionShapeIndex = sh_colPy,
                        basePosition = [0,0.5,1.5],baseOrientation=[0,0,0,1])

p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
#make to plane less slippery
p.changeDynamics(plId,-1,lateralFriction=10)

#connect foot to body
cid = p.createConstraint(bodyId,-1,footId,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,1])
#connect plates to body
cid2 = p.createConstraint(bodyId,-1,cubeId3,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0.5,0,0])
cid3 = p.createConstraint(bodyId,-1,cubeId4,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[-0.5,0,0])
cid4 = p.createConstraint(bodyId,-1,cubeId5,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0.5,0])
cid5 = p.createConstraint(bodyId,-1,cubeId6,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,-0.5,0])

#init
pivot=[0,0,0,1]
decomprPhase=0
xbody,orQT = p.getBasePositionAndOrientation(bodyId)
orDes=orQT
orEU=p.getEulerFromQuaternion(orQT)
t=0
tstr=0
while 1:
    t+=1
    time.sleep(.01)
    #compute positions, velocities, orientation angles and angular velocities
    xbodyprv=xbody
    orEUprv=orEU
    xbody,orQT = p.getBasePositionAndOrientation(bodyId)
    vxbody=(xbody[0]-xbodyprv[0])/0.01
    vybody=(xbody[1]-xbodyprv[1])/0.01
    vzbody=(xbody[2]-xbodyprv[2])/0.01
    orEU=p.getEulerFromQuaternion(orQT)
    omgx=(orEU[0]-orEUprv[0])/0.01
    omgy=(orEU[1]-orEUprv[1])/0.01

    xfoot,dum = p.getBasePositionAndOrientation(footId)
    #decompress condition: bottom of the stance phase is reached
    if vzbody>0 and decomprPhase==0:
        decomprPhase=1
        tstr=t
    #lift off condition: foot is off the ground
    if xfoot[2]>0.105 and decomprPhase==1:
        decomprPhase=0

    if decomprPhase==1:
        #decompressing: PD control on orientation of body during stance
        #orDesEU=p.getEulerFromQuaternion(orDes)
        #orDes = p.getQuaternionFromEuler(orDesEU + np.array([-0.07*omgx-0.3*orEU[0],
        #                                                     -0.07*omgy-0.3*orEU[1],0.0]))
        if ((t-tstr)<8):  #trust for a small time interval (increased spring force)
            p.changeConstraint(cid,pivot,jointChildFrameOrientation=orDes, maxForce=60)
        else:
            p.changeConstraint(cid,pivot,jointChildFrameOrientation=orDes, maxForce=30)
    else:
        #flight and compression: Reposition foot for next landing based on body horizontal velocity and orientation
        if (xfoot[2]>0.105):
            orDes = p.getQuaternionFromEuler(
                    [+0.2*(-(vybody-0.0)*np.cos(orEU[2])+(vxbody-0.0)*np.sin(orEU[2])) + orEU[0],
                     +0.2*( (vxbody-0.0)*np.cos(orEU[2])+(vybody-0.0)*np.sin(orEU[2])) + orEU[1], 0.0])
        p.changeConstraint(cid,pivot,jointChildFrameOrientation=orDes, maxForce=30)
p.removeConstraint(cid)
#p.disconnect()






#t0=time.time()
#t=time.time()
#while ((t-t0)<1):
#    t=time.time()



#print(cid)
#print(p.getConstraintUniqueId(0))
#prev=[0,0,1]

#orn = p.getQuaternionFromEuler([0,0,0])
#orEU=[0,0,0]

#p.changeDynamics(block,1,lateralFriction=0,angularDamping=0.000,linearDamping=0.000,spinningFriction=0.0,rollingFriction=0)
#p.changeDynamics(footId,-1,lateralFriction=1)

    #sep=dum[2]-dum2[2]
    #print(up)
    #if sep<0.75 and up==0:
    #    up=1
    #if sep>0.95 and up==1:
    #    up=0
