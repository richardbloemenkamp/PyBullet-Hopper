# -*- coding: utf-8 -*-
"""
Created on 2019
@author: Richard Bloemenkamp
"""
import pybullet as p
import time
import numpy as np

#p.disconnect()
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
p.createCollisionShape(p.GEOM_PLANE)
plId=p.createMultiBody(0,0)
p.resetDebugVisualizerCamera( cameraDistance=4, cameraYaw=10, cameraPitch=-20, 
                              cameraTargetPosition=[0.0, 0.0, 0.25])

sh_colFoot = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01,0.01,0.1])
#sh_colFoot = p.createCollisionShape(p.GEOM_SPHERE,radius=0.08)
#sh_colBody = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.1,0.1,0.1])
sh_colBody = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.2,0.2,0.1])
sh_colBody = p.createCollisionShape(p.GEOM_CYLINDER,radius=0.13, height=0.6)
sh_visBody = p.createVisualShape(p.GEOM_CYLINDER,radius=0.13, length=0.6, rgbaColor=[0.4,0.4,0.5,1])
sh_colPx = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01,0.1,0.1])
sh_colPy = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.1,0.01,0.1])

bodyId=p.createMultiBody(baseMass=1,baseCollisionShapeIndex = sh_colBody, baseVisualShapeIndex = sh_visBody,
                        basePosition = [0,0,1.5],baseOrientation=[0,0,0,1])
footId=p.createMultiBody(baseMass=1,baseCollisionShapeIndex = sh_colFoot,
                        basePosition = [0,0,0.5],baseOrientation=[0,0,0,1])
#4 inertia increasing plates
cubeId3=p.createMultiBody(baseMass=3,baseCollisionShapeIndex = sh_colPx,
                        basePosition = [-0.5,0,1.5],baseOrientation=[0,0,0,1])
cubeId4=p.createMultiBody(baseMass=3,baseCollisionShapeIndex = sh_colPx,
                        basePosition = [0.5,0,1.5],baseOrientation=[0,0,0,1])
cubeId5=p.createMultiBody(baseMass=3,baseCollisionShapeIndex = sh_colPy,
                        basePosition = [0,-0.5,1.5],baseOrientation=[0,0,0,1])
cubeId6=p.createMultiBody(baseMass=3,baseCollisionShapeIndex = sh_colPy,
                        basePosition = [0,0.5,1.5],baseOrientation=[0,0,0,1])


#Scenery e.g. an inclined box
boxHalfLength = 2.5
boxHalfWidth = 2.5
boxHalfHeight = 0.2
sh_colBox = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
sh_visBox = p.createVisualShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight], rgbaColor=[0,0,0,1])

block=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [-2,0,-0.1],baseOrientation=[0.0,0.1,0.0,1])
sth=0.15
block2=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox, baseVisualShapeIndex = sh_visBox,
                        basePosition = [5.75,0.15,-0.2+1*sth],baseOrientation=[0.0,0.0,0.0,1])
block3=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [5.75+0.33,0,-0.2+2*sth],baseOrientation=[0.0,0.0,0.0,1])
block4=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [5.75+0.66,0.2,-0.2+3*sth],baseOrientation=[0.1,0.0,0.0,1])
block5=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [5.75+0.99,0.1,-0.2+4*sth],baseOrientation=[0.0,-0.1,0.0,1])

box11l=0.5
box11w=0.5
box11h=0.1
sh_box11 = p.createCollisionShape(p.GEOM_BOX,halfExtents=[box11l,box11w,box11h])
sth=0.15
for k in range(10):
    p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_box11,
                       basePosition = [3+0.4*k,-1+k/200,k*sth],baseOrientation=[0.0,0.0,0.0,1])
p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_box11,
                  basePosition = [3+0.4*10,-1,k*sth+0.01],baseOrientation=[0.0,0.0,0.0,1])
for k in range(10):
    p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_box11,
                       basePosition = [3+0.4*10+k/200,-0.5+0.4*k,(k+10)*sth],baseOrientation=[0.0,0.0,0.0,1])
box14_1l=7
box14_1w=0.75
box11h=0.1
sh_box14_1 = p.createCollisionShape(p.GEOM_BOX,halfExtents=[box14_1l,box14_1w,box11h])
box14_1=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_box14_1,
                  basePosition = [-0.3,3.1,1.4],baseOrientation=[0.0,-0.1,0.0,1])



p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
#make to plane less slippery
p.changeDynamics(plId,-1,lateralFriction=10)
p.changeDynamics(block5,-1,lateralFriction=10)
p.changeDynamics(box14_1,-1,lateralFriction=10)

#connect foot to body
cid = p.createConstraint(bodyId,-1,footId,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,1])
#connect plates to body
cid2 = p.createConstraint(bodyId,-1,cubeId3,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0.25,0,0])
cid3 = p.createConstraint(bodyId,-1,cubeId4,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[-0.25,0,0])
cid4 = p.createConstraint(bodyId,-1,cubeId5,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0.25,0])
cid5 = p.createConstraint(bodyId,-1,cubeId6,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,-0.25,0])

#init
pivot=[0,0,0,1]
decomprPhase=0
xbody,orQT = p.getBasePositionAndOrientation(bodyId)
orDes=orQT
orEU=p.getEulerFromQuaternion(orQT)
t=0
tstr=0
vx=0
vy=0
zgnd=0
jmp=0

xgl=7
ygl=-1
while 1:
    p.resetDebugVisualizerCamera( cameraDistance=6, cameraYaw=-130+t/10, cameraPitch=-60, 
                              cameraTargetPosition=[xbody[0], xbody[1], 0.25])

    t+=1
    time.sleep(.01)
    keys = p.getKeyboardEvents()
    if keys.get(65297): #Up
        vx+=0.002
    if keys.get(65298): #Down
        vx-=0.002
    if keys.get(65296): #Right
        vy-=0.002
    if keys.get(65295): #Left
        vy+=0.002
    if keys.get(97):   #A
        if jmp==0:        
            vx*=3
            vy*=3
        jmp=1
    #print(vx,vy,zgnd)
    #print(xbody[0],xbody[1],ygl)

    if xbody[0]<xgl:
        vx=+0.08
    else:
        vx=-0.08
        if xgl==-8 and xbody[0]<6.5:
            vx=-0.04
        #xgl=7
        ygl=3.1
    if xbody[1]<ygl:
        vy=+0.08
        if xgl==-8 and xbody[0]<6.5:
            vy=+0.04
    else:
        vy=-0.08
        if ygl>3:
            xgl=-8
        if xgl==-8 and xbody[0]<6.5:
            vy=-0.04
            
        
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
        zgnd=xfoot[2]+0.05
    #lift off condition: foot is off the ground
    if xfoot[2]-zgnd>0.105 and decomprPhase==1:
        decomprPhase=2
        if jmp==1:
            vx=vx/3
            vy=vy/3
        jmp=0
    if xfoot[2]-zgnd<0.105 and decomprPhase==2:
        decomprPhase=0
        

    if decomprPhase==1:
        #decompressing: PD control on orientation of body during stance
        orDesEU=p.getEulerFromQuaternion(orDes)
        orDes = p.getQuaternionFromEuler(orDesEU + np.array([-0.07*omgx-0.3*orEU[0]   -0.15*(-(vybody-vy)*np.cos(orEU[2])+(vxbody-vx)*np.sin(orEU[2])),
                                                             -0.07*omgy-0.3*orEU[1]   -0.15*( (vxbody-vx)*np.cos(orEU[2])+(vybody-vy)*np.sin(orEU[2])),0.0]))
        if ((t-tstr)<8 and jmp==1):  #trust for a small time interval (increased spring force)
            p.changeConstraint(cid,pivot,jointChildFrameOrientation=orDes, maxForce=1300)
        elif ((t-tstr)<8):  #trust for a small time interval (increased spring force)
            p.changeConstraint(cid,pivot,jointChildFrameOrientation=orDes, maxForce=600)
        else:
            p.changeConstraint(cid,pivot,jointChildFrameOrientation=orDes, maxForce=300)
    else:
        #flight and compression: Reposition foot for next landing based on body horizontal velocity and orientation
        if (xfoot[2]-zgnd>0.105):
            orDes = p.getQuaternionFromEuler(
                    [+0.15*(-(vybody-vy)*np.cos(orEU[2])+(vxbody-vx)*np.sin(orEU[2])) + orEU[0],
                     +0.15*( (vxbody-vx)*np.cos(orEU[2])+(vybody-vy)*np.sin(orEU[2])) + orEU[1], 0.0])
        p.changeConstraint(cid,pivot,jointChildFrameOrientation=orDes, maxForce=300)
            
p.removeConstraint(cid)
#p.disconnect()

