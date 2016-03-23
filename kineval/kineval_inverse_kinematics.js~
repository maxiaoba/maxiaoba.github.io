
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and targetc
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
		var deltaX=[];
		var endeffector_position_world=[];
		endeffector_position_world=matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_position_local);
		var i;
		for(i=0;i<3;i++){
			deltaX[i]=[];
			deltaX[i][0]=endeffector_target_world.position[i][0]-endeffector_position_world[i];
		}
	if(kineval.params.ik_pseudoinverse){
		JJT=[[0,0,0],[0,0,0],[0,0,0]];
		kineval.findJJT(JJT,endeffector_joint,endeffector_position_world);
		textbar.innerHTML +="JJT: "+JJT[0]+' '+JJT[1]+' '+JJT[2]+'<br>';
		var JJTv=numeric.inv(JJT);
		textbar.innerHTML +="JJTv: "+JJTv[0]+' '+JJTv[1]+' '+JJTv[2]+'<br>';
		kineval.traverseIKJointP(JJTv,endeffector_joint,deltaX,endeffector_position_world);
	}
	else{
		kineval.traverseIKJoint(endeffector_joint,deltaX,endeffector_position_world);
	}	
}

kineval.traverseIKJoint = function traverseIKJoint(joint,deltaX,endeffector_position_world){
	//textbar.innerHTML +=joint;
	//var z=robot.joints[joint].axis;
	var z4=matrix_multiply(robot.joints[joint].xform,[[robot.joints[joint].axis[0]],[robot.joints[joint].axis[1]],[robot.joints[joint].axis[2]],[1]]);
	var z=[];
	var i;
	for(i=0;i<3;i++) z[i]=z4[i];
	var o2=endeffector_position_world;
	var o1Local=[[0],[0],[0],[1]];
	var o1=matrix_multiply(robot.joints[joint].xform,o1Local);
	var o2_o1=[]
	var i;
	for(i=0;i<3;i++){
		o2_o1[i]=o2[i]-o1[i];
	}
	
	var Jvdummy=vector_cross(z,o2_o1);
	var Jv=[];
	for(i=0;i<3;i++){
		Jv[i]=[];
		Jv[i][0]=Jvdummy[i];
	}
	textbar.innerHTML +='<br>'+joint+": Jv: "+Jv[0][0]+' '+Jv[1][0]+' '+Jv[2][0]+'<br>';
	var JvT=matrix_transpose(Jv);
	var alpha=kineval.params.ik_steplength;
	robot.joints[joint].control=alpha*matrix_multiply(JvT,deltaX);
	if(typeof robot.links[robot.joints[joint].parent].parent==='undefined') return;
	var parentJoint=robot.links[robot.joints[joint].parent].parent;
	kineval.traverseIKJoint(parentJoint,deltaX,endeffector_position_world);
}

kineval.findJJT = function findJJT(JJT,joint,endeffector_position_world){
	//textbar.innerHTML +=joint;
	//var z=robot.joints[joint].axis;
	var z4=matrix_multiply(robot.joints[joint].xform,[[robot.joints[joint].axis[0]],[robot.joints[joint].axis[1]],[robot.joints[joint].axis[2]],[1]]);
	var z=[];
	var i;
	for(i=0;i<3;i++) z[i]=z4[i];
	var o2=endeffector_position_world;
	var o1Local=[[0],[0],[0],[1]];
	var o1=matrix_multiply(robot.joints[joint].xform,o1Local);
	var o2_o1=[]
	var i;
	for(i=0;i<3;i++){
		o2_o1[i]=o2[i]-o1[i];
	}
	textbar.innerHTML +="<br>"+joint+": z: "+z[0]+' '+z[1]+' '+z[2]+'<br>';
	textbar.innerHTML +="o2: "+o2[0]+' '+o2[1]+' '+o2[2]+'<br>';
	textbar.innerHTML +="o1: "+o1[0]+' '+o1[1]+' '+o1[2]+'<br>';
	textbar.innerHTML +="o2_o1: "+o2_o1[0]+' '+o2_o1[1]+' '+o2_o1[2]+'<br>';
	var Jvdummy=vector_cross(z,o2_o1);
	textbar.innerHTML +="Jvdummy: "+Jvdummy[0]+' '+Jvdummy[1]+' '+Jvdummy[2]+'<br>';
	var Jv=[];
	for(i=0;i<3;i++){
		Jv[i]=[];
		Jv[i][0]=Jvdummy[i];
	}
	var JvT=matrix_transpose(Jv);
	textbar.innerHTML +="Jv: "+Jv[0][0]+' '+Jv[1][0]+' '+Jv[2][0]+'<br>';
	var JJTi=matrix_multiply(Jv,JvT);
	textbar.innerHTML +="JJTi: "+JJTi[0]+' '+JJTi[1]+' '+JJTi[2]+'<br>';	
	var j;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			JJT[i][j]+=JJTi[i][j];
		}
	}
	textbar.innerHTML +="JJT: "+JJT[0]+' '+JJT[1]+' '+JJT[2]+'<br>';
	if(typeof robot.links[robot.joints[joint].parent].parent==='undefined'){ 
		textbar.innerHTML +="JJT: "+JJT[0]+' '+JJT[1]+' '+JJT[2]+'<br>';
		return;
	}
	else{
		textbar.innerHTML +="~~~~~~~~~~~~~~~~~~~~~~~";
		var parentJoint=robot.links[robot.joints[joint].parent].parent;
		kineval.findJJT(JJT,parentJoint,endeffector_position_world);
	}
}

kineval.traverseIKJointP = function traverseIKJointP(JJTv,joint,deltaX,endeffector_position_world){
	//textbar.innerHTML +=joint;
	//var z=robot.joints[joint].axis;
	var z4=matrix_multiply(robot.joints[joint].xform,[[robot.joints[joint].axis[0]],[robot.joints[joint].axis[1]],[robot.joints[joint].axis[2]],[1]]);
	var z=[];
	var i;
	for(i=0;i<3;i++) z[i]=z4[i];
	var o2=endeffector_position_world;
	var o1Local=[[0],[0],[0],[1]];
	var o1=matrix_multiply(robot.joints[joint].xform,o1Local);
	var o2_o1=[]
	var i;
	for(i=0;i<3;i++){
		o2_o1[i]=o2[i]-o1[i];
	}
	
	var Jvdummy=vector_cross(z,o2_o1);
	var Jv=[];
	for(i=0;i<3;i++){
		Jv[i]=[];
		Jv[i][0]=Jvdummy[i];
	}
	var JvT=matrix_transpose(Jv);
	var alpha=kineval.params.ik_steplength;
	robot.joints[joint].control=alpha*matrix_multiply(matrix_multiply(JvT,JJTv),deltaX);
	if(typeof robot.links[robot.joints[joint].parent].parent==='undefined') return;
	var parentJoint=robot.links[robot.joints[joint].parent].parent;
	kineval.traverseIKJointP(JJTv,parentJoint,deltaX,endeffector_position_world);
}

