//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply
function quaternion_from_axisangle(axis,angle){	
	var q=[Math.cos(angle/2),axis[0]*Math.sin(angle/2),axis[1]*Math.sin(angle/2),axis[2]*Math.sin(angle/2)];
	return q;
}

function quaternion_normalize(q){
	if(q[0]===1) return q;
	var norm=Math.sqrt((q[1]*q[1]+q[2]*q[2]+q[3]*q[3])/(1-q[0]*q[0]));
	var q_n=[q[0],q[1]/norm,q[2]/norm,q[3]/norm];
	return q_n;
}

function quaternion_to_rotation_matrix(q){
	var mat=[[1-2*(q[2]*q[2]+q[3]*q[3]),2*(q[1]*q[2]-q[0]*q[3]),2*(q[0]*q[2]+q[1]*q[3]),0],
		 [2*(q[1]*q[2]+q[0]*q[3]),1-2*(q[1]*q[1]+q[3]*q[3]),2*(q[2]*q[3]-q[0]*q[1]),0],
		 [2*(q[1]*q[3]-q[0]*q[2]),2*(q[2]*q[3]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]),0],
		 [0,0,0,1]];
	return mat;
}

function quaternion_multiply(q1,q2){
	var q3=[q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3],
		q1[0]*q2[1]+q2[0]*q1[1]+q1[2]*q2[3]-q1[3]*q2[2],
		q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1],
		q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]];
	return q3;
}
