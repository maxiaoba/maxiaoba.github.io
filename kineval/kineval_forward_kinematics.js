
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms();
}

kineval.buildFKTransforms = function buildFKTransforms(){
	kineval.traverseFKBase();
}

kineval.traverseFKBase = function traverseFKBase(){

	var mat = [];
	var x=robot.origin.xyz[0],y=robot.origin.xyz[1],z=robot.origin.xyz[2];
	var roll=robot.origin.rpy[0],pitch=robot.origin.rpy[1],yaw=robot.origin.rpy[2];
	mat=generate_translation_matrix(x,y,z);
	mat=matrix_multiply(mat,generate_rotation_matrix_X(roll));
	mat=matrix_multiply(mat,generate_rotation_matrix_Y(pitch));
	mat=matrix_multiply(mat,generate_rotation_matrix_Z(yaw));
	robot.links[robot.base].xform=mat;
	robot_heading=matrix_multiply(mat,[[0],[0],[1],[1]]);
	robot_lateral=matrix_multiply(mat,[[1],[0],[0],[1]]);
	var a;
	if(typeof robot.links[robot.base].children==='undefined') return;
	for(a=0;a<robot.links[robot.base].children.length;a++){
		kineval.traverseFKJoint(robot.links[robot.base].children[a]);
	}
}

kineval.traverseFKJoint = function traverseFKJoint(a){
	var mat = [];
	var x=robot.joints[a].origin.xyz[0],y=robot.joints[a].origin.xyz[1],z=robot.joints[a].origin.xyz[2];
	var roll=robot.joints[a].origin.rpy[0],pitch=robot.joints[a].origin.rpy[1],yaw=robot.joints[a].origin.rpy[2];
	mat=generate_translation_matrix(x,y,z);
	mat=matrix_multiply(mat,generate_rotation_matrix_X(roll));
	mat=matrix_multiply(mat,generate_rotation_matrix_Y(pitch));
	mat=matrix_multiply(mat,generate_rotation_matrix_Z(yaw));
	mat=matrix_multiply(robot.links[robot.joints[a].parent].xform,mat);
	quaternion=quaternion_from_axisangle(robot.joints[a].axis,robot.joints[a].angle);
        //textbar.innerHTML +=quaternion[0]+''+quaternion[1]+''+quaternion[2]+''+quaternion[3];
	quaternion=quaternion_normalize(quaternion);
        //textbar.innerHTML +=quaternion[0]+''+quaternion[1]+''+quaternion[2]+''+quaternion[3];
	rotmat=quaternion_to_rotation_matrix(quaternion);
	mat=matrix_multiply(mat,rotmat);
	robot.joints[a].xform=mat;
	kineval.traverseFKLink(robot.joints[a].child);
}

kineval.traverseFKLink = function traverseFKLink(b){
	robot.links[b].xform=robot.joints[robot.links[b].parent].xform;
	var a;
	if(typeof robot.links[b].children==='undefined') return;
	for(a=0;a<robot.links[b].children.length;a++){
		kineval.traverseFKJoint(robot.links[b].children[a]);
	}
}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    //   if (robot.links_geom_imported) {
    //       var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));

