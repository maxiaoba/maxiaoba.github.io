//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}

function matrix_multiply(m1,m2) {

    var mat = [];
    var i,j;
    	for (i=0;i<m1.length;i++) { // for each row of m1
        	mat[i] = [];
        	for (j=0;j<m2[0].length;j++) { // for each column of m2
           		mat[i][j] = 0;
				for (k=0;k<m2.length;k++){
					mat[i][j]+=m1[i][k]*m2[k][j];
				}
        	}
    	}
    return mat;
}

function matrix_transpose(m1) {
    var mat = [];
    var i,j;
    for (i=0;i<m1[0].length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1.length;j++) { // for each column of m1
            mat[i][j] = m1[j][i];
        }
    }
    return mat;
}

function matrix_pseudoinverse(m1) {
		textbar.innerHTML +="inside matrix_pseudoinverse: ";
	var mat=[];
	var m1T=matrix_transpose(m1);
	textbar.innerHTML +="m1T: "+m1T[0];
	var m2=matrix_multiply(m1,m1T);
	textbar.innerHTML +="m2: "+m2[0]+' '+m2[1]+' '+m2[2];
	var m3=numeric.inv(matrix_multiply(m1,m1T));
	textbar.innerHTML +="m3: "+m3[0];
	mat=matrix_multiply(m1T,numeric.inv(matrix_multiply(m1,m1T)));
	textbar.innerHTML +="mat: "+mat[0];
	return mat;
}

function matrix_invert_affine(m1) {
}

function vector_normalize(v1) {
    var vec = [];
    var i;
    var length=0;
    for (i=0;i<v1.length;i++) {
	length+=v1[i]*v1[i];
    }
    length=Math.sqrt(length);
    for (i=0;i<v1.length;i++) {
	vec[i]=v1[i]/length;
    }
    return vec;
}

function vector_cross(v1,v2) {
	//only for v of length 3
    var vec = [];
    vec[0]=v1[1]*v2[2]-v1[2]*v2[1];
    vec[1]=-v1[0]*v2[2]+v1[2]*v2[0];
    vec[2]=v1[0]*v2[1]-v1[1]*v2[0];
    return vec;
}

function generate_identity(n){
    var mat = [];
    var i,j;
    for (i=0;i<n;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<n;j++) { // for each column of m1
        	if(i==j) mat[i][j]=1;
		else mat[i][j]=0;
        }
    }
    return mat;
}

function generate_translation_matrix(x,y,z){
    var mat=[];
    mat=generate_identity(4);
    mat[0][3]=x;
    mat[1][3]=y;
    mat[2][3]=z;
    return mat;
}

function generate_rotation_matrix_X(x){
    var mat=[];
    mat=generate_identity(4);
    mat[1][1]=Math.cos(x);
    mat[1][2]=-Math.sin(x);
    mat[2][1]=Math.sin(x);
    mat[2][2]=Math.cos(x);
    return mat;
}

function generate_rotation_matrix_Y(y){
    var mat=[];
    mat=generate_identity(4);
    mat[0][0]=Math.cos(y);
    mat[0][2]=Math.sin(y);
    mat[2][0]=-Math.sin(y);
    mat[2][2]=Math.cos(y);
    return mat;
}

function generate_rotation_matrix_Z(z){
    var mat=[];
    mat=generate_identity(4);
    mat[0][0]=Math.cos(z);
    mat[0][1]=-Math.sin(z);
    mat[1][0]=Math.sin(z);
    mat[1][1]=Math.cos(z);
    return mat;
}
    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z

