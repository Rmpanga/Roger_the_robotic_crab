/*********************************************************************
 *  File:         4D_math.c
 *  Description:  some procedures for doing 4D vector-matrix algebra
 *  Author:       Rod Grupen
 *  Date:         2-28-89
 *********************************************************************/

void copy_matrix(t1,t2)                  /* copy t1 into t2 */
double t1[4][4],t2[4][4];
{
  int i,j;
  for (i=0; i<4; ++i) {
    for (j=0; j<4; ++j) {
      t2[i][j] = t1[i][j];
    }
  }
}

// only for 4D matrices
double determinant(x)
double x[4][4];
{
  double det;

  det = 
      x[0][0]*(x[1][1]*x[2][2]*x[3][3] + x[1][2]*x[2][3]*x[3][1] + 
	       x[1][3]*x[2][1]*x[3][2] - x[3][1]*x[2][2]*x[1][3] - 
	       x[3][2]*x[2][3]*x[1][1] - x[3][3]*x[2][1]*x[1][2])
    - x[0][1]*(x[1][0]*x[2][2]*x[3][3] + x[1][2]*x[2][3]*x[3][0] +
	       x[1][3]*x[2][0]*x[3][2] - x[3][0]*x[2][2]*x[1][3] -
	       x[3][2]*x[2][3]*x[1][0] - x[3][3]*x[2][0]*x[1][2])
    + x[0][2]*(x[1][0]*x[2][1]*x[3][3] + x[1][1]*x[2][3]*x[3][0] +
	       x[1][3]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[1][3] -
	       x[3][1]*x[2][3]*x[1][0] - x[3][3]*x[2][0]*x[1][1])
    - x[0][3]*(x[1][0]*x[2][1]*x[3][2] + x[1][1]*x[2][2]*x[3][0] +
	       x[1][2]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[1][2] -
	       x[3][1]*x[2][2]*x[1][0] - x[3][2]*x[2][0]*x[1][1]);


  return(det);
}
  
void cofactors(x, cof)
double x[4][4], cof[4][4];
{
  int i,j;
  double minor[3][3];

  cof[0][0] = x[1][1]*x[2][2]*x[3][3] + x[1][2]*x[2][3]*x[3][1] +
              x[1][3]*x[2][1]*x[3][2] - x[3][1]*x[2][2]*x[1][3] -
              x[3][2]*x[2][3]*x[1][1] - x[3][3]*x[2][1]*x[1][2];
  cof[0][1] = -1.0*(x[1][0]*x[2][2]*x[3][3] + x[1][2]*x[2][3]*x[3][0] +
		    x[1][3]*x[2][0]*x[3][2] - x[3][0]*x[2][2]*x[1][3] -
		    x[3][2]*x[2][3]*x[1][0] - x[3][3]*x[2][0]*x[1][2]);
  cof[0][2] = x[1][0]*x[2][1]*x[3][3] + x[1][1]*x[2][3]*x[3][0] +
              x[1][3]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[1][3] -
              x[3][1]*x[2][3]*x[1][0] - x[3][3]*x[2][0]*x[1][1];
  cof[0][3] = -1.0*(x[1][0]*x[2][1]*x[3][2] + x[1][1]*x[2][2]*x[3][0] +
		    x[1][2]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[1][2] -
		    x[3][1]*x[2][2]*x[1][0] - x[3][2]*x[2][0]*x[1][1]);

  cof[1][0] = -1.0*( x[0][1]*x[2][2]*x[3][3] + x[0][2]*x[2][3]*x[3][1] +
		     x[0][3]*x[2][1]*x[3][2] - x[3][1]*x[2][2]*x[0][3] -
		     x[3][2]*x[2][3]*x[0][1] - x[3][3]*x[2][1]*x[0][2]);
  cof[1][1] = x[0][0]*x[2][2]*x[3][3] + x[0][2]*x[2][3]*x[3][0] +
              x[0][3]*x[2][0]*x[3][2] - x[3][0]*x[2][2]*x[0][3] -
              x[3][2]*x[2][3]*x[0][0] - x[3][3]*x[2][0]*x[0][2];
  cof[1][2] = -1.0*(x[0][0]*x[2][1]*x[3][3] + x[0][1]*x[2][3]*x[3][0] +
		    x[0][3]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[0][3] -
		    x[3][1]*x[2][3]*x[0][0] - x[3][3]*x[2][0]*x[0][1]);
  cof[1][3] = x[0][0]*x[2][1]*x[3][2] + x[0][1]*x[2][2]*x[3][0] +
              x[0][2]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[0][2] -
              x[3][1]*x[2][2]*x[0][0] - x[3][2]*x[2][0]*x[0][1];

  cof[2][0] = x[0][1]*x[1][2]*x[3][3] + x[0][2]*x[1][3]*x[3][1] +
              x[0][3]*x[1][1]*x[3][2] - x[3][1]*x[1][2]*x[0][3] -
              x[3][2]*x[1][3]*x[0][1] - x[3][3]*x[1][1]*x[0][2];
  cof[2][1] = -1.0*(x[0][0]*x[1][2]*x[3][3] + x[0][2]*x[1][3]*x[3][0] +
		    x[0][3]*x[1][0]*x[3][2] - x[3][0]*x[1][2]*x[0][3] -
		    x[3][2]*x[1][3]*x[0][0] - x[3][3]*x[1][0]*x[0][2]);
  cof[2][2] = x[0][0]*x[1][1]*x[3][3] + x[0][1]*x[1][3]*x[3][0] +
              x[0][3]*x[1][0]*x[3][1] - x[3][0]*x[1][1]*x[0][3] -
              x[3][1]*x[1][3]*x[0][0] - x[3][3]*x[1][0]*x[0][1];
  cof[2][3] = -1.0*(x[0][0]*x[1][1]*x[3][2] + x[0][1]*x[1][2]*x[3][0] +
		    x[0][2]*x[1][0]*x[3][1] - x[3][0]*x[1][1]*x[0][2] -
		    x[3][1]*x[1][2]*x[0][0] - x[3][2]*x[1][0]*x[0][1]);

  cof[3][0] = -1.0*(x[0][1]*x[1][2]*x[2][3] + x[0][2]*x[1][3]*x[2][1] +
		    x[0][3]*x[1][1]*x[2][2] - x[2][1]*x[1][2]*x[0][3] -
		    x[2][2]*x[1][3]*x[0][1] - x[2][3]*x[1][1]*x[0][2]);
  cof[3][1] = x[0][0]*x[1][2]*x[2][3] + x[0][2]*x[1][3]*x[2][0] +
              x[0][3]*x[1][0]*x[2][2] - x[2][0]*x[1][2]*x[0][3] -
              x[2][2]*x[1][3]*x[0][0] - x[2][3]*x[1][0]*x[0][2];
  cof[3][2] = -1.0*(x[0][0]*x[1][1]*x[2][3] + x[0][1]*x[1][3]*x[2][0] +
		    x[0][3]*x[1][0]*x[2][1] - x[2][0]*x[1][1]*x[0][3] -
		    x[2][1]*x[1][3]*x[0][0] - x[2][3]*x[1][0]*x[0][1]);
  cof[3][3] = x[0][0]*x[1][1]*x[2][2] + x[0][1]*x[1][2]*x[2][0] +
              x[0][2]*x[1][0]*x[2][1] - x[2][0]*x[1][1]*x[0][2] -
              x[2][1]*x[1][2]*x[0][0] - x[2][2]*x[1][0]*x[0][1];
}

/*********************************************************************/
void invert_matrix(in, out)
double in[4][4], out[4][4];
{
  double cof[4][4], det;
  double determinant();
  int i,j;
  //  double ans[4][4];

  // 1/det [cof]T

  det = determinant(in);
  //  printf("det=%lf\n", det);

  cofactors(in, cof);

  for (i=0; i<4; ++i) {
    for (j=0; j<4; ++j) {
      out[i][j] = cof[j][i]/det;
    }
  }
}




/*********************************************************************/
void matXmat(t1,t2,result)
double t1[4][4],t2[4][4],result[4][4];
{
  int i,j,k;
  for (i=0; i<4; ++i) {
    for (j=0; j<4; ++j) {
      result[i][j] = 0.0;
      for (k=0; k<4; ++k) {
	result[i][j] += t1[i][k] * t2[k][j];
      }
    }
  }
}

/*********************************************************************/
void matXvec(mat, vec, out)
double mat[4][4], vec[4], out[4];
{
  int i,j;
  double sum;
  for (i=0; i<4; ++i) {
    sum = 0.0;
    for (j=0; j<4; ++j) {
      sum += mat[i][j] * vec[j];
    }
    out[i] = sum;
  }
}

/*********************************************************************/
void matrix_sum(in1, in2, out)
double in1[4][4], in2[4][4], out[4][4];
{
  int i,j;
  for (i=0; i<4; ++i) {
    for (j=0; j<4; ++j) {
      out[i][j] = in1[i][j] + in2[i][j];
    }
  }
}

/*********************************************************************/
void matrix_diff(in1, in2, out)
double in1[4][4], in2[4][4], out[4][4];
{
  int i,j;
  for (i=0; i<4; ++i) {
    for (j=0; j<4; ++j) {
      out[i][j] = in1[i][j] - in2[i][j];
    }
  }
}

/*********************************************************************/
void vector_sum(in1, in2, out)
double in1[4], in2[4], out[4];
{
  int i;
  for (i=0; i<4; ++i) {
      out[i] = in1[i] + in2[i];
  }
}

/*********************************************************************/
void vector_diff(in1, in2, out)
double in1[4], in2[4], out[4];
{
  int i;
  for (i=0; i<4; ++i) {
      out[i] = in1[i] - in2[i];
  }
}

/*********************************************************************/
void matrix_transpose(in, out)
double in[4][4], out[4][4];
{
  int i,j;
  for (i=0; i<4; ++i) {
    for (j=0; j<4; ++j) {
      out[j][i] = in[i][j];
    }
  }
}

/******************* HOMOGENEOUS TRANSFORMS **************************/
void inv_transform(in, out)
double in[4][4], out[4][4];
{
  int i,j;
  for (i=0; i<3; ++i) {
    for (j=0; j<3; ++j) {
      out[i][j] = in[j][i];
    }
  }
  out[3][0] = out[3][1] = out[3][2] = 0.0; out[3][3] = 1.0;

  out[0][3] = -in[0][3]*in[0][0] - in[1][3]*in[1][0] - in[2][3]*in[2][0];
  out[1][3] = -in[0][3]*in[0][1] - in[1][3]*in[1][1] - in[2][3]*in[2][1];
  out[2][3] = -in[0][3]*in[0][2] - in[1][3]*in[1][2] - in[2][3]*in[2][2];
}



