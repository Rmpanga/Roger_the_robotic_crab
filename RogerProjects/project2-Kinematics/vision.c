/*************************************************************************/
/* File:        vision.c                                                 */
/* Description: User project #2                                          */
/* Date:        5-1-13                                                   */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
// #include "Xkw/Xkw.h"

#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

#define SIGMA_OBS     0.01    // the scale of the observation covariance
#define SIGMA_PROCESS 10.0 // noise in the forward/process model   
#define OBS_DT        0.001 // same as servo and render rate

void construct_wTb(), matrix_times_vector(), matrix_transpose22(), 
     matXmat2222();

void stereoJJT(roger, ur, ul, JJT)
    Robot * roger;
    double ur, ul, JJT[2][2]; /* observation covariance only assigns 2x2 */
{
    double coeff, J00, J01, J10, J11;
    double gl, gr, lambdaL;

    gl = roger->eye_theta[LEFT] + atan2((ul - 63.5), 64.0);
    gr = roger->eye_theta[RIGHT] + atan2((ur - 63.5), 64.0);

    if ((gr - gl) == 0.0) lambdaL = 20.0;
    else lambdaL = 2.0*BASELINE*cos(gr) / sin(gr - gl);

    // catch case where eyes are parallel and same pixels on both eyes (infinite
    // distance)                             
    if ((gr - gl) == 0.0) coeff = 20.0;
    else coeff = 2.0*BASELINE / SQR(sin(gr-gl));

    // calculate x,y coordinate in base frame
    //J00 = coeff * sin(gr)*cos(gr);
    //J01 = -coeff * sin(gl)*cos(gl);
    //J10 = coeff * SQR(sin(gr));
    //J11 = -coeff * SQR(sin(gl));

    J00 = coeff * cos(gr)*cos(gr);
    J01 = -coeff * cos(gl)*cos(gl);
    J10 = coeff * sin(gr)*cos(gr);
    J11 = -coeff * sin(gl)*cos(gl);

    //  printf(" ** J00=%6.4lf J01=%6.4lf J10=%6.4lf J11=%6.4lf\n",
    //         J00,J01,J10,J11);
    JJT[0][0] = SQR(J00) + SQR(J01);
    JJT[0][1] = JJT[1][0] = J00*J10 + J01*J11;
    JJT[1][1] = SQR(J10) + SQR(J11);
    //  printf(" ** JJT00=%6.4lf JJT01=%6.4lf JJT10=%6.4lf JJT11=%6.4lf\n",
    //     JJT[0][0], JJT[0][1], JJT[1][0], JJT[1][1]);       
}

void stereo_triangulate() { }

// triangulate the position of the red ball, transform from the base frame to
// to world frame and write into Observation obs
int stereo_observation(roger, obs)
    Robot * roger;
    Observation * obs;
{
    double ur, ul;
    double wTb[4][4], bTw[4][4], wRb[2][2], bRw[2][2], ref_b[4], ref_w[4];
    double lambdaL, gammaL, gammaR; // stereoJJt[2][2];
    double mat_22[2][2], cov_b[2][2];
    int compute_average_red_pixel();
    int no_red_blue_transitions = TRUE;;
    int check_stereo_FOV();
    /************************************************************************/
    // PROJECT2: triangulate to localize the red ball in the base frame
    //           convert to world frame, and
    //           write into Observation "obs"
    //
    // build a cascade of filters to identify "valid" stereo observations
    // is there "red" on both image planes?
    // are there any red-blue transitions?
    // are there any FOV issues?
    if (compute_average_red_pixel(roger, &ur, &ul) == TRUE)
    { 
            //      (no_red_blue_transitions) && (TRUE) ) { (no_red_blue_transitions) && (check_stereo_FOV() == TRUE) ) { 
      // get angle from base to object
      gammaL = roger->eye_theta[LEFT] + atan2((ul - 63.5), 64.0);
      gammaR = roger->eye_theta[RIGHT] + atan2((ur - 63.5), 64.0);

      // lambda_L: catch case where eyes are parallel and same pixels on both
      // eyes (infinite distance)
      if ((gammaR - gammaL) == 0.0) lambdaL = 20.0;
      else lambdaL = 2.0*BASELINE*cos(gammaR) / sin(gammaR - gammaL);
      // calculate x,y coordinate in base frame
      ref_b[X] = lambdaL * cos(gammaL);
      ref_b[Y] = BASELINE + lambdaL * sin(gammaL);
      ref_b[2] = 0.0;
      ref_b[3] = 1.0;
      // convert into world frame
      construct_wTb(roger->base_position, wTb);
      wRb[0][0] = wTb[0][0]; wRb[0][1] = wTb[0][1];
      wRb[1][0] = wTb[1][0]; wRb[1][1] = wTb[1][1];

      matrix_times_vector(wTb, ref_b, ref_w);

      obs->pos[X] = ref_w[X];
      obs->pos[Y] = ref_w[Y];

      // compute observation cov (JJT) 
      stereoJJT(roger, ur, ul, cov_b);
      // and rotate it into world coordinates ... [cov]_w = wRb [cov]_b wRb^T
      matrix_transpose22(wRb, bRw);
      matXmat2222(wRb, cov_b, mat_22);
      matXmat2222(mat_22, bRw, obs->cov);

      //obs->cov[0][0] = cov_b[0][0];
      //obs->cov[0][1] = cov_b[0][1];
      //obs->cov[1][0] = cov_b[1][0];
      //obs->cov[1][1] = cov_b[1][1];

      obs->cov[0][0] *= SQR(SIGMA_OBS);
      obs->cov[0][1] *= SQR(SIGMA_OBS);
      obs->cov[1][0] *= SQR(SIGMA_OBS);
      obs->cov[1][1] *= SQR(SIGMA_OBS);


      return(TRUE);
    }
    else return(FALSE);
}

/*************************************************************************/
// compute the (0<=position<128) of the average red pixel on both image planes
// if red is detected on both images:
//       write the index of the mean pixel into ul and ur;
//       return TRUE;
// else return FALSE;
/*************************************************************************/
int compute_average_red_pixel(roger, ur, ul)
    Robot* roger;
    double *ur, *ul;
{
    int i, nr, nl;
    double r,g,b,I,m,S,H;

    nr = nl = 0;
    *ul = *ur = 0.0;

    for (i=0;i<NPIXELS;++i) {
        /* PIXEL i IN LEFT EYE */
        r = (double) roger->image[LEFT][i][RED_CHANNEL];
        g = (double) roger->image[LEFT][i][GREEN_CHANNEL];
        b = (double) roger->image[LEFT][i][BLUE_CHANNEL];

        //    I = (r + g +  b)/3.0;
        //    m = MIN(MIN(r,g),b);
        //    S = 1.0 - m/I;
        H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
            * 180.0/M_PI;                                                // degrees
        if (b>g) H=360.0-H;

        //    if (i==64) printf("LEFT: HSI=(%5.2lf %5.4lf %5.2lf)", H,S,I);

        // this is "red"
        if ( (H<=10.0) || H>=350.0) { nl += 1; *ul += i;}

        /* PIXEL i IN RIGHT EYE */
        r = (double) roger->image[RIGHT][i][RED_CHANNEL];
        g = (double) roger->image[RIGHT][i][GREEN_CHANNEL];
        b = (double) roger->image[RIGHT][i][BLUE_CHANNEL];

        //    I = (r + g + b)/3.0;
        //    m = MIN(MIN(r,g), b);
        //    S = 1.0 - m/I;
        H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
            * 180.0/M_PI;                                                // degrees
        if (b>g) H=360.0-H;

        if ( (H<=10.0) || H>=350.0) { nr += 1; *ur += i;}

        //    if (i==64) printf("    RIGHT: HSI=(%5.2lf %5.4lf %5.2lf)\n", H,S,I);

        //    printf(" nl=%d ul=%d   nr=%d ur=%d\n", nl, *ul, nr, *ur);
    }

    if (nl>0) *ul /= (double) nl;
    if (nr>0) *ur /= (double) nr;
    if ((nl>0) && (nr>0)) {
        //    printf(" ul=%6.4lf ur=%6.4lf\n", *ul, *ur);
        return(TRUE);
    }
    else return(FALSE);
}

/*************************************************************************/
// if red is detected in pixel 0 and 127 on either image:
//       return FALSE
// else  return TRUE;
/*************************************************************************/
int check_stereo_FOV(roger)
    Robot* roger;
{
    int i, p[2][2]={{FALSE, FALSE},{FALSE, FALSE}};

    double r, g, b, I, m, S, H;

    /* PIXEL 0 IN LEFT EYE */
    r = (double) roger->image[LEFT][0][RED_CHANNEL];
    g = (double) roger->image[LEFT][0][GREEN_CHANNEL];
    b = (double) roger->image[LEFT][0][BLUE_CHANNEL];

    //  I = (r + g +  b)/3.0;
    //  m = MIN(MIN(r,g),b);
    //  S = 1.0 - m/I;
    H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
        * 180.0/M_PI;                                                // degrees
    if (b>g) H=360.0-H;

    // this is "red"
    if ( (H<=10.0) || H>=350.0) { p[LEFT][0]=TRUE; }

    /* PIXEL 127 IN LEFT EYE */
    r = (double) roger->image[LEFT][127][RED_CHANNEL];
    g = (double) roger->image[LEFT][127][GREEN_CHANNEL];
    b = (double) roger->image[LEFT][127][BLUE_CHANNEL];

    //  I = (r + g +  b)/3.0;
    //  m = MIN(MIN(r,g),b);
    //  S = 1.0 - m/I;
    H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
        * 180.0/M_PI;                                                // degrees
    if (b>g) H=360.0-H;

    // this is "red"
    if ( (H<=10.0) || H>=350.0) { p[LEFT][1]=TRUE; }

    /* PIXEL 0 IN RIGHT EYE */
    r = (double) roger->image[RIGHT][0][RED_CHANNEL];
    g = (double) roger->image[RIGHT][0][GREEN_CHANNEL];
    b = (double) roger->image[RIGHT][0][BLUE_CHANNEL];

    //  I = (r + g + b)/3.0;
    //  m = MIN(MIN(r,g), b);
    //  S = 1.0 - m/I;
    H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
        * 180.0/M_PI;                                                // degrees
    if (b>g) H=360.0-H;

    if ( (H<=10.0) || H>=350.0) { p[RIGHT][0]=TRUE; }

    /* PIXEL 127 IN RIGHT EYE */
    r = (double) roger->image[RIGHT][127][RED_CHANNEL];
    g = (double) roger->image[RIGHT][127][GREEN_CHANNEL];
    b = (double) roger->image[RIGHT][127][BLUE_CHANNEL];

    //  I = (r + g + b)/3.0;
    //  m = MIN(MIN(r,g), b);
    //  S = 1.0 - m/I;
    H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
        * 180.0/M_PI;                                                // degrees
    if (b>g) H=360.0-H;

    if ( (H<=10.0) || H>=350.0) { p[RIGHT][1]=TRUE; }

    /***********************************************************************/

    if (p[LEFT][0] || p[LEFT][1] || p[RIGHT][0] || p[RIGHT][1]) return(FALSE);
    else return(TRUE);
}

void convolve_pair_1X3(f,g,h)
    double f[NEYES][NPIXELS];
    double g[3];
    double h[NEYES][NPIXELS];
{
    int i, j, k, alpha;

    alpha = 1;
    h[LEFT][0] = h[LEFT][NPIXELS-1] = h[RIGHT][0] = h[RIGHT][NPIXELS-1] = 0;
    for (i=LEFT; i<=RIGHT; ++i) { 
        for (j=1; j<(NPIXELS-1); ++j) {
            h[i][j] = 0.0;
            for (k=-alpha; k<=alpha; ++k) {
                h[i][j] += g[k+alpha] * f[i][j+k];
            }
        }
    }
}

/*************************************************************************/
// compute the (0<pos<128) of the average edge pixel on both image planes 
// if at least one edge is detected on both images: 
//    write the index of the mean pixel into ul and ur; return TRUE;
// else return FALSE;
/*************************************************************************/
double Prewit[3] = {-1, 0, 1};
int compute_average_edge_pixel(roger, ur, ul)
    Robot * roger;
    double *ur, *ul;
{
    int i;
    double r, g, b, I[NEYES][NPIXELS];
    double w, wsuml, wsumr, suml, sumr;
    double edge[NEYES][NPIXELS];

    // make the pair of intensity images
    for (i=0;i<NPIXELS;++i) {
        /* PIXEL i IN LEFT EYE */
        r = (double) roger->image[LEFT][i][RED_CHANNEL];
        g = (double) roger->image[LEFT][i][GREEN_CHANNEL];
        b = (double) roger->image[LEFT][i][BLUE_CHANNEL];

        I[LEFT][i] = (r + g + b)/3.0;

        /* PIXEL i IN RIGHT EYE */
        r = (double) roger->image[RIGHT][i][RED_CHANNEL];
        g = (double) roger->image[RIGHT][i][GREEN_CHANNEL];
        b = (double) roger->image[RIGHT][i][BLUE_CHANNEL];

        I[RIGHT][i] = (r + g + b)/3.0;
    }

    convolve_pair_1X3(I, Prewit, edge); 

    wsuml = wsumr = 0.0;
    suml = sumr = 0.0;
    *ul = *ur = 0.0;

    for (i=0;i<NPIXELS;++i) {
        w = fabs(edge[LEFT][i]);
        wsuml += w*i;
        suml += w;
        w = fabs(edge[RIGHT][i]);
        wsumr += w*i;
        sumr += w;
    }
    if (suml > 0) *ul = wsuml/suml;
    if (sumr > 0) *ur = wsumr/sumr;
    if ((suml>0) && (sumr>0)) return(TRUE);
    else return(FALSE);
}
