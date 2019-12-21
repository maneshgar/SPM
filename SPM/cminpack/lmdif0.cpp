/* lmdif0.c -- driver for lmdif */

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <malloc.h>
#include "cminpak.h"
#include <memory.h>
#include <iostream>

// #include "RandomNumberGenerator.h"
void get_psum(int mpts, int n, double* p, double* psum);
void amoeba(int m, int n, double *p, double* y, double* fvec, double *params, double ftol, void fcn(int,int,double *,double *,int *, double *), int *nfunk);
double amotry(int m, int n, double* p, double* y, double* fvec, double* params, double* psum, void fcn(int,int,double *,double *,int *, double *),
              int ihi, double fac);
double amotry2(int m, int n, double* p, double* y, double* fvec, double* params, double* psum, void fcn(int,int,double *,double *,int *, double *),
               int ihi);


//  This sets up some parameters and calls lmdif, which
//  does a modified version of Levenberg-Marquardt optimization--tsh
//
int lmdif0(void fcn(int,int,double *,double *,int *, double *),int m, int n,double x[],int msk[],
           double fvec[],double tol,int *info,int *nfev, double params[])
{
    int j,maxfev,mode;
    int *ipvt;
    double ftol,xtol,gtol,epsfcn,factor,fnorm;
    double *diag,**fjac,*qtf,*wa1,*wa2,*wa3,*wa4;

    /* Check input parameters */
    if (n <= 0 || m < n || tol < 0.0) {
        *info = 0;
        return(1);
    }
    /* Allocate memory for working arrays. */
    ipvt = (int *)calloc(n,sizeof(int));
    diag = (double *)calloc(n,sizeof(double));
    qtf = (double *)calloc(n,sizeof(double));
    wa1 = (double *)calloc(n,sizeof(double));
    wa2 = (double *)calloc(n,sizeof(double));
    wa3 = (double *)calloc(n,sizeof(double));
    wa4 = (double *)calloc(m,sizeof(double));


    /* Create 2d matrix for Jacobian */
    fjac = (double **)calloc(n,sizeof(double *));
    for (j=0;j<n;j++)
        fjac[j] = (double *)calloc(m,sizeof(double));

    /* Set convergence tolerances */
    ftol = tol;
    xtol = tol;
    gtol = 0.0;

    maxfev = 8000;
    epsfcn = 0.0;
    mode = 1;
    factor = 10;
    *nfev = 0;

    lmdif(fcn,m,n,x,msk,fvec,ftol,xtol,gtol,maxfev,epsfcn,diag,mode,
          factor,info,nfev,fjac,ipvt,qtf,wa1,wa2,wa3,wa4,params);

    if (*info == 8) *info = 4;
    for (j=0;j<n;j++)
        free(fjac[j]);
    free(fjac);
    free(wa4);
    free(wa3);
    free(wa2);
    free(wa1);
    free(qtf);
    free(diag);
    free(ipvt);
    return(0);
}

double sserr(double* fvec, int m) {
    double x;
    x = 0.0;
    for (int i=0;i<m;i++) {
        x+= fvec[i]*fvec[i];
    }
    return x;
}

// This does a simple linesearch optimization algorithm--finds
// the gradient and moves along it until no improvements


int gd0(void fcn(int,int,double *,double *,int *, double *),int m, int n,double x[],int msk[],
        double fvec[],double tol,int *info,int *nfev, double params[])
{
    double *gradient;
    double *xpert;
    int iflag = 1;
    int maxev;
    double temp,h;
    double olderr,err;
    double sum,*stepsize;
    *nfev = 0;
    double h_step = 0.000001;

    gradient = new double[n];
    xpert = new double[n];
    stepsize = new double[n];

    fcn(m,n,x,fvec,&iflag,params);
    (*nfev)++;

    maxev = 2; //setting maxev to 2 assures that only one linesearch will be performed

    while (*nfev<maxev) {
        fcn(m,n,x,fvec,&iflag,params);
        (*nfev)++;

        olderr = sserr(fvec,m);

        // Compute the gradient, normalize it
        sum = 0.0;
        for (int i=0;i<n;i++) {
            temp = x[i];
            if (x[i]!=0.0) h = x[i]*h_step;
            else h = h_step;
            x[i] += h;

            fcn(m,n,x,fvec,&iflag,params);
            (*nfev)++;

            gradient[i] = (sserr(fvec,m)-olderr)/h;
            sum += gradient[i]*gradient[i];
            x[i] = temp;

        }
        sum = 1.0/sqrt(sum);
        for (int i=0;i<n;i++) {
            gradient[i] *= sum;
        }


        // Start with step of 0.01
        for (int i=0;i<n;i++)	{
            stepsize[i] = 0.01;
        }
        bool loop = true;
        // Step in negative gradient direction until no improvement
        while (loop) {
            for (int i=0;i<n;i++) {
                xpert[i] = x[i]-gradient[i]*stepsize[i];
            }


            fcn(m,n,xpert,fvec,&iflag,params);
            (*nfev)++;
            err = sserr(fvec,m);

            if (err>=olderr) {

                for (int i=0;i<n;i++)	{
                    stepsize[i] = -stepsize[i]*0.5;
                }
            }
            else {
                olderr = err;
                for (int i=0;i<n;i++) {
                    x[i] = xpert[i];
                    stepsize[i] = stepsize[i]*1.2;
                }

            }

            for (int i=0;i<n;i++)	{
                if (fabs(stepsize[i])<0.000000001)	{
                    loop=false;
                    break;
                }
            }
        }

    }

    fcn(m,n,x,fvec,&iflag,params);
    *nfev++;

    delete [] gradient;
    delete [] xpert;
    return(0);
}


int ds0(void fcn(int,int,double *,double *,int *, double *),int m, int n,double x[],int msk[],
        double fvec[],double tol,int *info,int *nfev, double params[]) {
    double *y,*p;
    int i,j;
    tol = tol*0.01;

    y = (double*) malloc((n+1)*sizeof(double));
    p = (double*) malloc((n*(n+1))*sizeof(double));

    for (i=0;i<n+1;i++) {
        for (j=0;j<n;j++) {
            p[i*n+j] = x[j];
        }
    }
    for (i=1;i<n+1;i++) {
        if (p[i*n+i-1]!=0.0) p[i*n+i-1]*=1.1;
        else p[i*n+i-1]+=0.0001;
    }


    amoeba(m, n, p, y, fvec, params, tol, fcn, nfev);

    for (j=0;j<n;j++) {
        x[j] = p[0*n+j];
    }
    fcn(m,n,x,fvec,0,params);

    free(p);
    free(y);

    return 0;
}


void get_psum(int mpts, int n, double* p, double* psum) {
    int i,j;
    double sum;
    sum = 0.0;

    for (j=0;j<n;j++) {
        for (i=0;i<mpts;i++) {
            sum += p[i*n+j];
        }
        psum[j] = sum;
    }
}
//
//void f2(int m,int n,double* x,double* fvec,int *iflag, double* params) {

//int gd0(void fcn(int,int,double *,double *,int *, double *),int m, int n,double x[],int msk[],
//    double fvec[],double tol,int *info,int *nfev, double params[])

void amoeba(int m, int n, double *p, double* y, double* fvec, double *params, double ftol, void fcn(int,int,double *,double *,int *, double *), int *nfunk) {

    const int NMAX = 10000;
    const double DP_TINY = 1.0e-10;
    int i,ihi,ilo,inhi,j;
    double tmpd;
    double rtol,ysave,ytry;
    double *psum;
    int mpts;
    mpts = n+1;

    psum = (double*) malloc(n*sizeof(double));

    *nfunk = 0;
    get_psum(mpts,n,p,psum);

    for (i=0;i<mpts;i++) {

        y[i] = amotry2(m,n,p,y,fvec,params,psum,fcn,i);
    }


    for (;;) {
        ilo = 0;
        // first we must determine which point is the highest (worst), next-highest, and lowest
        // (best), by looping over the points in the simplex
        ihi = y[0]>y[1] ? (inhi=1,0) : (inhi=0,1);
        for (i=0;i<mpts;i++) {
            if (y[i]<=y[ilo]) ilo = i;
            if (y[i]>y[ihi]) {
                inhi = ihi;
                ihi = i;
            }
            else if (y[i]>y[inhi] && i!=ihi) {
                inhi = i;
            }
        }
        rtol = 2.0*fabs(y[ihi]-y[ilo])/(fabs(y[ihi])+fabs(y[ilo])+DP_TINY);
        // compute the fractional range from highest to lowest and return if satisfactory
        if (rtol<ftol || *nfunk>=NMAX) {	// if returning, put best point in slot 1
            tmpd = y[0]; y[0] = y[ilo]; y[ilo] = tmpd;
            for (i=0;i<n;i++) {
                tmpd = p[0*n+i]; p[0*n+i] = p[ilo*n+i]; p[ilo*n+i] = tmpd;
            }
            break;
        }
        (*nfunk)+=2;
        // begin a new iteration.  first extrapolate by a factor -1 through the face of the simplex
        // across from the high point, i.e., reflect the simplex from the high point
        ytry = amotry(m,n,p,y,fvec,params,psum,fcn,ihi,-1.0);
        if (ytry<=y[ilo]) {
            // gives a result better than the best point, so try an additional extrapolation
            // by a factor 2
            ytry = amotry(m,n,p,y,fvec,params,psum,fcn,ihi,2.0);
        }
        else if (ytry>=y[inhi]) {
            // the reflected point is worse than the second-highest, so look for an intermediate
            // lower point, i.e., do a one-dimensional contraction
            ysave = y[ihi];
            ytry = amotry(m,n,p,y,fvec,params,psum,fcn,ihi,0.5);
            if (ytry>=ysave) {
                // can't seem to get rid of that high point. better contract around the lowest
                // (best) point
                for (i=0;i<mpts;i++) {
                    if (i!=ilo) {
                        for (j=0;j<n;j++) {
                            p[i*n+j]=psum[j]=0.5*(p[i*n+j]+p[ilo*n+j]);
                        }
                        fcn(m,n,psum,fvec,0,params);

                        y[i] = sserr(fvec,m);
                        //fcn(m,n,x,fvec,&iflag,params);
                    }
                }
                (*nfunk)+=n;
                get_psum(mpts,n,p,psum);
            }
        }
        else (*nfunk)--;
    }
    free(psum);
}

double amotry(int m, int n, double* p, double* y, double* fvec, double* params, double* psum, void fcn(int,int,double *,double *,int *, double *),
              int ihi, double fac) {
    int j;
    double fac1,fac2,ytry;

    double *ptry;

    ptry = (double*) malloc(n*sizeof(double));
    fac1 = (1.0-fac)/n;
    fac2 = fac1-fac;
    for (j=0;j<n;j++) {
        ptry[j]=psum[j]*fac1-p[ihi*n+j]*fac2;
    }
    //	ytry = fcn();
    fcn(m,n,ptry,fvec,0,params);

    ytry = sserr(fvec,m);

    if (ytry<y[ihi]) {
        y[ihi]=ytry;
        for (j=0;j<n;j++) {
            psum[j]+=ptry[j]-p[ihi*n+j];
            p[ihi*n+j] = ptry[j];
        }
    }
    free(ptry);
    return ytry;
}

double amotry2(int m, int n, double* p, double* y, double* fvec, double* params, double* psum, void fcn(int,int,double *,double *,int *, double *),
               int ihi) {
    int j;
    double ytry;

    double *ptry;

    ptry = (double*) malloc(n*sizeof(double));
    for (j=0;j<n;j++) {
        ptry[j]=p[ihi*n+j];
    }
    //	ytry = fcn();
    fcn(m,n,ptry,fvec,0,params);

    ytry = sserr(fvec,m);


    free(ptry);
    return ytry;
}





/*
int sa0(void fcn(int,int,double *,double *,int *, double *),int m, int n,double x[],int msk[],
        double fvec[],double tol,int *info,int *nfev, double params[])	{

    double energyPosition,iterationPosition;
    int iterations;
    double T;                     // the annealing temperature
    double *nextParameters;       // the randomly selected successors parameters
    double currentE, nextE, dE;      // energies
    double minValue, maxValue, bound;
    double *minValues,*maxValues;
    CRandomNumberGenerator randomNumberGenerator;
    double m_coolingParameter = 0.99;
    double m_percentChange = 0.01;
    double m_startingTemperature = 10.0;//100000000000.0;
    #define T_EPSILON 1e-12
    int m_numberOfIterations =1000000;
    int iflag=1;
    *nfev = 0;

    //Set the seed
    randomNumberGenerator.SetSeed((int) time(0));
    // initialize the parameters and the temperature
    T = m_startingTemperature;
    nextParameters = new double[n];
    minValues = new double[n];
    maxValues = new double[n];

    for (int i=0;i<n;i++)	{
        minValues[i] = x[i] - 0.3 * x[i];
        maxValues[i] = x[i] + 0.3 * x[i];
    }

    // find the initial energy
    fcn(m,n,x,fvec,&iflag,params);
    (*nfev)++;
    currentE = sserr(fvec,m);

    iterations = 0;
    //If the temperature reaches ~0 or the number of maximum iterations is reached then stop
    while ((T > T_EPSILON) && (m_numberOfIterations >= iterations))	{
            //Randomly select a successor
            for(int i=0; i<n; i++) {
                //Calculate the minValue
                bound = x[i]-m_percentChange*(maxValues[i] - minValues[i]);
                minValue = bound < minValues[i] ? minValues[i] : bound;
                //Calculate the maxValue
                bound = x[i]+m_percentChange*(maxValues[i] - minValues[i]);
                maxValue = bound > maxValues[i] ? maxValues[i] : bound;

                nextParameters[i] = (double) randomNumberGenerator.UniformRand((float) minValue, (float) maxValue);
            }

            //Calculate the energy of the new set of parameters
            fcn(m,n,nextParameters,fvec,&iflag,params);
            (*nfev)++;
            nextE = sserr(fvec,m);
            // Find dE
            dE = nextE - currentE;

            //If the energy is getting smaller then keep the new changes
            if( dE < 0.0f ) {
                memcpy(x, nextParameters, sizeof(double)*n);
                currentE = nextE;
            }
            //If the energy is getting larger then you made a bad move
            else {
                // accept changes with probability exp(-dE/T)
               if( exp(-dE/T) > randomNumberGenerator.UniformRand(0.0f,1.01f) ) {
                    memcpy(x, nextParameters, sizeof(double)*n);
                    currentE = nextE;
                }
            }
            //In case the system freezes
            iterations++;

            //Decrease the temperature based on the cooling parameter
            T = m_coolingParameter*T;
    }

    //Clean up
    delete [] nextParameters;
    delete [] minValues;
    delete [] maxValues;
    return 0;
}

*/
