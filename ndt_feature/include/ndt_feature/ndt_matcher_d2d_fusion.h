#pragma once

#include <ndt_feature/ndt_matcher_d2d_fusion_linesearch.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_registration/ndt_matcher_d2d_feature.h>
#include <ndt_feature/utils.h>


namespace ndt_feature {

inline Eigen::MatrixXd computeHessianMahalanobis(const Eigen::MatrixXd &C) {
  // Compute the Hessian for the mahalanobis distance C = the inverse of the covariance matrix(!)
  Eigen::MatrixXd H(6,6);
  H << C(0,0)+C(0,0), C(1,0)+C(0,1), C(2,0)+C(0,2), C(3,0)+C(0,3), C(4,0)+C(0,4), C(5,0)+C(0,5),
       C(0,1)+C(1,0), C(1,1)+C(1,1), C(2,1)+C(1,2), C(3,1)+C(1,3), C(4,1)+C(1,4), C(5,1)+C(1,5),
       C(0,2)+C(2,0), C(1,2)+C(2,1), C(2,2)+C(2,2), C(3,2)+C(2,3), C(4,2)+C(2,4), C(5,2)+C(2,5),
       C(0,3)+C(3,0), C(1,3)+C(3,1), C(2,3)+C(3,2), C(3,3)+C(3,3), C(4,3)+C(3,4), C(5,3)+C(3,5),
       C(0,4)+C(4,0), C(1,4)+C(4,1), C(2,4)+C(4,2), C(3,4)+C(4,3), C(4,4)+C(4,4), C(5,4)+C(4,5),
       C(0,5)+C(5,0), C(1,5)+C(5,1), C(2,5)+C(5,2), C(3,5)+C(5,3), C(4,5)+C(5,4), C(5,5)+C(5,5);
       
  return H;
}

inline double computeScoreMahalanobis(const Eigen::Matrix<double,6,1> &x,
                               const Eigen::MatrixXd &C) {
  return x.transpose()*C*x;
}

inline Eigen::Matrix<double,6,1> computeGradientMahalanobis(const Eigen::Matrix<double,6,1> &x,
                                                     const Eigen::MatrixXd &C) {
  return computeHessianMahalanobis(C)*x;
}


// ---------------------------------------------------------

inline double lineSearchMTFusionTcov(    Eigen::Matrix<double,6,1> &increment,
                                  std::vector<lslgeneric::NDTCell*> &sourceNDT,
                                  lslgeneric::NDTMap &targetNDT,
                                  std::vector<lslgeneric::NDTCell*> &sourceNDT_feat,
                                  lslgeneric::NDTMap &targetNDT_feat,
                                  lslgeneric::NDTMatcherD2D &matcher_d2d,
                                  lslgeneric::NDTMatcherFeatureD2D &matcher_feat_d2d,
                                  const Eigen::Matrix<double,6,1> &localpose,
                                  const Eigen::MatrixXd &C)
{

    // default params
    double stp = 1.0; //default step
    double recoverystep = 0.1;
    double dginit = 0.0;
    double ftol = 0.11111; //epsilon 1
    double gtol = 0.99999; //epsilon 2
    double stpmax = 4.0;
    double stpmin = 0.001;
    int maxfev = 40; //max function evaluations
    double xtol = 0.01; //window of uncertainty around the optimal step

    //my temporary variables
    std::vector<lslgeneric::NDTCell*> sourceNDTHere, sourceNDTHere_feat;
    double score_init = 0.0;

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> ps;
    ps.setIdentity();

    Eigen::Matrix<double,6,1> scg_here;
    Eigen::MatrixXd pincr(6,1), score_gradient_here(6,1), score_gradient_feat_here(6,1), score_gradient_Tcov_here;
    Eigen::MatrixXd pseudoH(6,6), pseudoH_feat(6,6);
    Eigen::Vector3d eulerAngles;
    /////

    int info = 0;			// return code
    int infoc = 1;		// return code for subroutine cstep

    Eigen::Matrix<double,6,1> X = localpose;

    score_gradient_here.setZero();
    score_gradient_feat_here.setZero();
    
    score_init = matcher_d2d.derivativesNDT(sourceNDT,targetNDT,score_gradient_here,pseudoH,false);
    score_init+= matcher_feat_d2d.derivativesNDT(sourceNDT_feat, targetNDT_feat, score_gradient_feat_here, pseudoH_feat, false);
    score_init += computeScoreMahalanobis(X,C);
    
    score_gradient_here += score_gradient_feat_here;
    score_gradient_here += computeGradientMahalanobis(X,C);

    pseudoH += pseudoH_feat;
    scg_here = score_gradient_here;
    dginit = increment.dot(scg_here);

    if (dginit >= 0.0)
    {
        std::cout << "MoreThuente::cvsrch - wrong direction (dginit = " << dginit << ")" << std::endl;
        increment = -increment;
        dginit = -dginit;

        if (dginit >= 0.0)
        {
            for(unsigned int i=0; i<sourceNDTHere.size(); i++)
            {
                if(sourceNDTHere[i]!=NULL)
                    delete sourceNDTHere[i];
            }
            for (unsigned int i=0; i<sourceNDTHere_feat.size(); i++)
            {
              if (sourceNDTHere_feat[i]!=NULL)
                delete sourceNDTHere_feat[i];
            }
            return recoverystep;
        }
    }
    else
    {
      // correct direction
    }

    // Initialize local variables.

    bool brackt = false;		// has the soln been bracketed?
    bool stage1 = true;		// are we in stage 1?
    int nfev = 0;			// number of function evaluations
    double dgtest = ftol * dginit; // f for curvature condition
    double width = stpmax - stpmin; // interval width
    double width1 = 2 * width;	// ???

    // initial function value
    double finit = 0.0;
    finit = score_init;

    // The variables stx, fx, dgx contain the values of the step,
    // function, and directional derivative at the best step.  The
    // variables sty, fy, dgy contain the value of the step, function,
    // and derivative at the other endpoint of the interval of
    // uncertainty.  The variables stp, f, dg contain the values of the
    // step, function, and derivative at the current step.

    double stx = 0.0;
    double fx = finit;
    double dgx = dginit;
    double sty = 0.0;
    double fy = finit;
    double dgy = dginit;

    // Start of iteration.
    double stmin, stmax;
    double fm, fxm, fym, dgm, dgxm, dgym;

    while (1)
    {
        // Set the minimum and maximum steps to correspond to the present
        // interval of uncertainty.
        if (brackt)
        {
            stmin = lslgeneric::NDTMatcherD2D::MoreThuente::min(stx, sty);
            stmax = lslgeneric::NDTMatcherD2D::MoreThuente::max(stx, sty);
        }
        else
        {
            stmin = stx;
            stmax = stp + 4 * (stp - stx);
        }

        // Force the step to be within the bounds stpmax and stpmin.
        stp = lslgeneric::NDTMatcherD2D::MoreThuente::max(stp, stpmin);
        stp = lslgeneric::NDTMatcherD2D::MoreThuente::min(stp, stpmax);

        // If an unusual termination is to occur then let stp be the
        // lowest point obtained so far.

        if ((brackt && ((stp <= stmin) || (stp >= stmax))) ||
                (nfev >= maxfev - 1) || (infoc == 0) ||
                (brackt && (stmax - stmin <= xtol * stmax)))
        {
            stp = stx;
        }

        // Evaluate the function and gradient at stp
        // and compute the directional derivative.
        ///////////////////////////////////////////////////////////////////////////

        pincr = stp*increment;

        X += pincr; // Keep track of the offset (wo using any affine transform - to use the inverse of the affine transforms to get the offset typically involves roll and pitch angles very close to +M_PI and -MPI).

        ps = Eigen::Translation<double,3>(pincr(0),pincr(1),pincr(2))*
             Eigen::AngleAxisd(pincr(3),Eigen::Vector3d::UnitX())*
             Eigen::AngleAxisd(pincr(4),Eigen::Vector3d::UnitY())*
             Eigen::AngleAxisd(pincr(5),Eigen::Vector3d::UnitZ());

        for(unsigned int i=0; i<sourceNDTHere.size(); i++)
        {
            if(sourceNDTHere[i]!=NULL)
                delete sourceNDTHere[i];
        }
        for(unsigned int i=0; i<sourceNDTHere_feat.size(); i++)
        {
            if(sourceNDTHere_feat[i]!=NULL)
                delete sourceNDTHere_feat[i];
        }
        sourceNDTHere.clear();
        sourceNDTHere_feat.clear();
        for(unsigned int i=0; i<sourceNDT.size(); i++)
        {
            lslgeneric::NDTCell *cell = sourceNDT[i];
            if(cell!=NULL)
            {
                Eigen::Vector3d mean = cell->getMean();
                Eigen::Matrix3d cov = cell->getCov();
                mean = ps*mean;
                cov = ps.rotation()*cov*ps.rotation().transpose();
                lslgeneric::NDTCell* nd = (lslgeneric::NDTCell*)cell->copy();
                nd->setMean(mean);
                nd->setCov(cov);
                sourceNDTHere.push_back(nd);
            }
        }
        for(unsigned int i=0; i<sourceNDT_feat.size(); i++)
        {
            lslgeneric::NDTCell *cell = sourceNDT_feat[i];
            if(cell!=NULL)
            {
                Eigen::Vector3d mean = cell->getMean();
                Eigen::Matrix3d cov = cell->getCov();
                mean = ps*mean;
                cov = ps.rotation()*cov*ps.rotation().transpose();
                lslgeneric::NDTCell* nd = (lslgeneric::NDTCell*)cell->copy();
                nd->setMean(mean);
                nd->setCov(cov);
                sourceNDTHere_feat.push_back(nd);
            }
        }

        double f = 0.0;
        score_gradient_here.setZero();
        score_gradient_feat_here.setZero();


        f = matcher_d2d.derivativesNDT(sourceNDTHere,targetNDT,score_gradient_here,pseudoH,false);
        double f_feat = matcher_feat_d2d.derivativesNDT(sourceNDT_feat,targetNDT_feat,score_gradient_feat_here,pseudoH_feat,false);

        f += f_feat;
        score_gradient_here += score_gradient_feat_here;
        pseudoH += pseudoH_feat;

        double f_Tcov = computeScoreMahalanobis(X,C);
        f += f_Tcov;

        score_gradient_here += score_gradient_feat_here;
        score_gradient_Tcov_here = computeGradientMahalanobis(X,C);

        score_gradient_here += score_gradient_Tcov_here;

        double dg = 0.0;
        scg_here = score_gradient_here;
        dg = increment.dot(scg_here);

        nfev ++;

///////////////////////////////////////////////////////////////////////////

        // Armijo-Goldstein sufficient decrease
        double ftest1 = finit + stp * dgtest;

        // Test for convergence.

        if ((brackt && ((stp <= stmin) || (stp >= stmax))) || (infoc == 0))
            info = 6;			// Rounding errors

        if ((stp == stpmax) && (f <= ftest1) && (dg <= dgtest))
            info = 5;			// stp=stpmax

        if ((stp == stpmin) && ((f > ftest1) || (dg >= dgtest)))
            info = 4;			// stp=stpmin

        if (nfev >= maxfev)
            info = 3;			// max'd out on fevals

        if (brackt && (stmax-stmin <= xtol*stmax))
            info = 2;			// bracketed soln

        // RPP sufficient decrease test can be different
        bool sufficientDecreaseTest = false;
        sufficientDecreaseTest = (f <= ftest1);  // Armijo-Golstein

        if ((sufficientDecreaseTest) && (fabs(dg) <= gtol*(-dginit)))
            info = 1;			// Success!!!!

        if (info != 0) 		// Line search is done
        {
            if (info != 1) 		// Line search failed
            {
              std::cout << "using recovery step" << std::endl;
                stp = recoverystep;
            }
            else 			// Line search succeeded
            {
              //              std::cout << "step accepted : " << stp << std::endl;
            }

            for(unsigned int i=0; i<sourceNDTHere.size(); i++)
            {
                if(sourceNDTHere[i]!=NULL)
                    delete sourceNDTHere[i];
            }
            for(unsigned int i=0; i<sourceNDTHere_feat.size(); i++)
            {
              if (sourceNDTHere_feat[i]!=NULL)
                delete sourceNDTHere_feat[i];
            }
            return stp;

        } // info != 0

        // RPP add
        //counter.incrementNumIterations();

        // In the first stage we seek a step for which the modified
        // function has a nonpositive value and nonnegative derivative.

        if (stage1 && (f <= ftest1) && (dg >= lslgeneric::NDTMatcherD2D::MoreThuente::min(ftol, gtol) * dginit))
        {
            stage1 = false;
        }

        // A modified function is used to predict the step only if we have
        // not obtained a step for which the modified function has a
        // nonpositive function value and nonnegative derivative, and if a
        // lower function value has been obtained but the decrease is not
        // sufficient.

        if (stage1 && (f <= fx) && (f > ftest1))
        {

            // Define the modified function and derivative values.

            fm = f - stp * dgtest;
            fxm = fx - stx * dgtest;
            fym = fy - sty * dgtest;
            dgm = dg - dgtest;
            dgxm = dgx - dgtest;
            dgym = dgy - dgtest;

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.

            //VALGRIND_CHECK_VALUE_IS_DEFINED(dgm);
            infoc = lslgeneric::NDTMatcherD2D::MoreThuente::cstep(stx,fxm,dgxm,sty,fym,dgym,stp,fm,dgm,
                                       brackt,stmin,stmax);

            // Reset the function and gradient values for f.

            fx = fxm + stx*dgtest;
            fy = fym + sty*dgtest;
            dgx = dgxm + dgtest;
            dgy = dgym + dgtest;

        }

        else
        {

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.

            //VALGRIND_CHECK_VALUE_IS_DEFINED(dg);
            infoc = lslgeneric::NDTMatcherD2D::MoreThuente::cstep(stx,fx,dgx,sty,fy,dgy,stp,f,dg,
                                       brackt,stmin,stmax);

        }

        // Force a sufficient decrease in the size of the
        // interval of uncertainty.

        if (brackt)
        {
            if (fabs(sty - stx) >= 0.66 * width1)
                stp = stx + 0.5 * (sty - stx);
            width1 = width;
            width = fabs(sty-stx);
        }

    } // while-loop

  
}



//perform line search to find the best descent rate (More&Thuente)
inline double lineSearchMTFusion(
    Eigen::Matrix<double,6,1> &increment,
    std::vector<lslgeneric::NDTCell*> &sourceNDT,
    lslgeneric::NDTMap &targetNDT,
    std::vector<lslgeneric::NDTCell*> &sourceNDT_feat,
    lslgeneric::NDTMap &targetNDT_feat,
    lslgeneric::NDTMatcherD2D &matcher_d2d,
    lslgeneric::NDTMatcherFeatureD2D &matcher_feat_d2d)
{
    // default params
    double stp = 1.0; //default step
    double recoverystep = 0.1;
    double dginit = 0.0;
    double ftol = 0.11111; //epsilon 1
    double gtol = 0.99999; //epsilon 2
    double stpmax = 4.0;
    double stpmin = 0.001;
    int maxfev = 40; //max function evaluations
    double xtol = 0.01; //window of uncertainty around the optimal step

    //my temporary variables
    std::vector<lslgeneric::NDTCell*> sourceNDTHere, sourceNDTHere_feat;
    double score_init = 0.0;

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> ps;
    ps.setIdentity();

    Eigen::Matrix<double,6,1> scg_here;
    Eigen::MatrixXd pincr(6,1), score_gradient_here(6,1), score_gradient_feat_here(6,1);
    Eigen::MatrixXd pseudoH(6,6), pseudoH_feat(6,6);
    Eigen::Vector3d eulerAngles;
    /////

    int info = 0;			// return code
    int infoc = 1;		// return code for subroutine cstep

    // Compute the initial gradient in the search direction and check
    // that s is a descent direction.

    //we want to maximize s, so we should minimize -s
    //score_init = scoreNDT(sourceNDT,targetNDT);

    //gradient directions are opposite for the negated function
    //score_gradient_init = -score_gradient_init;

//  cout<<"score_init "<<score_init<<endl;
//  cout<<"score_gradient_init "<<score_gradient_init.transpose()<<endl;
//  cout<<"increment "<<increment.transpose()<<endl;

    score_gradient_here.setZero();
    score_gradient_feat_here.setZero();
    
    //    std::cout << "computing gradients: " << std::endl;
    //    std::cout << "matcher_d2d.n_neighbours : " << matcher_d2d.n_neighbours << std::endl;
    score_init = matcher_d2d.derivativesNDT(sourceNDT,targetNDT,score_gradient_here,pseudoH,false);
    score_init+= matcher_feat_d2d.derivativesNDT(sourceNDT_feat, targetNDT_feat, score_gradient_feat_here, pseudoH_feat, false);
    //    std::cout << "score_gradient_ndt_here : " << score_gradient_here << std::endl;
    score_gradient_here += score_gradient_feat_here;
    //    std::cout << "score_gradient_feat_here : " << score_gradient_feat_here << std::endl;
    //    std::cout << "pseudoH_ndt : " << pseudoH << std::endl;
    //    std::cout << "pseudoH_feat: " << pseudoH_feat << std::endl;
    pseudoH += pseudoH_feat;
    scg_here = score_gradient_here;
    dginit = increment.dot(scg_here);
    //    std::cout<<"dginit "<<dginit<<std::endl;

    if (dginit >= 0.0)
    {
        std::cout << "MoreThuente::cvsrch - wrong direction (dginit = " << dginit << ")" << std::endl;
        //return recoverystep; //TODO TSV -1; //
        //return -1;

        increment = -increment;
        dginit = -dginit;

        if (dginit >= 0.0)
        {
            for(unsigned int i=0; i<sourceNDTHere.size(); i++)
            {
                if(sourceNDTHere[i]!=NULL)
                    delete sourceNDTHere[i];
            }
            for (unsigned int i=0; i<sourceNDTHere_feat.size(); i++)
            {
              if (sourceNDTHere_feat[i]!=NULL)
                delete sourceNDTHere_feat[i];
            }
            return recoverystep;
        }
    }
    else
    {
//     cout<<"correct direction (dginit = " << dginit << ")" << endl;
    }

    // Initialize local variables.

    bool brackt = false;		// has the soln been bracketed?
    bool stage1 = true;		// are we in stage 1?
    int nfev = 0;			// number of function evaluations
    double dgtest = ftol * dginit; // f for curvature condition
    double width = stpmax - stpmin; // interval width
    double width1 = 2 * width;	// ???

    //cout<<"dgtest "<<dgtest<<endl;
    // initial function value
    double finit = 0.0;
    finit = score_init;

    // The variables stx, fx, dgx contain the values of the step,
    // function, and directional derivative at the best step.  The
    // variables sty, fy, dgy contain the value of the step, function,
    // and derivative at the other endpoint of the interval of
    // uncertainty.  The variables stp, f, dg contain the values of the
    // step, function, and derivative at the current step.

    double stx = 0.0;
    double fx = finit;
    double dgx = dginit;
    double sty = 0.0;
    double fy = finit;
    double dgy = dginit;

    // Get the linear solve tolerance for adjustable forcing term
    //double eta_original = -1.0;
    //double eta = 0.0;
    //eta = eta_original;

    // Start of iteration.

    double stmin, stmax;
    double fm, fxm, fym, dgm, dgxm, dgym;

    while (1)
    {
        // Set the minimum and maximum steps to correspond to the present
        // interval of uncertainty.
        if (brackt)
        {
            stmin = lslgeneric::NDTMatcherD2D::MoreThuente::min(stx, sty);
            stmax = lslgeneric::NDTMatcherD2D::MoreThuente::max(stx, sty);
        }
        else
        {
            stmin = stx;
            stmax = stp + 4 * (stp - stx);
        }

        // Force the step to be within the bounds stpmax and stpmin.
        stp = lslgeneric::NDTMatcherD2D::MoreThuente::max(stp, stpmin);
        stp = lslgeneric::NDTMatcherD2D::MoreThuente::min(stp, stpmax);

        // If an unusual termination is to occur then let stp be the
        // lowest point obtained so far.

        if ((brackt && ((stp <= stmin) || (stp >= stmax))) ||
                (nfev >= maxfev - 1) || (infoc == 0) ||
                (brackt && (stmax - stmin <= xtol * stmax)))
        {
            stp = stx;
        }

        // Evaluate the function and gradient at stp
        // and compute the directional derivative.
        ///////////////////////////////////////////////////////////////////////////

        pincr = stp*increment;

        ps = Eigen::Translation<double,3>(pincr(0),pincr(1),pincr(2))*
             Eigen::AngleAxisd(pincr(3),Eigen::Vector3d::UnitX())*
             Eigen::AngleAxisd(pincr(4),Eigen::Vector3d::UnitY())*
             Eigen::AngleAxisd(pincr(5),Eigen::Vector3d::UnitZ());

        for(unsigned int i=0; i<sourceNDTHere.size(); i++)
        {
            if(sourceNDTHere[i]!=NULL)
                delete sourceNDTHere[i];
        }
        for(unsigned int i=0; i<sourceNDTHere_feat.size(); i++)
        {
            if(sourceNDTHere_feat[i]!=NULL)
                delete sourceNDTHere_feat[i];
        }
        sourceNDTHere.clear();
        sourceNDTHere_feat.clear();
        for(unsigned int i=0; i<sourceNDT.size(); i++)
        {
            lslgeneric::NDTCell *cell = sourceNDT[i];
            if(cell!=NULL)
            {
                Eigen::Vector3d mean = cell->getMean();
                Eigen::Matrix3d cov = cell->getCov();
                mean = ps*mean;
                cov = ps.rotation()*cov*ps.rotation().transpose();
                lslgeneric::NDTCell* nd = (lslgeneric::NDTCell*)cell->copy();
                nd->setMean(mean);
                nd->setCov(cov);
                sourceNDTHere.push_back(nd);
            }
        }
        for(unsigned int i=0; i<sourceNDT_feat.size(); i++)
        {
            lslgeneric::NDTCell *cell = sourceNDT_feat[i];
            if(cell!=NULL)
            {
                Eigen::Vector3d mean = cell->getMean();
                Eigen::Matrix3d cov = cell->getCov();
                mean = ps*mean;
                cov = ps.rotation()*cov*ps.rotation().transpose();
                lslgeneric::NDTCell* nd = (lslgeneric::NDTCell*)cell->copy();
                nd->setMean(mean);
                nd->setCov(cov);
                sourceNDTHere_feat.push_back(nd);
            }
        }

        double f = 0.0;
        score_gradient_here.setZero();
        score_gradient_feat_here.setZero();

        /*f = scoreNDT(sourceNDT,targetNDT,ps);
        derivativesNDT(sourceNDT,targetNDT,ps,score_gradient_here,pseudoH,false);
        std::cout<<"scg1  " <<score_gradient_here.transpose()<<std::endl;
        */

        //option 2:
        //f = scoreNDT(sourceNDTHere,targetNDT);
        f = matcher_d2d.derivativesNDT(sourceNDTHere,targetNDT,score_gradient_here,pseudoH,false);
        //        std::cout << "f_ndt : " << f << std::endl;
        double f_feat = matcher_feat_d2d.derivativesNDT(sourceNDT_feat,targetNDT_feat,score_gradient_feat_here,pseudoH_feat,false);
        //        std::cout << "f_feat : " << std::endl;
        
        //        std::cout << "score_gradient_ndt_here : " << score_gradient_here << std::endl;
        //        std::cout << "score_gradient_feat_here : " << score_gradient_feat_here << std::endl;
        //        std::cout << "pseudoH_ndt : " << pseudoH << std::endl;
        //        std::cout << "pseudoH_feat: " << pseudoH_feat << std::endl;

        f += f_feat;
        score_gradient_here += score_gradient_feat_here;
        pseudoH += pseudoH_feat;

        //std::cout<<"scg2  " <<score_gradient_here.transpose()<<std::endl;


        //cout<<"incr " <<pincr.transpose()<<endl;
        //cout<<"score (f) "<<f<<endl;

        double dg = 0.0;
        scg_here = score_gradient_here;
        dg = increment.dot(scg_here);


        //VALGRIND_CHECK_VALUE_IS_DEFINED(dg);
        //cout<<"dg = "<<dg<<endl;
        nfev ++;

///////////////////////////////////////////////////////////////////////////

        //cout<<"consider step "<<stp<<endl;
        // Armijo-Goldstein sufficient decrease
        double ftest1 = finit + stp * dgtest;
        //cout<<"ftest1 is "<<ftest1<<endl;

        // Test for convergence.

        if ((brackt && ((stp <= stmin) || (stp >= stmax))) || (infoc == 0))
            info = 6;			// Rounding errors

        if ((stp == stpmax) && (f <= ftest1) && (dg <= dgtest))
            info = 5;			// stp=stpmax

        if ((stp == stpmin) && ((f > ftest1) || (dg >= dgtest)))
            info = 4;			// stp=stpmin

        if (nfev >= maxfev)
            info = 3;			// max'd out on fevals

        if (brackt && (stmax-stmin <= xtol*stmax))
            info = 2;			// bracketed soln

        // RPP sufficient decrease test can be different
        bool sufficientDecreaseTest = false;
        sufficientDecreaseTest = (f <= ftest1);  // Armijo-Golstein

        //cout<<"ftest2 "<<gtol*(-dginit)<<endl;
        //cout<<"sufficientDecrease? "<<sufficientDecreaseTest<<endl;
        //cout<<"curvature ok? "<<(fabs(dg) <= gtol*(-dginit))<<endl;
        if ((sufficientDecreaseTest) && (fabs(dg) <= gtol*(-dginit)))
            info = 1;			// Success!!!!

        if (info != 0) 		// Line search is done
        {
            if (info != 1) 		// Line search failed
            {
                // RPP add
                // counter.incrementNumFailedLineSearches();

                //if (recoveryStepType == Constant)
                stp = recoverystep;

                std::cout << "using recovery step" << std::endl;

                //newgrp.computeX(oldgrp, dir, stp);

                //message = "(USING RECOVERY STEP!)";

            }
            else 			// Line search succeeded
            {
              std::cout << "step accepted" << std::endl;
                //message = "(STEP ACCEPTED!)";
            }

            //print.printStep(nfev, stp, finit, f, message);

            // Returning the line search flag
            //cout<<"LineSearch::"<<message<<" info "<<info<<endl;
            for(unsigned int i=0; i<sourceNDTHere.size(); i++)
            {
                if(sourceNDTHere[i]!=NULL)
                    delete sourceNDTHere[i];
            }
            for(unsigned int i=0; i<sourceNDTHere_feat.size(); i++)
            {
              if (sourceNDTHere_feat[i]!=NULL)
                delete sourceNDTHere_feat[i];
            }
//      std::cout<<"nfev = "<<nfev<<std::endl;
            std::cout << "line search return stp : " << stp << std::endl;
            return stp;

        } // info != 0

        // RPP add
        //counter.incrementNumIterations();

        // In the first stage we seek a step for which the modified
        // function has a nonpositive value and nonnegative derivative.

        if (stage1 && (f <= ftest1) && (dg >= lslgeneric::NDTMatcherD2D::MoreThuente::min(ftol, gtol) * dginit))
        {
            stage1 = false;
        }

        // A modified function is used to predict the step only if we have
        // not obtained a step for which the modified function has a
        // nonpositive function value and nonnegative derivative, and if a
        // lower function value has been obtained but the decrease is not
        // sufficient.

        if (stage1 && (f <= fx) && (f > ftest1))
        {

            // Define the modified function and derivative values.

            fm = f - stp * dgtest;
            fxm = fx - stx * dgtest;
            fym = fy - sty * dgtest;
            dgm = dg - dgtest;
            dgxm = dgx - dgtest;
            dgym = dgy - dgtest;

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.

            //VALGRIND_CHECK_VALUE_IS_DEFINED(dgm);
            infoc = lslgeneric::NDTMatcherD2D::MoreThuente::cstep(stx,fxm,dgxm,sty,fym,dgym,stp,fm,dgm,
                                       brackt,stmin,stmax);

            // Reset the function and gradient values for f.

            fx = fxm + stx*dgtest;
            fy = fym + sty*dgtest;
            dgx = dgxm + dgtest;
            dgy = dgym + dgtest;

        }

        else
        {

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.

            //VALGRIND_CHECK_VALUE_IS_DEFINED(dg);
            infoc = lslgeneric::NDTMatcherD2D::MoreThuente::cstep(stx,fx,dgx,sty,fy,dgy,stp,f,dg,
                                       brackt,stmin,stmax);

        }

        // Force a sufficient decrease in the size of the
        // interval of uncertainty.

        if (brackt)
        {
            if (fabs(sty - stx) >= 0.66 * width1)
                stp = stx + 0.5 * (sty - stx);
            width1 = width;
            width = fabs(sty-stx);
        }

    } // while-loop

}


// TODO - wrap this up and write a new class in ndt_registration
inline  bool matchFusion( lslgeneric::NDTMap& targetNDT,
		    lslgeneric::NDTMap& sourceNDT,
		    lslgeneric::NDTMap& targetNDT_feat,
		    lslgeneric::NDTMap& sourceNDT_feat,
		    const std::vector<std::pair<int, int> > &corr_feat,
		    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                    const Eigen::MatrixXd& Tcov,
		    bool useInitialGuess, bool useNDT, bool useFeat, bool step_control, int ITR_MAX = 30, int n_neighbours = 2, double DELTA_SCORE = 10e-4, bool useSoftConstraints = true, bool step_control_fusion = true, bool useTikhonovRegularization = false)
{
  std::cerr << "matchFusion()" << " useNDT : " << useNDT << " useFeat : " << useFeat << " step_control : " << step_control << " corr_feat.size() : " << corr_feat.size() << std::endl;

  // Combines two different NDT maps at once. One which holds NDT derrived from features with known correspondance (obtained earlier through RANSAC or similar) with two standard NDT maps (target could very well be obtained from the fuser).
  
  // Create matching objects. The matching is done below but this is only used to get access to compute derrivatives etc.
  lslgeneric::NDTMatcherD2D matcher_d2d;
  lslgeneric::NDTMatcherFeatureD2D matcher_feat_d2d(corr_feat);

  matcher_d2d.n_neighbours = n_neighbours;

    //locals
    bool convergence = false;
    double score_best = std::numeric_limits<double>::max();
    int itr_ctr = 0;
    int best_itr = 0;
    double step_size = 1;
    Eigen::Matrix<double,6,1>  pose_increment_v, scg, pose_local_v;
    Eigen::MatrixXd Hessian(6,6), score_gradient(6,1); //column vectors, pose_increment_v(6,1)
    Eigen::MatrixXd Hessian_ndt(6,6), score_gradient_ndt(6,1); //column vectors, pose_increment_v(6,1)
    Eigen::MatrixXd Hessian_feat(6,6), score_gradient_feat(6,1); //column vectors, pose_increment_v(6,1)
    Eigen::MatrixXd Hessian_Tcov(6,6), score_gradient_Tcov(6,1);

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR, Tbest, Tinit, Tlocal;
    Eigen::Vector3d transformed_vec, mean;
    bool ret = true;
    if(!useInitialGuess)
    {
        T.setIdentity();
    }
    Tbest = T;
    Tinit = T;
    Tlocal.setIdentity();
    pose_local_v.setZero();

    std::vector<lslgeneric::NDTCell*> nextNDT = sourceNDT.pseudoTransformNDT(T);
    std::vector<lslgeneric::NDTCell*> nextNDT_feat = sourceNDT_feat.pseudoTransformNDT(T);


    // Tikhonov variables... - NOT USED.
    Eigen::MatrixXd Q = Tcov.inverse();
    Eigen::Matrix<double,6,1> x0, Tx, X;
    while(!convergence)
    {
      //      std::cout << "itr_ctr : " << itr_ctr << std::endl;
        TR.setIdentity();
        Hessian.setZero();
        score_gradient.setZero();

        
	double score_here = 0.;
        double score_here_ndt = matcher_d2d.derivativesNDT(nextNDT,targetNDT,score_gradient_ndt,Hessian_ndt,true);
	//double score_here = matcher_d2d.derivativesNDT_2d(nextNDT,targetNDT,score_gradient_ndt,Hessian,true);
        double score_here_feat = matcher_feat_d2d.derivativesNDT(nextNDT_feat,targetNDT_feat,score_gradient_feat,Hessian_feat,true);
        

	// Sum them up...
	if (useNDT) {
	  score_here += score_here_ndt;
	  Hessian += Hessian_ndt;
	  score_gradient += score_gradient_ndt;
	}
	if (useFeat) {
	  score_here += score_here_feat;
	  Hessian += Hessian_feat;
	  score_gradient += score_gradient_feat;
	}

        double score_here_Tcov = 0.;

        if (useSoftConstraints) {

          // The relative posedifference between the initial estimate and the current estimate.
          //          Eigen::Affine3d tmp = T * Tinit.inverse();
          //          forceEigenAffine3dTo2dInPlace(tmp);
          //          convertAffineToVector(tmp, X);
          //          convertAffineToVector(T * Tinit.inverse(),X);
          //          convertAffineToVector(Tlocal, X);
          X = pose_local_v;
          score_here_Tcov = computeScoreMahalanobis(X,Q);
          score_here += score_here_Tcov;
          Hessian_Tcov = computeHessianMahalanobis(Q);
          Hessian += Hessian_Tcov;
          score_gradient_Tcov = computeGradientMahalanobis(X,Q);
          score_gradient += score_gradient_Tcov;
        }


        // Add the T_est with covariance using the Generalized Tikhonov regularization.
        if (useTikhonovRegularization) {
          std::cerr << "----------- TikhonovRegularization ---------- " << std::endl;
          // update the score gradient with the regularization part.
          //          Eigen::MatrixXd Q = Tcov.inverse();
          Eigen::MatrixXd P(6,6); P.setIdentity();
          Eigen::MatrixXd H = Hessian;
          Eigen::MatrixXd g = score_gradient;
          //          Eigen::Affine3d x0T = Tinit * T.inverse();
          Eigen::Affine3d x0T = T * Tinit.inverse();
          forceEigenAffine3dTo2dInPlace(x0T);
          
          //  Eigen::Matrix<double,6,1> x0;
          convertAffineToVector(x0T,x0);
          score_gradient = H.transpose()*P*g+Q*x0;
          Hessian = H.transpose()*P*H+Q;

          score_here += x0.transpose()*Q*x0;
        }

        scg = score_gradient;
	if(score_here < score_best) 
	{
	    Tbest = T;
	    score_best = score_here;
            best_itr = itr_ctr;
	    std::cout<<"best score "<<score_best<<" at "<<itr_ctr<<std::endl;
	}

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6> > Sol (Hessian);
        Eigen::Matrix<double,6,1> evals = Sol.eigenvalues().real();
        double minCoeff = evals.minCoeff();
        double maxCoeff = evals.maxCoeff();
        if(minCoeff < 0)  //|| evals.minCoeff()) // < 10e-5*evals.maxCoeff()) 
        {
            Eigen::Matrix<double,6,6> evecs = Sol.eigenvectors().real();
            double regularizer = score_gradient.norm();
	    regularizer = regularizer + minCoeff > 0 ? regularizer : 0.001*maxCoeff - minCoeff;
            //double regularizer = 0.001*maxCoeff - minCoeff;
            Eigen::Matrix<double,6,1> reg;
            //ugly
            reg<<regularizer,regularizer,regularizer,regularizer,regularizer,regularizer;
            evals += reg;
            Eigen::Matrix<double,6,6> Lam;
            Lam = evals.asDiagonal();
            Hessian = evecs*Lam*(evecs.transpose());
            std::cerr<<"regularizing\n";
        }


        if (score_gradient.norm()<= DELTA_SCORE)
        {
	  //	    std::cout<<"incr(:,"<<itr_ctr+1<<") = [0 0 0 0 0 0]';\n";
	  std::cout<<"\%gradient vanished\n";
	    if(score_here > score_best) 
	    {
              std::cout<<"crap iterations, best was "<<score_best<<" last was "<<score_here<<std::endl;
		T = Tbest;
	    }
            //de-alloc nextNDT
            for(unsigned int i=0; i<nextNDT.size(); i++)
            {
                if(nextNDT[i]!=NULL)
                    delete nextNDT[i];
            }            
	    for(unsigned int i=0; i<nextNDT_feat.size(); i++)
            {
                if(nextNDT_feat[i]!=NULL)
                    delete nextNDT_feat[i];
            }

            return true;
        }
        pose_increment_v = -Hessian.ldlt().solve(score_gradient);
        // const double max_pose_increment_norm = 0.02;
        // if (pose_increment_v.norm() > max_pose_increment_norm) {
        //   pose_increment_v.normalize();
        //   pose_increment_v *= max_pose_increment_norm;
        // }

        double dginit = pose_increment_v.dot(scg);
        std::cout << "pose_increment_v : " << pose_increment_v.transpose() << " norm : " << pose_increment_v.norm() << std::endl;

        if(dginit > 0)
        {
            //de-alloc nextNDT
	    if(score_here > score_best) 
	    {
              std::cout<<"crap iterations++++, best was "<<score_best<<" last was "<<score_here<<std::endl;
		T = Tbest;
	    }
            for(unsigned int i=0; i<nextNDT.size(); i++)
            {
                if(nextNDT[i]!=NULL)
                    delete nextNDT[i];
            }
            for(unsigned int i=0; i<nextNDT_feat.size(); i++)
            {
                if(nextNDT_feat[i]!=NULL)
                    delete nextNDT_feat[i];
            }

	    //	    std::cout<<"itr "<<itr_ctr<<" dScore "<< 0 <<std::endl;
            return true;
        }
;

	if(step_control) {
          //std::cout << "step_control: start" << std::endl;
          double step_size_ndt = 0.;
          double step_size_feat = 0.;
	  if (useNDT && useFeat && step_control_fusion && !useSoftConstraints) {
            step_size = lineSearchMTFusion(pose_increment_v,nextNDT,targetNDT,nextNDT_feat,targetNDT_feat,matcher_d2d,matcher_feat_d2d);
          }
          else {
            if (useSoftConstraints) {
              step_size = lineSearchMTFusionTcov(pose_increment_v,nextNDT,targetNDT,nextNDT_feat,targetNDT_feat,matcher_d2d,matcher_feat_d2d, pose_local_v, Q);
            }
            
            if (useNDT) {
              step_size_ndt = matcher_d2d.lineSearchMT(pose_increment_v,nextNDT,targetNDT);
            }
            if (useFeat) {
              step_size_feat = matcher_feat_d2d.lineSearchMT(pose_increment_v,nextNDT_feat, targetNDT_feat);
            }
            if (step_size_ndt != 0. && step_size_feat != 0.) {
              step_size = std::min(step_size_ndt, step_size_feat);
            }
            else {
              step_size = std::max(step_size_ndt, step_size_feat);
            }
          }
          
          //          std::cout << "step_size_ndt : " << step_size_ndt << " step_size_feat : " << step_size_feat << std::endl;

          //std::cout << "step_control: end" << std::endl;
	} else {
	    step_size = 1;
	}
        pose_increment_v = step_size*pose_increment_v;
        std::cout<<"\%iteration "<<itr_ctr<<" pose norm "<<(pose_increment_v.norm())<<" score "<<score_here<<" step "<<step_size<<std::endl;

        TR.setIdentity();
        TR =  Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
              Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;

        //transform source NDT
        T = TR*T;
        Tlocal = TR*Tlocal;

        pose_local_v += pose_increment_v;

        for(unsigned int i=0; i<nextNDT.size(); i++)
        {
	    //TRANSFORM
	    Eigen::Vector3d meanC = nextNDT[i]->getMean();
	    Eigen::Matrix3d covC = nextNDT[i]->getCov();
	    meanC = TR*meanC;
	    covC = TR.rotation()*covC*TR.rotation().transpose();
	    nextNDT[i]->setMean(meanC);
	    nextNDT[i]->setCov(covC);
        }
        for(unsigned int i=0; i<nextNDT_feat.size(); i++)
        {
	    //TRANSFORM
	    Eigen::Vector3d meanC = nextNDT_feat[i]->getMean();
	    Eigen::Matrix3d covC = nextNDT_feat[i]->getCov();
	    meanC = TR*meanC;
	    covC = TR.rotation()*covC*TR.rotation().transpose();
	    nextNDT_feat[i]->setMean(meanC);
	    nextNDT_feat[i]->setCov(covC);
        }



        if(itr_ctr>0)
        {
            convergence = ((pose_increment_v.norm()) < DELTA_SCORE);
            //convergence = ((score_gradient.norm()) < DELTA_SCORE);
        }
        if(itr_ctr>ITR_MAX)
        {
            convergence = true;
            ret = false;
        }
        itr_ctr++;
        //step_size *= alpha;
    }
    
    // This is only to compute the score
    double score_here_ndt = matcher_d2d.derivativesNDT(nextNDT,targetNDT,score_gradient_ndt,Hessian_ndt,false);
    //    double score_here = matcher_d2d.derivativesNDT_2d(nextNDT,targetNDT,score_gradient_ndt,Hessian_ndt,false);
    double score_here_feat = matcher_feat_d2d.derivativesNDT(nextNDT_feat,targetNDT_feat,score_gradient_feat,Hessian_feat,false);

    // Sum them up...
    double score_here = 0.;
    if (useNDT) {
      score_here += score_here_ndt;
    }
    if (useFeat) {
      score_here += score_here_feat;
    }

    double score_here_Tcov = 0.;
    if (useSoftConstraints) {
      
      // The relative posedifference between the initial estimate and the current estimate.
      // Eigen::Affine3d tmp = T * Tinit.inverse();
      // forceEigenAffine3dTo2dInPlace(tmp);
      // convertAffineToVector(tmp, X);
      //convertAffineToVector(T * Tinit.inverse(),X);
      //convertAffineToVector(Tlocal, X);
      X = pose_local_v;
      score_here_Tcov = computeScoreMahalanobis(X,Q);
      score_here += score_here_Tcov;
    }


    if (useTikhonovRegularization) {
      score_here += x0.transpose()*Q*x0;
    }
    
    if(score_here > score_best) 
    {
      std::cout<<"crap iterations----, best was "<<score_best<<" last was "<<score_here<<std::endl;
	T = Tbest;
    }
    for(unsigned int i=0; i<nextNDT.size(); i++)
    {
	if(nextNDT[i]!=NULL)
	    delete nextNDT[i];
    }
    for(unsigned int i=0; i<nextNDT_feat.size(); i++)
    {
	if(nextNDT_feat[i]!=NULL)
	    delete nextNDT_feat[i];
    }

    std::cerr << "#### best score : " << score_best << " at iter : " << best_itr << " (" << itr_ctr << ")" <<  std::endl;


//    std::cout<<"incr(:,"<<itr_ctr+1<<") = [0 0 0 0 0 0]';\n";
//    std::cout<<"grad(:,"<<itr_ctr+1<<") = [0 0 0 0 0 0]';\n";
    /*
        snprintf(fname,49,"final.wrl");
        fout = fopen(fname,"w");
        fprintf(fout,"#VRML V2.0 utf8\n");
        targetNDT.writeToVRML(fout,Eigen::Vector3d(1,0,0));
        for(unsigned int i=0; i<nextNDT.size(); i++)
        {
    	if(nextNDT[i]!=NULL)
    	{
    	    nextNDT[i]->writeToVRML(fout,Eigen::Vector3d(0,1,0));
    	}
        }
        fclose(fout);
    */
    //std::cout<<"res "<<current_resolution<<" itr "<<itr_ctr<<std::endl;

    return ret;
}



inline  bool matchFusion2d( lslgeneric::NDTMap& targetNDT,
                      lslgeneric::NDTMap& sourceNDT,
                      lslgeneric::NDTMap& targetNDT_feat,
                      lslgeneric::NDTMap& sourceNDT_feat,
                      const std::vector<std::pair<int, int> > &corr_feat,
                      Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T ,
                      bool useInitialGuess, bool useNDT, bool useFeat, bool step_control, int ITR_MAX = 30, int n_neighbours = 2, double DELTA_SCORE = 10e-4)
  {
    std::cerr << "matchFusion2d()" << " useNDT : " << useNDT << " useFeat : " << useFeat << " step_control : " << step_control << std::endl;
    
    // Only use the standard NDT mapping to start with.
    lslgeneric::NDTMatcherD2D_2D matcher_d2d_2d;
    matcher_d2d_2d.n_neighbours = n_neighbours;
    matcher_d2d_2d.step_control = step_control;
    matcher_d2d_2d.ITR_MAX = ITR_MAX;
    matcher_d2d_2d.DELTA_SCORE = DELTA_SCORE;
    return matcher_d2d_2d.match(targetNDT, sourceNDT, T, useInitialGuess);
}






} // namespace
