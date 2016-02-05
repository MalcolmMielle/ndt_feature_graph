#pragma once

#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_registration/ndt_matcher_d2d_feature.h>

namespace lslgeneric {


  bool matchFusion( NDTMap& targetNDT,
		    NDTMap& sourceNDT,
		    NDTMap& targetNDT_feat,
		    NDTMap& sourceNDT_feat,
		    const std::vector<std::pair<int, int> > &corr_feat,
		    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T ,
		    bool useInitialGuess, bool useNDT, bool useFeat, bool step_control, int ITR_MAX = 30, int n_neighbours = 2, double DELTA_SCORE = 10e-4)
{
  std::cerr << "matchFusion()" << " useNDT : " << useNDT << " useFeat : " << useFeat << " step_control : " << step_control << std::endl;

  // Combines two different NDT maps at once. One which holds NDT derrived from features with known correspondance (obtained earlier through RANSAC or similar) with two standard NDT maps (target could very well be obtained from the fuser).
  
  // Create matching objects. The matching is done below but this is only used to get access to compute derrivatives etc.
  NDTMatcherD2D matcher_d2d;
  NDTMatcherFeatureD2D matcher_feat_d2d(corr_feat);

  matcher_d2d.n_neighbours = n_neighbours;

    //locals
    bool convergence = false;
    //double score=0;
    double score_best = INT_MAX;
    //    double DELTA_SCORE = 0.0005;
    //int ITR_MAX = 30;
    //    bool step_control = true;
    //double NORM_MAX = current_resolution, ROT_MAX = M_PI/10; //
    int itr_ctr = 0;
    //double alpha = 0.95;
    double step_size = 1;
    Eigen::Matrix<double,6,1>  pose_increment_v, scg;
    Eigen::MatrixXd Hessian(6,6), score_gradient(6,1); //column vectors, pose_increment_v(6,1)
    Eigen::MatrixXd Hessian_ndt(6,6), score_gradient_ndt(6,1); //column vectors, pose_increment_v(6,1)
    Eigen::MatrixXd Hessian_feat(6,6), score_gradient_feat(6,1); //column vectors, pose_increment_v(6,1)
 
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR, Tbest;
    Eigen::Vector3d transformed_vec, mean;
    bool ret = true;
    if(!useInitialGuess)
    {
        T.setIdentity();
    }
    Tbest = T;
    

    Eigen::Array<double,6,1> weights;
    std::vector<NDTCell*> nextNDT = sourceNDT.pseudoTransformNDT(T);
    std::vector<NDTCell*> nextNDT_feat = sourceNDT_feat.pseudoTransformNDT(T);

    //std::cout<<"pose(:,"<<1<<") = ["<<T.translation().transpose()<<" "<<T.rotation().eulerAngles(0,1,2).transpose()<<"]';\n";
    //    std::cout << "score_best : " << score_best << std::endl;

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
	//	std::cerr << "score_here_ndt : " << score_here_ndt << "\t score_here_feat : " << score_here_feat << std::endl;

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

        //	std::cout << "score_gradient_ndt : " << score_gradient_ndt << std::endl;
        //	std::cout << "score_gradient_feat : " << score_gradient_feat << std::endl;
        //	std::cout << "score_gradient : " << score_gradient << std::endl;

        //derivativesNDT(nextNDT,targetNDT,score_gradient,Hessian,true);
        scg = score_gradient;
	//	std::cout<<"itr "<<itr_ctr<<std::endl;
	if(score_here < score_best) 
	{
	    Tbest = T;
	    score_best = score_here;
	    std::cout<<"best score "<<score_best<<" at "<<itr_ctr<<std::endl;
	}

        //		std::cout<<"T translation "<<T.translation().transpose()
        //			 <<" (norm) "<<T.translation().norm()<<std::endl;
        //		std::cout<<"T rotation "<<T.rotation().eulerAngles(0,1,2).transpose()
        //			 <<" (norm) "<<T.rotation().eulerAngles(0,1,2).norm()<<std::endl;
	


//	Hessian = Hessian + score_gradient.norm()*Eigen::Matrix<double,6,6>::Identity();
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
	    //            std::cerr<<"regularizing\n";
        }
	//	std::cout<<"s("<<itr_ctr+1<<") = "<<score_here<<";\n";
        // std::cout<<"H(:,:,"<<itr_ctr+1<<")  =  ["<< Hessian<<"];\n"<<std::endl;				  //
        // std::cout<<"H_ndt(:,:,"<<itr_ctr+1<<")  =  ["<< Hessian_ndt<<"];\n"<<std::endl;				  //
        // std::cout<<"H_feat(:,:,"<<itr_ctr+1<<")  =  ["<< Hessian_feat<<"];\n"<<std::endl;				  //
        // std::cout<<"H_inv(:,:,"<<itr_ctr+1<<") = [" << Hessian.inverse()<< "];\n"<<std::endl;
        // std::cout<<"H*H_inv(:,:,"<<itr_ctr+1<<") = [" << Hessian*Hessian.inverse()<< "];\n"<<std::endl;
        // std::cout<<"H_ndt_inv(:,:,"<<itr_ctr+1<<") = [" << Hessian_ndt.inverse()<< "];\n"<<std::endl;
        
	//	std::cout<<"grad (:,"<<itr_ctr+1<<")= ["<<score_gradient.transpose()<<"];"<<std::endl;         //

        if (score_gradient.norm()<= DELTA_SCORE)
        {
	  //	    std::cout<<"incr(:,"<<itr_ctr+1<<") = [0 0 0 0 0 0]';\n";
	  std::cout<<"\%gradient vanished\n";
	    if(score_here > score_best) 
	    {
	      //		std::cout<<"crap iterations, best was "<<score_best<<" last was "<<score_here<<std::endl;
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

	    std::cout<<"itr "<<itr_ctr<<" dScore "<< 0 <<std::endl;
            return true;
        }
        pose_increment_v = -Hessian.ldlt().solve(score_gradient);
        double dginit = pose_increment_v.dot(scg);
        if(dginit > 0)
        {
	  //	    std::cout<<"incr(:,"<<itr_ctr+1<<") = ["<<pose_increment_v.transpose()<<"]';\n";
	  //	    std::cout<<"\%dg  =  "<<dginit<<std::endl;     //
	  //            std::cout<<"\%can't decrease in this direction any more, done \n";
            //de-alloc nextNDT
	    if(score_here > score_best) 
	    {
	      //		std::cout<<"crap iterations, best was "<<score_best<<" last was "<<score_here<<std::endl;
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
	//	std::cout<<"score("<<itr_ctr+1<<") = "<<score_here<<";\n";

	if(step_control) {
	  // TODO - add feat?
	  step_size = 0.;
	  if (useNDT)
	    step_size += matcher_d2d.lineSearchMT(pose_increment_v,nextNDT,targetNDT);
	  if (useFeat)
	    step_size += matcher_feat_d2d.lineSearchMT(pose_increment_v,nextNDT_feat, targetNDT_feat);
	} else {
	    step_size = 1;
	}
        pose_increment_v = step_size*pose_increment_v;
	//        std::cout<<"\%iteration "<<itr_ctr<<" pose norm "<<(pose_increment_v.norm())<<" score "<<score_here<<" step "<<step_size<<std::endl;

        TR.setIdentity();
        TR =  Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
              Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;

	//        std::cout<<"incr= ["<<pose_increment_v.transpose()<<"]"<<std::endl;
        //transform source NDT
        T = TR*T;
	//	std::cout<<"incr(:,"<<itr_ctr+1<<") = ["<<pose_increment_v.transpose()<<"]';\n";
	//	std::cout<<"pose(:,"<<itr_ctr+2<<") = ["<<T.translation().transpose()<<" "<<T.rotation().eulerAngles(0,1,2).transpose()<<"]';\n";

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
        //std::cout<<"step size "<<step_size<<std::endl;
    }
    
//    std::cout<<"itr "<<itr_ctr<<" dScore "<< pose_increment_v.norm()<<std::endl;
    //std::vector<NDTCell*> nextNDT = sourceNDT.pseudoTransformNDT(T);

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
    
    if(score_here > score_best) 
    {
      //	std::cout<<"crap iterations, best was "<<score_best<<" last was "<<score_here<<std::endl;
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

//    this->finalscore = score/NUMBER_OF_ACTIVE_CELLS;

    return ret;
}



  bool matchFusion2d( NDTMap& targetNDT,
                      NDTMap& sourceNDT,
                      NDTMap& targetNDT_feat,
                      NDTMap& sourceNDT_feat,
                      const std::vector<std::pair<int, int> > &corr_feat,
                      Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T ,
                      bool useInitialGuess, bool useNDT, bool useFeat, bool step_control, int ITR_MAX = 30, int n_neighbours = 2, double DELTA_SCORE = 10e-4)
  {
    std::cerr << "matchFusion2d()" << " useNDT : " << useNDT << " useFeat : " << useFeat << " step_control : " << step_control << std::endl;
    
    // Only use the standard NDT mapping to start with.
    NDTMatcherD2D_2D matcher_d2d_2d;
    matcher_d2d_2d.n_neighbours = n_neighbours;
    matcher_d2d_2d.step_control = step_control;
    matcher_d2d_2d.ITR_MAX = ITR_MAX;
    matcher_d2d_2d.DELTA_SCORE = DELTA_SCORE;
    return matcher_d2d_2d.match(targetNDT, sourceNDT, T, useInitialGuess);
}






} // namespace
