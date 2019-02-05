#include "init.hpp"

void InitEstimator (Estimator *& estimator, ESTIMATOR est, const cv::Mat& points) {
    if (est == ESTIMATOR::Line2d) {
        estimator = new Line2DEstimator(points);

    } else if (est == ESTIMATOR::Homography) {
	    estimator = new HomographyEstimator(points);

    } else if (est == ESTIMATOR::Fundamental) {
	    estimator = new FundamentalEstimator(points);

    } else if (est == ESTIMATOR::Essential) {
	    estimator = new EssentialEstimator(points);

    } else {
        std::cout << "UNKOWN Estimator IN Init Estimator\n";
        exit (111);
    }
}

// ----------------------------------------------------------------------------------------
void InitSampler (Sampler *& sampler, const Model * const model, cv::InputArray points, 
	cv::InputArray neighbors) {

  //   unsigned int points_size = std::max ((int) neighbors.rows, (int) ns.size());

  //   if (model->sampler == SAMPLER::Uniform) {
  //       sampler = new UniformSampler (model->reset_random_generator);
	 //    sampler->setSampleSize(model->sample_size);
	 //    sampler->setPointsSize(points_size);

  //   } else if (model->sampler == SAMPLER::Prosac) {
  //  	    sampler = new ProsacSampler;
	 //    ProsacSampler * prosac_sampler = (ProsacSampler *)sampler;
	 //    prosac_sampler->initProsacSampler (model->sample_size, points_size);

  //   } else if (model->sampler == SAMPLER::Napsac) {
		// sampler = new NapsacSampler(model, points_size, model->reset_random_generator);
	 //    if (model->neighborsType == NeighborsSearch::Nanoflann) {
	 //        ((NapsacSampler *) sampler)->setNeighbors(neighbors);
	 //    } else {
	 //        ((NapsacSampler *) sampler)->setNeighbors(ns);
	 //    }
	        
  //   } else if (model->sampler == SAMPLER::Evsac) {
	 //    sampler = new EvsacSampler(points, points_size, k_nearest_neighbors, model->sample_size);
    
  //   } else if (model->sampler == SAMPLER::ProgressiveNAPSAC) {
	 //    sampler = new ProgressiveNapsac(points, model->sample_size);
    
  //   } else if (model->sampler == SAMPLER::ProsacNapsac) {
    
  //   } else {	
  //       std::cout << "UNKOWN Sampler IN Init Sampler\n";
  //       exit (111);
  //   }
}
void InitTerminationCriteria (TerminationCriteria *& termination_criteria, 
        const Model * const model, unsigned int points_size) {
    termination_criteria = new StandardTerminationCriteria;
    termination_criteria->init (model, points_size);
}

void InitProsacTerminationCriteria (TerminationCriteria *& termination_criteria, Sampler * prosac_sampler, 
            const Model * const model, Estimator * estimator, unsigned int points_size) {

    termination_criteria = new ProsacTerminationCriteria
                (((ProsacSampler *) prosac_sampler)->getGrowthFunction(), model, points_size, estimator);
}


// void InitLocalOptimization (LocalOptimizatio *& local_optimization, Model * model, Estimator * estimator,
//              Quality * quality, unsigned int points_size) {

//     if (model->lo == LocOpt::InItRsc) {
//         local_optimization = new InnerLocalOptimization (model, estimator, quality, points_size);

//     } else if (model->lo == LocOpt::GC) {


//     } else if (model->lo == LocOpt::IRLS) {
//         local_optimization = new Irls(model, estimator, quality, points_size);

//     } else {
//         std::cout << "UNKOWN LO method in Init\n";
//         exit(111);
//     }
// }