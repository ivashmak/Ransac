#include "Tests.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Sampler/ProsacSampler.h"
#include "../Usac/Sampler/EvsacSampler.h"
#include "../Usac/Sampler/NapsacSampler.h"
#include "../Usac/Sampler/GradualNapsacSampler.h"

void Tests::initProsac (Sampler *& sampler, unsigned int sample_number, unsigned int points_size) {
    sampler = new ProsacSampler;
    ProsacSampler * prosac_sampler = (ProsacSampler *)sampler;
    prosac_sampler->initProsacSampler (sample_number, points_size);
}

void Tests::initUniform (Sampler *& sampler, unsigned int sample_number, unsigned int points_size) {
    sampler = new UniformSampler;
    sampler->setSampleSize(sample_number);
    sampler->setPointsSize(points_size);
}

void Tests::initNapsac (Sampler *& sampler, const cv::Mat &neighbors, const std::vector<std::vector<int>> &ns, Model * model) {
    int points_size = std::max ((int) neighbors.rows, (int) ns.size());

    sampler = new NapsacSampler(model, points_size);
    if (model->neighborsType == NeighborsSearch::Nanoflann) {
        assert(! neighbors.empty());
        ((NapsacSampler *) sampler)->setNeighbors(neighbors);
    } else {
        assert(! ns.empty());
        ((NapsacSampler *) sampler)->setNeighbors(ns);
    }

}

void Tests::initEvsac (Sampler *& sampler, cv::InputArray points, unsigned int sample_number,
                       unsigned int points_size, unsigned int k_nearest_neighbors) {

    sampler = new EvsacSampler(points, points_size, k_nearest_neighbors, sample_number);
}

void Tests::initGraduallyIncreasingSampler (Sampler *& sampler, cv::InputArray points, unsigned int sample_number) {
    sampler = new GradualNapsacSampler(points, sample_number);
}

void Tests::initProsacNapsac1 (Sampler *& sampler, Model * model, const cv::Mat &nearest_neighors) {

}

void Tests::initProsacNapsac2 (Sampler *& sampler, Model * model, const cv::Mat &nearest_neighors) {

}

void Tests::initSampler (Sampler *& sampler, Model * model, unsigned int points_size, cv::InputArray points, const cv::Mat& neighbors, std::vector<std::vector<int>> ns) {
    Tests tests;
    if (model->sampler == SAMPLER::Uniform) {
        tests.initUniform(sampler, model->sample_number, points_size);
    } else if (model->sampler == SAMPLER::Prosac) {
        tests.initProsac(sampler, model->sample_number, points_size);
    } else if (model->sampler == SAMPLER::Napsac) {
        assert(model->k_nearest_neighbors > 0);
        tests.initNapsac(sampler, neighbors, ns, model);
    } else if (model->sampler == SAMPLER::Evsac) {
        assert(model->k_nearest_neighbors > 0);
        tests.initEvsac(sampler, points, model->sample_number, points_size, model->k_nearest_neighbors);
    } else if (model->sampler == SAMPLER::GradualNapsac) {
        tests.initGraduallyIncreasingSampler(sampler, points, model->sample_number);
    } else {
        std::cout << "UNKOWN SAMPLER\n";
        exit (100);
    }
}