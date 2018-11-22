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
    sampler->initRandomGenerator();
}

void Tests::initNapsac (Sampler *& sampler, const cv::Mat &neighbors, unsigned int k_nearest_neighbors,
                        unsigned int sample_number) {

    sampler = new NapsacSampler((int *)neighbors.data, k_nearest_neighbors, sample_number, neighbors.rows);
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
