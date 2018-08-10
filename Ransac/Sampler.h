#ifndef RANSAC_SAMPLER_H
#define RANSAC_SAMPLER_H

class Sampler {
public:
    virtual void getSample (int *points, int npoints, int total_points) = 0;
};

#endif //RANSAC_SAMPLER_H