#ifndef USAC_TERMINATIONCRITERIA_H
#define USAC_TERMINATIONCRITERIA_H


class TerminationCriteria {
public:
    virtual void init (const Model * const model) {}
    virtual unsigned int getUpBoundIterations (float inlier_points, float total_points) = 0;

};
#endif //USAC_TERMINATIONCRITERIA_H
