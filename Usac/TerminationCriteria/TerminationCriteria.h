#ifndef USAC_TERMINATIONCRITERIA_H
#define USAC_TERMINATIONCRITERIA_H


class TerminationCriteria {
protected:
    bool isinit = false;
public:
    bool isInit () { return isinit; }
    virtual void init (const Model * const model) {}
    virtual unsigned int getUpBoundIterations (float inlier_points, float total_points) = 0;

};
#endif //USAC_TERMINATIONCRITERIA_H
