#ifndef USAC_TERMINATIONCRITERIA_H
#define USAC_TERMINATIONCRITERIA_H


class TerminationCriteria {
protected:
    bool isinit = false;
public:
    bool isInit () { return isinit; }
    virtual void init (const Model * const model, unsigned int points_size) {
        std::cout << "NOT IMPLEMENTED init IN TERMINATION CRITERIA\n";
    }
    virtual unsigned int getUpBoundIterations (unsigned int inlier_size) = 0;
    virtual unsigned int getUpBoundIterations (unsigned int inlier_size, unsigned int points_size) = 0;
};
#endif //USAC_TERMINATIONCRITERIA_H
