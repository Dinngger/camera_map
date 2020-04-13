/**
 * Nearest Neighbor Search 
 * created date: 2020.4.13 13:51
 * by Dinger
 */

#ifndef __NNSEARCH_HPP
#define __NNSEARCH_HPP

#include <queue>
#include <set>
#include "AimDeps.hpp"
#include "CarModule.hpp"

struct Lbp_ptr {
    int id;
    int tar_id;
    double distance;
    Lbp_ptr(int id, int tar_id, double distance) :
        id(id), tar_id(tar_id), distance(distance) {}
    bool operator>(const Lbp_ptr &x) const {
        return distance > x.distance;
    }
};

class NNSearch {
private:
    std::vector<bool> found;
    std::vector<LightBarP> targetLBPs;
    std::vector<LightBarP> resultLBPs;
    std::priority_queue<Lbp_ptr, std::vector<Lbp_ptr>, std::greater<Lbp_ptr>> heap;
    int getDistance(int& min_id, double& min_dist, const LightBarP& lbp) const;
    int addToHeap(const std::vector<aim_deps::Light>& source, int i);
    int setSource(const std::vector<aim_deps::Light>& source);
    int setSource(const std::vector<aim_deps::Light>& source, std::set<int> id_set);
    int searchLoop(const std::vector<aim_deps::Light>& source, double min_dist);
public:
    std::vector<LightBarP>& getTargetLBPs() {
        return targetLBPs;
    }
    const std::vector<LightBarP>& getResultLBPs() const {
        return resultLBPs;
    }
    int finishSetTarget();
    int runSearch(const std::vector<aim_deps::Light>& source, double min_dist);
    int runSearch(const std::vector<aim_deps::Light>& source, double min_dist, std::set<int> id_set);
    int getHeapIdSet(std::set<int> &id_set);
};

#endif // __NNSEARCH_HPP