/**
 * Nearest Neighbor Search 
 * created date: 2020.4.13 13:51
 * by Dinger
 */

#ifndef __NNSEARCH_HPP
#define __NNSEARCH_HPP

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

int NNSearch::getDistance(int& min_id, double& min_dist, const LightBarP& lbp) const {
    for (size_t j=0; j<targetLBPs.size(); j++) {
        if (!found[j]) {
            double dist = (lbp.center - targetLBPs[j].center).norm();
            if (dist < min_dist) {
                min_dist = dist;
                min_id = j;
            }
        }
    }
    return 0;
}

int NNSearch::finishSetTarget() {
    for (size_t i=0; i<targetLBPs.size(); i++)
        found.emplace_back(false);
    return 0;
}

int NNSearch::addToHeap(const std::vector<aim_deps::Light>& source, int i) {
    const aim_deps::LightBox &lb = source[i].box;
    LightBarP lbp(lb.center, lb.vex[0] - lb.center);
    // better to use kd tree
    int min_id = -1;
    double min_dist = 1e10;
    getDistance(min_id, min_dist, lbp);
    heap.emplace(i, min_id, min_dist);
    return 0;
}

int NNSearch::setSource(const std::vector<aim_deps::Light>& source) {
    for (size_t i=0; i<source.size(); ++i) {
        addToHeap(source, i);
    }
    return 0;
}

int NNSearch::setSource(const std::vector<aim_deps::Light>& source, std::set<int> id_set) {
    for (int i : id_set) {
        addToHeap(source, i);
    }
    return 0;
}

int NNSearch::searchLoop(const std::vector<aim_deps::Light>& source, double min_dist) {
    while (heap.top().distance < min_dist) {
        Lbp_ptr tmp = heap.top();
        // std::cout << "id: " << tmp.id << " tar: " << tmp.tar_id << " dist: " << tmp.distance << "\n";
        heap.pop();
        const aim_deps::LightBox &lb = source[tmp.id].box;
        LightBarP lbp(lb.center, lb.vex[0] - lb.center);
        if (!found[tmp.tar_id]) {
            found[tmp.tar_id] = true;
            resultLBPs.emplace_back(targetLBPs[tmp.tar_id], lbp);
            if (heap.empty())
                break;
        } else {
            // better to use kd tree
            int min_id = -1;
            double min_dist = 1e10;
            getDistance(min_id, min_dist, lbp);
            heap.emplace(tmp.id, min_id, min_dist);
        }
    }
    return 0;
}

int NNSearch::runSearch(const std::vector<aim_deps::Light>& source, double min_dist) {
    setSource(source);
    searchLoop(source, min_dist);
    return 0;
}

int NNSearch::runSearch(const std::vector<aim_deps::Light>& source, double min_dist, std::set<int> id_set) {
    setSource(source, id_set);
    searchLoop(source, min_dist);
    return 0;
}

int NNSearch::getHeapIdSet(std::set<int> &id_set) {
    while (!heap.empty()) {
        id_set.insert(heap.top().id);
        heap.pop();
    }
    return 0;
}

#endif // __NNSEARCH_HPP