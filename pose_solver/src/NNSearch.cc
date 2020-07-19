/**
 * Nearest Neighbor Search 
 * created date: 2020.4.13 13:51
 * by Dinger
 */

#include "NNSearch.hpp"

int NNSearch::getDistance(int& min_id, double& min_dist, const LightBarP& lbp) const {
    for (size_t j=0; j<targetLBPs.size(); j++) {
        if (!found[j]) {
            if (targetLBPs[j].lb_id >= 0 && lbp.lb_id >= 0) {
                if (lbp.lb_id != targetLBPs[j].lb_id)
                    continue;
            }
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
    lbp.lb_id = source[i].isLeft;
    // better to use kd tree
    int min_id = -1;
    double min_dist = 1e10;
    getDistance(min_id, min_dist, lbp);
    heap.emplace(i, min_id, min_dist);
    return 0;
}

int NNSearch::setSource(const std::vector<aim_deps::Light>& source) {
    for (size_t i=0; i<source.size(); ++i) {
        if (source[i].isLeft < 0)
            continue;
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
    if (heap.empty())
        return 1;
    std::vector<Lbp_ptr> unknown_ptr;
    for (int cnt=0; cnt<2; cnt++) {
        if (heap.empty())
            break;
        while (heap.top().distance < min_dist) {
            Lbp_ptr tmp = heap.top();
            // std::cout << "id: " << tmp.id << " tar: " << tmp.tar_id << " dist: " << tmp.distance << "\n";
            heap.pop();
            if (cnt < 1) {
                if (source[tmp.id].isLeft < 0) {
                    unknown_ptr.push_back(tmp);
                    if (heap.empty())
                        break;
                    continue;
                }
            }
            const aim_deps::LightBox &lb = source[tmp.id].box;
            LightBarP lbp(lb.center, lb.vex[0] - lb.center);
            lbp.lb_id = source[tmp.id].isLeft;
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
        if (cnt < 1) {
            for (const Lbp_ptr& tmp : unknown_ptr) {
                const aim_deps::LightBox &lb = source[tmp.id].box;
                LightBarP lbp(lb.center, lb.vex[0] - lb.center);
                lbp.lb_id = source[tmp.id].isLeft;
                int min_id = -1;
                double min_dist = 1e10;
                getDistance(min_id, min_dist, lbp);
                heap.emplace(tmp.id, min_id, min_dist);
            }
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
