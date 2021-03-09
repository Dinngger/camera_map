#include "../include/CarMatch.hpp"

const std::string head_path = "/home/sentinel/camera_map/fifos/";

CarMatch::CarMatch() {
    const std::string input = head_path + "input.pipe";
    const std::string output = head_path + "output.pipe";
    if (access(input.c_str(), F_OK) != 0 || access(output.c_str(), F_OK) != 0) {
        printf("Error: broken pipe.\n");
        exit(-1);
    }
    mkfifo(input.c_str(), 0777);
    mkfifo(output.c_str(), 0777);
    in_fd = open(input.c_str(), O_SYNC | O_CREAT | O_RDWR);
    out_fd = open(output.c_str(), O_RDONLY);
}

void CarMatch::transformerMatch(const std::vector<aim_deps::Light> &hLights) {
    std::vector<aim_deps::Light> lights;
    lights.assign(hLights.begin(), hLights.end());
    std::sort(lights.begin(), lights.end(), 
        [&](const aim_deps::Light& l1, const aim_deps::Light& l2) {
            return l1.box.center.x < l2.box.center.x;
        }
    );
    memset(w_trans.data, 0, BUFFER_SIZE * sizeof(float));
    for (size_t i = 0; i < lights.size(); i++) {
        const cv::Point2f& center = lights[i].box.center;
        cv::Point2f vec = lights[i].box.vex[0] - center;
        w_trans.data[4 * i] = center.x;
        w_trans.data[4 * i + 1] = center.y; 
        w_trans.data[4 * i + 2] = vec.x;
        w_trans.data[4 * i + 3] = vec.y;
        w_trans.data[EXIST_INDEX + i] = 1;
    }
    write(in_fd, (void *)w_trans.buf, 4 * BUFFER_SIZE);

    /// TODO: 超市检测
    size_t total_num = 0;
    std::map<int, int> car_map;
    int n_car = 0;
    while (true) {
        total_num += read(out_fd, (void * )(r_trans + total_num), 13 - total_num);
        if (total_num < 13) {
            usleep(10);
            continue;
        }
        for (size_t i = 0; i < lights.size(); i++) {
            if (r_trans[i] > 0) {
                if (car_map.count(r_trans[i]) == 0) {
                    n_car++;
                    CarPossible cp;
                    if (lights[i].isLeft != -1) {
                        cp.first = lights[i].isLeft ? 0 : 1;
                    }
                    cp.lightPossibles.push_back(lights[i]);
                    carsPossible.push_back(cp);
                    car_map[r_trans[i]] = carsPossible.size() - 1;
                } else {
                    if (lights[i].isLeft != -1 && carsPossible[car_map[r_trans[i]]].first == -1) {
                        carsPossible[car_map[r_trans[i]]].first = lights[i].isLeft ? 0 : 1;
                    }
                    carsPossible[car_map[r_trans[i]]].lightPossibles.push_back(lights[i]);
                }
            }
        }
        break;
    }
}