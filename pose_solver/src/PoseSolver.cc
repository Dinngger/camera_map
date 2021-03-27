/**
 * Top module of Pose Solver
 * created date: 2020.3.3
 * by Dinger
 */

#include "PoseSolver.hpp"

Armor3d toArmor3d(const aim_deps::Armor& armor) {
    Armor3d _armor;
    _armor.t = Eigen::Vector3d(armor.t_vec.x / 1000, armor.t_vec.y / 1000, armor.t_vec.z / 1000);
    cv::Mat rotation;
    cv::Rodrigues(armor.r_vec, rotation);
    Eigen::Map<Eigen::Matrix3d> eR((double*)rotation.data);
    eR.transposeInPlace();
    _armor.r = Eigen::Quaterniond(eR);
    return _armor;
}

PoseSolver::PoseSolver(cv::Matx<double, 3, 3> &K, double time) : 
    K (K),
    module(K, time),
    new_module(time)
{}

int PoseSolver::newrun(const cv::Mat &frame, double time)
{
    match.saveImg(frame);
    match.findPossible();
    amp.matchAll(match.matches, match.possibles, tar_list);//查找匹配灯条
    pos_getter.batchProcess(tar_list);              ///外部pnp解算所有装甲板

    std::vector<Eigen::Vector4d> newcarPossible;
    std::vector<std::vector<cv::Point2f>> PointPossible;
    std::vector<double> y_vec;
    for(size_t i=0;i<tar_list.size();i++)
    {
        #define pi 3.1415926
        aim_deps::Armor armor=tar_list[i];
        cv::Mat rotM;
        cv::Rodrigues(armor.r_vec, rotM);  //将旋转向量变换成旋转矩阵
        double theta_x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2));
        printf("tanx=%lf\n",theta_x);
        // if(!((theta_x>2.8) || (theta_x<-2.8)))
        //      continue;
        // for(int c=0;c<4;c++)
        // {
        //     cv::line(frame, armor.vertex[c], 
        //     armor.vertex[(c+1)%4], cv::Scalar(0, 255, 255), 1);
        // }
        double x,z;
        x=armor.t_vec.x-0.2*sin(theta_x);
        z=armor.t_vec.z+0.2*cos(theta_x);
        y_vec.push_back(armor.t_vec.y);
        Eigen::Vector4d vect4(x,z,theta_x,0);
        newcarPossible.push_back(vect4);
        std::vector<cv::Point2f> pointvec; 
        for(int j=0;j<4;j++)
            pointvec.push_back(armor.vertex[j]);
        PointPossible.push_back(pointvec);
    }
    std::vector<Eigen::Vector4d> predictvec;        //第一二位为x、z 第三位 theta_x
    for(size_t i=0;i<new_module.newcars.size();i++)
    {
        newcar newCar=new_module.newcars[i];
        double pre_r;
        Eigen::Vector3d pre_t;
        newCar.predict(time-new_module.module_time,pre_r,pre_t);
        Eigen::Vector4d pre(pre_t[0],pre_t[2],pre_r,0);
        predictvec.push_back(pre);
    }
    std::vector<Eigen::Vector3d> dists;
    std::cout<<"\033[36mmatch:\n";
    for(size_t i=0;i<newcarPossible.size();i++){
        for(size_t j=0;j<predictvec.size();j++){
            double distance = sqrt(
            (newcarPossible[i][0]-predictvec[j][0])*
            (newcarPossible[i][0]-predictvec[j][0])+
            (newcarPossible[i][1]-predictvec[j][1])*
            (newcarPossible[i][1]-predictvec[j][1])
            );
            //printf("%lf %lf %lf %lf distance:%lf\n",newcarPossible[i][0],predictvec[j][0],newcarPossible[i][1],predictvec[j][1],distance);
            dists.push_back(Eigen::Vector3d(distance,(double)i,(double)j));
        }
    }
    std::cout<<"\033[0m";
    std::sort(dists.begin(), dists.end(),
        [&](const Eigen::Vector3d &d1, const Eigen::Vector3d& d2) {
            return d1[0] < d2[0];
        }
    );
    int matches[20];
    for(size_t i=0;i<dists.size();i++)
    {
        //printf("dists[%lu]=%lf\n",i,dists[i][0]);
        // if(dists[i][0]>60)
        //      break;
        if(newcarPossible[int(dists[i][1])][3]==0&&predictvec[int(dists[i][2])][3]==0)
        {
            newcarPossible[dists[i][1]][3]=1;
            predictvec[dists[i][2]][3]=1;
            matches[int(dists[i][1])]=int(dists[i][2]);
        }
    }
    std::vector<newcar>last_newcars;
    for(size_t i=0;i<new_module.newcars.size();i++)
        last_newcars.push_back(new_module.newcars[i]);
    new_module.newcars.clear();
    for(size_t i=0;i<newcarPossible.size();i++)
    {

        if(newcarPossible[i][3]){
            int temp=matches[i];
            pastFrame *p, *q;
            p=last_newcars[temp].frameHead;
            pastFrame *n;
            n=(pastFrame*)malloc(sizeof(pastFrame));
            for(size_t j=0;j<4;j++)
            {
                n->vertex[j]=PointPossible[i][j];
                last_newcars[temp].NowPoints[j]=PointPossible[i][j];
                //printf("lnctnp=%lf\n",last_newcars[temp].NowPoints[j].x);
                //std::cout<<"NowPoints["<<j<<"]: "<<last_newcars[temp].NowPoints[j]<<std::endl;
            }
            n->next=NULL;
            while(p->next)
            {
                p=p->next;
            }
            p->next=n;
            last_newcars[temp].frameNumber++;
            if(last_newcars[temp].frameNumber==11)
            {
                q=p->next;
                p->next=q->next;
                free(q);
                last_newcars[temp].frameNumber--;
            }
            //std::cout<<"last_newcars["<<temp<<"].frameNumber: "<<last_newcars[temp].frameNumber<<std::endl;
            last_newcars[temp].bundleAdjustment(time-new_module.module_time);
            last_newcars[temp].update_state(time-new_module.module_time);
            new_module.newcars.push_back(last_newcars[temp]);
        }
        else{
            new_module.add_newcar(Eigen::Vector3d(newcarPossible[i][0],y_vec[i],newcarPossible[i][1]),
            newcarPossible[i][2],PointPossible[i]);
        }
    }
    std::cout<<"newcarPossible.size():"<<newcarPossible.size()<<std::endl;
    std::cout<<"new_module.newcars.size():"<<new_module.newcars.size()<<std::endl;
    return 0;
}
int PoseSolver::run(const cv::Mat &frame, double time)
{
    match.saveImg(frame);
    match.findPossible();
    amp.matchAll(match.matches, match.possibles, tar_list);//查找匹配灯条
    pos_getter.batchProcess(tar_list);              ///外部pnp解算所有装甲板

    carMatch.transformerMatch(match.possibles);
    if (carMatch.carsPossible.size() < 1)
        return 0;
    std::vector<std::vector<LightBarP> > carsPossible;
    for (uint i = 0, n_cars = 0; i < carMatch.carsPossible.size(); i++) {
        if (carMatch.carsPossible[i].first < 0)
            continue;
        carsPossible.push_back(std::vector<LightBarP>());

        carsPossible.back().clear();

        if (carMatch.carsPossible[i].lightPossibles.size() <= 1) {
            n_cars ++;
            continue;
        }
        int first_index = carMatch.carsPossible[i].first_index;
        std::cout << "car: " << carMatch.carsPossible[i].first << " light: ";
        for (size_t j=0; j<carMatch.carsPossible[i].lightPossibles.size(); j++) {
            const aim_deps::LightBox &lb = carMatch.carsPossible[i].lightPossibles[j].box;
            LightBarP lbp(lb.center, lb.vex[0] - lb.center);
            if ((j & 0x1) == (first_index & 0x1))               // 与index同奇偶性，则标志应该相同
                lbp.lb_id = carMatch.carsPossible[i].first;
            else                                                // 与index不同奇偶性，则标志不同
                lbp.lb_id = 1 - carMatch.carsPossible[i].first;
            // hqy yyds!
            std::cout << lbp.lb_id << " ";
            carsPossible[n_cars].emplace_back(lbp);
        }
        n_cars ++;
        std::cout << "\n";
    }

    std::vector<LightBarP> predict2d;
    module.create_predict(time, predict2d);

    std::cout << "\033[33m" << "predict result: ";
    for (const LightBarP& temp_lbp : predict2d)
        std::cout << temp_lbp.car_id << temp_lbp.armor_id << temp_lbp.lb_id << ", ";
    std::cout << "\033[0m\n";

    for (std::vector<LightBarP>& car : carsPossible) {
        if (car.size() <= 1)
            continue;
        for (LightBarP& lbp : car) {
            double min_dist = 1e5;
            int min_id = -1;
            for (size_t j=0; j<predict2d.size(); j++) {
                if (predict2d[j].lb_id >= 0 && lbp.lb_id >= 0) {
                    if (lbp.lb_id != predict2d[j].lb_id)
                        continue;
                }
                double dist = (lbp.center - predict2d[j].center).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    min_id = j;
                }
            }
            if (min_id >= 0 && min_dist < 60) {
                lbp.car_id = predict2d[min_id].car_id;
                lbp.armor_id = predict2d[min_id].armor_id;
                lbp.lb_id = predict2d[min_id].lb_id;
            }
        }
    }

    for (std::vector<LightBarP>& car : carsPossible) {
        if (car.size() <= 1)
            continue;
        std::cout << "car: ";
        std::map<int, int> car_id_num;
        for (LightBarP& lbp : car) {
            if (car_id_num.count(lbp.car_id) == 0) {
                car_id_num[lbp.car_id] = 1;
            } else {
                car_id_num[lbp.car_id]++;
            }
        }
        int max_car_id = -1, max_car_num = 0;
        for (const std::pair<int, int>& pair : car_id_num) {
            if (pair.second > max_car_num) {
                max_car_id = pair.first;
                max_car_num = pair.second;
            }
        }
        for (LightBarP& lbp : car) {
            lbp.car_id = max_car_id;
        }
        std::cout << max_car_id << " armor: ";

        std::map<int, int> armor_id_num;
        for (size_t i=0; i<car.size(); i++) {
            int _armor_id;
            std::cout << car[i].armor_id << ":" << car[i].lb_id << " ";
            if (car[i].armor_id < 0)
                _armor_id = (car[i].lb_id + i) % 2 - 2;
            else
                _armor_id = (car[i].armor_id * 2 + car[i].lb_id - i + 8) % 8;
            if (armor_id_num.count(_armor_id) == 0) {
                armor_id_num[_armor_id] = 1;
            } else {
                armor_id_num[_armor_id]++;
            }
        }
        int max_armor_id = -1, max_armor_num = 0;
        for (const std::pair<int, int>& pair : armor_id_num) {
            if (pair.second > max_armor_num) {
                max_armor_id = pair.first;
                max_armor_num = pair.second;
            }
        }
        if (max_armor_id < 0)
            max_armor_id += 2;
        std::cout << "final_lb: ";
        for (size_t i=0; i<car.size(); i++) {
            car[i].armor_id = ((max_armor_id + i) % 8) / 2;
            car[i].lb_id = (max_armor_id + i) % 2;
            std::cout << car[i].armor_id << car[i].lb_id << " ";
        }
        std::cout << "\n";
    }

    module.bundleAdjustment(carsPossible, time);
    printf("finished ba\n");
    return 0;
}

int PoseSolver::getTwcs(std::vector<cv::Mat> &Twcs)
{
    std::vector<cv::Mat> rMats, tMats;
    pos_getter.packUp(rMats, tMats, tar_list);   ///取得rMats, tMats(内部clear这两个Mat容器)
    std::cout << "armor number: " << rMats.size() << "\n";
    for (size_t i=0; i<rMats.size(); i++) {
        cv::Mat temp =(cv::Mat_<double>(1,4) << 0,0,0,1);
        cv::Mat temp2, Twc;
        cv::hconcat(rMats[i], tMats[i] / 1000, temp2);
        cv::vconcat(temp2, temp, Twc);
        Twcs.push_back(Twc);
    }
    return 0;
}

int PoseSolver::draw(cv::Mat &frame)
{
    match.drawLights(frame, carMatch.r_trans);							//绘制所有灯条
    amp.drawArmorPlates(frame, tar_list, 0);		    //绘制装甲板
    return 0;
}

int PoseSolver::get_lbs(std::vector<cv::Point3d> &lbs)
{
    // module.get_lbs(lbs);
    new_module.get_lbs(lbs);
    return 0;
}

void PoseSolver::drawNewCarModule(cv::Mat &frame)
{
    //printf("Drawing: %lu\n",new_module.newcars.size());
    for(size_t i=0;i<new_module.newcars.size();i++) {
        for (int j = 0; j < 4; ++j) {
            cv::line(frame, new_module.newcars[i].NowPoints[j], 
            new_module.newcars[i].NowPoints[(j + 1) % 4], cv::Scalar(0, 255, 255), 1);
            //printf("NowPoints[%d]=(%lf,%lf)\n",j,new_module.newcars[i].NowPoints[j].x,new_module.newcars[i].NowPoints[j].y);   
        }
    }
}
