/*
灯条检测模块-找到图上所有合适的灯条,并且将其匹配为装甲板
last date of modification:2020.8.1
*/

#include "LightMatch.hpp"

LightMatch::LightMatch(){
	#ifdef LIGHT_CNT_TIME		
		time_sum 		= 0.0;
		cnt 			= 0;
	#endif // LIGHT_CNT_TIME
	setEnemyColor(true);												//初始设置敌人颜色
    opts.minimizer_type = ceres::LINE_SEARCH;
    opts.line_search_direction_type = ceres::LBFGS;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.line_search_type = ceres::WOLFE;
	opts.logging_type = ceres::SILENT;
    opts.minimizer_progress_to_stdout = false;
    opts.max_num_iterations = 100;
    opts.function_tolerance = 1e-4;
}

LightMatch::~LightMatch(){
	#ifdef LIGHT_CNT_TIME
		std::cout<< "Time for threshold:" << (double)(time_sum / cnt) << std::endl;
	#endif
}

void LightMatch::setEnemyColor(const bool _enemy_blue){
	enemy_blue = _enemy_blue;
	if(enemy_blue){								//设置颜色阈值
		thresh_low = aim_deps::light_params.blue_thresh_low;
		reflect_thresh = aim_deps::light_params.blue_reflection;
		filter_thresh = aim_deps::light_params.blue_filter;
		chan_diff = aim_deps::light_params.blue_green;
		channel_min = aim_deps::light_params.blue_reflect_min;
	}
	else{
		thresh_low = aim_deps::light_params.red_thresh_low;
		reflect_thresh = aim_deps::light_params.red_reflection;
		filter_thresh = aim_deps::light_params.red_filter;
		chan_diff = aim_deps::light_params.red_green;
		channel_min = aim_deps::light_params.red_reflect_min;
	}
}

void LightMatch::findPossible(){			//找出所有可能灯条，使用梯形匹配找出相匹配的灯条对
	#ifdef LIGHT_CNT_TIME
		double start_t = cv::getTickCount();
	#endif	//LIGHT_CNT_TIME
	reset();
	cv::Mat binary(1080, 1440, CV_8UC1);
	threshold(binary);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);	//寻找图上轮廓
	contourProcess(contours);

	#ifdef LIGHT_MATCH_DEBUG
	 	cv::imshow("binary", binary);
	#endif	//LIGHT_MATCH_DEBUG

	for (aim_deps::Light &light: possibles) {
		getTrapezoids(light.box.vex);								
	}

	getRealLight(possibles.size());
	#ifdef LIGHT_CNT_TIME
		double end_t = cv::getTickCount();
		time_sum += (end_t - start_t) / cv::getTickFrequency() * 1000; 
		++cnt;
	#endif	//LIGHT_CNT_TIME
}

void LightMatch::contourProcess(const std::vector<std::vector<cv::Point> >& ct){
	double min_pos = 0.0, max_pos = 0.0; 
	#pragma omp parallel for num_threads(3)
	for (size_t i = 0; i < ct.size(); i++) {	
		float area = cv::contourArea(ct[i]);
		cv::Rect bbox = cv::boundingRect(ct[i]);
		// 轮廓在bbox中占的面积合适 / bbox 本身的面积合适 / bbox 的高不能过小 / bbox的高 - bbox长 > 某一值
		if(bbox.area() <= 20 * area && area >= 4.0 && bbox.height >= 4){
			/// 判定bbox 是否存在宽比高大很多的情况
			if (!isGoodBoundingBox(bbox)){
				continue;
			}
			if(bbox.area() < 400){
				cv::Mat cnt_mat;
				if(enemy_blue){
					cv::threshold(proced[0](bbox), cnt_mat, reflect_thresh, 255, cv::THRESH_BINARY);
					cv::minMaxLoc(proced[0](bbox), &min_pos, &max_pos);
				}
				else{
					cv::threshold(proced[2](bbox), cnt_mat, reflect_thresh, 255, cv::THRESH_BINARY);
					cv::minMaxLoc(proced[2](bbox), &min_pos, &max_pos);
				} 
				/// TODO: 酌情调大
				if(max_pos <= channel_min){
					continue;						// 排除地面全反射灯条
				}
				int num = cv::countNonZero(cnt_mat);				// 排除车体反光灯条的影响
				// 反光灯条的特征是：灯条内亮度过小，灰度值基本不大于210，大于的也绝大多数只有一点
				// 血量显示条上匹配的错误灯条特征：灯条长宽比例不对(弱，不剔除，只指示可能不是灯条)
				double ratio = (double)bbox.width / (double)bbox.height;

				bool valid = (ratio > 1.67 || ratio < 0.6) && (num >= 2);
				doubleThresh(ct[i], bbox, valid);
			}	
			else{		// 选框够大，说明灯条无需二次阈值，亚像素检测以及角度修正
				cv::RotatedRect l = cv::minAreaRect(ct[i]);
				if (l.size.height / l.size.width < 1.6 && l.size.height / l.size.width > 0.6){
					continue;
				}
				aim_deps::Light _l(l, l.center, cv::max(l.size.height, l.size.width), true);
				if(bbox.area() < 2500){		// 在灯条大小不算太大时，长度准确，但角度不准
					std::vector<cv::Point> cont;
					if( !isAngleValid(_l.box) ) {
						continue;		//灯条角度不符合要求
					}
					if(enemy_blue){
						getBigDirection(proced[0](bbox), cont);
						readjustAngle(cont, _l, cv::Point(bbox.x, bbox.y), 1.0);
					}
					else{
						getBigDirection(proced[2](bbox), cont);
						readjustAngle(cont, _l, cv::Point(bbox.x, bbox.y), 1.0);
					}
				}		
				if( std::abs(_l.box.angle) < 40.0){
					mtx.lock();
					_l.index = (int)possibles.size();
					possibles.emplace_back(_l);			
					mtx.unlock();
				}
			}
		}
	}
}

void LightMatch::getRealLight(const int size){
	bool flag[size][size];					//两灯条是否满足要求,是个对称矩阵，当[i][j],[j][i]为真时，两灯条匹配
	for (int i = 0; i < size; ++i) {		//初始化		
        for(int j = 0; j < size; ++j)
		    flag[i][j] = false;
	}
	for (int i = 0; i < size; ++i) {		//梯形包含匹配
		for (int j = 0; j<size*2; ++j) {
			if (isInTrapezoid(possibles[i].box.vex, trapezoids[j])) {
				flag[i][j/2] = true;
			}
		}
	}
	for (int i = 0; i<size; ++i) {						//将可能的匹配结果放入容器matches中
		for (int j = i+1; j<size; ++j) {
			if (flag[i][j] == true && flag[j][i] == true) {
				flag[i][j] = false;
				matches.emplace_back(cv::Point(i, j));		//最后只保存配对的信息，灯条只保存一次
			}
		}
	}
}

//匹配的灯条其梯形将会互相包含
void LightMatch::getTrapezoids(const cv::Point2f corners[2]){
	std::vector<cv::Point2f> left_trapezoid;
	std::vector<cv::Point2f> right_trapezoid;
	cv::Point2f midpoint = (corners[0] + corners[1]) /2;
	cv::Point2f direction_vector;							//方向向量
	float d = sqrt(aim_deps::getPointDist(corners[0], corners[1]));
	cv::Point2f vertical_vector = cv::Point2f(corners[1].y - corners[0].y,
			corners[0].x - corners[1].x);						//获得垂直长方向的方向向量
	vertical_vector = d * aim_deps::LIGHT_PARAM1 * vertical_vector /
			sqrt(vertical_vector.x * vertical_vector.x + vertical_vector.y * vertical_vector.y);
	direction_vector = corners[1] - corners[0];					//平行于长边方向的方向向量
	left_trapezoid.emplace_back(corners[0]);
	left_trapezoid.emplace_back(corners[1]);
	left_trapezoid.emplace_back(midpoint - vertical_vector + aim_deps::LIGHT_PARAM2 * direction_vector);
	left_trapezoid.emplace_back(midpoint - vertical_vector - aim_deps::LIGHT_PARAM2 * direction_vector);
	right_trapezoid.emplace_back(corners[0]);
	right_trapezoid.emplace_back(corners[1]);
	right_trapezoid.emplace_back(midpoint + vertical_vector + aim_deps::LIGHT_PARAM2 * direction_vector);
	right_trapezoid.emplace_back(midpoint + vertical_vector - aim_deps::LIGHT_PARAM2 * direction_vector);
	trapezoids.emplace_back(left_trapezoid);						//灯条左右两边将会拓展出两个梯形
	trapezoids.emplace_back(right_trapezoid);
}

void LightMatch::drawLights(cv::Mat &src, char belong[]) const{
	char str[5];
	for(size_t i = 0; i < possibles.size() ; ++i){
		if(enemy_blue){
			if(possibles[i].valid){				//可能是反光的灯条不绘制
				cv::line(src, possibles[i].box.vex[0], possibles[i].box.vex[1], aim_deps::RED, 1);
			}
			else{
				cv::line(src, possibles[i].box.vex[0], possibles[i].box.vex[1], aim_deps::PURPLE, 1);
			}
		}
		else{
			if(possibles[i].valid){				//可能是反光的灯条不绘制
				cv::line(src, possibles[i].box.vex[0], possibles[i].box.vex[1], aim_deps::CYAN, 1);
			}
			else{
				cv::line(src, possibles[i].box.vex[0], possibles[i].box.vex[1], aim_deps::PURPLE, 1);
			}
		}
		//snprintf(str, 4, "%lu", j);
		//cv::putText(src, str, pts[j]+cv::Point2f(1,1), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 100, 255));
		int isLeft = possibles[i].isLeft < 0 ? 2 : 1 - possibles[i].isLeft;
		snprintf(str, 5, "%d", belong[i]);
		cv::putText(src, str, possibles[i].box.vex[0] + cv::Point2f(1, 1),
					cv::FONT_HERSHEY_PLAIN, 1.5, aim_deps::ORANGE);
		cv::circle(src, possibles[i].box.vex[0], 0, aim_deps::PINK, -1);
		cv::circle(src, possibles[i].box.vex[1], 0, aim_deps::PINK, -1);
		match_debug(rmlog::F_GREEN, "Light ", possibles[i].index, " with lenth: ", possibles[i].box.length,
			", angle: ", possibles[i].box.angle);
	}
}

void LightMatch::threshold(cv::Mat &dst, int diff_thresh) const{
	const cv::Mat &tmp = enemy_blue ? (proced[0] - proced[2]) : (proced[2] - proced[0]);
	cv::Mat _filter(1080, 1440, CV_8UC1);
	/// 蓝色时，红色通道可能偏高（蓝色灯条亮，需要找差异）
	/// 红色灯条只需过滤所有图上蓝色值超过filter_thresh即可(默认95)
	cv::threshold(tmp, _filter, filter_thresh, 255, cv::THRESH_BINARY_INV);		
	if(enemy_blue){
		dst = proced[0] - _filter;
	}
	else{
		const cv::Mat &gc = proced[2] - proced[1];		// 红色通道减去 绿色
		cv::Mat gc_tmp(1080, 1440, CV_8UC1);

		// 红色通道 - 绿色通道差值过小说明颜色不是标准红、橙色
		cv::threshold(gc, gc_tmp, chan_diff, 255, cv::THRESH_BINARY_INV);
		dst = proced[2] - _filter - gc_tmp;// - bc_tmp;
	}
	cv::threshold(dst, dst, thresh_low, 255, cv::THRESH_BINARY);
}

bool LightMatch::isInTrapezoid(cv::Point2f corners[2], const std::vector<cv::Point2f> &trapezoid){
	for (int i = 0; i<2; ++i) {
		if (cv::pointPolygonTest(trapezoid, corners[i], false) < 0)
			return false;	
	}
	/// 2个点都找到，才能返回true
	return true;
}

bool LightMatch::doubleThresh(const std::vector<cv::Point>& ct, cv::Rect& bbox, bool valid){
	cv::RotatedRect tmp_rec = cv::minAreaRect(ct);
	if (cv::max(tmp_rec.size.height, tmp_rec.size.width) < 4.0){
		return false;
	}
	extendRect(bbox, cv::Size(4, 4));
	cv::Mat tmp, _bin;
	tmp = enemy_blue ? proced[0](bbox) : proced[2](bbox);
	double top[2];
	double ctr[3];
	float offset_x = bbox.x, offset_y = bbox.y;
	lightDiffusion(tmp, top, ctr, 2.1);
	cv::Point2f tp(top[0] + offset_x, top[1] + offset_y), mp(ctr[0] + offset_x, ctr[1] + offset_y);
	float len = aim_deps::getPointDist(tp, mp) * 2;
	aim_deps::Light _l(tp, mp, len, valid);
	mtx.lock();
	_l.index = (int)possibles.size();
	possibles.emplace_back(_l);
	mtx.unlock();
	return true;
}

void LightMatch::readjustAngle(
	const std::vector<cv::Point>& contour,
	aim_deps::Light &l,
	cv::Point offset,
	double weaken_coeff
) const{
	// 对于特征点较多的灯条，我们认为minAreaRect计算准确 > 80
	// 对于特征点较少的灯条，我们认为不能使用优化（很可能点集中在中点处，造成优化出错）
	if(contour.size() > 80 || contour.size() < 6) {
		return;	
	}
	double angle = 0.0;
	cv::Point2f nv0;
	cv::Point2f lv0 = (l.box.vex[1] - l.box.center) /
			sqrt(aim_deps::getPointDist(l.box.center, l.box.vex[1]));		
	if(l.box.length < 10.0 && std::abs(l.box.angle) > 9.0){
		/// 灯条短而角度过大, 正常情况下，灯条小时角度不可能很大(角度很大拍不清楚)
		/// 设初始角度为 0 度
		angle = atan2(lv0.x, lv0.y);				
	}
	nv0.x = lv0.y;
	nv0.y = -lv0.x;									
	for(int i = 0; i < 48; ++i){
		double diff_sum = 0.0, diff2_sum = 0.0;						// error_sum = 0.0;

		cv::Point2f lv = aim_deps::Rotate(lv0, angle);				
		for(size_t j = 0; j < contour.size(); ++j){
			diff_sum += calcDiff(lv, l.box.center, contour[j] + offset);		//计算一阶导
			diff2_sum += calcDiff2(lv, l.box.center, contour[j] + offset);	//计算二阶导(二阶的效果显著好于一阶)
		}		
		//printf("Light %d iter %d: diff_sum: %f, diff2_sum: %f\n", l.index, i, diff_sum, diff2_sum);
		if(diff2_sum == 0.0) break;
		angle -= diff_sum / diff2_sum;				// 牛顿迭代
		if(std::abs(angle) > 0.6) {					// 一次旋转不可能超过40度,超过则说明原来的匹配有问题
			angle += atan2(lv.x, lv.y);				// 初始值若引起计算错误，则把角度设为0度
		}
		if(std::abs(diff_sum) <= 0.5) 
		{
			break;
		}
	}
	angle = safeCast(angle);
	l.box.rotate(angle * weaken_coeff);
}

void LightMatch::extendRect(cv::Rect &rect, const cv::Size sz)	{		/// 长宽按照中心扩大
	if(rect.x - sz.width < 0){
		rect.x = 0;
	}
	else rect.x -= sz.width;
	if(rect.y - sz.height < 0){
		rect.y = 0;
	}
	else rect.y -= sz.height;
	if( rect.x + 2 * sz.width + rect.width >= 1440){
		rect.width = 1439 - rect.x;
	}
	else rect.width += 2 * sz.width;
	if( rect.y + 2 * sz.height + rect.height >= 1080){
		rect.height = 1079 - rect.y;
	}
	else rect.height += 2 * sz.height;
}

bool LightMatch::isGoodBoundingBox(const cv::Rect &rect){
	/// 根据rect.area 来判定合适的height - width 最小允许值
	if (rect.area() > 120){
		return (rect.height - rect.width >= 1);
	}
	else if (rect.area() > 10){
		/// 一个拟合函数
		float w = (float)rect.width, h = (float)rect.height, a = (float)rect.area();
		return h - w >= - 1.83525e-04 * a * a +  6.04567e-02 * a - 3.62164;
	}
	else{
		return (rect.height - rect.width >= - 3);
	}
}

bool LightMatch::getBigDirection(const cv::Mat &src, std::vector<cv::Point> &ct) const{
	cv::Mat tmp;
	cv::threshold(src, tmp, 127, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(tmp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	const std::vector<cv::Point> &ctref = *std::max_element(contours.begin(), contours.end(),
		[&](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
            return c1.size() < c2.size();
        }
	);
	ct.clear();
	for(size_t i = 0; i < ctref.size(); i += 2){		// 取一半以降低计算量
		ct.emplace_back(ctref[i]);
	}
	return (ct.size() > 12);
}

void LightMatch::readAndConvert(cv::Mat& dst, std::vector<double>& pts) const{
    cv::threshold(dst, dst, 50, 255, cv::THRESH_TOZERO);
    cv::Mat dst2(cv::Size(dst.cols, dst.rows), CV_64FC1);
    dst.convertTo(dst2, CV_64FC1);
    cv::normalize(dst2, dst2, 1.0, 0.0, cv::NORM_INF);
    pts.resize(dst2.cols * dst2.rows);
    dst2.forEach<double>(
        [&](const double& pix, const int* pos){
            pts[pos[1] + pos[0] * dst2.cols] = pix;
        }
    );
}

void LightMatch::betterInitialize(const cv::Mat& src, double* _top, double* _ctr) const {
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat dst;
    cv::threshold(src, dst, 1, 255, cv::THRESH_BINARY);
    cv::findContours(dst, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    const std::vector<cv::Point>& pts = *std::max_element(contours.begin(), contours.end(), 
        [&](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
            return c1.size() < c2.size();
        }
    );
    cv::RotatedRect rect = cv::minAreaRect(pts);
    cv::Point2f tp, mp;
    aim_deps::getTopCenter(rect, tp, mp);
    _top[0] = tp.x;
    _top[1] = tp.y;
    _ctr[0] = mp.x;
    _ctr[1] = mp.y;
}

void LightMatch::lightDiffusion(cv::Mat& src, double* top, double* ctr, double radius) {
    std::vector<double> values;
    readAndConvert(src, values);
    betterInitialize(src, top, ctr);
    ctr[2] = 0.0;
    ceres::Problem prob;
    ceres::CostFunction* cost = ErrorTerm::Create(values, src.cols, src.rows, radius);
    prob.AddResidualBlock(cost, nullptr, top, ctr);
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &prob, &summary);
	double dx = top[0] - ctr[0], dy = top[1] - ctr[1], norm = std::sqrt(dx * dx + dy * dy), k = (radius + ctr[2]) / norm;
	top[0] += k * dx;
	top[1] += k * dy;
}

bool LightMatch::isAngleValid(const aim_deps::LightBox &lb){
	if(lb.length < 4) return false;
	if(lb.length < 8)
		return (std::abs(lb.angle) <= 30.0);
	else
		return (std::abs(lb.angle) <= 40.0);
}

double LightMatch::calcDiff(
	const cv::Point2f &_vec,
	const cv::Point2f &ctr,
	const cv::Point2f &p
){
	cv::Point2f d = p - ctr;
	return pow(d.x, 2) * _vec.x * _vec.y - pow(d.y, 2) * _vec.x * _vec.y +
			d.x * d.y * pow(_vec.y, 2) - d.x * d.y * pow(_vec.x, 2);	// 导数公式
}

double LightMatch::calcDiff2(
	const cv::Point2f &_vec,
	const cv::Point2f &ctr,
	const cv::Point2f &p
){
	cv::Point2f d = p - ctr;
	return pow( d.x * _vec.x + d.y * _vec.y, 2) - pow( d.x * _vec.y - d.y * _vec.x, 2); 
}

double LightMatch::safeCast(double angle){
	while(angle < -2 * CV_PI){
		angle += 2 * CV_PI;
	}
	while(angle > 2 * CV_PI){
		angle -= 2 * CV_PI;
	}
	return angle;
}