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
	enemy_blue = true;
	thresh_low = 90;
	filter_thresh = 20;
	chan_diff = 5;									//初始设置敌人颜色
    opts.minimizer_type = ceres::LINE_SEARCH;
    opts.line_search_direction_type = ceres::LBFGS;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.line_search_type = ceres::WOLFE;
	opts.logging_type = ceres::SILENT;
    opts.minimizer_progress_to_stdout = false;
    opts.max_num_iterations = 50;
    opts.function_tolerance = 2e-4;
}

LightMatch::~LightMatch(){
	#ifdef LIGHT_CNT_TIME
		std::cout<< "Time for threshold:" << (double)(time_sum / cnt) << std::endl;
	#endif
}

void LightMatch::setEnemyColor(bool _enemy_blue, int _thresh_low, int ch_diff, int filter){
	enemy_blue = _enemy_blue;
	thresh_low = _thresh_low;
	filter_thresh = filter;
	chan_diff = ch_diff;
}

void LightMatch::findPossible(){			//找出所有可能灯条，使用梯形匹配找出相匹配的灯条对
	
	reset();
	cv::Mat binary(1080, 1440, CV_8UC1);
	threshold(binary);
	cv::dilate(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
	cv::imshow("bin", binary);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);	//寻找图上轮廓
	#ifdef LIGHT_CNT_TIME
		double start_t = std::chrono::system_clock::now().time_since_epoch().count();
	#endif	//LIGHT_CNT_TIME
	std::vector<aim_deps::Light> lts;
	contourProcess(contours, lts);
	cv::imshow("bin2", proced[0]);
	#ifdef LIGHT_CNT_TIME
		double end_t = std::chrono::system_clock::now().time_since_epoch().count();
		++cnt;
		time_sum += (end_t - start_t) / 1e6; 
	#endif	//LIGHT_CNT_TIME

	std::sort(possibles.begin(), possibles.end(), 
		[&](const aim_deps::Light& la, const aim_deps::Light& lb) {
			return la.box.center.x < lb.box.center.x;
		}
	);
	std::sort(lts.begin(), lts.end(), 
		[&](const aim_deps::Light& la, const aim_deps::Light& lb) {
			return la.box.center.x < lb.box.center.x;
		}
	);
	for (size_t i = 0; i < lts.size(); i++) {									// 预先进行sort
		lts[i].index = i;
	}

	#ifdef LIGHT_MATCH_DEBUG
	 	cv::imshow("binary", binary);
	#endif	//LIGHT_MATCH_DEBUG

	for (aim_deps::Light &light: lts) {
		getTrapezoids(light.box.vex);								
	}
	std::vector<PtrPair> prs;
	getRealLight(lts, prs);
	int index = 0;
	for (aim_deps::Light& lt: lts) {
		if (lt.valid == true) {
			lt.index = index++;
			possibles.emplace_back(lt);
		}
	}
	for (const PtrPair& pr: prs) {
		matches.emplace_back(pr.first->index, pr.second->index);
	}
}

void LightMatch::contourProcess(const std::vector<std::vector<cv::Point> >& ct, std::vector<aim_deps::Light>& lts){
	#pragma omp parallel for num_threads(4)
	for (size_t i = 0; i < ct.size(); i++) {	
		float area = cv::contourArea(ct[i]);
		cv::Rect bbox = cv::boundingRect(ct[i]);
		cv::Mat roi = enemy_blue ? proced[0](bbox) : proced[2](bbox);
		// 轮廓在bbox中占的面积合适 / bbox 本身的面积合适 / bbox 的高不能过小 / bbox的高 - bbox长 > 某一值
		if(bbox.area() > 20 * area || area < 4 || bbox.height < 4) continue;
		/// 判定bbox 是否存在宽比高大很多的情况
		if (!isGoodBoundingBox(bbox)) {
			// LOG_SHELL_STREAM("BBOX is not appropriate.\n");	
			cv::rectangle(proced[0], bbox, cv::Scalar(255, 255, 255));
			continue;
		}
		if(bbox.area() < 400){
			cv::Mat cnt_mat;
			float mean = 0.0;
			cv::threshold(roi, cnt_mat, 90, 255, cv::THRESH_BINARY);
			mean = cv::mean(roi, cnt_mat)[0];
			cv::RotatedRect tmp_rec = cv::minAreaRect(ct[i]);
			if (mean < 130 || cv::max(tmp_rec.size.height, tmp_rec.size.width) < 4.0)
				continue;							// 均值小于 130 或者 最小包袱矩形尺寸太小
			doubleThresh(bbox, lts);
		}
		else{		// 选框够大，说明灯条无需二次阈值，亚像素检测以及角度修正
			cv::RotatedRect l = cv::minAreaRect(ct[i]);
			if (l.size.height / l.size.width < 1.6 && l.size.height / l.size.width > 0.6){
				continue;
			}
			aim_deps::Light _l(l, l.center, cv::max(l.size.height, l.size.width));
			if(bbox.area() < 2500){		// 在灯条大小不算太大时，长度准确，但角度不准
				std::vector<cv::Point> cont;
				if(!isAngleValid(_l.box)) continue;
				getBigDirection(roi, cont);
				readjustAngle(cont, _l, cv::Point(bbox.x, bbox.y), 1.0);	
			}					
			if( std::abs(_l.box.angle) < 40.0){
				mtx.lock();	
				lts.emplace_back(_l);
				mtx.unlock();
			}
		}
	}
}

void LightMatch::getRealLight(std::vector<aim_deps::Light>& lts, std::vector<PtrPair>& prs){
	int size = lts.size();
	bool flag[size][size];					//两灯条是否满足要求,是个对称矩阵，当[i][j],[j][i]为真时，两灯条匹配
	for (int i = 0; i < size; ++i) {		//初始化		
        for(int j = 0; j < size; ++j)
		    flag[i][j] = false;
	}
	for (int i = 0; i < size; ++i) {		//梯形包含匹配
		for (int j = 0; j<size*2; ++j) {
			if (isInTrapezoid(lts[i].box.vex, trapezoids[j])) {
				flag[i][j/2] = true;
			}
		}
	}
	for (int i = 0; i<size; ++i) {							//将可能的匹配结果放入容器matches中
		for (int j = i+1; j < size; ++j) {
			if (flag[i][j] == true && flag[j][i] == true) {
				flag[i][j] = false;
				lts[i].valid = true;					// 预匹配成功的灯条需要设置valid标签
				lts[j].valid = true;
				prs.emplace_back(&lts[i], &lts[j]);
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
				cv::line(src, possibles[i].box.vex[0], possibles[i].box.vex[1], aim_deps::CYAN, 1);
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
		snprintf(str, 5, "%lu", i);
		cv::putText(src, str, possibles[i].box.vex[0] + cv::Point2f(1, 1),
					cv::FONT_HERSHEY_PLAIN, 1.5, aim_deps::ORANGE);
		cv::circle(src, possibles[i].box.vex[0], 0, aim_deps::PINK, -1);
		cv::circle(src, possibles[i].box.vex[1], 0, aim_deps::PINK, -1);
	}
}

void LightMatch::threshold(cv::Mat &dst, int diff_thresh) const{
	const cv::Mat &tmp = enemy_blue ? (proced[0] - proced[2]) : (proced[2] - proced[0]);
	cv::Mat _filter(1080, 1440, CV_8UC1);
	cv::threshold(tmp, _filter, filter_thresh, 255, cv::THRESH_BINARY_INV);		
	if(enemy_blue){
		dst = proced[0] - _filter;
	}
	else{
		dst = proced[2] - _filter;
	}
	cv::threshold(dst, dst, 80, 255, cv::THRESH_BINARY);
}

bool LightMatch::isInTrapezoid(cv::Point2f corners[2], const std::vector<cv::Point2f> &trapezoid){
	for (int i = 0; i<2; ++i) {
		if (cv::pointPolygonTest(trapezoid, corners[i], false) < 0)
			return false;	
	}
	/// 2个点都找到，才能返回true
	return true;
}

bool LightMatch::doubleThresh(cv::Rect& bbox, std::vector<aim_deps::Light>& lts){
	extendRect(bbox, cv::Size(3, 3));
	cv::Mat tmp = enemy_blue ? proced[0](bbox) : proced[2](bbox);
	double top[2];
	double ctr[3];
	float offset_x = bbox.x, offset_y = bbox.y;
	if (lightDiffusion(tmp, top, ctr, 2.1) == false) {
		return false;
	}
	/// this needs to be deleted.
	proced[0](bbox) = tmp;
	cv::Point2f tp(top[0] + offset_x, top[1] + offset_y), mp(ctr[0] + offset_x, ctr[1] + offset_y);
	aim_deps::Light _l(tp, mp);
	if (!isAngleValid(_l.box)) return false;
	mtx.lock();
	lts.emplace_back(_l);
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
    cv::threshold(dst, dst, thresh_low, 255, cv::THRESH_TOZERO);
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

bool LightMatch::betterInitialize(const cv::Mat& src, double* _top, double* _ctr) const {
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat dst;
    cv::threshold(src, dst, 1, 255, cv::THRESH_BINARY);
    cv::findContours(dst, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	if (contours.size() > 0) {			// no usable result
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
		return true;
	}
    return false;
}

bool LightMatch::lightDiffusion(cv::Mat& src, double* top, double* ctr, double radius) {
    std::vector<double> values;
    readAndConvert(src, values);
    if (betterInitialize(src, top, ctr) == false) {
		return false;
	}
    ctr[2] = 0.0;
    ceres::Problem prob;
    ceres::CostFunction* cost = ErrorTerm::Create(values, src.cols, src.rows, radius);
    prob.AddResidualBlock(cost, nullptr, top, ctr);
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &prob, &summary);
	if (summary.IsSolutionUsable() == false) {
		return false;
	}
	double dx = top[0] - ctr[0], dy = top[1] - ctr[1], norm = std::sqrt(dx * dx + dy * dy), k = (radius + ctr[2]) / norm;
	top[0] += k * dx;
	top[1] += k * dy;
	return true;
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