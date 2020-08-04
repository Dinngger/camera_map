/*
灯条检测模块-找到图上所有合适的灯条,并且将其匹配为装甲板
last date of modification:2020.8.1
*/

#include "../include/LightMatch.hpp"

LightMatch::LightMatch(){
	#ifdef LIGHT_CNT_TIME
		time_sum 		= 0.0;
		cnt 			= 0;
	#endif // LIGHT_CNT_TIME
	setEnemyColor(true);												//初始设置敌人颜色
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

void LightMatch::reset(){
	matches.clear();
	possibles.clear();
	trapezoids.clear();
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
	#ifdef MULTI_THREAD
		//三线程加速(四线程及以上的速度没有太大区别)
		if(contours.size() >= 2){
			std::thread thresh_t1(&LightMatch::contourProcess, this, contours, 0, 3);
			std::thread thresh_t2(&LightMatch::contourProcess, this, contours, 1, 3);
			contourProcess(contours, 2, 3);
			thresh_t1.join();
			thresh_t2.join();
		}
		else{
			contourProcess(contours, 0, 1);
		}
	#else
		///单线程调试
		contourProcess(contours, 0, 1);
	#endif	//MULTI_THREAD

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

void LightMatch::contourProcess(const std::vector<std::vector<cv::Point> >& ct, int start, int step){
	double min_pos = 0.0, max_pos = 0.0;
	for (size_t i = start; i < ct.size(); i += step) {
		float area = cv::contourArea(ct[i]);
		cv::Rect bbox = cv::boundingRect(ct[i]);

		// 轮廓在bbox中占的面积合适 / bbox 本身的面积合适 / bbox 的高不能过小 / bbox的高 - bbox长 > 某一值
		if(bbox.area() <= 20 * area && area >= 4.0 && bbox.height >= 4){
			/// 判定bbox 是否存在宽比高大很多的情况
			if (!isGoodBoundingBox(bbox)){
				continue;
			}
			if(bbox.area() < 800){
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

void LightMatch::drawRotatedRect(cv::Mat &src, const cv::RotatedRect &rect){
	cv::Point2f pts[4];
	rect.points(pts);
	for(int i = 0; i < 4; ++i){
		cv::line(src, pts[i], pts[(i + 1) % 4], cv::Scalar(150));
	}
	cv::circle(src, rect.center, 0, cv::Scalar(50), -1);
}


bool LightMatch::doubleThresh(const std::vector<cv::Point>& ct, cv::Rect& bbox, bool valid){
	cv::RotatedRect tmp_rec = cv::minAreaRect(ct);
	if (cv::max(tmp_rec.size.height, tmp_rec.size.width) < 4.0){
		return false;
	}
	cv::Size subpix_sz = decideSize(tmp_rec);
	bool do_subpixel = extendRect(bbox, subpix_sz + cv::Size(2, 2));
	float _a = tmp_rec.size.area();

	cv::Mat tmp, _bin;
	if(enemy_blue){
		tmp = proced[0](bbox);					/// 敌方为蓝色，取蓝色通道
		int thresh_used = getDoubleThresh(_a);
		cv::threshold(tmp, _bin, thresh_used, 255, cv::THRESH_BINARY);
	}
	else{
		tmp = proced[2](bbox);
		cv::threshold(tmp, _bin, 127, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);
	}

	//drawRotatedRect(src, tmp_rec);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(_bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	//cv::rectangle(src, bbox, cv::Scalar(100));
	if(contours.size() >= 1){
		std::sort(contours.begin(), contours.end(), sizeCmp);
		printf("Light %lu started to find true contours.\n", possibles.size());
		const std::vector<cv::Point> &c = contours[getTrueContour(contours, tmp_rec.center, bbox.tl())];
		cv::RotatedRect rot_box = cv::minAreaRect(c);
		float maxi = cv::max(rot_box.size.width, rot_box.size.height);
		std::vector<cv::Point2f> contour;
		convertVector(c, contour);						//将c(cv::Point)转化为cv::Point2f类型
		if(contour.size() < 72 && do_subpixel){
			cv::TermCriteria criteria = cv::TermCriteria(
					cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 25, 0.01);
			if(maxi < 6){			//过小则需要设置不搜索范围
				std::vector<cv::Point2f> cont(contour);
				cv::cornerSubPix(tmp, contour,
					subpix_sz, cv::Size(0, (int)((maxi - 2) / 2)), criteria);

				///补充未优化点
				contour.insert(contour.end(), cont.begin(), cont.end());
			}
			else {
				cv::cornerSubPix(tmp, contour, subpix_sz, cv::Size(-1, -1), criteria);
			}
		}
		else{
			do_subpixel = false;
		}
		cv::RotatedRect light = cv::minAreaRect(contour);
		if(!do_subpixel){														// 没有进行亚像素的灯条会偏短
			light.size.height *= 1.1;
		}
		light.center.x += bbox.x;												// 局部ROI加偏置
		light.center.y += bbox.y;

		float max_len = cv::max(tmp_rec.size.height, tmp_rec.size.width);		// 灯条外包络参考长度
		aim_deps::Light _l(light, tmp_rec.center, maxi, valid);

		/// 判断是否出现内外包络的差异过大的情况
		if(max_len < _l.box.length * 1.7){
			//printf("(Light %lu)Before readjust: %f\n", possibles.size(), _l.box.angle);
			if( !isAngleValid(_l.box) ) {		//预先筛除不符合角度要求的灯条,可以省略耗时的角度优化
				return false;					//灯条角度不符合要求
			}
			std::vector<cv::Point> corners;
			if(enemy_blue == false && _l.box.length <= 11.0){			// 输入判定是否为小灯条
				/// 红色小灯条适合灰度化角度优化 - 当敌人为红色且很小时
				readjustAngleGray(tmp, _l, (float)bbox.x, (float)bbox.y);
			}
			else{
				///
				if(getDirection(bbox, corners, tmp_rec.center, (_l.box.length <= 11.0))){
					readjustAngle(corners, _l, bbox.tl());
				}
				else{
					readjustAngle(ct, _l, cv::Point(0, 0), 0.8);
				}
			}
			if( !isAngleValid(_l.box) ) {		//处理后二次筛除
				return false;		//灯条角度不符合要求
			}
			mtx.lock();
			_l.index = (int)possibles.size();
			possibles.emplace_back(_l);
			mtx.unlock();
			return true;
		}
	}
	// 对原本就很小，没能进行亚像素优化的灯条进行加长
	if(tmp_rec.size.height > tmp_rec.size.width){
		tmp_rec.size.height *= 1.15;
	}
	else{
		tmp_rec.size.width *= 1.15;
	}

	aim_deps::Light _l(tmp_rec, tmp_rec.center,
					cv::max(tmp_rec.size.height, tmp_rec.size.width), valid);

	if(!isAngleValid(_l.box)){								//预先筛除不符合角度要求的灯条,可以省略耗时的角度优化
		return false;
	}

	if(_l.box.length > 8.0){			// 输入判定是否为小灯条
		readjustAngle(ct, _l, cv::Point(0, 0), 0.5);
	}

	if( !isAngleValid(_l.box) ) {							//处理后二次筛除
		return false;
	}
	mtx.lock();
	_l.index = (int)possibles.size();
	possibles.emplace_back(_l);
	mtx.unlock();
	return true;
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

void LightMatch::drawLights(cv::Mat &src) const{
	char str[2];
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
		snprintf(str, 2, "%lu", i);
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

bool LightMatch::getDirection(
	const cv::Rect &rec,
	std::vector<cv::Point> &ct,
	const cv::Point2f &center,
	bool is_small
){
	cv::Mat cr_map, cg_map, cb_map, tmp;
	int thresh = (int)cv::threshold(proced[1](rec), cg_map, 50, 255, cv::THRESH_OTSU);
	if(enemy_blue){
		cv::threshold(proced[0](rec), cb_map, 120, 255, cv::THRESH_BINARY_INV);
		// 敌方为蓝色时，红色通道的阈值为绿色OTSU阈值的0.4倍(可能需要修正)
		cv::threshold(proced[2](rec), cr_map, (int)(thresh * 0.4), 255, cv::THRESH_BINARY_INV);
	}
	else{
		cv::threshold(proced[2](rec), cr_map, 120, 255, cv::THRESH_BINARY_INV);
		// 敌方为红色时，蓝色通道的阈值为绿色OTSU阈值的0.4倍(可能需要修正)
		cv::threshold(proced[0](rec), cb_map, (int)(thresh * 0.4), 255, cv::THRESH_BINARY_INV);
	}

	tmp = cg_map - cr_map - cb_map;		// 最后的处理图像是 绿色通道 - 红色 - 蓝色
	cv::threshold(tmp, tmp, 127, 255, cv::THRESH_BINARY);

	std::vector<std::vector<cv::Point> > cts;
	cv::findContours(tmp, cts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	std::sort(cts.begin(), cts.end(), sizeCmp);		// 找最多角点的轮廓，就是所求角点集合
	std::vector<cv::Point> &res = cts.front();
	if(cts.size() == 0){
		return false;
	}
	else if(cts.size() >= 2){
		res = cts[getTrueContour(cts, center, rec.tl())];
	}
	ct.assign(cts.front().begin(), cts.front().end());
	if(is_small && cts.size() < 2){		// 灯条过小时 直接使用阈值化后为255的点而不查找轮廓（size > 2时说明连通度 > 1）
		tmp.forEach<uchar>(				// forEach是异步的,所以快
			[&](uchar &pix, const int * pos) -> void{
				if(pix > 0){
					mtx.lock();
					ct.emplace_back(cv::Point(pos[1], pos[0]));
					mtx.unlock();
				}
			}
		);
	}
	return (ct.size() >= 10);
}

bool LightMatch::getBigDirection(const cv::Mat &src, std::vector<cv::Point> &ct) const{
	cv::Mat tmp;
	cv::threshold(src, tmp, 127, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(tmp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	std::sort(contours.begin(), contours.end(), sizeCmp);
	ct.clear();
	const std::vector<cv::Point> &ctref = contours.front();
	for(size_t i = 0; i < ctref.size(); i += 2){		// 取一半以降低计算量
		ct.emplace_back(ctref[i]);
	}
	return (ct.size() > 12);
}

bool LightMatch::isInTrapezoid(cv::Point2f corners[2], const std::vector<cv::Point2f> &trapezoid){
	for (int i = 0; i<2; ++i) {
		if (cv::pointPolygonTest(trapezoid, corners[i], false) < 0)
			return false;
	}
	/// 2个点都找到，才能返回true
	return true;
}

int LightMatch::getDoubleThresh(float _a){
	if(_a >= 64.0f) return 180;
	else if(_a >= 30.0f) return (int)(0.8 * _a + 129.283);
	else if(_a >= 16.0f) return (int)(2.7345 * _a + 71.249);
	else return 115;
}

bool LightMatch::extendRect(cv::Rect &rect, const cv::Size sz)	{		/// 长宽按照中心扩大
	bool within = true;							//选框是否超出边界？超出则无法获得足够大的ROI进行亚像素检测
	if(rect.x - sz.width < 0){
		rect.x = 0;
		within = false;
	}
	else rect.x -= sz.width;
	if(rect.y - sz.height < 0){
		rect.y = 0;
		within = false;
	}
	else rect.y -= sz.height;
	if( rect.x + 2 * sz.width + rect.width >= 1440){
		rect.width = 1439 - rect.x;
		within = false;
	}
	else rect.width += 2 * sz.width;
	if( rect.y + 2 * sz.height + rect.height >= 1080){
		rect.height = 1079 - rect.y;
		within = false;
	}
	else rect.height += 2 * sz.height;
	return within;
}

/// ====================灯条优化的实现尝试====================== ///
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
	#ifdef DRAW_CONTOUR
		std::vector<cv::Point> to_draw(contour.size());
		for(size_t i = 0; i < contour.size(); ++i){
			to_draw[i] = contour[i] + offset;
		}
		mtx.lock();
		cts_todraw.emplace_back(to_draw);
		mtx.unlock();
	#endif
}

void LightMatch::readjustAngleGray(cv::Mat &src, aim_deps::Light &l, float ofx, float ofy){
	/// 小灯条角度优化（大灯条使用Mat的计算量过大）
	double angle = 0.0;
	cv::Point2f nv0;
	cv::Point2f lv0 = (l.box.vex[1] - l.box.center) /
			sqrt(aim_deps::getPointDist(l.box.center, l.box.vex[1]));
	if(std::abs(l.box.angle) > 9.0){
		angle = atan2(lv0.x, lv0.y);
	}
	nv0.x = lv0.y;
	nv0.y = -lv0.x;
	for(int i = 0; i < 48; ++i){
		double diff_sum = 0.0, diff2_sum = 0.0;						// error_sum = 0.0;
		cv::Point2f lv = aim_deps::Rotate(lv0, angle);
		/// cv内置异步 灰度加权的角度优化
		src.forEach<uchar>(
			[&](uchar &pix, const int * pos)->void{
				if(pix > 40){
					mtx.lock();
					diff_sum += (double)pix / 127.0 * calcDiff(lv, l.box.center,
						cv::Point2f(pos[1] + 0.5 + ofx, pos[0] + 0.5 + ofy));
					diff2_sum += (double)pix / 127.0 * calcDiff2(lv, l.box.center,
						cv::Point2f(pos[1] + 0.5 + ofx, pos[0] + 0.5 + ofy));
					mtx.unlock();
				}
			}
		);
		// printf("Light %d iter %d: diff_sum: %f, diff2_sum: %f\n", l.index, i, diff_sum, diff2_sum);
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
	l.box.rotate(angle);
}

int LightMatch::getTrueContour(const std::vector<std::vector<cv::Point> > &cts,
	const cv::Point2f &center, const cv::Point &offset
){
	if(cts.size() == 1){
		return 0;
	}
	else{	// 只搜索size最大的两个contour
		double res0 = cv::pointPolygonTest(cts[0], center - cv::Point2f(offset), true),
			res1 = cv::pointPolygonTest(cts[1], center - cv::Point2f(offset), true);
		printf("[%lf][%lf].\n", res0, res1);
		if(res0 >= res1){
			return 0;
		}
		else{
			return 1;
		}
	}
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

cv::Size LightMatch::decideSize(const cv::RotatedRect &rect){
	float maxi = 0.0, mini = 0.0, length = 0.0;
	cv::Point2f upper, bottom;
	aim_deps::getMidPoints(rect, upper, bottom);
	length = sqrt(aim_deps::getPointDist(upper, bottom));
	float angle = aim_deps::getLineAngleRad(upper, bottom);
	maxi = length * cos(angle);
	mini = length * sin(angle);
	int x = 2, y = 3;
	if(maxi >= 24) y = 9;								// 过大则固定
	else if(maxi > 7.5){								// 一个拟合函数
		y = std::round( 0.4 * maxi - 0.48);
	}
	if(mini <= 2) x = 2;
	else if(mini < 3) x = 3;
	else x = 4;
	if(angle > CV_PI / 4) std::swap(x, y);				// 角度大于45度，x与y轴交换
	return cv::Size(x, y);
}

void LightMatch::convertVector(const std::vector<cv::Point>& src, std::vector<cv::Point2f> &dst){
	dst.resize(src.size());
	for(size_t i = 0; i < src.size(); ++i){
		dst[i] = cv::Point2f(src[i]);
	}
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

#ifdef DRAW_CONTOUR
void LightMatch::drawContour(cv::Mat &src){
	for(size_t i = 0; i < cts_todraw.size(); ++i){
		for(size_t j = 0; j < cts_todraw[i].size(); ++j){
			cv::circle(src, cts_todraw[i][j], 0, cv::Scalar(200, 0, 200));
		}
	}
	cts_todraw.clear();
}
#endif	// DRAW_CONTOUR