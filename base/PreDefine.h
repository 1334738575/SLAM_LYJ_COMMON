#ifndef SLAM_LYJ_PREDEFINE_H
#define SLAM_LYJ_PREDEFINE_H

// lyj
#include "Base.h"
#include "common/Point.h"
#include "common/Line.h"
#include "common/Plane.h"
#include "common/KdTree.h"
#include "common/RANSAC.h"
#include "common/ThreadPool.h"
#include "common/Tensor.h"
#include "common/CommonAlgorithm.h"
#include "common/Grid.h"
#include "Pose.h"
#include "CameraModule.h"
#include "config/config.h"

NSP_SLAM_LYJ_BEGIN

#define SYS_VERSION 1
#define SYS_DEBUG

class SLAM_LYJ_API GlobalInnerOption
{
public:
	std::string sysName = "";
	std::string sysHomePath = SLAM_LYJ_HOME_PATH;

public:
	GlobalInnerOption() {}
	~GlobalInnerOption() {}

	static GlobalInnerOption *get()
	{
		static GlobalInnerOption opt;
		return &opt;
	}

private:
};

NSP_SLAM_LYJ_END

#define LYJOPT SLAM_LYJ::GlobalInnerOption::get()

#endif