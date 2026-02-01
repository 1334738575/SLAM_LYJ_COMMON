#ifndef COMMON_LYJ_COLMAP_IO_H
#define COMMON_LYJ_COLMAP_IO_H

#include "base/Base.h"


namespace COMMON_LYJ
{
	template <typename T>
	T ReadBinaryLittleEndian(std::istream* stream) {
		T data_little_endian;
		stream->read(reinterpret_cast<char*>(&data_little_endian), sizeof(T));
		//return LittleEndianToNative(data_little_endian);
		return data_little_endian;
	}
	template <typename T>
	void WriteBinaryLittleEndian(std::ostream* stream, const T& data) {
		//const T data_little_endian = NativeToLittleEndian(data);
		const T data_little_endian = data;
		stream->write(reinterpret_cast<const char*>(&data_little_endian), sizeof(T));
	}
	class SLAM_LYJ_API ColmapImage
	{
	public:
		ColmapImage();
		~ColmapImage();

		void write_binary(std::ofstream& stream) const;
		void read_binary(std::ifstream& stream);

		uint32_t image_id = UINT_MAX;
		Eigen::Quaterniond qcw;
		Eigen::Vector3d tcw;
		uint32_t camId = UINT_MAX;
		std::string imgName = "";
		uint64_t num_points2D = 0;
		std::vector<Eigen::Vector2d> points2D;
		std::vector<uint64_t> point3D_ids;
	};
	struct SLAM_LYJ_API ColmapSensor
	{
		void write_binary(std::ofstream& stream) const;
		void read_binary(std::ifstream& stream);
		int sensor_type = -1;
		uint32_t sensor_id = UINT_MAX;
	};
	struct SLAM_LYJ_API ColmapFrameDataId
	{
		void write_binary(std::ofstream& stream) const;
		void read_binary(std::ifstream& stream);
		ColmapSensor sensor;
		uint64_t id;
	};
	class SLAM_LYJ_API ColmapFrame
	{
	public:
		ColmapFrame();
		~ColmapFrame();

		void write_binary(std::ofstream& stream) const;
		void read_binary(std::ifstream& stream);

		uint32_t frame_id = UINT_MAX;
		uint32_t rig_id = UINT_MAX;
		Eigen::Quaterniond qcw;
		Eigen::Vector3d tcw;
		uint32_t num_data_ids = 0;
		std::vector<ColmapFrameDataId> data_ids;

	private:

	};
	class SLAM_LYJ_API ColmapCamera
	{
	public:
		ColmapCamera();
		~ColmapCamera();

		void write_binary(std::ofstream& stream) const;
		void read_binary(std::ifstream& stream);

		int getParamsNum() const;

		uint32_t camera_id = UINT_MAX;
		int model_id = -1;
		uint64_t width = 0;
		uint64_t height = 0;
		std::vector<double> params;
	};
	class SLAM_LYJ_API ColmapPoint
	{
	public:
		ColmapPoint();
		~ColmapPoint();

		void write_binary(std::ofstream& stream) const;
		void read_binary(std::ifstream& stream);

		uint64_t point3D_id = UINT64_MAX;
		Eigen::Vector3d point3D;
		Eigen::Matrix<unsigned char, 3, 1> color;
		double error = 0;
		uint64_t track_length = 0;
		std::vector<Eigen::Matrix<uint32_t, 2, 1>> track;
	private:

	};
	class SLAM_LYJ_API ColmapRig
	{
	public:
		ColmapRig();
		~ColmapRig();

		void write_binary(std::ofstream& stream) const;
		void read_binary(std::ifstream& stream);

		uint32_t rig_id = UINT_MAX;
		uint32_t num_sensors = 0;
		ColmapSensor ref_sensor_id;
		std::vector<ColmapSensor> sensor_ids;//num_sensors - 1
		std::vector<Eigen::Quaterniond> qi0s;//num_sensors - 1
		std::vector<Eigen::Vector3d> ti0s;//num_sensors - 1
		std::vector<bool> hasPose;//num_sensors - 1
	private:

	};


	class SLAM_LYJ_API ColmapData
	{
	public:
		ColmapData();
		~ColmapData();

		void write_binary(std::ofstream& stream) const;
		void read_binary(std::ifstream& stream);

		bool writeFromColmap(const std::string _path) const;
		bool readFromColmap(const std::string _path);
	public:
		uint64_t num_reg_images = 0;
		std::vector<ColmapImage> images_;

		uint64_t num_frames = 0;
		std::vector<ColmapFrame> frames_;

		uint64_t num_cameras = 0;
		std::vector<ColmapCamera> cameras_;

		uint64_t num_rigs = 0;
		std::vector<ColmapRig> rigs_;

		uint64_t num_point3Ds = 0;
		std::vector<ColmapPoint> point3Ds_;
	};


	class SLAM_LYJ_API ColmapDataBase
	{
	public:
		// 1. 相机信息结构体（对应cameras表）
		struct Camera {
			int camera_id = 0;                // 相机ID（主键）
			std::string model;                // 相机模型（如PINHOLE、SIMPLE_RADIAL）
			int width = 0, height = 0;        // 相机对应的图像尺寸
			std::vector<float> params;        // 相机内参（float数组，BLOB解析）
		};

		// 2. 图像信息结构体（对应images表，关联相机）
		struct Image {
			int image_id = 0;                 // 图像ID（主键）
			std::string name;                 // 图像文件名
			int camera_id = 0;                // 关联相机ID（外键）
			const Camera* camera = nullptr;   // 关联的相机指针（快速访问，无需查表）
		};

		// 3. 特征点信息结构体（对应keypoints表，关联图像）
		struct KeyPoint {
			int image_id = 0;                 // 关联图像ID（外键）
			std::vector<std::pair<float, float>> coords; // 所有特征点(x,y)坐标（亚像素）
			int count = 0;                    // 特征点数量（对应rows字段）
		};

		// 4. 特征匹配信息结构体（对应matches表，关联图像对）
		struct Match {
			uint64_t pair_id = 0;             // COLMAP压缩的图像对ID
			int image_id1 = 0, image_id2 = 0; // 解码后的两个图像ID（image_id1 < image_id2）
			std::vector<std::pair<int, int>> matches; // 匹配对：<img1特征点索引, img2特征点索引>
			int count = 0;                    // 匹配对数量（对应rows字段）
		};
		// 5. 两视图几何验证信息结构体（对应two_view_geometries表，关联matches表）
		struct TwoViewGeometry {
			uint64_t pair_id = 0;             // 图像对压缩ID（与matches/two_view_geometries一致）
			int image_id1 = 0, image_id2 = 0; // 解码后的图像对（image_id1 < image_id2）
			int inlier_count = 0;             // 几何验证后有效内点数量（rows字段）
			int cols = 0;                     // 固定为3，单内点存储维度
			// 有效内点数据：<img1特征点索引, img2特征点索引, 重投影误差>
			std::vector<std::tuple<int, int, float>> inliers;
			std::vector<double> F;             // 基础矩阵 3x3 (9个float，行优先)
			std::vector<double> E;             // 本质矩阵 3x3 (9个float，行优先)
			std::vector<double> H;             // 单应矩阵 3x3 (9个float，行优先)
			std::vector<double> qvec;          // 旋转四元数 [qw, qx, qy, qz] (4个float)
			std::vector<double> tvec;          // 平移向量 [tx, ty, tz] (3个float)
		};

		// 全局存储：全量解析结果（按主键索引，支持O(1)查询）
		std::map<int, Camera> g_cameras;    // key: camera_id，存储所有相机
		std::map<int, Image> g_images;      // key: image_id，存储所有图像
		std::map<int, KeyPoint> g_keypoints;// key: image_id，存储所有特征点
		std::vector<Match> g_matches;       // 存储所有特征匹配（无唯一主键，用vector）
		std::map<uint64_t, TwoViewGeometry> g_two_view_geoms;

		ColmapDataBase();
		~ColmapDataBase();

		bool openDB(const std::string _path);
		bool readDataBase();
		bool closeDB();

		bool query_all_tables();
		bool query_table_schema(const std::string& table_name);
		bool query_images(int limit = 10);
		bool query_cameras();
		bool query_keypoint_count(int image_id);
		bool get_keypoint_coords(int target_image_id, std::vector<Eigen::Vector2f>& _ps);

		/**
		 * @brief 全量读取 cameras 表，存储到 g_cameras
		 * @param db 已打开的 SQLite3 数据库句柄
		 * @return true=成功，false=失败
		 */
		bool read_all_cameras();
		/**
		 * @brief 全量读取 images 表，关联 g_cameras，存储到 g_images
		 * @param db 已打开的 SQLite3 数据库句柄
		 * @return true=成功，false=失败
		 */
		bool read_all_images();
		/**
		 * @brief 全量读取 keypoints 表，解析特征点坐标，存储到 g_keypoints
		 * @param db 已打开的 SQLite3 数据库句柄
		 * @return true=成功，false=失败
		 */
		bool read_all_keypoints();
		/**
		 * @brief 全量读取 matches 表，解码pair_id，解析匹配对，存储到 g_matches
		 * @param db 已打开的 SQLite3 数据库句柄
		 * @return true=成功，false=失败
		 */
		bool read_all_matches();
		/**
		* @brief 全量读取 two_view_geometries 表，关联g_matches，存储到g_two_view_geoms
		* @param db 已打开的 SQLite3 数据库句柄
		* @return true=成功，false=失败
		*/
		bool read_all_two_view_geometries();
	private:
		/**
		 * @brief BLOB 二进制数据解析为 float 数组
		 * @param blob_data SQLite3 BLOB 数据指针
		 * @param blob_size BLOB 总字节数
		 * @return 解析后的 float 数组，空则表示解析失败
		 */
		std::vector<float> blob_to_float(const void* blob_data, int blob_size) {
			std::vector<float> res;
			if (blob_data == nullptr || blob_size <= 0 || blob_size % 4 != 0) {
				return res; // 非4的倍数，不是合法float数组
			}
			int float_count = blob_size / sizeof(float);
			res.resize(float_count);
			const float* src = static_cast<const float*>(blob_data);
			std::memcpy(res.data(), src, blob_size); // 内存拷贝，安全解析
			return res;
		}
		/**
		 * @brief 解码 COLMAP pair_id 为图像对 (image_id1, image_id2)
		 * @param pair_id COLMAP 压缩的图像对ID
		 * @return 解码后的图像对，保证 image_id1 < image_id2
		 */
		std::pair<int, int> decode_pair_id(uint64_t pair_id) {
			const uint64_t HALF = 1ULL << 31; // 2^31，COLMAP 固定压缩基数
			int img1 = static_cast<int>(pair_id / HALF);
			int img2 = static_cast<int>(pair_id % HALF + img1);
			return { img1, img2 };
		}
		/**
		 * @brief BLOB 二进制数据解析为**固定长度**的 float 数组
		 * @param blob_data SQLite3 BLOB 数据指针
		 * @param blob_size BLOB 总字节数
		 * @param expected_float_num 预期的 float 元素数量（如矩阵9个、四元数4个）
		 * @return 解析后的固定长度 float 数组，长度不满足则返回空
		 */
		std::vector<double> blob_to_double_fixed(const void* blob_data, int blob_size, int expected_float_num) {
			std::vector<double> res;
			// 验证：BLOB非空 + 字节数为4的倍数 + 元素数量匹配预期
			if (blob_data == nullptr || blob_size <= 0 || blob_size % 8 != 0) {
				return res;
			}
			int actual_float_num = blob_size / sizeof(double);
			if (actual_float_num != expected_float_num) {
				return res;
			}
			// 内存拷贝解析
			res.resize(expected_float_num);
			const double* src = static_cast<const double*>(blob_data);
			std::memcpy(res.data(), src, blob_size);
			return res;
		}
		// 扩展版：uint64_t转64位二进制字符串，每8位添加一个空格（提升可读性）
		std::string uint64_to_binary_with_sep(uint64_t num, char sep = ' ') {
			std::string binary_str;
			binary_str.reserve(71);  // 64位 + 7个分隔符 = 71位
			uint64_t mask = 1ULL << 63;

			for (int i = 0; i < 64; ++i) {
				binary_str += (num & mask) ? '1' : '0';
				mask >>= 1;
				// 每8位添加一个分隔符，最后一位后不添加
				if ((i + 1) % 8 == 0 && i != 63) {
					binary_str += sep;
				}
			}
			return binary_str;
		}

private:
		std::vector<float> parse_camera_params(const uint8_t* params_blob, int blob_size);
		void* sql_ = nullptr;
	};


}





#endif // !COMMON_LYJ_COLMAP_IO_H
