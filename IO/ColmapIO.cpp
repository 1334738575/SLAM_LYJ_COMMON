#include "ColmapIO.h"

#ifdef USESQLITE3
#include <sqlite3.h>
#define CHECK_SQLITE_OK(ret, msg) \
    if (ret != SQLITE_OK) { \
        std::cout << msg << " 错误码: " << ret << std::endl; \
        closeDB(); \
        return false; \
    }


namespace COMMON_LYJ
{
	ColmapImage::ColmapImage()
	{}
	ColmapImage::~ColmapImage()
	{}
	void ColmapImage::write_binary(std::ofstream& stream) const
	{
        WriteBinaryLittleEndian<uint32_t>(&stream, image_id);

        WriteBinaryLittleEndian<double>(&stream, qcw.w());
        WriteBinaryLittleEndian<double>(&stream, qcw.x());
        WriteBinaryLittleEndian<double>(&stream, qcw.y());
        WriteBinaryLittleEndian<double>(&stream, qcw.z());
        WriteBinaryLittleEndian<double>(&stream, tcw.x());
        WriteBinaryLittleEndian<double>(&stream, tcw.y());
        WriteBinaryLittleEndian<double>(&stream, tcw.z());

        WriteBinaryLittleEndian<uint32_t>(&stream, camId);

        const std::string name = imgName + '\0';
        stream.write(name.c_str(), name.size());

        WriteBinaryLittleEndian<uint64_t>(&stream, num_points2D);
        for (int i = 0; i < num_points2D;++i) {
            const auto& point2D = points2D[i];
            WriteBinaryLittleEndian<double>(&stream, point2D(0));
            WriteBinaryLittleEndian<double>(&stream, point2D(1));
            WriteBinaryLittleEndian<uint64_t>(&stream, point3D_ids[i]);
        }
	}
	void ColmapImage::read_binary(std::ifstream& stream)
	{
        image_id = (ReadBinaryLittleEndian<uint32_t>(&stream));

        qcw.w() = ReadBinaryLittleEndian<double>(&stream);
        qcw.x() = ReadBinaryLittleEndian<double>(&stream);
        qcw.y() = ReadBinaryLittleEndian<double>(&stream);
        qcw.z() = ReadBinaryLittleEndian<double>(&stream);
        tcw.x() = ReadBinaryLittleEndian<double>(&stream);
        tcw.y() = ReadBinaryLittleEndian<double>(&stream);
        tcw.z() = ReadBinaryLittleEndian<double>(&stream);

        camId = (ReadBinaryLittleEndian<uint32_t>(&stream));

        imgName.clear();
        char name_char;
        do {
            stream.read(&name_char, 1);
            if (name_char != '\0') {
                imgName += name_char;
            }
        } while (name_char != '\0');

        num_points2D = ReadBinaryLittleEndian<uint64_t>(&stream);

        points2D.clear();
        points2D.reserve(num_points2D);
        point3D_ids.clear();
        point3D_ids.reserve(num_points2D);
        for (size_t j = 0; j < num_points2D; ++j) {
            const double x = ReadBinaryLittleEndian<double>(&stream);
            const double y = ReadBinaryLittleEndian<double>(&stream);
            points2D.emplace_back(x, y);
            point3D_ids.push_back(ReadBinaryLittleEndian<uint64_t>(&stream));
        }
        //for (uint32_t point2D_idx = 0; point2D_idx < num_points2D; ++point2D_idx) {
        //    if (point3D_ids[point2D_idx] != UINT64_MAX) {
        //        image.SetPoint3DForPoint2D(point2D_idx, point3D_ids[point2D_idx]);
        //    }
        //}
	}


    void ColmapSensor::write_binary(std::ofstream& stream) const
    {
        WriteBinaryLittleEndian<int>(&stream, sensor_type);
        WriteBinaryLittleEndian<uint32_t>(&stream, sensor_id);
    }
    void ColmapSensor::read_binary(std::ifstream& stream)
    {
        sensor_type = (ReadBinaryLittleEndian<int>(&stream));
        sensor_id = (ReadBinaryLittleEndian<uint32_t>(&stream));
    }
    void ColmapFrameDataId::write_binary(std::ofstream& stream) const
    {
        sensor.write_binary(stream);
        WriteBinaryLittleEndian<uint64_t>(&stream, id);
    }
    void ColmapFrameDataId::read_binary(std::ifstream& stream)
    {
        sensor.read_binary(stream);
        id = (ReadBinaryLittleEndian<uint64_t>(&stream));
    }



    ColmapFrame::ColmapFrame()
    {}
    ColmapFrame::~ColmapFrame()
    {}
    void ColmapFrame::write_binary(std::ofstream& stream) const
    {
        (WriteBinaryLittleEndian<uint32_t>(&stream, frame_id));
        (WriteBinaryLittleEndian<uint32_t>(&stream, rig_id));

        WriteBinaryLittleEndian<double>(&stream, qcw.w());
        WriteBinaryLittleEndian<double>(&stream, qcw.x());
        WriteBinaryLittleEndian<double>(&stream, qcw.y());
        WriteBinaryLittleEndian<double>(&stream, qcw.z());
        WriteBinaryLittleEndian<double>(&stream, tcw.x());
        WriteBinaryLittleEndian<double>(&stream, tcw.y());
        WriteBinaryLittleEndian<double>(&stream, tcw.z());

        WriteBinaryLittleEndian<uint32_t>(&stream, num_data_ids);
        for (uint32_t j = 0; j < num_data_ids; ++j) {
            data_ids[j].write_binary(stream);
        }
    }
    void ColmapFrame::read_binary(std::ifstream& stream)
    {
        frame_id = (ReadBinaryLittleEndian<uint32_t>(&stream));
        rig_id = (ReadBinaryLittleEndian<uint32_t>(&stream));

        qcw.w() = ReadBinaryLittleEndian<double>(&stream);
        qcw.x() = ReadBinaryLittleEndian<double>(&stream);
        qcw.y() = ReadBinaryLittleEndian<double>(&stream);
        qcw.z() = ReadBinaryLittleEndian<double>(&stream);
        tcw.x() = ReadBinaryLittleEndian<double>(&stream);
        tcw.y() = ReadBinaryLittleEndian<double>(&stream);
        tcw.z() = ReadBinaryLittleEndian<double>(&stream);

        num_data_ids = ReadBinaryLittleEndian<uint32_t>(&stream);
        data_ids.resize(num_data_ids);
        for (uint32_t j = 0; j < num_data_ids; ++j) {
            data_ids[j].read_binary(stream);
        }
    }




    ColmapCamera::ColmapCamera()
    {}
    ColmapCamera::~ColmapCamera()
    {}
    void ColmapCamera::write_binary(std::ofstream& stream) const
    {
        WriteBinaryLittleEndian<uint32_t>(&stream, camera_id);
        (WriteBinaryLittleEndian<int>(&stream, model_id));
        WriteBinaryLittleEndian<uint64_t>(&stream, width);
        WriteBinaryLittleEndian<uint64_t>(&stream, height);
        int num = getParamsNum();
        for (int i = 0; i < num; ++i)
            WriteBinaryLittleEndian<double>(&stream, params[i]);
    }
    void ColmapCamera::read_binary(std::ifstream& stream)
    {
        camera_id = ReadBinaryLittleEndian<uint32_t>(&stream);
        model_id = (ReadBinaryLittleEndian<int>(&stream));
        width = ReadBinaryLittleEndian<uint64_t>(&stream);
        height = ReadBinaryLittleEndian<uint64_t>(&stream);
        int num = getParamsNum();
        params.resize(num, 0.);
        for(int i=0;i<num;++i)
            params[i] = ReadBinaryLittleEndian<double>(&stream);
    }
    int ColmapCamera::getParamsNum() const
    {
        switch (model_id)
        {
        case 0:
            return 4;
        case 1:
            return 4;
        case 2:
            return 6;
        case 3:
            return 7;
        case 4:
            return 8;
        case 5:
            return 8;
        case 6:
            return 12;
        case 7:
            return 5;
        case 8:
            return 6;
        case 9:
            return 7;
        case 10:
            return 10;
        case 11:
            return 12;
        default:
            return 0;
        }
        return 0;
    }





    ColmapPoint::ColmapPoint()
    {}
    ColmapPoint::~ColmapPoint()
    {}
    void ColmapPoint::write_binary(std::ofstream& stream) const
    {
        WriteBinaryLittleEndian<uint64_t>(&stream, point3D_id);

        WriteBinaryLittleEndian<double>(&stream, point3D(0));
        WriteBinaryLittleEndian<double>(&stream, point3D(1));
        WriteBinaryLittleEndian<double>(&stream, point3D(2));
        WriteBinaryLittleEndian<uint8_t>(&stream, color(0));
        WriteBinaryLittleEndian<uint8_t>(&stream, color(1));
        WriteBinaryLittleEndian<uint8_t>(&stream, color(2));
        WriteBinaryLittleEndian<double>(&stream, error);

        WriteBinaryLittleEndian<uint64_t>(&stream, track_length);
        for (size_t j = 0; j < track_length; ++j) {
            WriteBinaryLittleEndian<uint32_t>(&stream, track[j](0));
            WriteBinaryLittleEndian<uint32_t>(&stream, track[j](1));
        }
    }
    void ColmapPoint::read_binary(std::ifstream& stream)
    {
        point3D_id = ReadBinaryLittleEndian<uint64_t>(&stream);

        point3D(0) = ReadBinaryLittleEndian<double>(&stream);
        point3D(1) = ReadBinaryLittleEndian<double>(&stream);
        point3D(2) = ReadBinaryLittleEndian<double>(&stream);
        color(0) = ReadBinaryLittleEndian<uint8_t>(&stream);
        color(1) = ReadBinaryLittleEndian<uint8_t>(&stream);
        color(2) = ReadBinaryLittleEndian<uint8_t>(&stream);
        error = ReadBinaryLittleEndian<double>(&stream);

        track_length = ReadBinaryLittleEndian<uint64_t>(&stream);
        track.resize(track_length);
        for (size_t j = 0; j < track_length; ++j) {
            track[j](0) = ReadBinaryLittleEndian<uint32_t>(&stream);
            track[j](1) = ReadBinaryLittleEndian<uint32_t>(&stream);
        }
    }



    ColmapRig::ColmapRig()
    {}
    ColmapRig::~ColmapRig()
    {}
    void ColmapRig::write_binary(std::ofstream& stream) const
    {
         (WriteBinaryLittleEndian<uint32_t>(&stream, rig_id));
         WriteBinaryLittleEndian<uint32_t>(&stream, num_sensors);

        if (num_sensors > 0) {
            ref_sensor_id.write_binary(stream);
        }

        if (num_sensors > 1) {
            for (uint32_t j = 0; j < num_sensors - 1; ++j) {
                sensor_ids[j].write_binary(stream);

                uint8_t ttt = hasPose[j];
                WriteBinaryLittleEndian<uint8_t>(&stream, ttt);
                if (hasPose[j]) {
                    WriteBinaryLittleEndian<double>(&stream, qi0s[j].w());
                    WriteBinaryLittleEndian<double>(&stream, qi0s[j].x());
                    WriteBinaryLittleEndian<double>(&stream, qi0s[j].y());
                    WriteBinaryLittleEndian<double>(&stream, qi0s[j].z());
                    WriteBinaryLittleEndian<double>(&stream, ti0s[j].x());
                    WriteBinaryLittleEndian<double>(&stream, ti0s[j].y());
                    WriteBinaryLittleEndian<double>(&stream, ti0s[j].z());
                }
            }
        }
    }
    void ColmapRig::read_binary(std::ifstream& stream)
    {
        rig_id = (ReadBinaryLittleEndian<uint32_t>(&stream));
        num_sensors = ReadBinaryLittleEndian<uint32_t>(&stream);

        if (num_sensors > 0) {
            ref_sensor_id.read_binary(stream);
        }

        if (num_sensors > 1) {
            sensor_ids.resize(num_sensors - 1);
            qi0s.resize(num_sensors - 1);
            ti0s.resize(num_sensors - 1);
            hasPose.resize(num_sensors - 1, false);
            for (uint32_t j = 0; j < num_sensors - 1; ++j) {
                sensor_ids[j].read_binary(stream);

                hasPose[j] = ReadBinaryLittleEndian<uint8_t>(&stream);
                if (hasPose[j]) {
                    qi0s[j].w() =
                        ReadBinaryLittleEndian<double>(&stream);
                    qi0s[j].x() =
                        ReadBinaryLittleEndian<double>(&stream);
                    qi0s[j].y() =
                        ReadBinaryLittleEndian<double>(&stream);
                    qi0s[j].z() =
                        ReadBinaryLittleEndian<double>(&stream);
                    ti0s[j].x() =
                        ReadBinaryLittleEndian<double>(&stream);
                    ti0s[j].y() =
                        ReadBinaryLittleEndian<double>(&stream);
                    ti0s[j].z() =
                        ReadBinaryLittleEndian<double>(&stream);
                }
            }
        }
    }



    ColmapData::ColmapData()
    {}
    ColmapData::~ColmapData()
    {}
    void ColmapData::write_binary(std::ofstream& stream) const
    {
        WriteBinaryLittleEndian<uint64_t>(&stream, num_reg_images);
        for (int i = 0; i < num_reg_images; ++i)
            images_[i].write_binary(stream);

        WriteBinaryLittleEndian<uint64_t>(&stream, num_frames);
        for (int i = 0; i < num_frames; ++i)
            frames_[i].write_binary(stream);

        WriteBinaryLittleEndian<uint64_t>(&stream, num_cameras);
        for (int i = 0; i < num_cameras; ++i)
            cameras_[i].write_binary(stream);

        WriteBinaryLittleEndian<uint64_t>(&stream, num_rigs);
        for (int i = 0; i < num_rigs; ++i)
            rigs_[i].write_binary(stream);

        WriteBinaryLittleEndian<uint64_t>(&stream, num_point3Ds);
        for (int i = 0; i < num_point3Ds; ++i)
            point3Ds_[i].write_binary(stream);
    }
    void ColmapData::read_binary(std::ifstream& stream)
    {
        num_reg_images = ReadBinaryLittleEndian<uint64_t>(&stream);
        images_.resize(num_reg_images);
        for (int i = 0; i < num_reg_images; ++i)
            images_[i].read_binary(stream);

        num_frames = ReadBinaryLittleEndian<uint64_t>(&stream);
        frames_.resize(num_frames);
        for (int i = 0; i < num_frames; ++i)
            frames_[i].read_binary(stream);

        num_cameras = ReadBinaryLittleEndian<uint64_t>(&stream);
        cameras_.resize(num_cameras);
        for (int i = 0; i < num_cameras; ++i)
            cameras_[i].read_binary(stream);

        num_rigs = ReadBinaryLittleEndian<uint64_t>(&stream);
        rigs_.resize(num_rigs);
        for (int i = 0; i < num_rigs; ++i)
            rigs_[i].read_binary(stream);

        num_point3Ds = ReadBinaryLittleEndian<uint64_t>(&stream);
        point3Ds_.resize(num_point3Ds);
        for (int i = 0; i < num_point3Ds; ++i)
            point3Ds_[i].read_binary(stream);
    }
    bool ColmapData::writeFromColmap(const std::string _path) const
    {
        {
            std::string path = _path + "/images.bin";
            std::ofstream stream(path, std::ios::trunc | std::ios::binary);
            if (!stream.is_open())
            {
                printf("no images.bin\n");
            }
            else
            {
                WriteBinaryLittleEndian<uint64_t>(&stream, num_reg_images);
                for (int i = 0; i < num_reg_images; ++i)
                    images_[i].write_binary(stream);
            }
        }

        {
            std::string path = _path + "/frames.bin";
            std::ofstream stream(path, std::ios::trunc | std::ios::binary);
            if (!stream.is_open())
            {
                printf("no frames.bin\n");
            }
            else
            {
                WriteBinaryLittleEndian<uint64_t>(&stream, num_frames);
                for (int i = 0; i < num_frames; ++i)
                    frames_[i].write_binary(stream);
            }
        }

        {
            std::string path = _path + "/cameras.bin";
            std::ofstream stream(path, std::ios::trunc | std::ios::binary);
            if (!stream.is_open())
            {
                printf("no cameras.bin\n");
            }
            else
            {
                WriteBinaryLittleEndian<uint64_t>(&stream, num_cameras);
                for (int i = 0; i < num_cameras; ++i)
                    cameras_[i].write_binary(stream);
            }
        }

        {
            std::string path = _path + "/rigs.bin";
            std::ofstream stream(path, std::ios::trunc | std::ios::binary);
            if (!stream.is_open())
            {
                printf("no rigs.bin\n");
            }
            else
            {
                WriteBinaryLittleEndian<uint64_t>(&stream, num_rigs);
                for (int i = 0; i < num_rigs; ++i)
                    rigs_[i].write_binary(stream);
            }
        }

        {
            std::string path = _path + "/points3D.bin";
            std::ofstream stream(path, std::ios::trunc | std::ios::binary);
            if (!stream.is_open())
            {
                printf("no points3D.bin\n");
            }
            else
            {
                WriteBinaryLittleEndian<uint64_t>(&stream, num_point3Ds);
                for (int i = 0; i < num_point3Ds; ++i)
                    point3Ds_[i].write_binary(stream);
                return false;
            }
        }

        return true;
    }
    bool ColmapData::readFromColmap(const std::string _path)
    {
        {
            std::string path = _path + "/images.bin";
            std::ifstream stream(path, std::ios::binary);
            if (!stream.is_open())
            {
                printf("no images.bin\n");
            }
            else
            {
                num_reg_images = ReadBinaryLittleEndian<uint64_t>(&stream);
                images_.resize(num_reg_images);
                for (int i = 0; i < num_reg_images; ++i)
                    images_[i].read_binary(stream);
            }
        }

        {
            std::string path = _path + "/frames.bin";
            std::ifstream stream(path, std::ios::binary);
            if (!stream.is_open())
            {
                printf("no frames.bin\n");
            }
            else
            {
                num_frames = ReadBinaryLittleEndian<uint64_t>(&stream);
                frames_.resize(num_frames);
                for (int i = 0; i < num_frames; ++i)
                    frames_[i].read_binary(stream);
            }
        }

        {
            std::string path = _path + "/cameras.bin";
            std::ifstream stream(path, std::ios::binary);
            if (!stream.is_open())
            {
                printf("no cameras.bin\n");
            }
            else
            {
                num_cameras = ReadBinaryLittleEndian<uint64_t>(&stream);
                cameras_.resize(num_cameras);
                for (int i = 0; i < num_cameras; ++i)
                    cameras_[i].read_binary(stream);
            }
        }

        {
            std::string path = _path + "/rigs.bin";
            std::ifstream stream(path, std::ios::binary);
            if (!stream.is_open())
            {
                printf("no rigs.bin\n");
            }
            else
            {
                num_rigs = ReadBinaryLittleEndian<uint64_t>(&stream);
                rigs_.resize(num_rigs);
                for (int i = 0; i < num_rigs; ++i)
                    rigs_[i].read_binary(stream);
            }
        }

        {
            std::string path = _path + "/points3D.bin";
            std::ifstream stream(path, std::ios::binary);
            if (!stream.is_open())
            {
                printf("no points3D.bin\n");
            }
            else
            {
                num_point3Ds = ReadBinaryLittleEndian<uint64_t>(&stream);
                point3Ds_.resize(num_point3Ds);
                for (int i = 0; i < num_point3Ds; ++i)
                    point3Ds_[i].read_binary(stream);
            }
        }
        
        return true;
    }





    ColmapDataBase::ColmapDataBase()
    {}
    ColmapDataBase::~ColmapDataBase()
    {
        closeDB();
    }
    bool ColmapDataBase::openDB(const std::string _path)
    {
#ifdef USESQLITE3
        sqlite3* sqll = nullptr;
        int ret = sqlite3_open(_path.c_str(), &sqll);
        sql_ = (void*)sqll;
        if (ret != SQLITE_OK) {
            std::cout << "打开数据库失败，错误码：" << ret
                << "，信息：" << sqlite3_errmsg(sqll) << std::endl;
            closeDB(); // 即使打开失败，也需关闭句柄避免内存泄漏
            return false;
        }
        std::cout << "成功打开/创建数据库 test.db" << std::endl;

        return true;
#elif
        return false;
#endif // USESQLITE3
    }
    // 解析相机内参二进制数据（COLMAP 标准格式：float数组）
    // params_blob: 数据库中读取的二进制数据
    // blob_size: 二进制数据长度
    std::vector<float> ColmapDataBase::parse_camera_params(const uint8_t* params_blob, int blob_size) {
        std::vector<float> params;
        if (params_blob == nullptr || blob_size % sizeof(float) != 0) {
            std::cerr << "相机内参二进制数据格式错误" << std::endl;
            return params;
        }
        // 按float类型解析二进制数据
        int param_count = blob_size / sizeof(float);
        const float* param_ptr = reinterpret_cast<const float*>(params_blob);
        for (int i = 0; i < param_count; ++i) {
            params.push_back(param_ptr[i]);
        }
        return params;
    }
    // 新增：查询数据库中所有表名，输出到控制台
    bool ColmapDataBase::query_all_tables() {
        if (!sql_) {
            std::cerr << "错误：未连接数据库" << std::endl;
            return false;
        }
        sqlite3* sqll = (sqlite3*)sql_;

        // 查询sqlite_master系统表，筛选出type='table'的记录（所有表）
        const char* sql = "SELECT name FROM sqlite_master WHERE type = 'table' ORDER BY name;";
        sqlite3_stmt* stmt;
        int ret = sqlite3_prepare_v2(sqll, sql, -1, &stmt, nullptr);
        if (ret != SQLITE_OK) {
            std::cerr << "预处理查询表名SQL失败 错误码: " << ret
                << " 错误信息: " << sqlite3_errmsg(sqll) << std::endl;
            return false;
        }

        // 遍历所有表名
        std::cout << "\n===== 数据库中所有表名 =====" << std::endl;
        int table_count = 0;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            const char* table_name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            std::cout << ++table_count << ". " << table_name << std::endl;
        }

        if (table_count == 0) {
            std::cout << "数据库中无任何表" << std::endl;
        }

        sqlite3_finalize(stmt);
        return true;
    }

    // 新增：查询指定表的完整结构（字段属性、约束等）
    // table_name：要查询的表名（如"images"、"cameras"）
    bool ColmapDataBase::query_table_schema(const std::string& table_name) {
        if (!sql_) {
            std::cerr << "错误：未连接数据库" << std::endl;
            return false;
        }
        sqlite3* sqll = (sqlite3*)sql_;

        // 查询指定表的CREATE TABLE语句（带参数绑定，防SQL注入）
        const char* sql = "SELECT sql FROM sqlite_master WHERE type = 'table' AND name = ?;";
        sqlite3_stmt* stmt;
        int ret = sqlite3_prepare_v2(sqll, sql, -1, &stmt, nullptr);
        if (ret != SQLITE_OK) {
            std::cerr << "预处理查询表结构SQL失败 错误码: " << ret
                << " 错误信息: " << sqlite3_errmsg(sqll) << std::endl;
            return false;
        }

        // 绑定表名参数（?对应位置1）
        sqlite3_bind_text(stmt, 1, table_name.c_str(), -1, SQLITE_TRANSIENT);

        // 提取表结构
        std::cout << "\n===== 表 " << table_name << " 的完整结构 =====" << std::endl;
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            const char* create_sql = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            if (create_sql) {
                std::cout << create_sql << std::endl;
            }
            else {
                std::cout << "表 " << table_name << " 存在，但无结构信息" << std::endl;
            }
        }
        else {
            std::cout << "错误：数据库中不存在表 " << table_name << std::endl;
        }

        sqlite3_finalize(stmt);
        return true;
    }
    // 查询图像元信息（images表）：image_id, 文件名, 相机ID, 宽, 高
    // limit: 限制查询数量（0表示查询所有）
    bool ColmapDataBase::query_images(int limit) {
        if (!sql_) {
            std::cerr << "错误：未连接数据库" << std::endl;
            return false;
        }
        sqlite3* sqll = (sqlite3*)sql_;

        // 构造SQL查询语句
        std::string sql = "SELECT image_id, name, camera_id FROM images";
        if (limit > 0) {
            sql += " LIMIT " + std::to_string(limit);
        }
        sql += ";";

        // 执行SQL查询
        sqlite3_stmt* stmt;
        int ret = sqlite3_prepare_v2(sqll, sql.c_str(), -1, &stmt, nullptr);
        CHECK_SQLITE_OK(ret, "预处理图像查询SQL失败");

        // 遍历查询结果
        std::cout << "\n===== 图像元信息（前" << limit << "条）=====" << std::endl;
        std::cout << "ID 文件名 相机ID" << std::endl;

        while (sqlite3_step(stmt) == SQLITE_ROW) {
            // 提取列数据：SQLite3_COLUMN_* 对应数据类型
            int image_id = sqlite3_column_int(stmt, 0);
            const char* img_name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            int camera_id = sqlite3_column_int(stmt, 2);

            // 打印结果（格式化输出）
            std::cout << image_id << " " << img_name << " " << camera_id << std::endl;
        }

        // 释放语句句柄
        sqlite3_finalize(stmt);
        return true;
    }

    // 查询相机参数（cameras表）：相机ID, 模型, 宽, 高, 内参（fx/fy/cx/cy等）
    bool ColmapDataBase::query_cameras() {
        if (!sql_) {
            std::cerr << "错误：未连接数据库" << std::endl;
            return false;
        }
        sqlite3* sqll = (sqlite3*)sql_;

        // SQL查询：params为BLOB类型（存储二进制内参）
        const char* sql = "SELECT camera_id, model, width, height, params FROM cameras;";
        sqlite3_stmt* stmt;
        int ret = sqlite3_prepare_v2(sqll, sql, -1, &stmt, nullptr);
        CHECK_SQLITE_OK(ret, "预处理相机查询SQL失败");

        // 遍历相机数据
        std::cout << "\n===== 相机内参信息 =====" << std::endl;
        std::cout << "相机ID 相机模型 宽度 高度 内参（fx, fy, cx, cy, ...）" << std::endl;

        while (sqlite3_step(stmt) == SQLITE_ROW) {
            int cam_id = sqlite3_column_int(stmt, 0);
            const char* cam_model = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            int width = sqlite3_column_int(stmt, 2);
            int height = sqlite3_column_int(stmt, 3);
            // 提取BLOB类型数据：二进制内参
            const uint8_t* params_blob = reinterpret_cast<const uint8_t*>(sqlite3_column_blob(stmt, 4));
            int blob_size = sqlite3_column_bytes(stmt, 4);

            // 解析二进制内参为float数组
            std::vector<float> cam_params = parse_camera_params(params_blob, blob_size);

            // 打印结果
            std::cout << cam_id
                << " " << cam_model
                << " " << width
                << " " << height << " ";
            for (float p : cam_params) {
                std::cout << p << " ";
            }
            std::cout << std::endl;
        }

        sqlite3_finalize(stmt);
        return true;
    }

    // 查询指定图像的特征点数量（keypoints表）
    // image_id: 目标图像ID
    bool ColmapDataBase::query_keypoint_count(int image_id) {
        if (!sql_) {
            std::cerr << "错误：未连接数据库" << std::endl;
            return false;
        }
        sqlite3* sqll = (sqlite3*)sql_;

        // 构造带参数的SQL（避免SQL注入）
        std::string sql = "SELECT rows FROM keypoints WHERE image_id = ?;";
        sqlite3_stmt* stmt;
        int ret = sqlite3_prepare_v2(sqll, sql.c_str(), -1, &stmt, nullptr);
        CHECK_SQLITE_OK(ret, "预处理特征点查询SQL失败");

        // 绑定参数：? 对应位置1，绑定int类型
        sqlite3_bind_int(stmt, 1, image_id);

        // 执行查询
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            int kp_count = sqlite3_column_int(stmt, 0);
            std::cout << "\n===== 特征点数量 =====" << std::endl;
            std::cout << "图像ID " << image_id << " 的特征点数量：" << kp_count << std::endl;
        }
        else {
            std::cout << "图像ID " << image_id << " 未找到特征点数据" << std::endl;
        }

        sqlite3_finalize(stmt);
        return true;
    }
    /**
     * @brief 获取指定image_id的所有特征点坐标
     * @param db_path COLMAP数据库文件路径
     * @param target_image_id 目标帧的image_id
     * @return 特征点坐标集合，空则表示查询失败/无特征点
     */
    bool ColmapDataBase::get_keypoint_coords(int target_image_id, std::vector<Eigen::Vector2f>& _ps) {
        _ps.clear();

        if (!sql_) {
            std::cerr << "错误：未连接数据库" << std::endl;
            return false;
        }
        sqlite3* sqll = (sqlite3*)sql_;

        sqlite3_stmt* stmt = nullptr;
        const void* blob_data = nullptr;     // 存储BLOB二进制数据
        int blob_byte_size = 0;              // BLOB数据总字节数
        int kp_count = 0;                    // 特征点数量（rows字段）
        int kp_cols = 0;                     // 特征点维度（cols字段，固定6）

        // 步骤2：预处理SQL，同时查询rows/cols/data三个字段
        const char* sql = "SELECT rows, cols, data FROM keypoints WHERE image_id = ?;";
        int ret = sqlite3_prepare_v2(sqll, sql, -1, &stmt, nullptr);
        if (ret != SQLITE_OK) {
            std::cerr << "预处理SQL失败，详情：" << sqlite3_errmsg(sqll) << std::endl;
            sqlite3_close(sqll);
            return false;
        }

        // 步骤3：绑定target_image_id参数（?索引从1开始）
        sqlite3_bind_int(stmt, 1, target_image_id);

        // 步骤4：执行查询，验证是否存在记录
        if (sqlite3_step(stmt) != SQLITE_ROW) {
            std::cerr << "未找到image_id=" << target_image_id << "的特征点数据（可能未提取特征）" << std::endl;
            sqlite3_finalize(stmt);
            sqlite3_close(sqll);
            return false;
        }

        // 步骤5：提取rows/cols/data字段，完成数据验证
        kp_count = sqlite3_column_int(stmt, 0);    // 第0列：rows（特征点数量）
        kp_cols = sqlite3_column_int(stmt, 1);     // 第1列：cols（特征点维度，固定6）
        blob_data = sqlite3_column_blob(stmt, 2);  // 第2列：data（BLOB二进制数据）
        blob_byte_size = sqlite3_column_bytes(stmt, 2); // 获取BLOB总字节数

        // 核心验证：BLOB字节数必须等于 数量×维度×float字节数（4），否则数据损坏
        const int expected_byte_size = kp_count * kp_cols * sizeof(float);
        if (blob_byte_size != expected_byte_size || kp_cols != 6 || kp_count <= 0) {
            std::cerr << "特征点数据损坏/不合法："
                << "实际字节数=" << blob_byte_size
                << "，预期字节数=" << expected_byte_size
                << "，cols=" << kp_cols << std::endl;
            sqlite3_finalize(stmt);
            sqlite3_close(sqll);
            return false;
        }

        // 步骤6：解析BLOB数据，提取所有特征点(x,y)坐标（核心步骤）
        const float* kp_data = static_cast<const float*>(blob_data); // 强转为float指针，逐元素读取
        _ps.resize(kp_count);
        for (int i = 0; i < kp_count; ++i) {
            // 单特征点占6个float，索引偏移：i*6
            const int offset = i * 6;
            const float x = kp_data[offset];     // 第0个float：x坐标
            const float y = kp_data[offset + 1]; // 第1个float：y坐标
            _ps[i](0) = x;        // 存入坐标集合
            _ps[i](1) = y;        // 存入坐标集合
        }

        // 步骤7：释放所有资源（必须执行，避免内存泄漏）
        sqlite3_finalize(stmt);

        // 打印解析结果（可选，验证用）
        std::cout << "成功解析image_id=" << target_image_id
            << "的特征点，共" << kp_count << "个，坐标示例：" << std::endl;

        return true;
    }
    
    bool ColmapDataBase::readDataBase()
    {
#ifdef USESQLITE3
        read_all_cameras();
        read_all_images();
        read_all_keypoints();
        read_all_matches();
        read_all_two_view_geometries();
        return true;
#elif
        return false;
#endif // USESQLITE3
    }
    bool ColmapDataBase::closeDB()
    {
#ifdef USESQLITE3
        if (sql_)
        {
            sqlite3* sqll = (sqlite3*)sql_;
            sqlite3_close(sqll);
        }
        return true;
#elif
        return false;
#endif // USESQLITE3
    }



    bool ColmapDataBase::read_all_cameras() {
        sqlite3* db = (sqlite3*)sql_;
        g_cameras.clear();
        const char* sql = "SELECT camera_id, model, width, height, params FROM cameras;";
        sqlite3_stmt* stmt = nullptr;
        int ret = sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
        if (ret != SQLITE_OK) {
            std::cerr << "[ERROR] 预处理cameras表SQL失败：" << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        // 遍历所有相机记录
        int cam_count = 0;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            Camera cam;
            cam.camera_id = sqlite3_column_int(stmt, 0);
            cam.model = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            cam.width = sqlite3_column_int(stmt, 2);
            cam.height = sqlite3_column_int(stmt, 3);
            // 解析params BLOB为相机内参float数组
            const void* params_blob = sqlite3_column_blob(stmt, 4);
            int params_size = sqlite3_column_bytes(stmt, 4);
            cam.params = blob_to_float(params_blob, params_size);

            g_cameras[cam.camera_id] = cam;
            cam_count++;
        }

        sqlite3_finalize(stmt);
        std::cout << "[INFO] 成功读取 cameras 表 | 共 " << cam_count << " 个相机" << std::endl;
        return true;
    }
    bool ColmapDataBase::read_all_images() {
        sqlite3* db = (sqlite3*)sql_;
        g_images.clear();
        if (g_cameras.empty()) {
            std::cerr << "[ERROR] 请先读取cameras表，再读取images表！" << std::endl;
            return false;
        }

        const char* sql = "SELECT image_id, name, camera_id FROM images;";
        sqlite3_stmt* stmt = nullptr;
        int ret = sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
        if (ret != SQLITE_OK) {
            std::cerr << "[ERROR] 预处理images表SQL失败：" << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        // 遍历所有图像记录，关联相机
        int img_count = 0;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            Image img;
            img.image_id = sqlite3_column_int(stmt, 0);
            img.name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            img.camera_id = sqlite3_column_int(stmt, 2);
            // 关联相机指针（O(1)查询，无需反复查表）
            auto cam_it = g_cameras.find(img.camera_id);
            if (cam_it != g_cameras.end()) {
                img.camera = &(cam_it->second);
            }

            g_images[img.image_id] = img;
            img_count++;
        }

        sqlite3_finalize(stmt);
        std::cout << "[INFO] 成功读取 images 表 | 共 " << img_count << " 张图像" << std::endl;
        return true;
    }
    bool ColmapDataBase::read_all_keypoints() {
        sqlite3* db = (sqlite3*)sql_;
        g_keypoints.clear();
        if (g_images.empty()) {
            std::cerr << "[ERROR] 请先读取images表，再读取keypoints表！" << std::endl;
            return false;
        }

        const char* sql = "SELECT image_id, rows, cols, data FROM keypoints;";
        sqlite3_stmt* stmt = nullptr;
        int ret = sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
        if (ret != SQLITE_OK) {
            std::cerr << "[ERROR] 预处理keypoints表SQL失败：" << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        // 遍历所有特征点记录，解析x/y坐标
        int kp_count = 0;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            KeyPoint kp;
            kp.image_id = sqlite3_column_int(stmt, 0);
            kp.count = sqlite3_column_int(stmt, 1);
            int cols = sqlite3_column_int(stmt, 2);
            // 解析data BLOB为特征点float数组（单特征点6个float，前2个为x/y）
            const void* data_blob = sqlite3_column_blob(stmt, 3);
            int data_size = sqlite3_column_bytes(stmt, 3);

            // 数据合法性验证
            if (cols != 6 || kp.count <= 0 || data_size != kp.count * 6 * sizeof(float)) {
                std::cerr << "[WARN] 特征点数据不合法 | image_id=" << kp.image_id << std::endl;
                continue;
            }

            // 提取所有特征点(x,y)坐标
            const float* kp_data = static_cast<const float*>(data_blob);
            kp.coords.reserve(kp.count); // 预分配内存，提升效率
            for (int i = 0; i < kp.count; ++i) {
                int offset = i * 6;
                float x = kp_data[offset];
                float y = kp_data[offset + 1];
                kp.coords.emplace_back(x, y);
            }

            g_keypoints[kp.image_id] = kp;
            kp_count++;
        }

        sqlite3_finalize(stmt);
        std::cout << "[INFO] 成功读取 keypoints 表 | 共 " << kp_count << " 张图像的特征点" << std::endl;
        return true;
    }
    bool ColmapDataBase::read_all_matches() {
        sqlite3* db = (sqlite3*)sql_;
        g_matches.clear();
        if (g_keypoints.empty()) {
            std::cerr << "[ERROR] 请先读取keypoints表，再读取matches表！" << std::endl;
            return false;
        }

        const char* sql = "SELECT pair_id, rows, cols, data FROM matches;";
        sqlite3_stmt* stmt = nullptr;
        int ret = sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
        if (ret != SQLITE_OK) {
            std::cerr << "[ERROR] 预处理matches表SQL失败：" << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        // 遍历所有匹配记录，解析匹配对
        int match_count = 0;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            Match match;
            // pair_id是64位无符号整数，需用sqlite3_column_int64提取
            match.pair_id = static_cast<uint64_t>(sqlite3_column_int64(stmt, 0));
            // 解码pair_id为图像对
            auto [img1, img2] = decode_pair_id(match.pair_id);
            match.image_id1 = img1;
            match.image_id2 = img2;
            std::string binary_sep = uint64_to_binary_with_sep(match.pair_id);
            //if(img1 == 2)
                //std::cout << "带分隔符的64位二进制：" << binary_sep << std::endl;
            // 解析data BLOB为匹配对数组（单匹配对2个int，分别为两个图像的特征点索引）
            const void* data_blob = sqlite3_column_blob(stmt, 3);
            int data_size = sqlite3_column_bytes(stmt, 3);
            match.count = sqlite3_column_int(stmt, 1);

            // 数据合法性验证（单匹配对2个int，占8字节）
            if (match.count <= 0 || data_size != match.count * 2 * sizeof(int) ||
                g_keypoints.find(img1) == g_keypoints.end() ||
                g_keypoints.find(img2) == g_keypoints.end()) {
                //std::cerr << "[WARN] 匹配数据不合法 | pair_id=" << match.pair_id << std::endl;
                //std::cout << "带分隔符的64位二进制：" << binary_sep << std::endl;
                continue;
            }

            // 提取所有匹配对：<img1特征点索引, img2特征点索引>
            const int* match_data = static_cast<const int*>(data_blob);
            match.matches.reserve(match.count);
            for (int i = 0; i < match.count; ++i) {
                int offset = i * 2;
                int kp_idx1 = match_data[offset];   // 图像1的特征点索引
                int kp_idx2 = match_data[offset + 1];// 图像2的特征点索引
                match.matches.emplace_back(kp_idx1, kp_idx2);
            }

            g_matches.push_back(match);
            match_count++;
        }

        sqlite3_finalize(stmt);
        std::cout << "[INFO] 成功读取 matches 表 | 共 " << match_count << " 组图像对匹配" << std::endl;
        return true;
    }
    bool ColmapDataBase::read_all_two_view_geometries() {
        sqlite3* db = (sqlite3*)sql_;
        g_two_view_geoms.clear();
        // 依赖检查：必须先读取matches表（保证pair_id存在）
        if (g_matches.empty()) {
            std::cerr << "[ERROR] 请先读取matches表，再读取two_view_geometries表！" << std::endl;
            return false;
        }

        // 预处理SQL：查询所有字段（pair_id + 数量/维度 + 所有BLOB）
        const char* sql = "SELECT pair_id, rows, cols, data, config, F, E, H, qvec, tvec FROM two_view_geometries;";
        sqlite3_stmt* stmt = nullptr;
        int ret = sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
        if (ret != SQLITE_OK) {
            std::cerr << "[ERROR] 预处理two_view_geometries表SQL失败：" << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        // 遍历所有两视图几何记录
        int geom_count = 0;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            TwoViewGeometry tvg;
            // 1. 提取基础字段：pair_id + 内点数量 + 维度
            tvg.pair_id = static_cast<uint64_t>(sqlite3_column_int64(stmt, 0));
            tvg.inlier_count = sqlite3_column_int(stmt, 1);
            tvg.cols = sqlite3_column_int(stmt, 2);
            // 解码pair_id为图像对
            auto [img1, img2] = decode_pair_id(tvg.pair_id);
            tvg.image_id1 = img1;
            tvg.image_id2 = img2;

            // 2. 验证基础合法性：cols固定为3 + 内点数量>0 + pair_id在matches表中存在
            auto match_it = std::find_if(g_matches.begin(), g_matches.end(),
                [&](const Match& m) { return m.pair_id == tvg.pair_id; });
            if ((tvg.cols != 3 && tvg.cols != 2) || tvg.inlier_count <= 0 || match_it == g_matches.end()) {
                //std::cerr << "[WARN] 两视图几何数据不合法 | pair_id=" << tvg.pair_id << std::endl;
                continue;
            }

            // 3. 解析有效内点BLOB（核心：单内点3个float，cols=3）
            const void* inlier_blob = sqlite3_column_blob(stmt, 3);
            int inlier_blob_size = sqlite3_column_bytes(stmt, 3);
            int inliner_config = sqlite3_column_int(stmt, 4);
            const int expected_inlier_size = tvg.inlier_count * tvg.cols * sizeof(float);
            if (inlier_blob != nullptr && inlier_blob_size == expected_inlier_size) {
                const int* inlier_data = static_cast<const int*>(inlier_blob);
                tvg.inliers.reserve(tvg.inlier_count);
                for (int i = 0; i < tvg.inlier_count; ++i) {
                    int offset = i * tvg.cols;
                    // 单内点：img1特征点索引(0)、img2特征点索引(1)、重投影误差(2)
                    int kp_idx1 = static_cast<int>(inlier_data[offset]);
                    int kp_idx2 = static_cast<int>(inlier_data[offset + 1]);
                    float reproj_error = 0; //inlier_data[offset + 2];
                    tvg.inliers.emplace_back(kp_idx1, kp_idx2, reproj_error);
                }
            }

            // 4. 解析矩阵/位姿BLOB（固定长度，调用专用工具函数）
            tvg.F = blob_to_double_fixed(sqlite3_column_blob(stmt, 5), sqlite3_column_bytes(stmt, 5), 9);  // 基础矩阵3x3
            tvg.E = blob_to_double_fixed(sqlite3_column_blob(stmt, 6), sqlite3_column_bytes(stmt, 6), 9);  // 本质矩阵3x3
            tvg.H = blob_to_double_fixed(sqlite3_column_blob(stmt, 7), sqlite3_column_bytes(stmt, 7), 9);  // 单应矩阵3x3
            tvg.qvec = blob_to_double_fixed(sqlite3_column_blob(stmt, 8), sqlite3_column_bytes(stmt, 8), 4);// 四元数4个元素
            tvg.tvec = blob_to_double_fixed(sqlite3_column_blob(stmt, 9), sqlite3_column_bytes(stmt, 9), 3);// 平移向量3个元素

            // 5. 存储到全局容器（按pair_id索引）
            g_two_view_geoms[tvg.pair_id] = tvg;
            geom_count++;
        }

        // 释放资源
        sqlite3_finalize(stmt);
        std::cout << "[INFO] 成功读取 two_view_geometries 表 | 共 " << geom_count << " 组两视图几何验证记录" << std::endl;
        return true;
    }

}

#endif // USESQLITE3
