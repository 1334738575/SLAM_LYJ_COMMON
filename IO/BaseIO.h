#ifndef SLAM_LYJ_COMMON_BASEIO_H
#define SLAM_LYJ_COMMON_BASEIO_H
/******************************************************/
/*
* 读写二进制文件
* 指针中包含指针读取会出错，在自定义类中需要注意指针的读取和释放
* 自定义类需要实现void write_binary(std::ofstream& os) const 和 void read_binary(std::ifstream& os)
*/
/******************************************************/

#include <base/Base.h>
#include <fstream>
#include <iostream>
#include <type_traits>

namespace COMMON_LYJ
{
	// 步骤1：自定义清理类型修饰符（C++17 可直接用 remove_cvref_t，也可兼容写法）
	template <typename T>
	using RemoveCVRef = std::remove_cv_t<std::remove_reference_t<T>>;
	
	
	// 步骤2：核心模板（用偏特化替代 conditional_t，避免分支解析问题）
	// 主模板：非指针类型 → 返回自身
	template <typename T, typename = void>
	struct TargetType {
		using CleanT = RemoveCVRef<T>;
		static constexpr bool is_ptr = false;  // 标记：非 pointer
		using type = CleanT;
	};
	// 偏特化：指针类型 → 返回指向类型（仅指针类型匹配此模板）,declval创建伪对象
	template <typename T>
	struct TargetType<T, std::void_t<decltype(*std::declval<RemoveCVRef<T>>())>> {
		using CleanT = RemoveCVRef<T>;
		static constexpr bool is_ptr = true;  // 标记：是 pointer
		// 仅指针类型会走到这里，解引用推导指向类型（无报错）
		using type = std::remove_reference_t<decltype(*std::declval<CleanT>())>;
	};
	// 步骤3：简化别名（C++17 简洁写法）
	template <typename T>
	constexpr bool is_ptr_v = TargetType<T>::is_ptr; // 是否是 vector
	template <typename T>
	using target_type_t = typename TargetType<T>::type;
	// 辅助函数：打印类型名（调试用）
	template <typename T>
	void print_type(const char* desc) {
		std::cout << "  是否是指针: " << (is_ptr_v<T> ? "是" : "否") << "\n";
		std::cout << desc << "：" << typeid(T).name() << std::endl;
	}


	// 步骤2：核心模板 - 主模板（非 std::vector 类型 → 返回自身）
	template <typename T, typename = void>
	struct IsVectorAndGetElementType {
		static constexpr bool is_vector = false; // 标记：非 vector
		using type = RemoveCVRef<T>;             // 非 vector 则返回自身类型
	};
	// 步骤2.1：偏特化 - 仅匹配 std::vector<...> 类型
	// 模板参数：匹配任意 std::vector<ValueT, AllocT>
	template <typename ValueT, typename AllocT>
	struct IsVectorAndGetElementType<const std::vector<ValueT, AllocT>&, void> {
		static constexpr bool is_vector = true;  // 标记：是 vector
		using type = ValueT;                     // 返回 vector 的元素类型（核心需求）
	};
	template <typename ValueT, typename AllocT>
	struct IsVectorAndGetElementType<const std::vector<ValueT, AllocT>, void> {
		static constexpr bool is_vector = true;  // 标记：是 vector
		using type = ValueT;                     // 返回 vector 的元素类型（核心需求）
	};
	template <typename ValueT, typename AllocT>
	struct IsVectorAndGetElementType<std::vector<ValueT, AllocT>&, void> {
		static constexpr bool is_vector = true;  // 标记：是 vector
		using type = ValueT;                     // 返回 vector 的元素类型（核心需求）
	};
	template <typename ValueT, typename AllocT>
	struct IsVectorAndGetElementType<std::vector<ValueT, AllocT>, void> {
		static constexpr bool is_vector = true;  // 标记：是 vector
		using type = ValueT;                     // 返回 vector 的元素类型（核心需求）
	};
	// 步骤3：简化别名（对外接口）
	template <typename T>
	constexpr bool is_vector_v = IsVectorAndGetElementType<T>::is_vector; // 是否是 vector
	template <typename T>
	using vector_element_t = typename IsVectorAndGetElementType<T>::type; // 获取 vector 元素类型/非 vector 自身类型
	// 辅助函数：打印结果（调试用）
	template <typename T>
	void print_vector_info(const char* desc) {
		std::cout << desc << ":\n";
		std::cout << "  是否是 std::vector: " << (is_vector_v<T> ? "是" : "否") << "\n";
		if (is_vector_v<T>)
		{
			using TTT = vector_element_t<T>;
			std::cout << "  是否是 std::vector: " << (is_vector_v<TTT> ? "是" : "否") << "\n";
			if (is_vector_v<TTT>)
				std::cout << "  目标类型: " << typeid(vector_element_t<TTT>).name() << "\n\n";
			else
				std::cout << "  目标类型: " << typeid(vector_element_t<T>).name() << "\n\n";
		}
		else
			std::cout << "  目标类型: " << typeid(vector_element_t<T>).name() << "\n\n";
	}



	// 第一步：定义检测模板（核心）
	template <typename T, typename = void>
	struct has_func : std::false_type {
		static constexpr bool is_user = false;  // 标记：非 pointer
	}; // 默认：不存在
	// 特化版本：如果 T 包含 void func()，则匹配此版本（推导成功）
	template <typename T>
	struct has_func<T, std::void_t<decltype(std::declval<T>().write_binary(std::declval<std::ofstream&>()))>> : std::true_type {
		static constexpr bool is_user = true;  // 标记：非 pointer
	};
	// 步骤3：简化别名（对外接口）
	template <typename T>
	constexpr bool is_user_v = has_func<T>::is_user; // 是否自定义





	/*********************************write*****************************************/
	template<typename T>
	static void writeBinDirect(std::ofstream& file, const T* value, const uint64_t sz)
	{
		file.write(reinterpret_cast<const char*>(value), sizeof(T)*sz);
	}
	template<typename T>
	static void writeBinBasic(std::ofstream& file, const T& value)
	{
		file.write(reinterpret_cast<const char*>(&value), sizeof(T));
	}
	template<typename T>
	static void writeBinString(std::ofstream& file, const T& value)
	{
		//std::cout << "no string" << std::endl;
	}
	template<>
	static void writeBinString<std::string>(std::ofstream& file, const std::string& value)
	{
		//std::cout << "string" << std::endl;
		size_t sz = value.size();
		writeBinBasic<size_t>(file, sz);
		file.write(reinterpret_cast<const char*>(value.c_str()), sz * sizeof(char));
	}
	template<typename T>
	static void writeBinUser(std::ofstream& file, const T& value)
	{
		//SLAM_LYJ::BaseLYJ* ptr = (SLAM_LYJ::BaseLYJ*)(&value);
		//ptr->write_binary(file);
		value.write_binary(file);
	}
	template <typename T>
	static void writeBinBasicVectorEvery(std::ofstream& file, const std::vector<T>& vec) {
		size_t sz = vec.size();
		writeBinBasic<size_t>(file, sz);
		file.write(reinterpret_cast<const char*>(vec.data()), sz * sizeof(T));
	}

	template<typename T, std::enable_if_t<!is_ptr_v<T>, bool> = true, std::enable_if_t<!is_vector_v<T>, bool> = true, std::enable_if_t<!is_user_v<T>, bool> = true>
	static void writeBin(std::ofstream& file, const T& value)
	{
		using RT = RemoveCVRef<T>;             // 非 vector 则返回自身类型
		std::string tpName = typeid(value).name();
		//base type
		if (
			tpName == "char"
			|| tpName == "unsigned char"
			|| tpName == "short"
			|| tpName == "unsigned short"
			|| tpName == "int"
			|| tpName == "unsigned int"
			|| tpName == "long"
			|| tpName == "unsigned long"
			|| tpName == "long long"
			|| tpName == "unsigned long long"
			|| tpName == "float"
			|| tpName == "double"
			)
		{
			writeBinBasic<T>(file, value);
		}
		//std::vector
		else if (is_vector_v<RT>)
		{
			writeBin<RT>(file, value);
		}
		//std::string
		//if (tpName == "class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >")
		else if (tpName.find("class std::basic_string<char") == 0)
		{
			writeBinString<RT>(file, value);
		}
		else if (is_ptr_v<RT>)
		{
			//using TT = target_type_t<RT>;
			writeBin<RT>(file, value);
		}
		//defined by user
		else if(has_func<RT>::value == 1)
		{
			writeBin<RT>(file, value);
		}
		else
		{
			std::cout << "write binary type error!" << std::endl;
		}
	}
	template<typename T, std::enable_if_t<is_ptr_v<T>, bool> = true, std::enable_if_t<!is_vector_v<T>, bool> = true, std::enable_if_t<!is_user_v<T>, bool> = true>
	static void writeBin(std::ofstream& file, const T& value)
	{
		using RT = target_type_t<T>;
		using T2 = RemoveCVRef<RT>;             // 非 vector 则返回自身类型
		std::string tpName = typeid(T2).name();
		//base type
		if (
			tpName == "char"
			|| tpName == "unsigned char"
			|| tpName == "short"
			|| tpName == "unsigned short"
			|| tpName == "int"
			|| tpName == "unsigned int"
			|| tpName == "long"
			|| tpName == "unsigned long"
			|| tpName == "long long"
			|| tpName == "unsigned long long"
			|| tpName == "float"
			|| tpName == "double"
			)
		{
			writeBinBasic<T2>(file, *value);
		}
		//std::vector
		else if (is_vector_v<T2>)
		{
			writeBin<T2>(file, *value);
		}
		//std::string
		//if (tpName == "class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >")
		else if (tpName.find("class std::basic_string<char") == 0)
		{
			writeBinString<T2>(file, *value);
		}
		else if (is_ptr_v<T2>)
		{
			//using TT = target_type_t<T2>;
			writeBin<T2>(file, *value);
		}
		//defined by user
		else if (has_func<T2>::value == 1)
		{
			writeBin<T2>(file, *value);
		}
		else
		{
			std::cout << "write binary type error!" << std::endl;
		}
	}
	template<typename T, std::enable_if_t<!is_ptr_v<T>, bool> = true, std::enable_if_t<is_vector_v<T>, bool> = true, std::enable_if_t<!is_user_v<T>, bool> = true>
	static void writeBin(std::ofstream& file, const T& value)
	{
		using RT = vector_element_t<T>;
		using T2 = RemoveCVRef<RT>;             // 非 vector 则返回自身类型
		std::string tpName = typeid(T2).name();
		size_t sz = value.size();
		//base type
		if (
			tpName == "char"
			|| tpName == "unsigned char"
			|| tpName == "short"
			|| tpName == "unsigned short"
			|| tpName == "int"
			|| tpName == "unsigned int"
			|| tpName == "long"
			|| tpName == "unsigned long"
			|| tpName == "long long"
			|| tpName == "unsigned long long"
			|| tpName == "float"
			|| tpName == "double"
			)
		{
			writeBinBasicVectorEvery<T2>(file, value);
		}
		//std::vector
		else if (is_vector_v<T2>)
		{
			for (int i = 0; i < sz; ++i)
				writeBin<T2>(file, value[i]);
		}
		//std::string
		//if (tpName == "class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >")
		else if (tpName.find("class std::basic_string<char") == 0)
		{
			for (int i = 0; i < sz; ++i)
				writeBinString<T2>(file, value[i]);
		}
		else if (is_ptr_v<T2>)
		{
			for (int i = 0; i < sz; ++i)
				writeBin<T2>(file, value[i]);
		}
		//defined by user
		else if (has_func<T2>::value == 1)
		{
			for (int i = 0; i < sz; ++i)
				writeBin<T2>(file, value[i]);
		}
		else
		{
			std::cout << "write binary type error!" << std::endl;
		}
	}
	template<typename T, std::enable_if_t<!is_ptr_v<T>, bool> = true, std::enable_if_t<!is_vector_v<T>, bool> = true, std::enable_if_t<is_user_v<T>, bool> = true>
	static void writeBin(std::ofstream& file, const T& value)
	{
		writeBinUser<T>(file, value);
	}


	template<typename... Ts>
	static void writeBinDatas(std::ofstream& file, const Ts&... args)
	{
		//std::cout << "base" << std::endl;
	}
	template<typename T>
	static void writeBinDatas(std::ofstream& file, const T& _first)
	{
		//std::cout << typeid(_first).name() << std::endl;
		writeBin<T>(file, _first);
	}
	template<typename T, typename... Args>
	static void writeBinDatas(std::ofstream& file, const T& _first, const Args&... rest)
	{
		//std::cout << typeid(_first).name() << std::endl;
		writeBin<T>(file, _first);
		writeBinDatas<Args...>(file, rest...);
	}
	template<typename... Args>
	static bool writeBinFile(const std::string& _fileName, const Args&... args)
	{
		std::cout << "write filename: " << _fileName << std::endl;
		std::ofstream f(_fileName, std::ios::binary);
		if (!f.is_open())
		{
			std::cout << "open file failed!" << std::endl;
			return false;
		}
		writeBinDatas<Args...>(f, args...);
		f.close();
		return true;
	}



	/*********************************read*****************************************/
	template<typename T>
	static void readBinDirect(std::ifstream& file, T* value, const uint64_t sz)
	{
		file.read(reinterpret_cast<char*>(value), sizeof(T)*sz);
	}
	template<typename T>
	static void readBinBasic(std::ifstream& file, T& value)
	{
		file.read(reinterpret_cast<char*>(&value), sizeof(T));
	}
	template<typename T>
	static void readBinString(std::ifstream& file, T& value)
	{
		//std::cout << "no string" << std::endl;
	}
	template<>
	static void readBinString<std::string>(std::ifstream& file, std::string& value)
	{
		//std::cout << "string" << std::endl;
		size_t sz;
		readBinBasic<size_t>(file, sz);
		value.resize(sz);
		std::vector<char> chs(sz);
		file.read(reinterpret_cast<char*>(chs.data()), sz * sizeof(char));
		value.assign(chs.data(), sz);
	}
	template<typename T>
	static void readBinUser(std::ifstream& file, T& value)
	{
		//SLAM_LYJ::BaseLYJ* ptr = (SLAM_LYJ::BaseLYJ*)(&value);
		//ptr->read_binary(file);
		value.read_binary(file);
	}
	template <typename T>
	static void readBinBasicVectorEvery(std::ifstream& file, std::vector<T>& vec) {
		size_t sz;
		readBinBasic<size_t>(file, sz);
		vec.resize(sz);
		file.read(reinterpret_cast<char*>(vec.data()), sz * sizeof(T));
	}


	template<typename T, std::enable_if_t<!is_ptr_v<T>, bool> = true, std::enable_if_t<!is_vector_v<T>, bool> = true, std::enable_if_t<!is_user_v<T>, bool> = true>
	static void readBin(std::ifstream& file, T& value)
	{
		using RT = RemoveCVRef<T>;             // 非 vector 则返回自身类型
		std::string tpName = typeid(value).name();
		//base type
		if (
			tpName == "char"
			|| tpName == "unsigned char"
			|| tpName == "short"
			|| tpName == "unsigned short"
			|| tpName == "int"
			|| tpName == "unsigned int"
			|| tpName == "long"
			|| tpName == "unsigned long"
			|| tpName == "long long"
			|| tpName == "unsigned long long"
			|| tpName == "float"
			|| tpName == "double"
			)
		{
			readBinBasic<T>(file, value);
		}
		//std::vector
		else if (is_vector_v<RT>)
		{
			readBin<RT>(file, value);
		}
		//std::string
		//if (tpName == "class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >")
		else if (tpName.find("class std::basic_string<char") == 0)
		{
			readBinString<RT>(file, value);
		}
		else if (is_ptr_v<RT>)
		{
			//using TT = target_type_t<RT>;
			readBin<RT>(file, value);
		}
		//defined by user
		else if (has_func<RT>::value == 1)
		{
			readBin<RT>(file, value);
		}
		else
		{
			std::cout << "write binary type error!" << std::endl;
		}
	}
	template<typename T, std::enable_if_t<is_ptr_v<T>, bool> = true, std::enable_if_t<!is_vector_v<T>, bool> = true, std::enable_if_t<!is_user_v<T>, bool> = true>
	static void readBin(std::ifstream& file, T& value)
	{
		using RT = target_type_t<T>;
		using T2 = RemoveCVRef<RT>;             // 非 vector 则返回自身类型
		std::string tpName = typeid(T2).name();
		//base type
		if (
			tpName == "char"
			|| tpName == "unsigned char"
			|| tpName == "short"
			|| tpName == "unsigned short"
			|| tpName == "int"
			|| tpName == "unsigned int"
			|| tpName == "long"
			|| tpName == "unsigned long"
			|| tpName == "long long"
			|| tpName == "unsigned long long"
			|| tpName == "float"
			|| tpName == "double"
			)
		{
			readBinBasic<T2>(file, *value);
		}
		//std::vector
		else if (is_vector_v<T2>)
		{
			readBin<T2>(file, *value);
		}
		//std::string
		//if (tpName == "class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >")
		else if (tpName.find("class std::basic_string<char") == 0)
		{
			readBinString<T2>(file, *value);
		}
		else if (is_ptr_v<T2>)
		{
			//using TT = target_type_t<T2>;
			readBin<T2>(file, *value);
		}
		//defined by user
		else if (has_func<T2>::value == 1)
		{
			readBin<T2>(file, *value);
		}
		else
		{
			std::cout << "write binary type error!" << std::endl;
		}
	}
	template<typename T, std::enable_if_t<!is_ptr_v<T>, bool> = true, std::enable_if_t<is_vector_v<T>, bool> = true, std::enable_if_t<!is_user_v<T>, bool> = true>
	static void readBin(std::ifstream& file, T& value)
	{
		using RT = vector_element_t<T>;
		using T2 = RemoveCVRef<RT>;             // 非 vector 则返回自身类型
		std::string tpName = typeid(T2).name();
		size_t sz = value.size();
		//base type
		if (
			tpName == "char"
			|| tpName == "unsigned char"
			|| tpName == "short"
			|| tpName == "unsigned short"
			|| tpName == "int"
			|| tpName == "unsigned int"
			|| tpName == "long"
			|| tpName == "unsigned long"
			|| tpName == "long long"
			|| tpName == "unsigned long long"
			|| tpName == "float"
			|| tpName == "double"
			)
		{
			readBinBasicVectorEvery<T2>(file, value);
		}
		//std::vector
		else if (is_vector_v<T2>)
		{
			for (int i = 0; i < sz; ++i)
				readBin<T2>(file, value[i]);
		}
		//std::string
		//if (tpName == "class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >")
		else if (tpName.find("class std::basic_string<char") == 0)
		{
			for (int i = 0; i < sz; ++i)
				readBinString<T2>(file, value[i]);
		}
		else if (is_ptr_v<T2>)
		{
			for (int i = 0; i < sz; ++i)
				readBin<T2>(file, value[i]);
		}
		//defined by user
		else if (has_func<T2>::value == 1)
		{
			for (int i = 0; i < sz; ++i)
				readBin<T2>(file, value[i]);
		}
		else
		{
			std::cout << "write binary type error!" << std::endl;
		}
	}
	template<typename T, std::enable_if_t<!is_ptr_v<T>, bool> = true, std::enable_if_t<!is_vector_v<T>, bool> = true, std::enable_if_t<is_user_v<T>, bool> = true>
	static void readBin(std::ifstream& file, T& value)
	{
		readBinUser<T>(file, value);
	}

	template<typename... Ts>
	static void readBinDatas(std::ifstream& file, Ts&... args)
	{
		//std::cout << "base" << std::endl;
	}
	template<typename T>
	static void readBinDatas(std::ifstream& file, T& _first)
	{
		//std::cout << typeid(_first).name() << std::endl;
		readBin<T>(file, _first);
	}
	template<typename T, typename... Args>
	static void readBinDatas(std::ifstream& file, T& _first, Args&... rest)
	{
		//std::cout << typeid(_first).name() << std::endl;
		readBin<T>(file, _first);
		readBinDatas<Args...>(file, rest...);
	}
	/// <summary>
	/// 指针中包含指针读取会出错，在自定义类中需要注意指针的读取和释放
	/// </summary>
	/// <typeparam name="...Args"></typeparam>
	/// <param name="_fileName"></param>
	/// <param name="...args"></param>
	/// <returns></returns>
	template<typename... Args>
	static bool readBinFile(const std::string& _fileName, Args&... args)
	{
		std::cout << "read filename: " << _fileName << std::endl;
		std::ifstream f(_fileName, std::ios::binary);
		if (!f.is_open())
		{
			std::cout << "open file failed!" << std::endl;
			return false;
		}
		readBinDatas<Args...>(f, args...);
		f.close();
		return true;
	}


}


#endif // !SLAM_LYJ_COMMON_BASEIO_H
