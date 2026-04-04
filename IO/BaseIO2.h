#ifndef BASE_IO2_H
#define BASE_IO2_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <list>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace baseio2
{
class BinaryError : public std::runtime_error
{
public:
    explicit BinaryError(const std::string& message)
        : std::runtime_error(message)
    {
    }
};

class BinaryWriter;
class BinaryReader;

namespace detail
{
template <typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

template <typename T>
struct dependent_false : std::false_type
{
};

template <typename T>
struct is_string : std::false_type
{
};

template <>
struct is_string<std::string> : std::true_type
{
};

template <typename T>
struct is_std_array : std::false_type
{
};

template <typename T, std::size_t N>
struct is_std_array<std::array<T, N>> : std::true_type
{
};

template <typename T>
struct is_optional : std::false_type
{
};

template <typename T>
struct is_optional<std::optional<T>> : std::true_type
{
};

template <typename T>
struct is_pair : std::false_type
{
};

template <typename T1, typename T2>
struct is_pair<std::pair<T1, T2>> : std::true_type
{
};

template <typename T, typename = void>
struct is_tuple_like : std::false_type
{
};

template <typename T>
struct is_tuple_like<T, std::void_t<decltype(std::tuple_size<T>::value)>> : std::true_type
{
};

template <typename T, typename = void>
struct is_container : std::false_type
{
};

template <typename T>
struct is_container<
    T,
    std::void_t<
        typename T::value_type,
        decltype(std::declval<T&>().begin()),
        decltype(std::declval<T&>().end()),
        decltype(std::declval<const T&>().size())>> : std::true_type
{
};

template <typename T, typename = void>
struct is_map_like : std::false_type
{
};

template <typename T>
struct is_map_like<T, std::void_t<typename T::key_type, typename T::mapped_type>> : std::true_type
{
};

template <typename T, typename = void>
struct has_reserve : std::false_type
{
};

template <typename T>
struct has_reserve<T, std::void_t<decltype(std::declval<T&>().reserve(std::declval<typename T::size_type>()))>>
    : std::true_type
{
};

template <typename T, typename V, typename = void>
struct has_push_back : std::false_type
{
};

template <typename T, typename V>
struct has_push_back<T, V, std::void_t<decltype(std::declval<T&>().push_back(std::declval<V>()))>> : std::true_type
{
};

template <typename T, typename V, typename = void>
struct has_insert_value : std::false_type
{
};

template <typename T, typename V>
struct has_insert_value<T, V, std::void_t<decltype(std::declval<T&>().insert(std::declval<V>()))>> : std::true_type
{
};

template <typename Writer, typename T, typename = void>
struct has_member_write_binary : std::false_type
{
};

template <typename Writer, typename T>
struct has_member_write_binary<
    Writer,
    T,
    std::void_t<decltype(std::declval<const T&>().write_binary(std::declval<Writer&>()))>> : std::true_type
{
};

template <typename Reader, typename T, typename = void>
struct has_member_read_binary : std::false_type
{
};

template <typename Reader, typename T>
struct has_member_read_binary<
    Reader,
    T,
    std::void_t<decltype(std::declval<T&>().read_binary(std::declval<Reader&>()))>> : std::true_type
{
};

template <typename T, typename = void>
struct has_member_write_binary_stream : std::false_type
{
};

template <typename T>
struct has_member_write_binary_stream<
    T,
    std::void_t<decltype(std::declval<const T&>().write_binary(std::declval<std::ofstream&>()))>> : std::true_type
{
};

template <typename T, typename = void>
struct has_member_read_binary_stream : std::false_type
{
};

template <typename T>
struct has_member_read_binary_stream<
    T,
    std::void_t<decltype(std::declval<T&>().read_binary(std::declval<std::ifstream&>()))>> : std::true_type
{
};

template <typename Writer, typename T, typename = void>
struct has_member_serialize : std::false_type
{
};

template <typename Writer, typename T>
struct has_member_serialize<
    Writer,
    T,
    std::void_t<decltype(std::declval<const T&>().serialize(std::declval<Writer&>()))>> : std::true_type
{
};

template <typename Reader, typename T, typename = void>
struct has_member_deserialize : std::false_type
{
};

template <typename Reader, typename T>
struct has_member_deserialize<
    Reader,
    T,
    std::void_t<decltype(std::declval<T&>().deserialize(std::declval<Reader&>()))>> : std::true_type
{
};

template <typename Writer, typename T, typename = void>
struct has_adl_serialize : std::false_type
{
};

template <typename Writer, typename T>
struct has_adl_serialize<
    Writer,
    T,
    std::void_t<decltype(serialize(std::declval<Writer&>(), std::declval<const T&>()))>> : std::true_type
{
};

template <typename Reader, typename T, typename = void>
struct has_adl_deserialize : std::false_type
{
};

template <typename Reader, typename T>
struct has_adl_deserialize<
    Reader,
    T,
    std::void_t<decltype(deserialize(std::declval<Reader&>(), std::declval<T&>()))>> : std::true_type
{
};

template <typename T>
constexpr bool is_cross_platform_float_v =
    std::is_same_v<remove_cvref_t<T>, float> || std::is_same_v<remove_cvref_t<T>, double>;

template <typename T>
constexpr bool is_supported_scalar_v =
    std::is_same_v<remove_cvref_t<T>, bool> ||
    std::is_enum_v<remove_cvref_t<T>> ||
    (std::is_integral_v<remove_cvref_t<T>> && !std::is_same_v<remove_cvref_t<T>, bool>) ||
    is_cross_platform_float_v<T>;

template <typename T>
constexpr bool is_generic_container_v =
    is_container<remove_cvref_t<T>>::value &&
    !is_string<remove_cvref_t<T>>::value &&
    !is_std_array<remove_cvref_t<T>>::value &&
    !is_map_like<remove_cvref_t<T>>::value;

template <typename T>
void append_container(T& container, typename T::value_type&& value)
{
    if constexpr (has_push_back<T, typename T::value_type&&>::value)
    {
        container.push_back(std::move(value));
    }
    else if constexpr (has_insert_value<T, typename T::value_type&&>::value)
    {
        container.insert(std::move(value));
    }
    else
    {
        static_assert(dependent_false<T>::value, "Container type is not supported for deserialization");
    }
}

template <typename UInt>
void write_unsigned_little_endian(BinaryWriter& writer, UInt value);

template <typename UInt>
UInt read_unsigned_little_endian(BinaryReader& reader);

void write_size(BinaryWriter& writer, std::size_t size);
std::size_t read_size(BinaryReader& reader);

template <typename Tuple, std::size_t... I>
void write_tuple(BinaryWriter& writer, const Tuple& value, std::index_sequence<I...>);

template <typename Tuple, std::size_t... I>
void read_tuple(BinaryReader& reader, Tuple& value, std::index_sequence<I...>);

template <typename T>
void write_value(BinaryWriter& writer, const T& value);

template <typename T>
void read_value(BinaryReader& reader, T& value);
}  // namespace detail

class BinaryWriter
{
public:
    explicit BinaryWriter(std::ostream& output)
        : output_(output)
    {
    }

    void write_bytes(const void* data, std::size_t size)
    {
        output_.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
        if (!output_)
        {
            throw BinaryError("write_bytes failed");
        }
    }

    template <typename T>
    void write(const T& value)
    {
        detail::write_value(*this, value);
    }

    template <typename... Ts>
    void write_many(const Ts&... values)
    {
        (write(values), ...);
    }

    std::ostream& stream()
    {
        return output_;
    }

    const std::ostream& stream() const
    {
        return output_;
    }

private:
    std::ostream& output_;
};

class BinaryReader
{
public:
    explicit BinaryReader(std::istream& input)
        : input_(input)
    {
    }

    void read_bytes(void* data, std::size_t size)
    {
        input_.read(static_cast<char*>(data), static_cast<std::streamsize>(size));
        if (!input_)
        {
            throw BinaryError("read_bytes failed");
        }
    }

    template <typename T>
    void read(T& value)
    {
        detail::read_value(*this, value);
    }

    template <typename... Ts>
    void read_many(Ts&... values)
    {
        (read(values), ...);
    }

    std::istream& stream()
    {
        return input_;
    }

    const std::istream& stream() const
    {
        return input_;
    }

private:
    std::istream& input_;
};

namespace detail
{
template <typename UInt>
void write_unsigned_little_endian(BinaryWriter& writer, UInt value)
{
    static_assert(std::is_unsigned_v<UInt>, "UInt must be unsigned");
    unsigned char bytes[sizeof(UInt)];
    for (std::size_t i = 0; i < sizeof(UInt); ++i)
    {
        bytes[i] = static_cast<unsigned char>((value >> (i * 8U)) & static_cast<UInt>(0xFFU));
    }
    writer.write_bytes(bytes, sizeof(bytes));
}

template <typename UInt>
UInt read_unsigned_little_endian(BinaryReader& reader)
{
    static_assert(std::is_unsigned_v<UInt>, "UInt must be unsigned");
    unsigned char bytes[sizeof(UInt)] = {};
    reader.read_bytes(bytes, sizeof(bytes));

    UInt value = 0;
    for (std::size_t i = 0; i < sizeof(UInt); ++i)
    {
        value |= (static_cast<UInt>(bytes[i]) << (i * 8U));
    }
    return value;
}

inline void write_size(BinaryWriter& writer, std::size_t size)
{
    write_unsigned_little_endian<std::uint64_t>(writer, static_cast<std::uint64_t>(size));
}

inline std::size_t read_size(BinaryReader& reader)
{
    const std::uint64_t raw = read_unsigned_little_endian<std::uint64_t>(reader);
    if (raw > static_cast<std::uint64_t>(SIZE_MAX))
    {
        throw BinaryError("size value is too large for this platform");
    }
    return static_cast<std::size_t>(raw);
}

template <typename T>
void write_integral(BinaryWriter& writer, const T& value)
{
    using raw_t = remove_cvref_t<T>;
    using unsigned_t = std::make_unsigned_t<raw_t>;
    unsigned_t bits = 0;
    std::memcpy(&bits, &value, sizeof(raw_t));
    write_unsigned_little_endian<unsigned_t>(writer, bits);
}

template <typename T>
void read_integral(BinaryReader& reader, T& value)
{
    using raw_t = remove_cvref_t<T>;
    using unsigned_t = std::make_unsigned_t<raw_t>;
    const unsigned_t bits = read_unsigned_little_endian<unsigned_t>(reader);
    std::memcpy(&value, &bits, sizeof(raw_t));
}

template <typename T>
void write_float(BinaryWriter& writer, const T& value)
{
    static_assert(std::numeric_limits<T>::is_iec559, "Only IEEE 754 float/double are supported");
    if constexpr (sizeof(T) == sizeof(std::uint32_t))
    {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(T));
        write_unsigned_little_endian<std::uint32_t>(writer, bits);
    }
    else if constexpr (sizeof(T) == sizeof(std::uint64_t))
    {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(T));
        write_unsigned_little_endian<std::uint64_t>(writer, bits);
    }
    else
    {
        static_assert(dependent_false<T>::value, "Unsupported floating-point size");
    }
}

template <typename T>
void read_float(BinaryReader& reader, T& value)
{
    static_assert(std::numeric_limits<T>::is_iec559, "Only IEEE 754 float/double are supported");
    if constexpr (sizeof(T) == sizeof(std::uint32_t))
    {
        const std::uint32_t bits = read_unsigned_little_endian<std::uint32_t>(reader);
        std::memcpy(&value, &bits, sizeof(T));
    }
    else if constexpr (sizeof(T) == sizeof(std::uint64_t))
    {
        const std::uint64_t bits = read_unsigned_little_endian<std::uint64_t>(reader);
        std::memcpy(&value, &bits, sizeof(T));
    }
    else
    {
        static_assert(dependent_false<T>::value, "Unsupported floating-point size");
    }
}

template <typename Tuple, std::size_t... I>
void write_tuple(BinaryWriter& writer, const Tuple& value, std::index_sequence<I...>)
{
    writer.write_many(std::get<I>(value)...);
}

template <typename Tuple, std::size_t... I>
void read_tuple(BinaryReader& reader, Tuple& value, std::index_sequence<I...>)
{
    reader.read_many(std::get<I>(value)...);
}

template <typename T>
void write_value(BinaryWriter& writer, const T& value)
{
    using raw_t = remove_cvref_t<T>;

    if constexpr (std::is_same_v<raw_t, bool>)
    {
        const std::uint8_t flag = value ? 1U : 0U;
        write_unsigned_little_endian<std::uint8_t>(writer, flag);
    }
    else if constexpr (std::is_enum_v<raw_t>)
    {
        using underlying_t = std::underlying_type_t<raw_t>;
        write_integral(writer, static_cast<underlying_t>(value));
    }
    else if constexpr (std::is_integral_v<raw_t>)
    {
        write_integral(writer, value);
    }
    else if constexpr (is_cross_platform_float_v<raw_t>)
    {
        write_float(writer, value);
    }
    else if constexpr (is_string<raw_t>::value)
    {
        write_size(writer, value.size());
        if (!value.empty())
        {
            writer.write_bytes(value.data(), value.size());
        }
    }
    else if constexpr (is_optional<raw_t>::value)
    {
        writer.write(value.has_value());
        if (value.has_value())
        {
            writer.write(*value);
        }
    }
    else if constexpr (is_std_array<raw_t>::value)
    {
        for (const auto& item : value)
        {
            writer.write(item);
        }
    }
    else if constexpr (is_pair<raw_t>::value)
    {
        writer.write(value.first);
        writer.write(value.second);
    }
    else if constexpr (is_tuple_like<raw_t>::value && !is_pair<raw_t>::value && !is_std_array<raw_t>::value)
    {
        write_tuple(writer, value, std::make_index_sequence<std::tuple_size<raw_t>::value>{});
    }
    else if constexpr (is_map_like<raw_t>::value)
    {
        write_size(writer, value.size());
        for (const auto& item : value)
        {
            writer.write(item.first);
            writer.write(item.second);
        }
    }
    else if constexpr (is_generic_container_v<raw_t>)
    {
        write_size(writer, value.size());
        for (const auto& item : value)
        {
            writer.write(item);
        }
    }
    else if constexpr (has_member_write_binary<BinaryWriter, raw_t>::value)
    {
        value.write_binary(writer);
    }
    else if constexpr (has_member_write_binary_stream<raw_t>::value)
    {
        auto* file = dynamic_cast<std::ofstream*>(&writer.stream());
        if (file == nullptr)
        {
            throw BinaryError("write_binary(std::ofstream&) requires BinaryWriter backed by std::ofstream");
        }
        value.write_binary(*file);
    }
    else if constexpr (has_member_serialize<BinaryWriter, raw_t>::value)
    {
        value.serialize(writer);
    }
    else if constexpr (has_adl_serialize<BinaryWriter, raw_t>::value)
    {
        serialize(writer, value);
    }
    else
    {
        static_assert(dependent_false<raw_t>::value, "Unsupported type for binary serialization");
    }
}

template <typename T>
void read_value(BinaryReader& reader, T& value)
{
    using raw_t = remove_cvref_t<T>;

    if constexpr (std::is_same_v<raw_t, bool>)
    {
        const std::uint8_t flag = read_unsigned_little_endian<std::uint8_t>(reader);
        if (flag > 1U)
        {
            throw BinaryError("Invalid boolean value");
        }
        value = (flag != 0U);
    }
    else if constexpr (std::is_enum_v<raw_t>)
    {
        using underlying_t = std::underlying_type_t<raw_t>;
        underlying_t temp {};
        read_integral(reader, temp);
        value = static_cast<raw_t>(temp);
    }
    else if constexpr (std::is_integral_v<raw_t>)
    {
        read_integral(reader, value);
    }
    else if constexpr (is_cross_platform_float_v<raw_t>)
    {
        read_float(reader, value);
    }
    else if constexpr (is_string<raw_t>::value)
    {
        const std::size_t count = read_size(reader);
        value.resize(count);
        if (count != 0U)
        {
            reader.read_bytes(value.data(), count);
        }
    }
    else if constexpr (is_optional<raw_t>::value)
    {
        bool has_value = false;
        reader.read(has_value);
        if (has_value)
        {
            typename raw_t::value_type temp {};
            reader.read(temp);
            value = std::move(temp);
        }
        else
        {
            value.reset();
        }
    }
    else if constexpr (is_std_array<raw_t>::value)
    {
        for (auto& item : value)
        {
            reader.read(item);
        }
    }
    else if constexpr (is_pair<raw_t>::value)
    {
        reader.read(value.first);
        reader.read(value.second);
    }
    else if constexpr (is_tuple_like<raw_t>::value && !is_pair<raw_t>::value && !is_std_array<raw_t>::value)
    {
        read_tuple(reader, value, std::make_index_sequence<std::tuple_size<raw_t>::value>{});
    }
    else if constexpr (is_map_like<raw_t>::value)
    {
        const std::size_t count = read_size(reader);
        value.clear();
        if constexpr (has_reserve<raw_t>::value)
        {
            value.reserve(count);
        }
        for (std::size_t i = 0; i < count; ++i)
        {
            typename raw_t::key_type key {};
            typename raw_t::mapped_type mapped {};
            reader.read(key);
            reader.read(mapped);
            value.emplace(std::move(key), std::move(mapped));
        }
    }
    else if constexpr (is_generic_container_v<raw_t>)
    {
        const std::size_t count = read_size(reader);
        value.clear();
        if constexpr (has_reserve<raw_t>::value)
        {
            value.reserve(count);
        }
        for (std::size_t i = 0; i < count; ++i)
        {
            typename raw_t::value_type item {};
            reader.read(item);
            append_container(value, std::move(item));
        }
    }
    else if constexpr (has_member_read_binary<BinaryReader, raw_t>::value)
    {
        value.read_binary(reader);
    }
    else if constexpr (has_member_read_binary_stream<raw_t>::value)
    {
        auto* file = dynamic_cast<std::ifstream*>(&reader.stream());
        if (file == nullptr)
        {
            throw BinaryError("read_binary(std::ifstream&) requires BinaryReader backed by std::ifstream");
        }
        value.read_binary(*file);
    }
    else if constexpr (has_member_deserialize<BinaryReader, raw_t>::value)
    {
        value.deserialize(reader);
    }
    else if constexpr (has_adl_deserialize<BinaryReader, raw_t>::value)
    {
        deserialize(reader, value);
    }
    else
    {
        static_assert(dependent_false<raw_t>::value, "Unsupported type for binary deserialization");
    }
}
}  // namespace detail

template <typename... Ts>
void write(BinaryWriter& writer, const Ts&... values)
{
    writer.write_many(values...);
}

template <typename... Ts>
void read(BinaryReader& reader, Ts&... values)
{
    reader.read_many(values...);
}

template <typename... Ts>
void save_to_file(const std::string& file_name, const Ts&... values)
{
    std::ofstream output(file_name, std::ios::binary);
    if (!output.is_open())
    {
        throw BinaryError("Failed to open file for writing: " + file_name);
    }

    BinaryWriter writer(output);
    writer.write_many(values...);
}

template <typename... Ts>
void load_from_file(const std::string& file_name, Ts&... values)
{
    std::ifstream input(file_name, std::ios::binary);
    if (!input.is_open())
    {
        throw BinaryError("Failed to open file for reading: " + file_name);
    }

    BinaryReader reader(input);
    reader.read_many(values...);
}

}  // namespace baseio2

namespace COMMON_LYJ2
{
using BinaryError = baseio2::BinaryError;
using BinaryReader = baseio2::BinaryReader;
using BinaryWriter = baseio2::BinaryWriter;

template <typename T>
void writeBinDirect(std::ofstream& file, const T* value, const std::uint64_t count)
{
    baseio2::BinaryWriter writer(file);
    for (std::uint64_t i = 0; i < count; ++i)
    {
        writer.write(value[i]);
    }
}

template <typename T>
void writeBinBasic(std::ofstream& file, const T& value)
{
    baseio2::BinaryWriter writer(file);
    writer.write(value);
}

template <typename T>
void writeBinString(std::ofstream& file, const T& value)
{
    baseio2::BinaryWriter writer(file);
    writer.write(value);
}

template <typename T>
void writeBinUser(std::ofstream& file, const T& value)
{
    if constexpr (baseio2::detail::has_member_write_binary_stream<T>::value)
    {
        value.write_binary(file);
    }
    else
    {
        baseio2::BinaryWriter writer(file);
        writer.write(value);
    }
}

template <typename T>
void writeBinBasicVectorEvery(std::ofstream& file, const std::vector<T>& vec)
{
    baseio2::BinaryWriter writer(file);
    writer.write(vec);
}

template <typename T>
void writeBin(std::ofstream& file, const T& value)
{
    baseio2::BinaryWriter writer(file);
    writer.write(value);
}

template <typename... Args>
void writeBinDatas(std::ofstream& file, const Args&... args)
{
    baseio2::BinaryWriter writer(file);
    writer.write_many(args...);
}

template <typename... Args>
bool writeBinFile(const std::string& file_name, const Args&... args)
{
    try
    {
        std::ofstream file(file_name, std::ios::binary);
        if (!file.is_open())
        {
            return false;
        }
        writeBinDatas(file, args...);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

template <typename T>
void readBinDirect(std::ifstream& file, T* value, const std::uint64_t count)
{
    baseio2::BinaryReader reader(file);
    for (std::uint64_t i = 0; i < count; ++i)
    {
        reader.read(value[i]);
    }
}

template <typename T>
void readBinBasic(std::ifstream& file, T& value)
{
    baseio2::BinaryReader reader(file);
    reader.read(value);
}

template <typename T>
void readBinString(std::ifstream& file, T& value)
{
    baseio2::BinaryReader reader(file);
    reader.read(value);
}

template <typename T>
void readBinUser(std::ifstream& file, T& value)
{
    if constexpr (baseio2::detail::has_member_read_binary_stream<T>::value)
    {
        value.read_binary(file);
    }
    else
    {
        baseio2::BinaryReader reader(file);
        reader.read(value);
    }
}

template <typename T>
void readBinBasicVectorEvery(std::ifstream& file, std::vector<T>& vec)
{
    baseio2::BinaryReader reader(file);
    reader.read(vec);
}

template <typename T>
void readBin(std::ifstream& file, T& value)
{
    baseio2::BinaryReader reader(file);
    reader.read(value);
}

template <typename... Args>
void readBinDatas(std::ifstream& file, Args&... args)
{
    baseio2::BinaryReader reader(file);
    reader.read_many(args...);
}

template <typename... Args>
bool readBinFile(const std::string& file_name, Args&... args)
{
    try
    {
        std::ifstream file(file_name, std::ios::binary);
        if (!file.is_open())
        {
            return false;
        }
        readBinDatas(file, args...);
        return true;
    }
    catch (...)
    {
        return false;
    }
}
}  // namespace COMMON_LYJ

#endif  // BASE_IO2_H
