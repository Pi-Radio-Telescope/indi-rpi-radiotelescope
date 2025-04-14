#ifndef UTILITY_H
#define UTILITY_H

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace PiRaTe {

// helper function for determination of a value's sign
template <typename T>
constexpr int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

template <typename T, std::size_t N>
class Ringbuffer {
public:
    void add(T val);
    [[nodiscard]] auto mean() const -> T;
    [[nodiscard]] auto stddev() const -> T;
    [[nodiscard]] auto variance() const -> T;
    [[nodiscard]] auto entries() const -> std::size_t;

private:
    std::array<T, N> m_buffer { T {} };
    std::size_t m_index;
    bool m_full { false };
};

// +++++++++++++++++++++++++++++++
// implementation part starts here
// +++++++++++++++++++++++++++++++

// +++++++++++++++++++++++++++++++
// class Ringbuffer
template <typename T, std::size_t N>
void Ringbuffer<T, N>::add(T val)
{
    m_buffer[m_index++] = val;
    if (m_index >= N) {
        m_index = 0;
        m_full = true;
    }
}

template <typename T, std::size_t N>
auto Ringbuffer<T, N>::mean() const -> T
{
    T mean {};
    if (!m_full) {
        mean = std::accumulate(m_buffer.begin(), m_buffer.begin() + m_index, 0.0) / std::max<double>(m_index, 1.0);
    } else {
        mean = std::accumulate(m_buffer.begin(), m_buffer.end(), 0.0) / N;
    }
    return mean;
}

template <typename T, std::size_t N>
auto Ringbuffer<T, N>::stddev() const -> T
{
    return std::sqrt(this->variance);
}

template <typename T, std::size_t N>
auto Ringbuffer<T, N>::variance() const -> T
{
    T mean { this->mean() };
    T variance {};
    if (!m_full) {
        variance = 1.0 / (m_index - 1) * std::inner_product(
                       m_buffer.begin(), m_buffer.begin() + m_index, m_buffer.begin(), 0.0, [](T const& x, T const& y) { return x + y; }, [mean](T const& x, T const& y) { return (x - mean) * (y - mean); });
    } else {
        variance = 1.0 / (N - 1.0) * std::inner_product(
                       m_buffer.begin(), m_buffer.end(), m_buffer.begin(), 0.0, [](T const& x, T const& y) { return x + y; }, [mean](T const& x, T const& y) { return (x - mean) * (y - mean); });
    }
    return variance;
}

template <typename T, std::size_t N>
auto Ringbuffer<T, N>::entries() const -> std::size_t
{
    return ((m_full) ? N : m_index);
}
// -------------------------------

} // namespace PiRaTe

#endif // #define UTILITY_H
