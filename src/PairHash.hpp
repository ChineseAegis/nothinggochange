#pragma once
#include <cstddef> // For std::size_t
#include <functional> // For std::hash

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ (hash2 << 16 | hash2 >> 16);
    }
};

