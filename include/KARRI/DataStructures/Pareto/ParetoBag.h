#pragma once
#include <concepts>
#include <vector>

// A type that can be stored in a ParetoBag: It is copy-constructible and provides a static method
// bool dominates(const T&, const T&) that decides whether the first entry dominates the second.
template<typename T>
concept ParetoBagEntry = std::copy_constructible<T> && requires(const T &a, const T &b) {
    { T::dominates(a, b) } -> std::same_as<bool>;
};

template<ParetoBagEntry T>
struct ParetoBag {
    
    std::vector<T> entries;

    auto begin() const {
        return entries.begin();
    }

    auto end() const {
        return entries.end();
    }

    void clear() {
        entries.clear();
    }

    void addCandidate(const T &candidate) {
        // Check if this entry is dominated by any of the entries in the bag.
        bool isDominated = false;
        for (const auto &existing: entries) {
            if (T::dominates(existing, candidate)) {
                isDominated = true;
                break;
            }
        }
        if (isDominated)
            return;

        // Remove all existing entries that are dominated by the new entry (without maintaining order):
        int k = 0;
        while (k < entries.size()) {
            if (T::dominates(entries[k], candidate)) {
                entries[k] = entries.back();
                entries.pop_back();
                continue;
            }
            ++k;
        }

        // Add the new entry
        entries.push_back(candidate);
    }

};
