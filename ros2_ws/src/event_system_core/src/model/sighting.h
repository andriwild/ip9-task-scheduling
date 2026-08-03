#pragma once

#include <map>
#include <string>
#include <vector>

enum class SightingKind { PRESENT, ABSENT };
struct Sighting {
    int time;
    std::string personName;
    std::string location;
    SightingKind kind;
};

struct SightingCounts {
    int hits = 0;
    int misses = 0;
};

// Append-only log plus a running (person, room) tally. The tally makes a belief
// query O(rooms) instead of O(rooms x log), the raw entries stay for the trace export.
class SightingLog {
    std::vector<Sighting> m_entries;
    std::map<std::string, std::map<std::string, SightingCounts>> m_tally;

public:
    void add(const Sighting& sighting) {
        m_entries.push_back(sighting);
        SightingCounts& c = m_tally[sighting.personName][sighting.location];
        if (sighting.kind == SightingKind::PRESENT) {
            c.hits++;
        } else {
            c.misses++;
        }
    }

    SightingCounts counts(const std::string& person, const std::string& room) const {
        const auto p = m_tally.find(person);
        if (p == m_tally.end()) {
            return {};
        }
        const auto r = p->second.find(room);
        if (r == p->second.end()) {
            return {};
        }
        return r->second;
    }

    const std::vector<Sighting>& entries() const {
        return m_entries;
    }

    std::size_t size() const {
        return m_entries.size();
    }

    void clear() {
        m_entries.clear();
        m_tally.clear();
    }
};
