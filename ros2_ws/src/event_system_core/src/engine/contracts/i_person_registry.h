#pragma once

#include <map>
#include <optional>
#include <string>

#include "util/types.h"

namespace des {

class IPersonRegistry {
public:
    virtual ~IPersonRegistry() = default;
    virtual bool hasEmployee(const std::string& person) const = 0;
    virtual Person* getPersonByName(const std::string& person) const = 0;
    virtual const PersonList& getAllPersons() const = 0;

    // Person location registry (single source of truth)
    virtual std::string getPersonLocation(const std::string& name) const = 0;
    virtual const std::map<std::string, std::string>& getAllPersonLocations() const = 0;
    virtual void setPersonLocation(const std::string& name, const std::string& room) = 0;
    virtual std::optional<Point> getPersonPosition(const std::string& name) const = 0;
    virtual bool robotSeesPerson(const std::string& name) const = 0;
    virtual bool robotRecognizesPerson(const std::string& name) const = 0;
};

}  // namespace des
