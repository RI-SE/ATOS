#ifndef MAESTROBASE_HPP
#define MAESTROBASE_HPP

#include "module.hpp"

class MaestroBase : public Module
{
public:
    MaestroBase();
    ~MaestroBase();
    void initialize();
private:
    static inline std::string const moduleName = "maestro_base";

    void onExitMessage(const Empty::SharedPtr) override;
    // Module only provides plumbing, no need to handle abort
    void onAbortMessage(const Empty::SharedPtr) override {};
};


#endif