#ifndef TEST_H
#define TEST_H

#include <memory>
#include <aris.hpp>
#include <atomic>

using Size = std::size_t;
constexpr double PI = 3.141592653589793;
long timetmp = 0;
int add(int a,int b);

class MoveJS : public aris::plan::Plan
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit MoveJS(const std::string &name = "MoveJS_plan");
    ARIS_REGISTER_TYPE(MoveJS);
};
#endif
