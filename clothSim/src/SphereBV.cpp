///
///  @file SphereBV.cpp
///  @brief Implimentation of SphereBV struct

#include "SphereBV.h"

bool SphereBV::intersect(const SphereBV _sbv)
{
    auto dist = (_sbv.center - this->center).length();
    if(dist < (this->radius + _sbv.radius))
    {
        return true;
    }
    else
    {
        return false;
    }
}
