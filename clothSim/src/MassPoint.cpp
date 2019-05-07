///
///  @file MassPoint.cpp
///  @brief Implimentation of MassPoint class

#include <ngl/Vec3.h>
#include <ngl/Mat3.h>
#include <numeric>
#include "MassPoint.h"

void MassPoint::fixPoint()
{
    m_fixed = true;
    m_vel = ngl::Vec3(0.0f);
}
