///
///  @file SphereObj.cpp
///  @brief Implimentation of SphereObj class

#include <ngl/Types.h>
#include "SphereObj.h"

constexpr float TWO_PI= static_cast<float>(6.28318530717958647692);   //360
constexpr float PI2= static_cast<float>(1.57079632679489661923);      //90

SphereObj::SphereObj(float _radius, size_t _precision) : m_radius(_radius)
{
    /* The following code was written by Jon Macey.
       I stole this from VAOPrimitives. Would have just used it as-is,
       but I needed to be able to grab the vertex data myself.
       I'll note lines I modified to fit my own code.
       Universal changes: made all reals into floats and ints into size_ts. */

    //  Sphere code based on a function Written by Paul Bourke.
    //  http://astronomy.swin.edu.au/~pbourke/opengl/sphere/
    // the next part of the code calculates the P,N,UV of the sphere for tri_strips

    float theta1 = 0.0f;
    float theta2 = 0.0f;
    float theta3 = 0.0f;

    //CHANGE: vertex data is now a member variable

        // Disallow a negative number for radius.

  if( _radius < 0.0f )
    {
        _radius = -_radius;
    }
    // Disallow a negative number for _precision.
    if( _precision < 4 )
    {
        _precision = 4;
    }
  // now fill in a vertData structure and add to the data list for our sphere
  // CHANGE: Just needed the vertex data, so using a Vec3 instead of vertData
  ngl::Vec3 d;
  for( size_t i = 0; i < _precision/2; ++i )
  {
    theta1 = i * TWO_PI / _precision - PI2;
    theta2 = (i + 1) * TWO_PI / _precision - PI2;

    for( size_t j = 0; j <= _precision; ++j )
    {
        theta3 = j * TWO_PI / _precision;

        d.m_x = _radius * cosf(theta2) * cosf(theta3);
        d.m_y = _radius * sinf(theta2);
        d.m_z = _radius * cosf(theta2) * sinf(theta3);

        m_vertexData.push_back(d);

        d.m_x = _radius * cosf(theta1) * cosf(theta3);
        d.m_y = _radius * sinf(theta1);
        d.m_z = _radius * cosf(theta1) * sinf(theta3);

        m_vertexData.push_back(d);
      } // end inner loop
  }// end outer loop

  // Go through and clean the data - near-zero values mess up my code
  for(auto& v : m_vertexData)
  {
      if(FCompare(v.m_x, 0.0f))
      {
          v.m_x = 0.0f;
      }
      if(FCompare(v.m_y, 0.0f))
      {
          v.m_y = 0.0f;
      }
      if(FCompare(v.m_z, 0.0f))
      {
          v.m_z = 0.0f;
      }
  }
}

void SphereObj::init()
{
    if(m_collision)
    {
        std::vector<ngl::Vec3> tridata;
        this->exportTriangles(tridata);
        m_tree.init(tridata);
    }
}

void SphereObj::exportTriangles(std::vector<ngl::Vec3> &o_vertexData) const
{
    o_vertexData.reserve(m_vertexData.size()*3 - 2);
    // Transform from triangle strip into individual triangles
    for(size_t i = 2; i < (m_vertexData.size()); ++i)
    {
        o_vertexData.push_back(m_vertexData[i]);
        o_vertexData.push_back(m_vertexData[i-1]);
        o_vertexData.push_back(m_vertexData[i-2]);
    }
}

std::vector<ngl::Vec3> SphereObj::vertFromTriNum(const size_t _triNum) const
{
    std::vector<ngl::Vec3> vertexData;
    std::vector<ngl::Vec3> exportVertices;
    exportVertices.reserve(3);
    this->exportTriangles(vertexData);
    exportVertices.push_back(vertexData[_triNum * 3]);
    exportVertices.push_back(vertexData[(_triNum * 3) + 1]);
    exportVertices.push_back(vertexData[(_triNum * 3) + 2]);
    return exportVertices;
}

void SphereObj::moveSphere(const ngl::Vec3 _v)
{
    for(auto &vd : m_vertexData)
    {
        vd += _v;
    }
    m_center += _v;
}
