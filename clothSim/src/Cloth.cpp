///
///  @file Cloth.cpp
///  @brief Implimentation of Cloth class

#include <vector>
#include <iostream>
#include <math.h>
#include "Cloth.h"
#include "MassPoint.h"

void Cloth::init(size_t _height, size_t _width, float _restl, float _ycoord, bool vertical)
{
    // in case this is a re-init, clear out all previous member variables
    m_mspts.clear();
    m_springs.clear();
    m_bvtree.clear();
    // update height and width
    m_height = _height;
    m_width = _width;
    // determine number of mass points and springs
    size_t numMasses = _height * _width;
    size_t numSprings = (_width*(_height-1)) + (_height*(_width-1)) + ((_height-1)*(_width-1));
    // reserve data
    m_mspts.reserve(numMasses);
    m_springs.reserve(numSprings);
    // add in mass points - set position, zero velocity (auto), init forces
    for(size_t i = 0; i < numMasses; ++i)
    {
        if(vertical)
        {
            float xcoord = (i % _width) * _restl;
            float ycoord = _ycoord - ((i / _width) * _restl);
            m_mspts.push_back(MassPoint(ngl::Vec3(xcoord, ycoord, 0.0f)));
        }
        else
        {
            float xcoord = (i % _width) * _restl;
            float zcoord = (i / _width) * _restl;
            m_mspts.push_back(MassPoint(ngl::Vec3(xcoord, _ycoord, zcoord)));
        }
    }
    // for each mass point, create and assign springs
    for(size_t i = 0; i < numMasses; ++i)
    {
        // cardinal directions
        if(i >= _width) //north
        {
            createSpring(i, i - _width, _restl);
        }
        if((i + _width) < numMasses) //south
        {
            createSpring(i, i + _width, _restl);
        }
        if((i % _width) != (_width - 1)) //east
        {
            createSpring(i, i + 1, _restl);
        }
        if((i % _width) != 0) //west
        {
            createSpring(i, i - 1, _restl);
        }
        // diagonals
        if((i + 1) >= _width && ((i % _width) != (_width - 1))) //northeast
        {
            createSpring(i, i - _width + 1, sqrt(_restl * _restl * 2));
        }
        if(((i + _width - 1) < numMasses) && ((i % _width) != 0)) //southwest
        {
            createSpring(i, i + _width - 1, sqrt(_restl * _restl * 2));
        }
    }
    // Set up BVTree for collision detection
    if(m_collision)
    {
        std::vector<ngl::Vec3> tridata;
        this->exportTriangles(tridata);
        m_bvtree.init(tridata);
    }
}

ngl::Vec3 Cloth::center() const
{
    //Grab "center" point position - favor lower right
    size_t centerpt = (m_width * (m_height/2)) + (m_width/2);
    //Approximate center based on where the middle points are - depends on dimensions
    if((m_height % 2 != 0) && (m_width % 2 != 0))
    {
        return m_mspts[centerpt].pos();
    }
    else if ((m_height % 2 == 0) && (m_width % 2 == 0))
    {
        size_t otherpt = centerpt - m_width - 1;
        return ((m_mspts[centerpt].pos() - m_mspts[otherpt].pos()) * 0.5) + m_mspts[otherpt].pos();
    }
    else if (m_height % 2 != 0)
    {
        size_t otherpt = centerpt - 1;
        return ((m_mspts[centerpt].pos() - m_mspts[otherpt].pos()) * 0.5) + m_mspts[otherpt].pos();
    }
    else
    {
        size_t otherpt = centerpt - m_width;
        return ((m_mspts[centerpt].pos() - m_mspts[otherpt].pos()) * 0.5) + m_mspts[otherpt].pos();
    }
}

std::vector<ngl::Vec3> Cloth::vertFromTriNum(const size_t _triNum) const
{
    std::vector<ngl::Vec3> vertices;
    vertices.reserve(3);
    // Acquire starting id using _triNum
    size_t row = _triNum/((m_width - 1)*2);
    size_t col = _triNum%((m_width - 1)*2);
    size_t id = ((m_width * row )+ (col/2)) + m_width;
    // Grab vertices
    if(_triNum % 2 == 0)
    {
        vertices.push_back(m_mspts[id].pos());
        vertices.push_back(m_mspts[id - m_width].pos());
        vertices.push_back(m_mspts[id - m_width + 1].pos());
    }
    else
    {
        vertices.push_back(m_mspts[id].pos());
        vertices.push_back(m_mspts[id - m_width + 1].pos());
        vertices.push_back(m_mspts[id + 1].pos());
    }
    return vertices;
}

void Cloth::fixptCorners()
{
    m_mspts[0].fixPoint();
    m_mspts[m_width - 1].fixPoint();
    m_mspts[m_width * m_height - m_width].fixPoint();
    m_mspts[m_width * m_height - 1].fixPoint();
}

void Cloth::fixptHang()
{
    m_mspts[0].fixPoint();
    m_mspts[m_width - 1].fixPoint();
}

void Cloth::fixptFlag()
{
    m_mspts[m_width - 1].fixPoint();
    m_mspts[m_width * m_height - 1].fixPoint();
}

void Cloth::reposToOrigin(float _ycoord)
{
    ngl::Vec3 cntr = this->center();
    for(auto& m : m_mspts)
    {
        m.setPos(m.pos() - cntr);
        m.setPos(m.pos() + ngl::Vec3(0.0f, _ycoord, 0.0f));
    }
}

void Cloth::update(const float _h)
{
    // STEP 1 - Compute J matrix values for all springs and masses
    updateSpringJacobians();
    updateMassJacobians();

    // STEP 2 - Compute the inverse of Wii for each mass point
    std::vector<ngl::Mat3> wiiMatrices;
    calcWinverse(wiiMatrices, _h);

    // STEP 3 - Calculate force vector for each mass point, mult by Winverse
    std::vector<ngl::Vec3> pforces;
    ngl::Vec3 fgravity = ngl::Vec3(0.0f, -9.8f, 0.0f) * m_mass;
    fgravity.normalize();
    calcPforces(pforces, fgravity, _h);
    for(size_t i = 0; i < m_mspts.size(); ++i)
    {
        pforces[i] = wiiMatrices[i] * pforces[i];
    }

    // STEP 4 - Calculate new velocity and position for each mass point
    for(size_t i = 0; i < m_mspts.size(); ++i)
    {
        if(m_mspts[i].fixed() == false)
        {
            // Sum through springs for iteration
            ngl::Vec3 iter(0.0f);
            for(auto s : m_mspts[i].mySprings())
            {
                iter += m_springs.at(s).jacobian * pforces[s.val1];
            }
            // Compute velocity
            m_mspts[i].setVel(m_mspts[i].vel() + (pforces[i] + (_h*_h*(wiiMatrices[i] * iter))));
            // Compute position
            m_mspts[i].setPos(m_mspts[i].pos() + (_h * m_mspts[i].vel()));
        }
    }
    // STEP 5 - Collision Detection Prep
    if(m_collision)
    {
        // Update all surfaces
        std::vector<ngl::Vec3> tridata;
        this->exportTriangles(tridata);
        m_bvtree.updateSurfaces(tridata);
        // Update all SphereBV centers/radii
        m_bvtree.updateAllBV();
    }
}

bool Cloth::fullClothFixed() const
{
    for(auto m : m_mspts)
    {
        if(m.fixed() == false)
        {
            return false;
        }
    }
    return true;
}

void Cloth::exportTriangles(std::vector<ngl::Vec3> &o_vertexData)
{
    o_vertexData.reserve(3 * (m_height - 1) * (2 * (m_width - 1)));
    //pack data in as triangles, do it by row
    for(size_t i = 0; i < (m_height - 1); ++i)
    {
        for(size_t j = 0; j < (m_width -1); ++j)
        {
            // Acquire current id number
            size_t id = j + (i * m_width);
            id += m_width;
            // triangle 1
            o_vertexData.push_back(m_mspts[id].pos());
            o_vertexData.push_back(m_mspts[id - m_width].pos());
            o_vertexData.push_back(m_mspts[id - m_width + 1].pos());
            // triangle 2
            o_vertexData.push_back(m_mspts[id].pos());
            o_vertexData.push_back(m_mspts[id - m_width + 1].pos());
            o_vertexData.push_back(m_mspts[id + 1].pos());
        }
    }
}

void Cloth::modVertFromTriNum(const size_t _triNum, std::vector<ngl::Vec3> _v)
{
    // Get old vertices
    auto oldVert = this->vertFromTriNum(_triNum);
    // Acquire starting id using _triNum
    size_t row = _triNum/((m_width - 1)*2);
    size_t col = _triNum%((m_width - 1)*2);
    size_t id = ((m_width * row )+ (col/2)) + m_width;
    // Set vertices
    if(_triNum % 2 == 0)
    {
        m_mspts[id].setPos(_v[0]);
        m_mspts[id - m_width].setPos(_v[1]);
        m_mspts[id - m_width + 1].setPos(_v[2]);
        // If new vert is different from old vert, fix the point
        if(_v[0] != oldVert[0])
        {
            m_mspts[id].fixPoint();
        }
        if(_v[1] != oldVert[1])
        {
            m_mspts[id - m_width].fixPoint();
        }
        if(_v[2] != oldVert[2])
        {
            m_mspts[id - m_width + 1].fixPoint();
        }
    }
    else
    {
        m_mspts[id].setPos(_v[0]);
        m_mspts[id - m_width + 1].setPos(_v[1]);
        m_mspts[id + 1].setPos(_v[2]);
        // If new vert is different from old vert, fix the point
        if(_v[0] != oldVert[0])
        {
            m_mspts[id].fixPoint();
        }
        if(_v[1] != oldVert[1])
        {
            m_mspts[id - m_width + 1].fixPoint();
        }
        if(_v[2] != oldVert[2])
        {
            m_mspts[id + 1].fixPoint();
        }
    }
}

void Cloth::createSpring(size_t _ind0, size_t _ind1, float _restl)
{
    UnorderedPair up(_ind0, _ind1);
    Spring s(_restl, ngl::Mat3(0.0f));
    m_mspts[_ind0].addSpring(up);
    m_springs.insert({up, s});
}

void Cloth::updateSpringJacobians()
{
    for(auto& s : m_springs)
    {
        ngl::Vec3 dist = m_mspts[s.first.val1].pos() -
                         m_mspts[s.first.val0].pos();
        // Set the Jacobian values - dist * dist(transpose)
        s.second.jacobian.m_00 = dist[0]*dist[0];
        s.second.jacobian.m_01 = dist[0]*dist[1];
        s.second.jacobian.m_02 = dist[0]*dist[2];
        s.second.jacobian.m_10 = dist[1]*dist[0];
        s.second.jacobian.m_11 = dist[1]*dist[1];
        s.second.jacobian.m_12 = dist[1]*dist[2];
        s.second.jacobian.m_20 = dist[2]*dist[0];
        s.second.jacobian.m_21 = dist[2]*dist[1];
        s.second.jacobian.m_22 = dist[2]*dist[2];
        // Finish off the math
        s.second.jacobian*=(m_k/dist.lengthSquared());
    }
}

void Cloth::updateMassJacobians()
{
    for(auto& m : m_mspts)
    {
        // collect springs attached to this mass, sum up jacobians
        ngl::Mat3 sjsum(0.0f);
        for(auto s : m.mySprings())
        {
            sjsum += m_springs.at(s).jacobian;
        }
        // negate sum
        sjsum *= -1.0f;
        // update mass jacobians
        m.updateJacobian(sjsum);
    }
}

void Cloth::calcWinverse(std::vector<ngl::Mat3> &o_wiiMatrices, const float _h)
{
    o_wiiMatrices.reserve(m_mspts.size());
    for(auto m : m_mspts)
    {
        // grab jacobian matrix
        auto wii = m.jacobian();
        // multiply elements by h squared
        wii.m_00 *= (_h * _h);
        wii.m_01 *= (_h * _h);
        wii.m_02 *= (_h * _h);
        wii.m_10 *= (_h * _h);
        wii.m_11 *= (_h * _h);
        wii.m_12 *= (_h * _h);
        wii.m_20 *= (_h * _h);
        wii.m_21 *= (_h * _h);
        wii.m_22 *= (_h * _h);
        // subtract from mass matrix
        wii.m_00 = m_mass - wii.m_00;
        wii.m_11 = m_mass - wii.m_11;
        wii.m_22 = m_mass - wii.m_22;
        // add damping
        if(m_useDamping)
        {
            wii.m_00 += m.mySprings().size() * _h * m_damping;
            wii.m_11 += m.mySprings().size() * _h * m_damping;
            wii.m_22 += m.mySprings().size() * _h * m_damping;
        }
        // add inverse to wii list
        o_wiiMatrices.push_back(wii.inverse());
    }
}

void Cloth::calcPforces(std::vector<ngl::Vec3> &o_pforces,
                        const ngl::Vec3 _externalf, const float _h)
{
    o_pforces.reserve(m_mspts.size());
    // Calc for each mass point
    for(auto m : m_mspts)
    {
        ngl::Vec3 springForce(0.0f);
        ngl::Vec3 viscosityForce(0.0f);
        ngl::Vec3 airRes(0.0f);
        // For each spring, add to spring force + viscosity force totals
        if(m.fixed() == false)
        {
            for(auto s : m.mySprings())
            {
                // Spring force
                auto dist = m_mspts[s.val1].pos() - m.pos();
                springForce += m_k * (dist.length() - m_springs.at(s).restl) * (dist/dist.length());
                // Viscosity force
                auto sjacobian = m_springs.at(s).jacobian;
                if(m_useDamping)
                {
                    sjacobian.m_00 += _h * m_damping;
                    sjacobian.m_11 += _h * m_damping;
                    sjacobian.m_22 += _h * m_damping;
                }
                viscosityForce += _h*_h*(sjacobian * (m_mspts[s.val1].vel() - m.vel()));
            }
            // Air resistance
            airRes = m.vel();
            if(airRes != ngl::Vec3(0.0f))
            {
                airRes.normalize();
                airRes *= -1.0f * m.vel().lengthSquared();
            }
        }
        // Sum, add to pforces list
        o_pforces.push_back((_h * (_externalf + airRes + springForce)) + viscosityForce);
    }
}
