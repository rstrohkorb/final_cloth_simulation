#include <memory>
#include <gtest/gtest.h>
#include <ngl/Vec3.h>
#include "UnorderedPair.h"
#include "MassPoint.h"
#include "SphereBV.h"
#include "BVTree.h"
#include "SphereObj.h"
#include "Cloth.h"

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(UnorderedPair,userCtor)
{
    UnorderedPair up(1, 2);
    EXPECT_TRUE(up.val0 == 1);
    EXPECT_TRUE(up.val1 == 2);
}

TEST(MassPoint,defaultCtor)
{
    MassPoint m;
    //EXPECT_FLOAT_EQ(m.mass(), 1.0f);
    EXPECT_FALSE(m.fixed());
    EXPECT_TRUE(m.pos() == ngl::Vec3(0.0f));
    EXPECT_TRUE(m.vel() == ngl::Vec3(0.0f));
    EXPECT_TRUE(m.jacobian() == ngl::Mat3(0.0f));
}

TEST(MassPoint, usrCtor)
{
    MassPoint m(ngl::Vec3(1.0f));
    EXPECT_TRUE(m.pos() == ngl::Vec3(1.0f));
    EXPECT_TRUE(m.vel() == ngl::Vec3(0.0f));
}

TEST(MassPoint,setVelocity)
{
    MassPoint m;
    m.setVel(ngl::Vec3(0.8f));
    EXPECT_TRUE(m.vel() == ngl::Vec3(0.8f));
}

TEST(MassPoint, setPosition)
{
    MassPoint m;
    m.setPos(ngl::Vec3(0.4f));
    EXPECT_TRUE(m.pos() == ngl::Vec3(0.4f));
}

TEST(MassPoint,fixpoint)
{
    MassPoint m;
    m.setVel(ngl::Vec3(1.0f));
    m.fixPoint();
    EXPECT_TRUE(m.fixed());
    EXPECT_TRUE(m.vel() == ngl::Vec3(0.0f));
}

TEST(MassPoint,addSpring)
{
    MassPoint m;
    UnorderedPair spring(1,2);
    m.addSpring(spring);
    auto svector = m.mySprings();
    EXPECT_TRUE(svector[0].val0 == 1);
    EXPECT_TRUE(svector[0].val1 == 2);
}

TEST(SphereBV,defaultCtor)
{
    SphereBV sbv;
    EXPECT_TRUE(sbv.center == ngl::Vec3(0.0f));
    EXPECT_FLOAT_EQ(sbv.radius, 0.0f);
}

TEST(SphereBV,userCtor)
{
    std::vector<size_t> s = {1, 2, 3};
    SphereBV sbv(s);
    EXPECT_TRUE(sbv.surfaces == s);
}

TEST(BVTree,init)
{
    Cloth c(1.0f, 1.0f, 1.0f);
    c.init(3, 4, 0.4f);
    std::vector<ngl::Vec3> vertexData;
    c.exportTriangles(vertexData);
    BVTree bv;
    bv.init(vertexData);
    EXPECT_TRUE(bv.numSurfaces() == 12);
    EXPECT_TRUE(bv.numNodes() == 31);
}

TEST(BVTree,detectCollision)
{
    // Sphere/Sphere test
    SphereObj s1(5.0f, 15);
    SphereObj s2(5.0f, 15);
    s2.moveSphere(ngl::Vec3(0.0f, 12.0f, 0.0f));
    std::vector<ngl::Vec3> vertexData1;
    std::vector<ngl::Vec3> vertexData2;
    s1.exportTriangles(vertexData1);
    s2.exportTriangles(vertexData2);
    BVTree bv1;
    BVTree bv2;
    bv1.init(vertexData1);
    bv2.init(vertexData2);
    EXPECT_FALSE(bv1.detectCollision(bv2).detected);
    s2.moveSphere(ngl::Vec3(0.0f, -3.0f, 0.0f));
    vertexData2.clear();
    s2.exportTriangles(vertexData2);
    BVTree bv3;
    bv3.init(vertexData2);
    auto res = bv1.detectCollision(bv3);
    EXPECT_TRUE(res.detected);
    EXPECT_TRUE(res.collisions.size() == 77);
    // Sphere/Cloth test
    Cloth c(1.0f, 1.0f, 1.0f, true);
    c.init(20, 20, 0.5f, 0.0f);
    c.reposToOrigin(6.0f);
    std::vector<ngl::Vec3> clothvdata;
    c.exportTriangles(clothvdata);
    BVTree bvcloth;
    bvcloth.init(clothvdata);
    EXPECT_FALSE(bv1.detectCollision(bvcloth).detected);
    c.reposToOrigin(4.0f);
    clothvdata.clear();
    c.exportTriangles(clothvdata);
    BVTree bvcloth2;
    bvcloth2.init(clothvdata);
    EXPECT_TRUE(bv1.detectCollision(bvcloth2).detected);
    //std::cout<<"radius for 0.5 restl: "<<bvcloth2.getBV(bvcloth2.numNodes() - 1).radius<<'\n';
}

TEST(SphereObj,userCtor)
{
    SphereObj s(4.0f, 4);
    EXPECT_FLOAT_EQ(s.radius(), 4.0f);
    EXPECT_TRUE(s.center() == ngl::Vec3(0.0f));
    EXPECT_TRUE(s.numVertex() == 20);
}

TEST(SphereObj,init)
{
    SphereObj s(4.0f, 4);
    s.init();
    EXPECT_TRUE(s.numNodes() == 63);
}

TEST(SphereObj,exportTriangles)
{
    SphereObj s(4.0f, 4);
    std::vector<ngl::Vec3> tris;
    s.exportTriangles(tris);
    EXPECT_TRUE(tris.size() == 54);
}

TEST(SphereObj,moveSphere)
{
    SphereObj s(4.0f, 4);
    s.moveSphere(ngl::Vec3(3.0f, 3.0f, 3.0f));
    EXPECT_TRUE(s.center() == ngl::Vec3(3.0f, 3.0f, 3.0f));
}

TEST(Cloth,defaultCtor)
{
    Cloth c;
    EXPECT_TRUE(c.numSprings() == 0);
    EXPECT_TRUE(c.numMasses() == 0);
    EXPECT_TRUE(c.height() == 0);
    EXPECT_TRUE(c.width() == 0);
    EXPECT_FLOAT_EQ(c.point_mass(), 1.0f);
    EXPECT_FLOAT_EQ(c.spring_constant(), 1.0f);
    EXPECT_FLOAT_EQ(c.damping(), 1.0f);
    EXPECT_TRUE(c.useDamping());
    EXPECT_FALSE(c.collision());
}

TEST(Cloth,usrCtor)
{
    Cloth c(2.0f, 2.0f, 3.0f, true);
    EXPECT_FLOAT_EQ(c.point_mass(), 2.0f);
    EXPECT_FLOAT_EQ(c.spring_constant(), 2.0f);
    EXPECT_FLOAT_EQ(c.damping(), 3.0f);
    EXPECT_TRUE(c.collision());
}

TEST(Cloth,setDamping)
{
    Cloth c(1.0f, 1.0f, 1.0f);
    c.setDamping(false);
    EXPECT_FALSE(c.useDamping());
}

TEST(Cloth,init)
{
    Cloth c(1.0f, 1.0f, 1.0f);
    size_t height = 3;
    size_t width = 4;
    size_t numMasses = height * width;
    size_t numSprings = (width*(height-1)) + (height*(width-1)) + ((height-1)*(width-1));
    c.init(height, width, 0.4f, 0.0f);
    EXPECT_TRUE(c.numMasses() == numMasses);
    EXPECT_TRUE(c.numSprings() == numSprings);
}

TEST(Cloth,center)
{
    Cloth c1(1.0f, 1.0f, 1.0f);
    c1.init(3, 3, 1.0f, 0.0f);
    Cloth c2(1.0f, 1.0f, 1.0f);
    c2.init(4, 4, 1.0f, 0.0f);
    Cloth c3(1.0f, 1.0f, 1.0f);
    c3.init(3, 4, 1.0f, 0.0f);
    Cloth c4(1.0f, 1.0f, 1.0f);
    c4.init(4, 3, 1.0f, 0.0f);
    EXPECT_TRUE(c1.center() == ngl::Vec3(1.0f, 0.0f, 1.0f));
    EXPECT_TRUE(c2.center() == ngl::Vec3(1.5f, 0.0f, 1.5f));
    EXPECT_TRUE(c3.center() == ngl::Vec3(1.5f, 0.0f, 1.0f));
    EXPECT_TRUE(c4.center() == ngl::Vec3(1.0f, 0.0f, 1.5f));
}

TEST(Cloth,vertFromTriNum)
{
    Cloth c(1.0f, 1.0f, 1.0f);
    c.init(3, 4, 1.0f, 0.0f);
    std::vector<ngl::Vec3> checkVertTri0 = {ngl::Vec3(0.0f, 0.0f, 1.0f), ngl::Vec3(0.0f), ngl::Vec3(1.0f, 0.0f, 0.0f)};
    std::vector<ngl::Vec3> checkVertTri9 = {ngl::Vec3(1.0f, 0.0f, 2.0f), ngl::Vec3(2.0f, 0.0f, 1.0f), ngl::Vec3(2.0f, 0.0f, 2.0f)};
    EXPECT_TRUE(c.vertFromTriNum(0) == checkVertTri0);
    EXPECT_TRUE(c.vertFromTriNum(9) == checkVertTri9);
}

TEST(Cloth,reposToOrigin)
{
    Cloth c(1.0f, 1.0f, 1.0f);
    c.init(3, 4, 1.0f, 0.0f);
    c.reposToOrigin(0.0f);
    EXPECT_TRUE(c.center() == ngl::Vec3(0.0f));
    c.reposToOrigin(10.0f);
    EXPECT_TRUE(c.center() == ngl::Vec3(0.0f, 10.0f, 0.0f));
}

TEST(Cloth,update)
{
    Cloth c(1.0f, 1.0f, 1.0f);
    c.init(3, 4, 0.5f, 0.0f);
    ngl::Vec3 expected(0.75f, -3.10443f, 0.5f);
    for(size_t i = 0; i < 10; ++i)
    {
        c.update(0.5f);
    }
    EXPECT_TRUE(c.center() == expected);
}

TEST(Cloth,exportTriangles)
{
    Cloth c(1.0f, 1.0f, 1.0f);
    c.init(3, 4, 1.0f, 0.0f);
    std::vector<ngl::Vec3> tri;
    c.exportTriangles(tri);
    EXPECT_TRUE(tri.size() == 36);
}
