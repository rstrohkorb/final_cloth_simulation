#include <QMouseEvent>
#include <QGuiApplication>

#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/ShaderLib.h>
#include <ngl/SimpleVAO.h>
#include <ngl/VAOFactory.h>
#include <ngl/Transformation.h>
#include <iostream>

NGLScene::NGLScene()
{
    // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
    setTitle("Cloth Simulation Rachel Strohkorb");
    // Create cloth
    m_cloth = Cloth(1.0f, 720.0f, 720.0f, m_collisionOn);
    m_cloth.init(20, 20, 0.4f, 0.0f, true);
    m_cloth.reposToOrigin(0.0f);
    m_cloth.fixptHang();
    m_writeOutCloth = false;
    // Create Sphere
    m_sphere = SphereObj(4.0f, 40);
    m_sphere.init();
}


NGLScene::~NGLScene()
{
    std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
}

void NGLScene::timerEvent(QTimerEvent *_event)
{
    m_timerId = _event->timerId();
    if(m_cloth.fullClothFixed() == false)
    {
        // write cloth to obj
        if(m_writeOutCloth)
        {
            // update count - we want 4 places in the number, so leading 0's matter
            std::string count;
            auto numUpdates = m_cloth.numUpdates();
            if(numUpdates < 10)
            {
                count = "000" + std::to_string(numUpdates);
            }
            else if(numUpdates < 100)
            {
                count = "00" + std::to_string(numUpdates);
            }
            else if(numUpdates < 1000)
            {
                count = "0" + std::to_string(numUpdates);
            }
            else
            {
                count = std::to_string(numUpdates);
            }
            std::string filename;
            filename = m_filenameDefault + count + ".obj";
            m_cloth.writeToObj(filename);
        }
        // update cloth
        m_cloth.update(0.5f, false);
        // collisions
        if(m_collisionOn)
        {
            collisionResponse(m_sphere.detectCollision(m_cloth.exportTree()));
        }
    }
    // redraw
    update();
}

void NGLScene::resizeGL(int _w , int _h)
{
    m_win.width  = static_cast<int>( _w * devicePixelRatio() );
    m_win.height = static_cast<int>( _h * devicePixelRatio() );
    //field of view, aspect ratio, near clipping plane, far clipping plane
    m_project = ngl::perspective(45.0f, static_cast<float>(_w/_h),
                               0.5f, 200.0f);
}

constexpr auto ColorShader = "ColorShader";

void NGLScene::initializeGL()
{
    // we need to initialise the NGL lib which will load all of the OpenGL functions, this must
    // be done once we have a valid GL context but before we call any GL commands. If we dont do
    // this everything will crash
    ngl::NGLInit::instance();
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);			   // Grey Background
    // enable depth testing for drawing
    glEnable(GL_DEPTH_TEST);
    // enable multisampling for smoother drawing
    glEnable(GL_MULTISAMPLE);
    // load shaders
    ngl::ShaderLib *shader = ngl::ShaderLib::instance();
//    shader->loadShader(ColorShader, "shaders/ColorVertex.glsl",
//                     "shaders/ColorFragment.glsl");

    //set the camera
    ngl::Vec3 from(0.0f, 10.0f, 20.0f);
    m_view = ngl::lookAt(from, ngl::Vec3::zero(), ngl::Vec3::up());

    // ngl checker shader
    shader->use(ngl::nglCheckerShader);
    shader->setUniform("lightDiffuse",1.0f,1.0f,1.0f,1.0f);
    shader->setUniform("checkOn",true);
    shader->setUniform("lightPos",from);
    shader->setUniform("colour1",0.9f,0.9f,0.9f,1.0f);
    shader->setUniform("colour2",0.6f,0.6f,0.6f,1.0f);
    shader->setUniform("checkSize",15.0f);

    //make a simple vao for the cloth triangles
    m_clothVAO = ngl::VAOFactory::createVAO(ngl::simpleVAO, GL_TRIANGLES);
    //make a simple vao for the sphere triangle strips
    m_sphereVAO = ngl::VAOFactory::createVAO(ngl::simpleVAO, GL_TRIANGLE_STRIP);
}



void NGLScene::paintGL()
{
    // clear the screen and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0,0,m_win.width,m_win.height);
    // determine mouse rotation
    ngl::Mat4 rotx;
    ngl::Mat4 roty;
    ngl::Mat4 mouseRotation;
    rotx.rotateX(m_win.spinXFace);
    roty.rotateY(m_win.spinYFace);
    mouseRotation = roty * rotx;
    // draw the cloth    
    std::vector<float> tri;
    m_cloth.exportTriangles(tri);

    m_clothVAO->bind();
    m_clothVAO->setData(ngl::SimpleVAO::VertexData(tri.size()*sizeof(float), tri[0]));
    m_clothVAO->setVertexAttributePointer(0, 3, GL_FLOAT, 8*sizeof(float), 0);
    m_clothVAO->setVertexAttributePointer(1, 3, GL_FLOAT, 8*sizeof(float), 3);
    m_clothVAO->setVertexAttributePointer(2, 2, GL_FLOAT, 8*sizeof(float), 6);

    m_clothVAO->setNumIndices((tri.size()/8)*3);
    loadMatrixToCheckerShader(mouseRotation);

    m_clothVAO->draw();
    m_clothVAO->unbind();
    // draw the sphere
    if(m_sphereOn)
    {
        auto tristrip = m_sphere.exportForDraw();

        m_sphereVAO->bind();
        m_sphereVAO->setData(ngl::SimpleVAO::VertexData(tristrip.size()*sizeof(ngl::Vec3),
                                                        tristrip[0].m_x));
        m_sphereVAO->setVertexAttributePointer(0, 3, GL_FLOAT, sizeof(ngl::Vec3), 0);
        m_sphereVAO->setNumIndices(tristrip.size());
        loadMatrixToShader(mouseRotation, ngl::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
        m_sphereVAO->draw();
        m_sphereVAO->unbind();
    }
}

void NGLScene::loadMatrixToShader(const ngl::Mat4 &_tx, const ngl::Vec4 &_color)
{
    ngl::ShaderLib *shader = ngl::ShaderLib::instance();
    shader->use(ColorShader);
    shader->setUniform("MVP", m_project * m_view * _tx);
    shader->setUniform("vertColor", _color);
}

void NGLScene::loadMatrixToCheckerShader(const ngl::Mat4 &_tx)
{
    ngl::ShaderLib* shader = ngl::ShaderLib::instance();
    shader->use(ngl::nglCheckerShader);
    ngl::Mat4 MVP= m_project * m_view * _tx;
    ngl::Mat3 normalMatrix= m_view * _tx;
    normalMatrix.inverse().transpose();
    shader->setUniform("MVP",MVP);
    shader->setUniform("normalMatrix",normalMatrix);
}

void NGLScene::collisionResponse(ColDetectInfo _cdi)
{
    // If there are colliding triangles within these structures
    if(_cdi.detected)
    {
        // For each triangle collision
        for(auto t : _cdi.collisions)
        {
            // Vertices of offending cloth triangle
            auto vertCloth = m_cloth.vertFromTriNum(t.otherTriNum);
            // Check the vertices, move them outside the sphere if necessary
            for(auto &v : vertCloth)
            {
                auto dist = (m_sphere.center() - v).length();
                if(dist < m_sphere.radius())
                {
                    auto distToMove = m_sphere.radius() - dist;
                    auto vectorToMove = v - m_sphere.center();
                    vectorToMove.normalize();
                    v += (vectorToMove * distToMove);
                }
            }
            // Apply changes
            m_cloth.modVertFromTriNum(t.otherTriNum, vertCloth);
        }
    }
}

void NGLScene::collisionResponseFail(ColDetectInfo _cdi)
{
    /* This function is depricated. It could have been a more accurate way of detecting collisions with
     * more complex objects, but I wasn't able to get it working in time. This method was based on Volino and
     * Magnenat-Thalmann's paper "Resolving Surface Collisions through Intersection Contour Minimization."
     * While I was able to obtain the direction vector that would minimize the contour length, the paper was
     * unclear on what to do with this vector in practice.
     *
     * I decided to keep the code around in case I came back to this project later, as their method has
     * a lot of potential to work well with cloth-cloth collisions.
     */
    if(_cdi.detected)
    {
        ngl::Vec3 d(0.1f, 0.1f, 0.1f);
        for(auto t : _cdi.collisions)
        {
            // Grab vertices of surface triangle
            auto vertEnviron = m_sphere.vertFromTriNum(t.selfTriNum);
            // Calculate normal of surface triangle
            auto environNormal = (vertEnviron[1] - vertEnviron[0]).cross(vertEnviron[2] - vertEnviron[0]);
            // Go through each intersecting edge
            std::vector<ngl::Vec3> newfly1;
            std::vector<ngl::Vec3> newfly2;
            std::vector<ngl::Vec3> newfly3;
            for(size_t i = 0; i < t.edgeNums.size(); ++i)
            {
                if(t.edgeNums[i] == 0)
                {
                    // Gather planes connected to this edge
                    std::vector<std::vector<ngl::Vec3>> flyPlanes;
                    std::vector<size_t> edges;
                    flyPlanes.push_back(m_cloth.vertFromTriNum(t.otherTriNum));
                    edges.push_back(t.edgeNums[i]);
                    if((t.otherTriNum % 2 == 0) && (t.otherTriNum % ((m_cloth.width() -1) * 2) != 0))
                    {
                        flyPlanes.push_back(m_cloth.vertFromTriNum(t.otherTriNum - 1));
                        edges.push_back((t.edgeNums[i] + 1) % 3);
                    }
                    else if(t.otherTriNum % 2 != 0)
                    {
                        flyPlanes.push_back(m_cloth.vertFromTriNum(t.otherTriNum - 1));
                        edges.push_back((t.edgeNums[i] + 2) % 3);
                    }
                    // Loop over planes
                    ngl::Vec3 g(0.0f);
                    for(size_t i = 0; i < flyPlanes.size(); ++i)
                    {
                        // Grab necessary elements for equation
                        auto edge = edgeFromNum(edges[i], flyPlanes[i]);
                        auto flyNormal = (flyPlanes[i][1] - flyPlanes[i][0]).cross(flyPlanes[i][2] - flyPlanes[i][0]);
                        auto ri = environNormal.cross(flyNormal);
                        ri.normalize();
                        g += (ri - (2 * (edge.dot(ri)/edge.dot(environNormal)) * environNormal));
                    }
                    g.normalize();
                    newfly1.push_back(flyPlanes[0][0] + (g * d.length()));
                    newfly2.push_back(flyPlanes[0][1] + (g * d.length()));
                }
                else if(t.edgeNums[i] == 1)
                {
                    // Gather planes connected to this edge
                    std::vector<std::vector<ngl::Vec3>> flyPlanes;
                    std::vector<size_t> edges;
                    flyPlanes.push_back(m_cloth.vertFromTriNum(t.otherTriNum));
                    edges.push_back(t.edgeNums[i]);
                    if(((t.otherTriNum % 2 == 0) && (t.otherTriNum/((m_cloth.width() -1) * 2) != 0)))
                    {
                        flyPlanes.push_back(m_cloth.vertFromTriNum(t.otherTriNum - ((m_cloth.width() -1) * 2)));
                        edges.push_back((t.edgeNums[i] + 1) % 3);
                    }
                    else if ((t.otherTriNum % 2 != 0) && (t.otherTriNum % ((m_cloth.width() -1) * 2) != (((m_cloth.width() -1) * 2) - 1)))
                    {
                        flyPlanes.push_back(m_cloth.vertFromTriNum(t.otherTriNum + 1));
                        edges.push_back((t.edgeNums[i] + 2) % 3);
                    }
                    // Loop over planes
                    ngl::Vec3 g(0.0f);
                    for(size_t i = 0; i < flyPlanes.size(); ++i)
                    {
                        // Grab necessary elements for equation
                        auto edge = edgeFromNum(edges[i], flyPlanes[i]);
                        auto flyNormal = (flyPlanes[i][1] - flyPlanes[i][0]).cross(flyPlanes[i][2] - flyPlanes[i][0]);
                        auto ri = environNormal.cross(flyNormal);
                        ri.normalize();
                        g += (ri - (2 * (edge.dot(ri)/edge.dot(environNormal)) * environNormal));
                    }
                    g.normalize();
                    newfly2.push_back(flyPlanes[0][1] + (g * d.length()));
                    newfly3.push_back(flyPlanes[0][2] + (g * d.length()));
                }
                else
                {
                    // Gather planes connected to this edge
                    std::vector<std::vector<ngl::Vec3>> flyPlanes;
                    std::vector<size_t> edges;
                    flyPlanes.push_back(m_cloth.vertFromTriNum(t.otherTriNum));
                    edges.push_back(t.edgeNums[i]);
                    if(t.otherTriNum % 2 == 0)
                    {
                        flyPlanes.push_back(m_cloth.vertFromTriNum(t.otherTriNum + 1));
                        edges.push_back((t.edgeNums[i] + 1) % 3);
                    }
                    else if ((t.otherTriNum % 2 != 0) && ((t.otherTriNum / ((m_cloth.width() -1) * 2)) != (m_cloth.height() - 2)))
                    {
                        flyPlanes.push_back(m_cloth.vertFromTriNum(t.otherTriNum + ((m_cloth.width() -1) * 2)));
                        edges.push_back((t.edgeNums[i] + 2) % 3);
                    }
                    // Loop over planes
                    ngl::Vec3 g(0.0f);
                    for(size_t i = 0; i < flyPlanes.size(); ++i)
                    {
                        // Grab necessary elements for equation
                        auto edge = edgeFromNum(edges[i], flyPlanes[i]);
                        auto flyNormal = (flyPlanes[i][1] - flyPlanes[i][0]).cross(flyPlanes[i][2] - flyPlanes[i][0]);
                        auto ri = environNormal.cross(flyNormal);
                        ri.normalize();
                        g += (ri - (2 * (edge.dot(ri)/edge.dot(environNormal)) * environNormal));
                    }
                    g.normalize();
                    newfly3.push_back(flyPlanes[0][2] + (g * d.length()));
                    newfly1.push_back(flyPlanes[0][0] + (g * d.length()));
                }
            }
            // Average out the changes
            std::vector<ngl::Vec3> newFlyFinal;
            newFlyFinal.reserve(3);
            if(newfly1.size() != 0)
            {
                newFlyFinal.push_back(std::accumulate(newfly1.begin(), newfly1.end(), ngl::Vec3(0.0f)));
            }
            else
            {
                newFlyFinal.push_back(m_cloth.vertFromTriNum(t.otherTriNum)[0]);
            }
            if(newfly2.size() != 0)
            {
                newFlyFinal.push_back(std::accumulate(newfly2.begin(), newfly2.end(), ngl::Vec3(0.0f)));
            }
            else
            {
                newFlyFinal.push_back(m_cloth.vertFromTriNum(t.otherTriNum)[1]);
            }
            if(newfly3.size() != 0)
            {
                newFlyFinal.push_back(std::accumulate(newfly3.begin(), newfly3.end(), ngl::Vec3(0.0f)));
            }
            else
            {
                newFlyFinal.push_back(m_cloth.vertFromTriNum(t.otherTriNum)[2]);
            }
            // Apply changes
            m_cloth.modVertFromTriNum(t.otherTriNum, newFlyFinal);
        }
    }
}

ngl::Vec3 NGLScene::edgeFromNum(const size_t _num, std::vector<ngl::Vec3> _vert)
{
    if(_num == 0)
    {
        return _vert[0] - _vert[1];
    }
    else if(_num == 1)
    {
        return _vert[1] - _vert[2];
    }
    else
    {
        return _vert[2] - _vert[0];
    }
}

//----------------------------------------------------------------------------------------------------------------------

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
    // this method is called every time the main window recives a key event.
    // we then switch on the key value and set the camera in the GLWindow
    switch (_event->key())
    {
    // escape key to quite
    case Qt::Key_Escape : QGuiApplication::exit(EXIT_SUCCESS); break;
    case Qt::Key_Space :
        m_win.spinXFace=0;
        m_win.spinYFace=0;
        m_modelPos.set(ngl::Vec3::zero());
        break;
    case Qt::Key_W :
        if(m_wireframeOn == false)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            m_wireframeOn = true;
        }
        else
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            m_wireframeOn = false;
        }
        break;
    case Qt::Key_S : killTimer(m_timerId); break;
    case Qt::Key_U :
        if(m_cloth.fullClothFixed() == false)
        {
            m_cloth.update(0.5f, false);
            if(m_collisionOn)
            {
                collisionResponse(m_sphere.detectCollision(m_cloth.exportTree()));
            }
        }
        break;
    case Qt::Key_P : startTimer(7); break;
    case Qt::Key_R:
        killTimer(m_timerId);
        m_cloth = Cloth(1.0f, 720.0f, 720.0f, m_collisionOn);
        m_cloth.init(20, 20, 0.4f, 0.0f);
        m_cloth.reposToOrigin(5.0f);
        break;
    case Qt::Key_B: m_sphereOn = !m_sphereOn; break;
    case Qt::Key_C: m_collisionOn = !m_collisionOn; break;
    case Qt::Key_F: m_cloth.fixptCorners(); break;
    default : break;
    }
    // finally update the GLWindow and re-draw

    update();
}
