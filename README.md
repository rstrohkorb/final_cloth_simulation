ASE Project, Cloth Simulation
Author: Rachel Strohkorb

This project contains a basic cloth simulator based on the Mass-Spring system. It can detect and respond to collisions with
a sphere object also contained in this project.

To run, you will need Qt to run qmake. You will also need NGL (found here: https://github.com/NCCA/NGL), a graphics library
written by Jon Macey for the NCCA at Bournemouth University.

HOW TO RUN:
    To run the test suite, build Test.pro and execute.
    To run the cloth simulator, build clothsim.pro and execute. There are several keyboard commands:
        W - turn on/off wireframe view
        B - draw/don't draw the sphere
        P - play sim
        S - stop sim
        R - reset cloth (this will set it back to the initial state)
        F - fix the four corners of the cloth in place
        C - turn on/off collisions (will only take effect once the cloth has been reset)

REPORT:

This project was created in C++.

There was actually very little change from my initial design and CGITec report. The class diagram from the initial report
still holds, with some additions and deletions as far as methods and member variables go. I'll proceed to talk about the
designs of the major classes in this project.

NGLScene:
    This class was rather obviously copied from BlankNGL, with contributions from the Grid project we did in class. My personal
    additions include drawing the cloth and sphere objects, and managing collisions between them. I initially tried to use
    Volino and Magnenat-Thalmann's 2006 paper [4] in order to do this, but I was unable to impliment it in time. This method
    would have been more robust and would have allowed the cloth to collide with other objects, but for now the collision response
    can only handle cloth/sphere collisions. This collision response simply tests if a vertex is inside the sphere, then
    moves it outside the sphere and fixes the vertex so that it doesn't collide with the sphere again.

Cloth:
    The cloth object was implimented as a simple mass-spring model. The only springs in the model are structural, so the cloth
    is unable to bend realistically (seen if you use the 'fixPointsFlag()' method),  but it does hang and droop in a satisfactory
    manner. The cloth consists of a set of masspoints, initialized in a grid, and springs, which are initialized to their resting
    lengths between mass points. For each box of four masspoints, there are four springs for the vertical and horizontal directions
    and one spring traveling diagonally from the southwest to the norteast corners. The springs are stored in an unordered map so
    that they can be referenced easily, but also so that {0,1} and {1,0} register as the same spring.
    For integration, I used an implicit method defined in Kang and Cho's 2002 paper[2] to integrate the cloth. It is an approximation
    designed for speed. I explain the math in detail in my CGITec report, and I tried to make it as easy to follow as possible
    within the code. Essentially, I integrate over the masses while doing small iterations to approximate the current velocities
    of neighboring masspoints.

SphereObj:
    An environment object for the cloth to collide with. I nabbed the code for its creation from createSphre() in VAOPrimitives
    in NGL, which I attributed in the code. It remains static, but can be moved around for testing purposes.

BVTree:
    The most complicated element in this project other than the cloth integration, and the one of which I am the most proud. This is
    a bounding volume tree, as defined in Klosowski, Held, Mitchell, Sowizral, and Zikan's 1998 paper [3]. Originally, the bounding
    volumes for this were going to be extensible to more than spheres, but I ran out of time to impliment that. The disadvantage of
    using spheres is that it detects erroneous collisions more often, so expensive triangle/triangle collision detection has to be
    performed more often. This is why collision detection runs so slowly with this system.
    I create the binary tree in a std::vector for cache efficiency, with each parent easily able to travel to the index of its
    children. In typical tree fashion, it creates and searches itself recursively. The search algorithm used is also from [3].
    The triangle/triangle collisions are calculated using a method from [1], a very helpful web page I found on the topic (cited in code).
    This is the only non-Jon unoriginal code in the project.

CITATIONS:
[1] BraynzarSoft, 2015. "Triangle to Triangle Collision Detection." Available from:
    https://www.braynzarsoft.net/viewtutorial/q16390-24-triangle-to-triangle-collision-detection
    [Accessed 23 January 2019].

[2] Kang, Y. M., and Cho, H. G., 2002.
    Complex deformable objects in virtual reality.
    VRST '02 Proceedings of the ACM symposium on Virtual reality software and technology, ACM, 49-56.

[3] Klosowski, J. T., Held, M., Mitchell, J. S. B., Sowizral, H., and Zikan, K., 1998.
    Efficient Collision Detection Using Bounding Volume Hierarchies of k-DOPs.
    IEEE Transactions on Visualization and Computer Graphics, 4 (1), 21-36.

[4] Volino, P. and Magnenat-Thalmann, N., 2006.
    Resolving surface collisions through intersection contour minimization.
    ACM Transactions on Graphics, 25 (3), 1154-1159.
