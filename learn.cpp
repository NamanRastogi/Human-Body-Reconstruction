/*
 ***********************************************************************
 ***********************************************************************
 *
 * Header Files
 *
 ***********************************************************************
 ***********************************************************************
 */

#include "global.hpp"
#include "gl_operations.hpp"
#include "file_operations.hpp"
#include "scape_operations_learn.hpp"

#include <cstdlib>



/*
 ***********************************************************************
 ***********************************************************************
 *
 * Main
 *
 ***********************************************************************
 ***********************************************************************
 */

int main( int argc, char** argv )
{
    if( argc < 2 ) {
        std::cout << "Usage: " << argv[0] << " <Data Directory>" << std::endl;
        exit( EXIT_SUCCESS );
    }

    DATA_DIR = std::string( argv[1] );

    // Load meshes & skeleton and initialize mesh objects
    initData();

    // Compute matR, matQ, matS, deltaR, regression vector A
    // and PCA parameters
    processData();

    // Print bounding box
    std::cout << "Bounding Box :" << std::endl;
    std::cout << "  Corner 1 : " << BB_pCorner.x << " " << BB_pCorner.y << " " << BB_pCorner.z
              << std::endl;
    std::cout << "  Corner 2 : " << BB_nCorner.x << " " << BB_nCorner.y << " " << BB_nCorner.z
              << std::endl;

    // Initialize Camera position and direction
    cameraPosition = glm::vec3( BB_pCorner.x + ( BB_pCorner.x - BB_nCorner.x ) / 2,
                                BB_pCorner.y + ( BB_pCorner.y - BB_nCorner.y ) / 2,
                                BB_pCorner.z + ( BB_pCorner.z - BB_nCorner.z ) / 2 );
    float	distX   = BB_pCorner.x - BB_nCorner.x,
            distY   = BB_pCorner.y - BB_nCorner.y,
            distZ   = BB_pCorner.z - BB_nCorner.z,
            distXZ  = glm::sqrt( pow( distX,  2 ) + pow( distZ, 2 ) ),
            distXYZ = glm::sqrt( pow( distXZ, 2 ) + pow( distY, 2 ) );
    pitch =  glm::degrees( -( glm::atan( distY, distXZ ) ) );
    yaw = 180 + glm::degrees( glm::atan( distZ, distX ) );

    cameraSpeed = distXYZ / 200.0;

    // Print Camera position
    std::cout << std::endl;
    std::cout << "Camera Initial Position : " << cameraPosition.x << " " << cameraPosition.y
              << " " << cameraPosition.z << std::endl;

    /*
     * OpenGL stuff
     */
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE );
    // glutInitWindowPosition( 10, 10 );
    glutInitWindowSize( 800, 600 );
    // glutInitContextVersion( 2, 0 );
    // glutInitContextFlags( GLUT_FORWARD_COMPATIBLE | GLUT_DEBUG );
    glutCreateWindow( "Human Body Reconstruction" );

    glEnable( GLUT_MULTISAMPLE );
    glEnable( GL_DEPTH_TEST );

    // std::cout << std::endl << "OpenGL Version : " << glGetString( GL_VERSION ) << std::endl;

    glutDisplayFunc( displayCallback );
    glutIdleFunc( displayCallback );
    glutReshapeFunc( reshapeCallback );
    glutKeyboardFunc( keyboardCallback );
    glutSpecialFunc( keyboardSpecialDownCallback );
    glutSpecialUpFunc( keyboardSpecialUpCallback );
    glutMouseFunc( mouseButtonCallback );
    glutMotionFunc( mouseMotionCallback );

    glutIgnoreKeyRepeat( 1 );

    glutMainLoop();

    return 0;
}
