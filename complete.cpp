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
#include "scape_operations_complete.hpp"

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
    if( argc < 3 ) {
        std::cout << "Usage: " << argv[0] << " <Data Directory> <Point Cloud PTCLD File> <Actual Mesh file> <Initial matR File>" << std::endl;
        exit( EXIT_SUCCESS );
    }

    DATA_DIR = argv[1];
    std::string pointCloud_fileName = argv[2];
    std::string actual_mesh_fileName = argv[3];
    std::string initial_matR_fileName = argv[3];

    // Initialize mesh objects and load A and PCA parameters
    load_model();

    // Load Point Cloud from givel filename as command line arguement
    load_pointCloud( pointCloud_fileName );

    // Generate mesh approximating given point cloud
    // ** THIS PART DOES NOT WORK HERE **
    // ** PYTHON IMPLEMENTATION FOR THIS WORKS **
    generate_mesh( initial_matR_fileName );
    std::cout << "Final mesh estimation error: " << estimate_final_error( actual_mesh_fileName )
              << std::endl;

    /*
    std::cout << "Bounding Box :" << std::endl;
    std::cout << "  Corner 1 : " << BB_pCorner.x << " " << BB_pCorner.y << " " << BB_pCorner.z
              << std::endl;
    std::cout << "  Corner 2 : " << BB_nCorner.x << " " << BB_nCorner.y << " " << BB_nCorner.z
              << std::endl;

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

    std::cout << std::endl;
    std::cout << "Camera Initial Position : " << cameraPosition.x << " " << cameraPosition.y
              << " " << cameraPosition.z << std::endl;

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
    */

    return 0;
}
