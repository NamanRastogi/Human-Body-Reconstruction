#ifndef HUMAN_BODY_RECONSTRUCTION_GL_OPERATIONS_HPP
#define HUMAN_BODY_RECONSTRUCTION_GL_OPERATIONS_HPP

/*
 * NOTE:
 * This file containes the OpenGL callback functions
 * It does not contain much information about the SCAPE implementation
 */


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



/*
 ***********************************************************************
 ***********************************************************************
 *
 * Global Variables
 *
 ***********************************************************************
 ***********************************************************************
 */


// Camera position and directions
glm::vec3 upDirection = glm::vec3( 0.0f, 1.0f, 0.0f );
float cameraMoveFront = 0.0f,
      cameraMoveRight = 0.0f,
      cameraMoveUp = 0.0f;

double yaw = -90.0, pitch = 0.0;
int lastMousePosX, lastMousePosY;
float deltaYaw, deltaPitch;

float cameraSpeed = 0.5f;
float mouseSensetivity = 0.2f;

// Bool representing whether to render the Axes
bool enableAxes = true;



/*
 ***********************************************************************
 ***********************************************************************
 *
 * Function Definitions
 *
 ***********************************************************************
 ***********************************************************************
 */


/*
 * OpenGL callback for rendering stuff
 */
void displayCallback()
{
    // Initializations
    glMatrixMode( GL_MODELVIEW );

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );

    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    glLoadIdentity();

    if( pitch > 89.0f )
        pitch = 89.0f;
    if( pitch < -89.0f )
        pitch = -89.0f;
    if( ( pitch + deltaPitch ) > 89.0f )
        deltaPitch = 89.0f - pitch;
    if( ( pitch + deltaPitch ) < -89.0f )
        deltaPitch = -89.0f - pitch;

    // Set camera direction
    cameraDirection = glm::normalize( glm::vec3(
        glm::cos( glm::radians(pitch + deltaPitch) ) * glm::cos( glm::radians(yaw + deltaYaw) ),
        glm::sin( glm::radians(pitch + deltaPitch) ),
        glm::cos( glm::radians(pitch + deltaPitch) ) * glm::sin( glm::radians(yaw + deltaYaw) )
    ) );
    cameraRightDirection = glm::normalize( glm::cross( upDirection, cameraDirection ) );
    cameraPosition += ( cameraMoveFront * cameraDirection
                        + cameraMoveRight * cameraRightDirection
                        + cameraMoveUp * upDirection
                      ) * cameraSpeed;

    gluLookAt(cameraPosition.x,						// Eye position X
              cameraPosition.y,						// Eye position Y
              cameraPosition.z,						// Eye position Z
              cameraPosition.x + cameraDirection.x, // Target Position X
              cameraPosition.y + cameraDirection.y, // Target Position Y
              cameraPosition.z + cameraDirection.z, // Target Position Z
              upDirection.x,						// Up vector X
              upDirection.y,						// Up vector Y
              upDirection.z);						// Up vector Z

    // Draw Axis
    if( enableAxes ) {
        glBegin( GL_LINES );
        glColor3f( 1.0f, 0.0f, 0.0f );
        glVertex3f( -10.0f, 0.0f, 0.0f );
        glVertex3f( 10.0f, 0.0f, 0.0f );
        glColor3f( 0.0f, 1.0f, 0.0f );
        glVertex3f( 0.0f, -10.0f, 0.0f );
        glVertex3f( 0.0f, 10.0f, 0.0f );
        glColor3f( 0.0f, 0.0f, 1.0f );
        glVertex3f( 0.0f, 0.0f, -10.0f );
        glVertex3f( 0.0f, 0.0f, 10.0f );
        glEnd();
    }

    // Draw meshes
    /* glTranslatef(-10.0f,0.0f,0.0f);
    glBegin( GL_TRIANGLES );
        glColor3f( 1.0f, 1.0f, 1.0f );
        for( int i = 0; i < numFaces; ++i ) {
            glColor4f( boneColor[ vertBone[ template_mesh->Faces[i].x ] ].x,
                       boneColor[ vertBone[ template_mesh->Faces[i].x ] ].y,
                       boneColor[ vertBone[ template_mesh->Faces[i].x ] ].z, 0.9 );
            glVertex3f( obj_POSE[meshPose].Vertices[ size_t(template_mesh->Faces[i].x) ].x,
                        obj_POSE[meshPose].Vertices[ size_t(template_mesh->Faces[i].x) ].y,
                        obj_POSE[meshPose].Vertices[ size_t(template_mesh->Faces[i].x) ].z );
            glVertex3f( obj_POSE[meshPose].Vertices[ size_t(template_mesh->Faces[i].y) ].x,
                        obj_POSE[meshPose].Vertices[ size_t(template_mesh->Faces[i].y) ].y,
                        obj_POSE[meshPose].Vertices[ size_t(template_mesh->Faces[i].y) ].z );
            glVertex3f( obj_POSE[meshPose].Vertices[ size_t(template_mesh->Faces[i].z) ].x,
                        obj_POSE[meshPose].Vertices[ size_t(template_mesh->Faces[i].z) ].y,
                        obj_POSE[meshPose].Vertices[ size_t(template_mesh->Faces[i].z) ].z );
        }
    glEnd();
    glTranslatef(10.0f,0.0f,0.0f); */

    /* glTranslatef(1.2,0.0f,0.1f);
    glBegin( GL_TRIANGLES );
    glColor3f( 1.0f, 1.0f, 1.0f );
    for( int i = 0; i < numFaces; ++i ) {
        glColor4f( boneColor[ vertBone[ template_mesh->Faces[i].x ] ].x,
                   boneColor[ vertBone[ template_mesh->Faces[i].x ] ].y,
                   boneColor[ vertBone[ template_mesh->Faces[i].x ] ].z, 0.9 );
        glVertex3f( obj_SHAPE[meshShape].Vertices[ size_t(template_mesh->Faces[i].x) ].x,
                    obj_SHAPE[meshShape].Vertices[ size_t(template_mesh->Faces[i].x) ].y,
                    obj_SHAPE[meshShape].Vertices[ size_t(template_mesh->Faces[i].x) ].z );
        glVertex3f( obj_SHAPE[meshShape].Vertices[ size_t(template_mesh->Faces[i].y) ].x,
                    obj_SHAPE[meshShape].Vertices[ size_t(template_mesh->Faces[i].y) ].y,
                    obj_SHAPE[meshShape].Vertices[ size_t(template_mesh->Faces[i].y) ].z );
        glVertex3f( obj_SHAPE[meshShape].Vertices[ size_t(template_mesh->Faces[i].z) ].x,
                    obj_SHAPE[meshShape].Vertices[ size_t(template_mesh->Faces[i].z) ].y,
                    obj_SHAPE[meshShape].Vertices[ size_t(template_mesh->Faces[i].z) ].z );
    }
    glEnd();
    glTranslatef(-1.2f,0.0f,-0.1f); */

    // glTranslatef(10.0f,0.0f,0.0f);
    // glBegin( GL_TRIANGLES );
    // glColor3f( 1.0f, 1.0f, 1.0f );
    // for( int i = 0; i < numFaces; ++i ) {
    // 	glColor4f(  boneColor[ vertBone[ template_mesh->Faces[i].x ] ].x,
    // 				boneColor[ vertBone[ template_mesh->Faces[i].x ] ].y,
    // 				boneColor[ vertBone[ template_mesh->Faces[i].x ] ].z, 0.9 );
    // 	glVertex3f( Y->Vertices[ size_t(template_mesh->Faces[i].x) ].x,
    // 				Y->Vertices[ size_t(template_mesh->Faces[i].x) ].y,
    // 				Y->Vertices[ size_t(template_mesh->Faces[i].x) ].z );
    // 	glVertex3f( Y->Vertices[ size_t(template_mesh->Faces[i].y) ].x,
    // 				Y->Vertices[ size_t(template_mesh->Faces[i].y) ].y,
    // 				Y->Vertices[ size_t(template_mesh->Faces[i].y) ].z );
    // 	glVertex3f( Y->Vertices[ size_t(template_mesh->Faces[i].z) ].x,
    // 				Y->Vertices[ size_t(template_mesh->Faces[i].z) ].y,
    // 				Y->Vertices[ size_t(template_mesh->Faces[i].z) ].z );
    // }
    // glEnd();
    // glTranslatef(-10.0f,0.0f,0.0f);

    glBegin( GL_TRIANGLES );
    for( int i = 0; i < numFaces; ++i ) {
        glColor4f( boneColor[vertBone[Y->Faces[i].x]].x,
                   boneColor[vertBone[Y->Faces[i].x]].y,
                   boneColor[vertBone[Y->Faces[i].x]].z, 0.9 );
        glVertex3f( Y->Vertices[size_t( Y->Faces[i].x )].x, Y->Vertices[size_t( Y->Faces[i].x )].y,
                    Y->Vertices[size_t( Y->Faces[i].x )].z );
        glVertex3f( Y->Vertices[size_t( Y->Faces[i].y )].x, Y->Vertices[size_t( Y->Faces[i].y )].y,
                    Y->Vertices[size_t( Y->Faces[i].y )].z );
        glVertex3f( Y->Vertices[size_t( Y->Faces[i].z )].x, Y->Vertices[size_t( Y->Faces[i].z )].y,
                    Y->Vertices[size_t( Y->Faces[i].z )].z );
    }
    glEnd();

    /*glColor4f( 1.0f, 1.0f, 1.0f, 1.0f );
    glutSolidSphere(0.01, 10, 10);
    glBegin(GL_LINES);
    //		for( int b = 0; b < numBones; ++b ) {
        int b = boneDirTest; {
            glColor4f( boneColor[b].x, boneColor[b].y, boneColor[b].z, 1.0f );
            glVertex3f(0.0f,0.0f,0.0f);
            glVertex3f( obj_POSE[meshToRender].boneDirection[b].x * 0.5f,
                        obj_POSE[meshToRender].boneDirection[b].y * 0.5f,
                        obj_POSE[meshToRender].boneDirection[b].z * 0.5f );
        }
    glEnd();*/

    glutSwapBuffers();
}



/*
 * OpenGL callback for reshaping window
 */
void reshapeCallback( int new_width, int new_height )
{

    if( new_height == 0 )
        new_height = 1;
    float aspect_ratio = new_width / ( float )new_height;

    glMatrixMode( GL_PROJECTION );
    glViewport( 0, 0, new_width, new_height );
    glLoadIdentity();
    gluPerspective( 45.0f, aspect_ratio, 0.1f, 5000.0f );

    // glutPostRedisplay();
}



/*
 * OpenGL callback for key press
 */
void keyboardCallback( unsigned char key, int x, int y )
{
    static bool enableWireMesh = false;

    switch( key ) {
        case 27 /* ESC */:
        case 'q' :
            exit( 0 );
            break;
        case 'd' /*Depth Image*/ :
        case 'c' /*Color Image*/ :
            screenshot( key );
            break;
        case 'w' :
            enableWireMesh = !enableWireMesh;
            if( enableWireMesh )	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
            else					glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
            break;
        case 'a' :
            enableAxes = !enableAxes;
            break;
        default :
            return;
    }

    // glutPostRedisplay();
}



/*
 * OpenGL callback for holding the key
 */
void keyboardSpecialDownCallback( int key, int x, int y )
{
    switch( key ) {
    case GLUT_KEY_UP :
        cameraMoveFront = 1;
        break;
    case GLUT_KEY_DOWN :
        cameraMoveFront = -1;
        break;
    case GLUT_KEY_LEFT :
        cameraMoveRight = 1;
        break;
    case GLUT_KEY_RIGHT :
        cameraMoveRight = -1;
        break;
    case GLUT_KEY_PAGE_UP :
        cameraMoveUp = 1;
        break;
    case GLUT_KEY_PAGE_DOWN :
        cameraMoveUp = -1;
        break;
    default :
        return;
    }

    // glutPostRedisplay();
}



/*
 * OpenGL callback for key press
 */
void keyboardSpecialUpCallback( int key, int x, int y )
{
    switch( key ) {
    case GLUT_KEY_UP :
    case GLUT_KEY_DOWN :
        cameraMoveFront = 0;
        break;
    case GLUT_KEY_LEFT :
    case GLUT_KEY_RIGHT :
        cameraMoveRight = 0;
        break;
    case GLUT_KEY_PAGE_UP :
    case GLUT_KEY_PAGE_DOWN :
        cameraMoveUp = 0;
        break;
    default :
        return;
    }

    // glutPostRedisplay();
}



/*
 * OpenGL callback for mouse button press
 */
void mouseButtonCallback( int button, int state, int x, int y )
{

    if( button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN ) {

        unsigned char colorPixels[ 1 * 1 * 4 ];
        glReadPixels( x, glutGet( GLUT_WINDOW_HEIGHT ) - y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE,
                      &colorPixels[0] );

        float depthPixels[ 1 * 1 ];
        glReadPixels( x, glutGet( GLUT_WINDOW_HEIGHT ) - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT,
                      &depthPixels[0] );

        GLint viewport[4];
        GLdouble modelView[17];
        GLdouble projection[17];
        GLfloat winX, winY, winZ;
        GLdouble worldX, worldY, worldZ;

        glGetDoublev( GL_MODELVIEW_MATRIX, modelView );
        glGetDoublev( GL_PROJECTION_MATRIX, projection );
        glGetIntegerv( GL_VIEWPORT, viewport );

        winX = float( x );
        winY = float( glutGet( GLUT_WINDOW_HEIGHT ) - y );
        winZ = depthPixels[0];

        gluUnProject( winX, winY, winZ,
                      modelView, projection, viewport,
                      &worldX, &worldY, &worldZ );

        std::cout << std::endl;
        std::cout << "RGBA : (" << ( int )colorPixels[0] << ", " << ( int )colorPixels[1] << ", "
                  << ( int )colorPixels[2] << ", " << ( int )colorPixels[3] << ")" << std::endl;
        // std::cout << "Depth : " << depthPixels[0] << std::endl;
        // std::cout << "Image coordinates : ( " << winX << ", " << winY << ", " << winZ << " )"
        // 		  << std::endl;
        std::cout << "World Coordinates : ( " << worldX << ", " << worldY << ", " << worldZ << " )"
                  << std::endl;
    } else if( button == GLUT_LEFT_BUTTON ) {
        if( state == GLUT_DOWN ) {
            lastMousePosX = x;
            lastMousePosY = y;
        } else { // state == GLUT_UP
            lastMousePosX = -1;
            lastMousePosY = -1;
            yaw += deltaYaw;
            pitch += deltaPitch;
            deltaYaw = 0;
            deltaPitch = 0;
        }
    }

    // glutPostRedisplay();
}



/*
 * OpenGL callback for mouse movements
 */
void mouseMotionCallback( int x, int y )
{
    if( lastMousePosX >= 0 ) {
        deltaYaw = ( lastMousePosX - x ) * mouseSensetivity;
        deltaPitch = ( y - lastMousePosY ) * mouseSensetivity;
    }

    // glutPostRedisplay();
}



#endif //HUMAN_BODY_RECONSTRUCTION_GL_OPERATIONS_HPP
