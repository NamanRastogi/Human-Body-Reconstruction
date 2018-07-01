#define ASSERT(condition, message)											\
	if (!(condition))														\
	{																		\
		cerr << "Assertion `" #condition "` failed in " << __FILE__ 		\
				  << " line " << __LINE__ << ": " << message << endl;		\
		terminate();														\
	}

#define ERROR(message)														\
	{																		\
		cerr << "ERROR [" << __FUNCTION__ << "] : " << message << endl;		\
		terminate();														\
	}

#define WARNING(message) 													\
	cerr << "WARNING [" << __FUNCTION__ << "] : " << message << endl;


#define GLM_ENABLE_EXPERIMENTAL


#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <random>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/euler_angles.hpp>

using namespace std;

string DATA_DIR;

bool enableWireMesh = false;
int mesh_display_type = 0;

struct Mesh {
	int numFaces;
	int numVertices;
	int numBones;
	vector< glm::vec3 > Faces;
	vector< glm::vec3 > Vertices;
	glm::vec3 pCorner, nCorner;
	vector< unsigned int > bone;
	vector< glm::vec3 > boneColor;
};
Mesh* M = new Mesh();

void loadPLY( string fileName, glm::mat4 Transform = glm::mat4( 1.0f ) );
bool inFirstQuad( glm::vec3 );
void draw_grid( glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4,
				glm::vec4 color = glm::vec4( 1.0, 1.0, 1.0, 1.0 ), unsigned int divisions = 100 );
void attach_vertices_with_bone( int new_bone_no );
void save_skeleton( string skeleton_filename );
void renderBitmapString( glm::vec3 pos, void *font, string s );
void load_skeletonParts( string filename );

glm::mat3 R( 1.0 );
glm::vec3 T( 0.0 );

glm::vec3 cameraPosition( 0.0, 0.0, 25.0 );
glm::vec3 cameraDirection, cameraRightDirection;

glm::vec3 upDirection( 0.0, 1.0, 0.0 );
float cameraMoveFront = 0.0, cameraMoveRight = 0.0, cameraMoveUp = 0.0;

double yaw = -90.0, pitch = 0.0;
float deltaYaw, deltaPitch;
int lastMousePosX, lastMousePosY;

float cameraSpeed = 0.5;
float mouseSensetivity = 0.2;

bool enableAxes = true;
bool enablePlanes = true;

bool KEY_CTRL_ACTIVE = false;
bool KEY_ALT_ACTIVE = false;

void displayCallback();
void reshapeCallback( int new_width, int new_height );
void keyboardCallback( unsigned char key, int x, int y );
void keyboardSpecialDownCallback( int key, int x, int y );
void keyboardSpecialUpCallback( int key, int x, int y );
void mouseButtonCallback( int button, int state, int x, int y );
void mouseMotionCallback( int x, int y );


int main( int argc, char* argv[] )
{
	if( argc < 3 ) {
		cout << "Usage: " << argv[0] << " <DATA DIR> <initial #bones>" << endl;
		exit( EXIT_SUCCESS );
	}

	DATA_DIR = argv[1];

	string filename_mesh = DATA_DIR + "/Mesh_POSE/0.ply";
	string filename_skeleton = DATA_DIR + "/skeletonParts.dat";

	M->numBones = stoi( argv[2] );
	loadPLY( filename_mesh );

	cout << "Bounding Box :" << endl;
	cout << "    Corner 1 : " << M->nCorner.x << " " << M->nCorner.y << " " << M->nCorner.z
		 << endl;
	cout << "    Corner 2 : " << M->pCorner.x << " " << M->pCorner.y << " " << M->pCorner.z
		 << endl;

	// cameraPosition = glm::vec3(M->pCorner.x + (M->pCorner.x - M->nCorner.x) / 2,
	// 						   M->pCorner.y + (M->pCorner.y - M->nCorner.y) / 2,
	// 						   M->pCorner.z + (M->pCorner.z - M->nCorner.z) / 2);

	float distX = M->pCorner.x - M->nCorner.x,
	      distY = M->pCorner.y - M->nCorner.y,
	      distZ = M->pCorner.z - M->nCorner.z,
	      distXZ = glm::sqrt( pow( distX, 2 ) + pow( distZ, 2 ) ),
	      distXYZ = glm::sqrt( pow( distXZ, 2 ) + pow( distY, 2 ) );
	// pitch = glm::degrees(-(glm::atan(distY, distXZ)));
	// yaw = 180 + glm::degrees(glm::atan(distZ, distX));

	cout << endl;
	cout << "Camera Initial Position : " << cameraPosition.x << " " << cameraPosition.y << " "
		 << cameraPosition.z << endl;

	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE );
	// glutInitWindowPosition( 10, 10 );
	glutInitWindowSize( 800, 600 );
	// glutInitContextVersion( 2, 0 );
	// glutInitContextFlags( GLUT_FORWARD_COMPATIBLE | GLUT_DEBUG );
	glutCreateWindow( "Human Body Reconstruction" );

	glEnable( GLUT_MULTISAMPLE );
	glEnable( GL_DEPTH_TEST );

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

void loadPLY( string fileName, glm::mat4 Transform )
{
	ifstream fin( fileName );
	if( !fin.good() ) {
		ERROR( "Unable to read file " + fileName );
	} // if( !fin.good() )

	string inputString;
	fin >> inputString;
	while( inputString.compare( "end_header" ) != 0 ) {

		fin >> inputString;
		if( inputString.compare( "element" ) == 0 ) {
			fin >> inputString;
			if( inputString.compare( "vertex" ) == 0 ) {
				fin >> M->numVertices;
				//cout << "No of Vertices : " << M->numVertices << endl;
			} else if( inputString.compare( "face" ) == 0 ) {
				fin >> M->numFaces;
				//cout << "No of Faces : " << M->numFaces << endl;
			}
		}
	}

	double x, y, z;
	M->Vertices.clear();
	for( long int i = 0; i < M->numVertices; ++i ) {
		fin >> x >> y >> z;
		//			M->Vertices.emplace_back(x, y, z);
		glm::vec4 v4( x, y, z, 1.0 );
		M->Vertices.push_back( glm::vec3( Transform * v4 ) );
	}
	long int faceSize;
	M->Faces.clear();
	for( long int i = 0; i < M->numFaces; ++i ) {
		fin >> faceSize;
		fin >> x >> y >> z;
		ASSERT( faceSize == 3, " faceSize:" << faceSize << " (x,y,z):"
		        << "(" << x << "," << y << "," << z << ")" );
		M->Faces.emplace_back( x, y, z );
	}

	fin.close();

	M->pCorner.x = M->Vertices[0].x;
	M->pCorner.y = M->Vertices[0].y;
	M->pCorner.z = M->Vertices[0].z;
	M->nCorner.x = M->Vertices[0].x;
	M->nCorner.y = M->Vertices[0].y;
	M->nCorner.z = M->Vertices[0].z;
	for( int i = 1; i < M->numVertices; ++i ) {
		M->pCorner.x = max( M->pCorner.x, M->Vertices[i].x );
		M->pCorner.y = max( M->pCorner.y, M->Vertices[i].y );
		M->pCorner.z = max( M->pCorner.z, M->Vertices[i].z );
		M->nCorner.x = min( M->nCorner.x, M->Vertices[i].x );
		M->nCorner.y = min( M->nCorner.y, M->Vertices[i].y );
		M->nCorner.z = min( M->nCorner.z, M->Vertices[i].z );
	}

	M->bone.resize( M->numVertices, 0 );

	M->boneColor.resize( M->numBones );
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution( 0.0, 1.0 );
	for( int i = 0; i < M->numBones; ++i ) {
		M->boneColor[i].x = distribution( generator );
		M->boneColor[i].y = distribution( generator );
		M->boneColor[i].z = distribution( generator );
	}
}

void load_skeletonParts( string filename )
{
	ifstream fin( filename );
	if( !fin.good() ) {
		WARNING( "Unable to read file " + filename );
		return;
	} // if( !fin.good() )

	int num_verts, num_bones;
	fin >> num_verts >> num_bones;
	if( num_verts != M->numVertices )
		ERROR( "skeletonParts.dat has " << num_verts << " vertices instead of " << M->numVertices
			   << endl );
	M->numBones = num_bones;

	for( unsigned int i = 0; i < M->numVertices; ++i )
		fin >> M->bone[i];

	fin.close();

	M->boneColor.resize( M->numBones );
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution( 0.0, 1.0 );
	for( int i = 0; i < M->numBones; ++i ) {
		M->boneColor[i].x = distribution( generator );
		M->boneColor[i].y = distribution( generator );
		M->boneColor[i].z = distribution( generator );
	}

	cout << "Skeleton loaded from " << filename << endl;
}

void save_skeleton( string filename )
{
	ofstream fout( filename, ios::trunc );
	fout << M->numVertices << " " << M->numBones << endl;
	for( auto i = M->bone.begin(); i != M->bone.end(); ++i )
		fout << *i << " ";
	fout.close();

	cout << "Skeleton saved in " << filename << endl;
}

bool inFirstQuad( glm::vec3 A )
{
	if( A.x > 0 && A.y > 0 && A.z > 0 )
		return true;
	return false;
}

void draw_grid( glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4, glm::vec4 color,
				unsigned int divisions )
{
	glm::vec3 a, b;
	float m;

	glBegin( GL_LINES );
	glColor4f( color.x, color.y, color.z, color.w );

	// Lines in Direction 1
	for( unsigned int i = 0; i <= divisions; i++ ) {
		m = float( i ) / divisions;
		a = ( p1 * m ) + ( p2 * ( 1 - m ) );
		b = ( p4 * m ) + ( p3 * ( 1 - m ) );
		glVertex3f( a.x, a.y, a.z );
		glVertex3f( b.x, b.y, b.z );
	}

	// Lines in Direction 2
	for( unsigned int i = 0; i <= divisions; i++ ) {
		m = float( i ) / divisions;
		a = ( p1 * m ) + ( p4 * ( 1 - m ) );
		b = ( p2 * m ) + ( p3 * ( 1 - m ) );
		glVertex3f( a.x, a.y, a.z );
		glVertex3f( b.x, b.y, b.z );
	}

	glEnd();
}

void attach_vertices_with_bone( int new_bone_no )
{
	for( unsigned int i = 0; i < M->numVertices; i++ ) {
		if( inFirstQuad( ( R * M->Vertices[i] ) + T ) ) {
			M->bone[i] = new_bone_no;
		}
	}
}

void renderBitmapString( glm::vec3 pos, void *font, string s )
{
	/*
	 * Font Options:
	 * -------------
	 *
	 * GLUT_BITMAP_8_BY_13
	 * GLUT_BITMAP_9_BY_15
	 * GLUT_BITMAP_TIMES_ROMAN_10
	 * GLUT_BITMAP_TIMES_ROMAN_24
	 * GLUT_BITMAP_HELVETICA_10
	 * GLUT_BITMAP_HELVETICA_12
	 * GLUT_BITMAP_HELVETICA_18
	*/
	glRasterPos3f( pos.x, pos.y, pos.z );
	for( int i = 0; i < s.size(); ++i ) {
		glutBitmapCharacter( font, s[i] );
	}
}

void displayCallback()
{

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
	float axis_size = 15.0;
	if( enableAxes ) {
		glBegin( GL_LINES );
		glColor3f( 1.0f, 0.0f, 0.0f );
		glVertex3f( -axis_size, 0.0f, 0.0f );
		glVertex3f( axis_size, 0.0f, 0.0f );
		glColor3f( 0.0f, 1.0f, 0.0f );
		glVertex3f( 0.0f, axis_size, 0.0f );
		glVertex3f( 0.0f, axis_size, 0.0f );
		glColor3f( 0.0f, 0.0f, 1.0f );
		glVertex3f( 0.0f, 0.0f, axis_size );
		glVertex3f( 0.0f, 0.0f, axis_size );
		glEnd();
	}

	float plane_size = 15.0;
	unsigned int grid_density = 100;
	glm::vec4 plane_color( 0.0, 1.0, 1.0, 0.5 );
	if( enablePlanes ) {
		glm::vec3 o( 0.0, 0.0, 0.0 );
		glm::vec3 x( plane_size, 0.0, 0.0 );
		glm::vec3 y( 0.0, plane_size, 0.0 );
		glm::vec3 z( 0.0, 0.0, plane_size );
		glm::vec3 xy( plane_size, plane_size, 0.0 );
		glm::vec3 yz( 0.0, plane_size, plane_size );
		glm::vec3 zx( plane_size, 0.0, plane_size );
		draw_grid( x, o, y, xy, plane_color, grid_density );
		draw_grid( y, o, z, yz, plane_color, grid_density );
		draw_grid( x, o, z, zx, plane_color, grid_density );
	}

	float positive_opacity = 1.0;
	float negative_opacity = 0.5;

	if( mesh_display_type == 0 ) {

		glColor3f( 1.0, 1.0, 1.0 );
		glBegin( GL_TRIANGLES );
		for( int i = 0; i < M->numFaces; ++i ) {
			int a = M->Faces[i].x;
			int b = M->Faces[i].y;
			int c = M->Faces[i].z;
			glm::vec3 A = ( R * M->Vertices[a] ) + T;
			glm::vec3 B = ( R * M->Vertices[b] ) + T;
			glm::vec3 C = ( R * M->Vertices[c] ) + T;

			if( inFirstQuad( A ) )
				glColor4f( 1.0, 1.0, 1.0, positive_opacity );
			else
				glColor4f( M->boneColor[M->bone[a]].x, M->boneColor[M->bone[a]].y,
						   M->boneColor[M->bone[a]].z, positive_opacity );
			glVertex3f( A.x, A.y, A.z );

			if( inFirstQuad( B ) )
				glColor4f( 1.0, 1.0, 1.0, positive_opacity );
			else
				glColor4f( M->boneColor[M->bone[b]].x, M->boneColor[M->bone[b]].y,
						   M->boneColor[M->bone[b]].z, positive_opacity );
			glVertex3f( B.x, B.y, B.z );

			if( inFirstQuad( C ) )
				glColor4f( 1.0, 1.0, 1.0, positive_opacity );
			else
				glColor4f( M->boneColor[M->bone[c]].x, M->boneColor[M->bone[c]].y,
						   M->boneColor[M->bone[c]].z, positive_opacity );
			glVertex3f( C.x, C.y, C.z );
		}
		glEnd();
	} else {

		for( int i = 0; i < M->numVertices; ++i ) {
			glm::vec3 A = ( R * M->Vertices[i] ) + T;
			unsigned int bone_idx = M->bone[i];

			if( inFirstQuad( A ) )
				glColor4f( 1.0, 1.0, 1.0, positive_opacity );
			else
				glColor4f( M->boneColor[bone_idx].x, M->boneColor[bone_idx].y,
						   M->boneColor[bone_idx].z, negative_opacity );
			glTranslatef( A.x, A.y, A.z );
			glutSolidSphere( 0.03, 7, 5 );
			glTranslatef( -A.x, -A.y, -A.z );
		}
	}

	vector< glm::vec3 > center( M->numBones, glm::vec3( 0.0 ) );
	vector< int > num_pts( M->numBones, 0 );
	for( int i = 0; i < M->numVertices; ++i ) {
		glm::vec3 A = ( R * M->Vertices[i] ) + T;
		unsigned int bone_idx = M->bone[i];

		++num_pts[bone_idx];
		center[bone_idx] += A;
	}
	for( int i = 0; i < M->numBones; ++i ) {
		center[i] /= num_pts[i];
		renderBitmapString( center[i] + glm::vec3(0.0, 0.0, 0.0f), GLUT_BITMAP_TIMES_ROMAN_24, to_string( i ) );
	}

	glutSwapBuffers();
}

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

void keyboardCallback( unsigned char key, int x, int y )
{
	string skeleton_filename = DATA_DIR + "/skeletonParts.dat";
	int new_bone_no;

	switch( key ) {
	case 27 /* ESC */:
	case 'q':
		exit( 0 );
		break;
	case 'e' :
		enableWireMesh = !enableWireMesh;
		if( enableWireMesh )
			glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		else
			glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		break;
	case 'a':
		enableAxes = !enableAxes;
		break;
	case 'p' :
		enablePlanes = !enablePlanes;
		break;
	case 'm' :
		mesh_display_type = ( mesh_display_type + 1 ) % 2;
		break;
	case 'r' :
		T = glm::vec3( 0.0 );
		R = glm::mat3( 1.0 );
		break;
	case 'l' :
		// cout << "Enter skeleton file: ";
		// cin >> filename;
		load_skeletonParts( skeleton_filename );
		break;
	case 's' :
		cout << "Enter bone no: ";
		cin >> new_bone_no;
		if( new_bone_no < 0 )
			WARNING( "Entered no. should be non-negative" << endl );
		attach_vertices_with_bone( new_bone_no );
		break;
	case 'w' :
		save_skeleton( skeleton_filename );
		break;
	default:
		return;
	}

	// glutPostRedisplay();
}

void keyboardSpecialDownCallback( int key, int x, int y )
{
	switch( key ) {
	case GLUT_KEY_UP:
		if( KEY_CTRL_ACTIVE )
			T.z -= 0.1;
		else if( KEY_ALT_ACTIVE )
			R = glm::mat3( glm::eulerAngleX( glm::radians( -5.0 ) ) ) * R;
		else
			cameraMoveFront = 1;
		break;
	case GLUT_KEY_DOWN:
		if( KEY_CTRL_ACTIVE )
			T.z += 0.1;
		else if( KEY_ALT_ACTIVE )
			R = glm::mat3( glm::eulerAngleX( glm::radians( 5.0 ) ) ) * R;
		else
			cameraMoveFront = -1;
		break;
	case GLUT_KEY_LEFT:
		if( KEY_CTRL_ACTIVE )
			T.x -= 0.1;
		else if( KEY_ALT_ACTIVE )
			R = glm::mat3( glm::eulerAngleY( glm::radians( 5.0 ) ) ) * R;
		else
			cameraMoveRight = 1;
		break;
	case GLUT_KEY_RIGHT:
		if( KEY_CTRL_ACTIVE )
			T.x += 0.1;
		else if( KEY_ALT_ACTIVE )
			R = glm::mat3( glm::eulerAngleY( glm::radians( -5.0 ) ) ) * R;
		else
			cameraMoveRight = -1;
		break;
	case GLUT_KEY_PAGE_UP:
		if( KEY_CTRL_ACTIVE )
			T.y += 0.1;
		else if( KEY_ALT_ACTIVE )
			R = glm::mat3( glm::eulerAngleZ( glm::radians( 5.0 ) ) ) * R;
		else
			cameraMoveUp = 1;
		break;
	case GLUT_KEY_PAGE_DOWN:
		if( KEY_CTRL_ACTIVE )
			T.y -= 0.1;
		else if( KEY_ALT_ACTIVE )
			R = glm::mat3( glm::eulerAngleZ( glm::radians( -5.0 ) ) ) * R;
		else
			cameraMoveUp = -1;
		break;
	case GLUT_KEY_CTRL_L:
		KEY_CTRL_ACTIVE = true;
		break;
	case GLUT_KEY_ALT_L:
		KEY_ALT_ACTIVE = true;
		break;
	default:
		return;
	}

	// glutPostRedisplay();
}

void keyboardSpecialUpCallback( int key, int x, int y )
{
	switch( key ) {
	case GLUT_KEY_UP:
	case GLUT_KEY_DOWN:
		cameraMoveFront = 0;
		break;
	case GLUT_KEY_LEFT:
	case GLUT_KEY_RIGHT:
		cameraMoveRight = 0;
		break;
	case GLUT_KEY_PAGE_UP:
	case GLUT_KEY_PAGE_DOWN:
		cameraMoveUp = 0;
		break;
	case GLUT_KEY_CTRL_L:
		KEY_CTRL_ACTIVE = false;
		break;
	case GLUT_KEY_ALT_L:
		KEY_ALT_ACTIVE = false;
		break;
	default:
		return;
	}

	// glutPostRedisplay();
}

void mouseButtonCallback( int button, int state, int x, int y )
{
	if( button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN ) {

		unsigned char colorPixels[1 * 1 * 4];
		glReadPixels( x, glutGet( GLUT_WINDOW_HEIGHT ) - y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE,
				 	  &colorPixels[0] );

		float depthPixels[1 * 1];
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

		cout << endl;
		cout << "RGBA : (" << ( int )colorPixels[0] << ", " << ( int )colorPixels[1] << ", "
			 << ( int )colorPixels[2] << ", " << ( int )colorPixels[3] << ")" << endl;
		// cout << "Depth : " << depthPixels[0] << endl;
		// cout << "Image coordinates : ( " << winX << ", " << winY << ", " << winZ << " )" << endl;
		cout << "World Coordinates : ( " << worldX << ", " << worldY << ", " << worldZ << " )"
			 << endl;
	} else if( button == GLUT_LEFT_BUTTON ) {
		if( state == GLUT_DOWN ) {
			lastMousePosX = x;
			lastMousePosY = y;
		} else {
			// state == GLUT_UP
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

void mouseMotionCallback( int x, int y )
{
	if( lastMousePosX >= 0 ) {
		deltaYaw = ( lastMousePosX - x ) * mouseSensetivity;
		deltaPitch = ( y - lastMousePosY ) * mouseSensetivity;
	}

	// glutPostRedisplay();
}
