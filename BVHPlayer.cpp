/* CS3242 3D Modeling and Animation
 * Programming Assignment II
 * School of Computing
 * National University of Singapore
 */
 
#ifdef  WIN32
	#include <windows.h>
#endif

#include "BVHAnimator.h"

//#define TEST_IK

#include <GL/glut.h>

#include <ctime>

static float   camera_yaw = 0.0f;      
static float   camera_pitch = -20.0f;  
static float   camera_distance = 5.0f; 

static int     drag_mouse_r = 0; 
static int     drag_mouse_l = 0; 
static int     drag_mouse_m = 0; 
static int     last_mouse_x, last_mouse_y; 

// 0 - joints via OpenGL, 1 - joints via GLM matrix, 2 - joints via GLM quaternion
// 3 - skeleton, 4 - mannequin
static int animation_flag = 0; 
static char *animation_flag_string[] = {
    "Joint - OpenGL",
    "Joint - matrix",
    "Joint - quaternion",
    "Skeleton",
    "Mannequin"
};

static int win_width, win_height;

bool   on_animation = true;

float  animation_time = 0.0f;

float  animation_scale = 0.03f, animation_scale_step = 0.005f, animation_scale_max = 0.05f, animation_scale_min = 0.01f;

#ifdef TEST_IK
// initial target position
float target_x = -0.1, target_y = 1.4, target_z = 0.4;
#endif

// clock controling animation
clock_t ani_start, ani_time;

static int frame_no = 0;

BVH *bvh = NULL;
BVHAnimator *bvhani = NULL;

static void drawString(void *font, const char *str, float x, float y) {
    int len = strlen(str);
    glRasterPos2f(x, y);
    for (int i = 0; i < len; ++i) {
        glutBitmapCharacter(font, str[i]);  // this function automatically
	    			                        // advances raster position
    }
}

static void drawMessage(int line_no, char messages[][64], int num_messages)
{	
	if ( messages == NULL || num_messages <= 0)
		return;

	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0.0, win_width, win_height, 0.0 );

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	glColor3f( 0.0, 1.0, 0.0 );
    int y = 24 + 18 * line_no;
    for (int i = 0; i < num_messages; ++i) {
        drawString(GLUT_BITMAP_HELVETICA_18, messages[i], 8, y);
        y += 18;
    }

	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();
}


static void display(void)
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

    glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glTranslatef( 0.0, 0.0, - camera_distance );
	glRotatef( - camera_pitch, 1.0, 0.0, 0.0 );
	glRotatef( - camera_yaw, 0.0, 1.0, 0.0 );

	float  light0_position[] = { 10.0, 10.0, 10.0, 1.0 };
	glLightfv( GL_LIGHT0, GL_POSITION, light0_position );

	float  size = 1.5f;
	int  num_x = 10, num_z = 10;
	double  ox, oz;

	// the tiled floor
	glBegin( GL_QUADS );
		glNormal3d( 0.0, 1.0, 0.0 );
		ox = -(num_x * size) / 2;
		for ( int x=0; x<num_x; x++, ox+=size )
		{
			oz = -(num_z * size) / 2;
			for ( int z=0; z<num_z; z++, oz+=size )
			{
				if ( ((x + z) % 2) == 0 )
					glColor3f( 1.0, 1.0, 1.0 );
				else
					glColor3f( 0.7, 0.7, 0.7 );
				glVertex3d( ox, 0.0, oz );
				glVertex3d( ox, 0.0, oz+size );
				glVertex3d( ox+size, 0.0, oz+size );
				glVertex3d( ox+size, 0.0, oz );
			}
		}
	glEnd();

#ifdef TEST_IK
	// the target
	if (bvhani){
		static GLUquadricObj *quad_obj = gluNewQuadric();
		glPushMatrix();		
		glTranslated( target_x, target_y, target_z );
		glColor3f( 1.0f, 0.788f, 0.055f );
		gluSphere(quad_obj, 2*animation_scale, 16, 8);	
		glPopMatrix();
	}
#endif

	glColor3f( 1.0f, 0.0f, 0.0f );
	if ( bvhani )
		bvhani->renderFigure(frame_no, animation_scale, animation_flag);				

	char messages[3][64];
	if ( bvhani ) {
		sprintf( messages[0], "Time  : %.2fs", animation_time );
        sprintf( messages[1], "Frame : %d", frame_no );        
        sprintf( messages[2], "Animation : %s", animation_flag_string[animation_flag] );
        drawMessage(0, messages, 3);
    } else {
		sprintf( messages[0], "Press L key to load a BVH file" );
        drawMessage(0, messages, 1);
    }    
    

    glutSwapBuffers();
}


static void reshape(int w, int h)
{
    glViewport(0, 0, w, h);
	
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( 45, (double)w/h, 1, 500 );

	win_width = w;
	win_height = h;
}


static void mouse(int button, int state, int mx, int my)
{
	if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_l = 1;
	else if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_l = 0;

	if ( ( button == GLUT_RIGHT_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_r = 1;
	else if ( ( button == GLUT_RIGHT_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_r = 0;

	if ( ( button == GLUT_MIDDLE_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_m = 1;
	else if ( ( button == GLUT_MIDDLE_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_m = 0;

	glutPostRedisplay();

	last_mouse_x = mx;
	last_mouse_y = my;
}


static void motion(int mx, int my)
{
	if ( drag_mouse_l )
	{
		camera_yaw -= ( mx - last_mouse_x ) * 1.0;
		if ( camera_yaw < 0.0 )
			camera_yaw += 360.0;
		else if ( camera_yaw > 360.0 )
			camera_yaw -= 360.0;

		camera_pitch -= ( my - last_mouse_y ) * 1.0;
		if ( camera_pitch < -90.0 )
			camera_pitch = -90.0;
		else if ( camera_pitch > 90.0 )
			camera_pitch = 90.0;
	}
	if ( drag_mouse_r )
	{
		camera_distance += ( my - last_mouse_y ) * 0.2;
		if ( camera_distance < 2.0 )
			camera_distance = 2.0;
	}

	last_mouse_x = mx;
	last_mouse_y = my;

	glutPostRedisplay();
}


static void keyboard(unsigned char key, int mx, int my)
{
	// change animation mode
	if ( key == 'a' ) {
		animation_flag = (animation_flag + 1) % 5;		
	}
	if ( key == 'A' ) {
		animation_flag = (animation_flag + 4) % 5;
	}

	// change animation scale
	if ( key == '_' || key == '-' ){
		animation_scale -= animation_scale_step;
		if(animation_scale<animation_scale_min) animation_scale = animation_scale_min;
	}
	if ( key == '+' || key == '=' ){
		animation_scale += animation_scale_step;
		if(animation_scale>animation_scale_max) animation_scale = animation_scale_max;
	}

	if ( key == 'l' || key == 'L' )
	{
#ifdef  WIN32
		const int  file_name_len = 256;
		char  file_name[ file_name_len ] = "";
		OPENFILENAME	open_file;
		memset( &open_file, 0, sizeof(OPENFILENAME) );
		open_file.lStructSize = sizeof(OPENFILENAME);
		open_file.hwndOwner = NULL;
		open_file.lpstrFilter = "BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
		open_file.nFilterIndex = 1;
		open_file.lpstrFile = file_name;
		open_file.nMaxFile = file_name_len;
		open_file.lpstrTitle = "Select a BVH file";
		open_file.lpstrDefExt = "bvh";
		open_file.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;

		BOOL  ret = GetOpenFileName( &open_file );

		if( ret )
		{
			if ( bvh )
				delete  bvh;
			if ( bvhani)
				delete bvhani;
			bvh = new BVH(file_name);
			if ( !bvh->IsLoadSuccess() )
			{
				delete  bvh;
				bvh = NULL;
				delete bvhani;
				bvhani = NULL;
			}
			else{
				bvhani = new BVHAnimator(bvh);				
			}
			animation_time = 0.0f;
			frame_no = 0;	
			ani_start = clock();
			ani_time = ani_start;
		}
#endif
#ifdef TEST_IK
		if(bvh){
			bvhani->solveLeftArm(frame_no,animation_scale,target_x,target_y,target_z);
		}
#endif
	}

	glutPostRedisplay();
}

#ifdef TEST_IK
static void skeyboard(int key, int x, int y)
{
	//change animation mode
	if ( key == GLUT_KEY_LEFT ){
		target_x -= 0.1;
	}
	if ( key == GLUT_KEY_RIGHT ){
		target_x += 0.1;
	}
	if ( key == GLUT_KEY_UP ){
		target_y += 0.1;
	}
	if ( key == GLUT_KEY_DOWN ){
		target_y -= 0.1;
	}
	if ( key == GLUT_KEY_PAGE_UP ){
		target_z -= 0.1;
	}
	if ( key == GLUT_KEY_PAGE_DOWN ){
		target_z += 0.1;
	}

	if(bvh){
		bvhani->solveLeftArm(frame_no,animation_scale,target_x,target_y,target_z);
	}
	glutPostRedisplay();
}
#endif

static void idle(void)
{
	if ( on_animation )
	{
		ani_time = clock();

		// convert to second
		animation_time = (ani_time-ani_start) / (float)CLOCKS_PER_SEC;

#ifdef TEST_IK	

		frame_no = 0;

#else
        // -----------------------------------
        // TODO: [Part 1 - Animation Basics]
        // -----------------------------------

        if ( bvh )
		{
                        			
            frame_no = 0;

		}
		else
			frame_no = 0;

#endif

		glutPostRedisplay();
	}
}


static void initEnvironment(void)
{
	float  light0_position[] = { 10.0, 10.0, 10.0, 1.0 };
	float  light0_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
	float  light0_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	float  light0_ambient[] = { 0.1, 0.1, 0.1, 1.0 };
	glLightfv( GL_LIGHT0, GL_POSITION, light0_position );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, light0_diffuse );
	glLightfv( GL_LIGHT0, GL_SPECULAR, light0_specular );
	glLightfv( GL_LIGHT0, GL_AMBIENT, light0_ambient );
	glEnable( GL_LIGHT0 );

	glEnable( GL_LIGHTING );

	glEnable( GL_COLOR_MATERIAL );

	glEnable( GL_DEPTH_TEST );

	glCullFace( GL_BACK );
	glDisable( GL_CULL_FACE );

	// sky color
	glClearColor( 0.62, 0.77, 0.85, 0.0 );

	ani_start = clock();
}


int main(int argc, char ** argv)
{

    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL );
	glutInitWindowSize( 640, 640 );
	glutInitWindowPosition( 0, 0 );
    glutCreateWindow("Forward and Inverse Kinematics");
	
    glutDisplayFunc( display );
    glutReshapeFunc( reshape );
	glutMouseFunc( mouse );
	glutMotionFunc( motion );
	glutKeyboardFunc( keyboard );
#ifdef TEST_IK
	glutSpecialFunc( skeyboard );
#endif
	glutIdleFunc( idle );

	initEnvironment();

    glutMainLoop();
    return 0;
}
