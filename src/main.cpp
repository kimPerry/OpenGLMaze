#include <GL/glew.h> // glew must be included before the main gl libs
#include <GL/glut.h> // doing otherwise causes compiler shouting
#include <iostream>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h>
#include <bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <bullet/BulletCollision/CollisionShapes/btShapeHull.h>
#include <unistd.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp> //Makes passing matrices to shaders easier

//--Data types
//This object will define the attributes of a vertex(position, color, etc...)
struct Vertex
{
    GLfloat position[3];
    GLfloat color[3];
    GLfloat vnormal[3];
    GLfloat vt[2];
};

//This object will hold rgb values for materials
struct Material
{
    char matName[20];
    GLfloat rgb[3];
};

//--Evil Global variables
//Just for this example!


const int NUM_V = 5000;
int w = 1440, h = 1080;// Window size
int currentX = 0;
int currentY = 0;
Vertex board[NUM_V];
btTriangleMesh * boardMesh = new btTriangleMesh();
btTriangleMesh * ballMesh = new btTriangleMesh();
btTriangleMesh * horseMesh = new btTriangleMesh();
btTransform boardTransform;
Vertex ball[3000];
Vertex horse[2000];
GLuint program;// The GLSL program handle
GLuint vbo_geometry;// VBO handle for our geometry
GLuint vbo_ball;
GLuint vbo_horse;
//uniform locations
GLint loc_mvpmat;// Location of the modelviewprojection matrix in the shader
GLint loc_mvmat;// Location of the modelview matrix in the shader
GLint loc_normmat;
GLint myText;

//attribute locations
GLint loc_position;
float loc_color;
float loc_normal;
float loc_vt;
float oldX;
float oldY;
float oldZ;
int x = 0;
bool gamePaused = false;

btRigidBody * ballRigidBody;
btDiscreteDynamicsWorld* dynamicsWorld;
btTransform trans;
btRigidBody* groundRigidBody;
btCollisionShape* groundShape;
btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;    
//transform matrices
glm::mat4 boardModel;//obj->world each object should have its own model matrix
glm::mat4 ballModel;
glm::mat4 horseModel;
glm::mat4 meshModel;
glm::mat4 view;//world->eye
glm::mat4 projection;//eye->clip
glm::mat4 mvp;//premultiplied modelviewprojection
glm::mat4 mv;
glm::mat4 norm;


//--GLUT Callbacks
void render();
void renderModel(int model, int vbo, int vertices);
void update();
void reshape(int n_w, int n_h);
void keyboard(unsigned char key, int x_pos, int y_pos);
void motion (int x, int y);
void timerFunc(int x);
void menu(int id);
void pauseGame();
void DemoLight();
GLuint loadBMP_custom(const char * imagepath);
void meh();

btAlignedObjectArray<btCollisionShape*> m_CollisionShapes;

//--Resource management
bool initialize();
void cleanUp();

//text file reader
char *textFileRead(char *fn);

// model loader
bool loadOBJ(char * obj, char * mtl, Vertex geometry[], btTriangleMesh * mesh);

//--Random time things
float getDT();
std::chrono::time_point<std::chrono::high_resolution_clock> t1,t2;

//--Main
int main(int argc, char **argv)
{
    // Initialize glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(w, h);
    // Name and create the Window
    glutCreateWindow("Matrix Example");

    // Now that the window is created the GL context is fully set up
    // Because of that we can now initialize GLEW to prepare work with shaders
    GLenum status = glewInit();
    if( status != GLEW_OK)
    {
        std::cerr << "[F] GLEW NOT INITIALIZED: ";
        std::cerr << glewGetErrorString(status) << std::endl;
        return -1;
    }

    glutCreateMenu(menu);
    glutAddMenuEntry("start", 1);
    glutAddMenuEntry("pause/resume", 2);
    glutAddMenuEntry("exit", 3);
    glutAttachMenu(GLUT_RIGHT_BUTTON);

    // Set all of the callbacks to GLUT that we need
    glutDisplayFunc(render);// Called when its time to display
    glutReshapeFunc(reshape);// Called if the window is resized
    glutKeyboardFunc(keyboard);// Called if there is keyboard input
    glutMotionFunc(motion);
    //glutIdleFunc(update);

    // Initialize all of our resources(shaders, geometry)
    bool init = initialize();
    if(init)
    {
    t1 = std::chrono::high_resolution_clock::now();

    GLuint Texture = loadBMP_custom("rickross.bmp");

    myText = glGetUniformLocation(program, const_cast<const char*>("texty"));

	if(myText == -1)
	{
		std::cerr << "[F] myText NOT FOUND" << std::endl;
		//return false;
	}

    GLint i = 0;
    glUniform1i( Texture, i);


    // init physics

    // span
    btVector3 worldMin(-100,-100,-100);
    btVector3 worldMax(100,100,100);

    // bullet physics initializing
    btBroadphaseInterface* broadphase = new btAxisSweep3(worldMin,worldMax);
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,-20,0));

    // collision shapes
    groundShape = new btBvhTriangleMeshShape(boardMesh, true, true);
    btCollisionShape* ballShape = new btSphereShape(1.5);
    


    //std::cout<< ballShape->getMargin << std::endl;
    // scale the ground up
    //groundShape->setLocalScaling(btVector3(2,1,2));
    //ballShape->setLocalScaling(btVector3(10,10,10));

    // board transform0.5
    glm::mat4 m = view * boardModel;  
    boardTransform.setFromOpenGLMatrix((float *)&boardModel);
  
    // motion states/connect it all

    btDefaultMotionState* boardMotionState = new btDefaultMotionState(boardTransform);
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, boardMotionState, groundShape, btVector3(0,0,0));

    // board rigid body
    groundRigidBody = new btRigidBody(groundRigidBodyCI);
	
    // add it to the world
    dynamicsWorld->addRigidBody(groundRigidBody);

    // ball transform
    btTransform ballTransform;
    m = view * ballModel;
    ballTransform.setFromOpenGLMatrix((float *)&ballModel);
    //ballTransform.setOrigin(btVector3(-4.25, 0, 1.5));
    btDefaultMotionState* ballMotionState = new btDefaultMotionState(ballTransform);

    // ball is dynamic, so give it mass
    btScalar mass =70;
    btVector3 ballInertia(0,0,0);
    ballShape->calculateLocalInertia(mass,ballInertia);

    // glue it together
    btRigidBody::btRigidBodyConstructionInfo ballRigidBodyCI(mass, ballMotionState, ballShape, ballInertia);
    ballRigidBody = new btRigidBody(ballRigidBodyCI);
    //ballRigidBody->setCcdMotionThreshold(1.5f);
    //ballRigidBody->setCcdSweptSphereRadius(1.5f);	

    // add it to the world
    dynamicsWorld->addRigidBody(ballRigidBody);
    
    // register a collsion function, I'm using a custom one with the kinds of collsion shapes being used
    dispatcher->registerCollisionCreateFunc(TRIANGLE_MESH_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE, collisionConfiguration->getCollisionAlgorithmCreateFunc(TRIANGLE_MESH_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE));
    // declared it the other way around too to be safe
	dispatcher->registerCollisionCreateFunc( SPHERE_SHAPE_PROXYTYPE,TRIANGLE_MESH_SHAPE_PROXYTYPE, collisionConfiguration->getCollisionAlgorithmCreateFunc(TRIANGLE_MESH_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE));

    // make the board kinetic!! very important!!
    groundRigidBody->setCollisionFlags(groundRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);

    //enable custom material callback; maybe it helps?
    groundRigidBody->setCollisionFlags(groundRigidBody->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

    // kinetic objects sleep when they aren't active, don't let the board sleep
    groundRigidBody->setActivationState(DISABLE_DEACTIVATION);
ballRigidBody->getMotionState()->getWorldTransform(trans);   
	    // get new location for ball
	    std::cout <<  trans.getOrigin().getX() << " " << trans.getOrigin().getY()<< " " <<
	     trans.getOrigin().getZ() << std::endl;
    // run dis
    glutMainLoop();
    
    }


    // Clean up after ourselves
    cleanUp();

    return 0;
}

// Data read from the header of the BMP file
unsigned char header[54]; // Each BMP file begins by a 54-bytes header
unsigned int dataPos;     // Position in the file where the actual data begins
unsigned int bmpWidth, bmpHeight;
unsigned int imageSize;   // = width*height*3
// Actual RGB data
unsigned char * data;

GLuint loadBMP_custom(const char * imagepath)
{
	FILE * file = fopen(imagepath,"rb");
	if (!file)                              
	{
		printf("Image could not be opened\n"); 
		return 0;
	}

	if ( fread(header, 1, 54, file)!=54 )
	{ 
		// If not 54 bytes read : problem
    		printf("Not a correct BMP file\n");
    		return false;
	}

	if ( header[0]!='B' || header[1]!='M' )
	{
    		printf("Not a correct BMP file\n");
    		return 0;
	}

	// Read ints from the byte array
	dataPos    = *(int*)&(header[0x0A]);
	imageSize  = *(int*)&(header[0x22]);
	bmpWidth      = *(int*)&(header[0x12]);
	bmpHeight     = *(int*)&(header[0x16]);

	// Some BMP files are misformatted, guess missing information
	if(imageSize==0)
	{
		imageSize=bmpWidth*bmpHeight*3; // 3 : one byte for each Red, Green and Blue component
	}

	if (dataPos==0)
	{
		dataPos=54; // The BMP header is done that way
	}

	// Create a buffer
	data = new unsigned char [imageSize];
	 
	// Read the actual data from the file into the buffer
	fread(data,1,imageSize,file);
	 
	//Everything is in memory now, the file can be closed
	fclose(file);

	// Create one OpenGL texture
	GLuint textureID;
	glGenTextures(1, &textureID);
	 
	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID);
	 
	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, bmpWidth, bmpHeight, 0, GL_BGR, GL_UNSIGNED_BYTE, data);
	 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	

	return textureID;
}

// menu fun
void menu(int id)
{
	switch(id)
	{
		case 1: 
			glutIdleFunc(update);
			break;
		case 2:
			pauseGame();
			break;
		case 3:
			exit(0);
			break;
	}
}

void meh()
{
	
}

// work around for pausing bullet simulation
void pauseGame()
{
   	if(gamePaused == false)
	{
		gamePaused = true;
   		glutIdleFunc(meh);
	}
	else
	{
		glutIdleFunc(update);
		gamePaused = false;
	}
}

//--Implementations
void render()
{
    //--Render the scene

    //clear the screen
    glClearColor(0.0, 0.0, 0.2, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   //enable the shader program
    glUseProgram(program);

    renderModel(1, vbo_geometry, 10000);
    renderModel(2, vbo_ball, 3000);
    renderModel(3, vbo_horse, 10000); 

    //clean up
    glDisableVertexAttribArray(loc_position);
    glDisableVertexAttribArray(loc_color);
    glDisableVertexAttribArray(loc_normal);
    glDisableVertexAttribArray(loc_vt);
                           
    //swap the buffers
    glutSwapBuffers();
}

void renderModel(int model, int vbo, int vertices)
{
    //premultiply the matrix for this example
    if(model == 1)
    {
 	mvp = projection * view * boardModel;
	mv = view * boardModel;
    }
		//mvp = boardModel;
    else if(model == 2)
    {
    	mvp = projection * view * ballModel;
	mv = view * ballModel;
    }
    else if(model == 3)
    {
    	mvp = projection * view * horseModel;
	mv = view * horseModel;
    }
 
    norm = glm::transpose(glm::inverse(mv));

    //upload the matrix to the shader
    glUniformMatrix4fv(loc_mvpmat, 1, GL_FALSE, glm::value_ptr(mvp));
    glUniformMatrix4fv(loc_mvmat, 1, GL_FALSE, glm::value_ptr(mv));
    glUniformMatrix4fv(loc_normmat, 1, GL_FALSE, glm::value_ptr(norm));
   
    //set up the Vertex Buffer Object so it can be drawn
    glEnableVertexAttribArray(loc_position);
    glEnableVertexAttribArray(loc_color);
    glEnableVertexAttribArray(loc_normal);
    glEnableVertexAttribArray(loc_vt);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    //set pointers into the vbo for each of the attributes(position and color)
    glVertexAttribPointer( loc_position,//location of attribute
                           3,//number of elements
                           GL_FLOAT,//type
                           GL_FALSE,//normalized?
                           sizeof(Vertex),//stride
                           0);//offset

     glVertexAttribPointer( loc_color,
                           3,
                           GL_FLOAT,
                           GL_FALSE,
                           sizeof(Vertex),
                           (void*)offsetof(Vertex,color));

     glVertexAttribPointer( loc_normal,
                           3,
                           GL_FLOAT,
                           GL_FALSE,
                           sizeof(Vertex),
                           (void*)offsetof(Vertex,vnormal));

	glVertexAttribPointer( loc_vt,
                           2,
                           GL_FLOAT,
                           GL_FALSE,
                           sizeof(Vertex),
                           (void*)offsetof(Vertex,vt));

     
     	glDrawArrays(GL_TRIANGLES, 0, vertices);//mode, starting index, count
     
	
}
bool timeLock = true;
// move ball
void update()
{
   dynamicsWorld->debugDrawWorld();
 if(getDT()>(1/120))
    {
        dynamicsWorld->stepSimulation(1.0f/120.0f, 1, 1.0f/60.0f);
    }

	ballRigidBody->getMotionState()->getWorldTransform(trans);   
	    // get new location for ball
	    oldX =  trans.getOrigin().getX() - oldX;
	    oldY =  trans.getOrigin().getY() - oldY;
	    oldZ =  trans.getOrigin().getZ() - oldZ;

	    // translate it to that location
	    ballModel = glm::translate( glm::mat4(1.0f), glm::vec3(oldX, oldY, oldZ));
	    horseModel = glm::rotate(horseModel, 1.0f, glm::vec3(0, 0, 1)); 
    // Update the state of the scene
    glutPostRedisplay();//call the display callback   
}

void reshape(int n_w, int n_h)
{
    w = n_w;
    h = n_h;
    //Change the viewport to be correct
    glViewport( 0, 0, w, h);
    //Update the projection matrix as well
    //See the init function for an explaination
    projection = glm::perspective(45.0f, float(w)/float(h), 0.01f, 100.0f);

}

void keyboard(unsigned char key, int x_pos, int y_pos)
{
    // Handle keyboard input
    if(key == 27)//ESC
    {
        exit(0);
    }
    else if(key == 108)//l
    {
        boardModel = glm::rotate(boardModel, 1.0f, glm::vec3(0, 0, 1));
    } 
    else if(key == 106)//j
    {
        boardModel = glm::rotate(boardModel, -1.0f, glm::vec3(0, 0, 1));
    }
    else if(key == 105)//i
    {
        boardModel = glm::rotate(boardModel, 1.0f, glm::vec3(1, 0, 0));
    } 
    else if(key == 107)//k
    {
        boardModel = glm::rotate(boardModel, -1.0f, glm::vec3(1, 0, 0));
    }
 
    // find out where the board is (opengl)
    	boardTransform.setFromOpenGLMatrix((float *)&boardModel);
    // set the new board angle in bullet (important)
	groundRigidBody->getMotionState()->setWorldTransform(boardTransform);   
}

// mouse movement (works with drag and click)
void motion (int x, int y)
{
   if(x < currentX)
   {
	boardModel = glm::rotate(boardModel, -0.5f, glm::vec3(0, 0, 1));
   }
   else if(x > currentX)
   {
	boardModel = glm::rotate(boardModel, 0.5f, glm::vec3(0, 0, 1));
   }

   else if(y > currentY)
   {
	boardModel = glm::rotate(boardModel, -0.5f, glm::vec3(1, 0, 0));
   }
   else if(y < currentY)
   {
	boardModel = glm::rotate(boardModel, 0.5f, glm::vec3(1, 0, 0));
   }
   
   currentX = x;
   currentY = y;
   
   // find out where the board is (opengl)
    	boardTransform.setFromOpenGLMatrix((float *)&boardModel);
    // set the new board angle in bullet (important)
	groundRigidBody->getMotionState()->setWorldTransform(boardTransform); 
}

bool initialize()
{
	// Initialize basic geometry and shaders for this example

	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);

	// blender files
	char vFileName[] = "vertexFile.txt";
	char fFileName[] = "fragFile.txt";
	char oFileName[] = "holes.obj";
	char mFileName[] = "holes.mtl";
	char ballOFile[] = "smallSphere.obj";
	char ballMFile[] = "smallSphere.mtl";
	char horseOFile[] = "key.obj";
	char horseMFile[] = "key.mtl";

	// load models
	loadOBJ(oFileName, mFileName, board, boardMesh);
	loadOBJ(ballOFile, ballMFile, ball, ballMesh);
	loadOBJ(horseOFile, horseMFile, horse, horseMesh);

	// Create a Vertex Buffer object to store this vertex info on the GPU

	// generate an unused identifier for a buffer object in the buffer
	glGenBuffers(1, &vbo_geometry);
	// creates vbo_geometry as a new buffer object
	glBindBuffer(GL_ARRAY_BUFFER, vbo_geometry);
	// allocate memory for the vertex array 
	glBufferData(GL_ARRAY_BUFFER, sizeof(board), board, GL_STATIC_DRAW);

	// do the same for the ball
	glGenBuffers(1, &vbo_ball);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_ball);
	glBufferData(GL_ARRAY_BUFFER, sizeof(ball), ball, GL_STATIC_DRAW);

	// do the same for the debug lines
	glGenBuffers(1, &vbo_horse);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_horse);
	glBufferData(GL_ARRAY_BUFFER, sizeof(horse), horse, GL_STATIC_DRAW);

	//--Geometry done

	// create empty shader objects and return their identifiers
	GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

	//Shader Sources
	// Note the added uniform!
	const char *vs = textFileRead(vFileName);
	const char *fs = textFileRead(fFileName);

	//compile the shaders
	GLint shader_status;

	// Vertex shader first
	glShaderSource(vertex_shader, 1, &vs, NULL);
	glCompileShader(vertex_shader);
	char logbuffer[1000];
	int loglen;
	//check the compile status
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &shader_status);
	glGetShaderInfoLog(vertex_shader, sizeof(logbuffer), &loglen, logbuffer);
std::cout<<"v: " << logbuffer << std::endl;
	if(!shader_status)
	{
		std::cerr << "[F] FAILED TO COMPILE VERTEX SHADER!" << std::endl;
		return false;
	}

	// Now the Fragment shader
	glShaderSource(fragment_shader, 1, &fs, NULL);
	glCompileShader(fragment_shader);

	//check the compile status
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &shader_status);
	glGetShaderInfoLog(fragment_shader, sizeof(logbuffer), &loglen, logbuffer);
std::cout<<"f: " << logbuffer << std::endl;
 	if(!shader_status)
	{
		std::cerr << "[F] FAILED TO COMPILE FRAGMENT SHADER!" << std::endl;
		return false;
	}

	//Now we link the 2 shader objects into a program
	//This program is what is run on the GPU
	program = glCreateProgram();
	glAttachShader(program, vertex_shader);
	glAttachShader(program, fragment_shader);
	glLinkProgram(program);

	//check if everything linked ok
	glGetProgramiv(program, GL_LINK_STATUS, &shader_status);

	if(!shader_status)
	{
		std::cerr << "[F] THE SHADER PROGRAM FAILED TO LINK" << std::endl;
		return false;
	}

	//Now we set the locations of the attributes and uniforms
	//this allows us to access them easily while rendering
	loc_position = glGetAttribLocation(program, const_cast<const char*>("v_position"));

	if(loc_position == -1)
	{
		std::cerr << "[F] POSITION NOT FOUND" << std::endl;
		return false;
	}

	loc_color = glGetAttribLocation(program, const_cast<const char*>("v_color"));

	if(loc_color == -1)
	{
		std::cerr << "[F] V_COLOR NOT FOUND" << std::endl;
		return false;
	}

	loc_normal = glGetAttribLocation(program, const_cast<const char*>("v_normal"));

	if(loc_normal == -1)
	{
		std::cerr << "[F] V_NORMAL NOT FOUND" << std::endl;
		return false;
	}

	loc_vt = glGetAttribLocation(program, const_cast<const char*>("v_vt"));

	if(loc_vt == -1)
	{
		std::cerr << "[F] V_VT NOT FOUND" << std::endl;
		return false;
	}

	loc_mvpmat = glGetUniformLocation(program, const_cast<const char*>("mvpMatrix"));

	if(loc_mvpmat == -1)
	{
		std::cerr << "[F] MVPMATRIX NOT FOUND" << std::endl;
		return false;
	}

	loc_mvmat = glGetUniformLocation(program, const_cast<const char*>("mvMatrix"));

	if(loc_mvmat == -1)
	{
		std::cerr << "[F] MVMATRIX NOT FOUND" << std::endl;
		return false;
	}

        loc_normmat = glGetUniformLocation(program, const_cast<const char*>("normMatrix"));

	if(loc_normmat == -1)
	{
		std::cerr << "[F] NORMMAT NOT FOUND" << std::endl;
		return false;
	}

	//--Init the view and projection matrices
	//  if you will be having a moving camera the view matrix will need to more dynamic
	//  ...Like you should update it before you render more dynamic 
	//  for this project having them static will be fine
	view = glm::lookAt( glm::vec3(0.0, 80.0, -20.0), //Eye Position
		        glm::vec3(0.0, 0.0, 0.0), //Focus point
		        glm::vec3(0.0, 1.0, 0.0)); //Positive Y is up

	projection = glm::perspective( 45.0f, //the FoV typically 90 degrees is good which is what this is set to
		                   float(w)/float(h), //Aspect Ratio, so Circles stay Circular
		                   0.01f, //Distance to the near plane, normally a small value like this
		                   100.0f); //Distance to the far plane, 


	//enable depth testing
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	horseModel = glm::translate( glm::mat4(1.0f), glm::vec3(10.0f, 10.0f, 10.0f));
	//and its done
        DemoLight();
	return true;
}

void DemoLight()
   {
	
     glEnable(GL_LIGHTING);
     glEnable(GL_LIGHT0);
     glEnable(GL_NORMALIZE);
     
     // Light model parameters:
     // -------------------------------------------
     
     GLfloat lmKa[] = {0.0, 0.0, 0.0, 0.0 };
     glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmKa);
     
     glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, 1.0);
     glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 0.0);
     
     // -------------------------------------------
     // Spotlight Attenuation
     
     GLfloat spot_direction[] = {1.0, -1.0, -1.0 };
     GLint spot_exponent = 30;
     GLint spot_cutoff = 180;
     
     glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_direction);
     glLighti(GL_LIGHT0, GL_SPOT_EXPONENT, spot_exponent);
     glLighti(GL_LIGHT0, GL_SPOT_CUTOFF, spot_cutoff);
    
     GLfloat Kc = 1.0;
     GLfloat Kl = 0.0;
     GLfloat Kq = 0.0;
     
     glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION,Kc);
     glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, Kl);
     glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, Kq);
     
     
     // ------------------------------------------- 
     // Lighting parameters:

     GLfloat light_pos[] = {0.0f, 2.0f, 2.0f, 1.0f};
     GLfloat light_Ka[]  = {1.0f, 0.5f, 0.5f, 1.0f};
     GLfloat light_Kd[]  = {1.0f, 0.1f, 0.1f, 1.0f};
     GLfloat light_Ks[]  = {1.0f, 1.0f, 1.0f, 1.0f};

     glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
     glLightfv(GL_LIGHT0, GL_AMBIENT, light_Ka);
     glLightfv(GL_LIGHT0, GL_DIFFUSE, light_Kd);
     glLightfv(GL_LIGHT0, GL_SPECULAR, light_Ks);

     // -------------------------------------------
     // Material parameters:
     //GLfloat material_Ka[] = {0.138335f, 0.64f, 00.053679f, 1.0f};
     GLfloat material_Kd[] = {0.4f, 0.4f, 0.5f, 1.0f};
     GLfloat material_Ks[] = {0.8f, 0.8f, 0.0f, 1.0f};
     GLfloat material_Ke[] = {0.1f, 0.0f, 0.0f, 0.0f};
     GLfloat material_Se = 20.0f;

     //glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material_Ka);
     glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_Kd);
     glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material_Ks);
     glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, material_Ke);
     glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, material_Se);
   }

void cleanUp()
{
    // Clean up, Clean up
    glDeleteProgram(program);
    glDeleteBuffers(1, &vbo_geometry);
}

//returns the time delta
float getDT()
{
    float ret;
    t2 = std::chrono::high_resolution_clock::now();
    ret = std::chrono::duration_cast< std::chrono::duration<float> >(t2-t1).count();
    t1 = std::chrono::high_resolution_clock::now();
    return ret;
}

// grabbed from lighthouse3d
char *textFileRead(char *fn) {
 
    FILE *fp;
    char *content = NULL;
 
    int count=0;
 
    if (fn != NULL) {
        fp = fopen(fn,"rt");
 
        if (fp != NULL) {
 	
      fseek(fp, 0, SEEK_END);
      count = ftell(fp);
      rewind(fp);
 
            if (count > 0) {
                content = (char *)malloc(sizeof(char) * (count+1));
                count = fread(content,sizeof(char),count,fp);
                content[count] = '\0';
            }
	    
            fclose(fp);
        }
    }
    return content;
}

// hacked model loader 
bool loadOBJ(char * obj, char * mtl, Vertex geometry[], btTriangleMesh * mesh)
{
	Vertex * vertices = new Vertex[NUM_V];
        Material * materials = new Material[10];
	float vts[1000][2];

	int vertexCounter = 0;
	int normalCounter = 0;
	int geometryCounter = 0;
	int materialCounter = 0;
	int vtCounter = 0;
	int materialIndex;
        int triVCounter = 0;
        char currentMat[20];
	float temp;
	btVector3 VA, VB, VC;

	// first read in materials
	FILE * file = fopen(mtl, "r");
	if(file == NULL)
	{
		printf("Cannot open materials\n");
		return false;
	}

	while(true)
	{
		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
	
		if (res == EOF)
		{
			break; // EOF = End Of File. Quit the loop.
		}

		if ( strcmp( lineHeader, "newmtl" ) == 0 )
		{
			fscanf(file, "%s \n", materials[materialCounter].matName);
		}
		
		// else : parse lineHeader
		else if ( strcmp( lineHeader, "Kd" ) == 0 )
		{
			fscanf(file, "%f %f %f\n", &materials[materialCounter].rgb[0], &materials[materialCounter].rgb[1], &materials[materialCounter].rgb[2] );
			materialCounter++;
                   
		}
	}

	// then read in vertices and faces
	file = fopen(obj, "r");
	if(file == NULL)
	{
		printf("Cannot open model\n");
		return false;
	}

	while(true)
	{
		char lineHeader[128];
		char object[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
	
		if (res == EOF)
		{
			break; // EOF = End Of File. Quit the loop.
		}

		// else : parse lineHeader
		// grap them vertices
		if ( strcmp( lineHeader, "v" ) == 0 )
		{
			fscanf(file, "%f %f %f\n", &vertices[vertexCounter].position[0], &vertices[vertexCounter].position[1], &vertices[vertexCounter].position[2] );

			// also put the veritces into triangles so they can go in a mesh for bullet
			if(triVCounter == 0)
			{
    		        	VA.setX(vertices[vertexCounter].position[0]);
				VA.setY(vertices[vertexCounter].position[1]);
				VA.setZ(vertices[vertexCounter].position[2]);
			}
			else if(triVCounter == 1)
			{
    		        	VB.setX(vertices[vertexCounter].position[0]);
				VB.setY(vertices[vertexCounter].position[1]);
				VB.setZ(vertices[vertexCounter].position[2]);
			}
			else if(triVCounter == 2)
			{
    		        	VC.setX(vertices[vertexCounter].position[0]);
				VC.setY(vertices[vertexCounter].position[1]);
				VC.setZ(vertices[vertexCounter].position[2]);

				mesh->addTriangle(VA,VB,VC,false);
				triVCounter = -1;
		
			}

			triVCounter++;

			vertexCounter++;
			
		}

  		// material reading
		else if( strcmp( lineHeader, "usemtl" ) == 0 ) 
		{
			fscanf(file, "%s", currentMat);
			for(int i = 0; i < 5; i++)
			{
				if(strcmp(currentMat, materials[i].matName) == 0)
				{
					materialIndex = i;
					break;
				}
			}	
		}

		// get object
		else if( strcmp( lineHeader, "o" ) == 0 ) 
		{
			fscanf(file, "%s", object);
		}

                // normal reading
		else if( strcmp( lineHeader, "vn" ) == 0 ) 
		{
			fscanf(file, "%f %f %f\n", &vertices[normalCounter].vnormal[0], &vertices[normalCounter].vnormal[1], &vertices[normalCounter].vnormal[2] );

			normalCounter++;	
		}

		// normal reading
		else if( strcmp( lineHeader, "vt" ) == 0 ) 
		{
			
			if(strcmp( object, "Cube" ) == 0)
			{
				fscanf(file, "%f %f\n", &vts[vtCounter][0], &vts[vtCounter][1]);
			}

			else
			{
				fscanf(file, "%f %f %f\n", &vts[vtCounter][0], &vts[vtCounter][1], &temp );
			}
			vtCounter++;	
		}

		// getting faces to match up with materials

		else if( strcmp( lineHeader, "f" ) == 0 )
		{
			int vertex1, vertex2, vertex3;
			int normal1, normal2, normal3;
			int vt1, vt2, vt3;

			if(strcmp( object, "Sphere" ) != 0)
			{
				
				fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", 
				&vertex1, &vt1, &normal1, &vertex2, &vt2, &normal2, &vertex3, &vt3, &normal3);
			}	
			else
			{
				fscanf(file, "%d//%d %d//%d %d//%d\n", 
				&vertex1, &normal1, &vertex2, &normal2, &vertex3, &normal3); 
			}

			// x, y, z for first vertex of face
			for(int i = 0; i < 3; i++)
			{	
				geometry[geometryCounter].position[i] = vertices[vertex1-1].position[i];
				geometry[geometryCounter].vnormal[i] = vertices[normal1-1].vnormal[i];
				geometry[geometryCounter].color[i] = materials[materialIndex].rgb[i];
				
			}
			
			if(strcmp( object, "Sphere" ) != 0)
			{
				geometry[geometryCounter].vt[0] = vts[vt1-1][0];
				geometry[geometryCounter].vt[1] = vts[vt1-1][1];
			}

			geometryCounter++;

			// x, y, z for second vertex of face
			for(int i = 0; i < 3; i++)
			{	
				geometry[geometryCounter].position[i] = vertices[vertex2-1].position[i];
				geometry[geometryCounter].vnormal[i] = vertices[normal2-1].vnormal[i];
				geometry[geometryCounter].color[i] = materials[materialIndex].rgb[i];
			}

			if(strcmp( object, "Sphere" ) != 0)
			{
				geometry[geometryCounter].vt[0] = vts[vt2-1][0];
				geometry[geometryCounter].vt[1] = vts[vt2-1][1];
			}
		
			geometryCounter++;

			// x, y, z for third vertex of face
			for(int i = 0; i < 3; i++)
			{	
				geometry[geometryCounter].position[i] = vertices[vertex3-1].position[i];
				geometry[geometryCounter].vnormal[i] = vertices[normal3-1].vnormal[i];
				geometry[geometryCounter].color[i] = materials[materialIndex].rgb[i];
			}

			if(strcmp( object, "Sphere" ) != 0)
			{
				geometry[geometryCounter].vt[0] = vts[vt3-1][0];
				geometry[geometryCounter].vt[1] = vts[vt3-1][1];
			}	
		
			geometryCounter++;
		}
	}
	return true;
}

