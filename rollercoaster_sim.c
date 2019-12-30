#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <GL/glut.h>
#include <stdio.h>
#include <limits.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

#define RAD2DEG  180.0/M_PI
#define DEG2RAD  M_PI/180.0

#define MIN(a,b)  (((a)<(b))?(a):(b))
#define MAX(a,b)  (((a)>(b))?(a):(b))

#define TIME_DELTA  33
#define INPUT_BUF_SIZE  20
#define CURVE_INTERVAL  0.03
#define RAIL_TIE_INTERVAL  0.06
#define SUPPORT_BEAM_INTERVAL  0.40
#define GROUND_PLANE_Y_OFFSET  0.1
#define GRAVITY  9.81
#define MIN_CART_SPEED  0.00065
#define MAX_ROTATE_SPEED  1.1
#define MIN_ROTATE_SPEED  -MAX_ROTATE_SPEED


/* -- type definitions ------------------------------------------------------ */

typedef struct {
    double x, y, z;
} Point;

// struct for the cart that moves around the track
typedef struct {
    Point pos;
    int curveIndex;
    double u, height, speed, mass;
} Cart;

typedef struct {
    Point pos;
    Point target;
    // variables for rotating the camera around the coaster
    double rotateSpeed, theta_v, theta_h, distance;
} Camera;

typedef struct {
    // vertices are listed here in the order in which they should be drawn
    Point v1, v2, v3, v4;
} Plane;

typedef struct {
    Point* circlePoints;
    int numCirclePoints;
    double radius;
    Point baseTopOffset;
} Cylinder;

typedef struct {
    Point** rail_1;
    Point** rail_2;
    Point** centerRail;
    Cylinder* railTies;
    Cylinder* supportBeams;
    int pointsPerRail, numRailTies, numSupportBeams;
    double railRadius, centerRailRadius, railTieRadius;
} Rails;


/* -- function prototypes --------------------------------------------------- */

static void	myDisplay(void);
static void myTimer(int value);
static void myKey(unsigned char key, int x, int y);
static void keyPress(int key, int x, int y);
static void myReshape(int w, int h);

static void	init(void);

static Point q_func(double u, int curveIndex);
static Point q_i_func(double u, int curveIndex);
static Point q_ii_func(double u, int curveIndex);

static void draw_control_points();
static void draw_roller_coaster_track();
static void draw_ground_plane_and_skybox();
static void draw_cylinder(Cylinder cylinder, float* topColor, float* bottomColor);

static void init_camera_and_cart();
static void init_knot_values();
static void init_roller_coaster_points();
static void init_ground_plane_and_skybox();
static void init_rail_display_list();

static char* get_input();
static char** parse_input_into_lines(char* userInput);
static void parse_lines_into_control_points(char** lines);
static void move_cart();
static double clamp (double value, double min, double max);
static Point normalize(Point vector);
static Point cross_product(Point vectorA, Point vectorB);


/* -- global variables ------------------------------------------------------ */

static int numControlPoints, numKnotValues, numCoasterPoints, cameraRotating = 1, renderCtrlPoints = 0;
static int* knotValues;
static double xMax, yMax, debugTimer = 0.0;
static Point* controlPoints;
static Point* coasterPoints;
static Point coasterMidPoint, minCoasterPoint, maxCoasterPoint;
static Camera camera;
static Cart cart;
static Plane groundPlane;
static Cylinder skyCylinder;
static Rails coasterRails;
static float color_sky_light[3] = {0.0f, 1.0f, 1.0f}, 
             color_sky_dark[3] = {0.0f, 0.2f, 0.5f},
             color_ground[3] = {0.1f, 0.55f, 0.1f},
             color_rails[3] = {0.7f, 0.1f, 0.1f},
             color_rail_ties[3] = {0.4f, 0.4f, 0.4f},
             color_support_beams[3] = {0.8f, 0.3f, 0.0f}; 

static GLuint railDisplayList;


/* -- main ------------------------------------------------------------------ */

int
main(int argc, char *argv[]) {
    srand((unsigned int) time(NULL));

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1152, 648);
    glutCreateWindow("Rollercoaster Sim");
    glutDisplayFunc(myDisplay);
    glutIgnoreKeyRepeat(0);
    glutKeyboardFunc(myKey);
    glutSpecialFunc(keyPress);
    glutReshapeFunc(myReshape);
    glutTimerFunc(TIME_DELTA, myTimer, 0);
    glClearColor(color_sky_dark[0], color_sky_dark[1], color_sky_dark[2], 1.0);

    init();
    
    glutMainLoop();
    return 0;
}


/* ================================================ GLUT Callback Functions ============================================== */

void
myDisplay() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glEnable(GL_CULL_FACE);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    glLoadIdentity();
    gluLookAt(camera.pos.x, camera.pos.y, camera.pos.z, camera.target.x, camera.target.y, camera.target.z, 0.0, 1.0, 0.0);

    if(renderCtrlPoints){
        draw_control_points();
    }

    draw_ground_plane_and_skybox();
    draw_roller_coaster_track();

    // draw the cart on the roller coaster track
    glPushMatrix();
    glTranslated(cart.pos.x, cart.pos.y + cart.height, cart.pos.z);
    glColor3f(1.0, 1.0, 0.2);
    glutWireSphere(0.1, 5, 5);
    glPopMatrix();
    
    glutSwapBuffers();
}

void
myTimer(int value) {
    move_cart();

    debugTimer += TIME_DELTA;
    /*if(debugTimer > 1500){
        debugTimer -= 1500;
        printf("cart pos:   x = %.2f   y = %.2f   z = %.2f\n", cart.pos.x, cart.pos.y, cart.pos.z);
        printf("cart info   speed = %.2f  u = %.2f   index = %d\n\n", cart.speed, cart.u, cart.curveIndex);
    }*/

    if (cameraRotating) {
        // rotate the camera around the mid point of the roller coaster
        camera.pos.x = camera.target.x + camera.distance * cos(DEG2RAD * camera.theta_h);
        camera.pos.y = camera.target.y + camera.distance * sin(DEG2RAD * camera.theta_v);
        camera.pos.z = camera.target.z + camera.distance * sin(DEG2RAD * camera.theta_h);

        camera.theta_h += camera.rotateSpeed;
        if(camera.theta_h >= 360.0) camera.theta_h -= 360.0;
    } 

    else {
        // put the camera in the cart and add some extra height
        camera.pos = cart.pos;
        camera.pos.y += cart.height;

        // make the camera point in the direction of the cart's movement
        camera.target = q_i_func(cart.u, cart.curveIndex);
        camera.target.x += camera.pos.x;
        camera.target.y += camera.pos.y;
        camera.target.z += camera.pos.z;
    }

    glutPostRedisplay();
    
    glutTimerFunc(TIME_DELTA, myTimer, value);
}

void
myKey(unsigned char key, int x, int y) {
    switch(key) {
        case 'q':
            exit(0); break;
        case 'c':
            cameraRotating = abs(cameraRotating - 1);
            if(cameraRotating){
                // set the target point to be the center of the rollercoaster
                camera.target = coasterMidPoint;
            } 
            break;
        case 'p':
            renderCtrlPoints = abs(renderCtrlPoints - 1);
            break;
    }
}

/*
 *  This function is called when a special key is pressed.
 */
void
keyPress(int key, int x, int y) {
    if(cameraRotating){
        switch (key) {
            case 100:
                // left arrow key
                camera.rotateSpeed = clamp(camera.rotateSpeed+0.015, MIN_ROTATE_SPEED, MAX_ROTATE_SPEED); 
                break;

            case 101:
                // up arrow key
                if (glutGetModifiers() == GLUT_ACTIVE_SHIFT){
                    // change camera theta on the vertical axis
                    camera.theta_v = clamp(camera.theta_v+0.5, 10.0, 85.0);
                } else {
                    // zoom in
                    camera.distance = MAX(2.0, camera.distance-0.1); 
                }
                break;

    	    case 102:
                // right arrow key. 
                camera.rotateSpeed = clamp(camera.rotateSpeed-0.015, MIN_ROTATE_SPEED, MAX_ROTATE_SPEED); 
                break;

            case 103:
                // down arrow key
                if (glutGetModifiers() == GLUT_ACTIVE_SHIFT){
                    // change camera theta on the vertical axis
                    camera.theta_v = clamp(camera.theta_v-0.5, 10.0, 85.0);
                } else {
                    // zoom out
                    camera.distance = MIN(20.0, camera.distance+0.1); 
                } 
                break;
        }
    }
}


/*
 *  Reshape callback function. The upper and lower boundaries of the
 *  window are at 100.0 and 0.0, respectively. The aspect ratio is
 *  determined by the aspect ratio of the viewport
 */
void
myReshape(int w, int h) {
    xMax = 100.0*w/h;
    yMax = 100.0;
    float aspectRatio = (float)w/(float)h;

    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60.0, aspectRatio, 0.2, 100.0);
    glMatrixMode(GL_MODELVIEW);
}


/* =================================================== Processing Functions ============================================== */

/*
 *  Computes a 3D point on the spline from the given u value
 *
 *  Args:
 *     u - A value in [0-1)
 *     curveIndex - A value from [0 .. n-1] for the number of curves segments in the spline
 */
Point
q_func(double u, int curveIndex) {
    double u2 = pow(u, 2);
    double u3 = pow(u, 3);
    double r[4];

    r[0] = u3 / 6.0;
    r[1] = -0.5*u3 + 0.5*u2 + 0.5*u + 0.16666;
    r[2] = 0.5*u3 - u2 + 0.66666;
    r[3] = pow((1-u), 3) / 6.0;

    Point q, pi;
    q.x = 0.0; q.y = 0.0; q.z = 0.0;
    // curve segment 0 depends on cp3, cp2, cp1, and cp0
    int cpIndex = curveIndex + 3;

    for(int i = 0; i < 4; i++){
        pi = controlPoints[cpIndex];
        q.x += r[i] * pi.x;
        q.y += r[i] * pi.y;
        q.z += r[i] * pi.z;
        cpIndex = (cpIndex-1 < 0) ? numControlPoints-1 : cpIndex-1;
    }

    return q;
}

/*
 *  Computes a 3D point representing the velocity on the spline from the given u value
 *
 *  Args:
 *     u - A value in [0-1)
 *     curveIndex - A value from [0 .. n-1] for the number of curves segments in the spline
 */
Point
q_i_func(double u, int curveIndex) {
    double u2 = pow(u, 2);
    double r[4];

    r[0] = u2 / 2.0;
    r[1] = -1.5*u2 + u + 0.5;
    r[2] = 1.5*u2 - 2*u;
    r[3] = -pow((1-u), 2) / 2.0;

    Point q, pi;
    q.x = 0.0; q.y = 0.0; q.z = 0.0;
    // curve segment 0 depends on cp3, cp2, cp1, and cp0
    int cpIndex = curveIndex + 3;

    for(int i = 0; i < 4; i++){
        pi = controlPoints[cpIndex];
        q.x += r[i] * pi.x;
        q.y += r[i] * pi.y;
        q.z += r[i] * pi.z;
        cpIndex = (cpIndex-1 < 0) ? numControlPoints-1 : cpIndex-1;
    }

    return q;
}

/*
 *  Computes a 3D point representing the acceleration on the spline from the given u value
 *
 *  Args:
 *     u - A value in [0-1)
 *     curveIndex - A value from [0 .. n-1] for the number of curves segments in the spline
 */
Point
q_ii_func(double u, int curveIndex) {
    double r[4];
    r[0] = u;
    r[1] = -3.0*u + 1.0;
    r[2] = 3.0*u - 2.0;
    r[3] = 1.0 - u;

    Point q, pi;
    q.x = 0.0; q.y = 0.0; q.z = 0.0;
    // curve segment 0 depends on cp3, cp2, cp1, and cp0
    int cpIndex = curveIndex + 3;

    for(int i = 0; i < 4; i++){
        pi = controlPoints[cpIndex];
        q.x += r[i] * pi.x;
        q.y += r[i] * pi.y;
        q.z += r[i] * pi.z;
        cpIndex = (cpIndex-1 < 0) ? numControlPoints-1 : cpIndex-1;
    }

    return q;
}



/* ==================================================== Drawing Functions ================================================ */

void
draw_control_points(){
    for(int i = 0; i < numControlPoints; i++){
        glPushMatrix();
        glTranslated(controlPoints[i].x, controlPoints[i].y, controlPoints[i].z);
        glColor3f(0.2, 0.2, 1.0);
        glutWireSphere(0.1, 5, 5);
        glPopMatrix();
    }

    // draw the midpoint of the roller coaster
    /*glPushMatrix();
    glTranslated(coasterMidPoint.x, coasterMidPoint.y, coasterMidPoint.z);
    glColor3f(0.2, 1.0, 1.0);
    glutWireSphere(0.1, 5, 5);
    glPopMatrix();*/
}

void
draw_roller_coaster_track(){
    glCallList(railDisplayList);

    /* Track for debugging
    for(int i = 0; i < numCoasterPoints; i++){
        glPushMatrix();
        glTranslated(coasterPoints[i].x, coasterPoints[i].y, coasterPoints[i].z);
        glColor3f(1.0, 0.2, 0.2);
        glutWireSphere(0.1, 2, 2);
        glPopMatrix();   
    }*/
}

void
draw_ground_plane_and_skybox(){
    // draw the ground plane
    glPolygonMode(GL_FRONT, GL_FILL);
    glBegin(GL_QUADS);
        glColor3fv(color_ground);
        glVertex3f(groundPlane.v1.x, groundPlane.v1.y, groundPlane.v1.z);
        glVertex3f(groundPlane.v2.x, groundPlane.v2.y, groundPlane.v2.z);
        glVertex3f(groundPlane.v3.x, groundPlane.v3.y, groundPlane.v3.z);
        glVertex3f(groundPlane.v4.x, groundPlane.v4.y, groundPlane.v4.z);
    glEnd();


    // draw the sky cylinder around the rollercoaster
    glPolygonMode(GL_BACK, GL_FILL);
    glBegin(GL_QUAD_STRIP);
        glColor3fv(color_sky_light);
        draw_cylinder(skyCylinder, color_sky_dark, color_sky_light);
    glEnd();    

}

void 
draw_cylinder(Cylinder cylinder, float* topColor, float* bottomColor){
    for(int i = 0; i <= cylinder.numCirclePoints; i++){
        int ind = i % cylinder.numCirclePoints;
        glColor3fv(topColor);
        glVertex3f(cylinder.circlePoints[ind].x + cylinder.baseTopOffset.x, 
            cylinder.circlePoints[ind].y + cylinder.baseTopOffset.y, 
            cylinder.circlePoints[ind].z + cylinder.baseTopOffset.z);

        glColor3fv(bottomColor);
        glVertex3f(cylinder.circlePoints[ind].x, cylinder.circlePoints[ind].y, cylinder.circlePoints[ind].z);
    }
}

/* ================================================= Initialization Functions ============================================ */

/*
 * Takes in user input, parses it into control points, and initializes various other objects
 */
void
init() {
    char* userInput = get_input();
    printf("text: \n%s\n\n", userInput);

    char** lines = parse_input_into_lines(userInput);
    printf("numControlPoints: %d\n\n", numControlPoints);

    // initialize the global controlPoints array using the number of control points parsed from the input
    controlPoints = (Point*) realloc(controlPoints, numControlPoints * sizeof(Point));

    parse_lines_into_control_points(lines);

    printf("Coaster midpoint: (%.2f, %.2f, %.2f)\n", coasterMidPoint.x, coasterMidPoint.y, coasterMidPoint.z);

    init_camera_and_cart();
    init_knot_values();
    init_roller_coaster_points();
    init_rail_display_list();
    init_ground_plane_and_skybox();

    free(userInput);
    free(lines);
}

void
init_camera_and_cart(){
    camera.theta_h = 0.0;
    camera.theta_v = 45.0;
    camera.rotateSpeed = MAX_ROTATE_SPEED / 2.0;
    camera.distance = 10.0;

    // set the gluLookAt target point to be the middle of the roller coaster
    camera.target = coasterMidPoint;

    // initialize the cart on the track
    cart.u = 0.0;
    cart.curveIndex = 0;
    cart.pos = q_func(cart.u, cart.curveIndex);
    cart.height = 0.1;
    cart.pos.y += cart.height;
    cart.speed = MIN_CART_SPEED;
    cart.mass = 50.0;
}

/**
 *  Set uniformly-spaced knot values for the curve from 0 to n-3, 
 *  where n is the number of control points
 */
void
init_knot_values(){
    numKnotValues = numControlPoints - 3;
    knotValues = (int*) malloc(numKnotValues * sizeof(int));

    for (int i = 0; i < numKnotValues; i++){
        knotValues[i] = i;
    }

    printf("numKnotValues = %d\n", numKnotValues);
}


/*
 *  Initialize the set of points that make the rollercoaster by calling the q(u) function
 */
void
init_roller_coaster_points() {
    coasterRails.pointsPerRail = 12;
    coasterRails.railRadius = 0.02;
    coasterRails.centerRailRadius = 0.01;
    coasterRails.railTieRadius = coasterRails.centerRailRadius;
    coasterRails.numRailTies = 0;
    coasterRails.numSupportBeams = 0;

    int coasterPointIndex = 0, firstBeam = 1;
    double posOnCurve = 0.0, railTieIntervalCounter = 0.0, supportBeamIntervalCounter = 0.0;
    for(int knotIndex = 0; knotIndex < numKnotValues; knotIndex++){
        while (posOnCurve < 1.0){
            coasterPoints = (Point*) realloc(coasterPoints, (coasterPointIndex+1) * sizeof(Point));
            coasterRails.rail_1 = (Point**) realloc(coasterRails.rail_1, (coasterPointIndex+1) * sizeof(Point*));
            coasterRails.rail_2 = (Point**) realloc(coasterRails.rail_2, (coasterPointIndex+1) * sizeof(Point*));
            coasterRails.centerRail = (Point**) realloc(coasterRails.centerRail, (coasterPointIndex+1) * sizeof(Point*));

            // store this point on the curve
            Point newCoasterPoint = q_func(posOnCurve, knotIndex);
            coasterPoints[coasterPointIndex] = newCoasterPoint; 

            // create the points that make up the rails
            // create the up vector
            Point vector_up;
            vector_up.x = 0.0;
            vector_up.y = 1.0;
            vector_up.z = 0.0;

            // calculate the normalized n, u, and v vectors
            Point vector_n = normalize(q_i_func(posOnCurve, knotIndex));
            Point vector_u = normalize(cross_product(vector_up, vector_n));
            Point vector_v = normalize(cross_product(vector_n, vector_u));

            coasterRails.rail_1[coasterPointIndex] = (Point*) malloc(coasterRails.pointsPerRail * sizeof(Point));
            coasterRails.rail_2[coasterPointIndex] = (Point*) malloc(coasterRails.pointsPerRail * sizeof(Point));
            coasterRails.centerRail[coasterPointIndex] = (Point*) malloc(coasterRails.pointsPerRail * sizeof(Point));

            double railSpacing = 0.06;
            // create the points for rail 1 (the left rail)
            for(int i = 0; i < coasterRails.pointsPerRail; i++){
                double theta = 2.0*M_PI*i / coasterRails.pointsPerRail;
                coasterRails.rail_1[coasterPointIndex][i].x = newCoasterPoint.x - (vector_u.x * railSpacing) + (coasterRails.railRadius * cos(theta));
                coasterRails.rail_1[coasterPointIndex][i].y = newCoasterPoint.y - (vector_u.y * railSpacing);
                coasterRails.rail_1[coasterPointIndex][i].z = newCoasterPoint.z - (vector_u.z * railSpacing) + (coasterRails.railRadius * sin(theta)); 
            }

            // create the points for rail 2 (the right rail)
            for(int i = 0; i < coasterRails.pointsPerRail; i++){
                double theta = 2.0*M_PI*i / coasterRails.pointsPerRail;
                coasterRails.rail_2[coasterPointIndex][i].x = newCoasterPoint.x + (vector_u.x * railSpacing) + (coasterRails.railRadius * cos(theta));
                coasterRails.rail_2[coasterPointIndex][i].y = newCoasterPoint.y + (vector_u.y * railSpacing);
                coasterRails.rail_2[coasterPointIndex][i].z = newCoasterPoint.z + (vector_u.z * railSpacing) + (coasterRails.railRadius * sin(theta)); 
            }

            // create the points for the center supporting rail
            for(int i = 0; i < coasterRails.pointsPerRail; i++){
                double theta = 2.0*M_PI*i / coasterRails.pointsPerRail;
                coasterRails.centerRail[coasterPointIndex][i].x = newCoasterPoint.x + (coasterRails.centerRailRadius * cos(theta));
                coasterRails.centerRail[coasterPointIndex][i].y = newCoasterPoint.y - (vector_v.y * railSpacing / 1.3);
                coasterRails.centerRail[coasterPointIndex][i].z = newCoasterPoint.z + (coasterRails.centerRailRadius * sin(theta));
            }

            if(supportBeamIntervalCounter >= SUPPORT_BEAM_INTERVAL || firstBeam){
                firstBeam = 0;
                supportBeamIntervalCounter -= SUPPORT_BEAM_INTERVAL;
                coasterRails.numSupportBeams += 2;
                coasterRails.supportBeams = (Cylinder*) realloc(coasterRails.supportBeams, coasterRails.numSupportBeams * sizeof(Cylinder));

                Cylinder supportBeam1, supportBeam2;
                supportBeam1.numCirclePoints = coasterRails.pointsPerRail;
                supportBeam1.circlePoints = (Point*) malloc(supportBeam1.numCirclePoints * sizeof(Point));
                supportBeam1.radius = coasterRails.railRadius;
                supportBeam1.baseTopOffset.x = 0.0;
                supportBeam1.baseTopOffset.y = coasterRails.rail_1[coasterPointIndex][0].y + GROUND_PLANE_Y_OFFSET;
                supportBeam1.baseTopOffset.z = 0.0;

                for(int i = 0; i < supportBeam1.numCirclePoints; i++){
                    double theta = 2.0*M_PI*i / supportBeam1.numCirclePoints;
                    supportBeam1.circlePoints[i].x = newCoasterPoint.x - (vector_u.x * railSpacing) + (coasterRails.railRadius * cos(theta));
                    supportBeam1.circlePoints[i].y = groundPlane.v1.y - GROUND_PLANE_Y_OFFSET;
                    supportBeam1.circlePoints[i].z = newCoasterPoint.z - (vector_u.z * railSpacing) + (coasterRails.railRadius * sin(theta)); 
                }

                supportBeam2.numCirclePoints = coasterRails.pointsPerRail;
                supportBeam2.circlePoints = (Point*) malloc(supportBeam2.numCirclePoints * sizeof(Point));
                supportBeam2.radius = coasterRails.railRadius;
                supportBeam2.baseTopOffset.x = 0.0;
                supportBeam2.baseTopOffset.y = coasterRails.rail_2[coasterPointIndex][0].y + GROUND_PLANE_Y_OFFSET;
                supportBeam2.baseTopOffset.z = 0.0;
                
                for(int i = 0; i < supportBeam2.numCirclePoints; i++){
                    double theta = 2.0*M_PI*i / supportBeam2.numCirclePoints;
                    supportBeam2.circlePoints[i].x = newCoasterPoint.x + (vector_u.x * railSpacing) + (coasterRails.railRadius * cos(theta));
                    supportBeam2.circlePoints[i].y = groundPlane.v1.y - GROUND_PLANE_Y_OFFSET;
                    supportBeam2.circlePoints[i].z = newCoasterPoint.z + (vector_u.z * railSpacing) + (coasterRails.railRadius * sin(theta)); 
                }

                coasterRails.supportBeams[coasterRails.numSupportBeams-2] = supportBeam1;
                coasterRails.supportBeams[coasterRails.numSupportBeams-1] = supportBeam2;

            }

            // create the points for the rail ties
            if(railTieIntervalCounter >= RAIL_TIE_INTERVAL){
                railTieIntervalCounter -= RAIL_TIE_INTERVAL;
                coasterRails.numRailTies += 2;
                coasterRails.railTies = (Cylinder*) realloc(coasterRails.railTies, coasterRails.numRailTies * sizeof(Cylinder));
                
                Cylinder railTie1, railTie2;
                railTie1.numCirclePoints = coasterRails.pointsPerRail;
                railTie1.circlePoints = (Point*) malloc(railTie1.numCirclePoints * sizeof(Point));
                railTie1.radius = coasterRails.railTieRadius;
                railTie1.baseTopOffset.x = coasterRails.rail_1[coasterPointIndex][0].x - coasterRails.centerRail[coasterPointIndex][0].x;
                railTie1.baseTopOffset.y = coasterRails.rail_1[coasterPointIndex][0].y - coasterRails.centerRail[coasterPointIndex][0].y;
                railTie1.baseTopOffset.z = coasterRails.rail_1[coasterPointIndex][0].z - coasterRails.centerRail[coasterPointIndex][0].z;

                for(int i = 0; i < railTie1.numCirclePoints; i++){
                    double theta = 2.0*M_PI*i / railTie1.numCirclePoints;
                    railTie1.circlePoints[i].x = newCoasterPoint.x + (coasterRails.railTieRadius * cos(theta));
                    railTie1.circlePoints[i].y = coasterRails.centerRail[coasterPointIndex][0].y;
                    railTie1.circlePoints[i].z = newCoasterPoint.z + (coasterRails.railTieRadius * sin(theta));
                }

                railTie2.numCirclePoints = coasterRails.pointsPerRail;
                railTie2.circlePoints = (Point*) malloc(railTie2.numCirclePoints * sizeof(Point));
                railTie2.radius = coasterRails.railTieRadius;
                railTie2.baseTopOffset.x = coasterRails.rail_2[coasterPointIndex][0].x - coasterRails.centerRail[coasterPointIndex][0].x;
                railTie2.baseTopOffset.y = coasterRails.rail_2[coasterPointIndex][0].y - coasterRails.centerRail[coasterPointIndex][0].y;
                railTie2.baseTopOffset.z = coasterRails.rail_2[coasterPointIndex][0].z - coasterRails.centerRail[coasterPointIndex][0].z;
                railTie2.circlePoints = railTie1.circlePoints;

                coasterRails.railTies[coasterRails.numRailTies-2] = railTie1;
                coasterRails.railTies[coasterRails.numRailTies-1] = railTie2;
            }

            posOnCurve += CURVE_INTERVAL;
            railTieIntervalCounter += CURVE_INTERVAL;
            supportBeamIntervalCounter += CURVE_INTERVAL;
            coasterPointIndex++;
        }

        posOnCurve -= 1.0;
    }

    numCoasterPoints = coasterPointIndex;
    printf("numCoasterPoints = %d\n", numCoasterPoints);
    printf("numRailTies = %d\n", coasterRails.numRailTies);
}


void
init_ground_plane_and_skybox(){
    // initialize the sky cylinder
    double x_diff = maxCoasterPoint.x - minCoasterPoint.x;
    double y_diff = maxCoasterPoint.y - minCoasterPoint.y;
    double z_diff = maxCoasterPoint.z - minCoasterPoint.z;
    double ground_y_value = minCoasterPoint.y - GROUND_PLANE_Y_OFFSET;

    skyCylinder.radius = MAX(MAX(x_diff, z_diff) * 3.0, 60.0);
    skyCylinder.baseTopOffset.x = 0;
    skyCylinder.baseTopOffset.y = MAX(y_diff * 2.0, 20.0);
    skyCylinder.baseTopOffset.z = 0;
    skyCylinder.numCirclePoints = 50;
    skyCylinder.circlePoints = (Point*) malloc(skyCylinder.numCirclePoints * sizeof(Point));

    for (int i = 0; i < skyCylinder.numCirclePoints; i++) {
       double theta = 2.0*M_PI*i / skyCylinder.numCirclePoints;
       skyCylinder.circlePoints[i].x = coasterMidPoint.x + skyCylinder.radius * cos(theta);
       skyCylinder.circlePoints[i].y = ground_y_value;
       skyCylinder.circlePoints[i].z = coasterMidPoint.z + skyCylinder.radius * sin(theta);
    }

    printf("sky cylinder radius = %.2f\nsky cylinder height = %.2f\n", skyCylinder.radius, skyCylinder.baseTopOffset.y);


    // initialize the ground plane vertices
    groundPlane.v1.x = minCoasterPoint.x - skyCylinder.radius;
    groundPlane.v1.y = ground_y_value;
    groundPlane.v1.z = minCoasterPoint.z - skyCylinder.radius;

    groundPlane.v2.x = maxCoasterPoint.x + skyCylinder.radius;
    groundPlane.v2.y = ground_y_value;
    groundPlane.v2.z = minCoasterPoint.z - skyCylinder.radius;

    groundPlane.v3.x = maxCoasterPoint.x + skyCylinder.radius;
    groundPlane.v3.y = ground_y_value;
    groundPlane.v3.z = maxCoasterPoint.z + skyCylinder.radius;

    groundPlane.v4.x = minCoasterPoint.x - skyCylinder.radius;
    groundPlane.v4.y = ground_y_value;
    groundPlane.v4.z = maxCoasterPoint.z + skyCylinder.radius;

}


/*
 *  Creates a display list that draws the rails and support beams of the rollercoaster track
 */
void 
init_rail_display_list(){
    railDisplayList = glGenLists(1);
    glNewList(railDisplayList, GL_COMPILE);
        for(int coasterPointIndex = 0; coasterPointIndex < numCoasterPoints; coasterPointIndex++){
            // used for joining the first and last rail points with a quad strip
            int coasterWrapAroundIndex = (coasterPointIndex + 1)%(numCoasterPoints-1);
            // render rail 1 (left rail)
            glColor3fv(color_rails);
            glBegin(GL_QUAD_STRIP);
                for(int railPoint = 0; railPoint < coasterRails.pointsPerRail; railPoint++){
                    glVertex3f(coasterRails.rail_1[coasterPointIndex][railPoint].x, 
                        coasterRails.rail_1[coasterPointIndex][railPoint].y, 
                        coasterRails.rail_1[coasterPointIndex][railPoint].z);

                    glVertex3f(coasterRails.rail_1[coasterWrapAroundIndex][railPoint].x, 
                        coasterRails.rail_1[coasterWrapAroundIndex][railPoint].y, 
                        coasterRails.rail_1[coasterWrapAroundIndex][railPoint].z);   
                }
                // specify the first vertex again so the quad strip is joined
                glVertex3f(coasterRails.rail_1[coasterPointIndex][0].x, 
                        coasterRails.rail_1[coasterPointIndex][0].y, 
                        coasterRails.rail_1[coasterPointIndex][0].z);

                glVertex3f(coasterRails.rail_1[coasterWrapAroundIndex][0].x, 
                    coasterRails.rail_1[coasterWrapAroundIndex][0].y, 
                    coasterRails.rail_1[coasterWrapAroundIndex][0].z);
            glEnd();

            // render rail 2 (right rail)
            glBegin(GL_QUAD_STRIP);
                for(int railPoint = 0; railPoint < coasterRails.pointsPerRail; railPoint++){
                    glVertex3f(coasterRails.rail_2[coasterPointIndex][railPoint].x, 
                        coasterRails.rail_2[coasterPointIndex][railPoint].y, 
                        coasterRails.rail_2[coasterPointIndex][railPoint].z);

                    glVertex3f(coasterRails.rail_2[coasterWrapAroundIndex][railPoint].x, 
                        coasterRails.rail_2[coasterWrapAroundIndex][railPoint].y, 
                        coasterRails.rail_2[coasterWrapAroundIndex][railPoint].z);   
                }

                // specify the first vertex again so the quad strip is joined
                glVertex3f(coasterRails.rail_2[coasterPointIndex][0].x, 
                        coasterRails.rail_2[coasterPointIndex][0].y, 
                        coasterRails.rail_2[coasterPointIndex][0].z);

                glVertex3f(coasterRails.rail_2[coasterWrapAroundIndex][0].x, 
                    coasterRails.rail_2[coasterWrapAroundIndex][0].y, 
                    coasterRails.rail_2[coasterWrapAroundIndex][0].z);
            glEnd();

            glColor3fv(color_rail_ties);
            // render center rail (left rail)
            glBegin(GL_QUAD_STRIP);
                for(int railPoint = 0; railPoint < coasterRails.pointsPerRail; railPoint++){
                    glVertex3f(coasterRails.centerRail[coasterPointIndex][railPoint].x, 
                        coasterRails.centerRail[coasterPointIndex][railPoint].y, 
                        coasterRails.centerRail[coasterPointIndex][railPoint].z);

                    glVertex3f(coasterRails.centerRail[coasterWrapAroundIndex][railPoint].x, 
                        coasterRails.centerRail[coasterWrapAroundIndex][railPoint].y, 
                        coasterRails.centerRail[coasterWrapAroundIndex][railPoint].z);   
                }
                // specify the first vertex again so the quad strip is joined
                glVertex3f(coasterRails.centerRail[coasterPointIndex][0].x, 
                        coasterRails.centerRail[coasterPointIndex][0].y, 
                        coasterRails.centerRail[coasterPointIndex][0].z);

                glVertex3f(coasterRails.centerRail[coasterWrapAroundIndex][0].x, 
                    coasterRails.centerRail[coasterWrapAroundIndex][0].y, 
                    coasterRails.centerRail[coasterWrapAroundIndex][0].z);
            glEnd();
        }

        for(int i = 0; i < coasterRails.numRailTies; i++){
            glPolygonMode(GL_BACK, GL_FILL);
            glBegin(GL_QUAD_STRIP);
                draw_cylinder(coasterRails.railTies[i], color_rail_ties, color_rail_ties);
            glEnd();
        }

        for(int i = 0; i < coasterRails.numSupportBeams; i++){
            glPolygonMode(GL_BACK, GL_FILL);
            glBegin(GL_QUAD_STRIP);
                draw_cylinder(coasterRails.supportBeams[i], color_support_beams, color_support_beams);
            glEnd();
        }

    glEndList();
}

/* ===================================================== Helper Functions ================================================ */

/*
 *  Takes input from stdin and returns it as a char*
 */
char*
get_input(){
    char *inputText = calloc(1, sizeof(char));
    char buffer[INPUT_BUF_SIZE];
    
    while(fgets(buffer, INPUT_BUF_SIZE, stdin)) {
        inputText = realloc(inputText, strlen(inputText) + strlen(buffer) + 1);

        if(inputText == NULL){
            printf("Failed to read input\n");
            exit(1);
        }

        strcat(inputText, buffer);
    }

    return inputText;
}


/*
 *  Breaks user input down into separate lines by tokenizing on \n characters. The lines are returned as a char**
 */
char**
parse_input_into_lines(char* userInput){
    char * line = strtok(userInput, "\n");
    char ** lines = NULL;

    // put each input line into the 'lines' array and count the number of control points in the input
    while( line != NULL ) {
        lines = (char**) realloc(lines, (numControlPoints + 1) * sizeof(char*));
        lines[numControlPoints] = (char*) malloc(strlen(line + 1) * sizeof(char));
        strcpy(lines[numControlPoints], line);
        numControlPoints ++;
        line = strtok(NULL, "\n");
    }

    if(numControlPoints < 6){
        printf("Too few control points were given. Please specify more control points and try again.\n");
        exit(0);
    }

    return lines;
}


/*
 *  Tokenizes each line into x, y, and z string values, and then converts them into double values.
 *  A control point is then created from the parsed x, y, and z values and stored for use later.
 *  While parsing the control points, the midpoint of the roller coaster is calculated for 
 *  when the camera is set to rotate around the roller coaster.
 */
void
parse_lines_into_control_points(char** lines){
    minCoasterPoint.x = INT_MAX;
    minCoasterPoint.y = INT_MAX;
    minCoasterPoint.z = INT_MAX;

    maxCoasterPoint.x = INT_MIN;
    maxCoasterPoint.y = INT_MIN;
    maxCoasterPoint.z = INT_MIN;


    printf("filling the array with control points:\n");
    for (int i = 0; i < numControlPoints; i++) {
        char* line = lines[i];
        printf("line:\n%s\n", line);

        char* xStr = strtok(line, " ,");
        char* yStr = strtok(NULL, " ,");
        char* zStr = strtok(NULL, " ,");

        double x = 0.0, y = 0.0, z = 0.0;
        int convertSuccessful = 0;

        convertSuccessful = sscanf(xStr, "%lf", &x);
        if (!convertSuccessful){
            printf("Failed to parse x value \"%s\" to a number (control point %d)\n", xStr, i+1);
            exit(1);
        }

        convertSuccessful = sscanf(yStr, "%lf", &y);
        if (!convertSuccessful){
            printf("Failed to parse y value \"%s\" to a number (control point %d)\n", yStr, i+1);
            exit(1);
        }

        convertSuccessful = sscanf(zStr, "%lf", &z);
        if (!convertSuccessful){
            printf("Failed to parse z value \"%s\" to a number (control point %d)\n", zStr, i+1);
            exit(1);
        }

        if (strtok(NULL, " ,") != NULL){
            printf("Too many control point dimensions were specified for control point %d. Max = 3\n", i+1);
            exit(1);
        }

        printf("\tx: %.2f\n", x);
        printf("\ty: %.2f\n", y);
        printf("\tz: %.2f\n\n", z);

        controlPoints[i].x = x;
        controlPoints[i].y = y;
        controlPoints[i].z = z;

        if(i < numControlPoints - 3){
            // don't add the last 3 control points to the total since they are repeated and will skew the midpoint
            coasterMidPoint.x += x;
            coasterMidPoint.y += y;
            coasterMidPoint.z += z;

            if (x > maxCoasterPoint.x) maxCoasterPoint.x = x;
            if (y > maxCoasterPoint.y) maxCoasterPoint.y = y;
            if (z > maxCoasterPoint.z) maxCoasterPoint.z = z;

            if (x < minCoasterPoint.x) minCoasterPoint.x = x;
            if (y < minCoasterPoint.y) minCoasterPoint.y = y;
            if (z < minCoasterPoint.z) minCoasterPoint.z = z;
        }

        free(lines[i]);
    }

    // find the midpoint of the roller coaster (ignoring the last 3 control points which are repeated from the beginning)
    coasterMidPoint.x /= (numControlPoints - 3);
    coasterMidPoint.y /= (numControlPoints - 3);
    coasterMidPoint.z /= (numControlPoints - 3);

    printf("minCoasterPoint: (%f, %f, %f)\n", minCoasterPoint.x, minCoasterPoint.y, minCoasterPoint.z);
    printf("maxCoasterPoint: (%f, %f, %f)\n", maxCoasterPoint.x, maxCoasterPoint.y, maxCoasterPoint.z);

}


/*
 *  Increment the cart's u value, move to the next spline if u is not in [0, 1), 
 *  and update the cart's position
 */
void
move_cart(){
    //double wKin = 0.5 * cart.mass * pow(cart.speed, 2);
    //double wPot = cart.mass * GRAVITY * cart.pos.y;
    //double wTotal = wKin + wPot;
    //cart.speed = MAX(MIN_CART_SPEED, sqrt(2 * ((wTotal / cart.mass) - (GRAVITY * cart.pos.y))) );

    cart.u += cart.speed * TIME_DELTA;
    while (cart.u >= 1.0){
        cart.u -= 1.0;
        cart.curveIndex = (cart.curveIndex + 1) % numKnotValues;
    }

    cart.pos = q_func(cart.u, cart.curveIndex);
    cart.pos.y += cart.height;
}

/**
 *  clamps value between min and max
 */
double 
clamp (double value, double min, double max){
    value = value <= max ? value : max;
    value = value >= min ? value : min;
    return value;
}

/**
 *  Normalizes the argument vector and returns it
 */
Point
normalize(Point vector){
    Point norm_vec;
    double vector_sqrt = sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
    norm_vec.x = -vector.x / vector_sqrt;
    norm_vec.y = -vector.y / vector_sqrt;
    norm_vec.z = -vector.z / vector_sqrt;
    return norm_vec;
}

/**
 *  Computes the cross product of vectors A and B and returns it
 */
Point
cross_product(Point vectorA, Point vectorB){
    Point vectorCross;
    vectorCross.x = vectorA.y * vectorB.z - vectorA.z * vectorB.y;
    vectorCross.y = vectorA.z * vectorB.x - vectorA.x * vectorB.z; 
    vectorCross.z = vectorA.x * vectorB.y - vectorA.y * vectorB.x;
    return vectorCross;
}