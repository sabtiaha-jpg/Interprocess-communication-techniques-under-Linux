#include <GL/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "robot.h"

// Global variables
static Robot* robots = NULL;
static int num_robots = 0;
static Grid3D* grid = NULL;
static int current_step = 0;
static int max_steps = 0;
static int animation_speed = 50;  // ms per step
static bool paused = false;
static bool show_grid = true;
static bool show_paths = true;
static bool show_survivors = true;
static float grid_scale = 1.0f;

// Camera variables
static float camera_angle_x = 45.0f;
static float camera_angle_y = 45.0f;
static float camera_distance = 50.0f;
static int last_mouse_x, last_mouse_y;
static bool mouse_left_down = false;
static bool mouse_right_down = false;

// Colors
typedef struct {
    float r, g, b;
} Color;

Color robot_colors[] = {
    {1.0f, 0.0f, 0.0f},  // Red
    {0.0f, 1.0f, 0.0f},  // Green
    {0.0f, 0.0f, 1.0f},  // Blue
    {1.0f, 1.0f, 0.0f},  // Yellow
    {1.0f, 0.0f, 1.0f},  // Magenta
    {0.0f, 1.0f, 1.0f},  // Cyan
    {1.0f, 0.5f, 0.0f},  // Orange
    {0.5f, 0.0f, 0.5f},  // Purple
    {0.5f, 0.5f, 0.5f},  // Gray
    {0.0f, 0.5f, 0.0f}   // Dark Green
};

void draw_cube(float x, float y, float z, float size, Color color) {
    glPushMatrix();
    glTranslatef(x, y, z);
    glColor3f(color.r, color.g, color.b);
    
    float s = size / 2.0f;
    
    // Front face
    glBegin(GL_QUADS);
    glVertex3f(-s, -s, s);
    glVertex3f(s, -s, s);
    glVertex3f(s, s, s);
    glVertex3f(-s, s, s);
    
    // Back face
    glVertex3f(-s, -s, -s);
    glVertex3f(-s, s, -s);
    glVertex3f(s, s, -s);
    glVertex3f(s, -s, -s);
    
    // Top face
    glVertex3f(-s, s, -s);
    glVertex3f(-s, s, s);
    glVertex3f(s, s, s);
    glVertex3f(s, s, -s);
    
    // Bottom face
    glVertex3f(-s, -s, -s);
    glVertex3f(s, -s, -s);
    glVertex3f(s, -s, s);
    glVertex3f(-s, -s, s);
    
    // Right face
    glVertex3f(s, -s, -s);
    glVertex3f(s, s, -s);
    glVertex3f(s, s, s);
    glVertex3f(s, -s, s);
    
    // Left face
    glVertex3f(-s, -s, -s);
    glVertex3f(-s, -s, s);
    glVertex3f(-s, s, s);
    glVertex3f(-s, s, -s);
    glEnd();
    
    glPopMatrix();
}

void draw_sphere(float x, float y, float z, float radius, Color color) {
    glPushMatrix();
    glTranslatef(x, y, z);
    glColor3f(color.r, color.g, color.b);
    glutSolidSphere(radius, 10, 10);
    glPopMatrix();
}

void draw_cylinder(float x1, float y1, float z1, 
                   float x2, float y2, float z2, 
                   float radius, Color color) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    float length = sqrt(dx*dx + dy*dy + dz*dz);
    
    if (length < 0.001) return;
    
    glPushMatrix();
    
    // Move to start point
    glTranslatef(x1, y1, z1);
    
    // Rotation to align with direction vector
    float angle = acos(dz / length) * 180.0 / M_PI;
    float axis_x = -dy;
    float axis_y = dx;
    
    if (fabs(angle) > 0.001) {
        glRotatef(angle, axis_x, axis_y, 0.0f);
    }
    
    glColor3f(color.r, color.g, color.b);
    
    // Draw cylinder
    GLUquadric* quadric = gluNewQuadric();
    gluCylinder(quadric, radius, radius, length, 10, 10);
    gluDeleteQuadric(quadric);
    
    glPopMatrix();
}

void draw_grid_3d() {
    if (!grid || !show_grid) return;
    
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_LINES);
    
    // Draw grid lines in X direction
    for (int x = 0; x <= grid->size_x; x++) {
        for (int y = 0; y <= grid->size_y; y++) {
            glVertex3f(x * grid_scale, y * grid_scale, 0);
            glVertex3f(x * grid_scale, y * grid_scale, grid->size_z * grid_scale);
        }
    }
    
    // Draw grid lines in Y direction
    for (int y = 0; y <= grid->size_y; y++) {
        for (int z = 0; z <= grid->size_z; z++) {
            glVertex3f(0, y * grid_scale, z * grid_scale);
            glVertex3f(grid->size_x * grid_scale, y * grid_scale, z * grid_scale);
        }
    }
    
    // Draw grid lines in Z direction
    for (int z = 0; z <= grid->size_z; z++) {
        for (int x = 0; x <= grid->size_x; x++) {
            glVertex3f(x * grid_scale, 0, z * grid_scale);
            glVertex3f(x * grid_scale, grid->size_y * grid_scale, z * grid_scale);
        }
    }
    glEnd();
}

void draw_obstacles() {
    if (!grid) return;
    
    Color obstacle_color = {0.2f, 0.2f, 0.2f};
    
    for (int z = 0; z < grid->size_z; z++) {
        for (int y = 0; y < grid->size_y; y++) {
            for (int x = 0; x < grid->size_x; x++) {
                if (grid->cells[z][y][x]->type == CELL_OBSTACLE) {
                    draw_cube(x * grid_scale + grid_scale/2,
                             y * grid_scale + grid_scale/2,
                             z * grid_scale + grid_scale/2,
                             grid_scale * 0.9f, obstacle_color);
                }
            }
        }
    }
}

void draw_survivors() {
    if (!grid || !show_survivors) return;
    
    for (int z = 0; z < grid->size_z; z++) {
        for (int y = 0; y < grid->size_y; y++) {
            for (int x = 0; x < grid->size_x; x++) {
                if (grid->cells[z][y][x]->type == CELL_SURVIVOR) {
                    // Color based on priority
                    float priority = grid->cells[z][y][x]->survivor_priority;
                    float intensity = 0.3f + (priority / 10.0f) * 0.7f;
                    
                    Color survivor_color = {0.0f, intensity, 0.0f};  // Green based on priority
                    
                    draw_sphere(x * grid_scale + grid_scale/2,
                               y * grid_scale + grid_scale/2,
                               z * grid_scale + grid_scale/2,
                               grid_scale * 0.3f, survivor_color);
                }
            }
        }
    }
}

void draw_base_station() {
    if (!grid) return;
    
    Color base_color = {0.8f, 0.8f, 0.0f};  // Yellow
    draw_cube(grid->base_station.x * grid_scale + grid_scale/2,
              grid->base_station.y * grid_scale + grid_scale/2,
              grid->base_station.z * grid_scale + grid_scale/2,
              grid_scale * 1.2f, base_color);
}

void draw_robot_paths() {
    if (!robots || !show_paths) return;
    
    for (int i = 0; i < num_robots; i++) {
        if (!robots[i].active) continue;
        
        Chromosome* mission = &robots[i].assigned_mission;
        if (!mission->actual_path || mission->path_length < 2) continue;
        
        Color path_color = robot_colors[i % 10];
        path_color.r *= 0.7f;
        path_color.g *= 0.7f;
        path_color.b *= 0.7f;
        
        glColor3f(path_color.r, path_color.g, path_color.b);
        glBegin(GL_LINE_STRIP);
        
        // Draw complete path
        for (int j = 0; j < mission->path_length; j++) {
            Coordinate coord = mission->actual_path[j];
            glVertex3f(coord.x * grid_scale + grid_scale/2,
                      coord.y * grid_scale + grid_scale/2,
                      coord.z * grid_scale + grid_scale/2);
        }
        glEnd();
    }
}

void draw_robots() {
    if (!robots) return;
    
    for (int i = 0; i < num_robots; i++) {
        if (!robots[i].active) continue;
        
        Color robot_color = robot_colors[i % 10];
        Coordinate pos = robots[i].current_position;
        
        // Draw robot body
        draw_cube(pos.x * grid_scale + grid_scale/2,
                 pos.y * grid_scale + grid_scale/2,
                 pos.z * grid_scale + grid_scale/2,
                 grid_scale * 0.6f, robot_color);
        
        // Draw robot ID
        glPushMatrix();
        glTranslatef(pos.x * grid_scale + grid_scale/2,
                    pos.y * grid_scale + grid_scale/2,
                    pos.z * grid_scale + grid_scale + 0.5f);
        glColor3f(1.0f, 1.0f, 1.0f);
        glRasterPos3f(0, 0, 0);
        char id_str[10];
        sprintf(id_str, "R%d", robots[i].id);
        for (int j = 0; id_str[j]; j++) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, id_str[j]);
        }
        glPopMatrix();
        
        // Draw battery indicator
        float battery_level = robots[i].battery_level / 100.0f;
        Color battery_color;
        if (battery_level > 0.5) battery_color = (Color){0.0f, 1.0f, 0.0f};
        else if (battery_level > 0.2) battery_color = (Color){1.0f, 1.0f, 0.0f};
        else battery_color = (Color){1.0f, 0.0f, 0.0f};
        
        glPushMatrix();
        glTranslatef(pos.x * grid_scale + grid_scale/2,
                    pos.y * grid_scale - 0.3f,
                    pos.z * grid_scale + grid_scale/2);
        
        // Battery outline
        glColor3f(0.5f, 0.5f, 0.5f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(-0.3f, 0.0f, -0.1f);
        glVertex3f(0.3f, 0.0f, -0.1f);
        glVertex3f(0.3f, 0.0f, 0.1f);
        glVertex3f(-0.3f, 0.0f, 0.1f);
        glEnd();
        
        // Battery level
        glColor3f(battery_color.r, battery_color.g, battery_color.b);
        glBegin(GL_QUADS);
        float battery_width = 0.6f * battery_level;
        glVertex3f(-0.3f, 0.0f, -0.09f);
        glVertex3f(-0.3f + battery_width, 0.0f, -0.09f);
        glVertex3f(-0.3f + battery_width, 0.0f, 0.09f);
        glVertex3f(-0.3f, 0.0f, 0.09f);
        glEnd();
        glPopMatrix();
        
        // Draw survivors being carried
        if (robots[i].survivors_delivered > 0) {
            glPushMatrix();
            glTranslatef(pos.x * grid_scale + grid_scale/2,
                        pos.y * grid_scale + grid_scale/2 + 0.5f,
                        pos.z * grid_scale + grid_scale/2);
            
            char survivors_str[20];
            sprintf(survivors_str, "S:%d", robots[i].survivors_delivered);
            glColor3f(0.0f, 1.0f, 0.0f);
            glRasterPos3f(0, 0, 0);
            for (int j = 0; survivors_str[j]; j++) {
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, survivors_str[j]);
            }
            glPopMatrix();
        }
    }
}

void draw_robot_trails() {
    if (!robots) return;
    
    for (int i = 0; i < num_robots; i++) {
        if (!robots[i].active) continue;
        
        Chromosome* mission = &robots[i].assigned_mission;
        if (!mission->actual_path || mission->path_length < 2) continue;
        
        Color trail_color = robot_colors[i % 10];
        
        // Draw trail up to current step
        int steps_to_draw = (current_step < mission->path_length) ? current_step : mission->path_length;
        
        glColor3f(trail_color.r, trail_color.g, trail_color.b);
        glBegin(GL_LINE_STRIP);
        
        for (int j = 0; j < steps_to_draw; j++) {
            Coordinate coord = mission->actual_path[j];
            glVertex3f(coord.x * grid_scale + grid_scale/2,
                      coord.y * grid_scale + grid_scale/2,
                      coord.z * grid_scale + grid_scale/2);
        }
        glEnd();
    }
}

void draw_hud() {
    // Switch to 2D projection for HUD
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, 800, 0, 600);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    // Disable depth test for HUD
    glDisable(GL_DEPTH_TEST);
    
    // Draw semi-transparent background for text
    glColor4f(0.0f, 0.0f, 0.0f, 0.5f);
    glBegin(GL_QUADS);
    glVertex2f(10, 590);
    glVertex2f(300, 590);
    glVertex2f(300, 400);
    glVertex2f(10, 400);
    glEnd();
    
    // Draw text
    glColor3f(1.0f, 1.0f, 1.0f);
    
    // Step info
    glRasterPos2f(20, 580);
    char step_str[100];
    sprintf(step_str, "Step: %d / %d", current_step, max_steps);
    for (int i = 0; step_str[i]; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, step_str[i]);
    }
    
    // Robot info
    int y_pos = 550;
    for (int i = 0; i < num_robots; i++) {
        if (!robots[i].active) continue;
        
        glColor3f(robot_colors[i % 10].r, 
                  robot_colors[i % 10].g, 
                  robot_colors[i % 10].b);
        
        char robot_info[100];
        sprintf(robot_info, "Robot %d: Pos(%d,%d,%d) Bat:%.1f%% Sur:%d", 
                robots[i].id,
                robots[i].current_position.x,
                robots[i].current_position.y,
                robots[i].current_position.z,
                robots[i].battery_level,
                robots[i].survivors_delivered);
        
        glRasterPos2f(20, y_pos);
        for (int j = 0; robot_info[j]; j++) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, robot_info[j]);
        }
        y_pos -= 20;
    }
    
    // Controls info
    glColor3f(1.0f, 1.0f, 0.0f);
    glRasterPos2f(20, 150);
    char controls[] = "Controls: SPACE=pause/resume, R=reset, +/-=speed, ESC=exit";
    for (int i = 0; controls[i]; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, controls[i]);
    }
    
    // Status
    glColor3f(paused ? 1.0f : 0.0f, paused ? 0.0f : 1.0f, 0.0f);
    glRasterPos2f(20, 130);
    char status[20];
    sprintf(status, "Status: %s", paused ? "PAUSED" : "RUNNING");
    for (int i = 0; status[i]; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, status[i]);
    }
    
    // Restore 3D settings
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    // Set camera position
    glTranslatef(0, 0, -camera_distance);
    glRotatef(camera_angle_x, 1.0f, 0.0f, 0.0f);
    glRotatef(camera_angle_y, 0.0f, 1.0f, 0.0f);
    
    // Center the grid
    glTranslatef(-grid->size_x * grid_scale / 2,
                 -grid->size_y * grid_scale / 2,
                 -grid->size_z * grid_scale / 2);
    
    // Draw 3D grid
    if (show_grid) {
        draw_grid_3d();
    }
    
    // Draw obstacles
    draw_obstacles();
    
    // Draw survivors
    if (show_survivors) {
        draw_survivors();
    }
    
    // Draw base station
    draw_base_station();
    
    // Draw robot paths (complete)
    if (show_paths) {
        draw_robot_paths();
    }
    
    // Draw robot trails (up to current step)
    draw_robot_trails();
    
    // Draw robots
    draw_robots();
    
    // Draw HUD
    draw_hud();
    
    glutSwapBuffers();
}

void update_animation(int value) {
    if (!paused) {
        current_step++;
        if (current_step > max_steps) {
            current_step = max_steps;
        }
        
        // Update robot positions for current step
        for (int i = 0; i < num_robots; i++) {
            if (!robots[i].active) continue;
            
            Chromosome* mission = &robots[i].assigned_mission;
            if (mission->actual_path && current_step < mission->path_length) {
                robots[i].current_position = mission->actual_path[current_step];
            }
        }
    }
    
    glutPostRedisplay();
    glutTimerFunc(animation_speed, update_animation, 0);
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 27:  // ESC key
            exit(0);
            break;
        case ' ':  // Space bar
            paused = !paused;
            break;
        case 'r':
        case 'R':
            current_step = 0;
            break;
        case '+':
        case '=':
            animation_speed = (animation_speed > 10) ? animation_speed - 10 : 1;
            break;
        case '-':
        case '_':
            animation_speed += 10;
            break;
        case 'g':
        case 'G':
            show_grid = !show_grid;
            break;
        case 'p':
        case 'P':
            show_paths = !show_paths;
            break;
        case 's':
        case 'S':
            show_survivors = !show_survivors;
            break;
    }
    glutPostRedisplay();
}

void special_keys(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_UP:
            camera_angle_x -= 5.0f;
            break;
        case GLUT_KEY_DOWN:
            camera_angle_x += 5.0f;
            break;
        case GLUT_KEY_LEFT:
            camera_angle_y -= 5.0f;
            break;
        case GLUT_KEY_RIGHT:
            camera_angle_y += 5.0f;
            break;
        case GLUT_KEY_PAGE_UP:
            camera_distance -= 5.0f;
            if (camera_distance < 10.0f) camera_distance = 10.0f;
            break;
        case GLUT_KEY_PAGE_DOWN:
            camera_distance += 5.0f;
            break;
    }
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        mouse_left_down = (state == GLUT_DOWN);
        last_mouse_x = x;
        last_mouse_y = y;
    } else if (button == GLUT_RIGHT_BUTTON) {
        mouse_right_down = (state == GLUT_DOWN);
        last_mouse_x = x;
        last_mouse_y = y;
    }
}

void mouse_motion(int x, int y) {
    if (mouse_left_down) {
        camera_angle_y += (x - last_mouse_x) * 0.5f;
        camera_angle_x += (y - last_mouse_y) * 0.5f;
    } else if (mouse_right_down) {
        camera_distance += (y - last_mouse_y) * 0.1f;
        if (camera_distance < 10.0f) camera_distance = 10.0f;
        if (camera_distance > 200.0f) camera_distance = 200.0f;
    }
    
    last_mouse_x = x;
    last_mouse_y = y;
    glutPostRedisplay();
}

void reshape(int width, int height) {
    if (height == 0) height = 1;
    
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)width / (float)height, 0.1f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
}

void init_opengl() {
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(2.0f);
}

int find_max_path_length(Robot* robots, int num_robots) {
    int max_len = 0;
    for (int i = 0; i < num_robots; i++) {
        if (robots[i].active && robots[i].assigned_mission.path_length > max_len) {
            max_len = robots[i].assigned_mission.path_length;
        }
    }
    return max_len;
}

void visualize_robots_opengl(Robot* robot_array, int robot_count, Grid3D* grid_ptr) {
    robots = robot_array;
    num_robots = robot_count;
    grid = grid_ptr;
    
    // Calculate maximum path length
    max_steps = find_max_path_length(robots, num_robots);
    
    // Initialize GLUT
    int argc = 1;
    char* argv[1] = {(char*)"Robot Visualization"};
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("3D Robot Rescue Simulation - OpenGL");
    
    // Set callback functions
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special_keys);
    glutMouseFunc(mouse);
    glutMotionFunc(mouse_motion);
    
    // Initialize OpenGL
    init_opengl();
    
    // Start animation timer
    glutTimerFunc(animation_speed, update_animation, 0);
    
    printf("\n=== OpenGL Visualization Started ===\n");
    printf("Controls:\n");
    printf("  SPACE  : Pause/Resume animation\n");
    printf("  R      : Reset to step 0\n");
    printf("  +/-    : Increase/Decrease speed\n");
    printf("  G      : Toggle grid display\n");
    printf("  P      : Toggle path display\n");
    printf("  S      : Toggle survivors display\n");
    printf("  Arrow keys : Rotate camera\n");
    printf("  Mouse drag : Rotate/zoom\n");
    printf("  ESC    : Exit visualization\n");
    
    // Start main loop
    glutMainLoop();
}
