 #include <GL/gl.h>
#include <GL/glu.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <GLFW/glfw3.h>
#include "check_gl.h"
#include <array>
#include <random>
#include <algorithm>
#include <cmath>
// 新增头文件：<algorithm> 提供 std::max，<cmath> 提供 std::exp

/* #define IS3D */
#define FULLSCRN
static const float pointsz = 16.0f;
static const float fade = 1.5f;
static const float G = 0.1f;
static const size_t nstars = 3;
static float aspect = 1.0f;

// 鼠标小球
static const float mouseRadius = 0.6f;   // 排斥半径（世界坐标）
static const float mouseRepel  = 2.5f;   // 排斥强度
static glm::vec2 mouse_world(0.0f);      // 鼠标在世界坐标中的位置
static float winW = 640.0f, winH = 640.0f;
// 新增：鼠标小球的状态变量（半径、强度、世界坐标、窗口尺寸）

struct Star {
#ifdef IS3D
    glm::vec3 pos;
    glm::vec3 vel;
#else
    glm::vec2 pos;
    glm::vec2 vel;
#endif
};

using StarArray = std::array<Star, nstars>;
StarArray stars;

void init() {
    CHECK_GL(glEnable(GL_POINT_SMOOTH));
    CHECK_GL(glEnable(GL_BLEND));
    CHECK_GL(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    auto seed = std::random_device{}();
    printf("Seed: %u\n", seed);
    auto rng = std::mt19937(seed);
    auto unif = std::uniform_real_distribution<float>(-1.0f, 1.0f);
    auto unif2 = std::uniform_real_distribution<float>(-0.3f, 0.3f);
    for (size_t i = 0; i < stars.size(); i++) {
        for (size_t j = 0; j < stars[i].pos.length(); j++) {
            stars[i].pos[j] = unif(rng);
            stars[i].vel[j] = unif2(rng);
        }
        stars[i].pos.x *= aspect;
    }
}

void update_mouse(GLFWwindow *window) {
    double cx = 0.0, cy = 0.0;
    glfwGetCursorPos(window, &cx, &cy);
    // 屏幕像素 -> 归一化 [-1,1]；OpenGL 的 y 朝上，屏幕 y 朝下，需翻转
    float nx = (float)(cx / winW) * 2.0f - 1.0f;
    float ny = 1.0f - (float)(cy / winH) * 2.0f;
    mouse_world = glm::vec2(nx * aspect, ny);
}
// 每帧把光标像素坐标换算成世界坐标，更新 mouse_world

void render(float dt) {
    CHECK_GL(glColor4f(0.0f, 0.0f, 0.1f, 1.0f - std::exp(-fade * dt)));
    CHECK_GL(glRectf(-1.0f, -1.0f, 1.0f, 1.0f));

    CHECK_GL(glPointSize(pointsz));
    glBegin(GL_POINTS);
    glColor4f(0.9f, 0.8f, 0.7f, 1.0f);
    for (size_t i = 0; i < stars.size(); i++) {
#ifdef IS3D
        auto p = stars[i].pos;
#else
        auto p = glm::vec3(stars[i].pos, 0.0f);
#endif
        glVertex3f(p.x / aspect, p.y, p.z);
    }
    CHECK_GL(glEnd());

    // 画鼠标小球
    CHECK_GL(glColor4f(0.3f, 0.9f, 1.0f, 1.0f));
    glBegin(GL_POINTS);
    glVertex3f(mouse_world.x / aspect, mouse_world.y, 0.0f);
    CHECK_GL(glEnd());
    // 把鼠标小球画出来（青色点）
}

float energy() {
    auto potE = 0.0f;
    for (size_t i = 0; i < stars.size(); i++) {
        for (size_t j = 0; j < stars.size(); j++) {
            if (i == j) continue;
            auto p1 = stars[i].pos;
            auto p2 = stars[j].pos;
            // 引力势能（负）：E = -G SUM mi*mj/r_ij，r_ij 是i,j两质点距离
            potE += 0.5f * G / glm::length(p2 - p1);
        }
    }
    auto movE = 0.0f;
    for (size_t i = 0; i < stars.size(); i++) {
        auto v = glm::length(stars[i].vel);
        // 动能（正）：E = 0.5mv^2，v 是质点运动速度，m 是质量
        movE += 0.5f * v * v;
    }
    return movE - potE;
}

// Solve:
// y' = f(y)
// RK1:
// k = f(y)
// RK4:
// k1 = f(y)
// k2 = f(y + k1/2 dt)
// k3 = f(y + k2/2 dt)
// k4 = f(y + k3 dt)
// k = 1/6 (k1 + 2 k2 + 2 k3 + k4)

void derivative(StarArray const &stars, StarArray &d_stars) {
    for (size_t i = 0; i < stars.size(); i++) {
        auto acc = decltype(+stars[0].pos)();
        for (size_t j = 0; j < stars.size(); j++) {
            if (i == j) continue;
            auto p1 = stars[i].pos;
            auto p2 = stars[j].pos;
            auto r12 = p2 - p1;
            auto rlen = std::max(0.05f, glm::length(r12));
            auto rnorm = r12 / rlen;
            // 引力大小为：F = G*m1*m2/(|r|*|r|)，方向为 -r，其中 r 为两个质点之间距离，r = p1 - p2
            auto force = G / (rlen * rlen);
            // F = ma, a = F/m
            acc += force * rnorm;
        }
        // 鼠标排斥：靠近鼠标的质点被推开（越近越强，方向远离鼠标）
        {
            glm::vec2 toMouse = stars[i].pos - mouse_world;
            float d = glm::length(toMouse);
            if (d < mouseRadius && d > 1e-6f) {
                float strength = mouseRepel * (1.0f - d / mouseRadius);
                acc += (toMouse / d) * strength;
            }
        }
        // 靠近鼠标的质点会被推开
        d_stars[i].vel = acc;
        d_stars[i].pos = stars[i].vel;
    }
}

void integrate(StarArray &out_stars, StarArray const &stars, StarArray const &d_stars, float dt) {
    for (size_t i = 0; i < stars.size(); i++) {
        out_stars[i].pos = stars[i].pos + d_stars[i].pos * dt;
        out_stars[i].vel = stars[i].vel + d_stars[i].vel * dt;
    }
}

void integrate(StarArray &stars, StarArray const &d_stars, float dt) {
    for (size_t i = 0; i < stars.size(); i++) {
        stars[i].pos += d_stars[i].pos * dt;
        stars[i].vel += d_stars[i].vel * dt;
    }
}

void fixbounds(StarArray &stars) {
    for (size_t i = 0; i < stars.size(); i++) {
        if (stars[i].pos.x < -aspect && stars[i].vel.x < 0.0f
            || stars[i].pos.x > aspect && stars[i].vel.x > 0.0f) {
            stars[i].vel.x = -stars[i].vel.x;
        }
        if (stars[i].pos.y < -1.0f && stars[i].vel.y < 0.0f
            || stars[i].pos.y > 1.0f && stars[i].vel.y > 0.0f) {
            stars[i].vel.y = -stars[i].vel.y;
        }
#ifdef IS3D
        if (stars[i].pos.z < -1.0f && stars[i].vel.z < 0.0f
            || stars[i].pos.z > 1.0f && stars[i].vel.z > 0.0f) {
            stars[i].vel.z = -stars[i].vel.z;
        }
#endif
    }
}

void substep_rk4(float dt) {
    StarArray tmp;
    StarArray k1, k2, k3, k4, k;
    derivative(stars, k1);                  
    integrate(tmp, stars, k1, dt / 2.0f);   
    derivative(tmp, k2);                    
    integrate(tmp, stars, k2, dt / 2.0f);   
    derivative(tmp, k3);                    
    integrate(tmp, stars, k3, dt);          
    derivative(tmp, k4);                   
    k = StarArray();                        
    integrate(k, k1, 1.0f / 6.0f);          
    integrate(k, k2, 1.0f / 3.0f);          
    integrate(k, k3, 1.0f / 3.0f);         
    integrate(k, k4, 1.0f / 6.0f);          
    integrate(stars, k, dt);
    fixbounds(stars);
}

void substep_rk1(float dt) {
    StarArray k;
    derivative(stars, k);
    integrate(stars, k, dt);
    fixbounds(stars);
}

void advance(float dt) {
    size_t n = 100;
    for (size_t i = 0; i < n; i++) {
        substep_rk4(dt / n);
    }
    /* printf("Energy: %f\n", energy()); */
}

int main() {
    if (!glfwInit()) {
        return -1;
    }
#ifdef FULLSCRN
    GLFWmonitor *monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode *mode = glfwGetVideoMode(monitor);
    GLFWwindow *window = glfwCreateWindow(mode->width, mode->height, "Trisolar", NULL, NULL);
    glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
    aspect = (float)mode->width / (float)mode->height;
    winW = (float)mode->width;
    winH = (float)mode->height;
#else
    GLFWwindow *window = glfwCreateWindow(640, 640, "Trisolar", NULL, NULL);
    winW = 640.0f;
    winH = 640.0f;
#endif
// 记录窗口宽高，供 update_mouse 换算坐标用
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    init();
    // 隐藏系统光标，让鼠标小球就是你的"指针"（想还原系统光标就删掉这行）
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
    // 隐藏系统光标，让青色小球充当指针
    double t0 = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        /* CHECK_GL(glClear(GL_COLOR_BUFFER_BIT)); */
        update_mouse(window);
        // 每帧更新鼠标世界坐标
        double t1 = glfwGetTime();
        float dt = (float)(t1 - t0);
        render(dt);
        advance(dt);
        t0 = t1;
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}