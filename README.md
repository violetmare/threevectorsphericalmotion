本项目基于archibate开源发布。在这个基础上增加了鼠标作为小球对于物体之间的引力和碰撞
1. 排斥力长什么样
我在 main.cpp:140-148 写的是这样一段：
glm::vec2 toMouse = stars[i].pos - mouse_world;  // 从鼠标指向星星的向量
float d = glm::length(toMouse);                  // 星星到鼠标的距离
if (d < mouseRadius && d > 1e-6f) {
    float strength = mouseRepel * (1.0f - d / mouseRadius);
    acc += (toMouse / d) * strength;
}
拆开看：
方向：toMouse / d 是「从鼠标指向星星」的单位向量（长度 1），所以推力方向 = 远离鼠标 
大小：mouseRepel * (1 - d / mouseRadius)，这是个线性衰减：
d = 0（正中心）	mouseRepel（最大）	贴脸时最猛
d = mouseRadius	0	刚好到边界，没力
d > mouseRadius	不触发	完全不管
选「线性衰减」而不是 1/d²，是因为 1/d² 在 d→0 时会趋向无穷大（星星会像被炸飞），而线性衰减到中心是有限值 mouseRepel，手感平滑可控。

3. 它怎么混进物理里的
关键在于：这个程序里所有力都统一写成「加速度」，然后交给牛顿第二定律 F = ma。
引力加速度（main.cpp:136）：force = G / r²，方向指向对方
鼠标斥力加速度：上面那段，方向远离鼠标
两者在 derivative() 里直接相加，得到这个星星此刻的总加速度 acc：
acc = Σ(引力) + 鼠标斥力
然后这个 acc 交给 RK4 积分器（main.cpp:188-205），去解这个微分方程组：
dx/dt = v          （位置的变化 = 速度）
dv/dt = acc        （速度的变化 = 加速度）
所以「鼠标靠近 → 加速度多一个远离的分量 → 速度朝远离方向变 → 位置就躲开了」，整个过程是顺理成章被积分出来的，不是硬把星星挪走，而是给它加了真实的"力"。这也是它看起来运动很自然的原因。

4. 为什么还要做坐标换算
因为 GLFW 给鼠标的位置是像素坐标（原点左上角，y 朝下），而星星活在世界坐标（原点中心，y 朝上）。两个坐标系都对不上，根本没法算「距离」。所以 main.cpp:59-66 每帧做三步：
cx / winW → 归一化到 [0,1]
*2 - 1 → 映射到 [-1,1]
1 - ... → 翻转 y（屏幕 y 朝下，OpenGL y 朝上）
换算完，鼠标和星星就共用一套坐标系，后面的距离计算、绘图才成立。
4. 每帧的执行顺序（串起来看）

主循环：
  update_mouse(window)   拿到鼠标位置，换算成世界坐标
  render(dt)             把星星 + 鼠标小球画出来
  advance(dt)            跑 100 次 RK4 子步，每次子步里：
  derivative() 里算引力 + 鼠标斥力 → 
