#include "my_algo.h"

#define PI 3.1415926

// 当x逐像素增长时，y的坐标值也随之增长，x轴每前进一个像素，y轴前进dy/dx像素。error可以认为是当y轴累计的前进值。
// 当该值大于0.5时，我们可以认为当前像素点已经进入了屏幕像素矩阵的下一“行”，于是绘制的y值+=1。
// 当网格中心点的y值定义为0时，y在一个网格的取值范围为[-0.5, 0.5)，在进入下一个网格后，y值更新为在当前网格的值，故需要使error-=1.
// 摆脱了 float 计算，将 0.5 化成 1
// k (dy / dx) >= 0.5
// 2k (dy / dx) >= 1
// k * 2dy >= dx
void line(int x0, int y0, int x1, int y1, TGAImage& image, TGAColor color) {
    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dy = y1 - y0;
    int dx = x1 - x0;
    int derror = 2 * std::abs(dy);
    int error = 0;
    int y = y0;
    for (int x = x0; x <= x1; ++x) {
        if (steep)
            image.set(y, x, color);
        else
            image.set(x, y, color);
        error += derror;
        if (error >= dx) {
            error -= dx * 2;
            y += (y0 > y1 ? -1 : 1);
        }
    }
}

// 求点的重心坐标，以 [1 - u - v, u, v] 形式返回
// 点在三角形内部，则三个坐标都应大于等于0
static vec3 barycentric(std::vector<vec2> &points, vec2 P) {
    float v = ((points[0].y - points[1].y) * P.x + (points[1].x - points[0].x) * P.y + points[0].x * points[1].y - points[1].x * points[0].y) / ((points[0].y - points[1].y) * points[2].x + (points[1].x - points[0].x) * points[2].y + points[0].x * points[1].y - points[1].x * points[0].y);
    float u = ((points[0].y - points[2].y) * P.x + (points[2].x - points[0].x) * P.y + points[0].x * points[2].y - points[2].x * points[0].y) / ((points[0].y - points[2].y) * points[1].x + (points[2].x - points[0].x) * points[1].y + points[0].x * points[2].y - points[2].x * points[0].y);
    return { 1 - u - v, u, v };
}

// 重心坐标画三角形
void triangle(std::vector<vec3> &points, float *z_buffer, TGAImage& image, TGAColor color, int width) {
    vec2 bottom_left = { 0, 0 };
    vec2 up_right = { image.width() - 1, image.height() - 1 };
    for (int i = 0; i < 3; ++i) {
        bottom_left.x = std::max(0.0, std::min(bottom_left.x, points[i].x));
        bottom_left.y = std::max(0.0, std::min(bottom_left.y, points[i].y));
        up_right.x = std::min(static_cast<double>(image.width() - 1), std::max(up_right.x, points[i].x));
        up_right.y = std::min(static_cast<double>(image.height() - 1), std::max(up_right.y, points[i].y));
    }
    vec2 P;
    for (P.x = bottom_left.x; P.x <= up_right.x; ++P.x) {
        for (P.y = bottom_left.y; P.y <= up_right.y; ++P.y) {
            std::vector<vec2> temp(3);
            for (int i = 0; i < 3; ++i)
                temp[i] = proj<2, 3>(points[i]);
            vec3 bary = barycentric(temp, P);
            if (bary.x < 0 || bary.y < 0 || bary.z < 0)
                continue;
            float z = 0;
            for (int i = 0; i < 3; ++i)
                z += points[i][2] * bary[i];
            if (z_buffer[static_cast<int>(P.x + P.y * width)] < z) {
                z_buffer[static_cast<int>(P.x + P.y * width)] = z;
                image.set(P.x, P.y, color);
            }
        }
    }
}

mat4 rotation(float radian, vec3 axis) {
    mat4 ans;
    float s = std::sin(radian), c = std::cos(radian);
    axis = axis.normalized();
    ans[0][0] = axis.x * axis.x * (1 - c) + c;
    ans[0][1] = axis.x * axis.y * (1 - c) + axis.z * s;
    ans[0][2] = axis.x * axis.z * (1 - c) - axis.y * s;
    ans[1][0] = axis.x * axis.y * (1 - c) - axis.z * s;
    ans[1][1] = axis.y * axis.y * (1 - c) + c;
    ans[1][2] = axis.z * axis.y * (1 - c) + axis.x * s;
    ans[2][0] = axis.x * axis.z * (1 - c) + axis.y * s;
    ans[2][1] = axis.z * axis.y * (1 - c) - axis.x * s;
    ans[2][2] = axis.z * axis.z * (1 - c) + c;
    ans[3][3] = 1;
    return ans;
}

mat4 translation(float x, float y, float z) {
    mat4 ans = mat4::identity();
    ans[0][3] = x;
    ans[1][3] = y;
    ans[2][3] = z;
    return ans;
}

mat4 zoom(float x_factor, float y_factor, float z_factor) {
    mat4 ans;
    ans[0][0] = x_factor;
    ans[1][1] = y_factor;
    ans[2][2] = z_factor;
    ans[3][3] = 1;
    return ans;
}

float angle2radian(float angle) {
    return angle / 180 * PI;
}

mat4 lookAt(vec3 eye, vec3 center, vec3 up) {
    mat4 ans = mat4::identity();
    vec3 f = (center - eye).normalized();
    vec3 s = cross(f, up).normalized();
    vec3 u = cross(s, f);
    ans[0][0] = s.x;
    ans[0][1] = s.y;
    ans[0][2] = s.z;
    ans[1][0] = u.x;
    ans[1][1] = u.y;
    ans[1][2] = u.z;
    ans[2][0] = -f.x;
    ans[2][1] = -f.y;
    ans[2][2] = -f.z;
    ans[0][3] = -(s * eye);
    ans[1][3] = -(u * eye);
    ans[2][3] = f * eye;
    return ans;
}

mat4 perspective(float fovY, float aspect, float zNear, float zFar) {
    mat4 ans;
    float t = std::tan(aspect / 2);
    ans[0][0] = 1.0f / aspect / t;
    ans[1][1] = 1.0f / t;
    ans[2][2] = (zNear + zFar) / (zNear - zFar);
    ans[2][3] = 2 * zNear * zFar / (zNear - zFar);
    ans[3][2] = -1;
    return ans;
}

mat4 ortho(float left, float right, float bottom, float up, float front, float back) {
    mat4 ans;
    ans[0][0] = 2.0f / (right - left);
    ans[1][1] = 2.0f / (up - bottom);
    ans[2][2] = -2.0f / (back - front);
    ans[0][3] = -(right + left) / (right - left);
    ans[1][3] = -(up + bottom) / (up - bottom);
    ans[2][3] = -(front + back) / (back - front);
    return ans;
}

void triangle(std::vector<vec4> points, IShader& shader, TGAImage& image, float zbuffer[]) {
// 记录 w 分量的值，用于透视矫正，并将 xyz 除以 w
    vec3 correct;
    for (int i = 0; i < 3; ++i) {
        correct[i] = points[i][3];
        for (int j = 0; j < 4; ++j)
            points[i][j] /= points[i][3];
    }
// 投影到屏幕空间
    mat4 viewPort = viewport(image.width(), image.height(), 255);
    float width = image.width(), height = image.height();
    for (int i = 0; i < 3; ++i) {
        points[i].x = points[i].x * width / 2 + width / 2;
        points[i].y = points[i].y * height / 2 + height / 2;
    }

// 计算包围盒
    vec2 bottom_left = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    vec2 up_right = { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() };
    for (int i = 0; i < 3; ++i) {
        bottom_left.x = std::min(bottom_left.x, points[i].x);
        bottom_left.y = std::min(bottom_left.y, points[i].y);
        up_right.x = std::max(up_right.x, points[i].x);
        up_right.y = std::max(up_right.y, points[i].y);
    }
// 遍历每一个点
    TGAColor color;
    std::vector<vec2> temp(3);
    for (int i = 0; i < 3; ++i)
        temp[i] = proj<2, 4>(points[i]);
    for (int x = std::max(bottom_left.x, 0.0); x < std::min(static_cast<int>(up_right.x) + 1, image.width()); ++x) {
        for (int y = std::max(bottom_left.y, 0.0); y < std::min(static_cast<int>(up_right.y) + 1, image.height()); ++y) {
// 重心坐标
            vec3 bary = barycentric(temp, {static_cast<double>(x), static_cast<double>(y)});
            if (bary.x < 0 || bary.y < 0 || bary.z < 0)
                continue;
// 透视矫正
            for (int i = 0; i < 3; ++i)
                bary[i] /= correct[i];
            float sum = bary[0] + bary[1] + bary[2];
            for (int i = 0; i < 3; ++i)
                bary[i] /= sum;
// 插值计算深度，取颜色
            float z1 = 0;
            for (int i = 0; i < 3; ++i) {
                z1 += points[i].z * bary[i];
            }
            z1 = -z1;
            if (zbuffer[x + y * image.height()] > z1) {
                continue;
            }
                
            if (!shader.fragment(bary, color)) {
                zbuffer[x + y * image.height()] = z1;
                image.set(x, y, color);
            }
        }
    }
}

static float cofactor(int x, int y, const mat3& matrix) {
    int count = 0;
    float nums[4] = { 0 };
    for (int i = 0; i < 3; ++i) {
        if (i == x)
            continue;
        for (int j = 0; j < 3; ++j) {
            if (j != y)
                nums[count++] = matrix[i][j];
        }
    }
    return ((x + y) & 1) ? -(nums[0] * nums[3] - nums[2] * nums[1]) : (nums[0] * nums[3] - nums[2] * nums[1]);
}

mat3 adjointMatrix(const mat3& matrix) {
    mat3 ans;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            ans[j][i] = cofactor(i, j, matrix);
    }
    return ans;
}

mat4 viewport(float width, float height, float deepth) {
    mat4 ans = mat4::identity();
    ans[0][3] = ans[0][0] = width / 2;
    ans[1][3] = ans[1][1] = height / 2;
    ans[2][2] = deepth;
    return ans;
}

mat3 getTBN(vec3 E1, vec3 E2, vec2 delt_UV1, vec2 delt_UV2, vec3 N) {

}