#include <limits>
#include <cmath>
#include <filesystem>
#include "my_algo.h"

constexpr int width  = 800; // output image size
constexpr int height = 800;
vec3 light_dir{1,1,1}; // light source
vec3       eye{0,0,3}; // camera position
vec3    center{0,0,0}; // camera direction
vec3        up{0,1,0}; // camera up vector

constexpr TGAColor white = { 255, 255, 255, 255 };
constexpr TGAColor red = { 0, 0, 255, 255 };

//struct Shader : IShader {
//    const Model &model;
//    vec3 uniform_l;       // light direction in view coordinates
//    mat<2,3> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
//    mat<3,3> varying_nrm; // normal per vertex to be interpolated by FS
//    mat<3,3> view_tri;    // triangle in view coordinates
//
//    Shader(const Model &m) : model(m) {
//        uniform_l = proj<3>((ModelView*embed<4>(light_dir, 0.))).normalized(); // transform the light vector to view coordinates
//    }
//
//    virtual void vertex(const int iface, const int nthvert, vec4& gl_Position) {
//        varying_uv.set_col(nthvert, model.uv(iface, nthvert));
//        varying_nrm.set_col(nthvert, proj<3>((ModelView).invert_transpose()*embed<4>(model.normal(iface, nthvert), 0.)));
//        gl_Position= ModelView*embed<4>(model.vert(iface, nthvert));
//        view_tri.set_col(nthvert, proj<3>(gl_Position));
//        gl_Position = Projection*gl_Position;
//    }
//
//    virtual bool fragment(const vec3 bar, TGAColor &gl_FragColor) {
//        vec3 bn = (varying_nrm*bar).normalized(); // per-vertex normal interpolation
//        vec2 uv = varying_uv*bar; // tex coord interpolation
//
//        // for the math refer to the tangent space normal mapping lecture
//        // https://github.com/ssloy/tinyrenderer/wiki/Lesson-6bis-tangent-space-normal-mapping
//        mat<3,3> AI = mat<3,3>{ {view_tri.col(1) - view_tri.col(0), view_tri.col(2) - view_tri.col(0), bn} }.invert();
//        vec3 i = AI * vec3{varying_uv[0][1] - varying_uv[0][0], varying_uv[0][2] - varying_uv[0][0], 0};
//        vec3 j = AI * vec3{varying_uv[1][1] - varying_uv[1][0], varying_uv[1][2] - varying_uv[1][0], 0};
//        mat<3,3> B = mat<3,3>{ {i.normalized(), j.normalized(), bn} }.transpose();
//
//        vec3 n = (B * model.normal(uv)).normalized(); // transform the normal from the texture to the tangent space
//        double diff = std::max(0., n*uniform_l); // diffuse light intensity
//        vec3 r = (n*(n*uniform_l)*2 - uniform_l).normalized(); // reflected light direction, specular mapping is described here: https://github.com/ssloy/tinyrenderer/wiki/Lesson-6-Shaders-for-the-software-renderer
//        double spec = std::pow(std::max(-r.z, 0.), 5+sample2D(model.specular(), uv)[0]); // specular intensity, note that the camera lies on the z-axis (in view), therefore simple -r.z
//
//        TGAColor c = sample2D(model.diffuse(), uv);
//        for (int i : {0,1,2})
//            gl_FragColor[i] = std::min<int>(10 + c[i]*(diff + spec), 255); // (a bit of ambient light, diff + spec), clamp the result
//
//        return false; // the pixel is not discarded
//    }
//};

class GouraudShader : public IShader {
    vec3 pointsIntensity;   // 每个顶点的 intensity
    mat4 Fin;               // P * V * M
    mat3 normalMatrix;      // 法线变换矩阵
    bool texture = false;
    Model* model = nullptr;
    std::vector<vec2> UV = std::vector<vec2>(3);
public:
    int getFaces() {
        if (model == nullptr)
            return -1;
        return model->nfaces();
    }

    void loadModel(const std::string& str) {
        model = new Model(str);
    }

    void loadTexture() {
        texture = true;
    }

    void unloadTexture() {
        texture = false;
    }

    void prepare() {
        mat4 Mid = V * M;
        Fin = P * Mid;
        mat3 temp;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j)
                temp[i][j] = Mid[i][j];
        }
        normalMatrix = adjointMatrix(temp);
    }

    virtual vec4 vertex(int nthface, int nthvert) {
        vec4 vertex = Fin * embed<4, 3>(model->vert(nthface, nthvert), 1);
        vec3 normal = normalMatrix * model->normal(nthface, nthvert);
        normal = normal.normalized();
        pointsIntensity[nthvert] = std::max(0.0, normal * lightDir);
        if (texture)
            UV[nthvert] = model->uv(nthface, nthvert);
        return vertex;
    }

    virtual bool fragment(vec3 bar, TGAColor& color) {
        float intensity = bar * pointsIntensity;
        if (!texture)
            color = TGAColor({ static_cast<uint8_t>(255 * intensity), static_cast<uint8_t>(255 * intensity), static_cast<uint8_t>(255 * intensity) });
        else {
            float u = 0, v = 0;
            for (int i = 0; i < 3; ++i) {
                u += bar[i] * UV[i][0];
                v += bar[i] * UV[i][1];
            }
            color = intensity * model->diffuse().get(u * model->diffuse().width(), v * model->diffuse().height());
        }
        return false;
    }

    virtual ~GouraudShader() {
        if (model != nullptr)
            delete model;
    }
};

class PhongShader : public IShader {
    std::vector<vec3> pointsNormal = std::vector<vec3>(3);   // 每个顶点的法向量
    std::vector<vec3> points = std::vector<vec3>(3);   // 每个顶点的世界空间坐标
    mat4 Fin;               // P * V * M
    mat4 Mid;               // V * M
    mat3 normalMatrix;      // 法线变换矩阵
    mat3 TBN;               // TBN 矩阵
    bool texture = false;
    bool specular = false;
    bool normalMap = false;
    Model* model = nullptr;
    std::vector<vec2> UV = std::vector<vec2>(3);
    vec3 Ambient = { 0.05, 0.05, 0.05 };       // 环境光
    vec3 Diffuse = { 1, 1, 1 };                           // 漫反射强度
    vec3 Specular = { 1, 1, 1 };                          // 镜面反射强度
public:
    int getFaces() {
        if (model == nullptr)
            return -1;
        return model->nfaces();
    }

    Model* getModel() {
        return model;
    }

    void loadModel(const std::string& str) {
        model = new Model(str);
    }

    void loadTexture() {
        texture = true;
    }

    void unloadTexture() {
        texture = false;
    }

    void loadSpecular() {
        specular = true;
    }

    void unloadSpecular() {
        specular = false;
    }

    void loadNormal() {
        normalMap = true;
    }

    void unloadNormal() {
        normalMap = false;
    }

    void prepare() {
        Mid = V * M;
        Fin = P * Mid;
        mat3 temp;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j)
                temp[i][j] = Mid[j][i];
        }
        normalMatrix = adjointMatrix(temp);
    }

    void setTBN(const mat3& matrix) {
        TBN = matrix;
    }

    virtual vec4 vertex(int nthface, int nthvert) {
        vec4 vertex = Fin * embed<4, 3>(model->vert(nthface, nthvert), 1);
        vec3 normal = normalMatrix * model->normal(nthface, nthvert);
        pointsNormal[nthvert] = normal.normalized();
        points[nthvert] = proj<3, 4>(vertex);
        UV[nthvert] = model->uv(nthface, nthvert);
        return vertex;
    }

    virtual bool fragment(vec3 bar, TGAColor& color) {
        vec3 normal;
        if (normalMap) {

        }
        else {
            for (int i = 0; i < 3; ++i)
                normal = normal + bar[i] * pointsNormal[i];
        }
        normal = normal.normalized();
        float u = 0, v = 0;
        vec3 point;
        for (int i = 0; i < 3; ++i) {
            u += bar[i] * UV[i][0];
            v += bar[i] * UV[i][1];
            point = point + bar[i] * points[i];
        }
        point = point.normalized();

        vec3 intensity = Ambient;

        if (texture) {
            TGAColor temp = model->diffuse().get(u * model->diffuse().width(), v * model->diffuse().height());
            for (int i = 0; i < 3; ++i)
                Diffuse[i] = static_cast<float>(temp.bgra[2 - i]) / 255;
        }
        else {
            Diffuse = { 1, 1, 1 };
        }
        intensity = intensity + Diffuse * std::max(0.0, normal * lightDir);

        if (specular) {
            float temp = static_cast<float>(model->specular().get(u * model->diffuse().width(), v * model->diffuse().height()).bgra[0]) / 255;
            for (int i = 0; i < 3; ++i)
                Specular[i] = temp;
        }
        else {
            Specular = { 0.5, 0.5, 0.5 };
        }
        if (normal * point < 0) {
            for (int i = 0; i < 3; ++i)
                point[i] = -point[i];
        }
        if (normal * lightDir > 0)
            intensity = intensity + Specular * std::pow(normal * (lightDir.normalized() + point).normalized(), 64);

        for (int i = 0; i < 3; ++i) {
            if (intensity[i] > 1)
                intensity[i] = 1;
        }

        color = TGAColor({ static_cast<uint8_t>(255 * intensity[2]), static_cast<uint8_t>(255 * intensity[1]), static_cast<uint8_t>(255 * intensity[0])});
        return false;
    }

    virtual ~PhongShader() {
        if (model != nullptr)
            delete model;
    }
};


float zbuffer[width * height];

int main(int argc, char** argv) {
    //if (2>argc) {
    //    std::cerr << "Usage: " << argv[0] << " obj/model.obj" << std::endl;
    //    return 1;
    //}
    //TGAImage framebuffer(width, height, TGAImage::RGB); // the output image
    //lookat(eye, center, up);                            // build the ModelView matrix
    //viewport(width/8, height/8, width*3/4, height*3/4); // build the Viewport matrix
    //projection((eye-center).norm());                    // build the Projection matrix
    //std::vector<double> zbuffer(width*height, std::numeric_limits<double>::max());

    //for (int m=1; m<argc; m++) { // iterate through all input objects
    //    Model model(argv[m]);
    //    Shader shader(model);
    //    for (int i=0; i<model.nfaces(); i++) { // for every triangle
    //        vec4 clip_vert[3]; // triangle coordinates (clip coordinates), written by VS, read by FS
    //        for (int j : {0,1,2})
    //            shader.vertex(i, j, clip_vert[j]); // call the vertex shader for each triangle vertex
    //        triangle(clip_vert, shader, framebuffer, zbuffer); // actual rasterization routine call
    //    }
    //}
    //framebuffer.write_tga_file("framebuffer.tga");
    
    TGAImage image(width, height, TGAImage::RGB);
    for (int i = 0; i < width * height; ++i)
        zbuffer[i] = -std::numeric_limits<float>::max();
    PhongShader shader;
    shader.setLight(light_dir.normalized());
    shader.setModel(zoom(1, 1, 1));
    shader.setProjection(perspective(angle2radian(45), static_cast<float>(width) / height, 1, 4));
    shader.setView(lookAt(eye, center, up));
    shader.loadTexture();
    shader.loadSpecular();
    shader.prepare();
    shader.loadModel("../obj/african_head/african_head.obj");
    Model* model = shader.getModel();
    int n = shader.getFaces();
    for (int i = 0; i < n; ++i) {
        std::cout << i << std::endl;
        std::vector<vec4> coordinates(3);
        for (int j = 0; j < 3; ++j) {
            coordinates[j] = shader.vertex(i, j);
        }
        triangle(coordinates, shader, image, zbuffer);
    }
    shader.loadModel("../obj/african_head/african_head_eye_inner.obj");
    n = shader.getFaces();
    for (int i = 0; i < n; ++i) {
        std::cout << i << std::endl;
        std::vector<vec4> coordinates(3);
        vec3 E1 = model->vert(i, 2) - model->vert(i, 0);
        vec3 E2 = model->vert(i, 1) - model->vert(i, 0);
        vec2 delt_UV1 = model->uv(i, 2) - model->uv(i, 0);
        vec2 delt_UV2 = model->uv(i, 1) - model->uv(i, 0);
        shader.setTBN(getTBN(E1, E2, delt_UV1, delt_UV2, model->));
        for (int j = 0; j < 3; ++j) {
            coordinates[j] = shader.vertex(i, j);
        }
        triangle(coordinates, shader, image, zbuffer);
    }

    image.write_tga_file("./pictures/phong_head_temp.tga");
    return 0;
}

