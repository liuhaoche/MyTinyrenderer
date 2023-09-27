#pragma once
#include <cmath>
#include "model.h"
#include "tgaimage.h"

class IShader {
public:
	virtual ~IShader() {}
	virtual vec4 vertex(int nthface, int nthvert) = 0;
	virtual bool fragment(vec3 bar, TGAColor& color) = 0;
	void setModel(const mat4& _model) { M = _model; }
	void setView(const mat4& _view) { V = _view; }
	void setProjection(const mat4& _projection) { P = _projection; }
	void setLight(const vec3& _lightDir) { lightDir = _lightDir; }
protected:
	vec3 lightDir;
	mat4 M;
	mat4 V;
	mat4 P;
};

void triangle(std::vector<vec4> points, IShader& shader, TGAImage& image, float zbuffer[]);

mat4 rotation(float radian, vec3 axis);
mat4 translation(float x, float y, float z);
mat4 zoom(float x_factor, float y_factor, float z_factor);
float angle2radian(float angle);
mat4 lookAt(vec3 eye, vec3 center, vec3 up);
mat4 perspective(float fovY, float aspect, float zNear, float zFar);
mat4 ortho(float left, float right, float bottom, float up, float front, float back);
mat4 viewport(float width, float height, float deepth);
mat3 adjointMatrix(const mat3& matrix);
mat3 getTBN(vec3 E1, vec3 E2, vec2 delt_UV1, vec2 delt_UV2, vec3 N);
