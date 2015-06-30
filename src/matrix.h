#ifndef _matrix_h_
#define _matrix_h_

void normalize(float *x, float *y, float *z);										//得到单位向量
void mat_identity(float *matrix);													//单位化矩阵
void mat_translate(float *matrix, float dx, float dy, float dz);					//矩阵平移(dx,dy,dz)
void mat_rotate(float *matrix, float x, float y, float z, float angle);				//矩阵旋转(x,y,z单位轴，angle度)
void mat_vec_multiply(float *vector, float *a, float *b);							//向量乘矩阵
void mat_multiply(float *matrix, float *a, float *b);								//矩阵乘矩阵
void mat_apply(float *data, float *matrix, int count, int offset, int stride);		//向量变换
void frustum_planes(float planes[6][4], int radius, float *matrix);					//构建视锥面(6个面)
void mat_frustum(																	//构建视锥矩阵(上下左右前后)
    float *matrix, float left, float right, float bottom,
    float top, float znear, float zfar);
void mat_perspective(																//构建透视矩阵(视角，宽高比，前，后)
    float *matrix, float fov, float aspect,
    float near, float far);
void mat_ortho(																		//正投影矩阵(上下左右前后)
    float *matrix,
    float left, float right, float bottom, float top, float near, float far);
void set_matrix_2d(float *matrix, int width, int height);							//用正投影矩阵构建二维矩阵
void set_matrix_3d(																	//构建三维矩阵
    float *matrix, int width, int height,
    float x, float y, float z, float rx, float ry,
    float fov, int ortho, int radius);
void set_matrix_item(float *matrix, int width, int height, int scale);				//构建项目矩阵

#endif
