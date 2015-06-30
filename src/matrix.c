#include <math.h>
#include "config.h"
#include "matrix.h"
#include "util.h"

void normalize(float *x, float *y, float *z) {                  //得到单位向量
    float d = sqrtf((*x) * (*x) + (*y) * (*y) + (*z) * (*z));   //d = (x^2+y^2+z^2)^1/2
    *x /= d; *y /= d; *z /= d;                                  //(x,y,z)为单位向量
}

void mat_identity(float *matrix) {              //单位化
    matrix[0] = 1;                              //      1   0   0   0
    matrix[1] = 0;                              //      0   1   0   0
    matrix[2] = 0;                              //      0   0   1   0
    matrix[3] = 0;                              //      0   0   0   1
    matrix[4] = 0;
    matrix[5] = 1;
    matrix[6] = 0;
    matrix[7] = 0;
    matrix[8] = 0;
    matrix[9] = 0;
    matrix[10] = 1;
    matrix[11] = 0;
    matrix[12] = 0;
    matrix[13] = 0;
    matrix[14] = 0;
    matrix[15] = 1;
}

void mat_translate(float *matrix, float dx, float dy, float dz) {   //平移矩阵
    matrix[0] = 1;
    matrix[1] = 0;                              //      1   0   0   0
    matrix[2] = 0;                              //      0   1   0   0
    matrix[3] = 0;                              //      0   0   1   0
    matrix[4] = 0;                              //      dx  dy  dz  1
    matrix[5] = 1;
    matrix[6] = 0;
    matrix[7] = 0;
    matrix[8] = 0;
    matrix[9] = 0;
    matrix[10] = 1;
    matrix[11] = 0;
    matrix[12] = dx;
    matrix[13] = dy;
    matrix[14] = dz;
    matrix[15] = 1;
}

void mat_rotate(float *matrix, float x, float y, float z, float angle) {    //旋转矩阵
    normalize(&x, &y, &z);                               //化为单位向量
    float s = sinf(angle);                               //sinθ
    float c = cosf(angle);                               //cosθ
    float m = 1 - c;                                     //1-cosθ
    matrix[0] = m * x * x + c;                           //绕单位轴旋转θ角  
    matrix[1] = m * x * y - z * s;                          
    matrix[2] = m * z * x + y * s;                       
    matrix[3] = 0;                                       
    matrix[4] = m * x * y + z * s;                       //M = A + cosθ * ( I - A ) + sinθ * A*     
    matrix[5] = m * y * y + c;                           //P' = P * MT
    matrix[6] = m * y * z - x * s;                            
    matrix[7] = 0;                                       
    matrix[8] = m * z * x - y * s;                            
    matrix[9] = m * y * z + x * s;
    matrix[10] = m * z * z + c;
    matrix[11] = 0;
    matrix[12] = 0;
    matrix[13] = 0;
    matrix[14] = 0;
    matrix[15] = 1;
}

void mat_vec_multiply(float *vector, float *a, float *b) {  //矩阵乘以向量b*a
    float result[4];
    for (int i = 0; i < 4; i++) {       //a为矩阵，b为向量
        float total = 0;
        for (int j = 0; j < 4; j++) {   
            int p = j * 4 + i;
            int q = j;
            total += a[p] * b[q];
        }
        result[i] = total;
    }
    for (int i = 0; i < 4; i++) {
        vector[i] = result[i];      //得到向量结果
    }
}

void mat_multiply(float *matrix, float *a, float *b) {      //矩阵乘矩阵b*a
    float result[16];
    for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
            int index = c * 4 + r;
            float total = 0;
            for (int i = 0; i < 4; i++) {
                int p = i * 4 + r;
                int q = c * 4 + i;
                total += a[p] * b[q];
            }
            result[index] = total;
        }
    }
    for (int i = 0; i < 16; i++) {
        matrix[i] = result[i];      //得到结果矩阵
    }
}

void mat_apply(float *data, float *matrix, int count, int offset, int stride) { //向量变换
    float vec[4] = {0, 0, 0, 1};
    for (int i = 0; i < count; i++) {
        float *d = data + offset + stride * i;                  //定位数据
        vec[0] = *(d++); vec[1] = *(d++); vec[2] = *(d++);      //赋值给向量[x,y,z,1]
        mat_vec_multiply(vec, matrix, vec);                     //向量变换
        d = data + offset + stride * i;                         //重新定位
        *(d++) = vec[0]; *(d++) = vec[1]; *(d++) = vec[2];      //赋值给向量
    }
}

void frustum_planes(float planes[6][4], int radius, float *matrix) {    //构建视锥面
    float znear = 0.125;                    //近
    float zfar = radius * 32 + 64;          //远
    float *m = matrix;
    planes[0][0] = m[3] + m[0];
    planes[0][1] = m[7] + m[4];             //右    
    planes[0][2] = m[11] + m[8];
    planes[0][3] = m[15] + m[12];
    planes[1][0] = m[3] - m[0];
    planes[1][1] = m[7] - m[4];             //左
    planes[1][2] = m[11] - m[8];
    planes[1][3] = m[15] - m[12];
    planes[2][0] = m[3] + m[1];
    planes[2][1] = m[7] + m[5];             //上
    planes[2][2] = m[11] + m[9];
    planes[2][3] = m[15] + m[13];
    planes[3][0] = m[3] - m[1];
    planes[3][1] = m[7] - m[5];             //下
    planes[3][2] = m[11] - m[9];
    planes[3][3] = m[15] - m[13];
    planes[4][0] = znear * m[3] + m[2];
    planes[4][1] = znear * m[7] + m[6];     //前
    planes[4][2] = znear * m[11] + m[10];
    planes[4][3] = znear * m[15] + m[14];
    planes[5][0] = zfar * m[3] - m[2];
    planes[5][1] = zfar * m[7] - m[6];      //后
    planes[5][2] = zfar * m[11] - m[10];
    planes[5][3] = zfar * m[15] - m[14];
}

void mat_frustum(
    float *matrix, float left, float right, float bottom,
    float top, float znear, float zfar)
{                                           //构建视锥矩阵
    float temp, temp2, temp3, temp4;
    temp = 2.0 * znear;
    temp2 = right - left;                   //宽
    temp3 = top - bottom;                   //高
    temp4 = zfar - znear;                   //长
    matrix[0] = temp / temp2;
    matrix[1] = 0.0;
    matrix[2] = 0.0;
    matrix[3] = 0.0;
    matrix[4] = 0.0;
    matrix[5] = temp / temp3;
    matrix[6] = 0.0;
    matrix[7] = 0.0;
    matrix[8] = (right + left) / temp2;
    matrix[9] = (top + bottom) / temp3;
    matrix[10] = (-zfar - znear) / temp4;
    matrix[11] = -1.0;
    matrix[12] = 0.0;
    matrix[13] = 0.0;
    matrix[14] = (-temp * zfar) / temp4;
    matrix[15] = 0.0;
}

void mat_perspective(               //透视矩阵
    float *matrix, float fov, float aspect,
    float znear, float zfar)
{
    float ymax, xmax;
    ymax = znear * tanf(fov * PI / 360.0);          //ymax = znear * tanθ
    xmax = ymax * aspect;                           //xmax = ymax * 高宽比
    mat_frustum(matrix, -xmax, xmax, -ymax, ymax, znear, zfar); //将数据生成视锥矩阵
}

void mat_ortho(         //正投影矩阵
    float *matrix,
    float left, float right, float bottom, float top, float near, float far)
{
    matrix[0] = 2 / (right - left);
    matrix[1] = 0;
    matrix[2] = 0;
    matrix[3] = 0;
    matrix[4] = 0;
    matrix[5] = 2 / (top - bottom);
    matrix[6] = 0;
    matrix[7] = 0;
    matrix[8] = 0;
    matrix[9] = 0;
    matrix[10] = -2 / (far - near);
    matrix[11] = 0;
    matrix[12] = -(right + left) / (right - left);
    matrix[13] = -(top + bottom) / (top - bottom);
    matrix[14] = -(far + near) / (far - near);
    matrix[15] = 1;
}

void set_matrix_2d(float *matrix, int width, int height) {      //设置二维矩阵
    mat_ortho(matrix, 0, width, 0, height, -1, 1);              //采用正投影
}

void set_matrix_3d(                                             //设置三维矩阵
    float *matrix, int width, int height,                       //屏幕宽高
    float x, float y, float z, float rx, float ry,              //坐标x,y,z   旋转rx，ry
    float fov, int ortho, int radius)                           //视角，正投影尺寸，半径
{
    float a[16];                                                //矩阵a
    float b[16];                                                //矩阵b
    float aspect = (float)width / height;                       //屏幕宽高比
    float znear = 0.125;                                        //近处
    float zfar = radius * 32 + 64;                              //远处
    mat_identity(a);                                            //a单位化
    mat_translate(b, -x, -y, -z);                               //b沿向量(x,y,z)平移
    mat_multiply(a, b, a);                                      //a = a * b 保留对角线元素
    mat_rotate(b, cosf(rx), 0, sinf(rx), ry);                   //将b绕(cosf(rx), 0, sinf(rx))轴旋转ry角度
    mat_multiply(a, b, a);                                      //a = a * b
    mat_rotate(b, 0, 1, 0, -rx);                                //将b绕(0, 1, 0)轴旋转-rx角度
    mat_multiply(a, b, a);                                      //a = a * b
    if (ortho) {                                                //如果正投影
        int size = ortho;
        mat_ortho(b, -size * aspect, size * aspect, -size, size, -zfar, zfar);  //将b变为正投影矩阵
    }
    else {
        mat_perspective(b, fov, aspect, znear, zfar);           //将b变为透视矩阵
    }
    mat_multiply(a, b, a);                                      //a = a * b
    mat_identity(matrix);                                       //将matrix单位化
    mat_multiply(matrix, a, matrix);                            //matrix = matrix * a
}

void set_matrix_item(float *matrix, int width, int height, int scale) { //设置项目矩阵
    float a[16];                                                        //矩阵a
    float b[16];                                                        //矩阵b
    float aspect = (float)width / height;                               //屏幕宽高比
    float size = 64 * scale;                                            //大小size
    float box = height / size / 2;                                      //box = height/(2 * size)
    float xoffset = 1 - size / width * 2;                               //x偏移
    float yoffset = 1 - size / height * 2;                              //y偏移
    mat_identity(a);                                                    //a单位化
    mat_rotate(b, 0, 1, 0, -PI / 4);                                    //b绕y转-45度
    mat_multiply(a, b, a);                                              //a = a * b
    mat_rotate(b, 1, 0, 0, -PI / 10);                                   //b绕x转-18度
    mat_multiply(a, b, a);                                              //a = a * b
    mat_ortho(b, -box * aspect, box * aspect, -box, box, -1, 1);        //b按box的大小正投影
    mat_multiply(a, b, a);                                              //a = a * b
    mat_translate(b, -xoffset, -yoffset, 0);                            //b沿(-xoffset, -yoffset, 0)方向平移
    mat_multiply(a, b, a);                                              //a = a * b
    mat_identity(matrix);                                               //将matrix单位化
    mat_multiply(matrix, a, matrix);                                    //matrix = matrix * a
}
