#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "lodepng.h"
#include "matrix.h"
#include "util.h"

int rand_int(int n) {                                   //随机int(小于n)
    int result;
    while (n <= (result = rand() / (RAND_MAX / n)));
    return result;
}

double rand_double() {                                  //随机double
    return (double)rand() / (double)RAND_MAX;
}

void update_fps(FPS *fps) {                             //更新fps
    fps->frames++;
    double now = glfwGetTime();
    double elapsed = now - fps->since;
    if (elapsed >= 1) {
        fps->fps = round(fps->frames / elapsed);
        fps->frames = 0;
        fps->since = now;
    }
}

char *load_file(const char *path) {                     //加载文件
    FILE *file = fopen(path, "rb");
    fseek(file, 0, SEEK_END);
    int length = ftell(file);
    rewind(file);
    char *data = calloc(length + 1, sizeof(char));
    fread(data, 1, length, file);
    fclose(file);
    return data;
}

GLuint gen_buffer(GLsizei size, GLfloat *data) {                //生成buffer
    GLuint buffer;
    glGenBuffers(1, &buffer);                                   //创建缓冲区对象，在buffers数组中返回当前1个未使用的名称，表示缓冲区对象  
    glBindBuffer(GL_ARRAY_BUFFER, buffer);                      //激活缓冲区对象，指定当前活动缓冲区的对象
    glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);  //用数据分配和初始化缓冲区对象，target:可以是GL_ARRAY_BUFFER()（顶点数据），size:存储相关数据所需的内存容量，data:用于初始化缓冲区对象，可以是一个指向客户区内存的指针，也可以是NULL，usage:数据只指定一次，但是可以多次作为绘图和图像指定函数的源数据
    glBindBuffer(GL_ARRAY_BUFFER, 0);                           //激活缓冲区对象，指定当前活动缓冲区的对象
    return buffer;
}

void del_buffer(GLuint buffer) {
    glDeleteBuffers(1, &buffer);                                //清除缓冲区对象
}

GLfloat *malloc_faces(int components, int faces) {              //为物件的面分配空间
    return malloc(sizeof(GLfloat) * 6 * components * faces);
}

GLuint gen_faces(int components, int faces, GLfloat *data) {    //生成物件的面
    GLuint buffer = gen_buffer(
        sizeof(GLfloat) * 6 * components * faces, data);
    free(data);
    return buffer;
}

GLuint make_shader(GLenum type, const char *source) {   //生成着色器
    GLuint shader = glCreateShader(type);       //分别创建一个顶点着色器对象或一个片段着色器对象
    glShaderSource(shader, 1, &source, NULL);   //分别将顶点着色程序的源代码字符数组绑定到顶点着色器对象或将片段着色程序的源代码字符数组绑定到片段着色器对象
    glCompileShader(shader);                    //分别编译顶点着色器对象或片段着色器对象
    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);  //glGetShaderiv返回编译着色器状态.
    if (status == GL_FALSE) {
        GLint length;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        GLchar *info = calloc(length, sizeof(GLchar));
        glGetShaderInfoLog(shader, length, NULL, info);
        fprintf(stderr, "glCompileShader failed:\n%s\n", info);
        free(info);
    }
    return shader;
}

GLuint load_shader(GLenum type, const char *path) { //加载着色器
    char *data = load_file(path);
    GLuint result = make_shader(type, data);        //得到结果着色器
    free(data);
    return result;
}

GLuint make_program(GLuint shader1, GLuint shader2) {
    GLuint program = glCreateProgram();         //使用glCreaterProgram()创建一个（着色）程序对象
    glAttachShader(program, shader1);           //使用glAttachShader()分别将顶点着色器对象和片段着色器对象附加到（着色）程序对象上
    glAttachShader(program, shader2);
    glLinkProgram(program);                     //使用glLinkProgram()对（着色）程序对象执行链接操作
    GLint status;
    glGetProgramiv(program, GL_LINK_STATUS, &status);   //得到染色器程序状态
    if (status == GL_FALSE) {
        GLint length;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
        GLchar *info = calloc(length, sizeof(GLchar));
        glGetProgramInfoLog(program, length, NULL, info);
        fprintf(stderr, "glLinkProgram failed: %s\n", info);
        free(info);
    }
    glDetachShader(program, shader1);           //解除着色器的附加
    glDetachShader(program, shader2);
    glDeleteShader(shader1);                    //删除着色器
    glDeleteShader(shader2);
    return program;                             //返回所得程序
}

GLuint load_program(const char *path1, const char *path2) {
    GLuint shader1 = load_shader(GL_VERTEX_SHADER, path1);      //得到顶点着色器
    GLuint shader2 = load_shader(GL_FRAGMENT_SHADER, path2);    //得到片段着色器
    GLuint program = make_program(shader1, shader2);            //用着色器链接生成程序
    return program;
}

void flip_image_vertical(                                       //图片上下翻转
    unsigned char *data, unsigned int width, unsigned int height)
{
    unsigned int size = width * height * 4;
    unsigned int stride = sizeof(char) * width * 4;
    unsigned char *new_data = malloc(sizeof(unsigned char) * size);
    for (unsigned int i = 0; i < height; i++) {
        unsigned int j = height - i - 1;
        memcpy(new_data + j * stride, data + i * stride, stride);
    }
    memcpy(data, new_data, size);
    free(new_data);
}

void load_png_texture(const char *file_name) {                                  //加载图片纹理
    unsigned int error;
    unsigned char *data;
    unsigned int width, height;
    error = lodepng_decode32_file(&data, &width, &height, file_name);           //加载数据，宽，高
    if (error) {                                                                //判断错误
        fprintf(stderr, "error %u: %s\n", error, lodepng_error_text(error));
    }
    flip_image_vertical(data, width, height);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,          //指定的参数，生成一个2D纹理（Texture）
        GL_UNSIGNED_BYTE, data);
//      GL_APICALL void GL_APIENTRY glTexImage2D(GLenum target, GLint level, GLenum internalformat,
//      GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const void* pixels);
//      参数说明：
//      target 指定目标纹理，这个值必须是GL_TEXTURE_2D。
//      level 执行细节级别。0是最基本的图像级别，n表示第N级贴图细化级别。
//      internalformat 指定纹理中的颜色组件。可选的值有GL_ALPHA,GL_RGB,GL_RGBA,GL_LUMINANCE, GL_LUMINANCE_ALPHA 等几种。
//      width 指定纹理图像的宽度，必须是2的n次方。纹理图片至少要支持64个材质元素的宽度
//      height 指定纹理图像的高度，必须是2的m次方。纹理图片至少要支持64个材质元素的高度
//      border 指定边框的宽度。必须为0。
//      format 像素数据的颜色格式, 不需要和internalformatt取值必须相同。可选的值参考internalformat。
//      type 指定像素数据的数据类型。可以使用的值有GL_UNSIGNED_BYTE,GL_UNSIGNED_SHORT_5_6_5,GL_UNSIGNED_SHORT_4_4_4_4,GL_UNSIGNED_SHORT_5_5_5_1。
//      pixels 指定内存中指向图像数据的指针

    free(data);
}

char *tokenize(char *str, const char *delim, char **key) {
    char *result;
    if (str == NULL) {
        str = *key;
    }
    str += strspn(str, delim);
    if (*str == '\0') {
        return NULL;
    }
    result = str;
    str += strcspn(str, delim);
    if (*str) {
        *str++ = '\0';
    }
    *key = str;
    return result;
}

int char_width(char input) {                                        //字符宽度
    static const int lookup[128] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        4, 2, 4, 7, 6, 9, 7, 2, 3, 3, 4, 6, 3, 5, 2, 7,
        6, 3, 6, 6, 6, 6, 6, 6, 6, 6, 2, 3, 5, 6, 5, 7,
        8, 6, 6, 6, 6, 6, 6, 6, 6, 4, 6, 6, 5, 8, 8, 6,
        6, 7, 6, 6, 6, 6, 8,10, 8, 6, 6, 3, 6, 3, 6, 6,
        4, 7, 6, 6, 6, 6, 5, 6, 6, 2, 5, 5, 2, 9, 6, 6,
        6, 6, 6, 6, 5, 6, 6, 6, 6, 6, 6, 4, 2, 5, 7, 0
    };
    return lookup[input];
}

int string_width(const char *input) {                               //字符串宽度
    int result = 0;
    int length = strlen(input);
    for (int i = 0; i < length; i++) {
        result += char_width(input[i]);
    }
    return result;
}

int wrap(const char *input, int max_width, char *output, int max_length) {
    *output = '\0';
    char *text = malloc(sizeof(char) * (strlen(input) + 1));
    strcpy(text, input);
    int space_width = char_width(' ');
    int line_number = 0;
    char *key1, *key2;
    char *line = tokenize(text, "\r\n", &key1);
    while (line) {
        int line_width = 0;
        char *token = tokenize(line, " ", &key2);
        while (token) {
            int token_width = string_width(token);
            if (line_width) {
                if (line_width + token_width > max_width) {
                    line_width = 0;
                    line_number++;
                    strncat(output, "\n", max_length - strlen(output) - 1);
                }
                else {
                    strncat(output, " ", max_length - strlen(output) - 1);
                }
            }
            strncat(output, token, max_length - strlen(output) - 1);
            line_width += token_width + space_width;
            token = tokenize(NULL, " ", &key2);
        }
        line_number++;
        strncat(output, "\n", max_length - strlen(output) - 1);
        line = tokenize(NULL, "\r\n", &key1);
    }
    free(text);
    return line_number;
}
