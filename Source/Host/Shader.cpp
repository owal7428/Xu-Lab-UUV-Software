#include "Shader.hpp"

#include <cstdio>

Shader::Shader(const char *VShaderString, const char *FShaderString)
{
    // Compile vertex and fragment shaders
    
    this->VertexShader = this->Compile(GL_VERTEX_SHADER, VShaderString);
    this->FragmentShader = this->Compile(GL_FRAGMENT_SHADER, FShaderString);
    
    // Create shader program from vertex and fragment shaders
    
    this->Program = glCreateProgram();
    glAttachShader(this->Program, this->VertexShader);
    glAttachShader(this->Program, this->FragmentShader);
    glLinkProgram(this->Program);
}

GLuint Shader::Compile(GLenum ShaderType, const char *ShaderSource)
{
    // Compile shader from source string

    GLuint ShaderID = glCreateShader(ShaderType);
    glShaderSource(ShaderID, 1, &ShaderSource, nullptr);
    glCompileShader(ShaderID);

    // Check compilation for errors and print if there are any

    GLint CompilationStatus;
    glGetShaderiv(ShaderID, GL_COMPILE_STATUS, &CompilationStatus);
    
    if (!CompilationStatus)
    {
        char TempBuf[512];
        glGetShaderInfoLog(ShaderID, 512, nullptr, TempBuf);
        
        fprintf(stderr, "Shader error: %s\n", TempBuf);
    }

    return ShaderID;
}

void Shader::Bind()
{
    glUseProgram(this->Program);
}

Shader::~Shader()
{
    glUseProgram(0); // Switch to default shader program

    glDeleteShader(this->VertexShader);
    glDeleteShader(this->FragmentShader);
    glDeleteProgram(this->Program);
}
