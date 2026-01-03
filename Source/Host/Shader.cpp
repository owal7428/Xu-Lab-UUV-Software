#include "Shader.hpp"

#include <cstdio>
#include <string>
#include <fstream>
#include <sstream>

Shader::Shader(const char* ShaderString)
{
    // Get shader code from file

    std::string VShaderFile(ShaderString);
    VShaderFile += ".vert";

    std::string FShaderFile(ShaderString);
    FShaderFile += ".frag";

    const char* VShaderString = this->ReadFile(VShaderFile.c_str());
    const char* FShaderString = this->ReadFile(FShaderFile.c_str());

    // Compile vertex and fragment shaders
    
    this->VertexShader = this->Compile(GL_VERTEX_SHADER, VShaderString);
    this->FragmentShader = this->Compile(GL_FRAGMENT_SHADER, FShaderString);
    
    // Create shader program from vertex and fragment shaders
    
    this->Program = glCreateProgram();
    glAttachShader(this->Program, this->VertexShader);
    glAttachShader(this->Program, this->FragmentShader);
    glLinkProgram(this->Program);
}

const char *Shader::ReadFile(const char* FilePath)
{
    std::ifstream FileStream(FilePath);

    if (!FileStream)
        fprintf(stderr, "Could not find file: %s\n", FilePath);

    std::string StringLine;
    std::stringstream StringStream;

    while (std::getline(FileStream, StringLine))
    {
        StringStream << StringLine << '\n';
    }

    FileStream.close();

    return StringStream.str().c_str();
}

GLuint Shader::Compile(GLenum ShaderType, const char* ShaderSource)
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
