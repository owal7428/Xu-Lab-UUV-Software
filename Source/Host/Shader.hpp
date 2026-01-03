#ifndef HOST_SHADER_HPP_
#define HOST_SHADER_HPP_

#include <glad/gl.h>

class Shader
{
private:
    GLuint VertexShader;
    GLuint FragmentShader;
    GLuint Program;

    GLuint Compile(GLenum ShaderType, const char* ShaderSource);

public:
    /**
	 * @brief Constructs and compiles the shader.
	 * @param VShaderString String containing vertex shader code.
     * @param FShaderString String containing fragment shader code.
	 */
    Shader(const char* VShaderString, const char* FShaderString);
    
    /**
     * @brief Activates the shader.
	 */
    void Bind();
    
    /**
     * @brief Gets program shader ID.
     * @returns Shader program ID as GLuint.
	 */
    GLuint GetProgram() {return this->Program;}
    
    ~Shader();
};

#endif // HOST_SHADER_HPP_
