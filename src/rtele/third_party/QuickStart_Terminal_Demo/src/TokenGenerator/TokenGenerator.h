#ifndef TOKEN_GENERATOR_H
#define TOKEN_GENERATOR_H

#include <string>

class TokenGenerator {
public:
    static std::string generate(const std::string&, const std::string&, const std::string&, const std::string&);

private:
    static std::string generateServerToken(const std::string&, const std::string&, const std::string&, const std::string&);
};

#endif // TOKEN_GENERATOR_H