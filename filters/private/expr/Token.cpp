
#include "Token.hpp"

namespace pdal
{
namespace expr
{

TokenClass Token::cls() const
{
    switch (m_type)
    {
    case TokenType::Or:
        return TokenClass::Or;
    case TokenType::And:
        return TokenClass::And;
    case TokenType::Equal:
    case TokenType::NotEqual:
    case TokenType::Greater:
    case TokenType::Less:
    case TokenType::GreaterEqual:
    case TokenType::LessEqual:
        return TokenClass::Compare;
    case TokenType::Add:
    case TokenType::Subtract:
        return TokenClass::Add;
    case TokenType::Multiply:
    case TokenType::Divide:
        return TokenClass::Multiply;
    default:
        return TokenClass::None;
    }
}

} // namespace expr
} // namespace pdal

