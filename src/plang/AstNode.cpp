/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <pdal/plang/AstNode.hpp>

#include <string>
#include <iostream>

#include <pdal/plang/AstUtils.hpp>


namespace pdal { namespace plang {


static unsigned int s_id = 0;


AstNode::AstNode(DataType datatype, NodeType nodetype)
    : m_id(s_id)
    , m_datatype(datatype)
    , m_nodetype(nodetype)
{
    ++s_id;
    return;
}


AstNode::~AstNode()
{
    return;
}


bool AstNode::equal_to(const AstNode& other) const
{
    return ((getDataType() == other.getDataType()) &&
            (getNodeType() == other.getNodeType()));
}


void AstNode::setDataType(DataType datatype)
{
    m_datatype = datatype;
}


DataType AstNode::getDataType() const
{
    return m_datatype;
}


std::string AstNode::getDataTypeName() const
{
    return AstUtils::getName(getDataType());
}


NodeType AstNode::getNodeType() const
{
    return m_nodetype;
}


std::string AstNode::getNodeTypeName() const
{
    return AstUtils::getName(getNodeType());
}


void AstNode::spaces(int indent)
{
    for (int i=0; i<indent; i++)
        std::cout << " ";
}


// -------------------------------------------------------------------------


AstTempVector::AstTempVector(std::vector<AstNode*> v)
    : AstNode(DataType_Unknown, NodeType_NopV)
    , m_v(v)
{
    return;
}


AstTempVector::AstTempVector(AstNode* v1, AstNode* v2)
    : AstNode(DataType_Unknown, NodeType_NopV)
    , m_v()
{
    m_v.push_back(v1);
    m_v.push_back(v2);
    return;
}


AstTempVector::~AstTempVector()
{
    //delete m_v;
}


bool AstTempVector::equal_to(const AstNode& other) const
{
    const AstTempVector* p = dynamic_cast<const AstTempVector*>(&other);
    if (!p) return false;
    
    if (this->getChildren() != p->getChildren()) return false;

    return true;
}


void AstTempVector::print(int indent) const
{
    spaces(indent);
    std::cout << "NopV\n";
    for (unsigned int i=0; i<m_v.size(); i++)
    {
        spaces(indent+1);
        std::cout << "(" << i << ")\n";
        m_v[i]->print(indent+1);
    }

    return;
}


bool AstTempVector::evaluate(SymbolTable&, variant_t&)
{
    assert(0);
    return false;
}


AstNode* AstTempVector::simplify(SymbolTable& symbolTable)
{
    std::vector<AstNode*> new_children;

    // simplify our children
    for (unsigned int i=0; i<m_v.size(); i++)
    {
        AstNode* child = m_v[i]->simplify(symbolTable);
        if (child)
        {
            new_children.push_back(child);
        }
        //if (child != m_v[i])
        //{
        //    delete m_v[i];
        //}
        m_v[i] = NULL;
    }
    m_v.clear();

    if (new_children.size() == 0)
    {
        delete this;
        return NULL;
    } 

    m_v = new_children;

    new_children.clear();

    // now inline any chilren that are NopVs
    for (unsigned int i=0; i<m_v.size(); i++)
    {
        AstNode* child = m_v[i];
        assert(child);

        // if our child is itself a NopV, "inline" the child's children
        if (dynamic_cast<AstTempVector*>(child))
        {
            AstTempVector* vec = dynamic_cast<AstTempVector*>(child);
            const std::vector<AstNode*>& grandchildren = vec->getChildren();
            for (unsigned int j=0; j<grandchildren.size(); j++)
            {
                new_children.push_back(grandchildren[j]);
            }
            delete child;
        }
        else
        {
            new_children.push_back(child);
        }
    }

    // now simplify ourself
    assert(new_children.size() > 0);

    if (new_children.size() == 1)
    {
        delete this;
        return new_children[0];
    }

    assert(new_children.size() >= 2);
    this->m_v = new_children;

    for (unsigned int i=0; i<m_v.size(); i++)
    {
        AstNode* child = m_v[i];
        assert(child);
        assert(dynamic_cast<AstTempVector*>(child) == NULL);
    }
    
    AstNode* result = collapseChain(symbolTable);
    return result;
}


AstNode* AstTempVector::collapseChain(SymbolTable& symbolTable)
{
    // our children vector is something like [1, Mul1(2), Div1(3)]

    if (!isChain())
    {
        return this;
    }

    AstNode* expr = m_v[0];

    for (unsigned int i=1; i<m_v.size(); i++)
    {
        expr = fixOp1(expr, m_v[i]);
        expr = expr->simplify(symbolTable);
    }

    delete this;
    return expr;
}


static bool isOp1(const AstNode& n)
{
    const NodeType t = n.getNodeType();
    return (t == NodeType_Add1 || 
            t == NodeType_Subtract1 ||
            t == NodeType_Multiply1 ||
            t == NodeType_Divide1 ||
            t == NodeType_ArithXor1 ||
            t == NodeType_ArithOr1 ||
            t == NodeType_ArithAnd1 ||
            t == NodeType_LogicalOr1 ||
            t == NodeType_LogicalAnd1 ||
            t == NodeType_Greater1 ||
            t == NodeType_GreaterEq1 ||
            t == NodeType_Less1 ||
            t == NodeType_LessEq1 ||
            t == NodeType_Equal1 ||
            t == NodeType_NotEqual1);
}


bool AstTempVector::isChain() const
{
    if (isOp1(*m_v[0]))
    {
        return false;
    }

    for (unsigned int i=1; i<m_v.size(); i++)
    {
        const AstNode& n = *m_v[i];
        if (!isOp1(n))
        {
            return false;
        }
    }

    return true;
}


AstNode* AstTempVector::fixOp1(AstNode* expr, AstNode* op1)
{
    // convert [expr, Op1(a)] to [Op(expr, a)]

    AstTempOp1* op = dynamic_cast<AstTempOp1*>(op1);
    assert(op);
    AstNode* child = op->removeChild();
    NodeType nodetype = op->getNodeType();
    delete op;

    switch (nodetype)
    {
    case NodeType_Add1:
        return new AstBinaryOp(expr, child, NodeType_Add);
        break;
    case NodeType_Subtract1:
        return new AstBinaryOp(expr, child, NodeType_Subtract);
        break;
    case NodeType_Multiply1:
        return new AstBinaryOp(expr, child, NodeType_Multiply);
        break;
    case NodeType_Divide1:
        return new AstBinaryOp(expr, child, NodeType_Divide);
        break;
    case NodeType_Greater1:
        return new AstBinaryOp(expr, child, NodeType_Greater);
        break;
    case NodeType_GreaterEq1:
        return new AstBinaryOp(expr, child, NodeType_GreaterEq);
        break;
    case NodeType_Less1:
        return new AstBinaryOp(expr, child, NodeType_Less);
        break;
    case NodeType_LessEq1:
        return new AstBinaryOp(expr, child, NodeType_LessEq);
        break;
    case NodeType_Equal1:
        return new AstBinaryOp(expr, child, NodeType_Equal);
        break;
    case NodeType_NotEqual1:
        return new AstBinaryOp(expr, child, NodeType_NotEqual);
        break;
    case NodeType_ArithXor1:
        return new AstBinaryOp(expr, child, NodeType_ArithXor);
        break;
    case NodeType_ArithOr1:
        return new AstBinaryOp(expr, child, NodeType_ArithOr);
        break;
    case NodeType_ArithAnd1:
        return new AstBinaryOp(expr, child, NodeType_ArithAnd);
        break;
    case NodeType_LogicalOr1:
        return new AstBinaryOp(expr, child, NodeType_LogicalOr);
        break;
    case NodeType_LogicalAnd1:
        return new AstBinaryOp(expr, child, NodeType_LogicalAnd);
        break;
    }

    assert(0);
    return NULL;
}


// -------------------------------------------------------------------------


AstConvert::AstConvert(AstNode* node, DataType datatype)
    : AstNode(datatype, NodeType_Convert)
    , m_node(node)
{
    return;
}


AstConvert::~AstConvert()
{
    delete m_node;
}


bool AstConvert::equal_to(const AstNode& other) const
{
    const AstConvert* p = dynamic_cast<const AstConvert*>(&other);
    if (!p) return false;
    
    if (*(this->getChild()) != *(p->getChild())) return false;

    return true;
}


void AstConvert::print(int indent) const
{
    spaces(indent);
    std::cout << "ConvertTo" << getDataTypeName() << "\n";
    m_node->print(indent+1);
}


AstNode* AstConvert::simplify(SymbolTable& symbolTable)
{
    AstNode* rawnode = m_node->simplify(symbolTable);
    m_node = NULL;

    const DataType dstEnum = this->getDataType();
    const DataType srcEnum = rawnode->getDataType();

    // if the conversion is a no-op, just return the child
    if (srcEnum == dstEnum)
    {
        delete this;
        return rawnode;
    }

    // if child not a constant, we can do nothing
    if (rawnode->getNodeType() != NodeType_Constant)
    {
        m_node = rawnode;
        return this;
    }

    AstConstant* node = dynamic_cast<AstConstant*>(rawnode);
    assert(node);
    variant_t srcValue = node->getVariantValue();
    variant_t dstValue;

    bool ok = AstUtils::convert(srcEnum, srcValue, dstEnum, dstValue);
    if (!ok)
    {
        m_node = rawnode;
        return this;
    }

    node->setValue(dstValue);
    node->setDataType(dstEnum);

    delete this;

    return node;
}


bool AstConvert::evaluate(SymbolTable& symbolTable, variant_t& value)
{
    if (!m_node->evaluate(symbolTable, value))
    {
        return false;
    }

    const DataType dstEnum = this->getDataType();
    const DataType srcEnum = m_node->getDataType();

    // if the conversion is a no-op, just return the child
    if (srcEnum == dstEnum)
    {
        return true;
    }

    variant_t srcValue = value;
    variant_t dstValue;

    bool ok = AstUtils::convert(srcEnum, srcValue, dstEnum, dstValue);
    if (!ok)
    {
        return false;
    }

    value = dstValue;
    return true;
}


// -------------------------------------------------------------------------


AstNegate::AstNegate(AstNode* node)
    : AstNode(DataType_Unknown, NodeType_Negate)
    , m_node(node)
{
    return;
}


AstNegate::~AstNegate()
{
    delete m_node;
}


bool AstNegate::equal_to(const AstNode& other) const
{
    const AstNegate* p = dynamic_cast<const AstNegate*>(&other);
    if (!p) return false;
    
    if (*(this->getChild()) != *(p->getChild())) return false;

    return true;
}


void AstNegate::print(int indent) const
{
    spaces(indent);
    std::cout << "Negate\n";
    m_node->print(indent+1);
}


AstNode* AstNegate::simplify(SymbolTable& symbolTable)
{
    AstNode* rawnode = m_node->simplify(symbolTable);
    m_node = NULL;

    // if child not a constant, we can do nothing
    if (rawnode->getNodeType() != NodeType_Constant)
    {
        m_node = rawnode;
        return this;
    }

    AstConstant* node = dynamic_cast<AstConstant*>(rawnode);
    assert(node);
    const variant_t srcValue = node->getVariantValue();
    variant_t dstValue;

    bool ok = AstUtils::apply_negate(node->getDataType(), srcValue, dstValue);
    if (!ok)
    {
        m_node = rawnode;
        return this;
    }

    AstConstant* c = new AstConstant(dstValue);

    delete rawnode;
    delete this;

    return c;
}


bool AstNegate::evaluate(SymbolTable& symbolTable, variant_t& value)
{
    if (!m_node->evaluate(symbolTable, value))
    {
        return false;
    }

    // if child not a constant, we can do nothing
    if (m_node->getNodeType() != NodeType_Constant)
    {
        return false;
    }

    const variant_t srcValue = value;
    variant_t dstValue;

    bool ok = AstUtils::apply_negate(m_node->getDataType(), srcValue, dstValue);
    if (!ok)
    {
        return false;
    }

    value = dstValue;
    return true;
}


// -------------------------------------------------------------------------


AstTempOp1::AstTempOp1(AstNode* node, NodeType nodetype)
    : AstNode(DataType_Unknown, nodetype)
    , m_node(node)
{
    return;
}


AstTempOp1::~AstTempOp1()
{
    delete m_node;
}


bool AstTempOp1::equal_to(const AstNode& other) const
{
    const AstTempOp1* p = dynamic_cast<const AstTempOp1*>(&other);
    if (!p) return false;
    
    if (*(this->getChild()) != *(p->getChild())) return false;

    return true;
}


void AstTempOp1::print(int indent) const
{
    spaces(indent);
    std::cout << getNodeTypeName() << "\n";
    m_node->print(indent+1);
}


AstNode* AstTempOp1::simplify(SymbolTable& symbolTable)
{
    m_node = m_node->simplify(symbolTable);

    setDataType(m_node->getDataType());

    return this; 
}


bool AstTempOp1::evaluate(SymbolTable&, variant_t&)
{
    assert(0);
    return false;
}


// -------------------------------------------------------------------------


AstBinaryOp::AstBinaryOp(AstNode* left, AstNode* right, NodeType nodetype)
    : AstNode(DataType_Unknown, nodetype)
    ,  m_left(left)
    ,  m_right(right)
{
    return;
}


AstBinaryOp::~AstBinaryOp()
{
    delete m_left;
    delete m_right;
}


bool AstBinaryOp::equal_to(const AstNode& other) const
{
    const AstBinaryOp* p = dynamic_cast<const AstBinaryOp*>(&other);
    if (!p) return false;
    
    if (*(this->getLeft()) != *(p->getLeft())) return false;
    if (*(this->getRight()) != *(p->getRight())) return false;

    return true;
}


void AstBinaryOp::print(int indent) const
{
    spaces(indent);
    std::cout << getNodeTypeName() << "\n";
    m_left->print(indent+1);
    m_right->print(indent+1);
}


AstNode* AstBinaryOp::simplify(SymbolTable& symbolTable)
{
    AstNode* left = m_left->simplify(symbolTable);
    AstNode* right = m_right->simplify(symbolTable);
    m_left = m_right = NULL;

    assert(left->getDataType() == right->getDataType());

    switch (getNodeType())
    {
    case NodeType_Greater:
    case NodeType_GreaterEq:
    case NodeType_Less:
    case NodeType_LessEq:
    case NodeType_Equal:
    case NodeType_NotEqual:
        setDataType(DataType_Bool);
        break;
    default:
        setDataType(left->getDataType());
        break;
    }
    
    if (left->getNodeType() != NodeType_Constant || right->getNodeType() != NodeType_Constant)
    {
        // we can only simplify constants
        m_left = left;
        m_right = right;
        return this;
    }

    // we have two two constants of the same type
    AstConstant* leftc = dynamic_cast<AstConstant*>(left);
    AstConstant* rightc = dynamic_cast<AstConstant*>(right);
    const variant_t leftValue = leftc->getVariantValue();
    const variant_t rightValue = rightc->getVariantValue();
    variant_t dstValue;

    bool ok = AstUtils::apply_binop(getNodeType(), left->getDataType(), leftValue, rightValue, dstValue);
    if (!ok)
    {
        m_left = left;
        m_right = right;
        return this;
    }

    delete left; 
    delete right;
    delete this; 

    AstConstant* node = new AstConstant(dstValue);
    return node;
}


bool AstBinaryOp::evaluate(SymbolTable& symbolTable, variant_t& value)
{
    variant_t leftValue, rightValue;
    if (!m_left->evaluate(symbolTable, leftValue) || !m_right->evaluate(symbolTable, rightValue))
    {
        return false;
    }
    
    assert(m_left->getDataType() == m_right->getDataType());

    // we have two two constants of the same type
    variant_t dstValue;

    bool ok = AstUtils::apply_binop(getNodeType(), m_left->getDataType(), leftValue, rightValue, dstValue);
    if (!ok)
    {
        return false;
    }

    value = dstValue;
    return true;
}


// -------------------------------------------------------------------------


AstConstant::AstConstant(variant_t value)
    : AstNode(AstUtils::inferType(value), NodeType_Constant)
    , m_value(value)
{
    return;
}


AstConstant::~AstConstant()
{
    return;
}


bool AstConstant::equal_to(const AstNode& other) const
{
    const AstConstant* p = dynamic_cast<const AstConstant*>(&other);
    if (!p) return false;
    
    if (!(this->getVariantValue() == p->getVariantValue())) return false;

    return true;
}


AstNode* AstConstant::simplify(SymbolTable&)
{
    return this; 
}


bool AstConstant::evaluate(SymbolTable&, variant_t& value)
{
    value = m_value;
    return true;
}


void AstConstant::print(int indent) const
{
    spaces(indent);
    std::cout << "Constant " << getDataTypeName() << " " << m_value << "\n";
}


// -------------------------------------------------------------------------


AstVariableUse::AstVariableUse(SymbolName* symbolName)
    : AstNode(DataType_Unknown, NodeType_VariableUse)
    , m_symbolName(symbolName)
    , m_symbol(0)
{
    return;
}


AstVariableUse::~AstVariableUse()
{
    delete m_symbolName;
}


bool AstVariableUse::equal_to(const AstNode& other) const
{
    const AstVariableUse* p = dynamic_cast<const AstVariableUse*>(&other);
    if (!p) return false;
    
    if (m_symbolName != p->m_symbolName) return false;
    if (this->getSymbol() != p->getSymbol()) return false;

    return true;
}


void AstVariableUse::print(int indent) const
{
    spaces(indent);
    std::cout << "VariableUse" << getDataTypeName() << " " << getSymbol()->getName() << "\n";

    return;
}


Symbol* AstVariableUse::getSymbol() const 
{ 
    return m_symbol;
}


AstNode* AstVariableUse::simplify(SymbolTable& symbolTable)
{
    if (!m_symbol)
    {
        m_symbol = symbolTable.get(m_symbolName->getName());
        delete m_symbolName;
        m_symbolName = NULL;

        m_symbol->isUsed(true);
        if (m_symbol->isDefined() == false)
        {
            m_symbol->isFree(true);
        }

        setDataType(m_symbol->getDataType());
    }

    // never replace a variable use with a constant, because the
    // variable might be dependent on a free variable (could refine
    // this later)

    return this;
}


bool AstVariableUse::evaluate(SymbolTable&, variant_t& value)
{
    if (!m_symbol->hasValue())
    {
        return false;
    }

    value = m_symbol->getVariantValue();

    return true;
}


// -------------------------------------------------------------------------


AstVariableDef::AstVariableDef(SymbolName* symbolName, AstNode* rhs)
    : AstNode(DataType_Unknown, NodeType_VariableDef)
    , m_symbolName(symbolName)
    , m_symbol(0)
    , m_rhs(rhs)
{
    return;
}


AstVariableDef::~AstVariableDef()
{
    delete m_rhs;
    delete m_symbolName;
}


bool AstVariableDef::equal_to(const AstNode& other) const
{
    const AstVariableDef* p = dynamic_cast<const AstVariableDef*>(&other);
    if (!p) return false;
    
    if (this->m_symbolName->getName() != p->m_symbolName->getName()) return false;
    if (this->getRight() != p->getRight()) return false;

    return true;
}


void AstVariableDef::print(int indent) const
{
    spaces(indent);
    std::cout << "VariableDef" << getDataTypeName() << " " << getSymbol()->getName() << "\n";
    m_rhs->print(indent+1);

    return;
}


Symbol* AstVariableDef::getSymbol() const 
{ 
    return m_symbol;
}


AstNode* AstVariableDef::getRight() const 
{ 
    return m_rhs;
}


AstNode* AstVariableDef::simplify(SymbolTable& symbolTable)
{
    if (!m_symbol)
    {
        m_symbol = symbolTable.get(m_symbolName->getName());
        delete m_symbolName;
        m_symbolName = NULL;

        m_symbol->isDefined(true);

        setDataType(m_symbol->getDataType());
    }

    m_rhs = m_rhs->simplify(symbolTable);

    assert(m_symbol->getDataType() == m_rhs->getDataType());

    if (m_rhs->getNodeType() == NodeType_Constant)
    {
        AstConstant* c = dynamic_cast<AstConstant*>(m_rhs);
        assert(c);
        m_symbol->setValue(c->getVariantValue());
    }

    return this;
}


bool AstVariableDef::evaluate(SymbolTable& symbolTable, variant_t& value)
{
    if (!m_rhs->evaluate(symbolTable, value))
    {
        return false;
    }

    m_symbol->setValue(value);

    return true;
}


} } //namespaces
