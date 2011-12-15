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

#ifndef ASTNODE_H
#define ASTNODE_H

#include <pdal/pdal_internal.hpp>

#include <pdal/plang/SymbolTable.hpp>

#include <boost/cstdint.hpp>
#include <boost/variant.hpp>

#include <vector>
#include <iostream>


namespace pdal { namespace plang {


enum NodeType
{
    // variables
    NodeType_VariableUse, NodeType_VariableDef,
    NodeType_Program,

    // binary ops
    NodeType_Add, NodeType_Subtract, NodeType_Multiply, NodeType_Divide,
    NodeType_Greater, NodeType_GreaterEq, NodeType_Less, NodeType_LessEq, NodeType_Equal, NodeType_NotEqual,
    NodeType_ArithXor, NodeType_ArithAnd, NodeType_ArithOr,
    NodeType_LogicalAnd, NodeType_LogicalOr,


    // funky unary ops
    NodeType_Add1, NodeType_Subtract1, NodeType_Multiply1, NodeType_Divide1,
    NodeType_Greater1, NodeType_GreaterEq1, NodeType_Less1, NodeType_LessEq1, NodeType_Equal1, NodeType_NotEqual1,
    NodeType_ArithXor1, NodeType_ArithAnd1, NodeType_ArithOr1,
    NodeType_LogicalAnd1, NodeType_LogicalOr1,

    // constants
    NodeType_Constant,

    // others
    NodeType_Negate, NodeType_Convert, NodeType_Variable, NodeType_NopV
};


class PDAL_DLL AstNode
{
public:
    AstNode(DataType, NodeType);
    virtual ~AstNode();

    DataType getDataType() const;
    void setDataType(DataType);
    std::string getDataTypeName() const;

    NodeType getNodeType() const;
    std::string getNodeTypeName() const;

    virtual void print(int indent) const = 0;
    virtual AstNode* simplify(SymbolTable& symbolTable) = 0;
    virtual bool evaluate(SymbolTable& symbolTable, variant_t&) = 0;

    friend bool operator==(const AstNode& lhs, const AstNode& rhs) { return lhs.equal_to(rhs); }
    friend bool operator!=(const AstNode& lhs, const AstNode& rhs) { return !(lhs==rhs); }

    unsigned int getId() const { return m_id; }

protected:
    static void spaces(int indent);

    // see http://stackoverflow.com/questions/1765122/equality-test-for-derived-classes-in-c
    virtual bool equal_to(const AstNode& other) const;

private:
    unsigned int m_id;
    DataType m_datatype;
    const NodeType m_nodetype;

    AstNode& operator=(AstNode const& rhs); // nope
};


//---------------------------------------------------------------------------


class PDAL_DLL AstTempVector : public AstNode
{
public:
    AstTempVector(std::vector<AstNode*> v);
    AstTempVector(AstNode* v1, AstNode* v2);
    virtual ~AstTempVector();

    std::vector<AstNode*>& getChildren() { return m_v; }
    const std::vector<AstNode*>& getChildren() const { return m_v; }

    virtual void print(int indent) const;
    virtual AstNode* simplify(SymbolTable& symbolTable);
    virtual bool evaluate(SymbolTable& symbolTable, variant_t&);

protected:
    virtual bool equal_to(const AstNode& other) const;

private:
    bool isChain() const;
    AstNode* collapseChain(SymbolTable& symbolTable);
    AstNode* fixOp1(AstNode* expr, AstNode* op1);

    std::vector<AstNode*> m_v;
};


//---------------------------------------------------------------------------


class PDAL_DLL AstNegate : public AstNode
{
public:
    AstNegate(AstNode* node);
    virtual ~AstNegate();

    virtual void print(int indent) const;
    virtual AstNode* simplify(SymbolTable& symbolTable);
    virtual bool evaluate(SymbolTable& symbolTable, variant_t&);
    
    const AstNode* getChild() const { return m_node; }
    AstNode* getChild() { return m_node; }

protected:
    virtual bool equal_to(const AstNode& other) const;

private:
    AstNode* m_node;
};


//---------------------------------------------------------------------------

//
// Type conversion
//

class PDAL_DLL AstConvert : public AstNode
{
public:
    AstConvert(AstNode* node, DataType datatype);
    virtual ~AstConvert();

    virtual void print(int indent) const;
    virtual AstNode* simplify(SymbolTable& symbolTable);
    virtual bool evaluate(SymbolTable& symbolTable, variant_t&);

    const AstNode* getChild() const { return m_node; }
    AstNode* getChild() { return m_node; }

protected:
    virtual bool equal_to(const AstNode& other) const;

private:
    AstNode* m_node;
};


//---------------------------------------------------------------------------


class PDAL_DLL AstTempOp1 : public AstNode
{
public:
    AstTempOp1(AstNode* node, NodeType);
    virtual ~AstTempOp1();

    virtual void print(int indent) const;
    virtual AstNode* simplify(SymbolTable& symbolTable);
    virtual bool evaluate(SymbolTable& symbolTable, variant_t&);

    const AstNode* getChild() const { return m_node; }
    AstNode* removeChild() { AstNode* r = m_node; m_node = NULL; return r; }

protected:
    virtual bool equal_to(const AstNode& other) const;

private:
    AstNode* m_node;
};


//---------------------------------------------------------------------------


class PDAL_DLL AstBinaryOp : public AstNode
{
public:
    AstBinaryOp(AstNode* left, AstNode* right, NodeType optype);
    virtual ~AstBinaryOp();

    virtual void print(int indent) const;
    virtual AstNode* simplify(SymbolTable& symbolTable);
    virtual bool evaluate(SymbolTable& symbolTable, variant_t&);

    AstNode* getLeft() { return m_left; }
    AstNode* getRight() { return m_left; }
    const AstNode* getLeft() const { return m_left; }
    const AstNode* getRight() const { return m_left; }

protected:
    virtual bool equal_to(const AstNode& other) const;

private:
    AstNode* m_left;
    AstNode* m_right;
};


//---------------------------------------------------------------------------

//
// constants
//

class PDAL_DLL AstConstant : public AstNode
{
public:
    AstConstant(variant_t v);
    virtual ~AstConstant();

    variant_t getVariantValue() const
    {
        return m_value;
    }

    template<typename T>
    T getValue() const
    {
        return boost::get<T>(m_value);
    }

    void setValue(variant_t value)
    {
        m_value = value;
    }

    virtual void print(int indent) const;
    virtual AstNode* simplify(SymbolTable& symbolTable);
    virtual bool evaluate(SymbolTable& symbolTable, variant_t&);

protected:
    virtual bool equal_to(const AstNode& other) const;

private:
    variant_t m_value;
};


//---------------------------------------------------------------------------


class PDAL_DLL AstVariableUse : public AstNode
{
public:
    AstVariableUse(SymbolName*);
    virtual ~AstVariableUse();

    virtual void print(int indent) const;
    virtual AstNode* simplify(SymbolTable& symbolTable);
    virtual bool evaluate(SymbolTable& symbolTable, variant_t&);
    Symbol* getSymbol() const;

protected:
    virtual bool equal_to(const AstNode& other) const;

private:
    SymbolName* m_symbolName;
    Symbol* m_symbol;
};


class PDAL_DLL AstVariableDef : public AstNode
{
public:
    AstVariableDef(SymbolName* symbolName, AstNode* rhs);
    virtual ~AstVariableDef();

    virtual void print(int indent) const;
    virtual AstNode* simplify(SymbolTable& symbolTable);
    virtual bool evaluate(SymbolTable& symbolTable, variant_t&);

    Symbol* getSymbol() const;
    AstNode* getRight() const;

protected:
    virtual bool equal_to(const AstNode& other) const;

private:
    SymbolName* m_symbolName;
    Symbol* m_symbol;
    AstNode* m_rhs;
};


} } // namespaces


#endif
