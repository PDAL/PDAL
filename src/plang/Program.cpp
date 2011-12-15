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

#include <pdal/plang/Program.hpp>

#include <pdal/plang/AstUtils.hpp>

#include <string>
#include <iostream>


namespace pdal { namespace plang {


Program::Program(SymbolTable* symbolTable, AstNode* node)
    : m_symbolTable(symbolTable)
{
    assert(node->getNodeType() == NodeType_NopV);
    AstTempVector* tmpVector = dynamic_cast<AstTempVector*>(node);
    assert(node);

    for (unsigned int i=0; i<tmpVector->getChildren().size(); i++)
    {
        AstNode* child = tmpVector->getChildren()[i];
        assert(child);

        child = child->simplify(*symbolTable);

        m_assignments.push_back(child);
    }

    delete node;

    return;
}


Program::~Program()
{
    delete m_symbolTable;
    for (unsigned int i=0; i<m_assignments.size(); i++)
    {
        delete m_assignments[i];
    }
    return;
}


bool Program::evaluate()
{
    variant_t notused;

    for (unsigned int i=0; i<m_assignments.size(); i++)
    {
        AstNode* node = m_assignments[i];
        bool ok = node->evaluate(*m_symbolTable, notused);
        if (!ok) return false;
    }

    return true;
}


void Program::setVariable(const std::string& name, variant_t value)
{
    Symbol* s = m_symbolTable->get(name);
    assert(s);

    s->setValue(value);
}


variant_t Program::getVariable(const std::string& name) const
{
    Symbol* s = m_symbolTable->get(name);
    assert(s);

    return s->getVariantValue();
}


} } //namespaces
